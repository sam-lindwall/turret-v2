"""
simple_track.py -- closed-loop hand tracking, Pi 5 -> STM32.

"""
import csv
import time
import threading
import collections

import numpy as np
import cv2
import serial
from picamera2 import Picamera2
from ultralytics import YOLO

# ---- CONFIG --------------------------------------------------------------
TELEM_LAG = 0.030    # s

IMGSZ = 256        
STATS_EVERY = 20

LOGFILE = f"track_{time.strftime('%Y%m%d_%H%M%S')}.csv"
# --------------------------------------------------------------------------


# ---- clock domain --------------------------------------------------------
def measure_boot_mono_offset(n=9):
    """mono = boot + offset. Sandwich each read to reject scheduling jitter."""
    best_span, best_off = None, 0.0
    for _ in range(n):
        t0 = time.monotonic()
        b = time.clock_gettime(time.CLOCK_BOOTTIME)
        t1 = time.monotonic()
        span = t1 - t0
        if best_span is None or span < best_span:
            best_span, best_off = span, (t0 + t1) / 2.0 - b
    return best_off, best_span


BOOT_MONO_OFFSET, _span = measure_boot_mono_offset()

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.set_controls({
    "AeEnable": False,
    "ExposureTime": 10000,
    "AnalogueGain": 4.0,
})
picam2.start()
Model = YOLO("best.pt", task="detect")

fpx = 6/.00345 * 640/1456          # ~764 px  ->  1 deg ~ 13.3 px
CX, CY = 320, 240                  # frame is 640x480
ser = serial.Serial(port='/dev/ttyAMA0', baudrate=115200, timeout=0.1)


# ---------------------------------------------------------------------------
# SERIAL READER THREAD
# ---------------------------------------------------------------------------
stop_flag = threading.Event()
angle_lock = threading.Lock()
latest_angle = {"pan": None, "tilt": None, "t": 0.0}
angle_hist = collections.deque(maxlen=4000)
bad_lines = 0
n_rx = 0


def serial_reader():
    global bad_lines, n_rx
    while not stop_flag.is_set():
        try:
            line = ser.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        if not line:
            continue
        if line[0] != 'p':
            bad_lines += 1
            continue
        t_idx = line.find('t')
        if t_idx == -1:
            bad_lines += 1
            continue
        try:
            pan = float(line[1:t_idx])
            tilt = float(line[t_idx+1:])
        except ValueError:
            bad_lines += 1
            continue
        now = time.monotonic()
        with angle_lock:
            latest_angle["pan"] = pan
            latest_angle["tilt"] = tilt
            latest_angle["t"] = now
            angle_hist.append((now, pan, tilt))
            n_rx += 1


def read_angle():
    with angle_lock:
        return latest_angle["pan"], latest_angle["tilt"]


def angle_age():
    with angle_lock:
        if latest_angle["pan"] is None:
            return 1e9
        return time.monotonic() - latest_angle["t"]


# ---------------------------------------------------------------------------
# Encoder position at a given instant
# ---------------------------------------------------------------------------
def angle_at(t_mono):
    """Interpolated (pan, tilt, pan_rate) at monotonic time t_mono.

    A line received at rx describes the axis at rx - TELEM_LAG; querying
    for t is the same as querying the raw rx timeline at t + TELEM_LAG.

    """
    t = t_mono + TELEM_LAG
    with angle_lock:
        h = list(angle_hist)
    if len(h) < 2 or t < h[0][0] or t > h[-1][0]:
        return None, None, 0.0
    # walk back from the newest; the match is usually near end of buffer
    for i in range(len(h) - 1, 0, -1):
        t0, p0, l0 = h[i - 1]
        t1, p1, l1 = h[i]
        if t0 <= t <= t1:
            dt = t1 - t0
            f = (t - t0) / dt if dt > 0 else 0.0
            rate = (p1 - p0) / dt if dt > 0 else 0.0
            return p0 + f * (p1 - p0), l0 + f * (l1 - l0), rate
    return None, None, 0.0


serial_t = threading.Thread(target=serial_reader, daemon=True)
serial_t.start()

_t0 = time.monotonic()
while angle_age() > 1.0 and time.monotonic() - _t0 < 3.0:
    time.sleep(0.05)
if angle_age() > 1.0:
    print("WARNING: no angle from STM after 3s -- is it powered and streaming?")
else:
    print("STM stream alive")

# measure the telemetry rate; interpolation quality depends on it
_t0, _n0 = time.monotonic(), n_rx
time.sleep(1.0)
telem_hz = (n_rx - _n0) / (time.monotonic() - _t0)

print(f"clock offset (boot->mono) : {BOOT_MONO_OFFSET*1000:+.1f} ms "
      f"(read span {_span*1e6:.0f} us)")
if abs(BOOT_MONO_OFFSET) > 0.005:
    print("   NOTE: nonzero -- the system has suspended. Conversion applied;")
    print("   without it every capture timestamp would land in the wrong place.")
print(f"telemetry rate            : {telem_hz:.1f} Hz "
      f"({1000/max(telem_hz,1e-6):.1f} ms between samples)")
print(f"TELEM_LAG={TELEM_LAG*1000:.0f} ms  imgsz={IMGSZ or 640}")
print(f"logging -> {LOGFILE}")
print("Ctrl-C to stop.\n")


def grab_after(barrier, max_tries=60):
    """Newest frame captured at or after `barrier`, with cap_t in MONOTONIC."""
    stale = 0
    frame = cap_t = None
    for _ in range(max_tries):
        request = picam2.capture_request()
        try:
            frame = request.make_array("main")
            cap_boot = request.get_metadata()['SensorTimestamp'] / 1e9
        finally:
            request.release()
        cap_t = cap_boot + BOOT_MONO_OFFSET
        if cap_t >= barrier:
            return frame, cap_t, stale
        stale += 1
    return frame, cap_t, stale


n_cmd = 0
n_stale = 0
n_skip_interp = 0
cmd_times = []
rows = []
barrier = time.monotonic()

logf = open(LOGFILE, "w", newline="")
log = csv.writer(logf)
log.writerow(["t_send", "cap_t", "infer_ms", "off_x", "off_y",
              "enc_interp", "enc_latest", "enc_delta", "pan_rate",
              "cmd_p", "cmd_t", "sent"])

try:
    while True:
        frame, cap_t, stale = grab_after(barrier)
        n_stale += stale

        frame = frame[:, :, :3]
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        t_inf = time.monotonic()
        results = (Model(frame, imgsz=IMGSZ, verbose=False) if IMGSZ
                   else Model(frame, verbose=False))
        infer_ms = (time.monotonic() - t_inf) * 1000
        boxes = results[0].boxes

        if len(boxes) == 0:
            print("No hand detected")
            barrier = cap_t + 1e-6
            continue

        # pick the highest-confidence box, not whichever landed at index 0
        try:
            k = int(np.argmax(boxes.conf.cpu().numpy()))
        except Exception:
            k = 0
        x1, y1, x2, y2 = boxes.xyxy[k].tolist()
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2
        xoff_angle = np.degrees(np.arctan((center_x - CX) / fpx))
        yoff_angle = np.degrees(np.arctan((center_y - CY) / fpx))

        # ---- encoder sampled at the frame's capture instant ----
        xangle, yangle, pan_rate = angle_at(cap_t)
        if xangle is None:
            n_skip_interp += 1
            print("capture outside telemetry history -- skipping")
            barrier = cap_t + 1e-6
            continue

        enc_latest_p, _ = read_angle()

        xabs_angle = xangle + xoff_angle
        yabs_angle = yangle + yoff_angle

        sent = 0
        in_range = (abs(xabs_angle) < 70) and (abs(yabs_angle) < 90)

        if not in_range:
            print("angle too large")
        else:
            cmd = f"p{int(round(xabs_angle))} t{int(round(-1*yabs_angle))}\n" # ***************
            ser.write(cmd.encode())
            sent = 1
            n_cmd += 1
            cmd_times.append(time.monotonic())

        t_send = time.monotonic()
        log.writerow([f"{t_send:.4f}", f"{cap_t:.4f}", f"{infer_ms:.1f}",
                      f"{xoff_angle:+.3f}", f"{yoff_angle:+.3f}",
                      f"{xangle:+.3f}", f"{enc_latest_p:+.3f}",
                      f"{enc_latest_p - xangle:+.3f}", f"{pan_rate:+.2f}",
                      f"{xabs_angle:+.2f}", f"{yabs_angle:+.2f}", sent])
        rows.append((xoff_angle, pan_rate, enc_latest_p - xangle))

        barrier = cap_t + 1e-6

        if sent and n_cmd % STATS_EVERY == 0 and len(cmd_times) > 2:
            gaps = np.diff(cmd_times[-STATS_EVERY:])
            print(f"[stats] cmds={n_cmd} | rate={1/np.median(gaps):.2f} Hz | "
                  f"infer={infer_ms:.0f}ms | stale={n_stale} | "
                  f"interp_skip={n_skip_interp} | bad_lines={bad_lines}")

except KeyboardInterrupt:
    print("\nstopping")

finally:
    print("--- TEARDOWN ---")
    stop_flag.set()
    serial_t.join(timeout=1.0)
    try:
        picam2.stop()
        picam2.close()
    except Exception as e:
        print(f"camera release failed: {e}")
    try:
        ser.close()
    except Exception as e:
        print(f"ser.close() failed: {e}")
    try:
        logf.close()
    except Exception:
        pass

    print(f"\ncommands sent    : {n_cmd}")
    if len(cmd_times) > 2:
        print(f"command rate     : {1/np.median(np.diff(cmd_times)):.2f} Hz")
    print(f"stale discarded  : {n_stale}")
    print(f"interp skipped   : {n_skip_interp}")

    # ---- TELEM_LAG calibration ------------------------------------------
    if len(rows) > 30:
        off = np.array([r[0] for r in rows])
        rate = np.array([r[1] for r in rows])
        delta = np.array([r[2] for r in rows])
        m = np.abs(rate) > 1.0            # only frames where the axis moved
        print(f"\ninterp vs latest : mean |delta| = {np.mean(np.abs(delta)):.2f} "
              f"deg, max {np.max(np.abs(delta)):.2f} deg")
        print("   (that is the error a plain read_angle() would inject)")
        if m.sum() > 20:
            slope, icpt = np.polyfit(rate[m], off[m], 1)
            r = np.corrcoef(rate[m], off[m])[0, 1]
            print(f"off vs pan_rate  : slope = {slope*1000:+.1f} ms   "
                  f"r = {r:+.2f}   (n={m.sum()})")
            if abs(slope) > 0.010:
                print(f"   residual timing error ~{abs(slope)*1000:.0f} ms. "
                      f"Adjust TELEM_LAG by that much;")
                print( "   rerun -- if |slope| grew, go the other way.")
            else:
                print("   timing aligned. Residual is not explained by axis "
                      "motion.")
        else:
            print("off vs pan_rate  : not enough moving frames to fit. "
                  "Wave the target faster.")
    print(f"bad_lines={bad_lines}")