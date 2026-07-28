# Pan-Tilt Camera Turret — V1

Building a vision-guided pan-tilt camera turret. 

**Current status: end-to-end UART command link validated.**
A Raspberry Pi 5 sends motion commands over UART to an STM32, which parses them,
drives a TB6612FNG motor driver, and echoes a debug response back to the Pi.

## Architecture

```
Raspberry Pi 5  ──UART (GPIO 14/15)──▶  STM32 Nucleo-F446RE  ──▶  TB6612FNG  ──▶  DC gear motors
     ▲                                          │
     └──────────── USB debug echo ──────────────┘
```

- **STM32 Nucleo-F446RE** — real-time motor control: UART command parsing,
  PWM generation, direction logic.
- **Raspberry Pi 5** — command source; (upcoming: vision pipeline that converts
  detected-object pixel offsets into angular commands)
- **TB6612FNG** — dual H-bridge driver. VM = 12V motor power, VCC = 3.3V logic.
- **2× 12V 120RPM 37mm gear motors** — pan and tilt axes.

## Command protocol (V1)

ASCII command terminated by newline:

```
<dir><power>
```

- `dir`: `f` (forward), `r` (reverse), `s` (stop)
- `power`: integer 0–999, mapped directly to PWM compare value (ARR = 999)

Examples: `f800`, `r500`, `s`

The STM32 echoes back a parsed confirmation, e.g.:

```
RX: f800 | dir=f power=800 ccr=800
```


## Roadmap

- [x] Open-loop UART motor control
- [x] Pi-to-STM32 UART link validation (end-to-end echo confirmed)
- [ ] Closed-loop PID with MT6701 encoder feedback
- [ ] FreeRTOS re-vamp
- [ ] Full vision pipeline → UART → PID → motion (YOLO vision model to track hand)
- [ ] Gesture-based mode switching (to do something cool like shoot a nerf bullet on a closed fist!)
