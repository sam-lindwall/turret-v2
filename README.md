# Pan-tilt auto-tracking camera

Runs object detection on a Raspberry pi 5 and a PID loop built on an STM32 Nucelo-f446re to track a hand's motion


## Architecture

Two processors split by timing requirement: perception on the Pi where a
150–200 ms inference budget is acceptable, control on the STM32 where a
160 Hz loop has to be deterministic.

```mermaid
flowchart TB
    subgraph PI["Raspberry Pi 5 — perception, ~5 Hz"]
        CAM[IMX296 global shutter] --> YOLO[YOLO / NCNN]
        YOLO --> OFF[pixel offset x,y]
        CAM -- SensorTimestamp --> INTERP[interpolate encoder<br/>to capture instant]
        OFF --> CMD["cmd = enc·cap_t· + offset"]
        INTERP --> CMD
    end

    subgraph STM["STM32 Nucleo-F446RE — control, 160 Hz"]
        RX["USART3 RX byte ISR"] --> PARSE[parse] --> TGT[target angle]
        TIM3["TIM3 @ 160 Hz"] --> LOOP
        TGT --> LOOP["PID + friction feedforward<br/>deadband w/ hysteresis, slew limit"]
        ENC["MT6701 14-bit absolute<br/>I2C1 pan · I2C3 tilt"] --> LOOP
        LOOP --> PWM[TIM2 PWM] --> DRV[TB6612FNG] --> MOT["2× 12 V gearmotor<br/>pan · tilt"]
        MOT -.-> ENC
    end

    CMD -- "p{int} t{int}\n @ 115200" --> RX
    STM -- "p%.2ft%.2f\n telemetry" --> INTERP
```

- **STM32 Nucleo-F446RE** — real-time motor control: UART command parsing,
  PWM generation, direction logic.
- **Raspberry Pi 5** — command source; vision pipeline that converts
 detected-object pixel offsets into angular commands
- **TB6612FNG** — dual H-bridge driver.
- **2× 12V 120RPM 37mm gear motors** — pan and tilt axes.


## Roadmap

- [x] Open-loop UART motor control
- [x] Pi-to-STM32 UART link validation (end-to-end echo confirmed)
- [x] Closed-loop PID with MT6701 encoder feedback
- [x] Full vision pipeline → UART → PID → motion (YOLO vision model to track hand)
- [ ] Gesture-based mode switching (to do something cool like shoot a nerf bullet on a closed fist)
