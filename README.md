# Pan-tilt auto-tracking camera

Runs object detection on a Raspberry Pi 5 and a PID loop on an STM32 Nucleo-F446RE to track a hand's motion

## Concepts applied
- **PID Control**
- **UART, I2C**
- **Interrupts, Timers**
- **Pulse Width Modulation (PWM)**
- **State Machines**


<img width="699" height="643" alt="PNG image" src="https://github.com/user-attachments/assets/ec6f8476-9d0b-444c-a285-4556ea28e43d" />

## Architecture

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

- **STM32 Nucleo-F446RE** — real-time motor control: UART command parsing, I2C communication with encoders,
  PWM generation, direction logic.
- **Raspberry Pi 5** — vision pipeline that converts
 detected-object pixel offsets into absolute angle commands.
- **TB6612FNG** — dual H-bridge driver.
- **2× 12V 120RPM 37mm gear motors** — pan and tilt axes.
- **2x MT6701 Magnetic Encoder** - absolute angle reads.


## Roadmap

- [x] Open-loop UART motor control
- [x] Pi-to-STM32 UART link validation (end-to-end echo confirmed)
- [x] Closed-loop PID with MT6701 encoder feedback
- [x] Full vision pipeline → UART → PID → motion (YOLO vision model to track hand)
- [ ] Gesture-based mode switching (to do something cool like shoot a nerf bullet on a closed fist)
