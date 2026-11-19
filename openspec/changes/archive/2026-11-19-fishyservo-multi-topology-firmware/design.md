## Context

FishyServo features 4 H-bridges and STM32H503 timers:
- TIM1: PA8 (CH1), PA9 (CH2), PB15 (CH3N)
- TIM2: PA15 (CH1)
- TIM3: PA6 (CH1), PA7 (CH2), PB0 (CH3), PB5 (CH2)

## Goals / Non-Goals

**Goals:**
- Drive 4 DC motors with forward, reverse, and brake PWM.
- Read 4 ADC current channels synchronously.
- Expose `multi_motor_profile` over I2C2 and Commander over UART1/RTT.
