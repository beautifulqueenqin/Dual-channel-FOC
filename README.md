# Dual-Channel FOC

Firmware project for an STM32F405-based dual-channel field-oriented control (FOC) motor driver.

## Development Log

### 2026-11-08 — PI Control and Core FOC Mathematics

- Added shared motor-control data types for phase current, alpha-beta current and voltage, d-q current and voltage, PWM duty, motor identifiers, and sine/cosine values.
- Added a reusable discrete PI controller with output limiting and conditional-integration anti-windup.
- Added the pure FOC mathematics module:
  - value clamping;
  - Clarke transformation;
  - electrical-angle sine and cosine calculation;
  - Park transformation;
  - inverse Park transformation;
  - min-max SVPWM with bus-voltage limiting.
- Kept the control algorithms independent of STM32 HAL and peripheral registers for portability and reuse.
- Updated the `nFAULT` GPIO configuration to use an internal pull-up on the falling-edge interrupt input.
- Verified the PI and FOC mathematics source files with strict C syntax checks.

## Current Scope

The PI and mathematical modules are implemented as portable building blocks. They are not yet connected to the real-time ADC, encoder, PWM, or motor-current-loop execution path.

## Planned Next Steps

1. Define the single-motor runtime state and current-loop interface.
2. Implement the PWM, current-sampling, encoder, driver, and safety BSP layers.
3. Connect one current loop to the timer/ADC interrupt path.
4. Extend the design to coordinated dual-motor control.
