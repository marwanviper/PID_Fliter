# PID Motor Speed Control with Soft Start

This project uses an Arduino to control the speed of a DC motor using a PID controller. It also includes a **soft start** feature using an **exponential smoothing filter** to gradually increase motor speed and prevent sudden changes.                                                                                                                        
  

## How It Works

1. **PID Controller**: Adjusts the motor speed based on the difference (error) between the desired speed (setpoint) and the actual speed.
2. **Exponential Smoothing Filter**: Provides a soft start by gradually ramping up the speed to the setpoint.
3. **Rotary Encoder**: Measures the current motor speed for feedback.

## Wiring Setup

- Connect the motor to the motor driver.
- Connect the motor driver to the Arduino.
- Attach the rotary encoder to the motor and connect it to the Arduino.
  

  
### Key Variables:
- `Kp`, `Ki`, `Kd`: PID controller parameters.
- `alpha`: Smoothing factor for the soft start.


## Tuning the PID

You may need to adjust the `Kp`, `Ki`, and `Kd` values to suit your motor setup. Start by increasing `Kp` until the motor responds well, then fine-tune `Ki` and `Kd` for stability.
