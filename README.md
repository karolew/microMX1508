# Micro MX1508 Driver

This is micropython MX1508 driver.
Designed in less blocking approach.

# Usage

1. Copy `microMX1508.py` file to your project.
2. Create instance `motors = microMX1508(motor1_pins, motor2_pins, accel_rate=20)` where motor1_pins and motor2_pins are your board GPIOs to control the driver.
3. Set forwad speed at 50% `motors.set_motor1(50)`
4. Set backward speed at 80% `motors.set_motor1(-80)`
5. Apply change with `motors.update()`. This method must be called continuously in the main loop.
