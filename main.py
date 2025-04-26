# This is an example of microMX1508 utilized together with
# QMC5883L compass (see https://github.com/karolew/microGY271)
# Tested on ESP32 WROOM v2.

import time
from machine import Pin, I2C

from microMX1508 import microMX1508
from gy271compass import QMC5883L


if __name__ == "__main__":
    # Compass.
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
    compass = QMC5883L(i2c, corrections={"x_offset": 162, "x_scale": 1.04, "y_offset": -211, "y_scale": 0.97})

    # Motors.
    motor1_pins = (27, 14)
    motor2_pins = (12, 13)
    motors = microMX1508(motor1_pins, motor2_pins, accel_rate=5)
    time.sleep(1)

    def turn_left():
        motors.set_motor1(40)
        motors.set_motor2(-40)

    def turn_right():
        motors.set_motor1(-40)
        motors.set_motor2(40)

    def stop():
        motors.stop()

    previous_heading = 0
    heading = 0
    while True:
        try:
            heading = compass.get_heading()
        except Exception as e:
            print("Issue with I2C.")

        # Reaguj na zmiany większe niż 10 stopni.
        if abs(heading - previous_heading) >= 10:
            previous_heading = heading

            if heading < 100:
                print("LEFT")
                turn_left()

            elif heading > 200:
                print("RIGHT")
                turn_right()

            else:
                print("STOP")
                stop()

        time.sleep(0.25)
        print(heading)
        motors.update()
