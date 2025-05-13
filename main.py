# This is an example of microMX1508 utilized together with
# QMC5883L compass (see https://github.com/karolew/microGY271)
# Tested on ESP32 WROOM v2.

import time

from gy271compass import QMC5883L
from machine import I2C, Pin

from microMX1508 import microMX1508

if __name__ == "__main__":
    # Compass.
    i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
    compass = QMC5883L(i2c,
                       None,
                       (-2364, -496, 68),
                       [[1.118951, 0.0, 0.0],
                        [0.0, 1.07733, 0.0],
                        [0.0, 0.0, 0.8488354]])

    # Motors.
    motors =  microMX1508((27, 14),
                          (12, 13),
                          accel_step=200,
                          max_duty=512)
    time.sleep(1)

    previous_heading = 0
    heading = 0
    direction = None
    while True:
        try:
            heading = compass.get_heading()
        except Exception as e:
            print("Issue with I2C.")
            continue

        if heading <= 15:
            direction = "LEWO"
            motors.turn_left(1)

        elif heading >= 20:
            direction = "PRAWO"
            motors.turn_right(1)

        elif 15 < heading < 20:
            direction = "PROSTO"
            motors.forward()

        print(direction,
              heading,
              motors.motor1_current_duty,
              motors.motor1_target_direction,
              motors.motor2_current_duty,
              motors.motor2_target_direction)

        time.sleep(0.1)
        motors.update()
