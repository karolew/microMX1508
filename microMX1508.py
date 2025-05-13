import time

from machine import PWM, Pin


class States:
    """
    States to represent motor movements directions.
    """
    reverse = -1
    stop = 0
    forward = 1


class microMX1508:
    def __init__(self,
                 motor1_pins: tuple,
                 motor2_pins: tuple,
                 freq: int = 1000,
                 max_duty: int = 1023,
                 accel_step: int = 50,
                 accel_delay_ms: int = 20) -> None:
        """
        Initialize the motor controller.
        motor1_pins: Tuple (in1, in2) for motor 1 control pins
        motor2_pins: Tuple (in1, in2) for motor 2 control pins
        freq: PWM frequency in Hz
        max_duty: Maximum duty cycle value
        accel_step: Step size for acceleration (duty cycle increment)
        accel_delay_ms: Delay in ms between acceleration steps
        """
        # Initialize PWM pins for motor 1
        self.motor1_pin1 = PWM(Pin(motor1_pins[0]), freq=freq)
        self.motor1_pin2 = PWM(Pin(motor1_pins[1]), freq=freq)

        # Initialize PWM pins for motor 2
        self.motor2_pin1 = PWM(Pin(motor2_pins[0]), freq=freq)
        self.motor2_pin2 = PWM(Pin(motor2_pins[1]), freq=freq)

        # Configuration parameters
        self.max_duty = max_duty
        self.accel_step = accel_step
        self.accel_delay_ms = accel_delay_ms

        # Current state and target values
        self.motor1_current_duty = 0
        self.motor2_current_duty = 0
        self.motor1_target_duty = 0
        self.motor2_target_duty = 0
        self.motor1_target_direction = States.stop
        self.motor2_target_direction = States.stop
        self.motor1_current_direction = States.stop
        self.motor2_current_direction = States.stop

        # Timestamp for acceleration control
        self.last_update_time = time.ticks_ms()
        self.transitioning = False

        # Initialize motors to stopped state
        self._stop_immediately()

    def _set_motor_speed(self, motor_pin1: tuple, motor_pin2: tuple, speed: int, direction: int) -> None:
        if direction == States.stop or speed == 0:  # Stop
            motor_pin1.duty(0)
            motor_pin2.duty(0)
        elif direction == States.forward:
            motor_pin1.duty(speed)
            motor_pin2.duty(0)
        else:  # Reverse
            motor_pin1.duty(0)
            motor_pin2.duty(speed)

    def update(self) -> None:
        """
        Update motor speeds for smooth acceleration.
        This method should be called regularly in the main loop.
        """
        current_time = time.ticks_ms()
        # Check if it's time to update speeds (non-blocking)
        if time.ticks_diff(current_time, self.last_update_time) >= self.accel_delay_ms:
            self.last_update_time = current_time

            # Handle direction transitions
            # For motor 1
            if self.motor1_current_direction != self.motor1_target_direction:
                # If current speed is not zero, decelerate first
                if self.motor1_current_duty > 0:
                    self.motor1_current_duty = max(self.motor1_current_duty - self.accel_step, 0)
                    self._set_motor_speed(self.motor1_pin1,
                                          self.motor1_pin2,
                                          self.motor1_current_duty,
                                          self.motor1_current_direction)
                    self.transitioning = True
                else:
                    # Once speed reaches zero, change direction
                    self.motor1_current_direction = self.motor1_target_direction
                    self.transitioning = False
            else:
                # Update motor 1 speed when directions match
                if self.motor1_current_duty < self.motor1_target_duty:
                    self.motor1_current_duty = min(self.motor1_current_duty + self.accel_step,
                                                   self.motor1_target_duty)
                elif self.motor1_current_duty > self.motor1_target_duty:
                    self.motor1_current_duty = max(self.motor1_current_duty - self.accel_step,
                                                   self.motor1_target_duty)

                if self.motor1_current_duty > 0:
                    self._set_motor_speed(self.motor1_pin1,
                                          self.motor1_pin2,
                                          self.motor1_current_duty,
                                          self.motor1_current_direction)

            # For motor 2 - same logic as motor 1
            if self.motor2_current_direction != self.motor2_target_direction:
                # If current speed is not zero, decelerate first
                if self.motor2_current_duty > 0:
                    self.motor2_current_duty = max(self.motor2_current_duty - self.accel_step, 0)
                    self._set_motor_speed(self.motor2_pin1,
                                          self.motor2_pin2,
                                          self.motor2_current_duty,
                                          self.motor2_current_direction)
                    self.transitioning = True
                else:
                    # Once speed reaches zero, change direction
                    self.motor2_current_direction = self.motor2_target_direction
                    self.transitioning = False
            else:
                # Update motor 2 speed when directions match
                if self.motor2_current_duty < self.motor2_target_duty:
                    self.motor2_current_duty = min(self.motor2_current_duty + self.accel_step,
                                                   self.motor2_target_duty)
                elif self.motor2_current_duty > self.motor2_target_duty:
                    self.motor2_current_duty = max(self.motor2_current_duty - self.accel_step,
                                                   self.motor2_target_duty)

                if self.motor2_current_duty > 0:
                    self._set_motor_speed(self.motor2_pin1,
                                          self.motor2_pin2,
                                          self.motor2_current_duty,
                                          self.motor2_current_direction)

    def _stop_immediately(self) -> None:
        """
        Internal method to immediately stop motors (used for initialization)
        """
        self.motor1_current_duty = 0
        self.motor2_current_duty = 0
        self.motor1_target_duty = 0
        self.motor2_target_duty = 0
        self.motor1_current_direction = States.stop
        self.motor2_current_direction = States.stop
        self.motor1_target_direction = States.stop
        self.motor2_target_direction = States.stop
        self._set_motor_speed(self.motor1_pin1, self.motor1_pin2, 0, States.stop)
        self._set_motor_speed(self.motor2_pin1, self.motor2_pin2, 0, States.stop)
        self.transitioning = False

    def is_transitioning(self) -> bool:
        """
        Check if motors are transitioning between states.
        Returns True if motors are still accelerating/decelerating.
        """
        if self.transitioning:
            return True

        m1_transitioning = (self.motor1_current_duty != self.motor1_target_duty or
                            self.motor1_current_direction != self.motor1_target_direction)
        m2_transitioning = (self.motor2_current_duty != self.motor2_target_duty or
                            self.motor2_current_direction != self.motor2_target_direction)
        return m1_transitioning or m2_transitioning

    def forward(self) -> None:
        """
        Set both motors to move forward with full speed.
        """
        self.motor1_target_direction = States.forward
        self.motor2_target_direction = States.forward
        self.motor1_target_duty = self.max_duty
        self.motor2_target_duty = self.max_duty

    def stop(self) -> None:
        """
        Smoothly stop both motors.
        """
        self.motor1_target_direction = States.stop
        self.motor2_target_direction = States.stop
        self.motor1_target_duty = 0
        self.motor2_target_duty = 0

    def turn_right(self, turning_force_level: int = 1) -> None:
        """
        Turn left with specified force level.
        turning_force_level: Integer 0-3 controlling turn sharpness
        """
        self.motor1_target_direction = States.forward
        self.motor1_target_duty = self.max_duty

        # Apply turning force level to adjust right motor
        if turning_force_level == 0:  # Slight turn - right motor at 80%
            self.motor2_target_direction = States.forward
            self.motor2_target_duty = int(self.max_duty * 0.8)
        elif turning_force_level == 1:  # Medium turn - right motor at 50%
            self.motor2_target_direction = States.forward
            self.motor2_target_duty = int(self.max_duty * 0.5)
        elif turning_force_level == 2:  # Sharp turn - right motor stopped
            self.motor2_target_direction = States.stop
            self.motor2_target_duty = 0
        elif turning_force_level == 3:  # Max turn - right motor reversed
            self.motor2_target_direction = States.reverse
            self.motor2_target_duty = self.max_duty

    def turn_left(self, turning_force_level: int = 1) -> None:
        """
        Turn right with specified force level.
        turning_force_level: Integer 0-3 controlling turn sharpness
        """
        self.motor2_target_direction = States.forward
        self.motor2_target_duty = self.max_duty

        # Apply turning force level to adjust left motor
        if turning_force_level == 0:  # Slight turn - left motor at 80%
            self.motor1_target_direction = States.forward
            self.motor1_target_duty = int(self.max_duty * 0.8)
        elif turning_force_level == 1:  # Medium turn - left motor at 50%
            self.motor1_target_direction = States.forward
            self.motor1_target_duty = int(self.max_duty * 0.5)
        elif turning_force_level == 2:  # Sharp turn - left motor stopped
            self.motor1_target_direction = States.stop
            self.motor1_target_duty = 0
        elif turning_force_level == 3:  # Max turn - left motor reversed
            self.motor1_target_direction = States.reverse
            self.motor1_target_duty = self.max_duty
