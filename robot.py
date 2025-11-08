from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, UltrasonicSensor)
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait

import math


class Robot:
    def __init__(self):
        self.brick = EV3Brick()

        # Physical dimensions (TRACK width = center-to-center between wheels)
        self.length       = 0.18   # m (not used in kinematics)
        self.width        = 0.111  # m  <-- your measured TRACK WIDTH
        self.height       = 0.128  # m (not used)
        self.wheel_radius = 0.028  # m

        # Pose (radians, +CCW)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Motors
        self.left_motor  = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)

        # Encoders: reset and sync baselines
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.prev_left  = 0.0
        self.prev_right = 0.0

        # Sensors
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)  # facing LEFT side
        self.touch_sensor_left = TouchSensor(Port.S2)
        self.touch_sensor_right = TouchSensor(Port.S3)

    # -------------------- Helpers --------------------

    def get_ultrasonic_distance(self):
        # returns meters
        return self.ultrasonic_sensor.distance() / 1000.0

    def get_x(self): return self.x
    def get_y(self): return self.y
    def get_theta(self): return self.theta

    def distance_to_degrees(self, distance):
        return (distance / (2 * math.pi * self.wheel_radius)) * 360.0

    def degrees_to_distance(self, degrees):
        rotations = degrees / 360.0
        return rotations * (2 * math.pi * self.wheel_radius)

    def is_touching_wall(self):
        return self.touch_sensor_left.pressed() and self.touch_sensor_right.pressed()

    def beep(self):
        self.brick.speaker.beep()

    def wait_for_button_press(self):
        while Button.CENTER not in self.brick.buttons.pressed():
            wait(10)

    # -------------------- Motion primitives --------------------

    def creep_forward(self, speed=200):
        # Starts rolling forward; caller must loop update_position()
        self.left_motor.run(speed)
        self.right_motor.run(speed)

    def creep_forward_until(self, stop_fn, speed=200, period_ms=30):
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        while not stop_fn():
            self.update_position()
            wait(period_ms)
        self.left_motor.stop()
        self.right_motor.stop()

    def move_forward(self, distance, speed=200):
        """
        Moves robot forward by given distance (meters)
        at a given speed (deg/s on the motors). Adds 2 cm to account for bump stop.
        """
        distance += 0.02  # your original compensation

        rotation_degrees = (distance / (2 * math.pi * self.wheel_radius)) * 360.0

        self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
        self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)

        # Integrate this motion and resync baselines
        self.update_position()
        self.prev_left  = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()

    def move_backward(self, distance, speed=200):
        self.move_forward(-distance, speed)

    def turn(self, angle_deg, speed):
        """
        In-place turn. +angle = CCW (left), -angle = CW (right).
        """
        wheel_base_radius = self.width / 2.0
        arc_length = (math.pi * wheel_base_radius * abs(angle_deg)) / 180.0
        rotation_degrees = (arc_length / (2 * math.pi * self.wheel_radius)) * 360.0

        if angle_deg > 0:  # CCW (left)
            self.left_motor.run_angle(speed,  rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=True)
        else:              # CW (right)
            self.left_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed,  rotation_degrees, Stop.BRAKE, wait=True)

        # Update heading (no normalization)
        self.theta += (angle_deg * math.pi / 180.0)

        # Resync baselines after blocking turn
        self.prev_left  = self.left_motor.angle()
        self.prev_right = self.right_motor.angle()

    # -------------------- Odometry --------------------

    def update_position(self):
        r = self.wheel_radius
        L = self.width  # track width

        left_angle  = self.left_motor.angle()
        right_angle = self.right_motor.angle()

        dL = (left_angle  - self.prev_left)  * (math.pi / 180.0) * r
        dR = (right_angle - self.prev_right) * (math.pi / 180.0) * r

        self.prev_left  = left_angle
        self.prev_right = right_angle

        ds     = 0.5 * (dR + dL)
        dtheta = (dR - dL) / L

        # First-order exact integration (midpoint heading)
        self.x     += ds * math.cos(self.theta + 0.5 * dtheta)
        self.y     += ds * math.sin(self.theta + 0.5 * dtheta)
        self.theta += dtheta

    # -------------------- Wall following (LEFT-hand wall) --------------------

    def wall_follow(
        self,
        start_pos,
        return_pos,
        target_distance=0.15,   # desired distance to the LEFT wall
        base_speed=100,
        kp=250, ki=2.0, kd=50
    ):
        """
        Follow a convex wall on the LEFT side using the left-facing ultrasonic sensor
        until we return near 'return_pos'.
        """

        # Reset encoders AND resync baselines to avoid phantom jump on first odom tick
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.prev_left  = 0.0
        self.prev_right = 0.0

        last_error = 0.0
        integral = 0.0
        check = True

        I_MAX = 0.2
        CORR_CAP = 80.0
        MIN_SPD, MAX_SPD = 40, 120

        r = self.wheel_radius
        last_L = 0.0
        last_R = 0.0

        while True:
            # Stop when back near the start (your tolerances + check flag)
            if (
                abs(return_pos[0] - self.x) <= 0.15
                and abs(return_pos[1] - self.y) <= 0.10
                and not check
            ):
                break

            # Range reading with simple sanity handling
            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 1.0:
                distance = target_distance  # fallback

            # If too close to the LEFT wall, nudge RIGHT (CW)
            if distance < 0.08:
                self.turn(-8, 50)  # small right nudge away from left wall
                wait(100)
                distance = self.get_ultrasonic_distance()

            # Bumpers: back up and turn RIGHT to get off the left wall
            if self.touch_sensor_left.pressed() or self.touch_sensor_right.pressed():
                self.move_backward(0.15)
                self.turn(-90, 100)

            # -------- LEFT-wall PID control --------
            # error > 0  => too far from LEFT wall => steer LEFT (CCW)
            error = distance - target_distance
            derivative = error - last_error
            last_error = error

            integral += error
            if integral > I_MAX: integral = I_MAX
            if integral < -I_MAX: integral = -I_MAX

            correction = kp * error + ki * integral + kd * derivative
            if correction >  CORR_CAP: correction =  CORR_CAP
            if correction < -CORR_CAP: correction = -CORR_CAP

            # For LEFT-wall following, steer LEFT when error>0:
            # Make left wheel slower, right wheel faster
            left_speed  = base_speed - correction
            right_speed = base_speed + correction

            if left_speed  > MAX_SPD: left_speed  = MAX_SPD
            if left_speed  < MIN_SPD: left_speed  = MIN_SPD
            if right_speed > MAX_SPD: right_speed = MAX_SPD
            if right_speed < MIN_SPD: right_speed = MIN_SPD

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)
            self.update_position()

            # EV3 screen debug
            self.brick.screen.clear()
            self.brick.screen.print(
                str(abs(self.x - return_pos[0])) + "\n" +
                str(abs(self.y - return_pos[1])) + "\n" +
                str(self.theta)
            )

            # (Optional) wheel deltas if you need them
            cur_L = self.left_motor.angle()
            cur_R = self.right_motor.angle()
            _dL = (cur_L - last_L) * (2 * math.pi * r) / 360.0
            _dR = (cur_R - last_R) * (2 * math.pi * r) / 360.0
            last_L, last_R = cur_L, cur_R

            wait(30)

            # Clear 'check' only once we've moved away from the start box
            if abs(self.x - return_pos[0]) > 0.12 and abs(self.y - return_pos[1]) > 0.22:
                check = False

        # Stop and confirm
        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()
