from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait
import math


class Robot:
    def __init__(self):
        # Hub
        self.brick = EV3Brick()

        # Geometry (meters)
        self.wheel_radius = 0.028
        self.width = 0.111  # distance between wheels

        # Pose in world frame:
        # x right, y up, theta CCW from +x (radians)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder baselines for incremental odometry
        self.prev_left_deg = 0.0
        self.prev_right_deg = 0.0

        # Heading fusion weight (1.0 = gyro only)
        self.GYRO_W = 1.0

        # Devices
        self.left_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        self.touch_sensor_left = TouchSensor(Port.S2)
        self.touch_sensor_right = TouchSensor(Port.S3)
        self.gyro = GyroSensor(Port.S4)

        # Gyro bias so that gyro angle + bias = global heading (deg)
        self.heading_bias_deg = 0.0

        # Initialize
        self.reset_odometry(0.0, 0.0, 0.0)

    # ------------------------------------------------------
    # Odometry / pose setup
    # ------------------------------------------------------
    def reset_motor_baselines(self):
        self.prev_left_deg = self.left_motor.angle()
        self.prev_right_deg = self.right_motor.angle()

    def initialize_gyroscope(self, reference_theta_rad=0.0):
        """
        Align gyro so that its reading corresponds to reference_theta_rad
        in our global frame (0 = +x, 90° = +y).
        """
        self.left_motor.stop()
        self.right_motor.stop()
        self.gyro.reset_angle(0)
        wait(1500)
        self.heading_bias_deg = math.degrees(reference_theta_rad)

    def reset_odometry(self, x=0.0, y=0.0, theta=0.0):
        """
        Reset pose and encoders.
        theta in radians, CCW from +x.
        """
        self.x, self.y, self.theta = x, y, theta
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.reset_motor_baselines()
        self.initialize_gyroscope(reference_theta_rad=theta)

    def set_pose_deg(self, x, y, theta_deg):
        self.reset_odometry(x, y, math.radians(theta_deg))

    # ------------------------------------------------------
    # Basic getters
    # ------------------------------------------------------
    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_theta(self):
        return self.theta  # radians

    def get_theta_deg(self):
        return math.degrees(self.theta)

    # ------------------------------------------------------
    # Gyro heading helpers
    # ------------------------------------------------------
    def get_heading_deg_from_gyro(self):
        """
        Global heading in degrees, from gyro + bias.
        0 = +x, 90 = +y, CCW positive.
        """
        return self.gyro.angle() + self.heading_bias_deg

    def get_heading_rad_from_gyro(self):
        return math.radians(self.get_heading_deg_from_gyro())

    # Normalize angle to (-180, 180]
    def _normalize_deg(self, angle):
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle

    # ------------------------------------------------------
    # Odometry core
    # ------------------------------------------------------
    def distance_to_degrees(self, distance):
        return (distance / (2 * math.pi * self.wheel_radius)) * 360.0

    def degrees_to_distance(self, degrees):
        return (degrees / 360.0) * (2 * math.pi * self.wheel_radius)

    def update_odometry(self):
        """
        Update (x, y, theta) using encoders for distance and gyro for heading.
        Call often while moving.
        """
        left_deg = self.left_motor.angle()
        right_deg = self.right_motor.angle()

        dL_deg = left_deg - self.prev_left_deg
        dR_deg = right_deg - self.prev_right_deg

        self.prev_left_deg = left_deg
        self.prev_right_deg = right_deg

        dL = self.degrees_to_distance(dL_deg)
        dR = self.degrees_to_distance(dR_deg)
        d_center = (dL + dR) / 2.0

        # Heading from gyro
        theta_gyro = self.get_heading_rad_from_gyro()

        if self.GYRO_W >= 1.0:
            theta_new = theta_gyro
        else:
            dtheta_enc = (dR - dL) / self.width
            theta_enc = self.theta + dtheta_enc
            theta_new = (1 - self.GYRO_W) * theta_enc + self.GYRO_W * theta_gyro

        theta_mid = 0.5 * (self.theta + theta_new)

        self.x += d_center * math.cos(theta_mid)
        self.y += d_center * math.sin(theta_mid)
        self.theta = theta_new

    # ------------------------------------------------------
    # Basic sensing / utility
    # ------------------------------------------------------
    def get_ultrasonic_distance(self):
        return self.ultrasonic_sensor.distance() / 1000.0

    def wait_for_button_press(self):
        while Button.CENTER not in self.brick.buttons.pressed():
            wait(10)

    def beep(self):
        self.brick.speaker.beep()

    # ------------------------------------------------------
    # Motion: straight line
    # ------------------------------------------------------
    def move_forward(self, distance, speed=200):
        """
        Move straight by given distance (meters) along current heading.
        """
        rot_deg = self.distance_to_degrees(distance)

        self.left_motor.run_angle(speed, rot_deg, Stop.BRAKE, wait=False)
        self.right_motor.run_angle(speed, rot_deg, Stop.BRAKE, wait=True)

        self.update_odometry()

    def move_backward(self, distance, speed=200):
        self.move_forward(-distance, speed)

    # ------------------------------------------------------
    # Heading control: no manual angle math in main
    # ------------------------------------------------------
    def turn_to_heading_deg(self, target_deg, speed=120):
        """
        Rotate in place until global heading (from gyro) is target_deg.
        target_deg is in our global frame: 0 = +x, 90 = +y.
        """
        target_deg = self._normalize_deg(target_deg)

        while True:
            current = self._normalize_deg(self.get_heading_deg_from_gyro())
            error = self._normalize_deg(target_deg - current)

            if abs(error) < 1.0:
                break

            direction = 1 if error > 0 else -1
            self.left_motor.run(direction * speed)
            self.right_motor.run(-direction * speed)
            wait(10)

        self.left_motor.stop()
        self.right_motor.stop()
        wait(100)

        # Sync stored theta + encoder baselines
        self.theta = self.get_heading_rad_from_gyro()
        self.reset_motor_baselines()

    def turn(self, angle_deg, speed=120):
        """
        Relative turn by angle_deg (deg) using gyro internally.
        Positive = CCW, Negative = CW.
        """
        current = self.get_heading_deg_from_gyro()
        target = current + angle_deg
        target = self._normalize_deg(target)
        self.turn_to_heading_deg(target, speed=speed)

    def turn_towards_goal(self, goal_x, goal_y, speed=120):
        """
        Point the robot toward (goal_x, goal_y) in world coordinates.
        No explicit angle math in main; all internal here.
        """
        dx = goal_x - self.x
        dy = goal_y - self.y

        # atan2 gives desired global heading in our coordinate system
        desired_theta_rad = math.atan2(dy, dx)
        desired_deg = math.degrees(desired_theta_rad)

        self.turn_to_heading_deg(desired_deg, speed=speed)

    # ------------------------------------------------------
    # (Optional) wall_follow – unchanged logic except we
    # call update_odometry() each loop.
    # ------------------------------------------------------
    def wall_follow(self, start_pos, return_pos,
                    target_distance=0.15, base_speed=200,
                    kp=375, ki=0, kd=50):

        # Assumes pose already set before calling, ignores start_pos for now.
        self.reset_motor_baselines()
        last_error = 0.0
        integral = 0.0
        I_MAX = 0.2
        CORR_CAP = 80.0
        MIN_SPD, MAX_SPD = 80, 240
        MIN_TRAVEL_TO_ALLOW_EXIT = 0.35
        traveled_accum = 0.0
        prev_x, prev_y = self.x, self.y

        def near_return():
            dx = self.x - return_pos[0]
            dy = self.y - return_pos[1]
            return (abs(dx) <= 0.06) and (abs(dy) <= 0.06)

        while True:
            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 0.6:
                distance = 0.30

            # Too close to wall: turn away a bit
            if distance < 0.08:
                self.turn(-8, 60)
                self.reset_motor_baselines()
                prev_x, prev_y = self.x, self.y
                continue

            # Bump sensors: back off + turn away
            if self.touch_sensor_left.pressed() or self.touch_sensor_right.pressed():
                self.move_backward(0.15)
                self.turn(-90, 100)
                self.reset_motor_baselines()
                prev_x, prev_y = self.x, self.y
                continue

            # PID control
            error = target_distance - distance
            derivative = error - last_error
            last_error = error
            integral += error
            integral = max(-I_MAX, min(I_MAX, integral))

            correction = kp * error + ki * integral + kd * derivative
            correction = max(-CORR_CAP, min(CORR_CAP, correction))

            left_speed = base_speed + correction
            right_speed = base_speed - correction
            left_speed = max(MIN_SPD, min(MAX_SPD, left_speed))
            right_speed = max(MIN_SPD, min(MAX_SPD, right_speed))

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

            # Odometry update while moving
            self.update_odometry()

            dx_step = self.x - prev_x
            dy_step = self.y - prev_y
            step = math.sqrt(dx_step * dx_step + dy_step * dy_step)
            traveled_accum += step
            prev_x, prev_y = self.x, self.y

            self.brick.screen.clear()
            dx_ret = abs(self.x - return_pos[0])
            dy_ret = abs(self.y - return_pos[1])
            self.brick.screen.print(
                str(round(dx_ret, 3)) + "\n" +
                str(round(dy_ret, 3)) + "\n" +
                str(round(self.theta, 3))
            )

            if traveled_accum >= MIN_TRAVEL_TO_ALLOW_EXIT and near_return():
                break

            wait(30)

        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()
