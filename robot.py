from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog

import math, time


class Robot:
    def __init__(self):
        self.brick = EV3Brick()

        # Physical
        self.length       = 0.18   # meters
        self.width        = 0.111  # meters (track width L)
        self.height       = 0.128  # meters
        self.wheel_radius = 0.028  # meters

        # Pose (radians for theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Encoder baselines (deg)
        self.prev_left_deg  = 0.0
        self.prev_right_deg = 0.0

        # Heading fusion weight (1.0 = gyro only)
        self.GYRO_W = 1.0

        # Motors
        self.left_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)

        # Sensors
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        self.touch_sensor_left = TouchSensor(Port.S2)
        self.touch_sensor_right = TouchSensor(Port.S3)
        self.gyro = GyroSensor(Port.S4)

        # Gyro heading bias (we only use built-in angle; no unwrap, no rate)
        self.heading_bias_deg = 0.0

        # Initialize everything (includes clean gyro init)
        self.reset_odometry()


    # ------------------ Utilities ------------------

    def reset_motor_baselines(self):
        """Capture current motor angles as new baselines for delta computation."""
        self.prev_left_deg = self.left_motor.angle()
        self.prev_right_deg = self.right_motor.angle()

    def reset_odometry(self, x=0.0, y=0.0, theta=0.0):
        """Reset (x, y, theta) AND motor baselines AND gyro reference."""
        self.x, self.y, self.theta = x, y, theta
        # Reset motor encoders to 0 for simpler reasoning
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.reset_motor_baselines()
        # Realign gyro reference so current gyro angle = provided theta
        self.initialize_gyroscope(reference_theta_rad=theta)

    def _wrap_pi(self, a):
        """Wrap angle (rad) into (-pi, pi]."""
        while a <= -math.pi: a += 2*math.pi
        while a >   math.pi: a -= 2*math.pi
        return a

    # Smart, robust "turn to face" that avoids huge spins
    def turn_towards_point(self, x_tgt, y_tgt, prefer_right=True, speed=100, near_tol=0.02):
        """
        Turn to face the vector from (x,y) -> (x_tgt,y_tgt) using current heading (gyro).
        If prefer_right=True, try to choose CW (right) when sensible; otherwise take shortest.
        Skips turning if we're essentially at the target point.
        """
        # Ensure pose is fresh
        self.update_position()
        dx = x_tgt - self.x
        dy = y_tgt - self.y
        dist = math.hypot(dx, dy)
        if dist < near_tol:
            return  # don't spin in place when sitting on the target

        th = self.theta
        th_des = math.atan2(dy, dx)
        dth = self._wrap_pi(th_des - th)   # shortest turn in (-pi, pi]

        if prefer_right and dth > 0:
            # Only force a CW wrap if the "shortest left" is fairly large
            if dth >= math.radians(90):
                dth = dth - 2*math.pi  # make it negative (CW)
            # else: keep the small CCW (shortest) turn

        # Execute
        self.turn(math.degrees(dth), speed)

    # ------------------ Gyro handling ------------------

    def initialize_gyroscope(self, reference_theta_rad=0.0):
        """
        Reset the EV3 gyro while the robot is absolutely still.
        Use only the built-in angle (no rate, no manual unwrap).
        """
        # Ensure motors are stopped (no vibration during reset)
        self.left_motor.stop()
        self.right_motor.stop()

        self.gyro.reset_angle(0)
        wait(1500)  # let it settle while motionless

        # Bias so that reported heading starts at desired reference angle
        self.heading_bias_deg = math.degrees(reference_theta_rad)

    def read_gyro_heading_rad(self):
        """
        Heading = bias + raw angle from sensor (Pybricks provides a continuous angle that can exceed 360°).
        """
        raw_deg = self.gyro.angle()   # degrees
        heading_deg = self.heading_bias_deg + raw_deg
        return math.radians(heading_deg)


    # ------------------ Convenience getters ------------------

    def get_ultrasonic_distance(self):
        return self.ultrasonic_sensor.distance() / 1000.0  # meters

    def get_x(self): return self.x
    def get_y(self): return self.y
    def get_theta(self): return self.theta

    def distance_to_degrees(self, distance):
        return (distance / (2 * math.pi * self.wheel_radius)) * 360.0

    def degrees_to_distance(self, degrees):
        return (degrees / 360.0) * (2 * math.pi * self.wheel_radius)


    # ------------------ Motion primitives ------------------

    def creep_forward(self, speed=200):
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        # Streaming: pose updated in your loop via update_position()

    def move_forward(self, distance, speed=200):
        """Blocking forward move by distance (meters)."""
        # Optional fudge
        distance += 0.02
        rotation_degrees = self.distance_to_degrees(distance)
        self.left_motor.run_angle(speed,  rotation_degrees, Stop.BRAKE, wait=False)
        self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)
        # After a blocking move, re-baseline encoders to avoid a big delta on resume
        self.reset_motor_baselines()
        # One pose update, using fresh gyro heading
        self.update_position()

    def move_backward(self, distance, speed=200):
        self.move_forward(-distance, speed=speed)

    def turn(self, angle_deg, speed=100):
        """
        Positive angle = CCW (left turn). Negative = CW (right turn).
        Uses differential wheel rotation. Heading is NOT edited here — we trust the gyro.
        """
        L = self.width
        r = self.wheel_radius

        arc_length = (math.pi * (L / 2.0) * abs(angle_deg)) / 180.0
        rotation_degrees = (arc_length / (2 * math.pi * r)) * 360.0

        if angle_deg > 0:
            # left motor backward, right motor forward
            self.left_motor.run_angle(speed,  -rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed,  rotation_degrees,  Stop.BRAKE, wait=True)
        else:
            self.left_motor.run_angle(speed,   rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=True)

        # Re-baseline and update pose once after the turn
        self.reset_motor_baselines()
        self.update_position()


    # ------------------ Odometry (the only place we edit pose) ------------------

    def update_position(self):
        """
        Compute (x, y, theta) using encoder deltas for distance and
        gyro angle (optionally blended with encoder heading) for theta.
        """
        r = self.wheel_radius
        L = self.width

        # Read current motor angles (deg)
        left_deg  = self.left_motor.angle()
        right_deg = self.right_motor.angle()

        # Convert to traveled distances (meters)
        dL = (left_deg  - self.prev_left_deg)  * (math.pi / 180.0) * r
        dR = (right_deg - self.prev_right_deg) * (math.pi / 180.0) * r

        # Store baselines for next step
        self.prev_left_deg  = left_deg
        self.prev_right_deg = right_deg

        # Encoder-based incremental heading
        dtheta_enc = (dR - dL) / L

        # Gyro heading (absolute, from sensor + bias)
        theta_gyro = self.read_gyro_heading_rad()

        # Blend; set self.GYRO_W=1.0 for gyro-only
        theta_pred = self.theta + dtheta_enc
        theta_new = self.GYRO_W * theta_gyro + (1.0 - self.GYRO_W) * theta_pred

        # Use midpoint heading for integrating position
        ds = 0.5 * (dR + dL)
        theta_mid = self.theta + 0.5 * (theta_new - self.theta)

        self.x += ds * math.cos(theta_mid)
        self.y += ds * math.sin(theta_mid)
        self.theta = theta_new


    # ------------------ UI helpers ------------------

    def show_gyro_angle(self):
        self.brick.screen.clear()
        self.brick.screen.draw_text(0, 20, "Gyro angle display")
        wait(1000)
        while True:
            angle_deg = math.degrees(self.read_gyro_heading_rad())
            self.brick.screen.clear()
            self.brick.screen.print("Angle: " + str(round(angle_deg, 1)))
            if Button.CENTER in self.brick.buttons.pressed():
                self.brick.speaker.beep()
                break
            wait(100)

    def is_touching_wall(self):
        return self.touch_sensor_left.pressed() and self.touch_sensor_right.pressed()

    def beep(self):
        self.brick.speaker.beep()

    def wait_for_button_press(self):
        while Button.CENTER not in self.brick.buttons.pressed():
            wait(10)


    # ------------------ Wall follow (tidied) ------------------

    def wall_follow(
        self,
        start_pos,
        return_pos,
        target_distance=0.15,
        base_speed=100,
        kp=150, ki=15, kd=1.5
    ):
        """
        Follow the LEFT wall and stop when near return_pos (x,y).
        Uses single pose update per loop and gyro-only (or blended) heading.
        Requires moving a minimum distance before exiting to avoid "instant finish".
        """
        # Make sure encoder baselines are fresh as we enter streaming control
        self.reset_motor_baselines()

        last_error = 0.0
        integral = 0.0

        I_MAX = 0.2
        CORR_CAP = 80.0
        MIN_SPD, MAX_SPD = 40, 120

        # Require a minimum traveled distance before allowing "near return"
        MIN_TRAVEL_TO_ALLOW_EXIT = 0.35  # meters
        traveled_accum = 0.0
        prev_x, prev_y = self.x, self.y

        def near_return():
            dx = self.x - return_pos[0]
            dy = self.y - return_pos[1]
            return (abs(dx) <= 0.03) and (abs(dy) <= 0.02)

        while True:
            # Read distance; clamp outliers
            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 0.6:
                distance = target_distance  # fall back to neutral target

            # If too close, do a small right nudge
            if distance < 0.08:
                self.turn(-8, 60)     # heading handled by gyro
                prev_x, prev_y = self.x, self.y
                continue

            # Bumper safety
            if self.touch_sensor_left.pressed() or self.touch_sensor_right.pressed():
                self.move_backward(0.15)
                self.turn(-90, 100)
                prev_x, prev_y = self.x, self.y
                continue

            # PID
            error = target_distance - distance
            derivative = error - last_error
            last_error = error
            integral += error
            integral = max(-I_MAX, min(I_MAX, integral))

            correction = kp * error + ki * integral + kd * derivative
            correction = max(-CORR_CAP, min(CORR_CAP, correction))

            left_speed  = base_speed + correction
            right_speed = base_speed - correction
            left_speed  = max(MIN_SPD, min(MAX_SPD, left_speed))
            right_speed = max(MIN_SPD, min(MAX_SPD, right_speed))

            # Command wheels for this tick
            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

            # Single odometry update for this tick
            self.update_position()

            # Accumulate actual traveled distance to gate exit condition
            dx = self.x - prev_x
            dy = self.y - prev_y
            step = math.sqrt(dx*dx + dy*dy)

            traveled_accum += step
            prev_x, prev_y = self.x, self.y

            # Display (optional)
            self.brick.screen.clear()
            dx = abs(self.x - return_pos[0])
            dy = abs(self.y - return_pos[1])
            self.brick.screen.print(
                str(round(dx, 3)) + "\n" + str(round(dy, 3)) + "\n" + str(round(self.theta, 3))
            )

            # Only allow exit if we've actually gone somewhere
            if traveled_accum >= MIN_TRAVEL_TO_ALLOW_EXIT and near_return():
                break

            wait(30)

        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()

    # ------------------ Point-to-point helper ------------------

    def drive_to_point(self, x_tgt, y_tgt, speed=180, stop_tol=0.03, prefer_right=True):
        """
        Face target, then drive straight to it. Uses odometry distance and gyro heading.
        """
        # 1) Face target
        self.turn_towards_point(x_tgt, y_tgt, prefer_right=prefer_right, speed=100)

        # 2) Drive the straight-line distance
        self.update_position()
        dx = x_tgt - self.x
        dy = y_tgt - self.y
        dist = math.hypot(dx, dy)
        if dist > stop_tol:
            self.move_forward(dist, speed=speed)
