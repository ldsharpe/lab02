from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
import math, time

class Robot:
    def __init__(self):
        self.brick = EV3Brick()
        self.length = 0.18
        self.width = 0.122
        self.height = 0.128
        self.wheel_radius = 0.028
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_left_deg = 0.0
        self.prev_right_deg = 0.0

        self.GYRO_W = 1.0

        self.left_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        self.touch_sensor_left = TouchSensor(Port.S2)
        self.touch_sensor_right = TouchSensor(Port.S3)

        self.gyro = GyroSensor(Port.S4)
        self.heading_bias_deg = 0.0
        self.reset_odometry()

    def reset_motor_baselines(self):
        self.prev_left_deg = self.left_motor.angle()
        self.prev_right_deg = self.right_motor.angle()

    def reset_odometry(self, x=0.0, y=0.0, theta=0.0):
        self.x, self.y, self.theta = x, y, theta
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)
        self.reset_motor_baselines()
        self.initialize_gyroscope(reference_theta_rad=theta)

    def _wrap_pi(self, a):
        while a <= -math.pi: a += 2*math.pi
        while a > math.pi: a -= 2*math.pi
        return a

    def turn_towards_point(self, x_tgt, y_tgt, prefer_right=True, speed=100, near_tol=0.02):
        self.update_position()
        dx = x_tgt - self.x
        dy = y_tgt - self.y
        dist = math.hypot(dx, dy)

        if dist < near_tol:
            return
        
        th = self.theta
        th_des = math.atan2(dy, dx)
        dth = self._wrap_pi(th_des - th)

        if prefer_right and dth > 0:
            if dth >= math.radians(90):
                dth = dth - 2*math.pi
        self.turn(math.degrees(dth), speed)

    def initialize_gyroscope(self, reference_theta_rad=0.0):
        self.left_motor.stop()
        self.right_motor.stop()
        self.gyro.reset_angle(0)
        wait(1500)
        self.heading_bias_deg = math.degrees(reference_theta_rad)

    def read_gyro_heading_rad(self):
        raw_deg = self.gyro.angle()
        heading_deg = self.heading_bias_deg - raw_deg
        return math.radians(heading_deg)

    def get_ultrasonic_distance(self):
        return self.ultrasonic_sensor.distance() / 1000.0

    def get_x(self): return self.x
    def get_y(self): return self.y
    def get_theta(self): return self.theta

    def distance_to_degrees(self, distance):
        return (distance / (2 * math.pi * self.wheel_radius)) * 360.0

    def degrees_to_distance(self, degrees):
        return (degrees / 360.0) * (2 * math.pi * self.wheel_radius)

    def creep_forward(self, speed=200):
        self.left_motor.run(speed)
        self.right_motor.run(speed)

    def move_forward(self, distance, speed=200):
        distance += 0.02
        rotation_degrees = self.distance_to_degrees(distance)
        self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
        self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)

        self.update_position()

    def move_backward(self, distance, speed=200):
        self.move_forward(-distance, speed=speed)

    
    def move(self, left_speed, right_speed):
        self.left_motor.run(left_speed)
        self.right_motor.run(right_speed)

    
    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()


    def turn(self, angle_deg, speed=100):
        L = self.width
        r = self.wheel_radius

        arc_length = (math.pi * (L / 2.0) * abs(angle_deg)) / 180.0
        rotation_degrees = (arc_length / (2 * math.pi * r)) * 360.0

        if angle_deg > 0:
            self.left_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)
        else:
            self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=True)

        self.update_position()

    def update_position(self):
        r = self.wheel_radius

        # --- 1. Read encoder angles ---
        left_deg  = self.left_motor.angle()
        right_deg = self.right_motor.angle()

        # Convert degree change → meters for each wheel
        dL = math.radians(left_deg  - self.prev_left_deg) * r
        dR = math.radians(right_deg - self.prev_right_deg) * r

        # Save encoder state for next step
        self.prev_left_deg  = left_deg
        self.prev_right_deg = right_deg

        # Forward motion (average of both wheels)
        ds = 0.5 * (dL + dR)

        # --- 2. Get new heading from gyro ---
        theta_prev = self.theta
        theta_g = self._wrap_pi(self.read_gyro_heading_rad())
        dtheta_g = self._wrap_pi(theta_g - theta_prev)

        # --- 3. Update x,y ---
        if abs(dtheta_g) < 1e-6:
            # Essentially straight: use heading at midpoint for better accuracy
            theta_mid = theta_prev  # or 0.5 * (theta_prev + theta_g), almost same
            self.x += ds * math.cos(theta_mid)
            self.y += ds * math.sin(theta_mid)
        else:
            # Robot moved on a circular arc of radius R = ds / dtheta
            R = ds / dtheta_g
            self.x += R * (math.sin(theta_prev + dtheta_g) - math.sin(theta_prev))
            self.y -= R * (math.cos(theta_prev + dtheta_g) - math.cos(theta_prev))

        # --- 4. Commit new heading ---
        self.theta = theta_g



    def show_gyro_angle(self):
        self.brick.screen.clear()
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

    def wall_follow(self, start_pos, return_pos, target_distance=0.15, base_speed=200, kp=375, ki=0, kd=50):
        self.reset_motor_baselines()
        last_error = 0.0
        integral = 0.0
        I_MAX = 0.2
        CORR_CAP = 150.0
        MIN_SPD, MAX_SPD = 80, 280
        MIN_TRAVEL_TO_ALLOW_EXIT = 0.35
        traveled_accum = 0.0
        prev_x, prev_y = self.x, self.y

        def near_return():
            dx = self.x - return_pos[0]
            dy = self.y - return_pos[1]
            return (abs(dx) <= 0.03) and (abs(dy) <= 0.08)

        while True: 
            distance = self.get_ultrasonic_distance()
            if self.touch_sensor_left.pressed() or self.touch_sensor_right.pressed():
                self.move_backward(0.15)
                self.turn(-90, 100)
                prev_x, prev_y = self.x, self.y
                continue
            if distance <= 0 or distance > 0.6:
                distance = 0.30
            if distance < 0.08:
                self.turn(-8, 60)
                prev_x, prev_y = self.x, self.y
                continue


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
            self.update_position()

            dx = self.x - prev_x
            dy = self.y - prev_y
            step = math.sqrt(dx*dx + dy*dy)
            traveled_accum += step
            prev_x, prev_y = self.x, self.y
            self.brick.screen.clear()
            dx = abs(self.x - return_pos[0])
            dy = abs(self.y - return_pos[1])
            self.brick.screen.print(
                str(round(dx, 3)) + "\n" + str(round(dy, 3)) + "\n" + str(round(self.theta, 3))
            )
            if traveled_accum >= MIN_TRAVEL_TO_ALLOW_EXIT and near_return():
                break
            wait(30)
        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()

    def drive_to_point(self, x_tgt, y_tgt, speed=180, stop_tol=0.03, prefer_right=True):
        self.turn_towards_point(x_tgt, y_tgt, prefer_right=prefer_right, speed=100)
        self.update_position()
        dx = x_tgt - self.x
        dy = y_tgt - self.y
        dist = math.hypot(dx, dy)

        if dist > stop_tol:
            self.move_forward(dist, speed=speed)

    
    def print_pose(self):
        self.brick.screen.print(
                str(round(self.x, 3)) + "\n" + str(round(self.y, 3)) + "\n" + str(round(self.theta, 3))
            )

    def move_angle(self, left_speed, right_speed):
        while True:
            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)
            print(self.left_motor.angle(), self.right_motor.angle())

