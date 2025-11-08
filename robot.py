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
        self.length       = 0.18        # meters
        self.width        = 0.111       # meters
        self.height       = 0.128       # meters
        self.wheel_radius = 0.028       # meters 


        # Position
        self.x = 0.0      # meters
        self.y = 0.0       
        self.theta = 0.0  
        self.prev_left = 0.0
        self.prev_right = 0.0


        # Motors
        self.left_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)

        # Sensors
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        self.touch_sensor_left = TouchSensor(Port.S2)
        self.touch_sensor_right = TouchSensor(Port.S3)
        #self.gyro_sensor = GyroSensor(Port.S4)

        # Initialize Sensors
        #self.gyro_sensor.reset_angle(0)


    def get_ultrasonic_distance(self):
        return self.ultrasonic_sensor.distance() / 1000
    
    def get_x(self):
        return self.x
    
    def get_y(self):
        return self.y
    
    def get_theta(self):
        return self.theta

    def distance_to_degrees(self, distance):
        return (distance / (2 * math.pi * self.wheel_radius)) * 360

    def degrees_to_distance(self, degrees):
        rotations = degrees / 360
        distance = rotations * (2 * math.pi * self.wheel_radius)
        return distance

    def creep_forward(self, speed=200):
        self.left_motor.run(speed)
        self.right_motor.run(speed)
        self.update_position()

    def move_forward(self, distance, speed=200):
        """
        Moves robot forward by given distance (in meters)
        at a given speed (in deg/s on the motors)
        """

        distance += 0.02

        # Converting distance to wheel rotation required in degrees
        rotation_degrees = (distance / (2 * math.pi * self.wheel_radius)) * 360
        
        # Run both motors
        self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
        self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)

        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)

    
    def move_backward(self, distance, speed=200):
        self.move_forward(-distance)
    
    def is_touching_wall(self):
        return self.touch_sensor_left.pressed() and self.touch_sensor_right.pressed()

    def beep(self):
        self.brick.speaker.beep()

    def wait_for_button_press(self):
        while Button.CENTER not in self.brick.buttons.pressed():
            wait(10)
        
    """Archived wall_follow code which doesn't use integrals. Simpler fallback in case required."""
    # def wall_follow(self, target_distance=0.15, base_speed=100, kp=120, ki=0, kd=40):
    #     last_error = 0
    #     distance_travelled = 0

    #     while True:
    #         distance = self.get_ultrasonic_distance()
    #         if distance <= 0 or distance > 1.0:
    #             distance = target_distance

    #         if distance > 0.5:
    #             while distance > 0.25:
    #                 self.turn(10, 50)
    #                 distance = self.get_ultrasonic_distance()
    #                 wait(100)

    #         error = target_distance - distance
    #         derivative = error - last_error
    #         last_error = error

    #         correction = (kp * error) + (kd * derivative)

    #         # Restrict correction strength
    #         correction = max(min(correction, 50), -50)

    #         left_speed = base_speed + correction
    #         right_speed = base_speed - correction

    #         left_speed = max(min(left_speed, 120), 40)
    #         right_speed = max(min(right_speed, 120), 40)

    #         self.left_motor.run(left_speed)
    #         self.right_motor.run(right_speed)

    #         wait(30)

    def turn(self, angle, speed=100):
        wheel_base_radius = self.width / 2
        arc_length = (math.pi * wheel_base_radius * abs(angle)) / 180
        rotation_degrees = (arc_length / (2 * math.pi * self.wheel_radius)) * 360

        if angle > 0:
            self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=True)
        else:
            self.left_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)
        
        self.theta += angle * (math.pi / 180)


    def wall_follow_left(
        self,
        start_pos,
        return_pos=None,
        target_distance=0.15,    
        follow_distance=2.25,     
        base_speed=100, 
        kp=150, ki=2.0, kd=50
    ):
        # Initialize
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

        last_error = 0.0
        integral = 0.0

        I_MAX = 0.2
        CORR_CAP = 80.0
        MIN_SPD, MAX_SPD = 40, 120

        # Odometry constants
        r = self.wheel_radius
        last_L = 0.0
        last_R = 0.0
        distance_travelled = 0.0
        check = True

        # Keep following the wall until near return_pos (if given)
        while True:
            # Stop if we've reached target position
            if return_pos is not None:
                x, y = self.get_x(), self.get_y()
                if (
                    abs(return_pos[0] - x) <= 0.05
                    and abs(return_pos[1] - y) <= 0.05
                    and not check
                ):
                    break

            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 1.0:
                distance = target_distance

            # If wall too close or too far, adjust with small turn
            if distance < 0.08 or distance > 1.00:
                self.turn(8, 50)
                wait(100)
                distance = self.get_ultrasonic_distance()

            # PID control
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

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)
            self.update_position()
            self.brick.screen.clear()
            self.brick.screen.print(self.get_x(), self.get_y(), self.get_theta())

            # Update distance travelled (kinematics)
            cur_L = self.left_motor.angle()
            cur_R = self.right_motor.angle()
            dL = (cur_L - last_L) * (2 * math.pi * r) / 360.0
            dR = (cur_R - last_R) * (2 * math.pi * r) / 360.0
            last_L, last_R = cur_L, cur_R
            ds = 0.5 * (dL + dR)
            distance_travelled += abs(ds)
            wait(30)

            # Replicate "check" logic from main.py
            if return_pos is not None:
                if abs(self.get_x() - return_pos[0]) > 0.08 and abs(self.get_y() - return_pos[1]) > 0.08:
                    check = False

            # Safety stop if robot travels too far
            if distance_travelled >= follow_distance:
                break

        self.left_motor.stop()
        self.right_motor.stop()

        self.beep()
    
    def update_position(self):
    
        r = self.wheel_radius
        L = self.width     

        left_angle = self.left_motor.angle()
        right_angle = self.right_motor.angle()

        dL = (left_angle - self.prev_left) * (math.pi / 180) * r
        dR = (right_angle - self.prev_right) * (math.pi / 180) * r


        self.prev_left = left_angle
        self.prev_right = right_angle
        ds = (dR + dL) / 2.0
        dtheta = (dR - dL) / L
        self.x += ds * math.cos(self.theta + dtheta / 2.0)
        self.y += ds * math.sin(self.theta + dtheta / 2.0)
        self.theta += dtheta

    
    def wall_follow(
        self,
        start_pos,
        return_pos,
        target_distance=0.15,    
        base_speed=100, 
        kp=250, ki=2.0, kd=50
    ):
        # Initialize motors and PID state
        self.left_motor.reset_angle(0)
        self.right_motor.reset_angle(0)

        last_error = 0.0
        integral = 0.0
        check = True

        I_MAX = 0.2
        CORR_CAP = 80.0
        MIN_SPD, MAX_SPD = 40, 120

        r = self.wheel_radius
        last_L = 0.0
        last_R = 0.0

        print(start_pos)
        print(return_pos)

        while True:
            # --- Position check ---
            x, y = self.get_x(), self.get_y()

            # Stop when within ~5 cm of return position and the "check" flag is cleared
            if (
                abs(return_pos[0] - x) <= 0.10
                and abs(return_pos[1] - y) <= 0.20
                and not check
            ):
                print("fail")
                break

            # --- Ultrasonic measurement ---
            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 1.0:
                distance = 0.25

            # If wall too close or too far, make a small right correction turn
            if distance < 0.08:
                self.turn(8, 50)
                wait(100)
                distance = self.get_ultrasonic_distance()
            
            if self.touch_sensor_left.pressed() or self.touch_sensor_right.pressed():
                self.move_backward(0.15)
                self.turn(90, 100)


            # --- PID control ---
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

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)
            self.update_position()

            # --- Display odometry on EV3 screen ---
            self.brick.screen.clear()
            self.brick.screen.print(str(self.get_x()) + "\n" + str(self.get_y()))

            # --- Update wheel movement for next iteration ---
            cur_L = self.left_motor.angle()
            cur_R = self.right_motor.angle()
            dL = (cur_L - last_L) * (2 * math.pi * r) / 360.0
            dR = (cur_R - last_R) * (2 * math.pi * r) / 360.0
            last_L, last_R = cur_L, cur_R

            wait(30)

            # --- Update check flag logic (from your main.py loop) ---
            if abs(x - return_pos[0]) > 0.12 and abs(y - return_pos[1]) > 0.22:
                check = False

        # Stop motors and beep to confirm completion
        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()

