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

        # Motors
        self.left_motor = Motor(Port.B, Direction.CLOCKWISE)
        self.right_motor = Motor(Port.C, Direction.CLOCKWISE)

        # Sensors
        self.ultrasonic_sensor = UltrasonicSensor(Port.S1)
        self.touch_sensor_left = TouchSensor(Port.S2)
        self.touch_sensor_right = TouchSensor(Port.S3)
        self.gyro_sensor = GyroSensor(Port.S4)

        # Initialize Sensors
        self.gyro_sensor.reset_angle(0)


    def get_ultrasonic_distance(self):
        return self.ultrasonic_sensor.distance() / 1000

    def distance_to_degrees(self, distance):
        return (distance / (2 * math.pi * self.wheel_radius)) * 360

    def degrees_to_distance(self, degrees):
        rotations = degrees / 360
        distance = rotations * (2 * math.pi * self.wheel_radius)
        return distance

    def creep_forward(self, speed=100):
        self.left_motor.run(speed)
        self.right_motor.run(speed)

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
    
    def move_backward(self, distance, speed=200):
        self.move_forward(-distance)
    
    def is_touching_wall(self):
        return self.touch_sensor_left.pressed() and self.touch_sensor_right.pressed()

    def beep(self):
        self.brick.speaker.beep()

    def wait_for_button_press(self):
        while Button.CENTER not in self.brick.buttons.pressed():
            wait(10)
        
    def wall_follow(self, target_distance=0.15, base_speed=100, kp=120, ki=0, kd=40):
        last_error = 0
        distance_travelled = 0

        while True:
            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 1.0:
                distance = target_distance

            if distance > 0.5:
                while distance > 0.25:
                    self.turn(10, 50)
                    distance = self.get_ultrasonic_distance()
                    wait(100)

            error = target_distance - distance
            derivative = error - last_error
            last_error = error

            correction = (kp * error) + (kd * derivative)

            # Restrict correction strength
            correction = max(min(correction, 50), -50)

            left_speed = base_speed + correction
            right_speed = base_speed - correction

            left_speed = max(min(left_speed, 120), 40)
            right_speed = max(min(right_speed, 120), 40)

            self.left_motor.run(left_speed)
            self.right_motor.run(right_speed)

            wait(30)

    def turn(self, angle, speed):
        wheel_base_radius = self.width / 2
        arc_length = (math.pi * wheel_base_radius * abs(angle)) / 180
        rotation_degrees = (arc_length / (2 * math.pi * self.wheel_radius)) * 360

        if angle > 0:
            self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=True)
        else:
            self.left_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)


    def wall_follow_left(
        self,
        target_distance=0.16,    
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

        while distance_travelled < follow_distance:
            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 1.0:
                distance = target_distance

            # If wall too close, we move the robot right slightly
            if distance < 0.05 or distance > 0.50 :
                self.turn(8, 50)
                wait(100)
                distance = self.get_ultrasonic_distance()

            # PID error control
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

            # Tracking distance travelled using Differential Drive Kinmatics
            cur_L = self.left_motor.angle()
            cur_R = self.right_motor.angle()
            dL = (cur_L - last_L) * (2 * math.pi * r) / 360.0
            dR = (cur_R - last_R) * (2 * math.pi * r) / 360.0
            last_L, last_R = cur_L, cur_R
            ds = 0.5 * (dL + dR)
            distance_travelled += abs(ds)
            wait(30)

        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()
