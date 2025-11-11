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
        self.gyro = GyroSensor(Port.S4)
        self.initialize_gyroscope()


    def initialize_gyroscope(self):
        self.gyro.reset_angle(0)
        wait(1000)


    def show_gyro_angle(self):
        self.brick.screen.clear()
        wait(1000)

        while True:
            angle_deg = self.gyro.angle()   

            self.brick.screen.clear()
            self.brick.screen.print("Angle: " + str(angle_deg))

            if Button.CENTER in self.brick.buttons.pressed():
                self.brick.speaker.beep()
                break

            wait(100)

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

        self.update_position()


    
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
            self.left_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=True)
        else:
            self.left_motor.run_angle(speed, rotation_degrees, Stop.BRAKE, wait=False)
            self.right_motor.run_angle(speed, -rotation_degrees, Stop.BRAKE, wait=True)
        
        self.theta += math.radians(angle)
        # self.update_position()

    


 
    def update_position(self):
    
        r = self.wheel_radius
        L = self.width     

        left_angle = self.left_motor.angle()
        right_angle = self.right_motor.angle()

        dL = (left_angle - self.prev_left) * (math.pi / 180) * r
        dR = (right_angle - self.prev_right) * (math.pi / 180) * r

        self.prev_left = left_angle
        self.prev_right = right_angle

       # current_angle = self.gyro.angle()
       # if not hasattr(self, "last_gyro"):
        #    self.last_gyro = current_angle
       # dtheta_gyro = math.radians(current_angle - self.last_gyro)
       # self.last_gyro = current_angle
       # self.theta += dtheta_gyro

        dtheta = (dR - dL) / L
        self.theta += dtheta


        ds = (dR + dL) / 2.0

        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)



    
    def wall_follow(
        self,
        start_pos,
        return_pos,
        target_distance=0.15,    
        base_speed=100, 
        kp=175, ki=1.5, kd=50
    ):
        self.prev_left = 0
        self.prev_right = 0

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
            x, y = self.get_x(), self.get_y()

            if (
                abs(return_pos[0] - self.x) <= 0.04
                and abs(return_pos[1] - self.y) <= 0.04
                and not check
            ):
                break

            self.update_position()


            distance = self.get_ultrasonic_distance()
            if distance <= 0 or distance > 0.5:
                distance = 0.18
            

            if distance < 0.08:
                self.turn(-8, 50)
                wait(100)
                distance = self.get_ultrasonic_distance()
            
            if self.touch_sensor_left.pressed() or self.touch_sensor_right.pressed():
                self.move_backward(0.15)
                self.turn(-90, 100)

            self.update_position()

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
            self.brick.screen.print(str(abs(self.x - return_pos[0])) + "\n" + str(abs(self.y - return_pos[1])) + "\n" + str(self.theta))


            cur_L = self.left_motor.angle()
            cur_R = self.right_motor.angle()
            dL = (cur_L - last_L) * (2 * math.pi * r) / 360.0
            dR = (cur_R - last_R) * (2 * math.pi * r) / 360.0
            last_L, last_R = cur_L, cur_R

            self.update_position()

            wait(30)

            if abs(self.x - return_pos[0]) > 0.12 and abs(self.y - return_pos[1]) > 0.22:
                check = False

        self.left_motor.stop()
        self.right_motor.stop()
        self.beep()