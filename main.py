#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
import math

def main():
    bot = Robot()
    bot.GYRO_W = 1.0

    bot.wait_for_button_press()

    bot.reset_odometry(x=0.5, y=0.0, theta=math.radians(90))

    bot.turn_towards_goal(2.5, 2.5)
    bot.move_forward(0.5)

    bot.turn(-70)
    bot.move_forward(0.3)

    bot.turn_towards_goal(2.5, 2.5)
    bot.beep()

if __name__ == "__main__":
    main()
