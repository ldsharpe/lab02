#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
import math

def main():
    bot = Robot()
    bot.GYRO_W = 1.0

    bot.wait_for_button_press()
    bot.reset_odometry(x=0.5, y=0.0, theta=math.radians(90))

    # Turn toward goal initially
    bot.turn_towards_goal(goal_x=2.5, goal_y=2.5)

    # Move forward and turn
    bot.move_forward(0.5)
    bot.turn(-70)
    bot.move_forward(0.3)

    # 🧠 Make sure pose is fresh before next targeting
    bot.update_position()

    # Now turn again toward the same goal
    bot.turn_towards_goal(goal_x=2.5, goal_y=2.5)

    bot.beep()

if __name__ == "__main__":
    main()
