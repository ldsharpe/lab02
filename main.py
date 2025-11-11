#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
import math


# Lucas Sharpe - 730545934
# Yuvraj Jain  - 730465868

def main():
    bot = Robot()

    bot.wait_for_button_press()

    start_pos  = [0.0, 0.0, 0.0]
    return_pos = [0.0, 0.0, 0.0]

    bot.creep_forward(speed=200)
    while not bot.is_touching_wall():
        bot.update_position()
        wait(30)
    bot.left_motor.stop(); bot.right_motor.stop()

    bot.move_backward(0.18)
    bot.turn(-90, 100) 
    bot.update_position()

    return_pos = [bot.get_x(), bot.get_y(), bot.get_theta()]

    bot.wall_follow(start_pos, return_pos)
    bot.turn(-90, 100)
    
    while abs(0 - bot.get_x()) > 0.03:
        bot.creep_forward(speed=200)


if __name__ == "__main__":
    main()
