#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
import math


# Lucas Sharpe - 730545934
# Yuvraj Jain  - 730465868

def main():
    bot = Robot()

    # Wait for user
    bot.wait_for_button_press()

    start_pos  = [0.0, 0.0, 0.0]
    return_pos = [0.0, 0.0, 0.0]

    # Approach the (front) wall with live odometry
    bot.creep_forward(speed=200)
    while not bot.is_touching_wall():
        bot.update_position()
        wait(30)
    bot.left_motor.stop(); bot.right_motor.stop()

    # Back off and turn RIGHT so that the front wall becomes the LEFT wall
    bot.move_backward(0.18)
    bot.turn(-90, 100)  # right turn under +θ = CCW

    # Pose to return near (x, y, theta)
    return_pos = [bot.get_x(), bot.get_y(), bot.get_theta()]

    # Optional quick heading print
    bot.brick.screen.clear()
    bot.brick.screen.print("theta(deg): " + str(int(bot.get_theta() * 180.0 / math.pi)))
    wait(400)

    # Follow the LEFT wall until we loop back
    bot.wall_follow(start_pos, return_pos)


if __name__ == "__main__":
    main()
