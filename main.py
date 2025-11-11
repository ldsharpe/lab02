#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait

# Lucas Sharpe - 730545934
# Yuvraj Jain  - 730465868

def main():
    bot = Robot()
    bot.GYRO_W = 1.0   # strictly gyro heading

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

    # Back off and turn RIGHT so the front wall becomes the LEFT wall
    bot.move_backward(0.18)
    bot.turn(-90, 100)  # right turn (CW). Heading comes from gyro.

    # Pose to return near (x, y, theta) where wall-follow begins
    return_pos = [bot.get_x(), bot.get_y(), bot.get_theta()]

    # Follow the LEFT wall until we loop back (requires real movement before exit)
    bot.wall_follow(start_pos, return_pos)

    # Now turn toward the original start and drive back there.
    bot.turn_towards_point(start_pos[0], start_pos[1], prefer_right=True, speed=100)
    bot.drive_to_point(start_pos[0], start_pos[1], speed=200, stop_tol=0.03, prefer_right=True)
    bot.beep()


def x():
    bot = Robot()
    bot.wait_for_button_press()
    bot.show_gyro_angle()


if __name__ == "__main__":
    main()
