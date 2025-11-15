#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait

# Lucas Sharpe - 730545934
# Yuvraj Jain  - 730465868

def main():
    bot = Robot()
    bot.GYRO_W = 1.0 

    bot.wait_for_button_press()

    start_pos  = [0.0, 0.0, 0.0]
    return_pos = [0.0, 0.0, 0.0]

    bot.creep_forward(speed=200)
    while not bot.touch_sensor_left.pressed() and not bot.touch_sensor_right.pressed():
        bot.update_position()
        wait(30)
    bot.left_motor.stop(); bot.right_motor.stop()

    bot.move_backward(0.18)
    bot.turn(-90, 100)  
    bot.beep()

    return_pos = [bot.get_x(), bot.get_y(), bot.get_theta()]

    bot.wall_follow(start_pos, return_pos)

    bot.turn(-90, 100)
    bot.move_forward(abs(start_pos[0] - return_pos[0]))
    bot.beep()


def x():
    bot = Robot()


    bot.wait_for_button_press()

    # while True:
    #     bot.update_position()
    #     wait(1000)
    bot.move_forward(0.5)
    bot.turn(90, 100)
    bot.move_forward(0.5)
    bot.brick.screen.print("Angle: " + str(bot.get_theta()) + "\n" + str(bot.get_x()) + "\n" + str(bot.get_y()))
    wait(5000)


if __name__ == "__main__":
    x()
