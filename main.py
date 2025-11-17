#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait, StopWatch

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

    bot.move(175, 225)
    watch = StopWatch()
    duration = 8000    

    while watch.time() < duration:
        bot.update_position()
        wait(50)

    bot.stop()
    bot.print_pose()
    bot.wait_for_button_press()



if __name__ == "__main__":
    x()
