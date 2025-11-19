#!/usr/bin/env pybricks-micropython
from robot import Robot
<<<<<<< HEAD
from pybricks.tools import wait, StopWatch

# Lucas Sharpe - 730545934
# Yuvraj Jain  - 730465868
=======
from pybricks.tools import wait
import math
>>>>>>> efc0cf5e95e54e471201092bd498750626ea97d4

def main():
    bot = Robot()
    bot.GYRO_W = 1.0

    bot.wait_for_button_press()

    bot.reset_odometry(x=0.5, y=0.0, theta=math.radians(90))

<<<<<<< HEAD
    bot.creep_forward(speed=200)
    while not bot.touch_sensor_left.pressed() and not bot.touch_sensor_right.pressed():
        bot.update_position()
        wait(30)
    bot.left_motor.stop(); bot.right_motor.stop()
=======
    bot.turn_towards_goal(2.5, 2.5)
    bot.move_forward(0.5)
>>>>>>> efc0cf5e95e54e471201092bd498750626ea97d4

    bot.turn(-70)
    bot.move_forward(0.3)

    bot.turn_towards_goal(2.5, 2.5)
    bot.beep()

    return_pos = [bot.get_x(), bot.get_y(), bot.get_theta()]

    bot.wall_follow(start_pos, return_pos)

    bot.turn(-90, 100)
    bot.move_forward(abs(start_pos[0] - return_pos[0]))
    bot.beep()


def x():
    bot = Robot()

    bot.wait_for_button_press()

    goal_reached = False

    while not goal_reached:
        wait(100)
        bot.turn_towards_point(2.5, 2.5)
        wait(100)

        watch = StopWatch()
        watch.reset()

        while not bot.touch_sensor_left.pressed() and not bot.touch_sensor_right.pressed():
            bot.update_position()
            bot.move(200, 200)
            print(bot.get_x(), bot.get_y())
            wait(30)
            if bot.near_goal():
                goal_reached = True
                break
            if watch.time() > 2000:   
                bot.turn_towards_point(2.5, 2.5)
                watch.reset()

        if goal_reached is True:
            break         
        
        bot.move_backward(0.18)
        bot.turn(-90, 100)
        
        goal_reached = bot.wall_follow_goal()
    
    bot.turn_towards_point(2.5, 2.5)
    bot.beep()
        


if __name__ == "__main__":
    x()
