#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait, StopWatch

# Lucas Sharpe - 730545934
# Yuvraj Jain  - 730465868


def main():
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
            bot.move(350, 350)
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
        bot.turn(-90, 200)
        
        goal_reached = bot.wall_follow_goal()
    
    bot.turn_towards_point(2.5, 2.5)
    bot.beep()
        


if __name__ == "__main__":
    main()