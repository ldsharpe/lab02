#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
from pybricks.parameters import Button

# Lucas Sharpe - 730545934
# Yuvraj Jain - 730465868

def main():
    bot = Robot()
        
    bot.wait_for_button_press()

    while bot.is_touching_wall() == False:
        bot.creep_forward()
    
    bot.move_backward(0.15)
    bot.turn(90, 100)
    bot.wall_follow_left()


main()