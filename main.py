#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.tools import wait
from pybricks.parameters import Button

# Lucas Sharpe - 730545934
# Yuvraj Jain - 730465868

# accurate distance is goal distance - 0.02m

def main():
    bot = Robot()
        
    bot.wait_for_button_press()

    start_pos = [0, 0, 0]
    return_pos = [0, 0, 0]

    while bot.is_touching_wall() == False:
        bot.creep_forward()
    
    bot.move_backward(0.15)
    bot.turn(90, 100)

    return_pos = [bot.get_x(), bot.get_y(), bot.get_theta()]
    check = True

    bot.wall_follow(start_pos, return_pos)


    



main()