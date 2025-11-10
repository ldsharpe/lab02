from pybricks.hubs import EV3Brick
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

import math


def main():
    brick = EV3Brick()
    gyro = GyroSensor(Port.S4)

    # Reset and calibrate gyro
    brick.screen.clear()
    brick.screen.print("Calibrating...")
    wait(1000)
    gyro.reset_angle(0)
    brick.screen.clear()
    brick.screen.print("Ready!")

    wait(1000)

    # Main test loop
    while True:
        # Get angle and rate
        angle_deg = gyro.angle()     # in degrees
        rate_deg_s = gyro.speed()    # in degrees per second

        # Convert to radians if needed
        angle_rad = math.radians(angle_deg)

        # Display on EV3 screen
        brick.screen.clear()
        brick.screen.print("Angle: {:.2f}°".format(angle_deg))
        brick.screen.print("Rate: {:.2f}°/s".format(rate_deg_s))
        brick.screen.print("Theta: {:.2f} rad".format(angle_rad))

        # Beep every full rotation
        if abs(angle_deg) % 360 < 2:
            brick.speaker.beep()

        # Exit when center button pressed
        if Button.CENTER in brick.buttons.pressed():
            brick.speaker.beep()
            break

        wait(100)


if __name__ == "__main__":
    main()
