# Code for Shakaar robot
#
# By Mike Horne, based on code by Tom Oinn/Emma Norling code

# Need floating point division of integers
from __future__ import division

from time import sleep
import sys
import subprocess

g_motor_left_multiplier = -0.5
g_motor_right_multiplier = 0.5

try:
    import ThunderBorg
    TB=ThunderBorg.ThunderBorg()
    TB.Init()
    print("ThunderBorg initialised")

    if not TB.foundChip:
        print("No Thunderborg found")
        sys.exit()

    def set_speeds(power_left, power_right):
        """
        As we have an motor hat, we can use the motors

        :param power_left: 
            Power to send to left motor
        :param power_right: 
            Power to send to right motor, will be inverted to reflect chassis layout
        """

        motor_left_multiplier = g_motor_left_multiplier
        motor_right_multiplier = g_motor_right_multiplier

        # If one wants to see the 'raw' 0-100 values coming in
        # print("source left: {}".format(power_left))
        # print("source right: {}".format(power_right))

        # Take the 0-100 inputs down to 0-1 and reverse them if necessary
        power_left = (motor_left_multiplier * power_left) / 100
        power_right = (motor_right_multiplier * power_right) / 100

        # Print the converted values out for debug
        # print("left: {}".format(power_left))
        # print("right: {}".format(power_right))

        # If power is less than 0, we want to turn the motor backwards, otherwise turn it forwards
        TB.SetMotor2(power_left)
        TB.SetMotor1(power_right)

    def stop_motors():
        TB.MotorsOff()

except ImportError:
    print("Something occurred. Printing values instead")

    def set_speeds(power_left, power_right):
        """
        No motor hat - print what we would have sent to it if we'd had one.
        """

        motor_left_multiplier = g_motor_left_multiplier
        motor_right_multiplier = g_motor_left_multiplier
        power_left = (motor_left_multiplier * power_left) / 100
        power_right = (motor_right_multiplier * power_right) / 100

        print('DEBUG Left: {}, Right: {}'.format(power_left, power_right))
        sleep(0.3)


    def stop_motors():
        """
        No motor hat, so just print a message.
        """
        print('DEBUG Motors stopping')

# All we need, as we don't care which controller we bind to, is the ControllerResource
from approxeng.input.selectbinder import ControllerResource


# Enable logging of debug messages, by default these aren't shown
# import logzero
# logzero.setup_logger(name='approxeng.input', level=logzero.logging.DEBUG)

class RobotStopException(Exception):
    """
    The simplest possible subclass of Exception, we'll raise this if we want to stop the robot
    for any reason. Creating a custom exception like this makes the code more readable later.
    """
    pass


def mixer(yaw, throttle, max_power=100):
    """
    Mix a pair of joystick axes, returning a pair of wheel speeds. This is where the mapping from
    joystick positions to wheel powers is defined, so any changes to how the robot drives should
    be made here, everything else is really just plumbing.
    
    :param yaw: 
        Yaw axis value, ranges from -1.0 to 1.0
    :param throttle: 
        Throttle axis value, ranges from -1.0 to 1.0
    :param max_power: 
        Maximum speed that should be returned from the mixer, defaults to 100
    :return: 
        A pair of power_left, power_right integer values to send to the motor driver
    """
    left = throttle + yaw
    right = throttle - yaw
    scale = float(max_power) / max(1, abs(left), abs(right))
    return int(left * scale), int(right * scale)


# Outer try / except catches the RobotStopException we just defined, which we'll raise when we want to
# bail out of the loop cleanly, shutting the motors down. We can raise this in response to a button press
try:
    while True:
        # Inner try / except is used to wait for a controller to become available, at which point we
        # bind to it and enter a loop where we read axis values and send commands to the motors.
        try:
            # Bind to any available joystick, this will use whatever's connected as long as the library
            # supports it.
            with ControllerResource(print_events=False, controller_class=None, hot_zone=0.2, dead_zone=0.1) as joystick:
                print('Found a controller, HOME to exit, Left Stick to drive')

                # Loop until the joystick disconnects, or we deliberately stop by raising a
                # RobotStopException
                while joystick.connected:
                    print("Connected")

                    # Get joystick values from the left analogue stick
                    x_axis, y_axis = joystick['rx', 'ly']

                    # Get power from mixer function
                    power_left, power_right = mixer(yaw=x_axis, throttle=y_axis)

                    # Set motor speeds
                    set_speeds(power_left, power_right)

                    # Get a ButtonPresses object containing everything that was pressed since the last
                    # time around this loop.
                    button_presses = joystick.check_presses()

                    # Print out any buttons that were pressed, if we had any
                    if button_presses.has_presses:
                        print('Button presses: {}'.format(button_presses))

                    # If home was pressed, raise a RobotStopException to bail out of the loop
                    # Home is generally the PS button for playstation controllers, XBox for XBox etc
                    if 'triangle' in button_presses:
                        raise RobotStopException()

                    if 'dright' in button_presses:
                        if 'square' in button_presses:
                            command = '/usr/bin/sudo /sbin/shutdown -h now'
                            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
                            output = process.communicate()[0]
                            print output

                    if 'dleft' in button_presses:
                        if 'circle' in button_presses:
                            command = '/usr/bin/sudo /sbin/shutdown -r now'
                            process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
                            output = process.communicate()[0]
                            print output

                    if 'r2' in button_presses:
                        g_motor_left_multiplier = g_motor_left_multiplier - 0.1
                        g_motor_right_multiplier = g_motor_right_multiplier + 0.1

                    if 'r1' in button_presses:
                        g_motor_left_multiplier = g_motor_left_multiplier + 0.1
                        g_motor_right_multiplier = g_motor_right_multiplier - 0.1

                    if g_motor_left_multiplier < -1.0:
                        g_motor_left_multiplier = -1.0

                    if g_motor_right_multiplier > 1.0:
                        g_motor_right_multiplier = 1.0

                    if g_motor_left_multiplier > 0:
                        g_motor_left_multiplier = 0

                    if g_motor_right_multiplier < 0:
                        g_motor_right_multiplier = 0

                print('Joystick not connected')
                stop_motors()
                x_axis, y_axis = joystick['lx', 'ly']
                button_presses = joystick.check_presses()


        except IOError:
            # We get an IOError when using the ControllerResource if we don't have a controller yet,
            # so in this case we just wait a second and try again after printing a message.
            print('No controller found yet')
            sleep(1)

except RobotStopException:
    # This exception will be raised when the home button is pressed, at which point we should
    # stop the motors.
    stop_motors()
    # And exit
