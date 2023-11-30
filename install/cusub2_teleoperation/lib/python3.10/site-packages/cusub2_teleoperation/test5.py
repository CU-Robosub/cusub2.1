"""
an original attempt to spin motors
not used currently (10/14/2023)
"""

import serial
import time
import sys

def callback(args):
    # Define the serial port (change this to the appropriate serial port)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    print(ser)
    # Define the channel to which your motor is connected
    # channel = int(args[1])  # Adjust this to your motor's channel
    channel = 0
    # channel2 = 1
    # Set the target PWM value (range: 4000 to 8000 for typical servos)
    target_pwm = 4*int(args[2])  # Adjust this as needed
    # target_pwm_1 = 1700*4
    # target_pwm_2 = 1700*4
    # Calculate the target PWM as a 4-byte value
    target_pwm_bytes = [(target_pwm & 0x7F), ((target_pwm >> 7) & 0x7F)]
    # target_pwm_bytes2 = [(target_pwm_2 & 0x7F), ((target_pwm_2 >> 7) & 0x7F)]
    
    # Command to set the target PWM value
    set_pwm_cmd = [0x84, channel] + target_pwm_bytes
    # set_pwm_cmd_2 = [0x84, channel2] + target_pwm_bytes2

    neutral_pwm = 4*1490  # Adjust this if needed
    neutral_pwm_bytes = [(neutral_pwm & 0x7F), ((neutral_pwm >> 7) & 0x7F)]
    set_neutral_cmd = [0x84, channel] + neutral_pwm_bytes
    # set_neutral_cmd2 = [0x84, channel2] + neutral_pwm_bytes



    try:
        # Send the command to set the target PWM value
        ser.write(bytearray(set_pwm_cmd))
        # ser.write(bytearray(set_pwm_cmd_2))

        # Run the motor for 5 seconds
        time.sleep(int(args[3]))
        # time.sleep(2)
        ser.write(bytearray(set_neutral_cmd))
        print(target_pwm)
        # time.sleep(2)
        # ser.write(bytearray(set_neutral_cmd2))

    finally:
        # Stop the motor by setting the target PWM to the neutral position

        ser.write(bytearray(set_neutral_cmd))
        # ser.write(bytearray(set_neutral_cmd2))


        # Close the serial port when done
        ser.close()

callback(sys.argv)

"""
usage: python3 test5.py [channel] [target pwm] [time]
"""
