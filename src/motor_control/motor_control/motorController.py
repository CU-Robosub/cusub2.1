"""
    AUTHOR: JAKE TUCKER
    CONTACT: jakob.tucker@colorado.edu
    PURPOSE: Convert cmd_vel commands to PWM values 
"""
import serial
import time
import sys
from .submodules.Maestro import maestro
import yaml
import logging
from PID_controller import PID
import numpy as np
 

PWM_MULTIPLIER = 0
NEUTRAL_PWM = 1490
with open('src/cfg/sub_properties.yaml') as f:
    file = yaml.safe_load(f)
    PWM_MULTIPLIER = file['PWM_multiplier']
    NEUTRAL_PWM = file['neutral_PWM']
    MAX_PWM = file['max_PWM']
    MIN_PWM = file['min_PWM']
    PORT = file['maestro_port']

class motorController:
    
    def __init__(self):
        # initialize serial port , set baud rate, set timeout
        self.serial = None
        try:
            self.serial = serial.Serial(PORT, 9600, timeout=1) 
        except:
            print("Error opening serial port {port}")
        

    def run(self, channels, target=0, multiplier=PWM_MULTIPLIER, INVERT=False, raw_pwm=False):
        """Sends a PWM command to a set of servos

        Args:
            channels (int[]): list of integer channels from the maestro
            target (int): target PWM value (can be cmd_vel value or raw PWM value)
            multiplier (float, optional): multiplier for cmd_vel values. Defaults to PWM_MULTIPLIER.
            INVERT (bool, optional): inverts the target value. Defaults to False.
            raw_pwm (bool, optional): if True, target is raw PWM value. Defaults to False.
        """
        INVERTER = 1
        if(INVERT):
            INVERTER = -1
            
        if not raw_pwm:
            targetPWM = round(4 * (NEUTRAL_PWM + INVERTER * (target * multiplier)))
        else:
            targetPWM = round(target * 4)
        
        if (targetPWM > MAX_PWM * 4): targetPWM = MAX_PWM * 4
        elif (targetPWM < MIN_PWM * 4): targetPWM = MIN_PWM * 4
        
        targetBytes = [(targetPWM & 0x7F), ((targetPWM >> 7) & 0x7F)]
        for channel in channels: # loop through channels
            finalCommand = [0x84, channel] + targetBytes # Send 4 byte command to maestro
            if self.serial is not None: self.serial.write(bytearray(finalCommand))
        return targetPWM
    
    def calculate_motor_PWM(self, force):
        """
        Takes force input (Newtons), outputs PWM value for motors

        Does this using numpy polyval, which takes polynomial coefficients
        output from MATLAB to approximate the appropriate PWM value based on 
        motor torque curve given here under technical details

        https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/
        
        """
        if force < 0.6 and force > -0.6:
            # DONT TOUCH THESE -> from MATLAB polyfit
            coefficients = [
                357933.734034129,   
                51267.3411185555,   
                -262097.155303832,  
                -25749.0835774908,  
                69454.7836294624,   
                4065.09857423062,   
                -8128.74996899901,  
                -237.488151712237,  
                631.643782510791,   
                1499.76032846881
            ]
        else:
            # DONT TOUCH THESE -> from MATLAB polyfit
            coefficients = [
                0.0226802282868747,  # x^9
                0.0212913733046859,  # x^8
                -0.854526266622636,   # x^7
                -0.323698787684428,   # x^6
                10.5735153825628,    # x^5
                1.92971764787633,    # x^4
                -55.6668551735257,    # x^3
                -8.33712299251340,    # x^2
                241.446557413169,     # x^1
                1495.40998415651      # x^0 (constant)
            ]

        # Compute the output using the polynomial
        output = np.polyval(coefficients, force)

        if output > 1900:
            output = 1900
        if output < 1100:
            output = 1100


        return output
    

    def runDepthPID(self, channels, state, goal, pid):
        """Runs a PID controller on a set of servos

        Args:
            channels (int[]): list of integer channels from the maestro
            state (float): current depth of the system (meters)
            goal (float): goal depth of the system (meters)
            pid (PID): PID controller object
            duration (int, optional): duration of command. Defaults to -1 (runs once).
        """
        target = pid.calculateOutput(state, goal)
        target = self.calculate_motor_PWM(target)
        return self.run(channels, target)

    def killAll(self, channels):
        """Send the neutral PWM command to the list of servos

        Args:
            channels (int[]): list of integer channels from the maestro
        """
        target = 4*NEUTRAL_PWM # neutral target
        targetBytes = [(target & 0x7F), ((target >> 7) & 0x7F)]
        for channel in channels: # kill all channels
            finalCommand = [0x84, channel] + targetBytes
            if self.serial is not None: self.serial.write(bytearray(finalCommand))



 
