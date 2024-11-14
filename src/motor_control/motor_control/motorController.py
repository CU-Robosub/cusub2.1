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
    
    def calculate_motor_PWM(self, velocity):
        # todo: implement this function
        return velocity
    

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