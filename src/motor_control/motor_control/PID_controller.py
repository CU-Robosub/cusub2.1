import yaml
import time

class PID:
    # init with default values in YAML if values are not passed
    def __init__(self, KP = 0, KD = 0, KI = 0):
        self.KP = KP
        self.KD = KD
        self.KI = KI
        self.prev_error = 0
        self.integral = 0
        self.prev_time = 0
    def calculateOutput(self, state, goal):
        error = goal - state
        integral = self.integral + (error * (time.time() - self.prev_time))
        derivative = (error - self.prev_error) / (time.time() - self.prev_time)
        self.prev_error = error
        self.integral = integral
        self.prev_time = time.time()
        return self.KP * error + self.KD * derivative + self.KI * integral

    def setKP(self, p):
        self.KP = p   

    def setKD(self, d):
        self.KD = d
    
    def setKI(self, i):
        self.KI = i

