import yaml
import time
gKP = 0
gKD = 0
gKI = 0

with open('src/cfg/sub_properties.yaml') as f:
    file = yaml.safe_load(f)
    gKP = file['kp']
    gKD = file['kd']
    gKI = file['ki']

class PID():
    # init with default values in YAML if values are not passed
    def __init__(self, KP = gKP, KD = gKP, KI = gKI):
        self.KP = KP
        self.KD = KD
        self.KI = KI
        self.prev_error = 0
        self.integral = 0
        self.prev_time = 0
    def calculateOutput(self, state, goal):
        error = goal - state
        integral = self.integral + error
        derivative = (error - self.prev_error) / (time.time() - self.prev_time)
        self.prev_error = error
        self.integral = integral
        self.prev_time = time.time()
        return self.KP * error + self.KD * derivative + self.KI * integral