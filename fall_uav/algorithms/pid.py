import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd, ki_sat, dt):
        self.kp = kp # offset constante
        self.ki = ki # offset acumulado -> tende a nula no infinito
        self.kd = kd # ajusta a resp transitoria
        self.ki_sat = ki_sat
        self.dt = dt #step size in sec

        # Initialize integral term
        self.int = np.zeros(3)

    def reset(self):
        # Reset the integral term
        self.int = np.zeros(3)
    
    def compute(self, pos_error, vel_error):
        # Update integral controller
        self.int += pos_error * self.dt

        #Prevent windup
        over_mag = np.argwhere(np.array(self.int) > np.array(self.ki_sat))
        if over_mag.size != 0:
            for i in range(over_mag.size):
                mag = abs(self.int[over_mag[i][0]]) #get magnitude to find sign (direction)
                self.int[over_mag[i][0]] = (self.int[over_mag[i][0]] / mag) * self.ki_sat[over_mag[i][0]] #maintain direction (sign) but limit to saturation 


        # Calculate controller input for desired acceleration
        des_acc = self.kp * pos_error + self.ki * self.int + self.kd * 0
        return des_acc