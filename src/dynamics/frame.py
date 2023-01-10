import numpy as np

class DFFrame:
  def __init__(self, motors):
    self.motors = motors
    self.EA = []
    for motor in self.motors:
        m = [motor.roll, motor.pitch, motor.yaw, motor.thrust]
        self.EA.append(m)
    self.CA = np.linalg.pinv(self.EA)