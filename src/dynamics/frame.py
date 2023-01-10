import numpy as np

class DFFrame:
  def __init__(self, motors):
    self.motors = motors
    self.EA = []
    self.CA = []

  def getEA(self):
    motors = []
    for motor in self.motors:
        m = [motor.roll, motor.pitch, motor.yaw, motor.thrust]
        motors.append(m)

    return motors