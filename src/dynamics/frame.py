import enum
import numpy as np

from src.dynamics.motors import DFMotor

class DFFrame:
  def __init__(self, frame_type):
    self.EA = []
    self.CA = []
    self.CA_inv = []
    self.frame_type = frame_type
    self.motors = [0] * len(frame_type.value)

    for motor in self.frame_type.value:
        m = [motor.roll, motor.pitch, motor.yaw, motor.thrust]
        self.EA.append(m)
    self.CA = np.linalg.pinv(self.EA,)
    self.CA_inv = np.linalg.pinv(self.CA)

  def inject_fault(self, motor_num):
    self.CA[:,motor_num-1] = 0
    self.CA_inv = np.linalg.pinv(self.CA)

# Using enum class create enumerations
class Frames(enum.Enum):
   Hexa_X = [DFMotor(-1, 0, -1, 1)
    , DFMotor(1, 0, 1, 1),
     DFMotor(0.5,-0.866,-1, 1),
     DFMotor(-0.5,0.866,1, 1),
     DFMotor(-0.5,-0.866,1, 1),
     DFMotor(0.5,0.866,-1, 1)]