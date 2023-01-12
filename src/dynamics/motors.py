class DFMotor:
  def __init__(self, roll, pitch, yaw, thrust, thrust_coeff=0, moment_coeff=0, lx=0, ly=0, lz=0):
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw
    self.thrust = thrust
    # lift co-efficient
    self.thrust_coeff = thrust_coeff
    # drag co-efficient
    self.moment_coeff = moment_coeff
    self.distance = [lx, ly, lz]
    self.faulty = False