"""
TODO: A simple test file to give custom motor outputs.

How to run:
1. A. Have 2 terminals open
    a. ~/ardupilot_ws/src/ardupilot# ./Tools/autotest/sim_vehicle.py -v ArduCopter --vehicle=ArduCopter --frame=hexa
    B. mavproxy.py --master 127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553
1. B. (OR) Connect real vehicle on USB or RT
2. Open QGC/Ground control - it will auto connect to 127.0.0.1:14550 or 127.0.0.1:14551
3. Run this file
"""

import os
import sys

cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

from src.droneforce import DroneForce
from src.utility.logger import *
from src.dynamics.inertia import DFInertia
from src.dynamics.mass import DFMass
from src.dynamics.frame import DFFrame, Frames
from src.utility.map_range import torque_to_PWM
import numpy as np

if __name__ == '__main__':
    mass = DFMass(1000) # 1kg
    inertia = DFInertia(1,1,1)
    frame = DFFrame(frame_type=Frames.Hexa_X)

    with DroneForce(mass=mass, inertia=inertia, frame=frame) as drone:
        logging.debug("Ready: %s", drone)
        Torq = [0, 0, 0, 0] # Torque for each axis and Thrust
        u_input = np.matmul(drone.frame.CA_inv, Torq)
        # Convert motor torque (input u) to PWM
        PWM_out = []
        for i, input in u_input:
            PWM = torque_to_PWM(input, (frame.frame_type.value[i]))
            PWM_out.append(PWM)
        print(f"PWM outputs: {PWM_out}")