"""
TODO: Description of what this file does.

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
from src.dynamics.frame import DFFrame
from src.dynamics.motors import DFMotor

import numpy as np

logging.debug('Beginning of code...')


if __name__ == '__main__':
    mass = DFMass(1000)
    inertia = DFInertia(1,1,1)
    
    m1 = DFMotor(-1,1,1,1)
    m2 = DFMotor(1,-1,1,1)
    m3 = DFMotor(1,1,-1,1)
    m4 = DFMotor(-1,-1,-1,1)

    frame = DFFrame([m1,m2,m3,m4])

    with DroneForce(mass=mass, inertia=inertia, frame=frame) as drone:
        logging.debug("Ready: %s", drone)

        ea_matrix = drone.frame.getEA()
        ca_matrix = np.linalg.pinv(ea_matrix)
        logging.info(f'Effectiveness: \n{ea_matrix}')
        logging.info(f'Control Allocation: \n{ca_matrix}')


        # u matrix = u1, u2, u3, u4

        # roll = T0, pitch = T1, yaw = T2; T is torque

        # thrustX = t0, thrustX = t1, thrustX = t2; t is thrust

