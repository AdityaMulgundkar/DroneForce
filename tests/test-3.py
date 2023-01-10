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

    # Quad +
    m1 = DFMotor(-1,0,1,1)
    m2 = DFMotor(1,0,1,1)
    m3 = DFMotor(0,1,-1,1)
    m4 = DFMotor(0,-1,-1,1)
    
    # Quad X
    # m1 = DFMotor(-1,1,1,1)
    # m2 = DFMotor(1,-1,1,1)
    # m3 = DFMotor(1,1,-1,1)
    # m4 = DFMotor(-1,-1,-1,1)

    frame = DFFrame([m1,m2,m3,m4])

    with DroneForce(mass=mass, inertia=inertia, frame=frame) as drone:
        logging.debug("Ready: %s", drone)

        ea_matrix = drone.frame.EA
        ca_matrix = drone.frame.CA
        logging.info(f'Effectiveness: \n{ea_matrix}')
        logging.info(f'Control Allocation: \n{ca_matrix}')

        # newtonian method
        # u matrix = u1, u2, u3, u4
        # roll = T0, pitch = T1, yaw = T2; T is torque
        # thrustX = t0, thrustX = t1, thrustX = t2; t is thrust

        # Following is only for Quad-X; seen in:
        # https://doi.org/10.3182/20110828-6-IT-1002.02016
        # https://doi.org/10.1109/ICUAS54217.2022.9836215

        # eulerlagrange method
        # f1 = 
        # uf = f1 + f2 + f3 + f4
        # l = arm length
        # Tp = l * (f4 − f2)
        # Tq = l * (f3 − f1)
        # d = drag_coeff / thrust_coeff
        # Tr = d(f1 − f2 + f3 − f4)

        # EL improved
        # k = lift constant
        # Tp = k * l * (f4 − f2)
        # Tq = k * l * (f3 − f1)
        # d = drag_coeff / thrust_coeff
        # Tr = k * d(f1 − f2 + f3 − f4)

