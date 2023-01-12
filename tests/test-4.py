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
from src.dynamics.frame import DFFrame, Frames
from src.dynamics.motors import DFMotor

from src.utility.map_range import map_range, torque_to_PWM

import numpy as np

logging.debug('Beginning of code...')


if __name__ == '__main__':
    mass = DFMass(1000)
    inertia = DFInertia(1,1,1)

    frame = DFFrame(frame_type=Frames.Hexa_X)

    with DroneForce(mass=mass, inertia=inertia, frame=frame) as drone:
        logging.debug("Ready: %s", drone)

        ea_matrix = drone.frame.EA
        ca_matrix = drone.frame.CA
        logging.info(f'Frame type: \n{len(drone.frame.frame_type.value)}')

        logging.info(f'Effectiveness: \n{ea_matrix}')
        logging.info(f'Control Allocation: \n{ca_matrix}')

        # EL method
        # k is constant for now
        kT = 1

        Tp_des = 0
        Tq_des = 0
        Tr_des = 0
        T_des = 7

        Torq = [Tp_des, Tq_des, Tr_des, T_des]
        
        u_input = np.matmul(drone.frame.CA_inv, Torq)
        print(f"u1, u2, u3, u4, u5, u6: {u_input}")

        # Convert motor torque (input u) to PWM
        PWM_out = []
        i = 0
        for input in u_input:
            PWM = torque_to_PWM(input, (frame.frame_type.value[i]))
            i = i + 1
            PWM_out.append(PWM)

        print(f"PWM outputs: {PWM_out}")

        fault_motor_num = 1
        drone.frame.inject_fault(fault_motor_num)
        logging.info(f'Control Allocation: \n{drone.frame.CA}')
        logging.info(f'Control Allocation Inverse: \n{drone.frame.CA_inv}')

        # Add flags for fault
        # Dont consider PWM input in case fault exists

        u_input = np.matmul(drone.frame.CA_inv, Torq)
        print(f"u1, u2, u3, u4, u5, u6: {u_input}")
        
        # Convert motor torque (input u) to PWM
        PWM_out = []
        i = 0
        for input in u_input:
            PWM = torque_to_PWM(input, (frame.frame_type.value[i]))
            i = i + 1
            PWM_out.append(PWM)

        print(f"PWM outputs with Fault: {PWM_out}")