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
import time
from dronekit import VehicleMode
from pymavlink import mavutil
from controller.p_controller import P_Controller


cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

from src.droneforce import DroneForce
from src.autopilot import DFAutopilot
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

        connection_string = '127.0.0.1:14553'

        with DFAutopilot(connection_string=connection_string) as commander:
            logging.debug("Ready: %s", commander)

            # Reset all motor configs
            commander.set_motor_mode(1, 33)
            commander.set_motor_mode(2, 34)
            commander.set_motor_mode(3, 35)
            commander.set_motor_mode(4, 36)
            commander.set_motor_mode(5, 37)
            commander.set_motor_mode(6, 38)

            logging.debug("Basic pre-arm checks")
            # Don't try to arm until autopilot is ready
            while not commander.master.is_armable:
                logging.debug(" Waiting for vehicle to initialise...")
                time.sleep(1)

            print("Arming motors")
            # Copter should arm in GUIDED mode
            commander.master.mode = VehicleMode("GUIDED")
            commander.master.armed = True

            # Confirm vehicle armed before attempting to take off
            while not commander.master.armed:
                print(" Waiting for arming...")
                time.sleep(1)

            # Add P Controller takeoff
            print("Taking off!")

            with P_Controller(commander.master, 0.5) as controller:
                x=0
                # get rpy from dronekit
                z = commander.location.global_relative_frame.alt
                des_z = 10

            # Wait until the vehicle reaches a safe height before processing the goto
            #  (otherwise the command after Vehicle.simple_takeoff will execute
            #   immediately).
            while z!=des_z:
                print(" Altitude: ", commander.master.location.global_relative_frame.alt)
                Tp_des = 0
                Tq_des = 0
                Tr_des = 0
                T_des = controller.kp*(des_z - z)

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
                time.sleep(1)
                
            # sleep so we can see the change in map
            time.sleep(5)

            logging.debug("Last Heartbeat: %s", commander.last_heartbeat)

            time.sleep(10)
        