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


cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

from src.droneforce import DroneForce
from src.autopilot import DFAutopilot
from src.utility.logger import *
from src.dynamics.inertia import DFInertia
from src.dynamics.mass import DFMass
from src.dynamics.frame import DFFrame, Frames
from src.dynamics.motors import DFMotor
from src.controller.p_controller import P_Controller

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
            commander.set_motor_mode(1, 1)
            commander.set_motor_mode(2, 1)
            commander.set_motor_mode(3, 1)
            commander.set_motor_mode(4, 1)
            commander.set_motor_mode(5, 1)
            commander.set_motor_mode(6, 1)

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

            with P_Controller(Kp=0.085) as controller:
                # get rpy from dronekit

                # Wait until the vehicle reaches a safe height before processing the goto
                #  (otherwise the command after Vehicle.simple_takeoff will execute
                #   immediately).
                z = commander.master.location.global_relative_frame.alt
                des_z = 10

                Tp_des = 0
                Tq_des = 0
                Tr_des = 0
                T_des = 0
                while z!=des_z:
                    z = commander.master.location.global_relative_frame.alt
                    des_z = 10
                    T_des = T_des + controller.Kp*(des_z - z)
                    if T_des>2.25:
                        T_des = 2.25
                    if T_des<1.85:
                        T_des = 1.85

                    Torq = [Tp_des, Tq_des, Tr_des, T_des]
                    
                    u_input = np.matmul(drone.frame.CA_inv, Torq)
                    print(f"z: {z}")
                    print(f"des_z: {des_z}")
                    print(f"T_des: {T_des}")
                    print(f"Torq: {Torq}")

                    # Convert motor torque (input u) to PWM
                    PWM_out_values = []
                    i = 0
                    for input in u_input:
                        PWM = torque_to_PWM(input, (frame.frame_type.value[i]))
                        i = i + 1
                        PWM_out_values.append(PWM)

                    print(f"PWM outputs: {PWM_out_values}\n")
                    i=1
                    for PWM in PWM_out_values:
                        commander.set_servo(i, PWM)
                        i = i+1

                    time.sleep(1)
                    
                # sleep so we can see the change in map
                time.sleep(100)

                logging.debug("Last Heartbeat: %s", commander.last_heartbeat)