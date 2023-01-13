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

import math
import os
import sys
import time
from dronekit import VehicleMode
from pymavlink import mavutil

from simple_pid import PID

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
import matplotlib.pyplot as plt

logging.debug('Beginning of code...')

timestep_fast = 0.1
timestep_slow = 0.01
plt.show()

if __name__ == '__main__':
    mass = DFMass(1000)
    inertia = DFInertia(1,1,1)

    frame = DFFrame(frame_type=Frames.Quad_X)

    start_time = time.time() 
    logging.getLogger('matplotlib.font_manager').disabled = True


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

                z = commander.master.location.global_relative_frame.alt
                des_pitch = 0
                des_roll = 0
                des_yaw = 0
                des_z = 1

                Tp_des = 0
                Tq_des = 0
                Tr_des = 0
                T_des = 0
                Torq = [Tp_des, Tq_des, Tr_des, T_des]

                pid_p = PID(0.175, 0.00475, 0.001)
                pid_r = PID(0.175, 0.00475, 0.001)
                pid_y = PID(0.175, 0.00475, 0.001)

                # Munjaal manual tuning
                # 4.5 to 13
                # pid_T = PID(0.5, 0.0, 0.2)

                # 5 to 12.8
                # pid_T = PID(0.5, 0.0, 0.2)

                # 5 to 14
                # pid_T = PID(0.5, 0.2, 0.2)

                # 7.2 to 18
                # pid_T = PID(0.5, 0.2, 0.5)

                # 5.4 to 22
                # pid_T = PID(0.2, 0.2, 0.5)

                # 4.5 to 22.8
                # pid_T = PID(0.2, 0.2, 0.2)

                # 5.5 to 10.5
                # pid_T = PID(0.2, 0.02, 0.02)

                # 8.2 to 10.5
                # pid_T = PID(0.2, 0.02, 0.04)

                # 6.2 to 10.6
                # pid_T = PID(0.2, 0.02, 0.03)

                # 
                # pid_T = PID(0.25, 0.0, 0.025)

                # 3.6 to 10.5
                # pid_T = PID(0.4, 0.0, 0.1)

                # 8.2 to 10.5
                # pid_T = PID(0.2, 0.02, 0.3)

                # Baap behaviour
                # pid_T = PID(0.25, 0.02, 0.3)

                # Baap behaviour
                # pid_T = PID(0.25, 0.025, 0.3)

                # Baap behaviour
                pid_T = PID(0.25, 0.025, 0.3)

                # pid_T.proportional_on_measurement = True

                # Baap behaviour
                # temp_kp = -1/(1+(math.pow(2.71,-1.0*(des_z-z-3))))+1
                # pid_T = PID(0.5*temp_kp, 0.025, 0.3)
                
                pid_p.sample_time = timestep_slow
                pid_r.sample_time = timestep_slow
                pid_y.sample_time = timestep_slow
                pid_T.sample_time = timestep_fast

                pid_p.output_limits = (0, 1.0) 
                pid_r.output_limits = (0, 1.0) 
                pid_y.output_limits = (0, 1.0) 
                # pid_T.output_limits = (1.5, 2.5) 
                # pid_T.output_limits = (1.5, 10) 
                pid_T.output_limits = (1.5, 10) 
                # pid.output_limits = (0, 5) 
                
                while True:

                    pid_p.setpoint = des_roll
                    pid_r.setpoint = des_pitch
                    pid_y.setpoint = des_yaw
                    pid_T.setpoint = des_z

                    roll = commander.master.attitude.roll
                    pitch = commander.master.attitude.pitch
                    yaw = commander.master.attitude.yaw
                    z = commander.master.location.global_relative_frame.alt
                    Torq = [Tp_des, Tq_des, Tr_des, T_des]
                    u_input = np.matmul(drone.frame.CA_inv, Torq)

                    # Plot
                    y = z
                    x = time.time() - start_time

                    plt.scatter(x, y)
                    plt.draw()
                    plt.pause(timestep_fast)

                    print(f"z: {z}")
                    print(f"des_z: {des_z}")
                    print(f"T_des: {T_des}")
                    # print(f"Torq: {Torq}")
                    print(f"Velocity: {commander.master.velocity}")

                    # Compute new output from the PID according to the systems current value
                    T_p = pid_p(roll)
                    T_r = pid_r(pitch)
                    T_y = pid_y(yaw)
                    T_des = pid_T(z)
                    # print(f"PID: {T_p, T_r, T_des}")

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

                    time.sleep(timestep_fast)

            time.sleep(60)