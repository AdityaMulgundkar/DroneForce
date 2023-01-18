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
import math

def pi_clip(angle):
    if angle > 0:
        if angle > math.pi:
            return angle - 2*math.pi
    else:
        if angle < -math.pi:
            return angle + 2*math.pi
    return angle

logging.debug('Beginning of code...')

timestep_fast = 0.1
timestep_slow = 0.5

if __name__ == '__main__':
    mass = DFMass(1000)
    inertia = DFInertia(1,1,1)

    frame = DFFrame(frame_type=Frames.Quad_X)

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
            # commander.set_motor_mode(5, 1)
            # commander.set_motor_mode(6, 1)

            logging.debug("Basic pre-arm checks")
            # Don't try to arm until autopilot is ready

            print("Arming motors")
            # Copter should arm in GUIDED mode
            commander.master.mode = VehicleMode("AUTO")
            commander.master.armed = True

            # Add P Controller takeoff
            print("Taking off!")

            with P_Controller(Kp=0.085) as controller:
                # get rpy from dronekit

                z = commander.master.location.global_relative_frame.alt
                yaw = commander.master.attitude.yaw
                des_pitch = 0
                des_roll = 0
                des_yaw = 0
                des_z = 10

                Tp_des = 0
                Tq_des = 0
                Tr_des = 0
                T_des = 0
                Torq = [Tp_des, Tq_des, Tr_des, T_des]

                # pid_p is roll
                # pid_q is pitch
                # pid_r is yaw
                # pid_T is Thrust
                pid_p = PID(0.05, 0.0, 0.025)
                pid_q = PID(0.035, 0.0, 0.025)
                pid_r = PID(0.15, 0.01, 0.1)
                # Manually tuned behaviour
                pid_T = PID(0.5, 0.05, 0.5)
                

                pid_p.sample_time = timestep_fast
                pid_q.sample_time = timestep_fast
                pid_r.sample_time = timestep_slow
                pid_T.sample_time = timestep_slow

                pid_p.output_limits = (-0.5, 0.5) 
                pid_q.output_limits = (-0.5, 0.5) 
                pid_r.output_limits = (-0.5, 0.5) 
                # pid_T.output_limits = (1.5, 10) 
                pid_T.output_limits = (0, 7.5) 

                pid_p.error_map = pi_clip
                pid_q.error_map = pi_clip
                pid_r.error_map = pi_clip
                
                start_time = time.time()

                while True:
                    # if(time.time() - start_time > 1):
                    #     # Roll a little?
                    #     Tq_des = 1.5
                    #     start_time = time.time()
                    # else:
                    #     Tq_des = 0

                    roll = commander.master.attitude.roll
                    pitch = commander.master.attitude.pitch
                    yaw = commander.master.attitude.yaw
                    z = commander.master.location.global_relative_frame.alt

                    pid_p.setpoint = des_roll
                    pid_q.setpoint = des_pitch
                    pid_r.setpoint = des_yaw
                    pid_T.setpoint = des_z

                    # Compute new output from the PID according to the systems current value
                    T_p = pid_p(roll)
                    T_q = pid_q(pitch)
                    T_r = pid_r(yaw)
                    T_des = pid_T(z)

                    if (des_roll == 0 and des_pitch == 0 and des_yaw == 0):
                        # Posn only
                        # Torq = [0, 0, T_r, T_des]
                        Torq = [T_p, T_q, T_r, T_des]
                    else:
                        # Posn + att
                        Torq = [T_p, T_q, T_r, T_des]


                    # Proper
                    # Torq = [T_p, T_q, T_r, T_des]

                    # Torq = [Tp_des, Tq_des, T_r, T_des]
                    # print(f"\n Pre Torq CA_inv: \n{drone.frame.CA_inv}")
                    u_input = np.matmul(drone.frame.CA_inv, Torq)

                    # Convert motor torque (input u) to PWM
                    PWM_out_values = []
                    i = 0
                    for input in u_input:
                        PWM = torque_to_PWM(input, (frame.frame_type.value[i]))
                        i = i + 1
                        PWM_out_values.append(PWM)

                    print(f"Torq: {Torq}")
                    print(f"u inputs: {u_input}")
                    print(f"Sensor inputs: {roll, pitch, yaw, z}")
                    print(f"PWM outputs: {PWM_out_values}\n")

                    i=1
                    for PWM in PWM_out_values:
                        commander.set_servo(i, PWM)
                        i = i+1

                    time.sleep(timestep_fast)

            time.sleep(60)