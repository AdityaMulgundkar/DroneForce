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
from control.autopilot import Autopilot
from dronekit import VehicleMode
from pymavlink import mavutil


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

        connection_string = '127.0.0.1:14553'

        with Autopilot(connection_string) as commander:
            logging.debug("Ready: %s", commander)

            def set_servo(motor_num, pwm_value):
                pwm_value_int = int(pwm_value)
                msg = commander.master.message_factory.command_long_encode(
                    0, 0, 
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    motor_num,
                    pwm_value_int,
                    0,0,0,0,0
                    )
                commander.master.send_mavlink(msg)

            def set_motor_mode(motor_num, set_reset):
                # get servo function - what this motor does
                logging.debug("PRE SET PARAM %s: %s", f'SERVO{motor_num}_FUNCTION' ,commander.master.parameters[f'SERVO{motor_num}_FUNCTION'])
                time.sleep(0.1)

                # set servo function - change to 1 for RCPassThru
                commander.master.parameters[f'SERVO{motor_num}_FUNCTION'] = set_reset
                time.sleep(0.1)

                # get servo function - what this motor does
                logging.debug("POST SET PARAM %s: %s", f'SERVO{motor_num}_FUNCTION' ,commander.master.parameters[f'SERVO{motor_num}_FUNCTION'])
                time.sleep(0.1)

            # Reset all motor configs
            set_motor_mode(1, 33)
            set_motor_mode(2, 34)
            set_motor_mode(3, 35)
            set_motor_mode(4, 36)
            set_motor_mode(5, 37)
            set_motor_mode(6, 38)

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

            print("Taking off!")
            commander.master.simple_takeoff(20)  # Take off to target altitude

            # Wait until the vehicle reaches a safe height before processing the goto
            #  (otherwise the command after Vehicle.simple_takeoff will execute
            #   immediately).
            while True:
                print(" Altitude: ", commander.master.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if commander.master.location.global_relative_frame.alt >= 20 * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)
                
            # sleep so we can see the change in map
            time.sleep(5)

            logging.debug("Last Heartbeat: %s", commander.last_heartbeat)

            # TODO: Set motor modes to 1, for the motors you need to introduce fault into
            # set_motor_mode(1, 1)
            set_motor_mode(1, 1)
            set_motor_mode(2, 1)

            # TODO: Manually pass a PWM value to the selected motor. For simulating a fault, we pass 1000, which means the motor does not run at all.
            set_servo(1, 1000)
            set_servo(2, 1000)

            print("Taking off!")
            commander.master.simple_takeoff(10)  # Take off to target altitude
            
            while True:
                print(" Altitude: ", commander.master.location.global_relative_frame.alt)
                # Break and return from function just below target altitude.
                if commander.master.location.global_relative_frame.alt >= 10 * 0.95:
                    print("Reached target altitude")
                    break
                time.sleep(1)

            time.sleep(10)
        