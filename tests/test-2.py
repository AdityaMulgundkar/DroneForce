"""
TODO: Description of what this file does.

How to run:
1. Have 2 terminals open
    a. ~/ardupilot_ws/src/ardupilot# ./Tools/autotest/sim_vehicle.py -v ArduCopter --vehicle=ArduCopter --frame=hexa
    b. mavproxy.py --master 127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553
2. Open QGC/Ground control - it will auto connect to 127.0.0.1:14550 or 127.0.0.1:14551
3. Run this file
"""

import imp
import os
import sys
cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

from src.utility.logger import *
from src.control.motors import *
from src.control.frame import *

from dronekit import connect, mavutil, VehicleMode, LocationGlobalRelative

import time

import numpy as np
import numpy.matlib
import scipy.io
import json

from mixerlib import *

logging.debug('Beginning of code...')

import threading

class DroneForce:
    """ A DroneForce connection manager. """
    def __init__(self, connection_string, *args, **kwargs):
        logging.debug('connecting to Drone (or SITL/HITL) on: %s', connection_string)
        self.alive = True
        self.master = connect(connection_string, wait_ready=True)
        self.mavutil = mavutil.mavlink_connection('127.0.0.1:14552')

        # Add a heartbeat listener
        # Func for heartbeat
        def heartbeat_listener(_self, name, msg):
            self.last_heartbeat = msg

        self.heart = heartbeat_listener

        m1 = DFMotor(0,0,0,0)
        m2 = DFMotor(0,0,0,0)
        m3 = DFMotor(0,0,0,0)
        m4 = DFMotor(0,0,0,0)

        frame = DFFrame([m1,m2,m3,m4])

        logging.info('Drone connection successful')

    def __enter__(self):
        ''' Send regular heartbeats while using in a context manager. '''
        self.alive = True
        logging.info('__enter__ -> reviving heart (if required)')
        # Listen to the heartbeat
        self.master.add_message_listener('HEARTBEAT', self.heart)
       
        return self

    def __exit__(self, *args):
        ''' Automatically disarm and stop heartbeats on error/context-close. '''
        self.alive = False
        logging.info('__exit__ -> disarming, stopping heart and closing connection')
        # TODO: add reset parameters procedure. Can be based on flag?
        # Disarm if not disarmed
        if self.master.armed:
            self.master.armed = False
        # Kill heartbeat
        self.master.remove_message_listener('HEARTBEAT', self.heart)
        # Close Drone connection
        logging.info('disconnect -> closing Drone connection') 
        self.master.close()

if __name__ == '__main__':
    # Set up option parsing to get connection string
    import argparse  
    parser = argparse.ArgumentParser(description='Description of what this file does.')
    parser.add_argument('--connect', 
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect

    if not connection_string:
        connection_string = '127.0.0.1:14553'

    if not connection_string:
        logging.critical("No connection string specified, exiting code.")
        exit()

    with DroneForce(connection_string) as drone:
        logging.debug("Ready: %s", drone)

        def set_servo(motor_num, pwm_value):
            pwm_value_int = int(pwm_value)
            msg = drone.master.message_factory.command_long_encode(
                0, 0, 
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                motor_num,
                pwm_value_int,
                0,0,0,0,0
                )
            drone.master.send_mavlink(msg)

        def set_motor_mode(motor_num, set_reset):
            # get servo function - what this motor does
            logging.debug("PRE SET PARAM %s: %s", f'SERVO{motor_num}_FUNCTION' ,drone.master.parameters[f'SERVO{motor_num}_FUNCTION'])
            time.sleep(0.1)

            # set servo function - change to 1 for RCPassThru
            drone.master.parameters[f'SERVO{motor_num}_FUNCTION'] = set_reset
            time.sleep(0.1)

            # get servo function - what this motor does
            logging.debug("POST SET PARAM %s: %s", f'SERVO{motor_num}_FUNCTION' ,drone.master.parameters[f'SERVO{motor_num}_FUNCTION'])
            time.sleep(0.1)

        def set_motor_dir(motor_num, set_reset):
            # get servo function - what this motor does
            logging.debug("PRE SET PARAM %s: %s", f'SERVO{motor_num}_REVERSED' ,drone.master.parameters[f'SERVO{motor_num}_REVERSED'])
            time.sleep(0.1)

            # set servo function - change to 1 for Reverse direction
            drone.master.parameters[f'SERVO{motor_num}_REVERSED'] = set_reset
            time.sleep(0.1)

            # get servo function - what this motor does
            logging.debug("POST SET PARAM %s: %s", f'SERVO{motor_num}_REVERSED' ,drone.master.parameters[f'SERVO{motor_num}_REVERSED'])
            time.sleep(0.1)

        # Reset all motor configs
        set_motor_mode(1, 33)
        set_motor_mode(2, 34)
        set_motor_mode(3, 35)
        set_motor_mode(4, 36)
        set_motor_mode(5, 37)
        set_motor_mode(6, 38)

        # Reset all motor directions
        set_motor_dir(1, 0)
        set_motor_dir(2, 0)
        set_motor_dir(3, 0)
        set_motor_dir(4, 0)
        set_motor_dir(5, 0)
        set_motor_dir(6, 0)

        logging.debug("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not drone.master.is_armable:
            logging.debug(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        drone.master.mode = VehicleMode("GUIDED")
        drone.master.armed = True

        # Confirm vehicle armed before attempting to take off
        while not drone.master.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")

        logging.debug("Last Heartbeat: %s", drone.last_heartbeat)

        # TODO: Set motor modes to 1, for the motors you need to introduce fault into
        motor_num = 1
        des_pwm = 1400
        set_motor_mode(motor_num, 1)

        # TODO: Manually pass a PWM value to the selected motor. For simulating a fault, we pass 1000, which means the motor does not run at all.
        set_servo(motor_num, des_pwm)

        logging.debug("Goto Again")
        time.sleep(10)