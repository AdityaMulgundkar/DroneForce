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

from dronekit import connect, mavutil, VehicleMode, LocationGlobalRelative

import time

import numpy as np
import numpy.matlib
import scipy.io
import json

logging.debug('Beginning of code...')

import threading

class Autopilot:
    """ A Dronekit autopilot connection manager. """
    def __init__(self, connection_string, *args, **kwargs):
        logging.debug('connecting to Drone (or SITL/HITL) on: %s', connection_string)
        self.alive = True
        self.master = connect(connection_string, wait_ready=True)
        self.mavutil = mavutil.mavlink_connection('127.0.0.1:14552')

        self.schedules = []
        self.threads = []

        self.pitch_array = []
        self.roll_array = []
        self.rpy_timestamps = []

        self.pitchspeed_array = []
        self.rollspeed_array = []
        self.yawspeed_array = []
        self.w_timestamps = []

        # Add a heartbeat listener
        # Func for heartbeat
        def heartbeat_listener(_self, name, msg):
            self.last_heartbeat = msg

        self.heart = heartbeat_listener

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
        # Kill scheduler
        for i in range(0, len(self.schedules)-1):
            t = self.threads[i]
            t.stop()
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


    with Autopilot(connection_string) as drone:
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

        def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
            msg = drone.master.message_factory.set_position_target_local_ned_encode(
                0,       # time_boot_ms (not used)
                0, 0,    # target system, target component
                mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                0b0000111111000111, # type_mask (only speeds enabled)
                0, 0, 0, # x, y, z positions (not used)
                velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


            # send command to vehicle on 1 Hz cycle
            for x in range(0,duration):
                drone.master.send_mavlink(msg)
                time.sleep(0.1)

        # ArduCopter set to hexa (hexa plus) frame
        #    1
        # 5     4
        # 3     6
        #    2
        # Disabling 1,2 sets to quad-x with 3,4,5,6 motors in above config

        def change_yaw(yaw_value):
            yaw_pwm = np.abs(np.interp(yaw_value, (-1, 1), (-1800, 1800)))
            set_motor_mode(3, 1)
            set_motor_mode(4, 1)
            set_motor_mode(5, 1)
            set_motor_mode(6, 1)
            
            set_servo(3, 1800)
            set_servo(4, 1800)
            # reverse other two
            # set_motor_dir(5, 1)
            # set_motor_dir(6, 1)
            set_servo(5, 1800)
            set_servo(6, 1800)
            
            time.sleep(10)
            set_motor_mode(3, 35)
            set_motor_mode(4, 36)
            set_motor_mode(5, 37)
            set_motor_mode(6, 38)
            # set_motor_dir(3, 0)
            # set_motor_dir(4, 0)
            # set_motor_dir(5, 0)
            # set_motor_dir(6, 0)

        # RPY Thread
        def rpy_thread():
            if(drone.alive)==False:
                pass
            while drone.alive:
                rpy_logger()
                # TODO: Mayank - test this delay on Pi
                time.sleep(0.01)

        # W Thread
        def w_thread():
            if(drone.alive)==False:
                pass
            while drone.alive:
                w_logger()
                # TODO: Mayank - test this delay on Pi
                time.sleep(0.01)

        # Func for rpy logging
        def rpy_logger():
            # Procedure to track and store pitch-roll-yaw values
            nav_msg = drone.mavutil.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=False)
            # TODO: Mayank - test this delay on Pi
            time.sleep(0.01)
            if(nav_msg) is None:
                pass
            elif nav_msg.get_type()!='BAD_DATA':
                # 'nav_roll', 'nav_pitch', 'alt_error', 'aspd_error', 'xtrack_error', 'nav_bearing', 'target_bearing', 'wp_dist'
                nav_data = (nav_msg.nav_roll, nav_msg.nav_pitch, nav_msg.alt_error, nav_msg.aspd_error, nav_msg.xtrack_error, nav_msg.nav_bearing, nav_msg.target_bearing, nav_msg.wp_dist) 
                timestamp = getattr(nav_msg, '_timestamp', None)
                timestamp = int(timestamp*1.0e6)
                drone.pitch_array.append(nav_data[1])
                drone.roll_array.append(nav_data[0])
                drone.rpy_timestamps.append(timestamp)
                # TODO: Comment later on, or add exception to logger.py
                logging.debug("RPY Logged")

        # Func for rpy logging
        def w_logger():
            # Procedure to track and store pitch-roll-yaw values
            nav_msg = drone.mavutil.recv_match(type='ATTITUDE_QUATERNION_COV', blocking=False)
            # TODO: Mayank - test this delay on Pi
            time.sleep(0.01)
            if(nav_msg) is None:
                pass
            elif nav_msg.get_type()!='BAD_DATA':
                # TODO: (Mayank) - print nav_msg and see what vars it has
                # TODO: change nav_data to have exact same vars from print above
                nav_data = (nav_msg.time_usec, nav_msg.q, nav_msg.rollspeed, nav_msg.pitchspeed, nav_msg.yawspeed, nav_msg.covariance) 
                timestamp = getattr(nav_msg, '_timestamp', None)
                timestamp = int(timestamp*1.0e6)
                drone.pitchspeed_array.append(nav_data[3])
                drone.rollspeed_array.append(nav_data[2])
                drone.yawspeed_array.append(nav_data[4])
                drone.w_timestamps.append(timestamp)
                # TODO: Comment later on, or add exception to logger.py
                logging.debug("W Logged")

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

        # TODO: Uncomment the next two lines in case you are testing a HexaCopter. Keep them commented for a QuadCopter.
        # set_motor_mode(5, 37)
        # set_motor_mode(6, 38)

        logging.debug("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not drone.master.is_armable:
            logging.debug(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors")
        # Copter should arm in GUIDED mode
        drone.master.mode = VehicleMode("GUIDED")
        drone.master.armed = True
        
        # Start RPY Logger
        threading.Thread(target= rpy_thread).start()
        # Start W Logger
        threading.Thread(target= w_thread).start()

        # Confirm vehicle armed before attempting to take off
        while not drone.master.armed:
            print(" Waiting for arming...")
            time.sleep(1)

        print("Taking off!")
        drone.master.simple_takeoff(20)  # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto
        #  (otherwise the command after Vehicle.simple_takeoff will execute
        #   immediately).
        while True:
            print(" Altitude: ", drone.master.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if drone.master.location.global_relative_frame.alt >= 20 * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
            
        # sleep so we can see the change in map
        time.sleep(5)

        logging.debug("Last Heartbeat: %s", drone.last_heartbeat)

        # TODO: Set motor modes to 1, for the motors you need to introduce fault into
        # set_motor_mode(1, 1)
        set_motor_mode(1, 1)
        set_motor_mode(2, 1)

        # TODO: Manually pass a PWM value to the selected motor. For simulating a fault, we pass 1000, which means the motor does not run at all.
        set_servo(1, 1000)
        set_servo(2, 1000)

        print("Taking off!")
        drone.master.simple_takeoff(10)  # Take off to target altitude
        
        while True:
            print(" Altitude: ", drone.master.location.global_relative_frame.alt)
            # Break and return from function just below target altitude.
            if drone.master.location.global_relative_frame.alt >= 10 * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

        time.sleep(10)
        