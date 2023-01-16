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
from src.controller.asmc_controller import ASMC_Controller

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

            with ASMC_Controller(Kp=0.085) as controller:
                # get rpy from dronekit
                while True:
                    x = 0
                    # State callback
                    # Skipped

                    # Odometry callback
                    # controller.odomCb

                    # Create local Pose
                    posCb = controller.posCb

                    # Create new pose
                    newPoseCB = controller.newPoseCB

                    # MultiDoF callback
                    multiDoFCb = controller.multiDoFCb

                    # setpointposition/local publisher

            time.sleep(60)



# Main function
def main(argv):
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

    k=0
    while k<20:
        sp_pub.publish(cnt.sp)
        rate.sleep()
        k = k + 1

    # ROS main loop
    while not rospy.is_shutdown():
        cnt.pub_att()