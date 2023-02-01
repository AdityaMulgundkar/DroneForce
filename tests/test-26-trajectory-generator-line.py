#!/usr/bin/env python
"""
1. mavproxy.py --master 127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --out=udp:127.0.0.1:14554
2. cd ~/df_ws/src/ardupilot/Tools/autotest
sim_vehicle.py -v ArduCopter -f gazebo-iris  -m --mav10
3. roslaunch mavros apm.launch fcu_url:=udp://:14553@
4. cd ~/df_ws/src/ardupilot_gazebo/worlds 
gazebo --verbose iris_ardupilot.world
5. cd ~/df_ws/src/DroneForce/tests
python3 test-21-asmc-circle.py 
"""

import sys
import time
# ROS python API
import rospy

import tf.transformations as transformations
# from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import Float64, Bool
# from std_msgs.msg import Float64MultiArray

# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, TwistStamped, Transform, Twist
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt, MultiDOFJointTrajectoryPoint as MdjtP
# from msg_check.msg import PlotDataMsg


import numpy as np
from tf.transformations import *
#import RPi.GPIO as GPIO


import os
import sys
cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

curr_sp = PoseStamped()
next_sp = PoseStamped()


## local position callback
def posCb(msg):
    curr_sp.pose.position.x = msg.pose.position.x
    curr_sp.pose.position.y = msg.pose.position.y
    curr_sp.pose.position.z = msg.pose.position.z

# Main function
def main(argv):

    dest_sp = PoseStamped()


    dest_sp.pose.position.x = 0
    dest_sp.pose.position.y = 0
    dest_sp.pose.position.z = 3

    rospy.init_node('mdjt_node', anonymous=True)
    rate = rospy.Rate(15)

    # rospy.Subscriber('command/trajectory', Mdjt, multiDoFCb)
    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, posCb)

    sp_pub = rospy.Publisher('command/trajectory', Mdjt, queue_size=10)
    des_pub = rospy.Publisher('desired_position', PoseStamped, queue_size=10)
    fault_pub = rospy.Publisher('fault', Bool, queue_size=10)

    trajectory_timer = 0.25
    angle = 0
    angle_delta = 0.015
    start_time = time.time()
    last_time = time.time()

    pre_time1 = 0

    radius = 3
    is_faulty = False

    while not rospy.is_shutdown():
        is_faulty = False
        if (time.time() - last_time  > trajectory_timer):
            dt = rospy.get_time() - pre_time1
            pre_time1 = pre_time1 + dt
            if dt > 0.04:
                dt = 0.04

            angle = angle + angle_delta

            if(angle > 3.14/4):
                is_faulty = True
            else:
                is_faulty = False

            # print(f"is_faulty: {is_faulty}")
            curr_x = curr_sp.pose.position.x
            curr_y = curr_sp.pose.position.y
            curr_z = curr_sp.pose.position.z

            if(angle>10):
                angle = 10
            x = 0
            y = angle

            next_mdjt = Mdjt()
            next_mdjtP = MdjtP()

            next_sp.pose.position.x = x
            next_sp.pose.position.y = y
            next_sp.pose.position.z = dest_sp.pose.position.z

            tr = Transform()
            tr.translation.x = x
            tr.translation.y = y
            tr.translation.z = dest_sp.pose.position.z


            next_mdjtP.transforms = [tr]
            # next_mdjtP.transforms[0].translation.x = x
            # next_mdjtP.transforms[0].translation.y = y
            # next_mdjtP.transforms[0].translation.z = dest_sp.pose.position.z
            
            x1 = curr_x
            x2 = x
            y1 = curr_y
            y2 = y
            z1 = curr_z
            z2 = dest_sp.pose.position.z

            print(f"x1: {x1}, x2: {x2}")
            print(f"y1: {y1}, y2: {y2}")

            # vx = dt/abs(x2 - x1)
            # vy = dt/abs(y2 - y1)
            # vz = dt/abs(z2 - z1)
            
            vx = (x2 - x1)/dt
            vy = (y2 - y1)/dt
            vz = (z2 - z1)/dt

            clip = 0.05
            if(vx>clip):
                vx = clip
            elif(vx<-clip):
                vx = -clip
            if(vy>clip):
                vy = clip
            elif(vy<-clip):
                vy = -clip
            if(vz>clip):
                vz = clip
            elif(vz<-clip):
                vz = -clip

            tvel = Twist()
            tvel.linear.x = vx
            tvel.linear.y = vy
            tvel.linear.z = vz

            next_mdjtP.velocities = [tvel]

            # next_mdjtP.velocities[0].linear.x = vx
            # next_mdjtP.velocities[0].linear.y = vy
            # next_mdjtP.velocities[0].linear.z = vz

            # next_mdjt.points.append(next_mdjtP)

            next_mdjt.points.insert(1, next_mdjtP)

            # print(f"Publishing dt: {dt}, val: {next_mdjtP.velocities}")

            # multiDoFCb(next_mdjt)
            sp_pub.publish(next_mdjt)
            des_pub.publish(next_sp)
            fault_pub.publish(is_faulty)
            last_time = time.time()
            
        rate.sleep()
        # rospy.spin()

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass