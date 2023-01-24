#!/usr/bin/env python
"""
ASMC:
mavproxy.py --master 127.0.0.1:14551 --out=udp:127.0.0.1:14552 --out=udp:127.0.0.1:14553 --out=udp:127.0.0.1:14554
python3 sim_vehicle.py -v ArduCopter --vehicle=ArduCopter --frame=X
roslaunch mavros apm.launch fcu_url:=udp://:14553
"""
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
from geometry_msgs.msg import Point, PoseStamped, TwistStamped
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import *
from trajectory_msgs.msg import MultiDOFJointTrajectory as Mdjt
# from msg_check.msg import PlotDataMsg


import numpy as np
from tf.transformations import *
#import RPi.GPIO as GPIO


import os
import sys
cur_path=os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, cur_path+"/..")

from src.autopilot import DFAutopilot
               
class Controller:
    # initialization method
    def __init__(self):
        # Drone state
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PoseStamped()

        # initial values for setpoints
        self.cur_pose = PoseStamped()
        self.cur_vel = TwistStamped()
        self.sp.pose.position.x = 0.0
        self.sp.pose.position.y = 0.0
        self.ALT_SP = 10.0
        self.sp.pose.position.z = self.ALT_SP
        self.local_pos = Point(0.0, 0.0, self.ALT_SP)
        self.local_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.desVel = np.zeros(3)
        self.errInt = np.zeros(3)

        # self.df_cmd = Float64(0)
        self.torq_cmd = np.array([0, 0, 0])
        self.th_cmd = np.array([0, 0, 0])

        # Control parameters for the outer loop
        self.Kp0 =  np.array([0.1, 0.1, 0.1])
        self.Kp1 =  np.array([0.1, 0.1, 0.1])

        self.alpha_0 = np.array([1,1,1])
        self.alpha_1 = np.array([3,3,3])

        # Tuning for outer
        self.Lam = np.array([0.5, 0.4, 20.0])
        self.Phi = np.array([1.2, 1.5, 2.5])   #1.0 - 1.5
        
        self.M = 0.5
        self.alpha_m = 0.01  # 0.01 - 0.05
        # Close Tuning for outer

        self.norm_thrust_const = 0.06
        self.max_th = 18.0
        self.max_throttle = 0.95

        # Control parameters for the inner loop
        self.Kp0_q = np.array([0.1, 0.1, 0.1])
        self.Kp1_q = np.array([0.1, 0.1, 0.1])

        self.alpha_0_q = np.array([1,1,1])
        self.alpha_1_q = np.array([3,3,3])

        # Tuning for inner
        self.Lam_q = np.array([2.0, 4.0, 2.0])
        self.Phi_q = np.array([1.1, 1.5, 1.1])   #1.0 - 1.5
        # Close Tuning for inner

        self.norm_moment_const = 0.05
        self.max_mom = 10
        # self.max_mom_throttle = 0.33
        self.max_mom_throttle = 0.5

        self.v = 0.1
        
        self.gravity = np.array([0, 0, 9.8])
        self.pre_time1 = rospy.get_time()   
        self.pre_time2 =  rospy.get_time() 

        self.armed = False

        # Control allocation matrix
        self.EA = [
                [-1,1,1,1],
                [1,-1,1,1],
                [1,1,-1,1],
                [-1,-1,-1,1],
            ]

    def multiDoFCb(self, msg):
        pt = msg.points[0]
        self.sp.pose.position.x = pt.transforms[0].translation.x
        self.sp.pose.position.y = pt.transforms[0].translation.y
        self.sp.pose.position.z = pt.transforms[0].translation.z
        self.desVel = np.array([pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z])
        # self.desVel = np.array([pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z])

    ## local position callback
    def posCb(self, msg):
        self.local_pos.x = msg.pose.position.x
        self.local_pos.y = msg.pose.position.y
        self.local_pos.z = msg.pose.position.z
        self.local_quat[0] = msg.pose.orientation.x
        self.local_quat[1] = msg.pose.orientation.y
        self.local_quat[2] = msg.pose.orientation.z
        self.local_quat[3] = msg.pose.orientation.w

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def odomCb(self, msg):
        self.cur_pose.pose.position.x = msg.pose.pose.position.x
        self.cur_pose.pose.position.y = msg.pose.pose.position.y
        self.cur_pose.pose.position.z = msg.pose.pose.position.z

        self.cur_pose.pose.orientation.w = msg.pose.pose.orientation.w
        self.cur_pose.pose.orientation.x = msg.pose.pose.orientation.x
        self.cur_pose.pose.orientation.y = msg.pose.pose.orientation.y
        self.cur_pose.pose.orientation.z = msg.pose.pose.orientation.z

        self.cur_vel.twist.linear.x = msg.twist.twist.linear.x
        self.cur_vel.twist.linear.y = msg.twist.twist.linear.y
        self.cur_vel.twist.linear.z = msg.twist.twist.linear.z

        self.cur_vel.twist.angular.x = msg.twist.twist.angular.x
        self.cur_vel.twist.angular.y = msg.twist.twist.angular.y
        self.cur_vel.twist.angular.z = msg.twist.twist.angular.z

    def newPoseCB(self, msg):
        # if(self.sp.pose.position != msg.pose.position):
        #     x = msg.pose.position.x
        #     y = msg.pose.position.y
        #     z = msg.pose.position.z
        #     print(f"New pose received: {x, y, z}")
        self.sp.pose.position.x = msg.pose.position.x
        self.sp.pose.position.y = msg.pose.position.y
        self.sp.pose.position.z = msg.pose.position.z
   
        self.sp.pose.orientation.x = msg.pose.orientation.x
        self.sp.pose.orientation.y = msg.pose.orientation.y
        self.sp.pose.orientation.z = msg.pose.orientation.z
        self.sp.pose.orientation.w = msg.pose.orientation.w

    def vector2Arrays(self, vector):        
        return np.array([vector.x, vector.y, vector.z])

    def sigmoid(self, s, v):
        if np.absolute(s) > v:
            return s/np.absolute(s)
        else:
            return s/v

    def th_des(self):
        dt = rospy.get_time() - self.pre_time1
        self.pre_time1 = self.pre_time1 + dt
        if dt > 0.04:
            dt = 0.04

        curPos = self.vector2Arrays(self.cur_pose.pose.position)
        desPos = self.vector2Arrays(self.sp.pose.position)
        curVel = self.vector2Arrays(self.cur_vel.twist.linear)

        errPos = curPos - desPos
        errVel = curVel - self.desVel
        sv = errVel + np.multiply(self.Phi, errPos)
        # sv =  errPos


        if self.armed:
            self.Kp0 += (sv - np.multiply(self.alpha_0, self.Kp0))*dt
            self.Kp1 += (sv - np.multiply(self.alpha_1, self.Kp1))*dt
            self.Kp0 = np.maximum(self.Kp0, 0.0001*np.ones(3))
            self.Kp1 = np.maximum(self.Kp1, 0.0001*np.ones(3))
            self.M += (-sv[2] - self.alpha_m*self.M)*dt
            self.M = np.maximum(self.M, 0.1)
            # print(self.M)

        Rho = self.Kp0 + self.Kp1*errPos

        delTau = np.zeros(3)
        delTau[0] = Rho[0]*self.sigmoid(sv[0],self.v)
        delTau[1] = Rho[1]*self.sigmoid(sv[1],self.v)
        delTau[2] = Rho[2]*self.sigmoid(sv[2],self.v)

        # des_th = -np.multiply(self.Lam, sv) - delTau + self.M*self.gravity
        des_th = -np.multiply(self.Lam, sv) 


        # putting limit on maximum thrust vector
        if np.linalg.norm(des_th) > self.max_th:
            des_th = (self.max_th/np.linalg.norm(des_th))*des_th

        # print(f"Current Pose: {curPos}")
        # print(f"Des Pose: {desPos}")
        # print(f"Err Pose: {errPos}")
        # print(f"Thrust cmd: {self.df_cmd.data}")
    
        return des_th

    def acc2quat(self, des_th, des_yaw):
        proj_xb_des = np.array([np.cos(des_yaw), np.sin(des_yaw), 0.0])
        if np.linalg.norm(des_th) == 0.0:
            zb_des = np.array([0,0,1])
        else:    
            zb_des = des_th / np.linalg.norm(des_th)
        yb_des = np.cross(zb_des, proj_xb_des) / np.linalg.norm(np.cross(zb_des, proj_xb_des))
        xb_des = np.cross(yb_des, zb_des) / np.linalg.norm(np.cross(yb_des, zb_des))
       
        rotmat = np.transpose(np.array([xb_des, yb_des, zb_des]))
        return rotmat

    def geo_con_new(self):

        pose = transformations.quaternion_matrix(  
                numpy.array([self.cur_pose.pose.orientation.x, 
                             self.cur_pose.pose.orientation.y, 
                             self.cur_pose.pose.orientation.z, 
                             self.cur_pose.pose.orientation.w]))  #4*4 matrix
        pose_temp1 = np.delete(pose, -1, axis=1)
        rot_curr = np.delete(pose_temp1, -1, axis=0)   #3*3 current rotation matrix
        zb_curr = rot_curr[:,2]

        orientation_q = self.cur_pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll_curr, pitch_curr, yaw_curr) = euler_from_quaternion (orientation_list)

        #---------------------------------------------#
        des_th = self.th_des()    
        rot_des = self.acc2quat(des_th, 0)   #desired yaw = 0
        rot_44 = np.vstack((np.hstack((rot_des,np.array([[0,0,0]]).T)), np.array([[0,0,0,1]])))
        quat_des = quaternion_from_matrix(rot_44)

        orientation_list = [quat_des[0], quat_des[1], quat_des[2], quat_des[3]]
        (roll_des, pitch_des, yaw_des) = euler_from_quaternion (orientation_list)

        roll_err = roll_curr - roll_des
        pitch_err = -(pitch_curr - pitch_des)
        yaw_err = -(yaw_curr - yaw_des)

        # print(f"roll-pitch-yaw errs: {roll_err*1, pitch_err*1, yaw_err*1}")

        zb = rot_des[:,2]
        thrust = self.norm_thrust_const * des_th.dot(zb)
        thrust = np.maximum(0.0, np.minimum(thrust, self.max_throttle))
     
        angle_error_matrix = 0.5* (np.dot(np.transpose(rot_des), rot_curr) -
                                    np.dot(np.transpose(rot_curr), rot_des) ) #skew matrix

        roll_x_err = -angle_error_matrix[1,2]
        pitch_y_err = -angle_error_matrix[0,2]   
        yaw_z_err = angle_error_matrix[0,1]

        # Put minus if unable ot tune after multiple runs
        # self.euler_err = np.array([roll_x_err, pitch_y_err, yaw_z_err])

        # print(f"angle_errors: {roll_x_err, pitch_y_err, yaw_z_err}")

        self.euler_err = np.array([roll_x_err, pitch_y_err, yaw_z_err])

        self.des_q_dot = np.array([0 ,0, 0])
        des_euler_rate =  np.dot(np.multiply(np.transpose(rot_des), rot_curr), 
                                     self.des_q_dot)

        curr_euler_rate = np.array([self.cur_vel.twist.angular.x,
                                    -self.cur_vel.twist.angular.y,
                                    -self.cur_vel.twist.angular.z])

        self.euler_rate_err = curr_euler_rate - des_euler_rate

        self.th_cmd = thrust

    def moment_des(self):
        self.geo_con_new()

        dt = rospy.get_time() - self.pre_time2
        self.pre_time2 = self.pre_time2 + dt
        if dt > 0.04:
            dt = 0.04

        # print(f"Euler err: {self.euler_err*10000}")
        # self.euler_rate_err = np.array([0,0,0])
        sv_q = self.euler_rate_err + np.multiply(self.Phi_q, self.euler_err)

        if self.armed:
            self.Kp0_q += (sv_q - np.multiply(self.alpha_0_q, self.Kp0_q))*dt
            self.Kp1_q += (sv_q - np.multiply(self.alpha_1_q, self.Kp1_q))*dt
            self.Kp0_q = np.maximum(self.Kp0_q, 0.0001*np.ones(3))
            self.Kp1_q = np.maximum(self.Kp1_q, 0.0001*np.ones(3))

        Rho_q = self.Kp0_q + self.Kp1_q*self.euler_err

        delTau_q = np.zeros(3)
        delTau_q[0] = Rho_q[0]*self.sigmoid(sv_q[0],self.v)
        delTau_q[1] = Rho_q[1]*self.sigmoid(sv_q[1],self.v)
        delTau_q[2] = Rho_q[2]*self.sigmoid(sv_q[2],self.v)

        des_mom = - np.multiply(self.Lam_q, sv_q)  - delTau_q

        # putting limit on maximum vector
        if np.linalg.norm(des_mom) > self.max_mom:
            des_mom = (self.max_mom/np.linalg.norm(des_mom))*des_mom

        des_mom = self.norm_moment_const * des_mom

        moment = np.maximum(-self.max_mom_throttle, np.minimum(des_mom, self.max_mom_throttle))
        self.torq_cmd = moment
        

    def torque_to_PWM(self, value):
        fromMin, fromMax, toMin, toMax = -2, 2, 1000, 2000
        if(value>fromMax):
            value = fromMax
        if(value<fromMin):
            value = fromMin
        fromSpan = fromMax - fromMin
        toSpan = toMax - toMin

        valueScaled = float(value - fromMin) / float(fromSpan)

        val = int(toMin + (valueScaled * toSpan))
        return val

    def pub_att(self):
        self.moment_des()


# Main function
def main(argv):
    rospy.init_node('setpoint_node', anonymous=True)
    cnt = Controller()  # controller object
    rate = rospy.Rate(15)
    rospy.Subscriber('mavros/state', State, cnt.stateCb)
    rospy.Subscriber('mavros/local_position/odom', Odometry, cnt.odomCb)

    # Subscribe to drone's local position
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, cnt.posCb)
    rospy.Subscriber('new_pose', PoseStamped, cnt.newPoseCB)
    rospy.Subscriber('command/trajectory', Mdjt, cnt.multiDoFCb)
    sp_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    des_pub = rospy.Publisher('desired_position', PoseStamped, queue_size=10)
    fault_pub = rospy.Publisher('fault', Bool, queue_size=10)
    rate.sleep()

    print("ARMING")
    # while not cnt.state.armed:
        # modes.setArm()
        # cnt.armed = True
        # rate.sleep()

    cnt.armed = True
    k=0
    while k<20:
        sp_pub.publish(cnt.sp)
        fault_pub.publish(False)
        rate.sleep()
        k = k + 1

    # modes.setOffboardMode()
    print("---------")
    print("OFFBOARD")
    print("---------")

    # Trying to send actuator packets with DF
    connection_string = '127.0.0.1:14554'
    DFFlag = True

    trajectory_timer = 0.25
    angle = 0
    angle_delta = 0.025
    last_time = time.time()

    with DFAutopilot(connection_string=connection_string) as commander:
        if(DFFlag):
            for i in range(1,4):
                commander.set_motor_mode(i, 1)
                cnt.armed = True
            DFFlag = False
        
        while not rospy.is_shutdown():
            is_faulty = False
            # Generate trajectory point
            r = 3
            if(time.time() - last_time  > trajectory_timer):
                next_sp = PoseStamped()
                angle = angle + angle_delta
                # if(angle > 3.14):
                if(angle > 3.14/2):
                    is_faulty = True
                    cnt.EA = [
                            # [-0.9,0.9,0.9,0.9],
                            [-1,1,1,1],
                            [1,-1,1,1],
                            [1,1,-1,1],
                            [-1,-1,-1,1],
                        ]
                curr_x = cnt.cur_pose.pose.position.x
                curr_y =cnt.cur_pose.pose.position.y
                curr_z =cnt.sp.pose.position.z
                x = (r * math.sin(angle))
                y = (r * math.cos(angle))
                next_sp.pose.position.x = x
                next_sp.pose.position.y = y
                next_sp.pose.position.z = curr_z
                next_sp.pose.orientation = cnt.cur_pose.pose.orientation
                cnt.newPoseCB(next_sp)
                des_pub.publish(next_sp)

                fault_pub.publish(is_faulty)
                print(f"Fault: {is_faulty}, setpoint: {x, y, curr_z}")
                last_time = time.time()

            # Send attitude commands
            cnt.pub_att()
            Tp, Tq, Tr = cnt.torq_cmd
            T = cnt.th_cmd
            Torq = [Tp, Tq, Tr, T]

            # Compute CA matrix
            # cnt.CA = np.linalg.pinv(cnt.EA)
            # cnt.CA_inv = np.linalg.pinv(cnt.CA)
            # cnt.CA_inv = np.round(cnt.CA_inv, 5)
            u_input = np.matmul(cnt.EA, Torq)

            # Convert motor torque (input u) to PWM
            PWM_out_values = []
            i = 0
            for input in u_input:
                PWM = cnt.torque_to_PWM(input)
                PWM_out_values.append(PWM)
                i = i + 1

            i=1
            for PWM in PWM_out_values:
                commander.set_servo(i, PWM)
                i = i+1

            # print(f"Torq: {Torq}")
            # print(f"u inputs: {u_input}")
            # print(f"Sensor inputs: {roll, pitch, yaw, z}")
            # print(f"PWM outputs: {PWM_out_values}\n")
            rate.sleep()

if __name__ == '__main__':
    try:
        main(sys.argv[1:])
    except rospy.ROSInterruptException:
        pass