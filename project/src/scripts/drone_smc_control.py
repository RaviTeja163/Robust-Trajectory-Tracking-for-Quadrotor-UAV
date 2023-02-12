#!/usr/bin/env python3
from time import time
from traceback import print_tb
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position, xcor
import numpy as np
from numpy import NaN
import math
import rospy
import time
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
class Quadrotor():
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
        
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []
        self.omega = 0
        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)

    # TODO: include initialization codes if needed

    def traj_evaluate(self, start_pos, des_pos, T0, Tf):
        
        x0 = start_pos[0]
        xd0 = 0
        xdd0 = 0

        y0 = start_pos[1]
        yd0 = 0
        ydd0 = 0

        z0 = start_pos[2]
        zd0 = 0
        zdd0 = 0

        xf = des_pos[0]
        xdf = 0
        xddf = 0

        yf = des_pos[1]
        ydf = 0
        yddf = 0

        zf = des_pos[2]
        zdf = 0
        zddf = 0 
        
        A = np.array(
            [[1, T0, T0**2, T0**3, T0**4, T0**5],
             [0, 1, 2*T0, 3*T0**2, 4*T0**3, 5*T0**4],
             [0, 0, 2, 6*T0, 12*T0**2, 20*T0**3],
             [1, Tf, Tf**2, Tf**3, Tf**4, Tf**5],
             [0, 1, 2*Tf, 3*Tf**2, 4*Tf**3, 5*Tf**4],
             [0, 0, 2, 6*Tf, 12*Tf**2, 20*Tf**3],
            ])

        X = np.array(
            [[x0],
             [xd0],
             [xdd0],
             [xf],
             [xdf],
             [xddf]
            ])

        Y = np.array(
            [[y0],
             [yd0],
             [ydd0],
             [yf],
             [ydf],
             [yddf]
            ])

        Z = np.array(
            [[z0],
             [zd0],
             [zdd0],
             [zf],
             [zdf],
             [zddf]
            ])

        xcoeff = np.linalg.solve(A, X)
        ycoeff = np.linalg.solve(A, Y)
        zcoeff = np.linalg.solve(A, Z)

        # TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations

        return xcoeff, ycoeff, zcoeff
       
    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):

        # obtain the desired values by evaluating the corresponding trajectories
        if self.t > 0 and self.t <= 5:
            xc, yc, zc = self.traj_evaluate([0,0,0], [0,0,1], 0, 5)
        if self.t > 5 and self.t <= 20:
            xc, yc, zc = self.traj_evaluate([0,0,1], [1,0,1], 5, 20)
        if self.t > 20 and self.t <= 35:
            xc, yc, zc = self.traj_evaluate([1,0,1], [1,1,1], 20, 35)
        if self.t > 35 and self.t <= 50:
            xc, yc, zc = self.traj_evaluate([1,1,1], [0,1,1], 35, 50)
        if self.t > 50 and self.t <= 65:
            xc, yc, zc = self.traj_evaluate([0,1,1], [0,0,1], 50, 65)


        x_des = xc[5]*self.t**5 + xc[4]*self.t**4 + xc[3]*self.t**3 + xc[2]*self.t**2 + xc[1]*self.t + xc[0]
        x_des_dot = 5 * xc[5] *self.t**4 + 4 * xc[4]*self.t**3 + 3 * xc[3]*self.t**2 + 2 * xc[2]*self.t + xc[1]
        x_des_ddot = 20 * xc[5] *self.t**3 + 12 * xc[4]*self.t**2 + 6 * xc[3]*self.t + 2 * xc[2]

        y_des = yc[5]*self.t**5 + yc[4]*self.t**4 + yc[3]*self.t**3 + yc[2]*self.t**2 + yc[1]*self.t + yc[0]
        y_des_dot = 5 * yc[5] *self.t**4 + 4 * yc[4]*self.t**3 + 3 * yc[3]*self.t**2 + 2 * yc[2]*self.t + yc[1]
        y_des_ddot = 20 * yc[5] *self.t**3 + 12 * yc[4]*self.t**2 + 6 * yc[3]*self.t + 2 * yc[2]

        z_des = zc[5]*self.t**5 + zc[4]*self.t**4 + zc[3]*self.t**3 + zc[2]*self.t**2 + zc[1]*self.t + zc[0]
        z_des_dot = 5 * zc[5] *self.t**4 + 4 * zc[4]*self.t**3 + 3 * zc[3]*self.t**2 + 2 * zc[2]*self.t + zc[1]
        z_des_ddot = 20 * zc[5] *self.t**3 + 12 * zc[4]*self.t**2 + 6 * zc[3]*self.t + 2 * zc[2]

        # TODO: implement the Sliding Mode Control laws designed in Part 2 to calculate the control inputs "u"

        m = 0.027
        l = 46/1000
        Ix = 16.571710 * pow(10 ,-6)
        Iy = 16.571710 * pow(10 ,-6)
        Iz = 29.261652 * pow(10,-6)
        Ip = 12.65625 * pow(10,-8)
        kF = 1.28192 * pow(10,-8)
        kM = 5.964552 * pow(10,-3)
        wmax = 2618
        g = 9.8

        ex  = xyz[0, 0] - x_des
        ex_dot  = xyz_dot[0, 0] - x_des_dot
        ey  = xyz[1, 0] - y_des
        ey_dot  = xyz_dot[1, 0] - y_des_dot
        ez  = xyz[2, 0] - z_des
        ez_dot  = xyz_dot[2, 0] - z_des_dot
        
        Kp = 20
        Kd = -5
        
        Fx = m*(-Kp*ex - Kd*ex_dot + x_des_ddot)
        Fy = m*(-Kp*ey - Kd*ey_dot + y_des_ddot)

        phi = rpy[0, 0]
        theta = rpy[1, 0]
        psi = rpy[2, 0] 

        phi_dot = rpy_dot[0, 0]
        theta_dot = rpy_dot[1, 0]
        psi_dot = rpy_dot[2, 0] 
        
        coeff = 0.2  # for boundary layer
        lambda1 = 0.5
        s1 =  ez_dot + lambda1 * (ez)
        sat_s1 = min(max(s1/coeff, -1), 1)
        k1 = 1
        u1 = (m/(cos(phi)*cos(theta))) * (z_des_ddot  + g - lambda1 * ez_dot - k1 * np.sign(s1))
        # u1 = (m/(cos(phi)*cos(theta))) * (z_des_ddot  + g - lambda1 * ez_dot - k1 * sat_s1)
        
        f1 = Fx/u1
        if f1 < -1:
            f1 = -1
        if f1 > 1:
            f1 = 1

        f2 = Fy/u1
        if f2 < -1:
            f2 = -1
        if f2 > 1:
            f2 = 1

        theta_des = asin(f1)
        phi_des = asin(-f2)

        ephi = rpy[0, 0] - phi_des
        ephi_dot = rpy_dot[0, 0]

        etheta = rpy[1, 0] - theta_des
        etheta_dot = rpy_dot[1, 0]

        epsi = rpy[2, 0]
        epsi_dot = rpy_dot[2, 0]

        lambda2 = 10
        s2 = ephi_dot + lambda2*np.arctan2(np.sin(ephi),np.cos(ephi)) 
        sat_s2 = min(max(s2/coeff, -1), 1)
        k2 = 150
        u2 = -theta_dot * psi_dot * (Iy - Iz) + Ip * theta_dot * self.omega + Ix * (-lambda2 * phi_dot - k2 * np.sign(s2))
        # u2 = -theta_dot * psi_dot * (Iy - Iz) + Ip * theta_dot * self.omega + Ix * (-lambda2 * phi_dot - k2 * sat_s2)

        lambda3 = 15
        s3 = etheta_dot + lambda3*np.arctan2(np.sin(etheta),np.cos(etheta))
        sat_s3 = min(max(s3/coeff, -1), 1)
        k3 = 200
        # u3 = -phi_dot * psi_dot * (Iz - Ix) - Ip * phi_dot * self.omega + Iy * ( -lambda3 * theta_dot - k3 * sat_s3)
        u3 = -phi_dot * psi_dot * (Iz - Ix) - Ip * phi_dot * self.omega + Iy * ( -lambda3 * theta_dot - k3 * np.sign(s3))

        lambda4 = 10
        s4 = epsi_dot + lambda4*np.arctan2(np.sin(epsi),np.cos(epsi))
        sat_s4 = min(max(s4/coeff, -1), 1)
        k4 = 5 
        # u4 = -theta_dot * phi_dot * (Ix - Iy) + Iz * (-lambda4 * psi_dot - k4 * sat_s4)
        u4 = -theta_dot * phi_dot * (Ix - Iy) + Iz * (-lambda4 * psi_dot - k4 * np.sign(s4))
    
        aloc_matrix = np.array([[1/(4*kF), (2**0.5)*-1/(4*kF*l), (2**0.5)*-1/(4*kF*l), -1/(4*kM*kF)],
                                [1/(4*kF), (2**0.5)*-1/(4*kF*l), (2**0.5)*1/(4*kF*l), 1/(4*kM*kF)],
                                [1/(4*kF), (2**0.5)*1/(4*kF*l), (2**0.5)*1/(4*kF*l), -1/(4*kM*kF)],
                                [1/(4*kF), (2**0.5)*1/(4*kF*l), (2**0.5)*-1/(4*kF*l), 1/(4*kM*kF)]
                               ])

        u = np.array([u1,u2,u3,u4])
        
        w = np.matmul(aloc_matrix, u)
        w = np.abs(w)

        motor_vel = w**(1/2)
        # motor_vel1 = np.sqrt(w[0,0])
        # motor_vel2 = np.sqrt(w[1,0])
        # motor_vel3 = np.sqrt(w[2,0])
        # motor_vel4 = np.sqrt(w[3,0])

        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]

        # TODO: convert the desired control inputs "u" to desired rotor
        #velocities "motor_vel" by using the "allocation matrix"

        # TODO: maintain the rotor velocities within the valid range of [0 to 2618]
        
        if motor_vel[0] > wmax:
            motor_vel[0] = wmax

        if motor_vel[1] > wmax:
            motor_vel[1] = wmax

        if motor_vel[2] > wmax:
            motor_vel[2] = wmax

        if motor_vel[3] > wmax:
            motor_vel[3] = wmax 

        self.omega = motor_vel[0] - motor_vel[1] + motor_vel[2] - motor_vel[3]


        # publish the motor velocities to the associated ROS topic     
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0], motor_vel[1], motor_vel[2], motor_vel[3]]
        self.motor_speed_pub.publish(motor_speed)


    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
        
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear. y], [msg.twist.twist.linear.z]])
       
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position. y], [msg.pose.pose.position.z]])

        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
        [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
        [0, np.cos(rpy[0]), -np.sin(rpy[0])],
        [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
        ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
        # save the actual trajectory data

    def save_data(self):
        # TODO: update the path below with the correct path
        with open("/home/ravi/rbe502_project/src/project/src/scripts/log.pkl","wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series, self.x_series, self.y_series, self.z_series], fp)
            
if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")