#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t
import math 

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Custom Topics(queue size = rate)
        self.pub1 = rospy.Publisher("/x_position_end_effector", Float64, queue_size=100)
        self.pub2 = rospy.Publisher("/y_position_end_effector", Float64, queue_size=100)
        self.pub3 = rospy.Publisher("/z_position_end_effector", Float64, queue_size=100)

        self.error_x = rospy.Publisher("/x_error_end_effector", Float64, queue_size=100)
        self.error_z = rospy.Publisher("/z_error_end_effector", Float64, queue_size=100)

        self.obs1 = rospy.Publisher("/y_position_Green_Obstacle", Float64, queue_size=100)
        self.obs2 = rospy.Publisher("/y_position_Red_Obstacle", Float64, queue_size=100)
        
        self.joint_3_dist_Green = rospy.Publisher("/Distance_joint_3_from_Green_Obstacle", Float64, queue_size=100)
        self.joint_3_dist_Red = rospy.Publisher("/Distance_joint_3_from_Red_Obstacle", Float64, queue_size=100)
        
        self.joint_4_dist_Green = rospy.Publisher("/Distance_joint_4_from_Green_Obstacle", Float64, queue_size=100)
        self.joint_4_dist_Red = rospy.Publisher("/Distance_joint_4_from_Red_Obstacle", Float64, queue_size=100)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):

        #Values chosen by using path planning from given configuration while having obstacle avoidance task
        q1 = 0.1935087641537656
        q2 = 0.8437180160844724
        q3 = 0.15734128218780974
        q4 = 1.630450591441229
        q5 = 0.0025424271244816055
        q6 = 0.6933653307901801
        q7 = 9.526694282335768e-06

        # set configuration
        self.joint_angpos = [q1, q2, q3, q4, q5, q6, q7]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        self.joint5_pos_pub.publish(self.joint_angpos[4])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        self.joint7_pos_pub.publish(self.joint_angpos[6])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint1_pos_pub.publish(self.joint_angpos[0])
        tmp_rate.sleep()
        self.joint3_pos_pub.publish(self.joint_angpos[2])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()

        ### PATH PLANNING ###
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        A07 = self.A07
        
        #Duration of Movement from Starting to Final Position 
        T = 2
        Tmax = int(100*T+1)

        #Samples
        t = [(x/100) for x in range(Tmax)]

        Pe = A07[0:3,3]

        #Desired values for starting position 
        xd_0 = Pe.item(0)
        yd_0 = Pe.item(1)
        zd_0 = Pe.item(2)

        #Distance
        dist_AB = 0.4

        #Desired values for final position 
        xd_f = Pe[0]
        yd_f = Pe[1] + dist_AB
        zd_f = Pe[2]

        #Coefficients of Polynomial 
        a0 = yd_0 
        a2 = 3/(T**2) * (yd_0-yd_f) 
        a3 = -2/(T**3) * (yd_0-yd_f) 

        #(xd, yd, zd)   : position 
        #(vxd, vyd, vzd): velocity 

        xd = np.ones(Tmax)*xd_0
        vxd = np.zeros(Tmax)

        yd  = a0 + a2*np.power(t, 2) + a3*np.power(t, 3)
        vyd = 2*(a2*t) + 3*a3*np.power(t, 2)

        zd = np.ones(Tmax)*zd_0
        vzd = np.zeros(Tmax)

        ### ALGORITHM ### 
        from_A_to_B = True
        tk = 0
        while not rospy.is_shutdown():
            while (tk < Tmax) and (tk >= 0):
                # Compute each transformation matrix wrt the base frame from joints' angular positions
                self.A01 = self.kinematics.tf_A01(self.joint_angpos)
                self.A02 = self.kinematics.tf_A02(self.joint_angpos)
                self.A03 = self.kinematics.tf_A03(self.joint_angpos)
                self.A04 = self.kinematics.tf_A04(self.joint_angpos)
                self.A05 = self.kinematics.tf_A05(self.joint_angpos)
                self.A06 = self.kinematics.tf_A06(self.joint_angpos)
                self.A07 = self.kinematics.tf_A07(self.joint_angpos)

                # Compute jacobian matrix
                J = self.kinematics.compute_jacobian(self.joint_angpos)
                # pseudoinverse jacobian
                pinvJ = pinv(J)

                
                v1d = np.matrix([[vxd[tk]],\
                                 [vyd.item(tk)],\
                                 [vzd[tk]]])

                p1d = np.matrix([[0.6043],\
                                 [yd.item(tk)],\
                                 [0.1508]])

                f1q = np.matrix([[self.A07[0,3]],\
                                 [self.A07[1,3]],\
                                 [self.A07[2,3]]])

                #Task 1
                K1 = 100
                parenthesis1 = v1d + K1 * (p1d - f1q) 
                task1 = np.dot(pinvJ, parenthesis1)

                #Gains needed for task 2
                Kc = 10
                K23 = 15
                K24 = 25

                #Middle of obstacles
                yobst = (self.model_states.pose[1].position.y + self.model_states.pose[2].position.y) / 2
                
                if from_A_to_B:
                    yobst = 0.075 + yobst
                else:
                    yobst = -0.05 + yobst

                #Distances of Joints from the middle of the obstacles(Criteria)
                jdist3 = (1/2) * Kc * ((self.A03[1,3] - yobst) ** 2)
                jdist4 = (1/2) * Kc * ((self.A04[1,3] - yobst) ** 2)

                #Initialisation of joint lengths l2, l3, l4 and angle theta1
                l2 = self.kinematics.l2
                l3 = self.kinematics.l3
                l4 = self.kinematics.l4
                theta1 = self.kinematics.theta1

                #Angular positions
                q1 = self.joint_angpos[0]
                q2 = self.joint_angpos[1]
                q3 = self.joint_angpos[2]

                #Sine and cosine functions
                c1 = math.cos(q1)
                c2 = math.cos(q2)
                c3 = math.cos(q3)

                s1 = math.sin(q1)
                s2 = math.sin(q2)
                s3 = math.sin(q3)
                

                size = (7,1)

                #Gradient of the criteria
                vyd3 = np.zeros(size)
                vyd3[0] = -Kc * (self.A03[1,3] - yobst) * l2*c1*s2
                vyd3[1] = -Kc * (self.A03[1,3] - yobst) * l2*c2*s1
                vyd3[2] = 0
                vyd3[3] = 0
                vyd3[4] = 0
                vyd3[5] = 0
                vyd3[6] = 0
                
                vyd4 = np.zeros(size)
                vyd4[0] = -Kc * (self.A04[1,3] - yobst) * (l2*c1*s2 - l3*(s1*s3 - c1*c2*c3))
                vyd4[1] = -Kc * (self.A04[1,3] - yobst) * (l2*c2*s1 - l3*c3*s1*s2)
                vyd4[2] = -Kc * (self.A04[1,3] - yobst) * (l3*(c1*c3 - c2*s1*s3))
                vyd4[3] = 0
                vyd4[4] = 0
                vyd4[5] = 0
                vyd4[6] = 0
                
                #Computing task 2
                parenthesis21 = np.eye(7) - np.dot(pinvJ, J)                    
                parenthesis22 = K23 * vyd3 + K24 * vyd4
                
                task2 = np.dot(parenthesis21, parenthesis22)

                #Algorithm
                maximum = max(jdist3, jdist4)
                
                if (maximum >= 0.03):
                    for i in range(7):
                        self.joint_angvel[i] = task1[i,0] + task2[i,0]
                else:
                    for i in range(7):
                        self.joint_angvel[i] = task1[i,0] 

                # Convertion to angular position after integrating the angular speed in time
                # Calculate time interval
                time_prev = time_now
                rostime_now = rospy.get_rostime()
                time_now = rostime_now.to_nsec()
                dt = (time_now - time_prev)/1e9

                # Integration
                self.joint_angpos = np.add(self.joint_angpos, [index * dt for index in self.joint_angvel])

                # Publish the new joint's angular positions
                self.joint1_pos_pub.publish(self.joint_angpos[0])
                self.joint2_pos_pub.publish(self.joint_angpos[1])
                self.joint3_pos_pub.publish(self.joint_angpos[2])
                self.joint4_pos_pub.publish(self.joint_angpos[3])
                self.joint5_pos_pub.publish(self.joint_angpos[4])
                self.joint6_pos_pub.publish(self.joint_angpos[5])
                self.joint7_pos_pub.publish(self.joint_angpos[6])

                self.pub1.publish(self.A07[0,3])
                self.pub2.publish(self.A07[1,3])
                self.pub3.publish(self.A07[2,3])

                self.error_x.publish(abs(self.A07[0,3] - 0.6043))
                self.error_z.publish(abs(self.A07[2,3] - 0.1508))

                green = self.model_states.pose[1].position.y
                red = self.model_states.pose[2].position.y
                
                self.obs1.publish(green)
                self.obs2.publish(red)

                self.joint_3_dist_Green.publish(self.A03[1,3] - green)
                self.joint_3_dist_Red.publish(red - self.A03[1,3])

                self.joint_4_dist_Green.publish(self.A04[1,3] - green)
                self.joint_4_dist_Red.publish(red - self.A04[1,3])

                self.pub_rate.sleep()

                if(from_A_to_B):
                    tk += 1
                else:
                    tk -= 1

                    
            if(from_A_to_B):
                tk -= 2
            else:
                tk += 2
            from_A_to_B = not from_A_to_B

    def turn_off(self):
        pass

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.on_shutdown(controller.turn_off)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
