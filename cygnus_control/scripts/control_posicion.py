#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
import math
import matplotlib.pyplot as plt
import tf

class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 20,
            start_time = 0, alpha = 1., u_bounds = [float('-inf'), float('inf')]):
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)
        self.max_windup_ = float(max_windup) #Estipula el maximo para la parte integrativa
        self.alpha = float(alpha) #Factor que suaviza el efecto derivativo
        self.umin = u_bounds[0]
        self.umax = u_bounds[1]
        self.last_timestamp_ = 0.0
        self.set_point_ = 0.0
        self.start_time_ = start_time
        self.error_sum_ = 0.0
        self.last_error_ = 0.0
        #Aqui se almacena la historia del controlador para que pueda ser impresa
        self.u_p = [0]
        self.u_i = [0]
        self.u_d = [0]

    def reset(self):
        self.set_point_ = 0.0
        self.kp_ = 0.0
        self.ki_ = 0.0
        self.kd_ = 0.0
        self.error_sum_ = 0.0
        self.last_timestamp_ = 0.0
        self.last_error_ = 0
        self.last_last_error_ = 0
        self.last_windup_ = 0.0

    def setTarget(self, target):
        self.set_point_ = float(target)

    def setKP(self, kp):
        self.kp_ = float(kp)

    def setKI(self, ki):
        self.ki_ = float(ki)

    def setKD(self, kd):
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        self.max_windup_ = int(max_windup)

    def getTarget(self):
        target = float(self.set_point_)
        return target

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            return 0
        error = self.set_point_ - measured_value
        rospy.loginfo("el target en el pid es: " + str(self.set_point_))
        rospy.loginfo("el error = diff es: "+ str(error))
        self.last_timestamp_ = timestamp
        self.error_sum_ += error * delta_time
        delta_error = error - self.last_error_
        self.last_error_ = error
        if self.error_sum_ > self.max_windup_:
            self.error_sum_ = self.max_windup_
        elif self.error_sum_ < -self.max_windup_:
            self.error_sum_ = -self.max_windup_
        p = self.kp_ * error
        rospy.loginfo("El valor de p es: " + str(p))
        i = self.ki_ * self.error_sum_
        d = self.kd_ * (self.alpha * delta_error / delta_time + \
            (1 - self.alpha)  * self.last_error_ / delta_time)
        ########################################
        # Here we are storing the control effort history for analysis
        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)
        return p+d+i


def odometry_callback(data,args):
    global j, empuje_angles, pid_x, pid_y, Ax, Ay, target_z
    pid_x = args[2]
    pid_y = args[3]
    target_y = pid_y.getTarget()
    target_x = pid_x.getTarget()
    pub = args[4]
    # Diferencia entre los targets
    diff_x = float(target_x) - float(data.pose.pose.position.x)
    diff_y = float(target_y) - float(data.pose.pose.position.y)
    # Controladores PID para posicion X e Y
    u_x = pid_x.update(data.pose.pose.position.x, float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
    u_y = pid_y.update(data.pose.pose.position.y, float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
    rospy.loginfo("***********************")
    rospy.loginfo(u_x)
    rospy.loginfo("***********************")
    rospy.loginfo(u_y)
    ####################################################
    # theta_roll_d  = 0.09*[Ax*sin(yaw) - Ay*cos(yaw)]
    # omega_pitch_d = 0.09*[Ax*sin(yaw) + Ay*cos(yaw)]
    ####################################################

    # Euler from Quaternion
    quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    rospy.loginfo("****************")
    rospy.loginfo(float(euler[0]))
    rospy.loginfo(float(euler[1]))

    # La aceleracion se plantea constante aunque se podria leer de la imu la aceleracion real, mas adelante arreglar y ya no se necesitarian los if
    # para hacer un mejor debug se hara constante inicialmente math.sin(euler[2]) math.cos(euler[2])
    Ax = 0.1
    Ay = 0.1

    #if diff_x>0:
    #    if diff_y>0:
    #        theta_roll_d  = float(0.09*(Ax*math.sin(euler[2]) - Ay*math.cos(euler[2])) + u_x)
    #        omega_pitch_d = float(0.09*(Ax*math.sin(euler[2]) + Ay*math.cos(euler[2])) + u_y)
    #    else:
    #        theta_roll_d  = float(0.09*(Ax*math.sin(euler[2]) + Ay*math.cos(euler[2])) + u_x)
    #        omega_pitch_d = float(0.09*(Ax*math.sin(euler[2]) - Ay*math.cos(euler[2])) + u_y)
    #else:
    #    if diff_y>0:
    #        theta_roll_d  = float(0.09*(-Ax*math.sin(euler[2]) - Ay*math.cos(euler[2])) + u_x)
    #        omega_pitch_d = float(0.09*(-Ax*math.sin(euler[2]) + Ay*math.cos(euler[2])) + u_y)
    #    else:
    #        theta_roll_d  = float(0.09*(-Ax*math.sin(euler[2]) + Ay*math.cos(euler[2])) + u_x)
    #        omega_pitch_d = float(0.09*(-Ax*math.sin(euler[2]) - Ay*math.cos(euler[2])) + u_y)  

    #theta_roll_d  = 0.09*(Ax*math.sin(euler[2]) - Ay*math.cos(euler[2])) - u_y 
    #omega_pitch_d = 0.09*(Ax*math.sin(euler[2]) + Ay*math.cos(euler[2])) - u_x 

    k_1 = 0.35
    rospy.loginfo(" DIFF Y ")
    rospy.loginfo( diff_y )
    u_y_ = k_1*diff_y # k_2*(y.twist.twist.linear.x - 0.3) - 0.08
    rospy.loginfo(" CONTROL U_Y")
    rospy.loginfo( u_y_ )
    theta_roll_d  = 0.102*( math.sin(euler[2])*0  - math.cos(euler[2])*u_y_ )
    rospy.loginfo("ANGULO YAW")
    rospy.loginfo(euler[2])
    rospy.loginfo(" COSENO ANGULO YAW")
    rospy.loginfo( math.cos(euler[2]))
    rospy.loginfo(" Angulo deseado ")
    rospy.loginfo( theta_roll_d )

    #omega_pitch_d =


    #if (diff_y > 0):
    #    theta_roll_d  =  -0.04725 - u_y 
    #else:
    #    theta_roll_d  =  0.04725 - u_y 

    #theta_roll_d  =  euler[0] - u_y 
    if theta_roll_d > 0.08725:
        theta_roll_d = 0.08725
    elif theta_roll_d < -0.08725:
        theta_roll_d = -0.08725

    #omega_pitch_d =  euler[1] - u_x 
    omega_pitch_d = 0
    if omega_pitch_d > 0.08725:
        omega_pitch_d = 0.08725
    elif omega_pitch_d < -0.08725:
        omega_pitch_d = -0.08725

    #if j<=100 and diff_y > 0.2:
    #    theta_roll_d = 0.2
    #    j = j+ 1

    empuje_angles.roll = theta_roll_d
    rospy.loginfo("****************")
    rospy.loginfo("U-y es:")
    rospy.loginfo(u_y_)
    rospy.loginfo("ROLL deseado")
    rospy.loginfo(empuje_angles.roll)
    
    empuje_angles.pitch = omega_pitch_d
    rospy.loginfo("****************")
    rospy.loginfo(empuje_angles.pitch)

    if u_x!=0 and u_y!=0:
        #rospy.loginfo("publicando empuje")
        empuje_angles.thrust.z = target_z
        pub.publish(empuje_angles)
        rospy.sleep(0.01)
        

def pose_callback(data):
    global pid_x, pid_y, empuje_angles
    # Set altitude target
    target_x = data.pose.position.x
    target_y = data.pose.position.y
    empuje_angles.thrust.z = data.pose.position.z
    pid_x.setTarget(target_x)
    pid_y.setTarget(target_y)
    #rospy.spin()
    #rospy.sleep(1)

def imu_callback(data):
    global Ax, Ay
    Ax = data.linear_acceleration.x
    Ay = data.linear_acceleration.y 

def talker():
    global pid_x, pid_y, target_x, target_y
    rospy.init_node('controller_xy', anonymous=True)
    #rate = rospy.Rate(0.01) # 10hz
    rospy.loginfo("Definiendo el topico a publicar")
    pub = rospy.Publisher('/cygnus/command/roll_pitch_yawrate', RollPitchYawrateThrust, queue_size=5)
    while not rospy.is_shutdown():
        rospy.loginfo("Subscribiendo a poseStamped")
        rospy.Subscriber('/cygnus/command/pose', PoseStamped , pose_callback)
        rospy.loginfo("****************************************")
        rospy.loginfo("Nueva lectura de odometria")
        rospy.Subscriber('/cygnus/ground_truth/odometry', Odometry, odometry_callback,(target_y,target_x,pid_x,pid_y,pub) )
        rospy.Subscriber('/cygnus/imu', Imu , imu_callback)

        rospy.spin()

if __name__ == '__main__':
    try:
        #CONSTANTES
        kp = 0.015
        ki = 0
        kd = 0

        kp_y = 0.025
        ki_y = 0
        kd_y = 0

        Ax = 0.1
        Ay = 0.1
        target_x = 0
        target_y = 0
        target_z = 0

        umax = 5.0 # max controller output, (N)
        alpha = 1 # derivative filter smoothing factor
        j=0
        # Create instance of PID_Controller class and 
        # initalize and set all the variables
        pid_x = PIDController(kp = kp, ki = ki, kd = kd, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)

        pid_y = PIDController(kp = kp_y, ki = ki_y, kd = kd_y, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)

        empuje_angles = RollPitchYawrateThrust()

        rospy.loginfo("Iniciando el nodo")
        talker()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
