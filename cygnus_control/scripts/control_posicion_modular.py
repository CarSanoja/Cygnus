#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
import math
import tf
import message_filters

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
        self.actual_reference = 0.0
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
    def setActualRef(self, reference):
        self.actual_reference = reference
    def getActualRef(self):
        return self.actual_reference
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
    def update(self, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            return 0
        error = self.set_point_ - self.actual_reference
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

class angles_thrust:
    def __init__(self, roll = 0.0, pitch = 0.0, yaw = 0.0, yaw_rate=0.0, thrust = 0.0,  time=0.0):
        self.roll_ = float(roll)
        self.pitch_ = float(pitch)
        self.yaw_ = float(yaw)
        self.yaw_rate_ = float(yaw_rate)
        self.thrust_ = float(thrust)
        self.time_ = float(time)
    def setRoll(self, roll):
        self.roll_ = roll
    def getRoll(self):
        return self.roll_
    def setPitch(self, pitch):
        self.pitch_ = pitch 
    def getPitch(self):
        return self.pitch_
    def setYaw(self, yaw):
        self.yaw_ = yaw 
    def getYaw(self):
        return self.yaw_
    def setYawRate(self, yaw_rate):
        self.yaw_rate_ = yaw_rate
    def getYawRate(self):
        return self.yaw_rate_
    def setThrust(self, thrust):
        self.thrust_ = thrust
    def getThrust(self):
        return self.thrust_
    def setTime(self, time):
        self.time_ = time
    def getTime(self):
        return self.time_ 


def publish(message, pub_topic, angles):
	message.thrust.z = angles.getThrust()
	message.roll = angles.getRoll()
	message.pitch = angles.getPitch()
	message.yaw_rate = angles.getYawRate()
	pub_topic.publish(message)

def control_x(pid_x, angles, actual_velocity):
    target_x = pid_x.getTarget()
    actual_x = pid_x.getActualRef()
    actual_yaw = angles.getYaw()
    actual_time = angles.getTime()
    diff_x = float(target_x) - float(actual_x)
    u_x = pid_x.update(actual_time )
    k_1 = 0.14
    k_2 = 0.004
    u_x_ = k_1*diff_x + k_2*(actual_velocity - 0) #- 0.08
    omega_pitch_d  = 0.102*( math.cos(actual_yaw)*u_x_  + math.sin(actual_yaw)*0 )
    if omega_pitch_d > 0.08725:
        omega_pitch_d = 0.08725
    elif omega_pitch_d < -0.08725:
        omega_pitch_d = -0.08725
    rospy.loginfo("ANGULO PITCH a publicar")
    rospy.loginfo(omega_pitch_d)
    angles.setPitch(omega_pitch_d)

def control_y(pid_y, angles, actual_velocity):
    target_y = pid_y.getTarget()
    actual_y = pid_y.getActualRef()
    actual_yaw = angles.getYaw()
    actual_time = angles.getTime()
    diff_y = float(target_y) - float(actual_y)
    u_y = pid_y.update(actual_time )
    k_1 = 0.15
    k_2 = 0.009
    u_y_ = k_1*diff_y + k_2*(actual_velocity - 0) #- 0.08
    theta_roll_d  = 0.102*( math.sin(actual_yaw)*u_y  - math.cos(actual_yaw)*u_y )
    if theta_roll_d > 0.08725:
        theta_roll_d = 0.08725
    elif theta_roll_d < -0.08725:
        theta_roll_d = -0.08725
    rospy.loginfo("ANGULO ROLL a publicar")
    rospy.loginfo(theta_roll_d)
    angles.setRoll(theta_roll_d)


def control_z(pid, angles,pid_y):
    target = pid.getTarget()
    actual_thrust = angles.getThrust()
    actual_time = angles.getTime()
    actual_z = pid.getActualRef()
    diff = float(target) - float(actual_z)
    if (diff <= 0.002 and diff >= 0):
        pid_x.setTarget(1.5)
        pid_y.setTarget(1.5)
        pid.setTarget(1.5)
    u_ = pid.update(actual_time )
    if diff>0:
        aux_thrust = 1.43*10 + actual_thrust*float(u_) 
    else:
        aux_thrust = 0.98*01.43*10 + actual_thrust*float(u_) 
    if aux_thrust<13.6:
        aux_thrust = 13.60000
    elif aux_thrust>15:
        aux_thrust = 15.20000
    angles.setThrust(aux_thrust)
    rospy.loginfo("saliendo control z")
    
    
def odometry_callback(data, args):
    pub = args[0]
    empuje_angles_ = args[1]
    rospy.loginfo("entrando odometria")
    global pid_x, pid_y, pid, angles, empuje_angles_
    pid_x.setActualRef(data.pose.pose.position.x)
    pid_y.setActualRef(data.pose.pose.position.y)
    pid.setActualRef(data.pose.pose.position.z)
    # Euler from Quaternion
    quaternion = (data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    angles.setYaw(euler[2])
    angles.setTime( float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
    rospy.loginfo("saliendo odometria")

    rospy.loginfo("entrando a control_z")
    control_z(pid,angles,pid_y)

    rospy.loginfo("entrando a control y")
    control_y(pid_y,angles, data.twist.twist.linear.y)

    rospy.loginfo("entrando a control x")
    control_x(pid_x, angles, data.twist.twist.linear.x)

    rospy.loginfo("publicando")
    publish(empuje_angles_,pub,angles)


def pose_callback(data):
	global pid_x, pid_y, pid, goal
	pid_x.setTarget(data.pose.position.x)
	pid_y.setTarget(data.pose.position.y)
	pid.setTarget(data.pose.position.z)
	goal = True


def talker(pid_x_, pid_y_, pid_, empuje_angles_, angles_):
    global goal
    pid_.setTarget(0.5)
    rospy.init_node('controller_xy', anonymous=True)
    rospy.loginfo("Definiendo el topico a publicar")
    pub = rospy.Publisher('/cygnus/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
    while not rospy.is_shutdown():
    	rospy.loginfo("Subscribiendo a poseStamped")
    	rospy.Subscriber('/cygnus/command/pose', PoseStamped , pose_callback)
    	rospy.loginfo("****************************************")
    	if goal != True:
    		rospy.loginfo("Nueva lectura de odometria")
    		rospy.Subscriber('/cygnus/ground_truth/odometry', Odometry, odometry_callback, (pub,empuje_angles_))
    	rospy.spin()

if __name__ == '__main__':
    try:
        #CONSTANTES
        kp = 0.195
        ki = 0.05
        kd = 1.8

        kp_x = 0.015
        ki_x = 0
        kd_x = 0

        kp_y = 0.16
        ki_y = 0
        kd_y = 0.02

        umax = 5.0 # max controller output, (N)
        alpha = 1 # derivative filter smoothing factor

        # Create instance of PID_Controller class and 
        # initalize and set all the variables
        pid_x = PIDController(kp = kp_x, ki = ki_x, kd = kd_x, max_windup = 1, u_bounds
            = [0, umax], alpha = alpha)

        pid_y = PIDController(kp = kp_y, ki = ki_y, kd = kd_y, max_windup = 1, u_bounds
            = [0, umax], alpha = alpha)

        pid = PIDController(kp = kp, ki = ki, kd = kd, max_windup = 0.5, u_bounds
            = [0, umax], alpha = alpha)

        empuje_angles = RollPitchYawrateThrust()
        goal = False
        angles = angles_thrust()

        rospy.loginfo("Iniciando el nodo")
        talker(pid_x,pid_y,pid, empuje_angles, angles )
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
