#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
from mav_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
import matplotlib.pyplot as plt

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

    def update(self, measured_value, timestamp):
        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0:
            return 0
        error = self.set_point_ - measured_value
        rospy.loginfo("el target en el pid es: " + str(self.set_point_))
        rospy.loginfo("el error = diff es: "+ str(error))
        self.last_timestamp_ = timestamp
        delta_error = error - self.last_error_
        self.last_error_ = error
        p = self.kp_ * error
        rospy.loginfo("El valor de p es: " + str(p))
        d = self.kd_ * (self.alpha * delta_error / delta_time + \
            (1 - self.alpha)  * self.last_error_ / delta_time)
        ########################################
        # Here we are storing the control effort history for analysis
        #self.u_p.append(p)
        #self.u_d.append(d)
        return p+d

def calcular_wMotores(troll,tpitch,tyaw,ft):
	t1 = troll/(0.00000008182*0.225)
	t2 = tpitch/(0.00000008182*0.225)
	t3 = tyaw/0.00000000161
	t4 = ft/0.00000008182
	rospy.loginfo("*********")
	rospy.loginfo("troll: " + str(troll) + " y t1: " + str(t1))
	rospy.loginfo("tpitch: " + str(tpitch) + " y t2: " + str(t2))
	rospy.loginfo("tyaw: " + str(tyaw) + " y t3: " + str(t3))
	rospy.loginfo("ft es: " + str(ft) + " y t4 esta valiendo.." + str(t4))
	w1 = math.sqrt(-t1-t2+t3+t4)*0.1047/2  - 44
	w2 = math.sqrt(-t1+t2-t3+t4)*0.1047/2  - 44
	w3 = math.sqrt(t1+t2+t3+t4)*0.1047/2   - 44
	w4 = math.sqrt(t1-t2-t3+t4)*0.1047/2   - 44
	rospy.loginfo("velocidades angulares legal_ "+"w1: "+str(w1) + " w2: "+str(w2)+" w3: "+str(w3)+" w4: "+str(w4))
	return (w1,w2,w3,w4)


def odometry_callback(data,args):
	global pid_roll, pid_pitch, pid_yaw, pid_altitud, Ixx, Iyy, Izz, w_motores
	u_roll = pid_roll.update(data.pose.pose.orientation.x, float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
	u_pitch = pid_pitch.update(data.pose.pose.orientation.y, float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
	u_yaw = pid_yaw.update(data.pose.pose.orientation.z, float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
	u_altitud = pid_altitud.update(data.pose.pose.position.z, float( str(data.header.stamp.secs) +"." + str(data.header.stamp.nsecs) ) )
	rospy.loginfo("u_roll: " + str(u_roll) + " u_pitch: " + str(u_pitch) + " u_yaw: " + str(u_yaw) + " u_altitud: " + str(u_altitud))
	tao_roll = math.sqrt(2)*Ixx*u_roll
	tao_pitch = math.sqrt(2)*Iyy*u_pitch
	tao_yaw = math.sqrt(2)*Izz*u_yaw
	rospy.loginfo("tao_roll: " + str(tao_roll) + " tao_pitch: " + str(tao_pitch) + " tao_yaw: " + str(tao_yaw))
	w1, w2, w3, w4 = calcular_wMotores(tao_roll,tao_pitch,tao_yaw,args[0].thrust.z)

	w1 = w1 - u_roll - u_pitch + u_yaw + u_altitud
	if w1>= 652.4:
		w1=652.4
	elif w1<=647.3:
		w1=647.3
	else:
		pass
	w2 = w2 - u_roll + u_pitch - u_yaw + u_altitud
	if w2>= 652.4:
		w2=652.4
	elif w2<=647.3:
		w2=647.3
	else:
		pass
	w3 = w3 + u_roll + u_pitch + u_yaw + u_altitud
	if w3>= 652.4:
		w3=652.4
	elif w3<=647.3:
		w3=647.3
	else:
		pass
	w4 = w4 + u_roll - u_pitch - u_yaw + u_altitud
	if w4>= 652.4:
		w4=652.4
	elif w4<=647.3:
		w4=647.3
	else:
		pass

	w_motores.angular_velocities = [w1,w2,w3,w4] 
	rospy.loginfo("w1: "+str(w1) + " w2: "+str(w2)+" w3: "+str(w3)+" w4: "+str(w4))
	args[1].publish(w_motores)
	#rospy.sleep(0.003)



def altitud_callback(data,args):
	global pid_roll, pid_pitch, pid_altitud
	pid_roll.setTarget(data.roll)
	pid_pitch.setTarget(data.pitch)
	pid_yaw.setTarget(data.yaw_rate)
	pid_altitud.setTarget(1)
	rospy.Subscriber('/cygnus/ground_truth/odometry', Odometry, odometry_callback,(data,args))
	rospy.spin()


def talker():
    rospy.init_node('control_orientacion', anonymous=True)
    rospy.loginfo("Definiendo el topico a publicar")
    pub = rospy.Publisher('/cygnus/command/motor_speed', Actuators, queue_size=10)
    #while not rospy.is_shutdown():
    rospy.loginfo("Subscribiendo a controlador de altitud ")
    rospy.Subscriber('/cygnus/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust , altitud_callback,(pub))
    rospy.spin()

if __name__ == '__main__':
    try:
    	#Constates altitud
        kp = 1.4
        ki = 0
        kd = 1.2
        #CONSTANTES
        Ixx = 0.013735
        Iyy = 0.013868
        Izz = 0.025071
        kp_r = 0.000001
        kd_r = 0.0000002
        kp_p = 0.000001
        kd_p = 0.0000002
        kp_y = 0.000001
        kd_y = 0
        umax = 5.0 # max controller output, (N)
        alpha = 1 # derivative filter smoothing factor
        pid_roll = PIDController(kp = kp_r, ki = 0, kd = kd_r, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)
        pid_pitch = PIDController(kp = kp_p, ki = 0, kd = kd_p, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)
        pid_yaw = PIDController(kp = kp_y, ki = 0, kd = kd_y, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)
        pid_altitud = PIDController(kp = kp, ki = ki, kd = kd, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)
        w_motores = Actuators()
        
        rospy.loginfo("Iniciando el nodo")
        talker()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass