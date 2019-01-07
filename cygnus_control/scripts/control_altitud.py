#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from mav_msgs.msg import RollPitchYawrateThrust
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
    global j
    y = data
    pid = args[1]
    target = args[0]
    pub = args[2]
    global empuje_global, empuje
    #rospy.loginfo("****************************************")
    #rospy.loginfo("Definiendo el topico a publicar")
    #pub = rospy.Publisher('/firefly/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
    diff = float(target) - float(y.pose.pose.position.z)
    #rospy.loginfo("target es: "+str(target))
    #rospy.loginfo("posicion actual: " + str(y.pose.pose.position.z))
    #rospy.loginfo("diff es: "+str(diff))
    u_ = pid.update(y.pose.pose.position.z, float( str(y.header.stamp.secs) +"." + str(y.header.stamp.nsecs) ) )
    #rospy.loginfo("El valor de u es: " +str(u_))
    #rospy.loginfo("Empuje actual es: "+str(empuje.thrust.z))

    #Landing
    if j==30:
        rospy.loginfo("inicializando thrust")
        empuje.thrust.z = empuje_global
        j+=1
    if diff>0:
        empuje.thrust.z = 1.51*10 + empuje.thrust.z*float(u_)
    else:
        empuje.thrust.z = 0.98*01.51*10 + empuje.thrust.z*float(u_)

    rospy.loginfo("agregado: " + str(empuje.thrust.z*float(u_)) )
    if empuje.thrust.z<14.9:
        empuje.thrust.z = 14.90000
    elif empuje.thrust.z>15.4:
        empuje.thrust.z = 15.40000

    if u_!=0:
        rospy.loginfo("publicando empuje")
        rospy.loginfo(empuje.thrust.z)
        pub.publish(empuje)



def plot(target,pid,time):
    # Plot results
    SP = np.ones_like(time)*target # altitude set point
    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    ax1.plot(time, soln[:,0],time,SP,'--')
    ax1.set_xlabel('Time, (sec)')
    ax1.set_ylabel('Altitude, (m)')

    ax2 = fig.add_subplot(212)
    ax2.plot(time, soln[:,1])
    ax2.set_xlabel('Time, (sec)')
    ax2.set_ylabel('Speed, (m/s)')
    plt.tight_layout()
    plt.show()

    fig2 = plt.figure()
    ax3 = fig2.add_subplot(111)
    ax3.plot(time, pid.u_p, label='u_p', linewidth=3, color = 'red')
    ax3.plot(time, pid.u_i, label='u_i', linewidth=3, color = 'blue')
    ax3.plot(time, pid.u_d, label='u_d', linewidth=3, color = 'green')
    ax3.set_xlabel('Time, (sec)')
    ax3.set_ylabel('Control Effort')
    h, l = ax3.get_legend_handles_labels()
    ax3.legend(h, l)
    plt.tight_layout()
    plt.show()
    ##################
    y0 = soln[:,0] #altitude
    rise_time_index =  np.argmax(y0>target)
    RT = time[rise_time_index]
    rospy.loginfo("The rise time is {0:.3f} seconds".format(RT))
    OS = (np.max(y0) - target)/target*100
    if OS < 0:
        OS = 0
    rospy.loginfo("The percent overshoot is {0:.1f}%".format(OS))
    rospy.loginfo("The steady state offset at 30 seconds is {0:.3f} meters".format(abs(soln[-1,0]-r)))



def pose_callback(data,args):
    global pid
    # Set altitude target
    target = data.pose.position.z
    pid.setTarget(target)
    rospy.loginfo("****************************************")
    rospy.loginfo("Nueva lectura de odometria")
    rospy.Subscriber('/iris/ground_truth/odometry', Odometry, odometry_callback,(target,pid,args))
    rospy.spin()
    

def talker():
    rospy.init_node('controller_z', anonymous=True)
    rate = rospy.Rate(0.0001) # 10hz
    rospy.loginfo("Definiendo el topico a publicar")
    pub = rospy.Publisher('/iris/command/roll_pitch_yawrate_thrust', RollPitchYawrateThrust, queue_size=10)
    while not rospy.is_shutdown():
        rospy.loginfo("Subscribiendo a poseStamped")
        rospy.Subscriber('/iris/command/pose', PoseStamped , pose_callback,(pub))
        rospy.spin()

if __name__ == '__main__':
    try:
        #CONSTANTES
        kp = 0.01
        ki = 0.00001
        kd = 0.15
        umax = 5.0 # max controller output, (N)
        alpha = 1 # derivative filter smoothing factor
        # Simulation parameters
        N = 400 # number of simulation points
        t0 = 0  # starting time, (sec)
        tf = 4 # end time, (sec)
        time = np.linspace(t0, tf, N)
        dt = time[1] - time[0] # delta t, (sec)
        #y = [data.pose.pose.position.z,data.twist.twist.linear.z]
        y = [0,0]
        # Initialize array to store values
        soln = np.zeros((len(time),len(y)))
        j=0
        # Create instance of PID_Controller class and 
        # initalize and set all the variables
        pid = PIDController(kp = kp, ki = ki, kd = kd, max_windup = 1e6, u_bounds
            = [0, umax], alpha = alpha)
        empuje_global =15.3000000
        empuje = RollPitchYawrateThrust()
        

        rospy.loginfo("Iniciando el nodo")
        talker()
        #rospy.spin()
    except rospy.ROSInterruptException:
        pass
