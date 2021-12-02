#!/usr/bin/env python

import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from nav_msgs.msg import Odometry # for receiving odometry feedback
from std_msgs.msg import Empty

# Global variables 
global z_ref


command = Twist()


global x,y,z,psi,vx,vy
x = 0.0
y = 0.0
z = 0.0
psi = 0.0

psi_deg = 0.0
vx = 0.0
vy = 0.0
zp = 0.0

k = 0.5

# Node init
rospy.init_node('cmd', anonymous=True)

# reference
# can also be coded as global variable, input topic, value read from textfile, etc.
x_ref = 3.5
y_ref = 3.5
z_ref = 3.5
psi_ref = np.pi

# Publisher declaration on the topic of command
pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
publand = rospy.Publisher('/ardrone/land', Empty)
commandRate = rospy.Rate(10) 


# Measurements reading
def readXYZ(data):
    global x, y, z, psi
    # Data reading X axis
    x = data.pose.pose.position.x 
    # Data reading Y axis
    y = data.pose.pose.position.y    
    # Data reading Z axis
    z = data.pose.pose.position.z   



def readPsiVxVy(data):
    global psi_deg, vx, vy, z
    # Data reading Z axis from ultrasonic sensor
    #z = data.altd / 1000.0   
    # Data reading yaw angle (deg)
    psi_deg = data.rotZ
    vx = data.vx
    vy = data.vy
    #z = data.altd / 1000.0


# Subscriber declaration 
rospy.Subscriber("/ground_truth/state", Odometry, readXYZ)
rospy.Subscriber("/ardrone/navdata", Navdata, readPsiVxVy)


# Main: looping execution
if __name__ == '__main__':
    global x, y, z, vx, vy, psi_deg, x_ref, y_ref, z_ref, k
    
    psi_rad = 0.0
    while not rospy.is_shutdown():
        rospy.loginfo(" zref=%f m,  z=%f m", z_ref, z)
        rospy.loginfo(" yref=%f m,  y=%f m", y_ref, y)
        rospy.loginfo(" xref=%f m,  x=%f m", x_ref, x)
        rospy.loginfo(" psi-ref=%f m,  psi=%f m", psi_ref, psi_rad)
        
        psi_rad = psi_deg * (np.pi/180)
        
	vx = k*(x_ref  - x)
	vy = k*(y_ref  - y)
	corr_x = vx*np.cos(psi_rad)+np.sin(psi_rad)*vy
	corr_y = -vx*np.sin(psi_rad)+np.cos(psi_rad)*vy
	corr_z = k*(z_ref  - z)
	
	
	if (math.fabs(psi_ref-psi_rad)>math.pi):
	    psi_rad = psi_rad + math.copysign(2*math.pi,psi_ref)
	    
	corr_lacet = k*(psi_ref-psi_rad)
	"""
	if flag == 1 :
	    rospy.loginfo(" landing ")
	    publand.publish(Empty())
	  """
        command.linear.x = corr_x
        command.linear.y = corr_y
        command.linear.z = corr_z  

        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = corr_lacet

        # Command sending
	pubCommand.publish(command)
        commandRate.sleep()




