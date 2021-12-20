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
x_ref = [1.5, 1.5, 0.5, 0.5, 6.0, 6.0, 6.0, 0.0,0.0]
y_ref = [0, 7.8, 7.8, 7.8, 7.8, 7.8, 7.8, 0.0, 0.0]
z_ref = [1.0, 1.0, 2.5, 0.5, 2.5, 0.5, 0.5, 0.5, 0.0]
psi_ref = [np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2]

# Publisher declaration on the topic of command
pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
publand = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
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


i = 0
corr_x = 0.0
corr_y = 0.0
corr_z = 0.0
corr_lacet = 0.0
timer = None

def next_point(event):
    global corr_x, corr_y, corr_z, corr_lacet, i, k, timer
    rospy.loginfo(" Timer :" + str(event.current_real))
    rospy.loginfo(" i : " + str(i))
    if corr_x/k <= 0.1 and corr_x/k >= -0.1 and i < len(z_ref)-1:
      if corr_y/k <= 0.1 and corr_y/k >= -0.1 :
	if corr_z/k <= 0.1 and corr_z/k >= -0.1 :
	  if corr_lacet/k <= 0.1 and corr_lacet/k >= -0.1 :
	    i+=1
    if i is len(z_ref)-1:
      msg = Empty()
      publand.publish(msg)
      timer.shutdown()


# Main: looping execution
if __name__ == '__main__':
    global x, y, z, vx, vy, psi_deg, x_ref, y_ref, z_ref, k, timer, i, corr_x, corr_y, corr_z, corr_lacet
    
    timer = rospy.Timer(rospy.Duration(2.5), next_point)
    
    psi_rad = 0.0
    while not rospy.is_shutdown():
        rospy.loginfo(" zref=%f m,  z=%f m", z_ref[i], z)
        rospy.loginfo(" yref=%f m,  y=%f m", y_ref[i], y)
        rospy.loginfo(" xref=%f m,  x=%f m", x_ref[i], x)
        rospy.loginfo(" psi-ref=%f m,  psi=%f m", psi_ref[i], psi_rad)
        
        psi_rad = psi_deg * (np.pi/180)
        
	vx = k*(x_ref[i]  - x)
	vy = k*(y_ref[i]  - y)
	corr_x = vx*np.cos(psi_rad)+np.sin(psi_rad)*vy
	corr_y = -vx*np.sin(psi_rad)+np.cos(psi_rad)*vy
	corr_z = k*(z_ref[i]  - z)
	
	
	if (math.fabs(psi_ref[i]-psi_rad)>math.pi):
	    psi_rad = psi_rad + math.copysign(2*math.pi,psi_ref[i])
	    
	corr_lacet = k*(psi_ref[i]-psi_rad)
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




