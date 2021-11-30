#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from nav_msgs.msg import Odometry # for receiving odometry feedback

# Global variables 
global z_ref


command = Twist()


global x,y,z,psi,vx,vy
x = 0.0
y = 0.0
z = 0.0
psi_deg = 0.0
vx = 0.0
vy = 0.0


# Node init
rospy.init_node('cmd', anonymous=True)

# reference
# can also be coded as global variable, input topic, value read from textfile, etc.
z_ref = rospy.get_param("~zref",3.5) # m


# Publisher declaration on the topic of command
pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
commandRate = rospy.Rate(10) 



# Measurements reading
def readXYZ(data):
    global x, y, z
    # Data reading X axis
    x = data.pose.pose.position.x 
    # Data reading Y axis
    y = data.pose.pose.position.y    
    # Data reading Z axis
    z = data.pose.pose.position.z    
    
    
    
    
def readPsiVxVy(data):
    global psi_deg, vx, vy
    # Data reading Z axis from ultrasonic sensor
    #z = data.altd / 1000.0   
    # Data reading yaw angle (deg)
    psi_deg = data.rotZ
    vx = data.vx
    vy = data.vy
    

# Subscriber declaration 
rospy.Subscriber("/ground_truth/state", Odometry, readXYZ)
rospy.Subscriber("/ardrone/navdata", Navdata, readPsiVxVy)


# Main: looping execution
if __name__ == '__main__':
    global x, y, z, vx, vy, psi_deg, x_ref, y_ref, z_ref



    while not rospy.is_shutdown():
        rospy.loginfo(" zref=%f m,  z=%f m", z_ref, z)

        command.linear.x = 0.0
        command.linear.y = 0.0
        command.linear.z = 0.0     #  <-----  TO BE MODIFIED

        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = 0.0
    
        # Command sending
        pubCommand.publish(command)
        commandRate.sleep()




