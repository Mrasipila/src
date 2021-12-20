#!/usr/bin/env python

# *************************
#  (c) S. Bertrand 2020
# *************************

import rospy
#from std_msgs.msg import String
from opencv_apps.msg import FlowArrayStamped, Flow
import numpy as np
from std_msgs.msg import Float32
import math


# variables globales
flow = Flow()


# initialisation du noeud
rospy.init_node('flow_analyzer', anonymous=True)


# fonction de lecture des mesures et d'envoi de la commande
def getFlowData(data):
    
    flow = data.flow
    
    for i in flow : 
      val = math.sqrt(i.velocity.x**2 + i.velocity.y**2)
      if val >= 10 : 
	rospy.loginfo(" movement detected at : (%s,%s)", i.point.x, i.point.y)
	
    


# declaration d'un subscriber : appelle getFlowData a chq arrivee d'une donnee sur le topic flows
rospy.Subscriber("/fback_flow/flows", FlowArrayStamped, getFlowData)



# fonction main executee en boucle 
if __name__ == '__main__':
    rospy.spin()