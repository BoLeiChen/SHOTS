#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

from arm_controller.srv import ArmControl, ArmControlRequest, ArmControlResponse
from arm_controller.srv import GripperControl, GripperControlRequest, GripperControlResponse

from time import sleep
import tf
from tf_conversions import transformations

class Robot:
    def __init__(self):
        self.tf_listener = tf.TransformListener()    
        try:
            self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))    
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

 
    def get_pos(self):
        
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))

        return(trans)
    
class Camera:
    def __init__(self):
        self.tf_listener = tf.TransformListener()    
        try:
            self.tf_listener.waitForTransform('/map', '/camera_link', rospy.Time(), rospy.Duration(1.0))    

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return

 
    def get_pos(self):
        
        (trans, rot) = self.tf_listener.lookupTransform('/map', '/camera_link', rospy.Time(0))

        return(trans)

def talker():
    rospy.init_node('count', anonymous=True)
    sleep(10)
    robotpos=Robot()
    camerapos=Camera()
    robotlenth=0.0
    cameralenth=0.0
    robot_pub=rospy.Publisher("robot_length",Float32,queue_size=5)
    camera_pub=rospy.Publisher("camera_length",Float32,queue_size=5)
    sleep(2)
    robot_pospre=robotpos.get_pos()
    camera_pospre=camerapos.get_pos()
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        robot_posnow=robotpos.get_pos()
        camera_posnow=camerapos.get_pos()
        robotlenthnew=(robot_posnow[0]-robot_pospre[0])**2+(robot_posnow[1]-robot_pospre[1])
        if robotlenthnew<0.0001:
            robotlenthnew=0
        cameralenthnew=(camera_posnow[0]-camera_pospre[0])**2+(camera_posnow[1]-camera_pospre[1])
        if cameralenthnew<0.0001:
            cameralenthnew=0
        robot_pospre=robot_posnow
        camera_pospre=camera_posnow
        robotlenth+=robotlenthnew**0.5
        cameralenth+=cameralenthnew**0.5
        robot_pub.publish(float(robotlenth))
        camera_pub.publish(float(cameralenth))
        rate.sleep()
        
    



if __name__ == '__main__':
    needPub=False
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
