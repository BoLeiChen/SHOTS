#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
from geometry_msgs.msg import Pose

from arm_controller.srv import ArmControl, ArmControlRequest, ArmControlResponse
from arm_controller.srv import GripperControl, GripperControlRequest, GripperControlResponse

from time import sleep
import tf
from tf_conversions import transformations

robot_length=0
camera_length=0
start_time=0
voxel_count=0
total_space=9.85*9.85*2-1.70-2-0.216-0.14-0.12-9-2-0.039-0.078-0.031-0.03-0.02-0.01-0.004

def robotcallback(data):
    global robot_length
    robot_length=data.data
    
def cameracallback(data):
    global camera_length
    camera_length=data.data
    
def voxelcallback(data):
    global voxel_count
    voxel_count=data.data
    
def totalcallback(data):
    global start_time
    global robot_length
    global camera_length
    global voxel_count
    print(data.data)
    if data.data==0:
        start_time=rospy.get_time()
        print("Start Count")
        robot_length=0
        camera_length=0
        
    if data.data>=1:
        fp=open('/home/ros/ours_ws/real_exp/real1.txt','a+')
        print >> fp,str(data.data)+" "+str(rospy.get_time()-start_time)+" "+str(robot_length)+" "+str(camera_length)+" "+str(voxel_count)+" "+str(voxel_count/total_space)
        fp.close()
        print("------------------------")
        print("Step:",data.data)
        print("total time:",rospy.get_time()-start_time)
        print("robot length:",robot_length)
        print("camera length:",camera_length)
        print("explored space:",voxel_count)
        print("explored rate:",voxel_count/total_space)
        print("------------------------")
        
    
def listener():
 
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('main-count', anonymous=True)
 
    rospy.Subscriber("robot_length", Float32, robotcallback)
    rospy.Subscriber("camera_length", Float32, cameracallback)
    rospy.Subscriber("total_count", UInt16, totalcallback)
    rospy.Subscriber("voxel_count", Float64, voxelcallback)
 
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
    



if __name__ == '__main__':
    needPub=False
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
