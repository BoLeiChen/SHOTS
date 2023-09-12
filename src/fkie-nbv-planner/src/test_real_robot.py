#!/usr/bin/env python2
import rospy
import math
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from arm_controller.srv import ArmControl, ArmControlRequest, ArmControlResponse
from arm_controller.srv import GripperControl, GripperControlRequest, GripperControlResponse

from time import sleep
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import UInt16, Float64
from gazebo_msgs.msg import ModelStates
from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fkie_nbv_planner.msg import NbvPlannerAction,NbvPlannerGoal
import tf
from tf_conversions import transformations
import numpy as np

arm = moveit_commander.MoveGroupCommander('manipulator')

def arm_return_normal():
    arm_client = rospy.ServiceProxy('ArmControl', ArmControl)
    arm_client.wait_for_service()

    arm_req = ArmControlRequest()

    arm_pose=Pose()
    arm_pose.position.x=0.2
    arm_pose.position.y=0.0
    arm_pose.position.z=1.0
    q = tf.transformations.quaternion_from_euler(0,1.7,0)
    arm_pose.orientation.x=q[0]
    arm_pose.orientation.y=q[1]
    arm_pose.orientation.z=q[2]
    arm_pose.orientation.w=q[3]
    

    arm_req.pose = arm_pose
    arm_res = arm_client.call(arm_req)
    rospy.loginfo("Arm control success:%s",arm_res.success)
    
def ur5_home():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]=-2.35
    joint_goal[1]=-1.57
    joint_goal[2]=1.57
    joint_goal[3]=-2.96
    joint_goal[4]=-1.57
    joint_goal[5]=0
    print(joint_goal)
    arm.go(joint_goal,wait=True)
    
def bigarm_home():
    joint_goal = arm.get_current_joint_values()
    joint_goal[1]=-1.57
    joint_goal[2]=1.57
    joint_goal[3]=-2.96
    arm.go(joint_goal,wait=True)
    
def ur5_home_left_30():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]= -1.832
    arm.go(joint_goal,wait=True)
    joint_goal[1]= -1.047
    joint_goal[2]= 1.047
    joint_goal[3]= -2.704
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[3]= -2.704
    # arm.go(joint_goal,wait=True)
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[3]= -2.442
    # arm.go(joint_goal,wait=True)
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[3]= -2.966
    # arm.go(joint_goal,wait=True)
    # joint_goal[4]= 
    # joint_goal[5]= 
    # print(joint_goal)
    arm.go(joint_goal,wait=True)
    bigarm_home()
    
def ur5_home_left_60():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]= -1.308
    arm.go(joint_goal,wait=True)
    joint_goal[1]= -1.047
    joint_goal[2]= 1.047
    joint_goal[3]= -2.704
    arm.go(joint_goal,wait=True)
    bigarm_home()
    
def ur5_home_left_90():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]= -0.785
    arm.go(joint_goal,wait=True)
    joint_goal[1]= -1.047
    joint_goal[2]= 1.047
    joint_goal[3]= -2.704
    arm.go(joint_goal,wait=True)
    bigarm_home()
    
def ur5_home_right_30():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]= -2.878
    arm.go(joint_goal,wait=True)
    joint_goal[1]= -1.047
    joint_goal[2]= 1.047
    joint_goal[3]= -2.704
    arm.go(joint_goal,wait=True)
    bigarm_home()
    
def ur5_home_right_60():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]= -3.402
    arm.go(joint_goal,wait=True)
    joint_goal[1]= -1.047
    joint_goal[2]= 1.047
    joint_goal[3]= -2.704
    arm.go(joint_goal,wait=True)
    bigarm_home()
    
def ur5_home_right_90():
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]= -3.925
    arm.go(joint_goal,wait=True)
    joint_goal[1]= -1.047
    joint_goal[2]= 1.047
    joint_goal[3]= -2.704
    arm.go(joint_goal,wait=True)
    bigarm_home()

def talker():
    rospy.init_node('arm', anonymous=True)
    # arm_return_normal()
    ur5_home()
    ur5_home_left_30()
    # ur5_home_left_60()
    # ur5_home_left_90()
    # ur5_home_right_30()
    # ur5_home_right_60()
    # ur5_home_right_90()
    ur5_home()
    sleep(2)



if __name__ == '__main__':
    needPub=False
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
