#!/usr/bin/env python2

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

from arm_controller.srv import ArmControl, ArmControlRequest, ArmControlResponse
from arm_controller.srv import GripperControl, GripperControlRequest, GripperControlResponse

from time import sleep


def talker():
    rospy.init_node('arm', anonymous=True)
    arm_client = rospy.ServiceProxy('ArmControl', ArmControl)
    print('111')
    gripper_client = rospy.ServiceProxy('GripperControl', GripperControl)
    arm_client.wait_for_service()
    print('Wariting for gripper service ...')
    gripper_client.wait_for_service()
    print('Gripper service success')

    arm_req = ArmControlRequest()
    print('1')
    # 0.022510,0.054307,0.369000
    arm_pose=Pose()
    arm_pose.position.x=0.600
    arm_pose.position.y=0.0
    arm_pose.position.z=1.0
    arm_pose.orientation.x=0
    arm_pose.orientation.y=0.707
    arm_pose.orientation.z=0
    arm_pose.orientation.w=0.707
    print('2')
    arm_req.pose = arm_pose
    arm_res = arm_client.call(arm_req)
    
    open = False
    gripper_open = GripperControlRequest()
    gripper_open.open = open
    print('3')
    gripper_res = gripper_client.call(gripper_open)
    print('4')

    rospy.loginfo("Arm control success:%s",arm_res.success)
    rospy.loginfo("Gripper control success:%s",gripper_res.success)
    sleep(2)



if __name__ == '__main__':
    needPub=False
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
