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

pub = rospy.Publisher('/way_point', PointStamped, queue_size=5)

obj_truth_pose = Pose()

def obj_ground_truth_pose_callback(data):
    global obj_truth_pose
    obj_truth_pose = data.pose[-2] 

obj_pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, obj_ground_truth_pose_callback)
step = 0
success = False
success_thr = 0.045
search_mode = 0


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
    
rospy.init_node('arm', anonymous=True)    
step_pub=rospy.Publisher("total_count",UInt16,queue_size=5)
step = 0
def pub_data(event):
    global step
    step_pub.publish(step)
    step += 1
rospy.Timer(rospy.Duration(0.5), pub_data)
robotpos = Robot()
camerapos = Camera()

def husky_move_to_goal(goal,robotpos):
    waypointMsgs = PointStamped()
    waypointMsgs.header.frame_id = "map"
    waypointMsgs.header.stamp = rospy.Time().now()
    waypointMsgs.point.x = goal.target_pose.pose.position.x
    waypointMsgs.point.y = goal.target_pose.pose.position.y
    waypointMsgs.point.z = goal.target_pose.pose.position.z
    pub.publish(waypointMsgs)
    print("Published waypoint")
    try_times=0
    nowpose=robotpos.get_pos()
    fail_times=0
    
    while(True):
        nowpose=robotpos.get_pos()
        fail_times=fail_times+1
        if((nowpose[0]-goal.target_pose.pose.position.x)**2+(nowpose[1]-goal.target_pose.pose.position.y)**2<=0.25):
            try_times=try_times+1
        else:
            try_times=0
        if(try_times>10):
            break
        if(fail_times>=200):
            waypointMsgs.point.x = nowpose[0]
            waypointMsgs.point.y = nowpose[1]
            waypointMsgs.point.z = nowpose[2]
            
            break
        sleep(0.1)
        
    print("Start spin arm")
    (r, p, y) = tf.transformations.euler_from_quaternion([goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w])
    
    joint_goal = arm.get_current_joint_values()
    joint_goal[0]=y
    print(joint_goal)
    arm.go(joint_goal,wait=True)
    
    
def maxconfpoint_callback(data):
    global step
    print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    search_mode = 1
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = data.x
    goal.target_pose.pose.position.y = data.y
    goal.target_pose.pose.orientation.z = data.z
    goal.target_pose.pose.orientation.w = 1
    arm_return_normal()
    husky_move_to_goal(goal, robotpos)
    search_mode = 0
    step += 1
            

maxconfpoint_sub = rospy.Subscriber("/husky/maxconf_point", Point, maxconfpoint_callback) 

def con_callback(data):
    global success
    # print("confidence:" + str(data.data))
    if data.data >= success_thr:
        success = True
    if success:
        print("Robot claim success!!!!!!!")
        rpos = robotpos.get_pos()
        print("human pos:", obj_truth_pose.position.x, obj_truth_pose.position.y)
        print("robot pose", rpos[0], rpos[1])
        dis = math.sqrt((rpos[0]-obj_truth_pose.position.x)**2 + (rpos[1]-obj_truth_pose.position.y)**2)
        print("distance:", dis)
        if dis <= 2.0:
            print("Real success!!!")
        else:
            print("Fake success!!!")
            
confidence_sub = rospy.Subscriber('/husky/confidence_sum', Float64, con_callback)

def talker():
    global step
    sleep(2)
    initial_exporation(robotpos)
    #test(robotpos,camerapos)
    print("Start NBVP")
    fail_times=0
    while(True):
        if search_mode == 0:
            step_pub.publish(step)
            result=startNBVP()
            
            # if(result.complete_exploration and fail_times>=1):
            #     break
            if(result.complete_exploration and fail_times==0):
                print("Retrying")
                fail_times=fail_times+1
                step=0
            if(result.request_base_pose):
                arm_return_normal()
                print("Out arm range, trying moving base")
                goal0 = MoveBaseGoal()
                goal0.target_pose.pose.position.x = result.goal_pose_3d.pose.position.x
                goal0.target_pose.pose.position.y = result.goal_pose_3d.pose.position.y
                goal0.target_pose.pose.orientation.z = result.goal_pose_3d.pose.orientation.z
                goal0.target_pose.pose.orientation.w = result.goal_pose_3d.pose.orientation.w
                print(goal0.target_pose.pose)
                husky_move_to_goal(goal0,robotpos)
                # arm_quick_view(camerapos)
            
            else:
                for v in result.goals:
                    print("Received goal. Transfering world to base_link")
                    print(v)
                    print("Within arm range, trying moving arm")
                    now_pos=robotpos.get_pos()
                    arm_pose=Pose()
                    arm_pose.position.x=v.pose.position.x - now_pos[0]
                    arm_pose.position.y=v.pose.position.y - now_pos[1]
                    arm_pose.position.z=v.pose.position.z
                    arm_pose.orientation.x=v.pose.orientation.x
                    arm_pose.orientation.y=v.pose.orientation.y
                    arm_pose.orientation.z=v.pose.orientation.z
                    arm_pose.orientation.w=v.pose.orientation.w
                    arm_pose=transfer_normal(arm_pose)
                    print("Sending arm goal")
                    print(arm_pose)
                    arm_move_to_goal(arm_pose)
                    print(camerapos.get_pos())
            step+=1
         
        
        
                    

def test(robotpos,camerapos):
    arm_return_normal()
    arm_quick_view(camerapos)

def transfer_normal(arm_pose):

    (r, p, y) = tf.transformations.euler_from_quaternion([arm_pose.orientation.x, arm_pose.orientation.y, arm_pose.orientation.z, arm_pose.orientation.w])
    p=p+0.5*3.14
    q = tf.transformations.quaternion_from_euler(r,p,y)
    arm_pose.orientation.x = q[0]
    arm_pose.orientation.y = q[1]
    arm_pose.orientation.z = q[2]
    arm_pose.orientation.w = q[3]
    return arm_pose

def feedback_cb(feedback):
    pass

def startNBVP():
    print("Enter NBVP step, waiting for result.")
    client = actionlib.SimpleActionClient('husky/nbv_rrt', NbvPlannerAction)
    client.wait_for_server()


    goal = NbvPlannerGoal()


    client.send_goal(goal, feedback_cb=feedback_cb)


    client.wait_for_result()


    result = client.get_result()
    
    print(result)
    return result


def initial_exporation(robotpos):
    print("Initializing pre-exxploring.")
    
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[0]=1.5
    # print(joint_goal)
    # arm.go(joint_goal,wait=True)
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[0]=3.14
    # print(joint_goal)
    # arm.go(joint_goal,wait=True)
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[0]=4.6
    # print(joint_goal)
    # arm.go(joint_goal,wait=True)
    # joint_goal = arm.get_current_joint_values()
    # joint_goal[0]=6.28
    # print(joint_goal)
    # arm.go(joint_goal,wait=True)
    # arm_return_normal()
    
    # goal0 = MoveBaseGoal()
    # goal0.target_pose.pose.position.x = 1.5
    # goal0.target_pose.pose.position.y = 0.0
    # goal0.target_pose.pose.orientation.z = 0.0
    # goal0.target_pose.pose.orientation.w = 1
    # husky_move_to_goal(goal0,robotpos)
    arm_return_normal()
    
    print("Inilized")
    

def arm_move_to_goal(pose_goal):
    arm_client = rospy.ServiceProxy('ArmControl', ArmControl)
    arm_client.wait_for_service()

    arm_req = ArmControlRequest()

    arm_req.pose = pose_goal
    arm_res = arm_client.call(arm_req)
    rospy.loginfo("Arm control success:%s",arm_res.success)
   
    

def arm_return_normal():
    arm_client = rospy.ServiceProxy('ArmControl', ArmControl)
    arm_client.wait_for_service()

    arm_req = ArmControlRequest()

    arm_pose=Pose()
    arm_pose.position.x=0.2
    arm_pose.position.y=0.0
    arm_pose.position.z=1.0
    q = tf.transformations.quaternion_from_euler(0,1.7,3.14)
    arm_pose.orientation.x=q[0]
    arm_pose.orientation.y=q[1]
    arm_pose.orientation.z=q[2]
    arm_pose.orientation.w=q[3]
    

    arm_req.pose = arm_pose
    arm_res = arm_client.call(arm_req)
    rospy.loginfo("Arm control success:%s",arm_res.success)

def arm_quick_view(arm_pose):
    arm_client = rospy.ServiceProxy('ArmControl', ArmControl)
    arm_client.wait_for_service()

    arm_req = ArmControlRequest()
    arm_end_effector_link=arm.get_end_effector_link()
    arm_position=arm.get_current_pose(arm_end_effector_link)
    arm_rpy=arm.get_current_rpy(arm_end_effector_link)
    print(arm_position,arm_rpy)
    arm_pose=Pose()
    arm_pose.position.x=arm_position.pose.position.x
    arm_pose.position.y=arm_position.pose.position.y
    arm_pose.position.z=arm_position.pose.position.z
    q = tf.transformations.quaternion_from_euler(arm_rpy[0],arm_rpy[1]+0.7,arm_rpy[2])
    arm_pose.orientation.x=q[0]
    arm_pose.orientation.y=q[1]
    arm_pose.orientation.z=q[2]
    arm_pose.orientation.w=q[3]
    arm_req.pose = arm_pose
    arm_res = arm_client.call(arm_req)
    
    q = tf.transformations.quaternion_from_euler(arm_rpy[0],arm_rpy[1]-0.7,arm_rpy[2])
    arm_pose.orientation.x=q[0]
    arm_pose.orientation.y=q[1]
    arm_pose.orientation.z=q[2]
    arm_pose.orientation.w=q[3]
    arm_req.pose = arm_pose
    arm_res = arm_client.call(arm_req)
    

    arm_return_normal()

if __name__ == '__main__':
    needPub=False
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
