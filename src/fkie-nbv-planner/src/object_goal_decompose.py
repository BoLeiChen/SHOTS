#!/usr/bin/env python2
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import PointStamped
from gazebo_msgs.msg import ModelStates

from arm_controller.srv import ArmControl, ArmControlRequest, ArmControlResponse
from arm_controller.srv import GripperControl, GripperControlRequest, GripperControlResponse

from time import sleep
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import UInt16
from actionlib.action_client import GoalManager
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from fkie_nbv_planner.msg import NbvPlannerAction,NbvPlannerGoal
import tf
from tf_conversions import transformations
import numpy as np
from fkie_nbv_planner.srv import SetObjPose, SetObjPoseRequest, SetObjPoseResponse

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

    
class Goal_Decompose():
    def __init__(self):
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        self.pub = rospy.Publisher('/way_point', PointStamped, queue_size=5)
        self.obj_pose_srv = rospy.Service('set_obj_pose', SetObjPose, self.do_set_obj_pose)
        
        self.arm_client = rospy.ServiceProxy('ArmControl', ArmControl)
        self.arm_client.wait_for_service()
        
        self.gripper_client = rospy.ServiceProxy('GripperControl', GripperControl)
        self.gripper_client.wait_for_service()
        
        self.nbv_rrt_client = actionlib.SimpleActionClient('husky/nbv_rrt', NbvPlannerAction)
        self.nbv_rrt_client.wait_for_server()
        
        self.tf_listener = tf.TransformListener()    
        # try:
        #     self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))    
        # except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        #     return
        
        self.obj_pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.obj_ground_truth_pose_callback)
        
        # obj_pose = Pose() # rpy -0.000035, -0.000113 -0.804678 
        # obj_pose.position.x = 2.019109
        # obj_pose.position.y = 0.016416
        # obj_pose.position.z = 0.459063
        
        self.obj_pose = Pose()
        self.found = False
        # self.obj_pose = obj_pose
    
    def obj_ground_truth_pose_callback(self, data):
        self.obj_pose = data.pose[-2]
        
    def do_set_obj_pose(self, req):
        self.found = req.found
        self.obj_pose = req.obj_pose
        resp = SetObjPoseResponse()
        return resp
    
    def obj_pose_callback(self, data):
        self.found = True
        self.obj_pose = data
        
    def test(self, robotpos, camerapos):
        self.arm_return_normal()
        self.arm_quick_view(camerapos)
        
    def transfer_normal(self, arm_pose):

        (r, p, y) = tf.transformations.euler_from_quaternion([arm_pose.orientation.x, arm_pose.orientation.y, arm_pose.orientation.z, arm_pose.orientation.w])
        p=p+0.5*3.14
        q = tf.transformations.quaternion_from_euler(r,p,y)
        arm_pose.orientation.x = q[0]
        arm_pose.orientation.y = q[1]
        arm_pose.orientation.z = q[2]
        arm_pose.orientation.w = q[3]
        return arm_pose
    
    def feedback_cb(self, feedback):
        pass
    
    def startNBVP(self):
        print("Enter NBVP step, waiting for result.")

        goal = NbvPlannerGoal()


        self.nbv_rrt_client.send_goal(goal, feedback_cb=self.feedback_cb)


        self.nbv_rrt_client.wait_for_result()


        result = self.nbv_rrt_client.get_result()
        print(result)
        return result
    
    def initial_exporation(self, robotpos):
        print("Initializing pre-exxploring.")
        
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0]=1.5
        print(joint_goal)
        self.arm.go(joint_goal,wait=True)
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0]=3.14
        print(joint_goal)
        self.arm.go(joint_goal,wait=True)
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0]=4.6
        print(joint_goal)
        self.arm.go(joint_goal,wait=True)
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0]=6.28
        print(joint_goal)
        self.arm.go(joint_goal,wait=True)
        self.arm_return_normal()
        goal0 = MoveBaseGoal()
        goal0.target_pose.pose.position.x = 1.5
        goal0.target_pose.pose.position.y = 0.0
        goal0.target_pose.pose.orientation.z = 0.0
        goal0.target_pose.pose.orientation.w = 1
        self.husky_move_to_goal(goal0,robotpos)
        self.arm_return_normal()
        
        print("Inilized")
        
    def arm_move_to_goal(self, pose_goal):

        arm_req = ArmControlRequest()

        arm_req.pose = pose_goal
        arm_res = self.arm_client.call(arm_req)
        rospy.loginfo("Arm control success:%s",arm_res.success)
    
    def husky_move_to_goal(self,goal,robotpos):
        waypointMsgs = PointStamped()
        waypointMsgs.header.frame_id = "map"
        waypointMsgs.header.stamp = rospy.Time().now()
        waypointMsgs.point.x = goal.target_pose.pose.position.x
        waypointMsgs.point.y = goal.target_pose.pose.position.y
        waypointMsgs.point.z = goal.target_pose.pose.position.z
        self.pub.publish(waypointMsgs)
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
        
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0]=y
        print(joint_goal)
        self.arm.go(joint_goal,wait=True)
        
    def arm_return_normal(self):

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
        arm_res = self.arm_client.call(arm_req)
        rospy.loginfo("Arm control success:%s",arm_res.success)
        
    def arm_quick_view(self, arm_pose):

        arm_req = ArmControlRequest()
        arm_end_effector_link=self.arm.get_end_effector_link()
        arm_position=self.arm.get_current_pose(arm_end_effector_link)
        arm_rpy=self.arm.get_current_rpy(arm_end_effector_link)
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
        arm_res = self.arm_client.call(arm_req)
        
        q = tf.transformations.quaternion_from_euler(arm_rpy[0],arm_rpy[1]-0.7,arm_rpy[2])
        arm_pose.orientation.x=q[0]
        arm_pose.orientation.y=q[1]
        arm_pose.orientation.z=q[2]
        arm_pose.orientation.w=q[3]
        arm_req.pose = arm_pose
        arm_res = self.arm_client.call(arm_req)
        

        self.arm_return_normal()
        
    def gripper_open(self):
        req = GripperControlRequest()
        req.open = True
        self.gripper_client.call(req)
    
    def gripper_close(self):
        req = GripperControlRequest()
        req.open = False
        self.gripper_client.call(req)
        
    def decompose_base_goal(self, pose):
        base_goal = MoveBaseGoal()
        base_goal.target_pose.pose.position.x = pose.position.x - 0.38
        base_goal.target_pose.pose.position.y = pose.position.y
        base_goal.target_pose.pose.orientation.z = pose.orientation.z
        base_goal.target_pose.pose.orientation.w = pose.orientation.w
        
        return base_goal

    def decompose_arm_goal(self, pose):
        arm_goal=Pose()
        arm_goal.position.x=pose.position.x
        arm_goal.position.y=pose.position.y
        arm_goal.position.z=pose.position.z
        arm_goal.orientation.x=pose.orientation.x
        arm_goal.orientation.y=pose.orientation.y
        arm_goal.orientation.z=pose.orientation.z
        arm_goal.orientation.w=pose.orientation.w

        arm_goal2 = PoseStamped()
        arm_goal2.header.frame_id='/map'
        arm_goal2.header.stamp=rospy.Time(0)
        arm_goal2.pose=arm_goal

        # (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        arm_goal2 = self.tf_listener.transformPose('/base_link', arm_goal2)

        arm_goal = arm_goal2.pose
        # print(arm_goal)
        # print(pose)
        arm_goal.position.x = arm_goal.position.x-0.3
        arm_goal.position.z = arm_goal.position.z+0.05
        # arm_goal.position.z = pose.position.z
        # print(camerapos.get_pos())
        arm_goal=self.transfer_normal(arm_goal)
        return arm_goal
        
    def talker(self):
        
        robotpos=Robot()
        camerapos=Camera()
        step_pub=rospy.Publisher("total_count",UInt16,queue_size=5)
        sleep(2)
        self.initial_exporation(robotpos)
        #test(robotpos,camerapos)
        print("Start NBVP")
        fail_times=0
        step=0
        while(True):
            step_pub.publish(step)
            # self.found = True
            if(self.found):
                base_goal= self.decompose_base_goal(self.obj_pose)
                self.husky_move_to_goal(base_goal, robotpos)
                arm_goal = self.decompose_arm_goal(self.obj_pose)
                self.gripper_open()
                self.arm_move_to_goal(arm_goal)
                arm_goal.position.x = arm_goal.position.x+0.17
                self.arm_move_to_goal(arm_goal)
                self.gripper_close()
                self.arm_return_normal()
            else:
                result=self.startNBVP()
                
                if(result.complete_exploration and fail_times>=1):
                    break
                if(result.complete_exploration and fail_times==0):
                    print("Retrying")
                    fail_times=fail_times+1
                    step=0
                if(result.request_base_pose):
                    self.arm_return_normal()
                    print("Out arm range, trying moving base")
                    goal0 = MoveBaseGoal()
                    goal0.target_pose.pose.position.x = result.goal_pose_3d.pose.position.x
                    goal0.target_pose.pose.position.y = result.goal_pose_3d.pose.position.y
                    goal0.target_pose.pose.orientation.z = result.goal_pose_3d.pose.orientation.z
                    goal0.target_pose.pose.orientation.w = result.goal_pose_3d.pose.orientation.w
                    print(goal0.target_pose.pose)
                    self.husky_move_to_goal(goal0,robotpos)
                    self.arm_quick_view(camerapos)
                    
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
                        arm_pose=self.transfer_normal(arm_pose)
                        print("Sending arm goal")
                        # print(arm_pose)
                        self.arm_move_to_goal(arm_pose)
                        # print(camerapos.get_pos())
            step+=1
    
if __name__ == '__main__':
    # needPub=False
    rospy.init_node('arm', anonymous=True)
    gd = Goal_Decompose()
    try:
        gd.talker()
    except rospy.ROSInterruptException:
        pass
