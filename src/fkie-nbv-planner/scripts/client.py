#!/usr/bin/env python2
import rospy
import actionlib
from fkie_nbv_planner.msg import NbvPlannerAction,NbvPlannerGoal

def feedback_cb(feedback):
    pass

if __name__ == '__main__':
    rospy.init_node('my_action_client')
    client = actionlib.SimpleActionClient('husky/nbv_rrt', NbvPlannerAction)
    client.wait_for_server()


    goal = NbvPlannerGoal()


    client.send_goal(goal, feedback_cb=feedback_cb)


    client.wait_for_result()


    result = client.get_result()
    print(result)
