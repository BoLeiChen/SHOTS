#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import copy
import rospy
from time import sleep
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi  
import numpy as np
import tf
import math

# open [9.470167157399345e-05, -9.470167157399345e-05, 9.470167157399345e-05, 9.470167157399345e-05, -9.470167157399345e-05, 9.470167157399345e-05]
# close [0.39997525501279085, -0.39997525501279085, 0.39997525501279085, 0.39997525501279085, -0.39997525501279085, 0.39997525501279085]

gripper = moveit_commander.MoveGroupCommander('gripper')
gripper_goal = gripper.get_current_joint_values() 
# print(gripper_goal)
# gripper_goal[2] = 9.470167157399345e-05
gripper_goal[2] = 0.39997525501279085
gripper.go(gripper_goal,wait=True)
print("close gripper")