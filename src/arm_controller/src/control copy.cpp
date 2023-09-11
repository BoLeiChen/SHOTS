#include <ros/ros.h>
#include <ros/network.h>
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ExecuteTrajectoryActionGoal.h>
#include <moveit_msgs/MoveGroupActionGoal.h>

#include <arm_controller/ArmControl.h>
#include <arm_controller/GripperControl.h>



moveit::planning_interface::MoveGroupInterface* move_group_;
bool setTaskSpacePath(geometry_msgs::Pose target_pose, double path_time);
bool setToolControl(std::vector<double> joint_angle);

// service
bool doArmControl(arm_controller::ArmControl::Request  &req, arm_controller::ArmControl::Response &res);

// bool pubControl = false;


int main(int argc, char **argv){ 
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1); 
  spinner.start();

  std::string planning_group_name = "manipulator";
  move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);
  move_group_->setNumPlanningAttempts(10);

  ros::ServiceServer arm_control_srv = nh.advertiseService("ArmControl", doArmControl);

  // geometry_msgs::Pose target_pose;
  // target_pose.position.x=0.2;
  // target_pose.position.y=0.1;
  // target_pose.position.z=0.3;
  // setTaskSpacePath(target_pose,2);
  //  gripper_close();

  ROS_INFO("Ready to receive ArmControl.");
  ros::waitForShutdown();
  // ros::spin();
  return 0;
}

bool doArmControl(arm_controller::ArmControl::Request  &req, arm_controller::ArmControl::Response &res)
{
  ROS_INFO("I heard: [%f]", req.pose.position.x);
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
  res.success = setTaskSpacePath(req.pose,2);
  return true;
}

bool setTaskSpacePath(geometry_msgs::Pose target_pose0, double path_time)
{
  ros::AsyncSpinner spinner(1);
  spinner.start();

/*
  moveit_msgs::OrientationConstraint oc;
  oc.link_name = "end_effector_link";
  oc.header.frame_id = "link1";
  oc.orientation.w = 1.0;
  oc.absolute_x_axis_tolerance = 0.1;
  oc.absolute_y_axis_tolerance = 0.1;
  oc.absolute_z_axis_tolerance = 3.14;
  oc.weight = 1.0;
  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(oc);
  move_group_->setPathConstraints(constraints);
  */

  geometry_msgs::Pose target_pose;
  target_pose.position.x = target_pose0.position.x;
  target_pose.position.y = target_pose0.position.y;
  target_pose.position.z = target_pose0.position.z;
  target_pose.orientation.x=target_pose0.orientation.x;
  target_pose.orientation.y=target_pose0.orientation.y;
  target_pose.orientation.z=target_pose0.orientation.z;
  target_pose.orientation.w=target_pose0.orientation.w;
  move_group_->setPoseTarget(target_pose); // Cannot use setPoseTarget as the robot has only 4DOF.
  //move_group_->setPositionTarget(target_pose.position.x,target_pose.position.y,target_pose.position.z);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success == false)
    return false;

  move_group_->execute(my_plan);

  spinner.stop();
  // ros::spinOnce();
  return true;
}
