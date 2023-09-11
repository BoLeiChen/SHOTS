#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{

    double x, y, z;
    double roll, pitch, yaw;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
    tf::Quaternion quat;                                     //定义一个四元数
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat); //取出方向存储于四元数
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    static tf::TransformBroadcaster br;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        //first, we'll publish the transform over tf
    tf::Transform transform;

    transform.setOrigin( tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z) );
    transform.setRotation(quat);
    

    //send the transform
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_link"));

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "gdt_odometry_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber subOdom = n.subscribe("/ground_truth/state", 1000, OdomCallback);
    ros::spin();

}
