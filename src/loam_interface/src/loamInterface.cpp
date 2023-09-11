#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <geometry_msgs/Pose.h>
#include <message_filters/time_synchronizer.h>
#include <tf/message_filter.h>

#include <pcl/common/transforms.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/integrated_to_init";
string registeredScanTopic = "/velodyne_cloud_registered";
bool flipStateEstimation = true;
bool flipRegisteredScan = true;
bool sendTF = true;
bool reverseTF = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());

nav_msgs::Odometry odomData;
tf::StampedTransform odomTrans;
ros::Publisher *pubOdometryPointer = NULL;
tf::TransformBroadcaster *tfBroadcasterPointer = NULL;
tf::TransformListener *tflistener_ = NULL;

ros::Publisher *pubLaserCloudPointer = NULL;

void odometryHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  odomData = *odom;

  if (flipStateEstimation) {
    tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

    pitch = -pitch;
    yaw = -yaw;

    geoQuat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = odom->pose.pose.position.z;
    odomData.pose.pose.position.y = odom->pose.pose.position.x;
    odomData.pose.pose.position.z = odom->pose.pose.position.y;
  }

  // publish odometry messages
  odomData.header.frame_id = "/odom";
  odomData.child_frame_id = "/base_link";
  pubOdometryPointer->publish(odomData);

  // publish tf messages
  odomTrans.stamp_ = odom->header.stamp;
  odomTrans.frame_id_ = "/odom";
  odomTrans.child_frame_id_ = "/base_link";
  odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
  odomTrans.setOrigin(tf::Vector3(odomData.pose.pose.position.x, odomData.pose.pose.position.y, odomData.pose.pose.position.z));

  if (sendTF) {
    if (!reverseTF) {
      tfBroadcasterPointer->sendTransform(odomTrans);
    } else {
      tfBroadcasterPointer->sendTransform(tf::StampedTransform(odomTrans.inverse(), odom->header.stamp, "/sensor", "/map"));
    }
  }
}

// void laserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudIn)
// {
//   laserCloud->clear();
//   pcl::fromROSMsg(*laserCloudIn, *laserCloud);

//   if (flipRegisteredScan) {
//     int laserCloudSize = laserCloud->points.size();
//     for (int i = 0; i < laserCloudSize; i++) {
//       float temp = laserCloud->points[i].x;
//       laserCloud->points[i].x = laserCloud->points[i].z;
//       laserCloud->points[i].z = laserCloud->points[i].y;
//       laserCloud->points[i].y = temp;
//     }
//   }

//   //publish registered scan messages
//   sensor_msgs::PointCloud2 laserCloud2;
//   pcl::toROSMsg(*laserCloud, laserCloud2);
//   laserCloud2.header.stamp = laserCloudIn->header.stamp;
//   laserCloud2.header.frame_id = "/map";
//   pubLaserCloudPointer->publish(laserCloud2);
// }

class SegMapROSWraper  
{
private:
  ros::NodeHandle m_nh;  
  ros::Publisher m_globalcloudPub;  //发布局部地图点云
  message_filters::Subscriber<sensor_msgs::PointCloud2> *m_pointCloudSub;  //接收点云
  tf::MessageFilter<sensor_msgs::PointCloud2> *m_tfPointCloudSub;  //接收/tf消息的过滤器，应该是接收点云和tf同步化
  tf::TransformListener m_tfListener;  // 转化坐标系

public:
  SegMapROSWraper()
      : m_nh("~")  
  {
      
      m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2>(m_nh,registeredScanTopic, 100);    //接收rosbag中的点云消息
      m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2>(*m_pointCloudSub, m_tfListener, "/map", 100);  //接收tf和点云之后触发接收  world是frameid
      m_tfPointCloudSub->registerCallback(boost::bind(&SegMapROSWraper::insertCloudCallback, this, _1));   //回调函数
      m_globalcloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2, true);   //发布全局地图，用于rviz展示
  }

  ~SegMapROSWraper()
  {
      delete m_pointCloudSub;
      delete m_tfPointCloudSub;
  }
  
  void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud)  //接收到点云和tf之后，根据tf转化，然后回调函数
  {
      pcl::PointCloud<pcl::PointXYZI> pc;
      pcl::PointCloud<pcl::PointXYZI> pc_global;
      pcl::fromROSMsg(*cloud, pc);


      tf::StampedTransform sensorToWorldTf;   //定义存放变换关系的变量
      try
      {
          // 监听两个坐标系之间的变换， 其实就是点云坐标系（什么都行，我们的tf有很多）到世界坐标系
          m_tfListener.lookupTransform("/map", cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);   //需要从cloud->header.frame_id（left_camera）转化到/world
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
          return;
      }

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);   //直接得到矩阵
      pcl::transformPointCloud(pc, pc_global, sensorToWorld);   //得到世界坐标系下的点云
      // std::cout<< sensorToWorld <<std::endl;
      sensor_msgs::PointCloud2 map_cloud;
      pcl::toROSMsg(pc_global, map_cloud);  //搞成消息
      map_cloud.header.stamp = ros::Time::now();
      map_cloud.header.frame_id = "/map"; 
      m_globalcloudPub .publish(map_cloud);  //加上时间戳和frameid发布出来
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "loamInterface");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("registeredScanTopic", registeredScanTopic);
  nhPrivate.getParam("flipStateEstimation", flipStateEstimation);
  nhPrivate.getParam("flipRegisteredScan", flipRegisteredScan);
  nhPrivate.getParam("sendTF", sendTF);
  nhPrivate.getParam("reverseTF", reverseTF);
  SegMapROSWraper  SM;
  ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, odometryHandler);

  //ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> (registeredScanTopic, 5, laserCloudHandler);

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);
  pubOdometryPointer = &pubOdometry;

  tf::TransformBroadcaster tfBroadcaster;
  tfBroadcasterPointer = &tfBroadcaster;

  // tf::TransformListener tf_listener_;
  // tflistener_ = &tf_listener_;

  // ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("/registered_scan", 5);
  // pubLaserCloudPointer = &pubLaserCloud;

  ros::spin();

  return 0;
}
