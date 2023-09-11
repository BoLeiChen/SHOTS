
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加
#include <pcl/PCLPointCloud2.h>

#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/crop_box.h>
 
ros::Publisher pub;
pcl::PointCloud<pcl::PCLPointCloud2> outCloud;
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
 
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
 
  // Perform the actual filtering
  pcl::CropBox<pcl::PCLPointCloud2> corp;
  bool negative=true;
  corp.setInputCloud(cloudPtr);
  corp.setMin(Eigen::Vector4f(-0.4,-0.33,0.0,1.0));
  corp.setMax(Eigen::Vector4f(0.4,0.33,2.0,1.0));
  corp.setNegative(negative);
  corp.filter(cloud_filtered);
  
  cloud_filtered.header.frame_id="base_link";
  cloud_filtered.header.seq=cloud->header.seq;
  cloud_filtered.header.stamp=cloud->header.stamp;
 
  // Convert to ROS data type
  sensor_msgs::PointCloud2 cloud_rad;
  pcl_conversions::moveFromPCL(cloud_filtered, cloud_rad);
 
  // Publish the data
  pub.publish (cloud_rad);
}
 
int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "CorpRemoval");
  ros::NodeHandle nh;
 
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/cloud_input", 1, cloud_cb);
 
  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/points_raw_new", 1);
 
  // Spin
  ros::spin ();
}
