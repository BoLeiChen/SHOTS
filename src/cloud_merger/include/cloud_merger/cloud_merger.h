#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <iostream>

#define MAX_NSENSORS 8 

namespace CloudMerger{

struct pose
{
	double roll;
	double pitch;
	double yaw;
	double x;
	double y;
	double z;
	//below is not used
	bool isquaternion;
	double qx;
	double qy;
	double qz;
	double qw;
};//struct pose

class InputCloud
{
	public:
		InputCloud(pose p,std::string topic,ros::NodeHandle nh);
		~InputCloud() {}
		void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input);
		Eigen::Matrix4f poseTotfmatrix();
		pcl::PointCloud<pcl::PointXYZ> tfdinCloud;
	private:
		pose ps;
		std::string topic_name;
		std::string frame_id;
		ros::Subscriber sub;
		Eigen::Matrix4f transform;
		pcl::PointCloud<pcl::PointXYZ> inCloud;
};//class InputCloud

class OutputCloud
{
	public:
		OutputCloud(std::string topic,std::string frame,ros::NodeHandle nh);
		~OutputCloud() {}
		pcl::PointCloud<pcl::PointXYZ> outCloud;
		sensor_msgs::PointCloud2 outCloudMsg;
		ros::Publisher pub;
	private:
		std::string topic_name;
		std::string frame_id;
		
};//class OutputCloud

class CloudMerger
{
	public:
		CloudMerger(ros::NodeHandle node, ros::NodeHandle private_nh);
		void mergeNpub();
		~CloudMerger() {}
	private:
		int nsensors;
		InputCloud* inClAry[MAX_NSENSORS];
		OutputCloud* outCl;
};//class CloudMerger


}// namespace of CloudMerger

