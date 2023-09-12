#include <octomap_generator/octomap_generator_ros.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <octomap_msgs/conversions.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include "std_msgs/String.h"
#include <cmath>
#include <sstream>
#include <cstring> // For std::memcpy

using namespace octomap;

OctomapGeneratorNode::OctomapGeneratorNode(ros::NodeHandle& nh): nh_(nh)
{
  nh_.getParam("/octomap/tree_type", tree_type_);
  // Initiate octree
  if(tree_type_ == SEMANTICS_OCTREE_BAYESIAN || tree_type_ == SEMANTICS_OCTREE_MAX)
  {
    if(tree_type_ == SEMANTICS_OCTREE_BAYESIAN)
    {
      ROS_INFO("Semantic octomap generator [bayesian fusion]");
      octomap_generator_ = new OctomapGenerator<PCLSemanticsBayesian, SemanticsOctreeBayesian>();
    }
    else
    {
      ROS_INFO("Semantic octomap generator [max fusion]");
      octomap_generator_ = new OctomapGenerator<PCLSemanticsMax, SemanticsOctreeMax>();
    }
    service_ = nh_.advertiseService("toggle_use_semantic_color", &OctomapGeneratorNode::toggleUseSemanticColor, this);
  }
  else
  {
    ROS_INFO("Color octomap generator");
    octomap_generator_ = new OctomapGenerator<PCLColor, ColorOcTree>();
  }
  reset();
  fullmap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/husky/octomap_full", 1, true);
  binarymap_pub_ = nh_.advertise<octomap_msgs::Octomap>("/husky/octomap_binary", 1, true);
  arouse_sub = nh_.subscribe("arouse_fullmap", 10, &OctomapGeneratorNode::ArouseCallback,this);
  pointcloud_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, pointcloud_topic_, 5);
  tf_pointcloud_sub_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointcloud_sub_, tf_listener_, world_frame_id_, 5);
  tf_pointcloud_sub_->registerCallback(boost::bind(&OctomapGeneratorNode::insertCloudCallback, this, _1));
  octomapFullService = nh_.advertiseService("octomap_full", &OctomapGeneratorNode::octomapFullSrv, this);
  octomapBinaryService = nh_.advertiseService("octomap_binary", &OctomapGeneratorNode::octomapBinarySrv, this);
}

OctomapGeneratorNode::~OctomapGeneratorNode() {}
/// Clear octomap and reset values to paramters from parameter server
void OctomapGeneratorNode::reset()
{
  nh_.getParam("/octomap/pointcloud_topic", pointcloud_topic_);
  nh_.getParam("/octomap/world_frame_id", world_frame_id_);
  nh_.getParam("/octomap/resolution", resolution_);
  nh_.getParam("/octomap/max_range", max_range_);
  nh_.getParam("/octomap/raycast_range", raycast_range_);
  nh_.getParam("/octomap/clamping_thres_min", clamping_thres_min_);
  nh_.getParam("/octomap/clamping_thres_max", clamping_thres_max_);
  nh_.getParam("/octomap/occupancy_thres", occupancy_thres_);
  nh_.getParam("/octomap/prob_hit", prob_hit_);
  nh_.getParam("/octomap/prob_miss", prob_miss_);
  nh_.getParam("/tree_type", tree_type_);
  octomap_generator_->setClampingThresMin(clamping_thres_min_);
  octomap_generator_->setClampingThresMax(clamping_thres_max_);
  octomap_generator_->setResolution(resolution_);
  octomap_generator_->setOccupancyThres(occupancy_thres_);
  octomap_generator_->setProbHit(prob_hit_);
  octomap_generator_->setProbMiss(prob_miss_);
  octomap_generator_->setRayCastRange(raycast_range_);
  octomap_generator_->setMaxRange(max_range_);
}

bool OctomapGeneratorNode::toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  octomap_generator_->setUseSemanticColor(!octomap_generator_->isUseSemanticColor());
  if(octomap_generator_->isUseSemanticColor())
    ROS_INFO("Using semantic color");
  else
    ROS_INFO("Using rgb color");
  if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
     fullmap_pub_.publish(map_msg_);
  else
     ROS_ERROR("Error serializing OctoMap");
  return true;
}

void OctomapGeneratorNode::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  // Voxel filter to down sample the point cloud
  // Create the filtering object
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());

  pcl_conversions::toPCL(*cloud_msg, *cloud);
  // Get tf transform
  tf::StampedTransform sensorToWorldTf;
  try
  {
    tf_listener_.lookupTransform(world_frame_id_, cloud_msg->header.frame_id, cloud_msg->header.stamp, sensorToWorldTf);
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }
  // Transform coordinate
  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  /*********************************************/
  //pcl::PassThrough<pcl::PCLPointCloud2> pass_y;
  //pass_y.setFilterFieldName("y");
  //pass_y.setFilterLimits(-1.50, 0.08); 
  //pass_y.setInputCloud(cloud);
  //pass_y.filter(*cloud);               //对点云z轴方向进行过滤
  /*********************************************/
  octomap_generator_->insertPointCloud(cloud, sensorToWorld);
  // Publish octomap
  map_msg_.header.frame_id = world_frame_id_;
  map_msg_.header.stamp = cloud_msg->header.stamp;
  if (octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), map_msg_))
     //以fullmap的形式发布整个地图
     fullmap_pub_.publish(map_msg_);
  else
     ROS_ERROR("Error serializing Full_OctoMap");

  map_msg_1.header.frame_id = world_frame_id_;
  map_msg_1.header.stamp = cloud_msg->header.stamp;
  if (octomap_msgs::binaryMapToMsg(*octomap_generator_->getOctree(), map_msg_1))
     //以binarymap的形式发布二进制地图
     binarymap_pub_.publish(map_msg_1);
  else
     ROS_ERROR("Error serializing Binary_OctoMap");
}

//用于保存地图全部信息的服务
//由roslaunch octomap_server octomap_saver -f /home/cbl/filename.ot发出请求
bool OctomapGeneratorNode::octomapFullSrv(octomap_msgs::GetOctomap::Request  &req,
                                    octomap_msgs::GetOctomap::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  
  res.map.header.frame_id = world_frame_id_;
  res.map.header.stamp = ros::Time::now();

  //save("/home/cbl/map11.ot");
  if (!octomap_msgs::fullMapToMsg(*octomap_generator_->getOctree(), res.map))
    return false;

  return true;
}
//用于保存二进制地图的服务
//由roslaunch octomap_server octomap_saver /home/cbl/filename.bt发出请求
bool OctomapGeneratorNode::octomapBinarySrv(octomap_msgs::GetOctomap::Request  &req,
                                    octomap_msgs::GetOctomap::Response &res)
{
  ROS_INFO("Sending binary map data on service request");
  
  res.map.header.frame_id = world_frame_id_;
  res.map.header.stamp = ros::Time::now();

  //save("/home/cbl/map11.ot");
  if (!octomap_msgs::binaryMapToMsg(*octomap_generator_->getOctree(), res.map))
    return false;

  return true;
}
//加载.ot地图文件，发布到/octomap_full话题，测试时使用的
/*void OctomapGeneratorNode::ArouseCallback(const std_msgs::String::ConstPtr& msg)
{
  // 将接收到的消息打印出来
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  octomap::OcTree* m_octree=new octomap::OcTree(0.02);
  std::string filename = msg->data;
  std::string suffix = filename.substr(filename.length()-3, 3);
  if(suffix == ".ot")
  {
      octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
      if(!tree)
      {
        ROS_ERROR("Read Full_OctoMap Failed!");
      }  
      if (m_octree){
          delete m_octree;
          m_octree = NULL;
      }
      m_octree = dynamic_cast<octomap::OcTree*>(tree);
      if (!m_octree){
        ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      }
  }
  else
  {
    ROS_ERROR("Filename Error!");
  }
  map_msg_.header.frame_id = world_frame_id_;
  map_msg_.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(*m_octree, map_msg_))
     fullmap_pub_.publish(map_msg_);
  else
     ROS_ERROR("Error arouse Full_OctoMap"); 
}*/
//加载.ot地图文件，发布到/octomap_full话题
void OctomapGeneratorNode::ArouseCallback(const std_msgs::String::ConstPtr& msg)
{
  // 将接收到的消息打印出来
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  std::string filename = msg->data;
  /*if(!read(filename.c_str()))
  {
    ROS_ERROR("Error read Full_OctoMap!");
  }*/
  std::ifstream infile(filename.c_str(), std::ios_base::in |std::ios_base::binary);
  if (!infile.is_open()) 
  {
    std::cout << "file "<< filename << " could not be opened for reading.\n";
  }
  octomap::AbstractOcTree *tree = octomap::AbstractOcTree::read(infile);
  map_msg_.header.frame_id = world_frame_id_;
  map_msg_.header.stamp = ros::Time::now();
  if (octomap_msgs::fullMapToMsg(*tree, map_msg_))
  {
      fullmap_pub_.publish(map_msg_);
      ROS_INFO("Publish fullmap Successful!");
  }
  else
     ROS_ERROR("Error arouse Full_OctoMap"); 
}
bool OctomapGeneratorNode::save(const char* filename)
{
  octomap_generator_->save(filename);
}
bool OctomapGeneratorNode::read(const char* filename)
{
  octomap_generator_->read(filename);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "octomap_generator");
  ros::NodeHandle nh;
  OctomapGeneratorNode octomapGeneratorNode(nh);
  ros::spin();
  /*std::string save_path;
  nh.getParam("/octomap/save_path", save_path);
  octomapGeneratorNode.save(save_path.c_str());
  ROS_INFO("OctoMap saved.");*/
  return 0;
}
