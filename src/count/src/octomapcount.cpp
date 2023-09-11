#include <ros/ros.h>
#include <mutex>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <std_msgs/Float64.h>

std::shared_ptr<octomap::OcTree> ot_;
std::mutex otMutex_;
ros::Subscriber octomap_sub_;
ros::Publisher voxel_pub_;
double total_voxel=0;

void octomapCallback(const octomap_msgs::Octomap &msg);

int main(int argc, char **argv){ 
  ros::init(argc, argv, "control");
  ros::NodeHandle nh;

  octomap_sub_ = nh.subscribe("/husky/octomap_full", 10,octomapCallback);
  voxel_pub_ = nh.advertise<std_msgs::Float64>("voxel_count", 1000);

  ROS_INFO("OctomapCount:Ready to receive Octomap.");

  ros::spin();

  return 0;
}


void octomapCallback(const octomap_msgs::Octomap &msg)
  {
    octomap::AbstractOcTree *aot = octomap_msgs::msgToMap(msg);
    {
      std::lock_guard<std::mutex> lockGuard(otMutex_);
      octomap::OcTree *ot = (octomap::OcTree *)aot;
      double resolution=ot->getResolution();
      double size=0;
      for(octomap::OcTree::leaf_iterator it = ot->begin_leafs(),
        end=ot->end_leafs(); it!= end; ++it)
      {
        if(it.getX()<=4.925&&it.getX()>=-4.925&&it.getY()<=4.925&&it.getY()>=-4.925&&it.getZ()<=2&&it.getZ()>=0){
          size+=it.getSize()*it.getSize()*it.getSize();
        }
      }
      total_voxel=size;
      std_msgs::Float64 msg;//声明变量
      msg.data = total_voxel;
      voxel_pub_.publish(msg);    // 发布该消息
      if (ot)
      {
        delete ot;
      }
    }
  }
