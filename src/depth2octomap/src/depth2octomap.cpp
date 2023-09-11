#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
 
// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
 
// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
 
using namespace std;
using namespace cv;
 
// 相机内参
const double camera_factor = 1000;
const double camera_cx = 319.025;
const double camera_cy = 236.750;
const double camera_fx = 384.657;
const double camera_fy = 384.657;
 
// 全局变量：图像矩阵和点云
cv_bridge::CvImagePtr color_ptr, depth_ptr;
cv::Mat color_pic, depth_pic;
 
/***  RGB处理  ***/
void color_Callback(const sensor_msgs::ImageConstPtr& color_msg)
{
    try
    {
        color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        cv::waitKey(1050); // 不断刷新图像，频率时间为int delay，单位为ms
    }
    catch (cv_bridge::Exception& e )
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", color_msg->encoding.c_str());
    }
    color_pic = color_ptr->image;
}
 
/***  Depth处理  ***/
void depth_Callback(const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        cv::waitKey(1050);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", depth_msg->encoding.c_str());
    }
 
    depth_pic = depth_ptr->image;
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "publish_octomap");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/d435/color/image_raw", 1, color_Callback);
    image_transport::Subscriber sub1 = it.subscribe("/d435/depth/image_raw", 1, depth_Callback);
    ros::Publisher pointcloud_publisher = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/output", 1);
    // 点云变量
    PointCloud::Ptr cloud ( new PointCloud );
    sensor_msgs::PointCloud2 pub_pointcloud;
 
    double sample_rate = 1.0; // 1HZ
    ros::Rate naptime(sample_rate); // use to regulate loop rate
 
    while (ros::ok()) {
        // 遍历深度图
        for (int m = 0; m < depth_pic.rows; m++){
            for (int n = 0; n < depth_pic.cols; n++){
                // 获取深度图中(m,n)处的值
                float d = depth_pic.ptr<float>(m)[n];//ushort d = depth_pic.ptr<ushort>(m)[n];
                // d 可能没有值，若如此，跳过此点
                if (d == 0)
                    continue;
                // d 存在值，则向点云增加一个点
                pcl::PointXYZRGB p;
 
                // 计算这个点的空间坐标
//                p.z = double(d) / camera_factor;
//                p.x = (n - camera_cx) * p.z / camera_fx;
//                p.y = (m - camera_cy) * p.z / camera_fy;
 
                // 相机模型是垂直的
                p.x = double(d) / camera_factor;
                p.y = -(n - camera_cx) * p.x / camera_fx;
                p.z = -(m - camera_cy) * p.x / camera_fy;
 
                // 从rgb图像中获取它的颜色
                // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
                p.b = color_pic.ptr<uchar>(m)[n*3];
                p.g = color_pic.ptr<uchar>(m)[n*3+1];
                p.r = color_pic.ptr<uchar>(m)[n*3+2];
 
                // 把p加入到点云中
                cloud->points.push_back( p );
            }
        }
 
        // 设置并保存点云
        cloud->height = 1;
        cloud->width = cloud->points.size();
        ROS_INFO("point cloud size = %d",cloud->width);
        cloud->is_dense = false;// 转换点云的数据类型
        pcl::toROSMsg(*cloud,pub_pointcloud);
        pub_pointcloud.header.frame_id = "ur5_wrist_3_link";
        pub_pointcloud.header.stamp = ros::Time::now();
        // 发布合成点云
        pointcloud_publisher.publish(pub_pointcloud);
        // 清除数据并退出
        cloud->points.clear();
 
        ros::spinOnce(); //allow data update from callback;
        naptime.sleep(); // wait for remainder of specified period;
    }
}
