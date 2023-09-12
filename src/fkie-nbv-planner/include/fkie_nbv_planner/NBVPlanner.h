#ifndef RRT_ALGORITHM_H
#define RRT_ALGORITHM_H

#include <iostream>
#include <algorithm>
#include <random>
#include <math.h>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <ros/time.h>
#include <chrono>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include <octomap_msgs/conversions.h>
#include <Eigen/Dense>
#include <fkie_measurement_msgs/BoundaryPolygon.h>

#include <fkie_nbv_planner/NbvPlannerAction.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "fkie_nbv_planner/RRTNode.h"
#include "fkie_nbv_planner/Tree2NanoflannAdapter.hpp"
#include "fkie_nbv_planner/nanoflann.hpp"
#include <geometry_msgs/PolygonStamped.h>

#include "fkie_nbv_planner/NBVParameters.hpp"
#include "fkie_nbv_planner/NBVRviz.h"
#include "fkie_nbv_planner/SparseGrid.hpp"
#include "fkie_nbv_planner/utils.hpp"

#include "PathOptimizer.h"

#include <semantics_octree/semantics_octree.h>
#include <semantics_octree/semantics_bayesian.h>
#include <semantics_octree/semantics_max.h>
#include <semantics_point_type/semantics_point_type.h>
#include <octomap_generator/octomap_generator_base.h>
#include <octomap_generator/octomap_generator.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <octomap/ColorOcTree.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/Pointcloud.h>
#include <octomap/octomap.h>
#include <boost/shared_ptr.hpp>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <string>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>
#include "std_msgs/String.h"
#include <sstream>
#include <cstring> // For std::memcpy


#define COLOR_OCTREE 0
#define SEMANTICS_OCTREE_MAX 1
#define SEMANTICS_OCTREE_BAYESIAN 2

namespace fkie_nbv_planner
{
     typedef SparseGrid<MeasurementValue, IndexGrid, IndexGridHasher> SparseGridMeasurement3D;
     typedef SparseGrid<VisitedValue, IndexGrid2D, IndexGrid2DHasher> SparseGridVisited2D;

     class NBVPlanner
     {
     public:
          NBVPlanner(const ros::NodeHandle &nh);

          /**
           * @brief Reset the trees: [tree_adapter_] and [kdtree_]
           */
          void resetTree();

          /**
           * @brief Subscribe the pose of the camera in world frame
           */
          void cameraPoseCallback(const geometry_msgs::PoseStamped &msg);
          void timerCallback(const ros::TimerEvent& event);

          /**
           * @brief Subscribe the octomap from octomap server
           */
          void octomapCallback(const octomap_msgs::Octomap &msg);

          /**
           * @brief Request octomap from rtabmap services
           */
          void getOctomap();

          /**
           * @brief Subscribe the footprint of the robot to create a self-collision bounding box
           */
          void robotFootprintCallback(const geometry_msgs::PolygonStamped &msg);

          /**
           * @brief Main planner execution action server
           */
          void executePlan(const fkie_nbv_planner::NbvPlannerGoalConstPtr &goal);

          /**
           * @brief Trace the branch from best node to root
           */
          std::vector<std::shared_ptr<RRTNode>> getBestNodeBranch(std::shared_ptr<RRTNode> node);

          /**
           * @brief Calculate the gain of free space based on the volume of unmapped space
           */
          std::pair<double, double> gainCubature(const geometry_msgs::Pose pot_pose) const;

          /**
           * @brief Compute gain for the [node] based on free space and measurement estimation
           */
          void updateGain(std::shared_ptr<RRTNode> node) const;
          void updateGain(RRTNode &node) const;

          /**
           * @brief Compute the yaw of the [node] based on its gain cubature or measurement estimations
           */
          void computeYaw(std::shared_ptr<RRTNode> node);

          void updateGainCachedNodes();
          void removeLowGainCacheNodes();

          /**
           * @brief Extract frontiers based on [cached_nodes] using parameter [num_frontiers]
           * @param frontiers Output vector with frontiers
           * @return true if vector of frontiers is not empty
           */
          [[nodiscard]] bool getFrontiers(std::vector<RRTNode> &frontiers);
          bool findHighUtilityFrontier(const std::vector<RRTNode> &frontiers, RRTNode &best_frontier);

          std::pair<std::vector<bool>, int> expandTreeTowardsFrontiers(std::vector<bool> f_vec, int i, const std::vector<RRTNode> &curr_frontiers);

          [[nodiscard]] bool initializeBoundary(fkie_measurement_msgs::BoundaryPolygon boundary_coords);
          [[nodiscard]] bool initializeDefaultBoundary();

          /**
           * @brief Expand the RRT and provide the NBV branch
           */
          std::vector<geometry_msgs::PoseStamped> getNbvBranch(const std::vector<RRTNode> &curr_frontiers);
          [[nodiscard]] bool octomapReceived();
          void publishBoundaryBBx();

          // ---------------- RRT ----------------
          void cleanCurrentRRT();

          /**
           * @brief Expand the RRT and calculate gain for each node to find the node with best utility
           */
          [[nodiscard]] bool expandRRT(const std::vector<RRTNode> &curr_frontiers);

          [[nodiscard]] bool findClosestNeighbor(const Eigen::Vector3d &sample, std::shared_ptr<RRTNode> &closest_neighbor);
          void findNewRRTNode(const Eigen::Vector3d &origin, const Eigen::Vector3d &point, Eigen::Vector3d &qnew);

          std::shared_ptr<RRTNode> initializeRoot();

          /**
           * @brief Extract goals from best branch and visualise it
           */
          std::vector<geometry_msgs::PoseStamped> extractNBVPoses();

          fkie_nbv_planner::NbvPlannerResult generateGoalsToSubFrontier(fkie_nbv_planner::NbvPlannerResult result);
          [[nodiscard]] bool isSampleCloseToRRTNode(const Eigen::Vector3d &sample, const double &distance) const;

          /**
           * @brief Find all the samples within a given radius
           */
          std::vector<std::shared_ptr<RRTNode>> findSamplesWithinRadius(const double &radius, const Eigen::Vector3d &sample);

          /**
           * @brief Find K-nearest neighbors
           */
          std::vector<std::shared_ptr<RRTNode>> findKNearestNeighbors(const int &k);

          // ----------------  TF ----------------
          geometry_msgs::PoseStamped transformPose(geometry_msgs::PoseStamped input_pose, std::string from_frame, std::string to_frame);

          /**
           * @brief Tests if the point is inside or outside the cylinder
           */
          [[nodiscard]] bool collisionLine(Eigen::Vector3d p1, Eigen::Vector3d p2, double r);

          void rewire(std::vector<std::shared_ptr<RRTNode>> near_nodes, std::shared_ptr<RRTNode> new_node);

          Eigen::Vector3d sampleNewPoint(double radius) const;

//////////////////////////////////////////////////////////////////////////////////

         void reset();
         void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
         void ArouseCallback(const std_msgs::String::ConstPtr& msg);
         bool save(const char* filename) ;
         bool read(const char* filename) ;

       protected:
         OctomapGeneratorBase* octomap_generator_; ///<Octomap instance pointer
         ros::ServiceServer service_;  ///<ROS service to toggle semantic color display
         ros::ServiceServer octomapFullService;
         ros::ServiceServer octomapBinaryService;
         bool toggleUseSemanticColor(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response); ///<Function to toggle whether write semantic color or rgb color as when serializing octree
         bool octomapFullSrv(octomap_msgs::GetOctomap::Request& req, octomap_msgs::GetOctomap::Response& res);
         bool octomapBinarySrv(octomap_msgs::GetOctomap::Request& req,octomap_msgs::GetOctomap::Response& res);
         ros::Publisher fullmap_pub_,binarymap_pub_; ///<ROS publisher for octomap message
         message_filters::Subscriber<sensor_msgs::PointCloud2>* pointcloud_sub_; ///<ROS subscriber for pointcloud message
         tf::MessageFilter<sensor_msgs::PointCloud2>* tf_pointcloud_sub_; ///<ROS tf message filter to sychronize the tf and pointcloud messages
         tf::TransformListener tf_listener_; ///<Listener for the transform between the camera and the world coordinates
         ros::Subscriber arouse_sub;
         std::string world_frame_id_; ///<Id of the world frame
         std::string pointcloud_topic_; ///<Topic name for subscribed pointcloud message
         float max_range_; ///<Max range for points to be inserted into octomap
         float raycast_range_; ///<Max range for points to perform raycasting to free unoccupied space
         float clamping_thres_max_; ///<Upper bound of occupancy probability for a node
         float clamping_thres_min_; ///<Lower bound of occupancy probability for a node
         float resolution_; ///<Resolution of octomap
         float occupancy_thres_; ///<Minimum occupancy probability for a node to be considered as occupied
         float prob_hit_;  ///<Hit probability of sensor
         float prob_miss_; ///<Miss probability of sensor
         int tree_type_; ///<0: color octree, 1: semantic octree using bayesian fusion, 2: semantic octree using max fusion
         octomap_msgs::Octomap map_msg_; ///<ROS octomap message
         octomap_msgs::Octomap map_msg_1;

     private:
          ros::NodeHandle nh_;
          NBVRviz nbv_publisher;
          PathOptimizer p_optimizer;
          SparseGridMeasurement3D measurement_grid = SparseGridMeasurement3D("measurement_grid");
          SparseGridVisited2D visited_grid = SparseGridVisited2D("visited_grid");

          double max_node_score = 0.0;

          //---------------- Planner parameters ----------------
          NBVParameters *params = NULL;

          //---------------- TF ----------------
          tf2_ros::Buffer tfBuffer;
          tf2_ros::TransformListener tfListener;
          //---------------- Camera pose and footprint subscriber ----------------
          bool current_camera_pose_initialized_;
          geometry_msgs::Pose curr_camera_pose_;
          ros::Subscriber camera_pose_sub_;
          ros::Subscriber robot_footprint_sub_;

          //---------------- Action server ----------------
          actionlib::SimpleActionServer<fkie_nbv_planner::NbvPlannerAction> planning_as_;
          //---------------- Counts ----------------
          int node_count;
          int planning_iteration;
          //---------------- Nanoflann KD Tree ----------------
          // construct a kd-tree index:
          typedef nanoflann::KDTreeSingleIndexDynamicAdaptor<
              nanoflann::L2_Simple_Adaptor<double, Tree2NanoflannAdapter>,
              Tree2NanoflannAdapter, 3>
              kd_tree_nano;

          // KD-tree for neighbor search
          std::unique_ptr<kd_tree_nano> kdtree_;
          Tree2NanoflannAdapter tree_adapter_;

          //---------------- Octomap ----------------
          std::shared_ptr<octomap::OcTree> ot_;
          std::mutex otMutex_;
          ros::Subscriber octomap_sub_;

          //---------------- Publishers  ----------------
          ros::Publisher exploration_bbx_pub_;
          ros::Publisher planning_poly_pub_;
          ros::Publisher confidence_pub_;
          ros::Publisher maxconfpose_pub_;

          //---------------- timers  ----------------
          ros::Timer timer_;

          //---------------- Planner variables ----------------
          std::shared_ptr<RRTNode> best_node_;
          RRTNode best_frontier_;
          bool found_best_frontier_;

          std::vector<geometry_msgs::PoseStamped> best_branch_;
          int curr_boundary_id_;
          fkie_measurement_msgs::BoundaryPolygon curr_boundary_;
          fkie_measurement_msgs::BoundaryPolygon roi_boundary_viz;
          geometry_msgs::Polygon curr_robot_footprint;
          visualization_msgs::MarkerArray marker_array_rrt_;
          bool root_initialized_;
          bool boundary_initialized_;
          bool allow_decrementing_i;

          mutable double current_utility_max_value_free_space = 0.0;
          mutable double current_utility_max_value_measurement = 0.0;
          mutable double current_utility_max_value_visited_cell = 0.0;

          // Cache nodes whose gains are calculated explicitly
          std::vector<RRTNode> cached_nodes_;
          double min_gain_ = -1000;
     };
} // namespace fkie_nbv_planner
#endif
