#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

ros::Publisher pub_cloud, pub_pose;

sensor_msgs::PointCloud2 local_map_pcl;
sensor_msgs::PointCloud2 local_depth_pcl;

ros::Subscriber odom_sub;
ros::Subscriber global_map_sub, local_map_sub, pointcloud_sub;

ros::Timer local_sensing_timer, pose_timer;

bool has_global_map(false);
bool has_local_map(false);
bool has_odom(false);

nav_msgs::Odometry odom_;
Eigen::Matrix4d sensor2body, sensor2world;

double sensing_horizon, sensing_rate, estimation_rate;
double x_size, y_size, z_size;
double gl_xl, gl_yl, gl_zl;
double resolution, inv_resolution;
int GLX_SIZE, GLY_SIZE, GLZ_SIZE;

ros::Time last_odom_stamp = ros::TIME_MAX;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index)
{
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
  pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
  pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt)
{
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);

  return idx;
};

void rcvOdometryCallback(const nav_msgs::Odometry& odom)
{
  has_odom = true;
  odom_ = odom;

  Matrix4d body2world = Matrix4d::Identity();

  Eigen::Vector3d body_pos(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  Eigen::Quaterniond body_ori(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);

  body2world.block<3, 3>(0, 0) = body_ori.toRotationMatrix();
  body2world.block<3, 1>(0, 3) = body_pos;

  sensor2world = body2world * sensor2body;

  Eigen::Quaterniond q(sensor2world.block<3, 3>(0, 0));

  geometry_msgs::PoseStamped sensor_pose;
  sensor_pose.header = odom_.header;
  sensor_pose.header.frame_id = "/map";
  sensor_pose.pose.position.x = sensor2world(0, 3);
  sensor_pose.pose.position.y = sensor2world(1, 3);
  sensor_pose.pose.position.z = sensor2world(2, 3);
  sensor_pose.pose.orientation.w = q.w();
  sensor_pose.pose.orientation.x = q.x();
  sensor_pose.pose.orientation.y = q.y();
  sensor_pose.pose.orientation.z = q.z();
  pub_pose.publish(sensor_pose);
}

void rcvPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  pub_cloud.publish(*cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_render");
  ros::NodeHandle nh("~");

  nh.getParam("sensing_horizon", sensing_horizon);
  nh.getParam("sensing_rate", sensing_rate);
  nh.getParam("estimation_rate", estimation_rate);

  nh.getParam("map/x_size", x_size);
  nh.getParam("map/y_size", y_size);
  nh.getParam("map/z_size", z_size);

  // Subscribe to the point cloud topic
  pointcloud_sub = nh.subscribe("/obs/pcl", 1, rcvPointCloudCallback);

  // Subscribe to the odom topic
  odom_sub = nh.subscribe("/obs/odom", 50, rcvOdometryCallback);

  // Publisher for point cloud and sensor pose
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/cloud", 10);
  pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pcl_render_node/sensor_pose", 10);

  inv_resolution = 1.0 / resolution;
  gl_xl = -x_size / 2.0;
  gl_yl = -y_size / 2.0;
  gl_zl = 0.0;
  GLX_SIZE = (int)(x_size * inv_resolution);
  GLY_SIZE = (int)(y_size * inv_resolution);
  GLZ_SIZE = (int)(z_size * inv_resolution);

  sensor2body << 0.0, 0.0, 1.0, 0.0,
                 -1.0, 0.0, 0.0, 0.0,
                 0.0, -1.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 1.0;

  ros::Rate rate(100);
  bool status = ros::ok();
  while (status)
  {
    ros::spinOnce();
    status = ros::ok();
    rate.sleep();
  }
}
