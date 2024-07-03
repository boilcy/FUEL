#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Geometry>

class SensorNode
{
public:
  SensorNode(ros::NodeHandle &nh) : nh_(nh)
  {
    // Get parameters
    nh_.param<double>("publishing_rate", publishing_rate_, 10.0);

    // Set up subscribers
    odom_sub_ = nh_.subscribe("/obs/odom", 10, &SensorNode::odomCallback, this);

    // Set up publishers
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/sensing_node/sensor_pose", 1000);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_, sensor_sub_;
  ros::Publisher sensor_pub_, pose_pub_;
  ros::Timer publish_timer_;

  std::string sensing_type_;
  double publishing_rate_;
  nav_msgs::Odometry latest_odom_;
  sensor_msgs::PointCloud2 latest_lidar_;
  sensor_msgs::Image latest_depth_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    latest_odom_ = *msg;
  }

  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
  {
    latest_lidar_ = *msg;
  }

  void depthCallback(const sensor_msgs::Image::ConstPtr &msg)
  {
    latest_depth_ = *msg;
  }

  void publishCallback(const ros::TimerEvent &)
  {
    publishSensorPose();
    publishSensorData();
  }

  void publishSensorPose()
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = latest_odom_.header;
    pose_msg.pose = latest_odom_.pose.pose;
    pose_pub_.publish(pose_msg);
  }

  void publishSensorData()
  {
    if (sensing_type_ == "lidar")
    {
      sensor_pub_.publish(latest_lidar_);
    }
    else if (sensing_type_ == "depth")
    {
      sensor_pub_.publish(latest_depth_);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensor_node");
  ros::NodeHandle nh("~");
  SensorNode node(nh);
  ros::spin();
  return 0;
}