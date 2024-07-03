#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

Eigen::Matrix4d readCam2BodyTransform(const std::string &yaml_file)
{
  Eigen::Matrix4d cam2body = Eigen::Matrix4d::Identity();

  try
  {
    YAML::Node config = YAML::LoadFile(yaml_file);

    // Read translation
    if (config["translation"])
    {
      cam2body(0, 3) = config["translation"]["x"].as<double>();
      cam2body(1, 3) = config["translation"]["y"].as<double>();
      cam2body(2, 3) = config["translation"]["z"].as<double>();
    }

    // Read rotation
    if (config["rotation"]["matrix"])
    {
      const auto &matrix = config["rotation"]["matrix"];
      for (int i = 0; i < 3; ++i)
      {
        for (int j = 0; j < 3; ++j)
        {
          cam2body(i, j) = matrix[i][j].as<double>();
        }
      }
    }
  }
  catch (const YAML::Exception &e)
  {
    std::cerr << "Error reading YAML file: " << e.what() << std::endl;
  }

  return cam2body;
}

class SensorNode
{
public:
  SensorNode(ros::NodeHandle &nh) : nh_(nh)
  {
    // Get parameters
    nh_.param<std::string>("body_odom_topic", body_odom_topic_, "/obs/odom");
    nh_.param<std::string>("publishing_rate", sensor_pose_topic_, "/sensing_node/sensor_pose");
    nh_.param<std::string>("config", config_file_);

    // Set up subscribers
    odom_sub_ = nh_.subscribe(body_odom_topic_, 10, &SensorNode::odomCallback, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(sensor_pose_topic_, 1000);

    cam2body_ = readCam2BodyTransform(config_file_);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher pose_pub_;

  std::string config_file_;
  std::string body_odom_topic_, sensor_pose_topic_;

  Eigen::Matrix4d cam2body_;

  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
  {
    Eigen::Quaterniond request_pose(odom->pose.pose.orientation.w,
                                    odom->pose.pose.orientation.x,
                                    odom->pose.pose.orientation.y,
                                    odom->pose.pose.orientation.z);

    Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
    body2world.block<3, 3>(0, 0) = request_pose.toRotationMatrix();
    body2world.block<3, 1>(0, 3) = Eigen::Vector3d(odom->pose.pose.position.x,
                                                   odom->pose.pose.position.y,
                                                   odom->pose.pose.position.z);

    Eigen::Matrix4d cam2world = body2world * cam2body_;
    Eigen::Quaterniond cam2world_quat(cam2world.block<3, 3>(0, 0));

    geometry_msgs::PoseStamped camera_pose;
    camera_pose.header = odom->header;
    camera_pose.header.frame_id = "/map";
    camera_pose.pose.position.x = cam2world(0, 3);
    camera_pose.pose.position.y = cam2world(1, 3);
    camera_pose.pose.position.z = cam2world(2, 3);
    camera_pose.pose.orientation.w = cam2world_quat.w();
    camera_pose.pose.orientation.x = cam2world_quat.x();
    camera_pose.pose.orientation.y = cam2world_quat.y();
    camera_pose.pose.orientation.z = cam2world_quat.z();

    pose_pub_.publish(camera_pose);
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