#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define VELOCITY2D_CONTROL 0b101111111000 // 设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
class AirSimControl
{
public:
    AirSimControl();
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
    void control(const ros::TimerEvent &);

    tf::StampedTransform ts;                       // 用来发布无人机当前位置的坐标系坐标轴
    tf::TransformBroadcaster tfBroadcasterPointer; // 广播坐标轴

    unsigned short velocity_mask = VELOCITY2D_CONTROL;

    ros::NodeHandle nh;
    quadrotor_msgs::PositionCommand pos_cmd;
    nav_msgs::Odometry pos_msg;
    double first_pos_roll, first_pos_pitch, first_pos_yaw;
    geometry_msgs::Pose first_pos;

    mavros_msgs::PositionTarget mavros_goal;

    bool goal_received, odom_received, first_pos_flag;
    ros::Subscriber state_sub, cmd_sub, goal_sub, pos_sub;
    ros::Publisher local_pub;
    ros::Timer timer;
};
AirSimControl::AirSimControl()
{
    timer = nh.createTimer(ros::Duration(0.02), &AirSimControl::control, this);
    pos_sub = nh.subscribe("/mavros/local_position/odom", 10, &AirSimControl::odom_cb, this);
    goal_sub = nh.subscribe("move_base_simple/goal", 10, &AirSimControl::goal_cb, this);
    cmd_sub = nh.subscribe("/planning/pos_cmd", 10, &AirSimControl::cmd_cb, this);
    local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    goal_received = false;
    odom_received = false;
    first_pos_flag = false;
}

// read vehicle odometry
void AirSimControl::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    pos_msg = *msg;
    if (!first_pos_flag)
    {
        tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(first_pos_roll, first_pos_pitch, first_pos_yaw);

        first_pos = msg->pose.pose;

        first_pos_flag = true;
    }
    /*
    tf::Vector3 pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_frame";
    ts.setRotation(quat);
    ts.setOrigin(pos);
    tfBroadcasterPointer.sendTransform(ts); */

    if (!odom_received)
    {
        odom_received = true;
    }
}

void AirSimControl::goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (!goal_received)
    {
        goal_received = true;
    }
}

void AirSimControl::cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg)
{
    pos_cmd = *msg;
}

void AirSimControl::control(const ros::TimerEvent &)
{
    if (!odom_received)
    {
        ROS_INFO("[Mavros Control]: Wait for receiving Odom");
        return;
    }
    if (!goal_received)
    {
        mavros_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        mavros_goal.header.stamp = ros::Time::now();
        mavros_goal.type_mask = velocity_mask;
        mavros_goal.position.x = first_pos.position.x;
        mavros_goal.position.y = first_pos.position.y;
        mavros_goal.position.z = first_pos.position.z;

        mavros_goal.yaw = first_pos_yaw;
        ROS_INFO("[Mavros Control 1]: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                 mavros_goal.position.x,
                 mavros_goal.position.y,
                 mavros_goal.position.z,
                 first_pos_yaw);
    }
    else
    {
        mavros_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        mavros_goal.header.stamp = ros::Time::now();
        mavros_goal.type_mask = velocity_mask;
        mavros_goal.position.x = pos_cmd.position.x;
        mavros_goal.position.y = pos_cmd.position.y;
        mavros_goal.position.z = pos_cmd.position.z;
        mavros_goal.yaw = pos_cmd.yaw;
        ROS_INFO("[Mavros Control 2]: x=%.2f, y=%.2f, z=%.2f, yaw=%.2f",
                 pos_cmd.position.x,
                 pos_cmd.position.y,
                 pos_cmd.position.z,
                 pos_cmd.yaw);
    }
    local_pub.publish(mavros_goal);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "airsim_ctrl");
    setlocale(LC_ALL, "");
    AirSimControl ctrl;
    ros::spin();
    return 0;
}