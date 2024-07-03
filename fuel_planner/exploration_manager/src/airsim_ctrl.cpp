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
// #define VELOCITY2D_CONTROL 0b101111111000
#define VELOCITY2D_CONTROL 0
class AirSimControl
{
public:
    AirSimControl();
    void state_cb(const mavros_msgs::State::ConstPtr &msg);
    void odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void goal_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr &msg);
    void control(const ros::TimerEvent &);

    void set_offboard();
    void set_arm();

    void check_state();

    unsigned short velocity_mask = VELOCITY2D_CONTROL;

    ros::NodeHandle nh;

    bool goal_received, odom_received;
    ros::Timer timer;
    ros::Subscriber cmd_sub, goal_sub, odom_sub;
    ros::Publisher local_pub;

    nav_msgs::Odometry odom_msg;
    quadrotor_msgs::PositionCommand pos_cmd;
    // first odom infomation
    geometry_msgs::Pose first_pos;
    double first_pos_roll, first_pos_pitch, first_pos_yaw;

    mavros_msgs::PositionTarget mavros_goal;

    // state monitor
    mavros_msgs::State current_state;
    ros::Subscriber state_sub;
    ros::ServiceClient set_mode_client;
    ros::ServiceClient arming_client;

    ros::Time last_request_time;
};
AirSimControl::AirSimControl()
{
    timer = nh.createTimer(ros::Duration(0.05), &AirSimControl::control, this);

    // state monitor
    state_sub = nh.subscribe("mavros/state", 10, &AirSimControl::state_cb, this);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    odom_sub = nh.subscribe("/mavros/local_position/odom", 100, &AirSimControl::odom_cb, this);
    goal_sub = nh.subscribe("move_base_simple/goal", 100, &AirSimControl::goal_cb, this);
    cmd_sub = nh.subscribe("/planning/pos_cmd", 100, &AirSimControl::cmd_cb, this);
    local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 1000);

    goal_received = false;
    odom_received = false;

    last_request_time = ros::Time::now();
}

void AirSimControl::state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}
void AirSimControl::set_offboard()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    set_mode_client.call(offb_set_mode);
    if (offb_set_mode.response.mode_sent)
    {
        ROS_WARN("[Mavros Control]: set mode offboard success");
    }
    else
    {
        ROS_WARN("[Mavros Control]: set mode offboard failed");
    }
}
void AirSimControl::set_arm()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    arming_client.call(arm_cmd);
    if (arm_cmd.response.success)
    {
        ROS_WARN("[Mavros Control]: set arming success");
    }
    else
    {
        ROS_WARN("[Mavros Control]: set arming failed");
    }
}
void AirSimControl::check_state()
{
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request_time > ros::Duration(5.0)))
    {
        set_offboard();
        last_request_time = ros::Time::now();
    }
    else if (!current_state.armed && (ros::Time::now() - last_request_time > ros::Duration(5.0)))
    {
        set_arm();
        last_request_time = ros::Time::now();
    }
}
void AirSimControl::odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_msg = *msg;

    if (!odom_received)
    {
        tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(first_pos_roll, first_pos_pitch, first_pos_yaw);

        first_pos = msg->pose.pose;
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
    check_state();
    if (!goal_received)
    {
        mavros_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        mavros_goal.header.stamp = ros::Time::now();
        mavros_goal.type_mask = velocity_mask;
        mavros_goal.position.x = first_pos.position.x + 1;
        mavros_goal.position.y = first_pos.position.y;
        mavros_goal.position.z = first_pos.position.z < 1.0 ? 1.0 : first_pos.position.z;

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