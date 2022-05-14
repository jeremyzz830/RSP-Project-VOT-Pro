#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

int sequence = 0;
ros::Publisher goal_pub;
double ori_z, ori_w;
geometry_msgs::PoseStamped pose_stamped_msg;

void waypointCB(const geometry_msgs::Point msg)
{
    double x = msg.x;
    double y = msg.y;
    double z = msg.z;

    pose_stamped_msg.header.seq = sequence;
    pose_stamped_msg.header.stamp = ros::Time::now();
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose.position.x = x;
    pose_stamped_msg.pose.position.y = y;
    pose_stamped_msg.pose.position.z = z;
    pose_stamped_msg.pose.orientation.x = 0;
    pose_stamped_msg.pose.orientation.y = 0;
    pose_stamped_msg.pose.orientation.z = ori_z;
    pose_stamped_msg.pose.orientation.w = ori_w;

    goal_pub.publish(pose_stamped_msg);
    sequence++;
}

void odomCB(const nav_msgs::Odometry msg)
{
    ori_z = msg.pose.pose.orientation.z;
    ori_w = msg.pose.pose.orientation.w;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "robot_nav");
    ros::NodeHandle nh;

    ros::Publisher InitPub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 5);

    ros::Subscriber OdomSub = nh.subscribe("/odom", 10, odomCB);

    ros::service::waitForService("amcl/set_parameters");
    geometry_msgs::PoseWithCovarianceStamped init_pose_cov_stamp;
    init_pose_cov_stamp.header.seq = 1;
    init_pose_cov_stamp.header.stamp = ros::Time::now();
    init_pose_cov_stamp.header.frame_id = "map";

    double init_x, init_y, init_z, init_w;

    if (nh.getParam("/p_x", init_x))
    {
        init_pose_cov_stamp.pose.pose.position.x = init_x;
    }

    if (nh.getParam("/p_y", init_y))
    {
        init_pose_cov_stamp.pose.pose.position.y = init_y;
    }
    if (nh.getParam("/q_z", init_z))
    {
        init_pose_cov_stamp.pose.pose.orientation.z = init_z;
    }
    if (nh.getParam("/q_w", init_w))
    {
        init_pose_cov_stamp.pose.pose.orientation.w = init_w;
    }

    init_pose_cov_stamp.pose.pose.position.z = 0;

    init_pose_cov_stamp.pose.pose.orientation.x = 0;
    init_pose_cov_stamp.pose.pose.orientation.y = 0;

    for (int i = 0; i < 36; i++)
    {
        init_pose_cov_stamp.pose.covariance[i] = 0.0;
        if (i % 7 == 0)
        {
            init_pose_cov_stamp.pose.covariance[i] = 0.01;
        }
    }

    ros::Duration(1).sleep();
    InitPub.publish(init_pose_cov_stamp);

    ros::Duration(20).sleep();
    nh.setParam("/stop_auto_move", 1);

    nh.setParam("/move_base/DWAPlannerROS/max_vel_trans", 0.15);
    nh.setParam("/move_base/DWAPlannerROS/min_vel_theta", 0.2);
    nh.setParam("/move_base/DWAPlannerROS/max_vel_theta", 0.4);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 5);
    // ros::Subscriber CircleSub = nh.subscribe("ball_position", 10, waypointCB);

    while (nh.ok())
    {

        ros::Duration(5).sleep();
        ros::spinOnce();
    }

    return 0;
}