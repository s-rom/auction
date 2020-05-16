#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>

#include <iostream>
#include <string>
#include <signal.h>

#include <Point2D.h>
#include "GoalManager.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

Auction::GoalManager * goal_manager_ptr;

void sigintHandler(int sig)
{
    ROS_INFO("Process killed by user. Shuting down node.");
    goal_manager_ptr->close_info_reporter();
    ros::shutdown(); 
}

void ros_polling_loop()
{
    ros::Rate rate(1); //Hz
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr & msg)
{
    auto pos = msg->pose.pose.position;
    Auction::Point2D new_p(msg->pose.pose.position.x, msg->pose.pose.position.y); 
    goal_manager_ptr->update_position(new_p); 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_sender");
    signal(SIGINT, sigintHandler);
    signal(SIGTERM, sigintHandler);


    ros::NodeHandle nh("~");

    std::string robot_name;
    std::string odom_topic;
    std::string move_base_server;
    std::string map_frame;
    std::string goal_topic = "/goal";

    if (!nh.getParam("robot_name", robot_name))
    {
        ROS_INFO_STREAM("Could not load param 'robot_name'");
        robot_name = "";
        odom_topic = "/odom";
        move_base_server = "/move_base";
        map_frame = "/map";
    }
    else 
    {
        map_frame = robot_name+"/map";
        move_base_server = robot_name+"/move_base";
        odom_topic = "/"+robot_name+"/odom";
    }
    
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>(goal_topic, 1);


    std::string log_path = "/home/sergi/Desktop/" + robot_name + "_goal_sender.log";
    Auction::GoalManager goal_manager(pub, log_path);
    goal_manager_ptr = &goal_manager;
    
    ros::Subscriber odom_subscriber = nh.subscribe(odom_topic, 1, odom_callback);
    
    boost::thread ros_thread(&ros_polling_loop);

    ros_thread.join();
}