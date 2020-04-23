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

    if (!nh.getParam("robot_name", robot_name))
    {
        ROS_INFO_STREAM("Could not load param 'robot_name'");
        robot_name = "";
        odom_topic = "/odom";
        move_base_server = "/move_base";
    }
    else 
    {
        move_base_server = robot_name+"/move_base";
        odom_topic = "/"+robot_name+"/odom";
    }
    
    std::string log_path = "/home/sergi/Desktop/" + robot_name + "_goal_sender.log";
    
    MoveBaseClient aux(move_base_server, true);
    ros::Subscriber odom_subscriber = nh.subscribe(odom_topic, 1, odom_callback);
    Auction::GoalManager goal_manager(&aux, log_path);
    goal_manager_ptr = &goal_manager;
    
    
    goal_manager.set_goal(Auction::Point2D(-5, 7));
    goal_manager.set_delivery(Auction::Point2D(0,0));
    goal_manager.set_total_travels(1);
    
    boost::thread ros_thread(&ros_polling_loop);
    boost::thread goal_thread(&Auction::GoalManager::goal_loop, &goal_manager);

    ros_thread.join();
    goal_thread.join();
}