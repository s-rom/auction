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



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


boost::atomic<bool> any_goal(false);

boost::atomic<Auction::Point2D> goal(Auction::Point2D(0, 0));
boost::atomic<Auction::Point2D> delivery(Auction::Point2D(-5,-5));
boost::atomic<Auction::Point2D> * current;

void sigintHandler(int sig)
{
    ROS_INFO("Process killed by user. Shuting down node.");
    ros::shutdown(); 
}

void odom_callback(const nav_msgs::Odometry::ConstPtr & msg)
{
    auto pos = msg->pose.pose.position;
    boost::atomic<Auction::Point2D> new_current(Auction::Point2D(pos.x,pos.y));
    current = &new_current;
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

MoveBaseClient * ac;


void goal_management_loop()
{
    while (!ac->waitForServer(ros::Duration(2.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    float tolerance = 2;

    while(true)
    {
        Auction::Point2D current_goal;

        if (current != nullptr && Auction::Point2D::equals(*current, goal, tolerance))
        {           
            current_goal = delivery;
        } 
        else if (current != nullptr && Auction::Point2D::equals(*current, delivery, tolerance))
        {
            current_goal = goal;
        }


        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = "/map";
        goal_msg.target_pose.header.stamp = ros::Time::now();

        goal_msg.target_pose.pose.position.x = current_goal.x;
        goal_msg.target_pose.pose.position.y = current_goal.y;
        goal_msg.target_pose.pose.orientation.w = 1;

        ROS_INFO_STREAM("Sending goal to "<<current_goal.x<<","<<current_goal.y<<"\n");
        ac->sendGoal(goal_msg);
        ac->waitForResult();
        if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal achieved!");
        }
        else
        {
            ROS_INFO("Goal not achieved");
        }


    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_sender");
    signal(SIGINT, sigintHandler);

    ros::NodeHandle nh("~");

    std::string robot_name;
    if (!nh.getParam("robot_name", robot_name))
        ROS_INFO_STREAM("Could not load param 'robot_name'");
    
    
    MoveBaseClient aux(robot_name+"/move_base", true);
    ac = &aux;

    ros::Subscriber odom_subscriber = nh.subscribe("/"+robot_name+"/odom",1,odom_callback);

    
    boost::thread ros_thread(&ros_polling_loop);
    boost::thread goal_thread(&goal_management_loop);
    ros_thread.join();
    goal_thread.join();
}