#include <iostream>

#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "RobotManager.h"
#include "GoalManager.h"

#include <signal.h>
#include <ctime>
#include <cstdlib>

#include <ros/ros.h>


boost::atomic<bool> running(true);
Auction::RobotManager * r_ptr;
Auction::GoalManager * goal_ptr;

void sigint_handler(int signal)
{
    std::cout <<"\nNode killed by user. Shuting down..."<<std::endl;
    running = false;
    r_ptr->close_info_reporter("[RobotAuction]: Killed by SIGINT\n");
    goal_ptr->close_info_reporter();
    exit(0);
}

/**
 * Returns an integer between a min and max (both inclusive)
 * 
 * @returns rand int in [min,max]
 */
int get_rand_range(int min, int max)
{
    int rnd = (rand() % (max - min + 1)) + min;
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
    goal_ptr->update_position(new_p); 
}


int main(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    using namespace Auction;
    srand(time(0));
    
    // if (argc < 3)
    // {
    //     cout << "Usage: auction host port" << endl;
    //     return -1;
    // }

    // Initializes ros
    ros::init(argc,argv,"robot_node", ros::init_options::AnonymousName);
     

    // Get param robot_name
    ros::NodeHandle nh("~");
    std::string hostname;
    std::string port;
    std::string robot_name;
    std::string odom_topic;
    std::string move_base_server;
    std::string map_frame;

    if (!nh.getParam("port", port))
    {
        ROS_INFO_STREAM("Port param is missing!");
        return -1;
    }
    else 
    {
        port.erase(port.begin());
        ROS_INFO_STREAM("Port: "<<port<<"\n");
    }
    
    if (!nh.getParam("hostname", hostname))
    {
        ROS_INFO_STREAM("Hostname param is missing!");
        return -1;
    }


    if (!nh.getParam("robot_name", robot_name))
    {
        ROS_INFO_STREAM("Could not load param 'robot_name'");
        odom_topic = "/odom";
        robot_name = "";
        move_base_server = "/move_base";
        map_frame = "/map";
    }
    else 
    {
        move_base_server = robot_name+"/move_base";
        odom_topic = "/"+robot_name+"/odom";
        map_frame=robot_name+"/map";
        ROS_INFO_STREAM("Robot name: "<<robot_name);
    }

    std::string log_path = "/home/sergi/Desktop/" + robot_name + "_goal_sender.log";
    
    MoveBaseClient goal_client(move_base_server, true);
    ros::Subscriber odom_subscriber = nh.subscribe(odom_topic, 1, odom_callback);
    GoalManager goal_manager(&goal_client, log_path);
    goal_manager.set_map_frame(map_frame);
    goal_ptr = &goal_manager;
    
    // Constructs the Robot's NetProfile
    NetProfile np(hostname, port);

    // Constructs the RobotManager object
    RobotManager r(std::string(argv[0]),np);

    // Randomizes max speed and load capacity
    int max_vel = get_rand_range(3,6);
    int load_capacity = get_rand_range(1,4);

    r.set_goal_manager(goal_ptr);
    r.set_max_linear_vel(max_vel);
    r.set_load_capacity(load_capacity);

    std::cout << "Generated robot with max_vel: "<<max_vel<<" and load_capacity: "<<load_capacity<<"\n";

    // For SIGINT handling
    r_ptr = &r;
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);
    
    // Starts threads
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    boost::thread auction_thread(&RobotManager::auction_process, &r, boost::ref(running));
    boost::thread periodic_thread(&RobotManager::periodic_behaviour, &r, boost::ref(running));
    boost::thread ros_thread(&ros_polling_loop);
    boost::thread goal_management_thread(&GoalManager::goal_loop, &goal_manager);
    
    server_thread.join();
    auction_thread.join();
    periodic_thread.join();
    ros_thread.join();
    goal_management_thread.join();
}






