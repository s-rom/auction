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

void sigint_handler(int signal)
{
    std::cout <<"\nNode killed by user. Shuting down..."<<std::endl;
    running = false;
    r_ptr->close_info_reporter("[RobotAuction]: Killed by SIGINT\n");
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
    boost::atomic<Auction::Point2D> new_current(Auction::Point2D(pos.x,pos.y));
    // Update current position
}

int main(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    using namespace Auction;
    srand(time(0));

    if (argc < 3)
    {
        cout << "Usage: auction host port" << endl;
        return -1;
    }

    // Initializes ros
    ros::init(argc,argv,"robot_node", ros::init_options::AnonymousName);
    
    // Get param robot_name
    ros::NodeHandle nh("~");
    std::string robot_name;
    if (!nh.getParam("robot_name", robot_name))
        ROS_INFO_STREAM("Could not load param 'robot_name'");
    
    
    // Creates a goal_client, a subscriber to odometry and a GoalManager
    MoveBaseClient goal_client(robot_name+"/move_base", true);
    ros::Subscriber odom_subscriber = nh.subscribe("/"+robot_name+"/odom",1,odom_callback);
    GoalManager goalManager(&goal_client);

    char * host = argv[1];
    char * port = argv[2];
    
    // Constructs the Robot's NetProfile
    NetProfile np(host,port);

    // Constructs the RobotManager object
    RobotManager r(std::string(argv[0]),np);

    // Randomizes max speed and load capacity
    int max_vel = get_rand_range(3,6);
    int load_capacity = get_rand_range(1,4);

    r.set_max_linear_vel(max_vel);
    r.set_load_capacity(load_capacity);

    std::cout << "Generated robot with max_vel: "<<max_vel<<" and load_capacity: "<<load_capacity<<"\n";

    // For SIGINT handling
    r_ptr = &r;
    signal(SIGINT, sigint_handler);
    
    // Starts threads
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    boost::thread auction_thread(&RobotManager::auction_process, &r, boost::ref(running));
    boost::thread periodic_thread(&RobotManager::periodic_behaviour, &r, boost::ref(running));
    boost::thread ros_thread(&ros_polling_loop);
    
    server_thread.join();
    auction_thread.join();
    periodic_thread.join();
    ros_thread.join();

}






