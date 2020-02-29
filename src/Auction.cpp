#include <iostream>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include "RobotManager.h"

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
    r_ptr->close_info_reporter();
    // ros::shutdown();
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

int main(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    using namespace Auction;
    srand(time(0));

    // ros::init(argc,argv,"robot_node", ros::init_options::AnonymousName);
    // ros::NodeHandle nh("~");

    // std::string host_param;
    // int port_param;
    // nh.getParam("host",host_param);
    // nh.getParam("port",port_param);

    // std::cout <<"Params: "<< host_param << "," << port_param << "\n";
    // std::string host = host_param.c_str();
    // std::string port = std::to_string(port_param);

    if (argc < 3)
    {
        cout << "Usage: auction host port" << endl;
        return -1;
    }

    char * host = argv[1];
    char * port = argv[2];
    
    NetProfile np(host,port);

    // Constructs the RobotManager object
    RobotManager r(std::string(argv[0]),np);

    int max_vel = get_rand_range(3,15);
    int load_capacity = get_rand_range(1,14);

    r.set_max_linear_vel(max_vel);
    r.set_load_capacity(load_capacity);

    std::cout << "Generated robot with max_vel: "<<max_vel<<" and load_capacity: "<<load_capacity<<"\n";

    // For SIGINT handling
    r_ptr = &r;
    signal(SIGINT, sigint_handler);
    
    // Starts threads
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    boost::thread auction_thread(&RobotManager::auction_process, &r, boost::ref(running));
    
    server_thread.join();
    auction_thread.join();

}






