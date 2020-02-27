#include <iostream>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include "RobotManager.h"

#include <signal.h>

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

int main(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    using namespace Auction;


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
    RobotManager r(std::string(argv[0]),np);
    r_ptr = &r;
    signal(SIGINT, sigint_handler);
    
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    boost::thread auction_thread(&RobotManager::auction_process, &r, boost::ref(running));
    
    server_thread.join();
    auction_thread.join();

}







