#include <iostream>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include "SafeQueue.h"

#include "RobotManager.h"

#include <signal.h>


boost::atomic<bool> running(true);

void sigint_handler(int signal)
{
    std::cout <<"\nNode killed by user. Shuting down..."<<std::endl;
    running = false;
    exit(0);
}



int main(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    using namespace Auction;


    if (argc < 3)
    {
        cout << "Usage: auction host port" << endl;
        return -1;
    }


    char * host = argv[1];
    char * port = argv[2];
    NetProfile np(host,port);
    RobotManager r(np);
    
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    boost::thread auction_thread(&RobotManager::auction_process, &r, boost::ref(running));
    
    server_thread.join();
    auction_thread.join();

}



