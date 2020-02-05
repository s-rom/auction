#include <iostream>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>


#include "RobotManager.h"

#include <signal.h>


boost::atomic<bool> running(true);

void sigint_handler(int signal)
{
    std::cout <<"\nNode killed by user. Shuting down..."<<std::endl;
    running = false;
    exit(0);
}


void prueba_robot(Auction::RobotManager& r)
{
    using namespace Auction;
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    server_thread.join();
}

int main()
{
    using std::cout;
    using std::endl;
    using namespace Auction;

    RobotManager r(0);

    signal(SIGINT, sigint_handler);
    prueba_robot(r);
}


