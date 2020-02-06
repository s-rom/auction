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


void prueba_robot(Auction::RobotManager& r)
{
    using namespace Auction;
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    server_thread.join();
}

void prueba_lider(Auction::RobotManager & r)
{
    using namespace Auction;
    Task task(Point2D(0,0), Point2D(1,2), 1, 1, 1);
    
    boost::thread leader_thread(&Auction::RobotManager::leader_request, &r, boost::ref(task));
    leader_thread.join();
}


int main(int argc, char ** argv)
{
    using std::cout;
    using std::endl;
    using namespace Auction;


    // RobotManager r(0,);

    // signal(SIGINT, sigint_handler);
    // // prueba_lider(r);
    // prueba_robot(r);

}



