#ifndef ROBOT_MANAGER
#define ROBOT_MANAGER


// Auction
#include "NetProfile.h"
#include "Task.h"
#include "Point2D.h"
#include "Message.h"
#include "MessageSystem.h"
#include "SafeQueue.h"

// std
#include <iostream>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <vector>
#include <unordered_map>

// Boost
#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>


namespace Auction
{
    using namespace Auction;
    class RobotManager;
}


class Auction::RobotManager
{
public:
    // Constructors
    RobotManager(int id);

    // Member functions
    void message_server(boost::atomic<bool> & running);
    void auction_process(boost::atomic<bool> & running);

    void leader_request();
    

    // Member attributes
    int id;                                            // Robot unique id
    MessageSystem message_system;                      // Message system
    std::unordered_map<int, NetProfile> net_list;      // List of net profiles of other robots
    std::unordered_map<int, Task> task_list;           // List of known tasks
    SafeQueue<Message*> message_queue;                 // Queue of received messages
};

#endif