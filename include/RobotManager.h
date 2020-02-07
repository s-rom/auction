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
#include <chrono>

// Boost
#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>


namespace Auction
{
    using std::chrono::system_clock;
    using std::chrono::duration_cast;
    using namespace Auction;
    class RobotManager;
}


class Auction::RobotManager
{
public:
    // Constructors
    RobotManager(NetProfile & net_info);

    
    // Thread functions
    void message_server(boost::atomic<bool> & running);
    void auction_process(boost::atomic<bool> & running);

    // Auction algorithms
    void leader_request(Task & t);
    

    // Member attributes
    int id;                                            // Robot unique id
    MessageSystem message_system;                      // Message system
    std::unordered_map<int, NetProfile> net_list;      // List of net profiles of other robots
    std::unordered_map<int, Task> task_list;           // List of known tasks
    SafeQueue<Message*> message_queue;                 // Queue of received messages
    NetProfile net_info;                               // Own net profile

private:
    // Called in constructor. Requests unique id and broadcasts net profile 
    void request_id(NetProfile & np);
    void new_robot_message_handler(NewRobotMessage & nr);                                                       
    

    float get_work_capacity(Task & t);
    const float LOAD_CAPACITY = 1;                     // Maximum load capacity - kg (PLACEHOLDER)
    const float V_MAX = 10;                            // Max linear velocity - m/s  (PLACEHOLDER)                   
    const int TIME_LEADERSHIP = 3000;                  // Max time for the leader request - millis (PLACEHOLDER)
};

#endif