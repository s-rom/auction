#ifndef MONITOR_AUCTION
#define MONITOR_AUCTION

#include <cstdlib>
#include <ctime>

#include "MessageSystem.h"
#include "Message.h"
#include "NetProfile.h"
#include "SafeQueue.h"
#include "InfoReporter.h"

#include <boost/atomic/atomic.hpp>
#include <boost/thread.hpp>

#include <unordered_map>
#include <signal.h>





namespace Auction
{
    class Monitor;
}

class Auction::Monitor
{
public:
    Monitor();
    Monitor(std::string & program_path);
    
    void message_processor(boost::atomic<bool>& running);
    void new_robot_message_handler(Auction::NewRobotMessage * nr);
    void message_server(boost::atomic<bool>& running);

    Auction::SafeQueue<Auction::Message*> message_queue;
    Auction::MessageSystem message_system;
    Auction::InfoReporter info_report;

    int get_number_of_robots()
    {
        return num_of_robots;
    }

    
private:
    std::string get_log_path(std::string & program_path);
    Auction::Task generate_random_task();
    int robot_id = 0;
    int num_of_tasks = 0;
    int num_of_robots = 0;
    int next_task_id();
    int next_robot_id();

};


#endif