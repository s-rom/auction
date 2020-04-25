#ifndef MONITOR_AUCTION
#define MONITOR_AUCTION

#include <cstdlib>
#include <ctime>

#include "MessageSystem.h"
#include "Message.h"
#include "NetProfile.h"
#include "SafeQueue.h"
#include "InfoReporter.h"
#include "RobotStatus.h"
#include "TaskStatus.h"

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
    void message_server(boost::atomic<bool>& running);

    void leader_alive_message_handler(Auction::SimpleMessage & leader_alive);
    void new_task_message_handler(Auction::NewTaskMessage & new_task);
    void new_robot_message_handler(Auction::NewRobotMessage * nr);
    void robot_alive_message_handler(Auction::SimpleMessage & robot_alive);
    void kill_robot_message_handler(Auction::SimpleMessage & kill_robot);

    Auction::SafeQueue<Auction::Message*> message_queue;
    Auction::MessageSystem message_system;
    Auction::InfoReporter info_report;
    std::unordered_map<int, Auction::RobotStatusInfo> robot_status;
    
    // Task monitoring -- TODO unused
    std::unordered_map<int, Auction::RobotRole> robot_role;
    std::unordered_map<int, std::pair<Auction::Task, Auction::TaskStatus>> task_list;

    int get_number_of_robots()
    {
        return num_of_robots;
    }

    int get_number_of_tasks()
    {
        return num_of_tasks;
    }

    
private:
    std::string get_log_path(std::string & program_path);
    Auction::Task generate_random_task();
    void check_robot_status();
    int robot_id = 0;
    int num_of_tasks = 0;
    int num_of_robots = 0;
    int next_task_id();
    int next_robot_id();

    const int NULL_ID = -1;
    const int NULL_TASK = NULL_ID;
};


#endif