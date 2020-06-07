#include "Monitor.h"

using Auction::Monitor;

std::string Monitor::get_log_path(std::string & program_path)
{
    std::string path(program_path.substr(0,program_path.find("devel")));
    path += "src/auction/logs/";
    return path;
}


Monitor::Monitor()
{
}

Monitor::Monitor(std::string & program_path)
:
    info_report(get_log_path(program_path)+"monitor.log",
        Auction::Mode::COUT | Auction::Mode::TIME | Auction::Mode::FILE)
{
}


void Monitor::message_processor(boost::atomic<bool>& running)
{
    info_report << "[Message thread] ---> running" << "\n";
    while(running)
    {
        if (!message_queue.isEmpty())
        {
            Message * m;
            message_queue.pop(m);
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1));

            switch (m->type)
            {
                case NEW_ROBOT:
                {
                    NewRobotMessage * nr = dynamic_cast<NewRobotMessage*>(m);
                    new_robot_message_handler(nr);
                    break;
                }
                case LEADER_ALIVE:
                {
                    MonitoringMessage * lead_alive = dynamic_cast<MonitoringMessage*>(m);
                    leader_alive_message_handler(*lead_alive);
                    break;
                }
                case NEW_TASK:
                {
                    NewTaskMessage * new_task = dynamic_cast<NewTaskMessage*>(m);
                    new_task_message_handler(*new_task);
                    break;
                }
                case ROBOT_ALIVE:
                {
                    MonitoringMessage * robot_alive = dynamic_cast<MonitoringMessage*>(m);
                    robot_alive_message_handler(*robot_alive);
                    break;
                }
                
                case HELPER_ALIVE:
                {
                    MonitoringMessage * helper_alive = dynamic_cast<MonitoringMessage*>(m);
                    helper_alive_message_handler(*helper_alive);
                    break;
                }

                case ROBOT_KILL:
                {
                    SimpleMessage * kill_robot = dynamic_cast<SimpleMessage*>(m);
                    kill_robot_message_handler(*kill_robot);
                    break;
                }

            }
            delete m;    
        }

        check_robot_status();

    }
}



void Monitor::kill_robot_message_handler(Auction::SimpleMessage & kill_robot)
{
    info_report << "[KillRobotMessageHandler]: Received an order to kill robot "<< kill_robot.robot_src<<"\n";
    auto it_robot = robot_status.find(kill_robot.robot_src);

    if (it_robot != robot_status.end()
        && it_robot->second.current_status != RobotStatus::DEAD)
    {
        SimpleMessage new_kill_robot(Task::NULL_TASK, kill_robot.robot_src, Auction::MessageType::ROBOT_KILL);
        info_report << "[KillRobotMessageHandler]: Sending kill robot to robot "<< kill_robot.robot_src<<"\n";
        info_report << "[KillRobotMessageHandler]: " << new_kill_robot.serialize() << "\n";
        message_system.send_message(new_kill_robot, new_kill_robot.robot_src);
        it_robot->second.current_status = RobotStatus::DEAD;
    }
}


void Monitor::new_task_message_handler(Auction::NewTaskMessage & new_task)
{
    info_report << "[NewRobotMessageHandler]: Received new task from web service.";
    new_task.t.task_id = next_task_id();

    auto pair_task = std::make_pair(new_task.t, TaskStatus::WAITING);
    task_list[new_task.t.task_id] = pair_task;

    info_report << "Broadcasting as task "<< new_task.t.task_id << "\n";
    message_system.broadcast_message(new_task);
}

void Monitor::leader_alive_message_handler(Auction::MonitoringMessage & leader_alive)
{
    info_report << "[LeaderAliveMessageHandler] Received from " << leader_alive.robot_src 
        << " who is leading task " << leader_alive.task_id << "\n";

    Auction::RobotStatusInfo & info = this->robot_status[leader_alive.robot_src];
    // TODO: Change task status in task_list
    info.update_last_time_point();   
    info.current_role = RobotRole::LEADING;
    info.first_time_point = false;
}

void Monitor::robot_alive_message_handler(Auction::MonitoringMessage & robot_alive)
{
    Auction::RobotStatusInfo & info = this->robot_status[robot_alive.robot_src];    
    info.update_last_time_point();
    info.first_time_point = false;    
    info.current_role = RobotRole::IDLE;
}


void Monitor::helper_alive_message_handler(Auction::MonitoringMessage & helper_alive)
{
    Auction::RobotStatusInfo & info = this->robot_status[helper_alive.robot_src];    
    info.current_role = RobotRole::HELPING;
    info.update_last_time_point();
    info.first_time_point = false;    
}



void Monitor::check_robot_status()
{
    for (auto it = robot_status.begin(); it != robot_status.end(); it++)
    {
        int robot_id = it->first;
        Auction::RobotStatusInfo & info = it->second; 

        float elapsed = info.get_elapsed_millis();
        if (!info.first_time_point && 
            info.current_status != Auction::RobotStatus::DEAD &&
            elapsed > (Auction::RobotStatusInfo::TIME_LEAD_ALIVE_MILLIS + Auction::RobotStatusInfo::TOLERANCE))
        {
            info.current_status = Auction::RobotStatus::DEAD;
            info_report << "[CheckRobotStatus]: Robot "<< robot_id << " is considered dead\n";
        }

    }
}

void Monitor::new_robot_message_handler(NewRobotMessage * nr)
{
    // add netprofile to unordered map
    if (nr->unique_id == NewRobotMessage::REQUEST_ID)
    {
        info_report << "Received NewRobot message from "<<nr->np.to_string()<< "\n";

        int next_id;
        next_id = message_system.find_robot_id(nr->np);

        // If not found, generate new ID
        if (next_id == -1)
            next_id = next_robot_id();
 
        nr->unique_id = next_id;
        message_system.add_robot_info(nr->unique_id, nr->np); // store new robot net profile
        
        Auction::RobotStatusInfo robot_status_info;
        this->robot_status[next_id] = robot_status_info;
        
        for (int id = 1; id<=num_of_robots; id++)
        {
            NetProfile np = message_system.get_robot_info(id);
            nr->np = np;
            nr->unique_id = id;

            message_system.broadcast_message(*nr);
        }
    }
}


void Monitor::message_server(boost::atomic<bool>& running)
{
    NetProfile monitor_info = message_system.get_monitor_info();
    if (monitor_info.host == NULL || monitor_info.port == NULL)
    {
        info_report << "NULL member" << "\n";
        return;
    }


    int socket_descriptor = RcSocket::passiveSocket(monitor_info.port, "udp", 0);
    if (socket_descriptor < 0)
    {
        info_report << "Error in RcSocket::passiveSocket"<< "\n";
        return;
    }

    info_report << "[Monitor] --> listening on port " << monitor_info.port << "\n";

    while (running)
    {
        struct sockaddr sender;
        uint addrlen;
        char message_systemg[256];
        memset(&message_systemg, 0, 256);
        // Recibe la informacion y la guarda en el buffer
        if (!recvfrom(socket_descriptor, message_systemg, sizeof(message_systemg), 0, &sender, &addrlen))
        {
            printf("[Server thread] Error de recepcion \n");
        }
        else
        { 
            Message * m = this->message_system.create_message_from(message_systemg);
            message_queue.push(m);
        }
    }
}

Auction::Task Monitor::generate_random_task()
{
    using namespace Auction;
    Point2D p(rand() % 20, rand() % 20); // 0 to 19 x y
    Point2D delivery(rand() % 20, rand() % 20); // 0 to 19 x y
    float workload = (rand() % 15) + 1; // 1 to 15 kg
    float dead_line = (rand() % 20000) + 10000; // 10s to 30s

    Task t(p,delivery, workload, dead_line, next_task_id());
    return t;
}



int Monitor::next_task_id()
{
    return ++num_of_tasks;
}

int Monitor::next_robot_id()
{
    return ++num_of_robots;
}
