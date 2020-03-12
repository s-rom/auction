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
        if (message_queue.isEmpty()) continue;
        info_report << "[Message processor] Processing new message"<< "\n";
        Message * m;
        message_queue.pop(m);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1));

        switch (m->type)
        {
            case NEW_ROBOT:
                NewRobotMessage * nr = dynamic_cast<NewRobotMessage*>(m);
                new_robot_message_handler(nr);
                break;
        }
        delete m;    

        if (num_of_robots == 5)
        {
            const int TASK_NUM = 1;
            info_report << "[Message processor] Generating and sending " <<TASK_NUM<<(TASK_NUM>1?"tasks":"task") << "\n";
            for (int i = 0; i<TASK_NUM; i++)
            {
                boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
                NewTaskMessage nt(generate_random_task());
                message_system.broadcast_message(nt);
            }
        }

    }
}

void Monitor::new_robot_message_handler(NewRobotMessage * nr)
{
    // add netprofile to unordered map
    if (nr->unique_id == NewRobotMessage::REQUEST_ID)
    {
        info_report << "Received NewRobot message from "<<nr->np.to_string()<< "\n";
        nr->unique_id = next_robot_id();    // replace REQUEST_ID with the new unique_id
        message_system.add_robot_info(nr->unique_id, nr->np); // store new robot net profile

        
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
