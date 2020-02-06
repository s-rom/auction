#include "RobotManager.h"

using Auction::RobotManager;

RobotManager::RobotManager(int id, NetProfile & net_info)
:
    id(id)
{
    net_list[id] = net_info;
}

void RobotManager::message_server(boost::atomic<bool> & running)
{
    NetProfile * nptr = &(net_list.at(id));
    if (nptr == nullptr) 
    {
        std::cout << "NULL NetProfile" << std::endl;
        return;
    }
    
    if (nptr->host == NULL || nptr->port == NULL)
    {
        std::cout << "NULL member" << std::endl;
        return;
    }


    int socket_descriptor = RcSocket::passiveSocket(net_list[id].port,"udp", 0);
    if (socket_descriptor < 0)
    {
        std::cout << "Error in RcSocket::passiveSocket"<< std::endl;
        return;
    }

    std::cout << "Server listening on port " << net_list[id].port << std::endl;

    while (running)
    {
        struct sockaddr sender;
        uint addrlen;
        char msg[256];
        memset(&msg, 0, 256);
        // Recibe la informacion y la guarda en el buffer
        if (!recvfrom(socket_descriptor, msg, sizeof(msg), 0, &sender, &addrlen))
        {
            printf("Error de recepcion \n");
        }
        else
        { 
            Message * m = this->message_system.create_message_from(msg);
            message_queue.push(m);
        }
    }
}

void RobotManager::auction_process(boost::atomic<bool> & running)
{

}


// #define now system_clock::now()
// #define elapsed(t1, t2) (duration_cast<std::chrono::milliseconds>(t1 - t1).count())

void RobotManager::leader_request(Task & t)
{
    std::cout << "Comenzado algoritmo de leader" << std::endl;
    auto time_init = system_clock::now();
    auto delta_time = duration_cast<std::chrono::milliseconds>
                        (time_init - time_init).count();
    float bid = this->get_work_capacity(t); // bid


    while (delta_time < TIME_LEADERSHIP)
    {
        // snapshot of the message queue
        auto end = this->message_queue.end();
        auto it = this->message_queue.begin();
        while (it != end)
        {   
            Message * m = *it;
            if (m->type == MessageType::LEADER_REQUEST)
            {
                LeaderRequestMessage * lreq = dynamic_cast<LeaderRequestMessage *>(m);
                // Found message requesting this task with a better bid
                if (lreq->task_id == t.task_id && lreq->bid > bid)
                {
                    //remove this message
                    //delete this message memory
                    //give up
                    std::cout << "Found message "<<lreq->task_id<<" requesting task "<<t.task_id<<
                    "with a better bid ----> Give up"<< std::endl;
                    return;
                } 
            } 
            else if (m->type == MessageType::LEADER_OF_TASK)
            {
                LeaderOfTaskMessage * lmess = dynamic_cast<LeaderOfTaskMessage *>(m);
                //remove this message
                //delete this message memory
                //give up
                std::cout << "Found message "<<lmess->task_id<<" requesting task "<<t.task_id<<
                "as a new leader ----> Give up"<< std::endl;
                return;
            }
            it++;
        }

        // Update delta time
        delta_time = duration_cast<std::chrono::milliseconds>
                        (system_clock::now() - time_init).count();
    }

    std::cout << "I'm new leader of task "<<t.task_id<<std::endl;
}

float RobotManager::get_work_capacity(Task& t)
{
    float distance = Point2D::euclidean_distance(t.delivery_point,t.task_location);
    float a = LOAD_CAPACITY * V_MAX;
    float b = 2 * (LOAD_CAPACITY * V_MAX + distance);
    return b == 0? 0 : a / b;
}
