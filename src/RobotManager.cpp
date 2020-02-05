#include "RobotManager.h"

using Auction::RobotManager;

RobotManager::RobotManager(int id)
:
    id(id)
{
    NetProfile my_netp("localhost", "25555");
    net_list[id] = my_netp;
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

void RobotManager::leader_request()
{

}

