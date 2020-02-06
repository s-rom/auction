#include "MessageSystem.h"
#include "Message.h"
#include "NetProfile.h"
#include "SafeQueue.h"
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <unordered_map>

namespace Auction
{
    using namespace Auction;
    class Monitor;
}

class Auction::Monitor
{
public:
    Monitor(){}


    void message_processor(boost::atomic<bool>& running)
    {
        while(running)
        {
            auto it = message_list.begin();
            auto end = message_list.end();
            while (it != end)
            {
                Message * m = *it;
                if (m->type == NEW_ROBOT)
                {
                    // dynamic cast
                    // add netprofile to unordered map
                    // broadcast to all other robots with NewRobot
                    // message NewId to robot src
                }

                it++;
            }
        }
    }

    
    void message_server(boost::atomic<bool>& running)
    {
        NetProfile& monitor_info = ms.monitor_info;
        if (monitor_info.host == NULL || monitor_info.port == NULL)
        {
            std::cout << "NULL member" << std::endl;
            return;
        }


        int socket_descriptor = RcSocket::passiveSocket(monitor_info.port,"udp", 0);
        if (socket_descriptor < 0)
        {
            std::cout << "Error in RcSocket::passiveSocket"<< std::endl;
            return;
        }

        std::cout << "Monitor listening on port " << monitor_info.port << std::endl;

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
                Message * m = this->ms.create_message_from(msg);
                message_list.push(m);
            }
        }
    }


    int next_robot_id()
    {
        return ++num_of_robots;
    }

    SafeQueue<Message*> message_list;
    MessageSystem ms;
    std::unordered_map<int, NetProfile> net_list;
    int robot_id = 0;
    int num_of_robots = 0;
};


boost::atomic<bool> running;

int main()
{
    using namespace Auction;
    MessageSystem ms;    

    


}