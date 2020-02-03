#include <iostream>
#include "NetProfile.h"
#include "Task.h"
#include "Point2D.h"
#include "Message.h"
#include <cstring>

#include <sstream>
#include <algorithm>

#include "MessageSystem.h"

#include <vector>
#include <unordered_map>
#include <queue>
#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include <signal.h>

namespace Auction
{
    struct RobotManager;
}

struct Auction::RobotManager
{

    int id;
    typedef std::queue<Message*> safe_queue; // tiene que ser ptr por polimorfia
    MessageSystem message_system;
    std::unordered_map<int, NetProfile*> net_list;
    std::unordered_map<int, Task> task_list;
    safe_queue message_queue;

    RobotManager(int id)
    :
        id(id)
    {
        net_list[id] = new NetProfile("localhost", "25555");
    }

    void message_server(boost::atomic<bool> & running)
    {
        int socket_descriptor = RcSocket::passiveSocket(net_list[id]->port, "udp", 0);
        if (socket_descriptor < 0)
            std::cout << "Error in RcSocket::passiveSocket"<< std::endl;
            

        while (running)
        {
            std::cout << "Server listening on port " << net_list[id]->port << std::endl;
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



};


boost::atomic<bool> running(true);

void sigint_handler(int signal)
{
    std::cout <<"Node killed by user. Shuting down..."<<std::endl;
    running = false;
    exit(0);
}

int main()
{
    using std::cout;
    using std::endl;
    using namespace Auction;

    signal(SIGINT, sigint_handler);
   
    RobotManager r(0);
    boost::thread server_thread(&RobotManager::message_server, &r, boost::ref(running));
    server_thread.join();
}


