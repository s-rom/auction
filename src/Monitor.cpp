#include "MessageSystem.h"
#include "Message.h"
#include "NetProfile.h"
#include "SafeQueue.h"
#include <unordered_map>

#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

namespace Auction
{
    using std::cout;
    using std::endl;
    using namespace Auction;
    class Monitor;
}

class Auction::Monitor
{
public:
    Monitor(){}


    void message_processor(boost::atomic<bool>& running)
    {
        std::cout << "Message processor is running" << std::endl;
        while(running)
        {
            if (message_list.isEmpty()) continue;
            std::cout << "[Message thread] Processing new message"<<std::endl;
            Message * m;
            message_list.pop(m);

            switch (m->type)
            {
                case NEW_ROBOT:
                    NewRobotMessage * nr = dynamic_cast<NewRobotMessage*>(m);
                    new_robot_message_handler(nr);
                    break;
            }
            delete m;            
        }
    }

    void new_robot_message_handler(NewRobotMessage * nr)
    {
        // add netprofile to unordered map
        if (nr->unique_id == NewRobotMessage::REQUEST_ID)
        {
            std::cout << "Received NewRobot message from "<<nr->np.to_string()<< std::endl;
            nr->unique_id = next_robot_id();    // replace REQUEST_ID with the new unique_id
            net_list[nr->unique_id] = nr->np;   // store new robot net profile
            ms.broadcast_message(*nr,net_list); // broadcast to all robots the new one 
            std::cout << "Broadcast id: "<<nr->unique_id<< std::endl;
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
                cout << "[Server thread] Received message: "<<string(msg)<<endl;
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



int main()
{
    using namespace Auction; 
    Monitor m;
    boost::atomic<bool> running(true);
    boost::thread server_thread(&Auction::Monitor::message_server, &m, boost::ref(running));
    boost::thread message_thread(&Auction::Monitor::message_processor, &m, boost::ref(running));


    server_thread.join();
    message_thread.join();
}