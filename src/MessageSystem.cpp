#include "MessageSystem.h"

#include <iostream>

using namespace Auction;

void MessageSystem::send_message(Message &m, NetProfile &dst)
{
    string msg_str = m.serialize();
    char msg_ptr[msg_str.length() + 1];
    strcpy(msg_ptr, msg_str.c_str());

    RcSocket::sendMessage(msg_ptr, dst.host, dst.port);
}


Message* MessageSystem::create_message_from(char * msg)
{
    using std::string;
    using std::stringstream;
    using std::getline;

    string serialized_message(msg);
    stringstream ss (serialized_message);
    string token;
    char delim = Message::DELIM;
    getline(ss, token, delim);
    getline(ss,token, delim);
    int type_id  = std::stoi(token);
    switch(type_id)
    {
        case MessageType::NEW_TASK:
            std::cout << "new task" << std::endl;
            return new NewTaskMessage(serialized_message);
            break;
        case MessageType::LEADER_OF_TASK:
            std::cout << "leader of task" << std::endl;
            return new LeaderOfTaskMessage(serialized_message);
            break;
        case MessageType::LEADER_REQUEST:
            std::cout << "leader request" << std::endl;
            return new LeaderRequestMessage(serialized_message);
            break;
    }

}

void MessageSystem::broadcast_message(Message &m, std::unordered_map<int,NetProfile> &net_list)
{
    auto it = net_list.begin();
    auto end = net_list.end();
    while (it != end)
    {
        std::pair<int,NetProfile> element = *it;
        send_message(m,element.second);
        it++;
    }
}
