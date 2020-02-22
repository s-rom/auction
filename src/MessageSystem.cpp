#include "MessageSystem.h"

#include <iostream>

using namespace Auction;


MessageSystem::MessageSystem()
:
    monitor_info("localhost","25555")
{
    net_info = monitor_info;
    RcSocket::initRcSocket();
}

MessageSystem::MessageSystem(NetProfile & np)
:
    monitor_info("localhost","25555"),
    net_info(np)
{
    RcSocket::initRcSocket();
}

void MessageSystem::send_message(Message &m, int robot_id)
{
    if (robot_id == MONITOR_ID)
    {
        this->send_message(m, MessageSystem::monitor_info);
    }
    else 
    {
        NetProfile & dst = net_list[robot_id];
        this->send_message(m, dst);
    }
}

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
    // determines message type with a copy of the serialized message
    stringstream ss (serialized_message);
    string token;
    char delim = Message::DELIM;
    getline(ss, token, delim);
    getline(ss,token, delim);
    // dynamically casts to its derived class
    int type_id  = std::stoi(token);
    switch(type_id)
    {
        case MessageType::NEW_TASK:
            return new NewTaskMessage(serialized_message);
            break;
        case MessageType::LEADER_OF_TASK:
            return new LeaderOfTaskMessage(serialized_message);
            break;
        case MessageType::LEADER_REQUEST:
        case MessageType::BID_FOR_TASK:
            return new BidMessage(serialized_message);
            break;
        case MessageType::NEW_ROBOT:
            return new NewRobotMessage(serialized_message);
            break;

        case MessageType::BID_REQUEST:
            return new SimpleMessage(serialized_message);
            break;
    }

}

void MessageSystem::broadcast_message(Message &m)
{
    std::cout << "Broadcasting to:"<<std::endl;
    auto it = net_list.begin();
    auto end = net_list.end();
    while (it != end)
    {
        std::pair<int,NetProfile> element = *it;
        std::cout << "\t"<<element.second.to_string() << std::endl;
        send_message(m,element.second);
        it++;
    }
}
void MessageSystem::send_message_monitor(Message &m)
{
    send_message(m, this->monitor_info);
}


void MessageSystem::add_robot_info(int id, NetProfile & np)
{
    this->net_list[id] = np;
}

void MessageSystem::request_unique_id()
{
    NewRobotMessage m(NewRobotMessage::REQUEST_ID, this->net_info);
    send_message_monitor(m);
}


NetProfile MessageSystem::get_monitor_info()
{
    return MessageSystem::monitor_info;
}

NetProfile MessageSystem::get_robot_info(int id)
{
    return net_list[id];
}
