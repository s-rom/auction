#ifndef MESSAGE_SYSTEM
#define MESSAGE_SYSTEM


#include "Message.h"
#include "NetProfile.h"
#include "RcSocket.h"
#include <unordered_map>

namespace Auction
{
	using Auction::Message;
	using Auction::NetProfile;
	class MessageSystem;
}



class Auction::MessageSystem
{
public:
	MessageSystem()
	:
		monitor_info("localhost","25555")
	{
		RcSocket::initRcSocket();
	}

	void send_message_monitor(Message &m);
	void send_message(Message &m,NetProfile &dst);
	void broadcast_message(Message &m, std::unordered_map<int,NetProfile> &net_list);
	Message* create_message_from(char * msg);
	NetProfile monitor_info;
};

#endif