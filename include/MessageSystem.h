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
	{
		RcSocket::initRcSocket();
	}
	void send_message(Message &m, NetProfile &dst);
	void broadcast_message(Message &m, std::unordered_map<int,NetProfile> &net_list);
	Message* create_message_from(char * msg);


	// buena o mala idea??
	void cast_message(Message * m, NewTaskMessage * ntm);
	void cast_message(Message * m, LeaderOfTaskMessage * ltm);
	void cast_message(Message * m, LeaderRequestMessage * lreq);
};

#endif