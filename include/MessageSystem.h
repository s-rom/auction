#pragma once
#include "Message.h"
#include "NetProfile.h"
#include "RcSocket.h"


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
	Message* createMessageFrom(char * msg);
};