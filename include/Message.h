#ifndef MESSAGE
#define MESSAGE

#include <string>
#include <sstream>
#include <algorithm>
#include "Task.h"
#include "NetProfile.h"
#include <iostream>

namespace Auction
{
	using std::string;
	using Auction::Task;
	class Message;
	class NewTaskMessage;
	class LeaderRequestMessage;
	class LeaderOfTaskMessage;
	class NewRobot;
	enum MessageType{ NEW_TASK = 0, LEADER_REQUEST, LEADER_OF_TASK, NEW_ROBOT, NEW_ID};

}

class Auction::Message
{
public:
	MessageType type;

	const static char DELIM='#';
	string virtual serialize() = 0; 
};


#define next_float_token(id) getline(ss,token,DELIM); id = stof(token);
#define next_int_token(id) getline(ss,token,DELIM); id = stoi(token);
#define next_token(id) getline(ss,token,DELIM); id = token;

class Auction::NewTaskMessage : public Auction::Message
{
public:
	NewTaskMessage(Task t)
	:
		t(t)
	{
		this->type = Auction::MessageType::NEW_TASK;
	};

	NewTaskMessage(string serialized_message)
	:
		t(serialized_message,DELIM)
	{
		this->type = Auction::MessageType::NEW_TASK;
	}

	string serialize()
	{ 
		return t.serialize(DELIM);
	}

	Task t;
};

class Auction::LeaderRequestMessage : public Auction::Message
{
public:
	LeaderRequestMessage(int task_id, int src_id, int dst_id, float bid)
	:	
		task_id(task_id),
		robot_src_id(src_id),
		robot_dst_id(dst_id),
		bid(bid)
	{
		this->type = Auction::MessageType::LEADER_REQUEST;
	};

	/**
	 * Creates a Message given a serialized string
	 * 
	 * Format required:  
	 * #type#task_id#src#dst#bid#
	 */
	LeaderRequestMessage(string serialized_message)
	{
		using std::stringstream;
		using std::getline;
		using std::stoi;
		using std::stof;

		this->type = MessageType::LEADER_REQUEST;

		stringstream ss(serialized_message);
		string token;
		
		getline(ss,token,DELIM); // DELIM
		getline(ss,token,DELIM); // type
		// getline(ss,token,DELIM); // task_id
		// task_id = stoi(token);

		// getline(ss,token,DELIM); // src
		// robot_src_id = stoi(token);

		// getline(ss,token,DELIM); // dst
		// robot_dst_id = stoi(token);

		// getline(ss,token,DELIM); // bid
		// bid = stof(token);
	
	
		next_int_token(task_id);
		next_int_token(robot_src_id);
		next_int_token(robot_dst_id);
		next_float_token(bid);
	}
 	
	/**
     * Creates a serialized string of this Message
     * 
     * Format:  
     * #type#task_id#src#dst#bid#
	 */
	string serialize()
	{
		using std::to_string;
		string s;
		s = DELIM +	
				to_string(type) + DELIM +
				to_string(task_id) + DELIM +
				to_string(robot_src_id) + DELIM +
				to_string(robot_dst_id) + DELIM +
				to_string(bid) +
			DELIM;
		return s;
	}

	int task_id;
	int robot_src_id;
	int robot_dst_id;
	float bid;
};


class Auction::LeaderOfTaskMessage : public Auction::Message
{
public:
	LeaderOfTaskMessage()
	{
		this->type = Auction::MessageType::LEADER_OF_TASK;
	}
	
	/**
	 * Creates a Message given a serialized string
	 * 
	 * Format:  
	 * #type#task_id#leader_id#
	 */
	LeaderOfTaskMessage(string serialized_message)
	{
		this->type = Auction::MessageType::LEADER_OF_TASK;

		using std::stringstream;
		using std::getline;
		using std::stoi;
		using std::stof;

		stringstream ss(serialized_message);
		string token;
		getline(ss,token,DELIM); // DELIM
		getline(ss,token,DELIM); // type
		getline(ss,token,DELIM); // task_id
		task_id = stoi(token);
		getline(ss,token,DELIM); // robot_leader
		robot_leader = stoi(token);
	}


	/**
     * Creates a serialized string of this Message
     * 
     * Format:  
     * #task_id#leader_id#
	 */
	string serialize()
	{
		using std::to_string;
		string s;
		s = DELIM + 
			to_string(type) + DELIM +
			to_string(task_id) + DELIM +
			to_string(robot_leader) + DELIM;
	}

	int task_id;
	int robot_leader;
};


class Auction::NewRobot : public Auction::Message
{
public:
	NewRobot(int unique_id, NetProfile np)
	:
		unique_id(unique_id)
	{
		this->np = np;
		this->type = Auction::MessageType::NEW_ROBOT;
	};

	/**
	 * Format: #type#unique_id#host#port# 
	 */
	NewRobot(string serialized_message)
	{
		this->type = Auction::MessageType::NEW_ROBOT;

		using std::stringstream;
		using std::getline;
		using std::stoi;
		using std::stof;

		stringstream ss(serialized_message);
		string token;
		getline(ss,token,DELIM); 	// DELIM
		getline(ss,token,DELIM); 	// type
		next_int_token(unique_id); 	// unique_id
		string host, port;
		next_token(host); next_token(port);
		this->np = NetProfile(host, port);

	}

	/**
	 * Format: #type#unique_id#host#port# 
	 */
	string serialize()
	{
		using std::to_string;
		string s = DELIM + 
				to_string(this->type) + DELIM +
				to_string(this->unique_id) + DELIM +
				string(np.host) + DELIM +
				string(np.port) + 
			DELIM; 
	}

	NetProfile np;
	int unique_id;

};

#endif