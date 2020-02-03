#pragma once
#include <string>
#include <sstream>
#include <algorithm>
#include "Task.h"
#include <iostream>

namespace Auction
{
	using std::string;
	using Auction::Task;
	class Message;
	class NewTaskMessage;
	class LeaderRequestMessage;
	class LeaderOfTaskMessage;
	enum MessageType{ NEW_TASK = 0, LEADER_REQUEST, LEADER_OF_TASK};

}

class Auction::Message
{
public:
	MessageType type;

	const static char DELIM='#';
	string virtual serialize() = 0; 
};


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
		getline(ss,token,DELIM); // task_id
		task_id = stoi(token);

		getline(ss,token,DELIM); // src
		robot_src_id = stoi(token);

		getline(ss,token,DELIM); // dst
		robot_dst_id = stoi(token);

		getline(ss,token,DELIM); // bid
		bid = stof(token);
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