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
	class SimpleMessage;
	class NewTaskMessage;
	class BidMessage;
	class LeaderOfTaskMessage;
	class NewRobotMessage;
	class MonitoringMessage;

	enum MessageType
	{ 
		// ---------- DESCRIPTION---------------------------------------------	  ---- Message Class ----
		
		// Used by monitor to report a new task to all robots						- NewTaskMessage
		NEW_TASK = 0, 
		
		// Used by a robot that wants to request being the leader of a task			- BidMessage
		LEADER_REQUEST, 
		
		// Used by a robot who won a Leader round auction to inform all robots		- LeaderOfTaskMessage
		LEADER_OF_TASK, 

		// Used between a robot and the monitor for reporting system information	- NewRobotMessage
		NEW_ROBOT,

		// Used by leader, who requests robots to bid for his task					- SimpleMessage
		BID_REQUEST,																
		
		// Used to bid for a task, in the first or second round. 					- BidMessage
		BID_FOR_TASK,

		// Used by non leader robot to accept a group								- SimpleMessage
		ROBOT_ALIVE,

		// Used by non leader robot to refuse a group								- SimpleMessage
		REFUSE,

		// TODO																		- MonitoringMessage
		LEADER_ALIVE,

		// TODO																		- MonitoringMessage
		HELPER_ALIVE,

		// Used by monitor to kill robots											- MonitoringMessage
		ROBOT_KILL															
	};

}


#define next_float_token(id) getline(ss,token,DELIM); id = stof(token);
#define next_int_token(id) getline(ss,token,DELIM); id = stoi(token);
#define next_token(id) getline(ss,token,DELIM); id = token;

class Auction::Message
{
public:
	MessageType type;

	const static char DELIM='#';
	string virtual serialize() = 0; 
};


class Auction::SimpleMessage : public Auction::Message
{
public:

	SimpleMessage(int task_id, int robot_src, MessageType type)
	:
		task_id(task_id),
		robot_src(robot_src)
	{
		this->type = type;
	}
	
	/**
	 * Creates a Message given a serialized string
	 * 
	 * Format required:  
	 * #type#task_id#src#
	 */
	SimpleMessage(string serialized_message)
	{
		using std::stringstream;
		using std::getline;
		using std::stoi;
		using std::stof;


		stringstream ss(serialized_message);
		string token;
		
		getline(ss,token,DELIM); // DELIM
		getline(ss,token,DELIM); // type

		this->type = static_cast<MessageType>(std::stoi(token));

		next_int_token(task_id);
		next_int_token(robot_src);
	}
 	
	/**
     * Creates a serialized string of this Message
     * 
     * Format:  
     * #type#task_id#src#
	 */
	string serialize()
	{
		using std::to_string;
		string s;
		s = DELIM +	
				to_string(type) + DELIM +
				to_string(task_id) + DELIM +
				to_string(robot_src) + 
			DELIM;
		return s;
	}

	int robot_src;
	int task_id;
};



class Auction::MonitoringMessage : public Auction::Message
{
public:

	MonitoringMessage(int task_id, int robot_src, int load, bool completed,  MessageType type)
	:
		task_id(task_id),
		robot_src(robot_src),
		completed(completed),
		load(load)
	{
		this->type = type;
	}
	
	/**
	 * Creates a Message given a serialized string
	 * 
	 * Format required:  
	 * #type#task_id#src#load#completed#
	 */
	MonitoringMessage(string serialized_message)
	{
		using std::stringstream;
		using std::getline;
		using std::stoi;
		using std::stof;


		stringstream ss(serialized_message);
		string token;
		
		getline(ss,token,DELIM); // DELIM
		getline(ss,token,DELIM); // type

		this->type = static_cast<MessageType>(std::stoi(token));

		next_int_token(task_id);
		next_int_token(robot_src);
		next_int_token(load);
		next_int_token(completed);
	}
 	
	/**
     * Creates a serialized string of this Message
     * 
     * Format:  
     * #type#task_id#src#load#completed#
	 */
	string serialize()
	{
		using std::to_string;
		string s;
		s = DELIM +	
				to_string(type) 		+ DELIM +
				to_string(task_id) 		+ DELIM +
				to_string(robot_src) 	+ DELIM +
				to_string(load) 		+ DELIM + 
				to_string(completed ? 1 : 0) 	+
			DELIM;
		return s;
	}

	int robot_src;
	int task_id;
	bool completed;
	int load;
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

class Auction::BidMessage : public Auction::Message
{
public:
	BidMessage(int task_id, int src_id, float bid, float bid2, MessageType type)
	:	
		task_id(task_id),
		robot_src_id(src_id),
		bid(bid),
		bid2(bid2)
	{
		this->type = type;
	};

	/**
	 * Creates a Message given a serialized string
	 * 
	 * Format required:  
	 * #type#task_id#src#bid#bid2#
	 */
	BidMessage(string serialized_message)
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
		this->type = static_cast<MessageType>(std::stoi(token));

		next_int_token(task_id);
		next_int_token(robot_src_id);
		next_float_token(bid);
		next_float_token(bid2);

	}
 	
	/**
     * Creates a serialized string of this Message
     * 
     * Format:  
     * #type#task_id#src#bid#bid2#
	 */
	string serialize()
	{
		using std::to_string;
		string s;
		s = DELIM +	
				to_string(type) + DELIM +
				to_string(task_id) + DELIM +
				to_string(robot_src_id) + DELIM +
				to_string(bid) + DELIM +
				to_string(bid2) +
			DELIM;
		return s;
	}

	int task_id;
	int robot_src_id;
	float bid;
	float bid2;
};


class Auction::LeaderOfTaskMessage : public Auction::Message
{
public:
	LeaderOfTaskMessage(int task_id, int lead_id)
	:
		task_id(task_id),
		robot_leader(lead_id)
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
		return s;
	}

	int task_id;
	int robot_leader;
};


class Auction::NewRobotMessage : public Auction::Message
{
public:
	NewRobotMessage(int unique_id, NetProfile np)
	:
		unique_id(unique_id)
	{
		this->np = np;
		this->type = Auction::MessageType::NEW_ROBOT;
	};

	/**
	 * Format: #type#unique_id#host#port# 
	 */
	NewRobotMessage(string serialized_message)
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
		return s;
	}

	NetProfile np;
	int unique_id;
	// if monitor receives a NewRobot message with a unique_id == REQUEST_ID
	// it answers the robot with its unique id
	const static int REQUEST_ID = -1; 
};


#endif