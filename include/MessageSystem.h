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

	NetProfile net_info; // Own net profile

	/**
	 * Default constructor. 
	 * 
	 * net_info member is equivalent to monitor info. Used only
	 * for system's monitor 
	 */
	MessageSystem();


	/**
	 * Constructor 
	 * 
	 * @param np Robot's net profile
	 */
	MessageSystem(NetProfile & np);

	/**
	 * Sends a message to the system's monitor.
	 * 
	 * @param m reference to any Message child
	 */
	void send_message_monitor(Message &m);
	
	/**
	 * Sends a message to the net profile corresponding to the 
	 * unique id.
	 * @param m Reference to any Message child
	 * @param robot_id unique id of the destination robot. If 0, the 
	 * destination corresponds to the monitor
	 */
	void send_message(Message& m, int robot_id);

	/**
	 * Broadcasts a message to all other robots. The monitor is not included.
	 * 
	 * @param m reference to any Message child
	 */
	void broadcast_message(Message &m);
	
	/**
	 * Parses a received message and instanciates a Message pointer
	 * 
	 * @param msg string in serialized format (See Message.h) 
	 */
	Message* create_message_from(char * msg);

	/**
	 * Adds a robot net profile to unordered map, indexed by it's id 
	 * 
	 */
	void add_robot_info(int id, NetProfile & np);
	
	/**
	 * Provides a NetProfile corresponding to the unique id
	 * 
	 * @returns robot's NetProfile
	 */ 
	NetProfile get_robot_info(int id);


	/**
     * Requests a unique id to the monitor. To be called in constructor. 
     */
	void request_unique_id();


	/**
	 * Provides the monitor net profile 
	 * 
	 * @returns monitor NetProfile
	 */
	NetProfile get_monitor_info();


private:
	std::unordered_map<int, NetProfile> net_list; // List of net profiles of other robots
	const int MONITOR_ID = 0;		              // Unique id of system's monitor
	NetProfile monitor_info; 			          // Monitor's net profiles
	
	/**
	 * Sends a message to a NetProfile.
	 * 
	 * @param m Reference to any Message child
	 * @param dst Reference to a NetProfile object
	 */
	void send_message(Message &m,NetProfile &dst);

};

#endif