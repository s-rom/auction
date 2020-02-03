#include "Point2D.h"
#include <sstream>
#include <algorithm>
#pragma once

namespace Auction
{
    struct Task;
}


struct Auction::Task
{
    const float MAX_UTILITY; // Maximum utility of the task
    float task_id;           // Unique (for each execution) id of the task
    Point2D task_location;   // Location of the task object
    Point2D delivery_point;  // Location of the delivery point
    float task_work_load;    // Weigh units of the task object
    float dead_line;         // Deadline of the task, in milliseconds

    Task()
    :
        MAX_UTILITY(1.0f)
    {

    }

    Task(Point2D location, Point2D delivery, float workload, float dead_line, float id)
    :
        task_location(location),
        delivery_point(delivery),
        task_work_load(workload),
        dead_line(dead_line),
        task_id(id),
        MAX_UTILITY(1.0f)
    {}



    /**
     * Returns the task utility Uj in function of the elapsed time.
     *  Currently, a hard deadline function is implemented, where:
     *      -> Uj = MAX_UTILITY  if delta_time < dead_line
     *      -> Uj = 0            otherwise
     *  Uj (tj) where tj is the time elapsed since the task started (delta_time)
     */
    float utility_function(float delta_time)
    {
        if (delta_time < dead_line) return MAX_UTILITY;
        else return 0;
    }


    /**
     * Creates a Task given a serialized string
     * 
     * Format:  
     * #0#task_id#location#delivery#workload#dead_line#
     */
    Task(string serialized_message, char delim)
    :
        MAX_UTILITY(1.0f)
    {
        using std::getline;
		std::stringstream ss(serialized_message);
		string token;
		getline(ss,token,delim); // DELIM
        getline(ss,token,delim); // type <--> 0
        getline(ss,token,delim); // task_id
		task_id = std::stoi(token);

        getline(ss,token,delim); // location x
		task_location.x = std::stof(token);
        getline(ss,token,delim); // location y
        task_location.y = std::stof(token);

        getline(ss,token,delim); // delivery x
    	delivery_point.x = std::stof(token);
        getline(ss,token,delim); // delivery y
		delivery_point.y = std::stof(token);

        getline(ss,token,delim); // task_workload
		task_work_load = std::stof(token);
        getline(ss,token,delim); // dead_line
		dead_line = std::stof(token);
    }

    /**
     * Creates a serialized string of this Task
     * 
     * Format:  
     * #0#task_id#location#delivery#workload#dead_line#
     */
    string serialize(char delim)
    {
        using std::to_string;
        string s;
        s = delim +
            to_string(0) + delim +
            to_string(task_id) + 
            task_location.serialize(delim) +
            delivery_point.serialize(delim) + 
            to_string(task_work_load) + delim +
            to_string(dead_line) + delim; 
        return s;
    }


};

