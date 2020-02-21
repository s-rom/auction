#include "Point2D.h"
#include "MathUtil.h"
#include <sstream>
#include <algorithm>

#ifndef TASK
#define TASK

namespace Auction
{
    struct Task;
}


struct Auction::Task
{
    const float MAX_UTILITY; // Maximum utility of the task
    int task_id;             // Unique (for each execution) id of the task
    Point2D task_location;   // Location of the task object
    Point2D delivery_point;  // Location of the delivery point
    float task_work_load;    // Weigh units of the task object
    float dead_line;         // Deadline of the task, in milliseconds

    /**
     * Default constructor. 
     * 
     */
    Task()
    :
        MAX_UTILITY(1.0f)
    {

    }

    /**
     * Constructor.
     * 
     * @param location location of the task load
     * @param delivery location of the delivery point
     * @param workload weight (in kg) of the task load
     * @param dead_line deadline of the task (in ms) 
     * @param id unique task id
     */
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
     * Currently, a soft deadline function is implemented, see MathUtil.h
     * 
     * @param delta_time elapsed time from the beginning of the task
     */
    float utility_function(float delta_time)
    {
        return Auction::soft_deadline_utility(dead_line, MAX_UTILITY, delta_time);
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
                to_string(task_id) + delim +
                task_location.serialize(delim) + delim +
                delivery_point.serialize(delim) + delim +
                to_string(task_work_load) + delim +
                to_string(dead_line) + 
            delim; 
        return s;
    }


    Task& operator=(const Task& other)
    {
        this->dead_line = other.dead_line;
        this->delivery_point = other.delivery_point;
        this->task_id = other.task_id;
        this->task_location = other.task_location;
        this->task_work_load = other.task_work_load;
        return *this;
    }


    string to_string()
    {
        using std::to_string;
        string s;
        s = "Task "+to_string(task_id)+"\n"+
            "\tDelivery: "+delivery_point.to_string()+"\n"+
            "\tLocation: "+task_location.to_string()+"\n"+
            "\tWorkLoad: "+to_string(task_work_load)+" kg\n"+
            "\tDeadline: "+to_string((int)(dead_line/1000))+" s\n";
        return s;
    }

};

#endif