#include "WebService.h"


using MonitorWS::MonitorApplication;

MonitorApplication::MonitorApplication(cppcms::service &srv)
:
    cppcms::application(srv)
{
    set_dispatcher_mappings();
}

void MonitorApplication::robot_info_response()
{
    response().set_header("Access-Control-Allow-Origin","*");

    Auction::Monitor & m = *(this->monitor);
    if (m.get_number_of_robots() == 0)
    {
        response().out() << "No hay robots registrados";
        return;
    }

    std::string resp = "{\"robots\":[";
    for (int id = 1; id <= m.get_number_of_robots(); id++)
    {
        resp += json_robot_info(id);

        if (id < m.get_number_of_robots()) resp += COMMA;
        
        resp += ENDL;

    }
    resp += "]}";

    response().out() << resp;
}

/*
    id
    host
    port
*/
std::string MonitorApplication::json_robot_info(int robot_id)
{
    if (robot_id <= 0) return nullptr;

    Auction::NetProfile netp = (monitor->message_system).get_robot_info(robot_id);
    Auction::RobotStatusInfo robot_info = monitor->robot_status[robot_id];
    std::vector<std::string> status_equivalent;
    status_equivalent.push_back("UNKNOWN");
    status_equivalent.push_back("ALIVE");
    status_equivalent.push_back("DEAD");

    std::string net_status = status_equivalent.at(robot_info.current_status); 


    std::string robot_json = "{";
    robot_json += get_json_int("id",robot_id) + COMMA + ENDL;
    robot_json += get_json_string("host",std::string(netp.host)) + COMMA + ENDL;
    robot_json += get_json_string("port",std::string(netp.port)) + COMMA + ENDL;
    robot_json += get_json_string("net_status", net_status) + ENDL;
    robot_json += "}";

    return robot_json;
}

std::string MonitorApplication::json_task_info(int task_id)
{
    if (task_id <= 0) return nullptr;

    auto & task_pair = monitor->task_list[task_id];
    Auction::Task & t = task_pair.first; 

    std::vector<std::string> status_names;
    status_names.push_back("WAITING");
    status_names.push_back("AUCTIONING");
    status_names.push_back("CONDUCTING");
    status_names.push_back("COMPLETED");


    std::string task_json = "{";
    task_json += get_json_int("id",task_id) + COMMA + ENDL;
    task_json += get_json_float("workload", t.task_work_load) + COMMA + ENDL;
    task_json += get_json_string("delivery", t.delivery_point.to_string()) + COMMA + ENDL;
    task_json += get_json_string("goal", t.task_location.to_string()) + COMMA + ENDL;
    task_json += get_json_float("deadline", t.dead_line) + COMMA + ENDL;
    task_json += get_json_string("status", status_names[task_pair.second]) + ENDL;
    task_json += "}";
    return task_json;
}


std::string MonitorApplication::json_robot_pose_info(int id)
{
    if (id < 0) return "";
    auto &pair =monitor->robot_points[id];
    Auction::Point2D pose = pair.first;
    float yaw = pair.second / M_PI * 180.0f;

    std::string pose_json = "{";
    pose_json += get_json_float("x", pose.x)+COMMA+ENDL;
        pose_json += get_json_float("y", pose.y)+COMMA+ENDL;
    pose_json += get_json_float("yaw", yaw) + ENDL;
    pose_json += "}";

    return pose_json;
}

void MonitorApplication::robot_pose_response()
{
    response().set_header("Access-Control-Allow-Origin","*");
    std::string resp = "{\"positions\":[";

    if (monitor->robot_points.size()== 0)
    {
        response().out() << "No hay robots en el sistema";
        return;
    }


    int count = 0;
    for (const auto & record : monitor->robot_points)
    {
        int id = record.first;
        resp += this->json_robot_pose_info(id);
     
        if (count < monitor->robot_points.size()-1)  resp+= COMMA;

        count++;
    }

    resp += "]}";

    response().out() << resp;
}



void MonitorApplication::task_info_response()
{
    response().set_header("Access-Control-Allow-Origin","*");

    Auction::Monitor & m = *(this->monitor);
    if (m.task_list.size() == 0)
    {
        response().out() << "No hay tareas en el sistema";
        return;
    }

    std::string resp = "{\"tasks\":[";
    for (int id = 1; id <= m.get_number_of_tasks(); id++)
    {
        resp += json_task_info(id);

        if (id < m.get_number_of_tasks()) resp += COMMA;
        
        resp += ENDL;

    }
    resp += "]}";

    response().out() << resp;

}

void MonitorApplication::robot_kill_response(std::string id_str)
{    
    response().set_header("Access-Control-Allow-Origin","*");
    int id = std::stoi(id_str);

    Auction::SimpleMessage * kill_robot = 
        new Auction::SimpleMessage(Auction::Task::NULL_TASK, id, Auction::MessageType::ROBOT_KILL);
    monitor->message_queue.push(kill_robot); 

    response().out() << kill_robot->robot_src; 
}


void MonitorApplication::new_task_response(std::string serialized_task)
{
    response().set_header("Access-Control-Allow-Origin","*");
    std::cout << "[WebService]: New task requested by user ==> " << serialized_task << "\n";
    
    auto end = serialized_task.end();
    Auction::DeadlineType deadline_type = *(end-2) == '0'? 
        Auction::DeadlineType::SOFT : Auction::DeadlineType::HARD;


    Auction::Task t(serialized_task, '_');
    t.set_deadline_type(deadline_type);

    response().out() << t.to_string() << "\n";
    std::cout << "[WebService]: \n"<< t.to_string()<<"\n";

    Auction::NewTaskMessage * new_task_msg = new Auction::NewTaskMessage(t);
    this->monitor->message_queue.push(new_task_msg);
}

void MonitorApplication::set_dispatcher_mappings()
{
    dispatcher().assign("/get_robots_info",&MonitorApplication::robot_info_response, this);  
    mapper().assign("get_robots_info","/get_robots_info");  

    dispatcher().assign("/new_task/(.+)", &MonitorApplication::new_task_response, this, 1); 
    mapper().assign("new_task","/new_task/{1}");  

    dispatcher().assign("/robot_kill/(\\d+)", &MonitorApplication::robot_kill_response, this, 1); 
    mapper().assign("robot_kill","/robot_kill/{1}");  

    dispatcher().assign("/get_tasks_info",&MonitorApplication::task_info_response, this);  
    mapper().assign("get_tasks_info","/get_tasks_info");

    dispatcher().assign("/get_robots_positions",&MonitorApplication::robot_pose_response, this);  
    mapper().assign("get_robots_positions","/get_robots_positions");



    mapper().root("/monitor");
}

void MonitorApplication::set_monitor_pointer(Auction::Monitor * monitor)
{
    this->monitor = monitor;
}


std::string MonitorApplication::get_json_string(std::string var, std::string value)
{
    return "\""+var+"\": \""+value+"\"";
}

std::string MonitorApplication::get_json_int(std::string var, int value)
{
    return "\""+var+"\": "+std::to_string(value);
}

std::string MonitorApplication::get_json_float(std::string var, float value)
{
    return "\""+var+"\": "+std::to_string(value);
}