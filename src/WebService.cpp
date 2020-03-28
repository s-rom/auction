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

void MonitorApplication::task_info_response()
{

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

    dispatcher().assign("/new_task/(.+)",
        &MonitorApplication::new_task_response, this, 1); 
    mapper().assign("new_task","/new_task/{1}");  

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