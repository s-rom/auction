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
    std::cout << "[WebService] Received get_robots_info petition\n";
    
    // response().set_header("Access-Control-Allow-Headers","Content-Type");    
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

    std::string robot_json = "{";
    robot_json += get_json_int("id",robot_id) + COMMA + ENDL;
    robot_json += get_json_string("host",std::string(netp.host)) + COMMA + ENDL;
    robot_json += get_json_string("port",std::string(netp.port)) + ENDL;
    robot_json += "}";

    return robot_json;
}

void MonitorApplication::task_info_response()
{

}

void MonitorApplication::new_task_response()
{

}

void MonitorApplication::set_dispatcher_mappings()
{
    dispatcher().assign("/get_robots_info",&MonitorApplication::robot_info_response, this);  
    mapper().assign("get_robots_info","/get_robots_info");  

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