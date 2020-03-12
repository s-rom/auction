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
    response().out() << "hola";
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
