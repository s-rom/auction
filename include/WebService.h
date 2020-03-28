#ifndef MONITOR_WS
#define MONITOR_WS

#include <cppcms/application.h>
#include <cppcms/service.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/url_mapper.h>
#include <cppcms/applications_pool.h>
#include <iostream>
#include <string>


#include "Monitor.h"


namespace MonitorWS
{
    class MonitorApplication;
}


class MonitorWS::MonitorApplication : public cppcms::application
{
public:

    MonitorApplication(cppcms::service &srv);

    void robot_info_response();
    void task_info_response();
    void new_task_response(std::string serialized_task);

    void set_monitor_pointer(Auction::Monitor * monitor);

private:

    std::string get_json_string(std::string var, std::string value);
    std::string get_json_int(std::string var, int value);
    std::string get_json_float(std::string var, float value);

    std::string json_robot_info(int robot_id);

    const std::string COMMA = ",";
    const std::string ENDL = "\n";

    void set_dispatcher_mappings();
    Auction::Monitor * monitor;
};


#endif