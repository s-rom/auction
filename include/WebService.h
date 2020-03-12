#ifndef MONITOR_WS
#define MONITOR_WS

#include <cppcms/application.h>
#include <cppcms/service.h>
#include <cppcms/http_response.h>
#include <cppcms/url_dispatcher.h>
#include <cppcms/url_mapper.h>
#include <cppcms/applications_pool.h>
#include <iostream>

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
    void new_task_response();

    void set_monitor_pointer(Auction::Monitor * monitor);

private:
    void set_dispatcher_mappings();
    Auction::Monitor * monitor;
};


#endif