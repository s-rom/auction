#include "Monitor.h"
#include "WebService.h"


#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

Auction::Monitor * monitor_ptr;
cppcms::service * service_ptr;
boost::atomic<bool> running(true);


void init_ws(cppcms::service & srv)
{
    try 
    {  
        std::cout << "[WebService] MonitorWS running in background...\n";
        srv.run();
    }
    catch(std::exception const & e)
    {
        std::cerr << e.what() << std::endl;
    }
}

void sigint_handler(int signal)
{
    Auction::InfoReporter & info_report = monitor_ptr->info_report;
    info_report << "[Monitor] <--- killed by SIGINT" << "\n";
    monitor_ptr->info_report.close();
    service_ptr->shutdown();
    running = false;
    exit(0);
}

int main(int argc, char ** argv)
{
    std::string program_path(argv[0]);

    Auction::Monitor m(program_path);
    monitor_ptr = &m;


    cppcms::service srv(argc, argv);
    
    service_ptr = &srv;

    booster::intrusive_ptr<MonitorWS::MonitorApplication> app_ptr = new MonitorWS::MonitorApplication(srv);
    srv.applications_pool().mount(app_ptr);

    app_ptr->set_monitor_pointer(monitor_ptr);

    signal(SIGINT, sigint_handler);

    boost::thread web_service_thread(&init_ws, boost::ref(srv));
    boost::thread server_thread(&Auction::Monitor::message_server, &m, boost::ref(running));
    boost::thread message_thread(&Auction::Monitor::message_processor, &m, boost::ref(running));
    server_thread.join();
    message_thread.join();
    web_service_thread.join();
}