#include "Monitor.h"
#include "WebService.h"


#include <boost/thread.hpp>
#include <boost/atomic/atomic.hpp>

#include <ros/ros.h>
#include <ros/master.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>

Auction::Monitor * monitor_ptr = nullptr;
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



void ros_polling_loop()
{
    ros::Rate rate(1); //Hz
    while (ros::ok())
    {
        ros::spinOnce();
        //rate.sleep();
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr & msg)
{
    //std::cout << "Odom received: "<<msg->pose.pose.position << "\n";
    if (monitor_ptr == nullptr) return;
    
    std::string frame_id = msg->header.frame_id;
    frame_id = frame_id.substr(frame_id.find('_')+1, 1);
    int id = std::stoi(frame_id);

    Auction::Point2D p(msg->pose.pose.position.x, msg->pose.pose.position.y);
   
    
    float x = msg->pose.pose.orientation.x;
    float y = msg->pose.pose.orientation.y;
    float z = msg->pose.pose.orientation.z;
    float w = msg->pose.pose.orientation.w;
    
    double siny_cosp = 2 * (w * z + x * y);
	double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    monitor_ptr->update_robot_pose(id, p, yaw);

}


int main(int argc, char ** argv)
{
    std::string program_path(argv[0]);
    
    ros::init(argc,argv,"monitor");
    
    ros::NodeHandle nh;
  
    ros::master::V_TopicInfo topic_infos;   
    ros::master::getTopics(topic_infos);

    // subscribers get released when out of scope
    std::vector<ros::Subscriber> odom_subs;
    
    for (const auto & topic : topic_infos){
        if (topic.name.find("odom") != std::string::npos){
            std::cout << "Subscribed to " <<topic.name<< std::endl;
            odom_subs.push_back(nh.subscribe(topic.name, 1, odom_callback));
        }
    }

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
    boost::thread ros_thread(&ros_polling_loop);


    ros_thread.join();
    server_thread.join();
    message_thread.join();
    web_service_thread.join();
}