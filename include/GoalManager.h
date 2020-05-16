#ifndef AUCTION_GOALMANAGER
#define AUCTION_GOALMANAGER


#include "Point2D.h"
#include "InfoReporter.h"

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>
#include <string>


namespace Auction
{
    class GoalManager;
}


class Auction::GoalManager 
{
public:
    GoalManager(ros::Publisher goal_pub);
    GoalManager(ros::Publisher goal_pub, std::string info_path);

    void set_goal(Auction::Point2D goal);
    void set_delivery(Auction::Point2D delivery);
    void update_position(Auction::Point2D position);
    void set_total_travels(int travels);
    void cancel_goal();
    void close_info_reporter();
    void goal_loop();
    bool is_goal_completed();

private:
    bool goal_valid;
    int total_travels;
    int current_travels;
    float tolerance = 1.0f;

    // std::string scan_topic_  = "base_scan";
    // std::string odom_topic_  = "odom";
    // std::string cmd_vel_topic_  = "cmd_vel";
    // std::string goal_topic_ = "goal";

    ros::Publisher goal_pub;

    Auction::Point2D goal;
    Auction::Point2D delivery;

    Auction::Point2D current;
    boost::mutex m;

    Auction::InfoReporter info_report;
};


#endif // AUCTION_GOALMANAGER