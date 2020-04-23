#ifndef AUCTION_GOALMANAGER
#define AUCTION_GOALMANAGER


#include "Point2D.h"
#include "InfoReporter.h"

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>


namespace Auction
{
    class GoalManager;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
}


class Auction::GoalManager 
{
public:
    GoalManager(Auction::MoveBaseClient * moveBaseClient);
    GoalManager(Auction::MoveBaseClient * moveBaseClient, std::string info_path);

    void set_goal(Auction::Point2D goal);
    void set_delivery(Auction::Point2D delivery);
    void update_position(Auction::Point2D position);
    void set_total_travels(int travels);
    void cancel_goal();
    void close_info_reporter();

    void goal_loop();

private:

    bool goal_valid;
    int total_travels;
    int current_travels;
    float tolerance = 2.0f;

    Auction::MoveBaseClient * moveBaseClient;

    Auction::Point2D goal;
    Auction::Point2D delivery;


    Auction::Point2D current;
    boost::mutex m;

    Auction::InfoReporter info_report;
};


#endif // AUCTION_GOALMANAGER