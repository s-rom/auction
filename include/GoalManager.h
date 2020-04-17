#ifndef AUCTION_GOALMANAGER
#define AUCTION_GOALMANAGER

#include "Point2D.h"
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>

namespace Auction
{
    class GoalManager;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
}


class Auction::GoalManager 
{
public:
    GoalManager(Auction::MoveBaseClient * moveBaseClient);

    void set_goal(Auction::Point2D goal);
    void set_delivery(Auction::Point2D delivery);
    void set_travels(int travels);
    void cancel_goal();

    void goal_loop();

private:

    bool goal_valid;
    Auction::Point2D goal;
    Auction::Point2D delivery;
    Auction::MoveBaseClient * moveBaseClient;

};


#endif // AUCTION_GOALMANAGER