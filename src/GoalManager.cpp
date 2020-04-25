#include "GoalManager.h"

using namespace Auction;

GoalManager::GoalManager(MoveBaseClient * moveBaseClient)
:
    moveBaseClient(moveBaseClient)
{
    total_travels = 0;
    current_travels = 0;
    goal_valid = false;
}

GoalManager::GoalManager(MoveBaseClient * moveBaseClient, std::string info_path)
:
    moveBaseClient(moveBaseClient),
    info_report(info_path)
{
    total_travels = 0;
    current_travels = 0;
    goal_valid = false;
}

bool GoalManager::is_goal_completed()
{
    return this->current_travels == this->total_travels;
}

void GoalManager::set_goal(Auction::Point2D goal)
{
    this->goal = goal;
    this->goal_valid = true;
    info_report << "[GoalManager] Set new goal: " << this->goal.to_string() << "\n";
}


void GoalManager::set_delivery(Auction::Point2D delivery)
{
    this->delivery = delivery;
    info_report << "[GoalManager] Set new delivery: " << this->delivery.to_string() << "\n";
}


void GoalManager::update_position(Auction::Point2D position)
{
    this->m.lock();
    this->current = boost::atomic<Auction::Point2D>(position);
    this->m.unlock();
}


void GoalManager::set_total_travels(int travels)
{
    info_report << "[GoalManager] Total travels: "<<2*travels<<"\n";
    this->total_travels = (travels >= 0) ? (2 * travels) : 0;
} 


void GoalManager::cancel_goal()
{
    info_report << "[GoalManager] Goal canceled\n";
    this->goal_valid = false;
    moveBaseClient->cancelGoal();
}


void GoalManager::close_info_reporter()
{
    this->info_report.close();
}


void GoalManager::set_map_frame(std::string map_frame)
{
    this->map_frame = map_frame;
}


void GoalManager::goal_loop()
{
    while (!moveBaseClient->waitForServer(ros::Duration(2.0)));

    info_report << "[GoalLoop] running...\n";

    while(true)
    {
        if (!goal_valid) continue;
        if (total_travels == 0) continue;
        if (is_goal_completed())
        {
            info_report << "[GoalLoop] The goal management is completed "
                << "with " << total_travels / 2 << " travels between delivery and goal point\n";

            goal_valid = false;
            current_travels = 0;
            total_travels = 0;
            continue;
        }

        info_report << "[GoalLoop] current: " << current.to_string()
            << ", current_travels: " << this->current_travels << "\n";

        Auction::Point2D target;

        if (current_travels % 2 == 0)
        {
            if (current_travels == 0)
                info_report << "[GoalLoop] First travel set to goal\n";
            target = goal;   
        }
        else 
        {
            target = delivery;
        }

        move_base_msgs::MoveBaseGoal goal_msg;
        goal_msg.target_pose.header.frame_id = this->map_frame;
        goal_msg.target_pose.header.stamp = ros::Time::now();

        goal_msg.target_pose.pose.position.x = target.x;
        goal_msg.target_pose.pose.position.y = target.y;
        goal_msg.target_pose.pose.orientation.w = 1;

        info_report << "[Goal Loop] Sending goal to "<<target.x<<","<<target.y<<"\n";
        moveBaseClient->sendGoal(goal_msg);
        if (moveBaseClient->waitForResult())
        {
            info_report << "[Goal Loop] Goal achieved, incrementing travels\n";
            this->current_travels++;
        }
        
    }
}