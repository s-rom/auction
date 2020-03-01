#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>
#include <signal.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void sigintHandler(int sig)
{
    ROS_INFO("Process killed by user. Shuting down node.");
    ros::shutdown(); 
}


MoveBaseClient * ac;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    signal(SIGINT, sigintHandler);

    MoveBaseClient aux("move_base", true);
    ac = &aux;


    while (!ac->waitForServer(ros::Duration(2.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    
    double goal_x, goal_y, goal_w=1.0;
    if (argc == 4) //nombre x y w
    {
        goal_x = std::stod(argv[1]);
        goal_y = std::stod(argv[2]);
        goal_w = std::stod(argv[3]);
    }
    else 
    {
        ROS_INFO_STREAM("Params: rosrun package node x y w");
        exit(0);
    }


    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "/map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goal_x;
    goal.target_pose.pose.position.y = goal_y;
    goal.target_pose.pose.orientation.w = goal_w;

    ROS_INFO_STREAM("Enviando objetivo a (" << goal_x << ", " << goal_y << ") "
        << "respecto a SDC: " << goal.target_pose.header.frame_id);
    ac->sendGoal(goal);
    ac->waitForResult();

    if (ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Goal achieved!");
    }
    else
    {
        ROS_INFO("Goal not achieved");
    }
}