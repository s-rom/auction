/*
 * Copyright (c) 2012, Stefano Rosa, Luca Carlone
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VFH_NODE_H_
#define VFH_NODE_H_

#include "vfh_algorithm.h"
#include "Point2D.h"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>


#define DEG2RAD(a) ((a) * M_PI / 180.0)

std::string scan_topic_  = "base_scan";
std::string odom_topic_  = "odom";
std::string cmd_vel_topic_  = "cmd_vel";
std::string goal_topic_ = "goal";



class VFH_node
{
public:
	VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string robot_name);
	~VFH_node();
	void update();

	void set_goal(Auction::Point2D goal);
	void set_delivery(Auction::Point2D delivery);
	void set_total_travels(int travels);


	bool is_goal_completed();
	void force_stop();

private:

	VFH_Algorithm *m_vfh;

	float get_angle_fwd_dir();
	float get_last_yaw();
	void set_goal(tf2::Vector3 new_goal);

	tf2::Quaternion last_orientation;
	tf2::Vector3 last_position;
	tf2::Vector3 current_goal;

	tf2::Vector3 delivery;
	tf2::Vector3 target;

	int current_travels = 0;
	int total_travels = 0;

	bool has_valid_goal;
	bool rotating_in_place;

	float goal_tolerance = 0.5f;
	float rotation_tolerance = 4.0f;
	
	double m_cell_size;			// 100 mm
	int m_window_diameter;		// cells
	int m_sector_angle;			// in deg
	double m_safety_dist_0ms;
	double m_safety_dist_1ms;
	int m_max_speed;
	int m_max_speed_narrow_opening;
	int m_max_speed_wide_opening;
	int m_max_acceleration;
	int m_min_turnrate;
	int m_max_turnrate_0ms;
	int m_max_turnrate_1ms;
	double m_min_turn_radius_safety_factor;
	double m_free_space_cutoff_0ms;
	double m_obs_cutoff_0ms;
	double m_free_space_cutoff_1ms;
	double m_obs_cutoff_1ms;
	double m_weight_desired_dir;
	double m_weight_current_dir;

	double m_robot_radius;
	double m_robotVel;
    double m_laser_ranges[361][2];

	int chosen_speed,chosen_turnrate;

	


	// ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber scan_subscriber_;
    ros::Subscriber odom_subscriber_;
	ros::Subscriber goal_subscriber_;
    ros::Publisher  vel_publisher_;


    void goalCallback (const nav_msgs::Odometry::ConstPtr& goal_msg);
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg);
};

#endif /* VFH_NODE_H_ */
