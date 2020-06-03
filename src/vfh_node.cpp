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

#include "vfh_node.h"


VFH_node::VFH_node(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string robot_name):
nh_(nh), nh_private_(nh_private)
{
	ROS_INFO("Starting VFH");

	rotating_in_place = true;
	has_valid_goal = false;
	m_cell_size = 100;							// mm, cell dimension
	m_window_diameter = 60;						// number of cells
	m_sector_angle = 5;							// deg, sector angle

	if (!nh_private_.getParam ("m_safety_dist_0ms", m_safety_dist_0ms))
		m_safety_dist_0ms = 100; 				// mm, double, safe distance at 0 m/s

	if (!nh_private_.getParam ("m_safety_dist_1ms", m_safety_dist_1ms))
		m_safety_dist_1ms = 100; 				// mm, double, safe distance at 1 m/s

	if (!nh_private_.getParam ("m_max_speed", m_max_speed))
		m_max_speed= 200;						// mm/sec, int, max speed

	if (!nh_private_.getParam ("m_max_speed_narrow_opening", m_max_speed_narrow_opening))
		m_max_speed_narrow_opening= 200; 		// mm/sec, int, max speed in the narrow opening

	if (!nh_private_.getParam ("m_max_speed_wide_opening", m_max_speed_wide_opening))
		m_max_speed_wide_opening= 300; 			// mm/sec, int, max speed in the wide opening

	if (!nh_private_.getParam ("m_max_acceleration", m_max_acceleration))
		m_max_acceleration = 200;    			// mm/sec^2, int, max acceleration

	if (!nh_private_.getParam ("m_min_turnrate", m_min_turnrate))
		m_min_turnrate = 40;	 				// deg/sec, int, min turn rate <--- not used

	if (!nh_private_.getParam ("m_max_turnrate_0ms", m_max_turnrate_0ms))
		m_max_turnrate_0ms = 40;				// deg/sec, int, max turn rate at 0 m/s

	if (!nh_private_.getParam ("m_max_turnrate_1ms", m_max_turnrate_1ms))
		m_max_turnrate_1ms = 40;				// deg/sec, int, max turn rate at 1 m/s

	m_min_turn_radius_safety_factor = 1.0; 		// double ????

	if (!nh_private_.getParam ("m_free_space_cutoff_0ms", m_free_space_cutoff_0ms))
		m_free_space_cutoff_0ms = 2000000.0; 	//double, low threshold free space at 0 m/s

	if (!nh_private_.getParam ("m_obs_cutoff_0ms", m_obs_cutoff_0ms))
		m_obs_cutoff_0ms = 4000000.0;			//double, high threshold obstacle at 0 m/s

	if (!nh_private_.getParam ("m_free_space_cutoff_1ms", m_free_space_cutoff_1ms))
		m_free_space_cutoff_1ms = 2000000.0; 	//double, low threshold free space at 1 m/s

	if (!nh_private_.getParam ("m_obs_cutoff_1ms", m_obs_cutoff_1ms))
		m_obs_cutoff_1ms = 4000000.0;			//double, high threshold obstacle at 1 m/s

	if (!nh_private_.getParam ("m_weight_desired_dir", m_weight_desired_dir))
		m_weight_desired_dir = 5.0;				//double, weight desired direction

	if (!nh_private_.getParam ("m_weight_current_dir", m_weight_current_dir))
		m_weight_current_dir = 1.0;				//double, weight current direction

	if (!nh_private_.getParam ("m_robot_radius", m_robot_radius))
		m_robot_radius = 300.0;					// robot radius in mm

	if (!nh_private_.getParam ("scan_topic", scan_topic_))
		scan_topic_ = "base_scan";				// scan topic name

	if (!nh_private_.getParam ("odom_topic", odom_topic_))
		odom_topic_ = "odom";					// odom topic name

	if (!nh_private_.getParam ("cmd_vel_topic", cmd_vel_topic_))
		cmd_vel_topic_ = "cmd_vel";				// cmd topic name


	if (!nh_private_.getParam ("goal_topic", cmd_vel_topic_))
		goal_topic_ = "goal";					// goal topic name

	if (!nh_private_.getParam ("rotation_tolerance", rotation_tolerance))
		rotation_tolerance = 4.0f;

	if (!nh_private_.getParam ("goal_tolerance", goal_tolerance))
		goal_tolerance = 1.0f;

	m_vfh = new VFH_Algorithm(m_cell_size, m_window_diameter, m_sector_angle,
			m_safety_dist_0ms, m_safety_dist_1ms, m_max_speed,
			m_max_speed_narrow_opening, m_max_speed_wide_opening,
			m_max_acceleration, m_min_turnrate, m_max_turnrate_0ms,
			m_max_turnrate_1ms, m_min_turn_radius_safety_factor,
			m_free_space_cutoff_0ms, m_obs_cutoff_0ms, m_free_space_cutoff_1ms,
			m_obs_cutoff_1ms, m_weight_desired_dir, m_weight_current_dir);

	m_vfh->SetRobotRadius(m_robot_radius);
	m_vfh->Init();


	// scan_topic_ = robot_name + scan_topic_;
	// odom_topic_ = robot_name + odom_topic_;
	
	std::cout << "scan_topic: "<<scan_topic_<<"\n";
	std::cout << "odom_topic: "<<odom_topic_<<"\n";



	// subscribe to topics
	goal_subscriber_ = nh_.subscribe(
			goal_topic_, 1, &VFH_node::goalCallback, this);
	scan_subscriber_ = nh_.subscribe(
			scan_topic_, 1, &VFH_node::scanCallback, this);
	odom_subscriber_ = nh_.subscribe(
			odom_topic_, 1, &VFH_node::odomCallback, this);
	// cmd_vel publisher
	vel_publisher_= nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_,5);

}

VFH_node::~VFH_node()
{
	force_stop();
	delete m_vfh;
}

void VFH_node::force_stop()
{
	has_valid_goal = false;
	geometry_msgs::Twist cmd_vel;
	cmd_vel.linear.x=0.0;
	cmd_vel.angular.z=0.0;
	vel_publisher_.publish(cmd_vel);
}

void VFH_node::goalCallback (const nav_msgs::Odometry::ConstPtr& goal_msg)
{
	tf2::convert(goal_msg->pose.pose.position, current_goal);
	has_valid_goal = true;
}

bool VFH_node::is_goal_completed()
{
	return current_travels == total_travels;
}

void VFH_node::odomCallback (const nav_msgs::Odometry::ConstPtr& odom_msg)
{
	ROS_DEBUG("odomCallback(): received odometry");
	
	tf2::convert(odom_msg->pose.pose.position, this->last_position);
	tf2::convert(odom_msg->pose.pose.orientation, this->last_orientation);

	if (!has_valid_goal || total_travels == 0){
		force_stop();
		return;
	}

	if (is_goal_completed()){
		// info_report << "[GoalLoop] The goal management is completed "
		// 	<< "with " << total_travels / 2 << " travels between delivery and goal point\n";

		has_valid_goal = false;
		current_travels = 0;
		total_travels = 0;
		return;
	}
	
	float distTar = tf2::tf2Distance2(last_position, target);
	float distDel = tf2::tf2Distance2(last_position, delivery);


	// if (current_travels % 2 == 0)
    // {
	// 	current_goal = target;   
	// }			   
	// else 
	// {
	// 	current_goal = delivery;
	// }
	

	if (current_goal == target && distTar < goal_tolerance)
	{
		current_goal = delivery;
		rotating_in_place = true;
		current_travels++;
	}

	if (current_goal == delivery && distDel < goal_tolerance)
	{
		current_travels++;
		current_goal = target;	
		rotating_in_place = true;
	}


	float desiredAngle = get_angle_fwd_dir();
	// rotating_in_place = std::abs(desiredAngle) > rotation_tolerance;
	rotating_in_place = false;
	

	// ROS_INFO_STREAM("last pos: "<<last_position.x() << ","<<last_position.y());
	// ROS_INFO_STREAM("last orientation: "<<last_orientation.getAngle() * 180.0f / M_PI);
	m_robotVel = odom_msg->twist.twist.linear.x * 1000.0;
}


void VFH_node::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	ROS_DEBUG("scanCallback(): received scan, ranges %d",scan_msg->ranges.size());

	unsigned int n = scan_msg->ranges.size();
	for (unsigned i = 0; i < 361; i++)
		m_laser_ranges[i][0] = -1;

	int step=1;
	int startIndex=0;
	float laserSpan = scan_msg->angle_max - scan_msg->angle_min;

	if(laserSpan > M_PI || n>180) // in case we are using HOKUYO
	{
		startIndex = (- M_PI/2 - scan_msg->angle_min) /scan_msg->angle_increment;
		float rays_per_degree = (M_PI/180.0)/scan_msg->angle_increment;
		ROS_DEBUG("scanCallback(): startIndex %d, raysxdeg %f", startIndex, rays_per_degree);
		for (unsigned i = 0; i<180; i++)
		{
			step = int(rays_per_degree * i);
			// calculate position in laser frame
			if (startIndex+step > n-1) // probably this is not necessary :/
				step = step-1;

			double r = scan_msg->ranges[startIndex+step]*1000.0;

			if (r<10)
				r = scan_msg->range_max *1000.0;

			ROS_DEBUG("%d:%f\n",i,r);
			m_laser_ranges[i*2][0] = r;
			m_laser_ranges[i*2 + 1][0] = r;
		}
	}
	else
	{
		for (unsigned i = 0; i<180; i++) // in case we are using SICK
		{
			// calculate position in laser frame
			double r = scan_msg->ranges[i]*1000.0;
			m_laser_ranges[i*2][0] = r;
			m_laser_ranges[i*2 + 1][0] = r;
		}
	}

	// perform vfh+
	update();
}

void VFH_node::set_total_travels(int travels)
{
	this->total_travels = (travels >= 0) ? (2 * travels) : 0;
}


void VFH_node::set_goal(Auction::Point2D goal)
{
	this->has_valid_goal = true;
	this->target.setX(goal.x);
	this->target.setY(goal.y);
	this->target.setZ(0);

	this->current_travels = 0;
	this->current_goal = target;
	ROS_INFO_STREAM("New goal set: "<<target.x()<<", "<<target.y());
}

void VFH_node::set_delivery(Auction::Point2D delivery)
{
	this->delivery.setX(delivery.x);
	this->delivery.setY(delivery.y);
	this->delivery.setZ(0);
	ROS_INFO_STREAM("New delivery set: "<<this->delivery.x()<<", "<<this->delivery.y());
}


/**
 * @returns yaw angle in range [-pi, pi]
 */
float VFH_node::get_last_yaw()
{
	/* 
		x --- q0 
	   	y --- q1
	   	z --- q2
	   	w --- q3
	*/

	tf2::Quaternion q = last_orientation;
	// 2 * (q3 q2 + q0 q1)
	double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
	// 1 - 2 * (q1^2 + q2^2)
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    double yaw = std::atan2(siny_cosp, cosy_cosp);
	return yaw;
}

/**
 * 
 * @returns the angle between forward and direction
 *	(pointing from robot to current goal) in range [0, 360] 
 */
float VFH_node::get_angle_fwd_dir()
{
	tf2::Vector3 dir = current_goal - last_position;
	dir.setZ(0.0f);
	dir.normalize();

	// forward vector
	float curr_rad = get_last_yaw();
	tf2::Vector3 forward
	(
		std::cos(curr_rad),
		std::sin(curr_rad),
		0
	);

	
	float angleBetween = tf2::tf2Angle(dir, forward);
	angleBetween *= 180.0f / M_PI;
	return angleBetween;
}


void VFH_node::update()
{

	if (!has_valid_goal)
	{	
		return;
	} 	

	float desiredAngle = get_angle_fwd_dir() + 90;
	float desiredDist=100000.0;
	float currGoalDistanceTolerance=250;

	m_vfh->Update_VFH(
		m_laser_ranges, 
		(int) (m_robotVel), 
		desiredAngle,
		desiredDist, 
		currGoalDistanceTolerance,
		chosen_speed,
		chosen_turnrate
	);


	

	geometry_msgs::Twist cmd_vel;

	// ROS_INFO("---------------------------------------------");
	// ROS_INFO_STREAM("Desired: "<<desiredAngle);
	// ROS_INFO_STREAM("Yaw: "<<get_last_yaw() * 180.0f / M_PI);
	// ROS_INFO("---------------------------------------------\n");
	// ROS_INFO_STREAM("Current goal: "<<current_goal.x()<<", "<<current_goal.y());

	int turn_dir = 1;
	if (this->rotating_in_place)
		cmd_vel.linear.x = 0;
	else 
	{
		cmd_vel.linear.x=(float)(chosen_speed)/1000.0;
	}
	
	cmd_vel.angular.z= DEG2RAD(turn_dir * chosen_turnrate);
	vel_publisher_.publish(cmd_vel);

	ROS_DEBUG("chosen_speed %d, chosen_turnrate %d", chosen_speed,
			chosen_turnrate);

}

// int main(int argc, char** argv)
// {
// 	ros::init(argc, argv, "VFH");
// 	ros::NodeHandle nh;
// 	ros::NodeHandle nh_private("~");
// 	VFH_node vfh_node(nh,nh_private);
// 	ros::spin();

// 	return 0;
// }
