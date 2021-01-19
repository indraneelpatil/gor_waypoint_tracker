// Created by Indraneel on 11/10/20

#ifndef GLOBAL_WAYPOINTS_ACCEPTOR_H
#define GLOBAL_WAYPOINTS_ACCEPTOR_H

// ROS_LIBS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <global_waypoints_acceptor/GlobalWaypointsAcceptor.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <tf/tf.h>

// GLOBAL_PLANNER INTERFACE
/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>


namespace GorGlobalWaypointsAcceptor {
  
class GlobalWaypointsAcceptor : public nav_core::BaseGlobalPlanner {
public:
    GlobalWaypointsAcceptor (ros::NodeHandle &); 
    GlobalWaypointsAcceptor ();
    GlobalWaypointsAcceptor(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  
    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start, 
		const geometry_msgs::PoseStamped& goal, 
		std::vector<geometry_msgs::PoseStamped>& plan
	       );

    // Parameters
    double max_waypoint_dist;


private:
    ros::NodeHandle ROSNodeHandle;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
    bool waypoints_rcvd;
    std::vector<geometry_msgs::PoseStamped> waypoints_vec;
    ros::ServiceServer gwa_service;
    ros::Publisher global_plan_pub_;

    bool gwa_callback(global_waypoints_acceptor::GlobalWaypointsAcceptor::Request& request, \
                        global_waypoints_acceptor::GlobalWaypointsAcceptor::Response& response);
    double EucDistance(double x1, double y1, double x2, double y2);

};

};
#endif


