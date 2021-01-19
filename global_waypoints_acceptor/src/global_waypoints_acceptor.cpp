// Created by Indraneel on 11/10/20

#include "global_waypoints_acceptor.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(GorGlobalWaypointsAcceptor::GlobalWaypointsAcceptor, nav_core::BaseGlobalPlanner)


namespace GorGlobalWaypointsAcceptor
{

//Default Constructor
GlobalWaypointsAcceptor::GlobalWaypointsAcceptor():initialized_(false)
{

}
GlobalWaypointsAcceptor::GlobalWaypointsAcceptor(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;

}

GlobalWaypointsAcceptor::GlobalWaypointsAcceptor(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}

void GlobalWaypointsAcceptor::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
      if (!initialized_)
  {
      ros::NodeHandle nh;
      ros::NodeHandle private_nh("~/" + name);
      gwa_service = private_nh.advertiseService("waypoints_acceptor_service", &GlobalWaypointsAcceptor::gwa_callback,this);
      global_plan_pub_ = private_nh.advertise<geometry_msgs::PoseArray>("global_plan", 1);

      // Load parameters
      private_nh.param("max_waypoint_dist", max_waypoint_dist, 50.0);

      ROS_INFO("[GWA] Global Waypoints Acceptor initialized successfully!");

      // Initial declarations
      initialized_ = true;
      waypoints_rcvd = false;
  }
  else
    ROS_WARN("[GWA] This planner has already been initialized... doing nothing");

}

bool GlobalWaypointsAcceptor::gwa_callback(global_waypoints_acceptor::GlobalWaypointsAcceptor::Request& request, \
                    global_waypoints_acceptor::GlobalWaypointsAcceptor::Response& response)
{
  if(!waypoints_rcvd)
  {
    waypoints_rcvd = true;
    ROS_INFO("[GWA] Waypoints acceptor received %d waypoints in %s frame",(int)(request.num_waypoints),(request.frame_id));
    waypoints_vec.clear();

    geometry_msgs::PoseStamped waypoint;
    waypoint.header.frame_id = request.frame_id;
    waypoint.header.stamp = ros::Time::now();
    for(int i=0;i<request.num_waypoints;i++)
    {
      waypoint.pose.position.x = request.waypoints[i].x;
      waypoint.pose.position.y = request.waypoints[i].y;
      waypoint.pose.position.z = 0.0f;

      tf::Quaternion simple_quat = tf::Quaternion();
      simple_quat.setEuler(request.waypoints[i].theta,0.0f,0.0f); 
      waypoint.pose.orientation.x = simple_quat.getX();
      waypoint.pose.orientation.y = simple_quat.getY();
      waypoint.pose.orientation.z = simple_quat.getZ();
      waypoint.pose.orientation.w = simple_quat.getW();

      waypoints_vec.push_back(waypoint);
    }

    response.result = true;
    return true;
  }
  else
  {
    // @TODO
  }
}

bool GlobalWaypointsAcceptor::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("[GWA] Global Waypoints Acceptor has not been initialized, please call initialize() and then retry");
    return false;
  }

  if(waypoints_rcvd)
  {
    plan.push_back(start);
    waypoints_vec.push_back(goal);
    waypoints_rcvd = false;
    // Downsample waypoints if needed and add them to the plan
    for(int it=0;it<waypoints_vec.size();it++)
    {
      if(EucDistance(plan[it].pose.position.x,plan[it].pose.position.y \
                        ,waypoints_vec[it].pose.position.x,waypoints_vec[it].pose.position.y)>max_waypoint_dist)
      {
        // @TODO Downsample
      }
      plan.push_back(waypoints_vec[it]);

    }
    ROS_INFO("[GWA] Added %d poses to the global plan!",(int)(plan.size()));

    // Visualise plan in rviz
    // create a the path message
    geometry_msgs::PoseArray gui_path;
    gui_path.poses.resize(plan.size());
    gui_path.header.frame_id = plan[0].header.frame_id;
    gui_path.header.stamp = plan[0].header.stamp;

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < plan.size(); i++) {
        gui_path.poses[i] = plan[i].pose;
    }
    global_plan_pub_.publish(gui_path);
    return true;

  }
  else
  {
    return false;
  }
  
}

// Euclidean distance 
double GlobalWaypointsAcceptor::EucDistance(double x1, double y1, double x2, double y2) {
    double x = x1 - x2; // calculating number to square in next step
    double y = y1 - y2;
    double dist;

    dist = std::pow(x, 2) + std::pow(y, 2); // calculating Euclidean distance
    dist = std::sqrt(dist);

    return dist;
  }


};
