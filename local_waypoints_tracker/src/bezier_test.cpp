#include "local_waypoints_tracker.h"
#include <thread>

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

bool planner_alive = true;
ros::Publisher local_plan_pub_;

void PlannerFunction(GorLocalWaypointsTracker::LocalWaypointsTracker &planner)
{   
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    for(float i=0.0;i<=1.3;i+=(1.0/(float)(planner.num_bezier_samples_)))
    {
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = -1*i;
        planner.find_closest_list_point(pose);
        planner.visualise_local_bezier_path(local_plan_pub_);
        ROS_INFO("X:%f Angular velocity : %f",i,planner.stanley_controller(pose));
        if(!planner_alive)
            break;
        std::this_thread::sleep_for(std::chrono::seconds((int)(1)));
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bezier_planner");
    ros::NodeHandle n;

    local_plan_pub_ = n.advertise<nav_msgs::Path>("local_plan", 1);

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
    }

    GorLocalWaypointsTracker::LocalWaypointsTracker planner;
    planner.max_a_lat_ = 1.0;
    planner.waypoint_it = 0;
    planner.num_bezier_samples_ = 50;
    planner.planner_timestep = 0.025; //in seconds
    planner.Kp_heading_error_ = 0.1;
    planner.Ki_heading_error_ = 0.001;
    planner.debug_mode_ = false;
    planner.min_heading_error_ = 0.0;
    planner.min_ct_error_ = 0.0;
    planner.ct_gain_parameter_ = 0.03;

    // Final position
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = -1.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();


    // Add waypoint
    planner.global_plan_.push_back(pose);

    // Initial position
    pose.pose.position.x = 0.0;
    pose.pose.position.y = -0.3;
    pose.pose.position.z = 0.0;

    q.setRPY(0, 0, degreesToRadians(10.0));
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    planner.PlanLocalPath(pose);
    ROS_INFO("Size of bezier queue : %d",planner.bezier_coordinates_q.size());
    std::this_thread::sleep_for(std::chrono::seconds((int)(2)));
    planner.visualise_local_bezier_path(local_plan_pub_);

    std::thread planner_thread = std::thread(PlannerFunction,std::ref(planner));

    ros::spin();
    planner_alive = false;

    planner_thread.join();
    return 0;
}