// Created by Indraneel on 11/10/20

#ifndef LOCAL_WAYPOINTS_TRACKER_H
#define LOCAL_WAYPOINTS_TRACKER_H

// ROS_LIBS
#include <ros/ros.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <dynamic_reconfigure/server.h>


// LOCAL PLANNER INTERFACE
#include <base_local_planner/trajectory_planner_ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

// C++ LIBS
#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <limits.h>

#define nCr(n,r) LocalWaypointsTracker::factorial(n)/(LocalWaypointsTracker::factorial(r)*LocalWaypointsTracker::factorial(n-r))

// Converts degrees to radians.
#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0)
// Converts radians to degrees.
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI)

namespace GorLocalWaypointsTracker {

class LocalWaypointsTracker :  public nav_core::BaseLocalPlanner {
public:
    LocalWaypointsTracker();
    ~LocalWaypointsTracker();
    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
    bool isGoalReached();
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
    bool is_equal(double x, double y);

    typedef Eigen::Matrix<float, 3, 1> Vector3f;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MatrixXd;

    void s_curve_profiling();
    void compute_next_velocity();
    double deceleration_checker_module(double distance_left);
    bool PlanLocalPath(geometry_msgs::PoseStamped current_pose);
    void visualise_local_bezier_path(ros::Publisher& path_pub);
    double find_closest_list_point(geometry_msgs::PoseStamped current_pose);
    float stanley_controller(geometry_msgs::PoseStamped current_pose);
    float openloop_controller(geometry_msgs::PoseStamped current_pose);

    double Vcurve;  // curvature bound
    double Vref;
    double Vtarget; 
    double Vcurrent;
    double Vinit;
    double Vemergency;
    int timestep;
    int waypoint_it;
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::queue<std::pair<float,float>> bezier_coordinates_q; 

    // Parameters
    double max_a_lat_;
    double max_v_lin_;
    double max_a_lin_;
    double max_a_ang_;
    double max_j_lin_;
    double max_dist_from_path_;
    double min_heading_error_;
    double min_ct_error_;
    int num_bezier_samples_;
    double planner_timestep; //in sec
    float Kp_heading_error_;
    float Ki_heading_error_;
    double goal_tolerance_trans_;
    double stopping_speed_;
    double ct_gain_parameter_;
    double max_steering_angle_;
    bool debug_mode_;

     typedef enum s_curve_prof_state
    {
        S_CURVE_ACCEL,
        CONST_V,
        S_CURVE_DECEL,
        S_CURVE_TRANSITION
    }s_curve_prof_state_e;

    s_curve_prof_state_e profile_state;

private:
    costmap_2d::Costmap2DROS *costmap_ros_;
    tf2_ros::Buffer *tf_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    base_local_planner::TrajectoryPlannerROS collision_planner_;
    bool initialised_;
    bool is_goal_reached;
    ros::Publisher local_plan_pub_;
    ros::Publisher control_pts_pub_;
    ros::Publisher feedback_pose_pub_;
    float heading_goal; // heading at next waypoint
    double controller_frequency;
    geometry_msgs::PoseArray control_points_dbg; 
    int face_reverse_multiplier;

    float sum_heading_error;
    double dist_left_to_travel;
    int state_timesteps;
    int const_jerk_timesteps;
    double state_t;
    double const_j_t;

    double HeadingDiff(Vector3f robot_pose,Vector3f target_pose);
    double EucDistance(double x1, double y1, double x2, double y2);
    void calculate_bezier_derivative_ctrl_pts(std::vector<MatrixXd> &points_vector);
    float find_bezier_max_curvature(std::vector<MatrixXd> points_vector);
    int factorial (int n);
    std::pair<float, float> bezier_bernstein_poly(float t,MatrixXd control_points);

    
};

};

#endif