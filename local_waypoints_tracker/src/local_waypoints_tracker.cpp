// Created by Indraneel on 11/10/20

#include "local_waypoints_tracker.h"
#include <pluginlib/class_list_macros.hpp>

// All units in m, sec and rad unless otherwise specified

PLUGINLIB_EXPORT_CLASS(GorLocalWaypointsTracker::LocalWaypointsTracker, nav_core::BaseLocalPlanner)

namespace GorLocalWaypointsTracker {

LocalWaypointsTracker::LocalWaypointsTracker(): costmap_ros_(NULL),initialised_(false),tfListener(tfBuffer) {}

LocalWaypointsTracker::~LocalWaypointsTracker() {
}

void LocalWaypointsTracker::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    ros::NodeHandle node_private("~/" + name);
    ros::NodeHandle node; // public
    ros::NodeHandle nh_move_base("~");

    collision_planner_.initialize(name + "/collision_planner", tf_, costmap_ros_);

    local_plan_pub_ = node_private.advertise<nav_msgs::Path>("local_plan", 1);
    
    //@TODO Set up dynamic reconfigure

    // Load parameters from parameter server
    node_private.param("max_a_lat", max_a_lat_, 50.0);
    node_private.param("max_a_lin", max_a_lin_, 0.59);
    node_private.param("max_v_lin", max_v_lin_, 0.8);
    node_private.param("max_a_ang", max_a_ang_, 0.052);
    node_private.param("max_j_lin", max_j_lin_, 1.0);
    node_private.param("max_dist_from_path", max_dist_from_path_, 0.5);
    node_private.param("min_heading_error", min_heading_error_, 0.1); // In degrees
    node_private.param("min_ct_error", min_ct_error_, 0.005); // in m
    node_private.param("num_bezier_samples", num_bezier_samples_, 50);
    node_private.param("Kp_heading_error", Kp_heading_error_, 0.05f);
    node_private.param("Ki_heading_error", Ki_heading_error_, 0.001f);
    node_private.param("goal_tolerance_trans", goal_tolerance_trans_, 0.05);
    node_private.param("stopping_speed", stopping_speed_, 0.07);
    node_private.param("ct_gain_parameter", ct_gain_parameter_, 0.0);
    node_private.param("max_steering_angle", max_steering_angle_, 1000.0);
    node_private.param("debug_mode", debug_mode_, false);

    nh_move_base.param("controller_frequency", controller_frequency, 40.0);

    // Initial declarations
    planner_timestep = 1.0/controller_frequency;
    waypoint_it = 0;
    Vref = 0.0;
    Vtarget = 0.0;
    Vcurrent = 0.0;
    timestep = 0;
    Vcurve = std::numeric_limits<double>::max();
    Vemergency = std::numeric_limits<double>::max();
    profile_state = CONST_V;
    is_goal_reached = false;

    if(debug_mode_)
    {
        control_pts_pub_ = node_private.advertise<geometry_msgs::PoseArray>("debug/control_points", 1);
        feedback_pose_pub_ = node_private.advertise<geometry_msgs::PoseStamped>("debug/feedback_pose", 1);
        control_points_dbg = geometry_msgs::PoseArray();
        control_points_dbg.poses.resize(4);
    }

    ROS_INFO("[LWT] Local Waypoints tracker is now initialised");
    initialised_ = true;
}

bool LocalWaypointsTracker::isGoalReached(){
    // @TODO Consider adding another condition of velocity along with position
    return is_goal_reached;
}

bool LocalWaypointsTracker::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
    // @TODO Transform plan to the right frame if required

    // @TODO Switching to new plan from existing plan by check on first waypoint in new plan

    //@TODO If new plan reset all variables
    is_goal_reached = false;
    global_plan_.clear();
    waypoint_it = 0;
    timestep = 0;
    profile_state = CONST_V;
    Vcurrent = 0.0;

    if(!global_plan.empty())
    {
        // Append waypoints from global plan to class member
        global_plan_.insert(global_plan_.end(),global_plan.begin()+1,global_plan.end());

        // Print number of waypoints 
        ROS_INFO("[LWT] Number of waypoints added %lu, Total poses in plan : %lu",global_plan.size(),global_plan_.size());
        
    }
    else
    {
        ROS_ERROR("Waypoints tracker received empty path!");
        return false;
    }

    return true;
}


bool LocalWaypointsTracker::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){

    bool result = false;
    //get the current pose of the robot 
    geometry_msgs::PoseStamped robot_pose,robot_pose_temp;
    geometry_msgs::TransformStamped transformStamped;
    if(!costmap_ros_->getRobotPose(robot_pose)){
      ROS_ERROR("[LWT] Can't get robot pose");
      geometry_msgs::Twist empty_twist;
      cmd_vel = empty_twist;
      return result;
    }

    try{
          transformStamped = tfBuffer.lookupTransform("map", "base_footprint", \
                                  ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ROS_WARN("map->base_footprint look up failed!");
        //ros::Duration(1.0).sleep();
    }

    robot_pose_temp.header = transformStamped.header;
    robot_pose_temp.pose.position.x = transformStamped.transform.translation.x;
    robot_pose_temp.pose.position.y = transformStamped.transform.translation.y;
    robot_pose_temp.pose.position.z = transformStamped.transform.translation.z;

    robot_pose_temp.pose.orientation = transformStamped.transform.rotation;

    if(debug_mode_)
    {
        feedback_pose_pub_.publish(robot_pose_temp);
    }
    // Check if bezier local path is already initialised
    if(bezier_coordinates_q.empty())
    {
        result = PlanLocalPath(robot_pose);
        sum_heading_error = 0.0f;
        if(!result)
        {
            ROS_ERROR("[LWT] Feasible Local path not found");
            Vemergency = 0.0;
            return result;
        }

    }

    // Closed loop bezier tracking of path begins here
    double dist_from_path = find_closest_list_point(robot_pose);
    if(dist_from_path>max_dist_from_path_)
    {
        result = false;
        Vemergency = 0.0;
    }
    visualise_local_bezier_path(local_plan_pub_);

     // Check if entered deceleration phase and set Vref to zero
    dist_left_to_travel = 0.0;
    double dist_to_next_waypoint = EucDistance(global_plan_[waypoint_it].pose.position.x,global_plan_[waypoint_it].pose.position.y, \
                                                    robot_pose.pose.position.x,robot_pose.pose.position.y);
    dist_left_to_travel += dist_to_next_waypoint;
    for(int j=waypoint_it;j<global_plan_.size()-1;j++)
    {
        dist_left_to_travel += EucDistance(global_plan_[waypoint_it].pose.position.x,global_plan_[waypoint_it].pose.position.y, \
                                                    global_plan_[waypoint_it+1].pose.position.x,global_plan_[waypoint_it+1].pose.position.y);
    }
    
    Vref = std::min({Vcurve,deceleration_checker_module(dist_left_to_travel),max_v_lin_});

    // If reference velocity has changed enter linear velocity profiling
    if(!is_equal(Vref,Vtarget))
    {
        s_curve_profiling();
    }
    // Pop next linear velocity element from S curve
    compute_next_velocity();

    cmd_vel.linear.x = Vcurrent*face_reverse_multiplier;
    float angular_velo = stanley_controller(robot_pose);//*face_reverse_multiplier;
    cmd_vel.angular.z = fabs(angular_velo) > max_a_ang_ ? copysignf(max_a_ang_,angular_velo) : angular_velo;

    if(fabs(cmd_vel.linear.x) < fabs(cmd_vel.angular.z))
        cmd_vel.angular.z = 0.0;
        
    ROS_INFO("[LWT] ######### CMD_VEL LIN : %f, ANG : %f, DistWP : %f, DistGoal : %f Bez_left : %d",cmd_vel.linear.x,cmd_vel.angular.z,dist_to_next_waypoint,dist_left_to_travel,bezier_coordinates_q.size());

    // Check if reached next waypoint -> empty bezier list increment waypoint_it
    if(dist_to_next_waypoint<goal_tolerance_trans_)
    {
        if(debug_mode_)
        {
            // Send bezier curve for plotting if in debug mode
            control_pts_pub_.publish(control_points_dbg);
        }

        while(!bezier_coordinates_q.empty())
        {
            bezier_coordinates_q.pop();
        }
        ROS_INFO("[LWT] Reached Waypoint %d!!",waypoint_it);
        waypoint_it++;

        if(waypoint_it>=global_plan_.size())
        {
            ROS_INFO("[LWT] Reached goal!!");
            is_goal_reached = true;
            geometry_msgs::Twist empty_twist;
            cmd_vel = empty_twist;
        }
    }

    result = true;
    return result;
}

bool LocalWaypointsTracker::PlanLocalPath(geometry_msgs::PoseStamped current_pose)
{
    // Local variables
    float shortest_distance,heading_current,heading_next,temp,max_curvature;
    heading_current = std::atan2(global_plan_[waypoint_it].pose.position.y-current_pose.pose.position.y,\
                         global_plan_[waypoint_it].pose.position.x-current_pose.pose.position.x);


    // Find initial and final heading
    Vector3f pnt_c(current_pose.pose.position.x, current_pose.pose.position.y,tf2::getYaw(current_pose.pose.orientation));
    
    if(waypoint_it == global_plan_.size()-1)
    {
        // Last waypoint
        heading_goal = heading_current;
    }
    else
    {
        heading_next = std::atan2(global_plan_[waypoint_it+1].pose.position.y-global_plan_[waypoint_it].pose.position.y,\
                         global_plan_[waypoint_it+1].pose.position.x-global_plan_[waypoint_it].pose.position.x);
        //heading_goal = heading_current + (heading_next-heading_current)/2;
        heading_goal = heading_next;
    }
    Vector3f pnt_t(global_plan_[waypoint_it].pose.position.x,global_plan_[waypoint_it].pose.position.y,heading_goal);

    // Find which face to use for move
    double front_yaw_diff = HeadingDiff(pnt_c,pnt_t);
    Vector3f pnt_c_back(current_pose.pose.position.x, current_pose.pose.position.y, M_PI + tf2::getYaw(current_pose.pose.orientation));
    double back_yaw_diff = HeadingDiff(pnt_c_back,pnt_t);

    if(fabs(front_yaw_diff)>fabs(back_yaw_diff))
    {
        ROS_INFO("[LWT] Using Back face for Move %f > %f!",radiansToDegrees(front_yaw_diff),radiansToDegrees(back_yaw_diff));
        pnt_c = pnt_c_back;
        face_reverse_multiplier = -1;
    }
    else
    {
        ROS_INFO("[LWT] Using Front face for Move %f < %f!",radiansToDegrees(front_yaw_diff),radiansToDegrees(back_yaw_diff));
        face_reverse_multiplier = 1;
    }

    shortest_distance = EucDistance(pnt_c[0],pnt_c[1],pnt_t[0],pnt_t[1]);

    ROS_INFO("[LWT] Current robot pose %f %f ==> %f", pnt_c[0], pnt_c[1], radiansToDegrees(pnt_c[2]));
    ROS_INFO("[LWT]  Target robot pose %f %f ==> %f", pnt_t[0], pnt_t[1], radiansToDegrees(pnt_t[2]));

    // Four point cubic bezier curve
    MatrixXd control_points(4,2);
    control_points << pnt_c[0],pnt_c[1],
        pnt_c[0]+ 0.5*shortest_distance*cos(pnt_c[2]),pnt_c[1]+ 0.5*shortest_distance*sin(pnt_c[2]),
        pnt_t[0]- 0.5*shortest_distance*cos(pnt_t[2]),pnt_t[1]- 0.5*shortest_distance*sin(pnt_t[2]),
        pnt_t[0],pnt_t[1];    
    
    // Derivative of a bezier is also a bezier @https://pomax.github.io/bezierinfo/#derivatives
    std::vector<MatrixXd> control_points_vec;
    control_points_vec.push_back(control_points);
    calculate_bezier_derivative_ctrl_pts(control_points_vec);

    // Find max curvature of the bezier curve using iterative sampling
    max_curvature = find_bezier_max_curvature(control_points_vec);
    
    // Set Vcurve for velocity profiler bounded by max curvature 
    Vcurve = pow(max_a_lat_/fabs(max_curvature),0.5);
    ROS_INFO("[LWT] Max curvature of current path : %f and Vcurve is %f",max_curvature,Vcurve);

    // @TODO Add sanity checks on the path after finding one
    ROS_INFO("[LWT] Bezier queue has %d number of points",(int)(bezier_coordinates_q.size()));

    if(debug_mode_)
    {        
        control_points_dbg.header.stamp = ros::Time::now();
        for(int i=0;i<4;i++)
        {
            control_points_dbg.poses[i].position.x = control_points(i,0);
            control_points_dbg.poses[i].position.y = control_points(i,1);
        }
    }
    return true;
}


// Euclidean distance 
double LocalWaypointsTracker::EucDistance(double x1, double y1, double x2, double y2) {
    double x = x1 - x2; // calculating number to square in next step
    double y = y1 - y2;
    double dist;

    dist = std::pow(x, 2) + std::pow(y, 2); // calculating Euclidean distance
    dist = std::sqrt(dist);

    return dist;
  }

void LocalWaypointsTracker::calculate_bezier_derivative_ctrl_pts(std::vector<MatrixXd> &points_vector)
{
    int poly_order = points_vector[0].rows()-1;
    // A bezier of order n has n-1 meaningful derivatives 
    for(int i=poly_order-1;i>0;i--)
    {
        // poly order n has n+1 control points
        MatrixXd ctrl_pts(i+1,2);
        for(int j=0;j<=i;j++)
        {
            ctrl_pts(j,0) = (i+1)*(points_vector[points_vector.size()-1](j+1,0)-points_vector[points_vector.size()-1](j,0));
            ctrl_pts(j,1) = (i+1)*(points_vector[points_vector.size()-1](j+1,1)-points_vector[points_vector.size()-1](j,1));
        }
        points_vector.push_back(ctrl_pts);
    }

}

float LocalWaypointsTracker::find_bezier_max_curvature(std::vector<MatrixXd> points_vector)
{
    float max_curvature = 0.0f; // Indicates straight line path

    if(points_vector.size()>2)
    {
        for(int i=0;i<=num_bezier_samples_;i++)
        {
            float t = (float)(i)/(float)(num_bezier_samples_);
            std::pair<float,float> b = bezier_bernstein_poly(t,points_vector[0]);
            std::pair<float,float> db = bezier_bernstein_poly(t,points_vector[1]);
            std::pair<float,float> ddb = bezier_bernstein_poly(t,points_vector[2]);
            float curvature = (db.first*ddb.second - db.second*ddb.first)/pow(pow(db.first,2)+pow(db.second,2),1.5f);
            if(fabs(curvature)>fabs(max_curvature))
                max_curvature = curvature;

            // Add bezier coordinate to the list
            bezier_coordinates_q.push(b);
        }
        return max_curvature;
    }
    {
        ROS_ERROR("[LWT] Curvature calculation needs atleast two derivatives,aborting!");
        return max_curvature;
    }
}

int LocalWaypointsTracker::factorial(int n)
{
    return (n == 1 || n == 0) ? 1 : factorial(n-1)*n;
}

std::pair<float, float> LocalWaypointsTracker::bezier_bernstein_poly(float t,MatrixXd control_points)
{
    int poly_order = control_points.rows() - 1;
    float x_value_sum = 0.0f;
    float y_value_sum = 0.0f;
    for(int i=0;i<=poly_order;i++)
    {
        float bernstein_value = nCr(poly_order,i)*pow(t,i)*pow(1-t,poly_order-i);
        x_value_sum += bernstein_value*control_points(i,0);
        y_value_sum += bernstein_value*control_points(i,1);
    }

    return std::make_pair(x_value_sum,y_value_sum);
}

// Pops elements until the closest list point
double LocalWaypointsTracker::find_closest_list_point(geometry_msgs::PoseStamped current_pose)
{
    // Local variables
    double dist_from_path = std::numeric_limits<double>::max();
    int min_dist_it,it = 0;
    std::queue<std::pair<float,float>> local_q = bezier_coordinates_q;

    if(!bezier_coordinates_q.empty())
    {
        while(!local_q.empty())
        {
            std::pair<float,float> coordinates = local_q.front();
            if(EucDistance(coordinates.first,coordinates.second,current_pose.pose.position.x,current_pose.pose.position.y)<dist_from_path)
            {
                min_dist_it = it;
                dist_from_path = EucDistance(coordinates.first,coordinates.second,current_pose.pose.position.x,current_pose.pose.position.y);
            }
            local_q.pop();
            it++;
        }
        // Pop elements from q till min_dist_it (Last element is never popped)
        //ROS_DEBUG("Dist from path :%f, Popping %d elements, Elements left : %d",dist_from_path,min_dist_it,(int)(bezier_coordinates_q.size()-min_dist_it));    
        while(min_dist_it!=0)
        {
            bezier_coordinates_q.pop();
            min_dist_it--;
        }
    }

    return dist_from_path;

}

void LocalWaypointsTracker::visualise_local_bezier_path(ros::Publisher& path_pub)
{
    // Local variables
    std::queue<std::pair<float,float>> local_q = bezier_coordinates_q;

    // create a path message
    nav_msgs::Path gui_path;
    gui_path.poses.resize(local_q.size());
    gui_path.header.frame_id = global_plan_[0].header.frame_id;
    gui_path.header.stamp = ros::Time::now();

    int i = 0;
    while(!local_q.empty()) {
        std::pair<float,float> coordinates = local_q.front();
        gui_path.poses[i].pose.position.x = coordinates.first;
        gui_path.poses[i].pose.position.y = coordinates.second;
        gui_path.poses[i].pose.position.z = 0.0;
        local_q.pop();
        i++;
    }
    path_pub.publish(gui_path);
}

double LocalWaypointsTracker::HeadingDiff(Vector3f robot_pose,Vector3f target_pose)
{
    // Move vector
    double move_x = target_pose[0] - robot_pose[0];
    double move_y = target_pose[1] - robot_pose[1];

    // Robot vector
    double robot_x = cos(robot_pose[2]);
    double robot_y = sin(robot_pose[2]);

    // Return a signed angle between move and robot vectors
    double dot_product = move_x*robot_x + move_y*robot_y;
    // Magnitude of the z component in the perpendicular direction
    double cross_product = move_x*robot_y - move_y*robot_x;


    return -1.0*atan2(cross_product,dot_product);

}

float LocalWaypointsTracker::openloop_controller(geometry_msgs::PoseStamped current_pose)
{
    // Local variables
    float result,target_heading,heading_error,control_output,cross_track_error,current_heading;

    if(face_reverse_multiplier>0.0)
        current_heading = tf2::getYaw(current_pose.pose.orientation);
    else
        current_heading = M_PI + tf2::getYaw(current_pose.pose.orientation);
    std::pair<float,float> target_point,next_point,next2next_point;

    if(!bezier_coordinates_q.empty())
    {
        std::queue<std::pair<float,float>> local_q = bezier_coordinates_q;
        target_point = local_q.front();
        if(bezier_coordinates_q.size()==2)
        {
            // Final element left
            target_heading = heading_goal;
        }
        else
        {
            local_q.pop();
            next_point = local_q.front();
            current_heading = atan2((next_point.second-target_point.second),(next_point.first-target_point.first));
            local_q.pop();
            next2next_point = local_q.front();
            target_heading = atan2((next2next_point.second-next_point.second),(next2next_point.first-next_point.first));
        }
        
        result = (target_heading-current_heading)/planner_timestep;

        ROS_INFO("[LWT] OLC Target Heading %f deg, Angular Velo %f rad/sec ",radiansToDegrees(target_heading),result);

    }
    else
    {
        ROS_ERROR("[LWT] OLC received empty bezier list!");
        result = 0.0;
    }


    return result;
}

float LocalWaypointsTracker::stanley_controller(geometry_msgs::PoseStamped current_pose)
{
    // Local variables
    float result,target_heading,heading_error,control_output,cross_track_error,current_heading;

    if(face_reverse_multiplier>0.0)
        current_heading = tf2::getYaw(current_pose.pose.orientation);
    else
        current_heading = M_PI + tf2::getYaw(current_pose.pose.orientation);
    std::pair<float,float> target_point,next_point;

    if(!bezier_coordinates_q.empty())
    {
        std::queue<std::pair<float,float>> local_q = bezier_coordinates_q;
        target_point = local_q.front();
        if(bezier_coordinates_q.size()==1)
        {
            // Final element left
            target_heading = heading_goal;
        }
        else
        {
            local_q.pop();
            next_point = local_q.front();
            target_heading = atan2((next_point.second-target_point.second),(next_point.first-target_point.first));
        }
        // Heading error
        heading_error = radiansToDegrees(target_heading) - radiansToDegrees(current_heading);
        heading_error += (heading_error>180.0f) ? -360.0f : (heading_error<-180.0f) ? 360.0f : 0.0f;
        heading_error = fabs(heading_error)<min_heading_error_ ? 0.0 : heading_error;
        heading_error = degreesToRadians(heading_error);
        
        // Cross Track Error
        float xtargetTranslated = target_point.first-current_pose.pose.position.x;
        float ytargetTranslated = target_point.second-current_pose.pose.position.y;

        double xtargetRelRobot = xtargetTranslated * cos(current_heading) - ytargetTranslated * sin(current_heading);
        double ytargetRelRobot = xtargetTranslated * sin(current_heading) + ytargetTranslated * cos(current_heading);

        if(ytargetRelRobot>0.0)
        {
            cross_track_error = EucDistance(xtargetRelRobot,ytargetRelRobot,0.0,0.0);
        }
        else
        {
            cross_track_error = -1.0*EucDistance(xtargetRelRobot,ytargetRelRobot,0.0,0.0);
        }
        cross_track_error = fabs(cross_track_error)<min_ct_error_ ? 0.0 : cross_track_error;

        // Controller
        sum_heading_error += heading_error;

        control_output = Kp_heading_error_*heading_error + Ki_heading_error_*sum_heading_error + \
                        atan2(ct_gain_parameter_*cross_track_error,(stopping_speed_ + Vcurrent ));
        //control_output = fabs(control_output)>max_steering_angle_ ? max_steering_angle_ : control_output;
        result = control_output/planner_timestep;

        ROS_INFO("[LWT] Stanley HE %f deg, PC: %f , IC: %f  CTE: %f m",radiansToDegrees(heading_error),Kp_heading_error_*heading_error,Ki_heading_error_*sum_heading_error,cross_track_error);

    }
    else
    {
        ROS_ERROR("[LWT] Stanley controller received empty bezier list!");
        result = 0.0;
    }


    return result;
}

/** @brief : based on page 79 of 'Trajectory planning for automatic machines and robots(2008)*/
void LocalWaypointsTracker::s_curve_profiling()
{
    // Local variables
    double Vdiff;

    // Wait for const V phase to accept new waypoint
    if(profile_state == CONST_V)
    {
        // Set profile state calculate parameters
        Vtarget = Vref;
        if(Vcurrent < Vtarget)
        {
            profile_state = S_CURVE_ACCEL;
            Vdiff = Vtarget - Vcurrent;
            ROS_INFO("[LWT] S-Curve profile state changed to acceleration with Vdiff %f!",Vdiff);
        }
        else
        {
            profile_state = S_CURVE_DECEL;
            Vdiff = Vcurrent - Vtarget;
            ROS_INFO("[LWT] S-Curve profile state changed to deceleration with Vdiff %f!",Vdiff);
        }

        //check if amax is reached
        if((Vdiff)*max_j_lin_ < pow(max_a_lin_,2))
        {
            // amax not reached
            const_j_t = pow((Vdiff)/max_j_lin_,0.5);
            const_jerk_timesteps = (int)(const_j_t/planner_timestep);
            state_t = 2*const_j_t;
            state_timesteps = 2*const_jerk_timesteps;
        }
        else
        {
            // amax reached
            const_j_t = max_a_lin_/ max_j_lin_;
            const_jerk_timesteps = (int)((const_j_t/planner_timestep));
            state_t = const_j_t + (Vdiff/max_a_lin_);
            state_timesteps = (int)(state_t/planner_timestep);
        }

        ROS_INFO("[LWT] ST : %f in %d steps, CJT: %f in %d steps",state_t,state_timesteps,const_j_t,const_jerk_timesteps);

        timestep = 0;
        Vinit = Vcurrent;

        // Set deceleration distance
    }
    else
    {
        // Delete Zero jerk phase from ongoing profile if any
        // Or abort immediately if already in zero jerk phase
        if(timestep<const_jerk_timesteps)
        {
            state_timesteps = 2*const_jerk_timesteps;
            ROS_INFO("[LWT] Deleted zero jerk phase for new setpoint!");
        }
        else if(timestep>=const_jerk_timesteps && (timestep<(state_timesteps-const_jerk_timesteps)))
        {
            state_timesteps = timestep+const_jerk_timesteps;
            ROS_INFO("[LWT] Truncated zero jerk phase for new setpoint!");
        }
    }
}


void LocalWaypointsTracker::compute_next_velocity()
{
    double t = (double)(timestep)*planner_timestep;
    double a_lim_a = max_j_lin_*const_j_t;
    double a_lim_d = -1*max_j_lin_*const_j_t;
    static double V_zero_jend;
    static double t_zero_jend;


    switch (profile_state)
    {
        case S_CURVE_ACCEL:
        {
            if(timestep<const_jerk_timesteps)
            {
                Vcurrent = Vinit + max_j_lin_*pow(t,2)/2;
                V_zero_jend = Vcurrent;
                t_zero_jend = t;
            }
            else if(timestep>=const_jerk_timesteps && (timestep<(state_timesteps-const_jerk_timesteps)))
            {
                Vcurrent = Vinit + a_lim_a*(t-const_j_t/2);
                V_zero_jend = Vcurrent;
                t_zero_jend = t;
            }
            else if((timestep>=(state_timesteps-const_jerk_timesteps)) && timestep<state_timesteps)
            {
                t = t-t_zero_jend;
                Vcurrent = V_zero_jend +a_lim_a*t - max_j_lin_*(pow(t,2)/2);
            }
            else
            {
                // finished state
                profile_state = CONST_V;
                ROS_INFO("[LWT] S-Curve profile state changed to ConstV!");
                compute_next_velocity();
            }
            Vcurrent = std::min(Vcurrent,Vtarget);
            break;
        }

        case S_CURVE_DECEL:
        {
            if(timestep<const_jerk_timesteps)
            {
                Vcurrent = Vinit - max_j_lin_*(pow(t,2)/2);
                V_zero_jend = Vcurrent;
                t_zero_jend = t;
            }
            else if(timestep>=const_jerk_timesteps && timestep<state_timesteps-const_jerk_timesteps)
            {
                Vcurrent = Vinit + a_lim_d*(t-const_j_t/2);
                V_zero_jend = Vcurrent;
                t_zero_jend = t;
            }
            else if(timestep>=state_timesteps-const_jerk_timesteps && timestep<state_timesteps)
            {
                //Vcurrent = Vtarget + max_j_lin_*(pow(state_t - t,2)/2);
                t = t-t_zero_jend;
                Vcurrent = V_zero_jend +a_lim_d*t + max_j_lin_*(pow(t,2)/2);
            }
            else
            {
                // finished state
                profile_state = CONST_V;
                ROS_INFO("[LWT] S-Curve profile state changed to ConstV!");
                compute_next_velocity();
            }
            Vcurrent = std::max(Vcurrent,Vtarget);
            break;
        }
        case CONST_V:
        {
            // No change
            Vcurrent = Vcurrent;
            break;
        }

    }

    timestep++;
}

// Target velocity is zero
double LocalWaypointsTracker::deceleration_checker_module(double dist_left)
{
    // Local variables
    double time_to_amax,time_to_set_speeds;
    double stopping_dist = 0.45; //in m
    dist_left -= stopping_dist;

    // Only applicable for finite positive non zero velocities
    if(Vcurrent > 0.0)
    {
       // Algo to check if it is feasible to decelerate in dist_left from Vcurrent
       time_to_amax = max_a_lin_/max_j_lin_;
       time_to_set_speeds = pow(Vcurrent/max_j_lin_,0.5);

       if(time_to_amax<time_to_set_speeds)
       {
            if(dist_left>0.5*Vcurrent*(time_to_amax+Vcurrent/max_a_lin_))
                return std::numeric_limits<double>::max();
            else
                return stopping_speed_;
       }
       else
       {
           if(dist_left>time_to_set_speeds*Vcurrent)
                return std::numeric_limits<double>::max();
            else
                return stopping_speed_;
       }

    }
    else
        return std::numeric_limits<double>::max();
}

bool LocalWaypointsTracker::is_equal(double x, double y)
{
    return std::fabs(x - y) <= std::numeric_limits<double>::epsilon();
}

};



