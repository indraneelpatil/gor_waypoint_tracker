#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "local_waypoints_tracker.h"
#include <thread>
#include <ros/console.h>

bool planner_alive = true;
float setpoint = 0.0f;
ros::Publisher velo_pub,accel_pub;
double dist_to_travel = 10; //in m

void SetpointCallback(const std_msgs::Float32::ConstPtr& msg)
{
  ROS_INFO("Setting setpoint [%f]", msg->data);
  setpoint = msg->data;
}

void PlannerFunction(GorLocalWaypointsTracker::LocalWaypointsTracker &planner)
{
  // Local variables
  std_msgs::Float32 current_velocity = std_msgs::Float32();
  std_msgs::Float32 current_acceleration = std_msgs::Float32();

  ROS_INFO("Local planner is now running with frequency %f!",1/planner.planner_timestep); 
  while(planner_alive)
  {
    planner.Vref = setpoint;
    //planner.Vref = std::min(planner.max_v_lin_,planner.deceleration_checker_module(dist_to_travel));
    if(!planner.is_equal(planner.Vref ,planner.Vtarget))
    {
        ROS_INFO("Change in target velocity detected with dist_left %f!",dist_to_travel); 
        planner.s_curve_profiling();
    }

    planner.compute_next_velocity();
    
    current_acceleration.data = (planner.Vcurrent-current_velocity.data)/planner.planner_timestep;
    current_velocity.data = planner.Vcurrent;
    velo_pub.publish(current_velocity);
    accel_pub.publish(current_acceleration);
    dist_to_travel -= planner.planner_timestep*planner.Vcurrent;
    std::this_thread::sleep_for(std::chrono::milliseconds((int)(planner.planner_timestep*1000)));
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "s_curve");
  ros::NodeHandle n;

  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
   ros::console::notifyLoggerLevelsChanged();
}

  ros::Subscriber sub = n.subscribe("/s_curve/setpoint", 1000, SetpointCallback);

  velo_pub = n.advertise<std_msgs::Float32>("/s_curve/velocity", 1000);
  accel_pub = n.advertise<std_msgs::Float32>("/s_curve/acceleration", 1000);

  GorLocalWaypointsTracker::LocalWaypointsTracker planner;

  // Initialisations
  planner.Vref = 0.0;  // curvature bound
  planner.Vtarget = 0.0; 
  planner.Vcurrent = 0.0;

  planner.timestep = 0; 
  planner.planner_timestep = 0.025; //in seconds
  planner.max_a_lin_ = 0.59;
  planner.max_j_lin_ = 1.0;
  planner.max_v_lin_ = 1.2;

  planner.profile_state = GorLocalWaypointsTracker::LocalWaypointsTracker::s_curve_prof_state_e::CONST_V;

  
  std::thread planner_thread = std::thread(PlannerFunction,std::ref(planner));

  ros::spin();
  planner_alive = false;

  planner_thread.join();
  return 0;
}