#!/usr/bin/env python
import rospy
from velocity_to_path import FakeOdometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray,Pose,Twist,PoseStamped
from threading import Thread, Event, Condition
import rospkg

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import time
import math # for trigonometric functions
import json # for velocity dumps
import os   # to create folders for dumps


butler_version_mapper = {
	'1.5': 'butler_xl',
	'2.0': 'butler_m_2_0',
	'2.1': 'butler_m_2_1',
	'XL1.0': 'butler_xl',
	'0.0': 'unknown' 
	}

rospack = rospkg.RosPack()

class LogTrajectory(Thread):

    def __init__(self, butler_version):
        Thread.__init__(self)
        self.butler_ver_info = butler_version

        self.traj_dir = 'trajectories/'
        self.package_dir = rospack.get_path('local_waypoints_tracker')

        with open(self.package_dir+'/scripts/config/config.json', 'r') as config_file:
            config = json.load(config_file)
        rospy.logdebug("Config string : %s",config)
        bot_variant = butler_version_mapper[butler_version]
        rospy.loginfo("Bot variant used for trajectory logger: %s",bot_variant)
        self.ticks_per_meter = config[bot_variant]["ticks_per_meter"]
        self.base_width = config[bot_variant]["base_width"] # The wheel base width in meters

        self.control_point_list = list()
        self.commanded_velocities_list = list()
        self.measured_poses_x_list = list()
        self.measured_poses_y_list = list()

        self.num_bezier_points = 200
        self.node_active = True

        # Subscribers
        self.cp_sub = rospy.Subscriber("/move_base/LocalWaypointsTracker/debug/control_points"\
                                    , PoseArray, self.control_pts_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        self.feedback_pose_sub = rospy.Subscriber("/move_base/LocalWaypointsTracker/debug/feedback_pose"\
                                    , PoseStamped, self.fb_pose_callback)

        self.cvar  = Condition()

        try:
            rospy.logdebug("Starting thread")
            Thread.start(self)
        except RuntimeError as err:
            rospy.logdebug("Couldn't spin thread")

    def control_pts_callback(self,data):
        # Add new bezier points to the list
        rospy.logdebug("Control points :")
        for pose in data.poses:
            control_point = {}
            control_point['x'] = pose.position.x
            control_point['y'] = pose.position.y
            self.control_point_list.append(control_point)
            rospy.logdebug("(%f,%f)",pose.position.x,pose.position.y)

        # Log the trajectory
        self.cvar.acquire()
        try:
            self.cvar.notify()
        finally:
            # Releasing the lock 
            self.cvar.release()
                   

    def cmd_vel_callback(self,data):
        commanded_velo = {}
        commanded_velo['lin_vel'] = data.linear.x
        commanded_velo['ang_vel'] = data.angular.z
        commanded_velo['timestamp'] = rospy.Time.now().to_sec()

        # Face specific calculation (susp)
        commanded_velo['v_left'] =  data.linear.x - data.angular.z
        commanded_velo['v_right'] =  data.linear.x + data.angular.z

        self.commanded_velocities_list.append(commanded_velo)

    def fb_pose_callback(self,data):
        self.measured_poses_x_list.append(data.pose.position.x)
        self.measured_poses_y_list.append(data.pose.position.y)


    def run(self):
        thread_name = Thread.getName(self)
        rospy.logdebug("%s: Thread spinning",thread_name)
        while self.node_active == True:
            #rospy.loginfo("Config used for Butler Version: %s",self.butler_ver_info)
            rospy.loginfo("%s: Waiting for step to end!",thread_name)
            self.cvar.acquire()
            self.cvar.wait()
            if self.node_active == True:
                rospy.loginfo("%s: Step ended!",thread_name)
                rospy.loginfo("%s : Received %d bezier CPs, %d measured poses and %d commanded velocities",\
                                thread_name,len(self.control_point_list),len(self.measured_poses_x_list),len(self.commanded_velocities_list))
                self.generate_plots()

                # Clear previous lists and get ready for new step
                del self.control_point_list[:]
                del self.commanded_velocities_list[:]
                del self.measured_poses_x_list[:]
                del self.measured_poses_y_list[:]

        rospy.loginfo("%s: Received interrupt, stopping",thread_name)
        return

    def get_init_pose(self):
        if self.control_point_list != []:
            rospy.logdebug("Length of control points list : %d",len(self.control_point_list))
            P0 = self.control_point_list[0]
            P1 = self.control_point_list[1]
            P2 = self.control_point_list[2]
            P3 = self.control_point_list[3]

            dX_bez = P0['x']
            dY_bez = P0['y']
            dT_bez = math.atan2((P1['y'] - dY_bez),(P1['x']-dX_bez))

            #Cartesian to REP-103
            dX_bot = dX_bez
            dY_bot = dY_bez 
            dT_bot = dT_bez 

            rospy.logdebug("Setting init pose : %f, %f, %f",dX_bot,dY_bot, dT_bot)

            return (dX_bot, dY_bot, dT_bot) #return x0, y0, theta
        else:
            rospy.logwarn("Control points list is empty!")
            return (0.0, 0.0, 0.0)

    def get_path(self, velocity_list):
        poses_x = []
        poses_y = []
        first = True
        rospy.logdebug("%s: Got %d velocities",Thread.getName(self),len(velocity_list))
        x_init, y_init, theta_init = self.get_init_pose()
        odom = FakeOdometry(self.base_width, self.ticks_per_meter)
        for velocity in velocity_list:
            v_left    = float(velocity['v_left'])
            v_right   = float(velocity['v_right'])
            timestamp = float(velocity['timestamp'])
            # self.log.debug('velocity {}'.format(velocity))
            if first == True:
                pose = odom.init(timestamp, v_left, v_right, x_init, y_init, theta_init)
                # self.log.debug('Pose ({})'.format(pose))
                first = False
            else:
                pose = odom.update(v_left, v_right, timestamp)
                # self.log.debug('Pose ({})'.format(pose))
                poses_x.append(pose[0])
                poses_y.append(-1*pose[1])
        return (poses_x, poses_y)

    def generate_plots(self):
        plt.clf()

        #draw commanded trajectory
        poses_x, poses_y = self.get_path(self.commanded_velocities_list)
        h1, = plt.plot(poses_x, poses_y, label='commanded_traj')

        #draw measured_trajectory
        h2, = plt.plot(self.measured_poses_x_list, self.measured_poses_y_list, label='measured_traj')

        #draw bezier trajectory
        bez_x, bez_y = self.get_bezier_path()
        h3, = plt.plot(bez_x, bez_y, label='bezier_traj')

        #Plot!
        plt.grid(True)
        plt.legend(handles=[h1,h2,h3])
        traj_filename = self.package_dir + '/scripts/' + self.traj_dir  + str(rospy.Time.now().to_sec()) + '.png'
        rospy.loginfo(" %s: Saving plot : %s",Thread.getName(self),traj_filename)
        plt.savefig(traj_filename)

    def get_bezier_path(self):
        N = self.num_bezier_points
        bez_x = []
        bez_y = []
        
        if self.control_point_list != []:
            P0 = self.control_point_list[0]
            P1 = self.control_point_list[1]
            P2 = self.control_point_list[2]
            P3 = self.control_point_list[3]

            for count in range(N):
                t = float(count)/N
                X =  ((pow(1 - t, 3) * P0['x']) + (3 * t * pow(1 - t, 2) * P1['x']) + (3 * t * t * (1 - t) * P2['x']) + (t * t * t * P3['x'])) 
                Y =  ((pow(1 - t, 3) * P0['y']) + (3 * t * pow(1 - t, 2) * P1['y']) + (3 * t * t * (1 - t) * P2['y']) + (t * t * t * P3['y'])) 
            
                #Append as REP-103 coord
                bez_x.append(X)
                bez_y.append(Y)

        return (bez_x, bez_y)


   

if __name__ == '__main__':
    rospy.init_node('log_trajectory', anonymous=True,log_level=rospy.DEBUG)
    log_traj_obj = LogTrajectory('2.1')
    rospy.spin()
    log_traj_obj.node_active = False

    # Notify thread to close
    log_traj_obj.cvar.acquire()
    try:
        log_traj_obj.cvar.notify()
    finally:
        # Releasing the lock 
        log_traj_obj.cvar.release()
    log_traj_obj.join()
    
