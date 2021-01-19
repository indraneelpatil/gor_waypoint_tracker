#!/usr/bin/env python

"""
    ##  MODIFIED TO REMOVE THE ROS APIs ##

   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import numpy as np
# import roslib
# roslib.load_manifest('differential_drive')
from math import sin, cos, pi

# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf.broadcaster import TransformBroadcaster
# from std_msgs.msg import Int32

#from rolling_median_filter import RollingMedianFilter

#############################################################################
class FakeOdometry:
#############################################################################

    #############################################################################
    def __init__(self, base_width = 0.245, ticks_meter = 31329.7):
    #############################################################################
        
        self.ticks_meter = ticks_meter  # The number of wheel encoder ticks per meter of travel
        #print ("ticks_meter: %f" %(self.ticks_meter))
        self.base_width = base_width # The wheel base width in meters
        #print ("base_width: %f" %(self.base_width))
        
        # self.base_frame_id = rospy.get_param('~base_frame_id','base_link') # the name of the base frame of the robot
        # self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom') # the name of the odometry reference frame
        
        self.encoder_min = -32768
        self.encoder_max = 32768
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min 
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min 
 
        # self.t_delta = rospy.Duration(1.0/self.rate)
        # self.t_next = rospy.Time.now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.vx_filtered = 0.0
        self.lpf_coeff = 0.15
        self.then = 0

        #self.vxMedFilt = RollingMedianFilter(11)

    def init(self, init_timestamp, vleft, vright, x0, y0, theta0):
        self.then = init_timestamp
        self.dx = (vleft + vright) * 0.5
        self.dr = (vright - vleft) / self.base_width
        self.x = x0
        self.y = y0
        self.th = theta0

        
        # subscriptions
        # rospy.Subscriber("lwheel", Int32, self.lwheelCallback)
        # rospy.Subscriber("rwheel", Int32, self.rwheelCallback)
        # self.odomPub = rospy.Publisher("wheel_odometry/odom2", Odometry, queue_size=10)
        # self.odomBroadcaster = TransformBroadcaster()
        
    # #############################################################################
    # def spin(self):
    # #############################################################################
    #     r = rospy.Rate(self.rate)
    #     while not rospy.is_shutdown():
    #         self.update()
    #         r.sleep()
       
     
    #############################################################################
    def update(self, vleft, vright, now):
    #############################################################################
        # now = rospy.Time.now()
        # if now > self.t_next:
        elapsed = now - self.then
        self.then = now
        # elapsed = elapsed.to_sec()

        if elapsed < 0:
            return None
        
        # distance traveled is the average of the two wheels 
        d = (( vleft + vright ) / 2) * elapsed
        # this approximation works (in radians) for small angles
        th = (( vright - vleft ) / self.base_width) * elapsed
        # calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed
        #if(abs(self.dx) > 0):
        #    return None
       
         
        if (d != 0):
            # calculate distance traveled in x and y
            x = cos( th ) * d
            y = -sin( th ) * d
            # calculate the final position of the robot
            self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
            self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
        if( th != 0):
            self.th = self.th + th

        # list of [x, y, yaw, vx]
        return (self.x, -self.y, self.th, self.dx)
                
            # publish the odom information
            # quaternion = Quaternion()
            # quaternion.x = 0.0
            # quaternion.y = 0.0
            # quaternion.z = -sin( self.th / 2 )
            # quaternion.w = cos( self.th / 2 )
            # self.odomBroadcaster.sendTransform(
            #     (self.x, -self.y, 0),
            #     (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            #     rospy.Time.now(),
            #     self.base_frame_id,
            #     self.odom_frame_id
            #     )
            
            # odom = Odometry()
            # odom.header.stamp = now
            # odom.header.frame_id = self.odom_frame_id
            # odom.pose.pose.position.x = self.x
            # odom.pose.pose.position.y = -self.y
            # odom.pose.pose.position.z = 0
            # odom.pose.pose.orientation = quaternion
            # #odom.pose.covariance = self.poseCovariance.ravel().tolist()
            # odom.pose.covariance = tuple(   [ 0.01, 0, 0, 0, 0, 0, \
            #                                   0, 0.1, 0, 0, 0, 0, \
            #                                   0, 0, 1e4 ,0, 0, 0, \
            #                                   0, 0, 0, 1e4, 0, 0, \
            #                                   0, 0, 0, 0, 1e4, 0, \
            #                                   0, 0, 0, 0, 0, 0.01 ])
            # odom.child_frame_id = self.base_frame_id
            # # simple low pass filter to supress jitters
            # self.vx_filtered = self.vx_filtered + self.lpf_coeff * (self.dx - self.vx_filtered)
            # odom.twist.twist.linear.x = self.vx_filtered
            # #odom.twist.twist.linear.x = self.vxMedFilt.step(odom.twist.twist.linear.x)
            # odom.twist.twist.linear.y = 0
            # odom.twist.twist.angular.z = -self.dr
            # #odom.twist.covariance = tuple(self.twistCovariance.ravel().tolist())
            # odom.twist.covariance = tuple(  [ 0.01, 0, 0, 0, 0, 0, \
            #                                   0, 0.001, 0, 0, 0, 0, \
            #                                   0, 0, 1e4, 0, 0, 0, \
            #                                   0, 0, 0, 1e4, 0, 0, \
            #                                   0, 0, 0, 0, 1e4, 0, \
            #                                   0, 0, 0, 0, 0, 0.0001 ])
            # self.odomPub.publish(odom)
            
            


    #############################################################################
    def lwheelCallback(self, enc):
    #############################################################################
        # enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, enc):
    #############################################################################
        # enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

#############################################################################
#############################################################################
#if __name__ == '__main__':
#    """ main """
#    try:
#        diffTf = DiffTf()
#        diffTf.spin()
#    except rospy.ROSInterruptException:
#        pass
