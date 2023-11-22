#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import WheelsCmdStamped 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler
import math



class OdometryPublisher:
    def __init__(self):
        rospy.init_node('custom_odometry_publisher')
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/duckiebot/wheels_driver_node/wheels_cmd', WheelsCmdStamped, self.cmd_vel_callback)

        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.orientation.w=1

	self.sub_distancia = rospy.Subscriber("/duckiebot/distancia_X", Float32, self.callback_x)
        self.last_time = rospy.Time.now()
        self.current_time = rospy.Time.now()

        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster()
    def callback_x(self,msg):
	self.odom.pose.pose.position.x = msg.data

    def cmd_vel_callback(self, msg):
        # Assuming your robot's velocity commands are in m/s
        # linear_vel = msg.linear.x
        # angular_vel = msg.angular.z

      	 self.current_time = rospy.Time.now()
      	 dt = (self.current_time - self.last_time).to_sec()

        # Calculate the new odometry based on your custom motion model
        # You need to implement your custom odometry update logic here
        # For this example, we will assume simple linear motion
        # delta_x =  self.sub_distancia
        # delta_theta = angular_vel * dt

        # self.odom.pose.pose.position.x += delta_x
        # self.odom.pose.pose.orientation = quaternion_from_euler(0, 0, delta_theta)
        # self.odom.twist.twist.linear.x = linear_vel
        # self.odom.twist.twist.angular.z = angular_vel

       	 self.last_time = self.current_time

        # Publish the updated odometry
       	 self.odom.header.stamp = self.current_time
       	 self.odom_pub.publish(self.odom)
	
        # Publish the transform between odom and base_link
       	 self.tf_broadcaster.sendTransform(
            (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z),
            (self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w),
            self.current_time,
            "base_link",
            "odom"
        )

if __name__ == '__main__':
    try:
        odom_publisher = OdometryPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



