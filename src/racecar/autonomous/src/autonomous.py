#! /usr/bin/env python

import rospy
import time
import math
import sys
import cv2
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from pid_controller import PID_Controller
from sign_finder import SignFinder

class Autonomous(object):
	def __init__(self):
		rospy.init_node('autonomous')
		rospy.Subscriber('/ekf_localization_node', Odometry, self.odom_callback)
		rospy.Subscriber('/scan', LaserScan, self.scan_callback)
		rospy.Subscriber('/zed/rgb/image_rect_color', Image, self.image_callback)
		#rospy.Subscriber('/ackermann_cmd', AckermannDriveStamped, self.drive_callback)
		self.ackermann_pub = rospy.Publisher('/ackermann_cmd_mux/input/teleop', AckermannDriveStamped)		

		#Visuals
		self.marker_pub = rospy.Publisher('/markers', MarkerArray)	
		self.marker_array = MarkerArray()

		self.robot_pos_pub = rospy.Publisher('/robot_marker', Marker)

		self.pid = PID_Controller()		

		self.pid.init(0.25, 0.0, 0.5)

		#Init vars
		self.WP_CLOSE_DIST = 0.16
		self.WP_PASS_THRESH = 0.6

		self.bridge = CvBridge()
		self.cv_img = None
		self.sign_state = -1
		self.speed = 0
		self.thr = 0
		self.steer_value = 0
		self.prev_t = time.time()
		self.current_x = 0
		self.current_y = 0
		self.current_rot = 0		
		self.pose_idx = 0
		self.ranges = 1000		

		#sign finder init
		self.sign_finder = SignFinder()

		#driveway waypoints
		#self.wps = [[0,0], [10.7,-1], [31.0,-7.4]]

		#road waypoints
		self.wps = [[0,0], [5.7, 0.0], [12.2, -0.75], [21.4, -1.6], [30.8, -2.38], [33.3, -3.0], [38.6, -3.6], [45.4, -4.5]]		

		#test waypoints
		#self.wps = [[0,0], [0.75,-0.5], [1,-0.75], [1.25,-1], [1.75,-1.5]]
		
		self.wp_idx = 1
		self.prev_wp = self.wps[self.wp_idx-1]
		self.current_wp = self.wps[self.wp_idx]

		self.current_dist = 0
		self.prev_dist = 0

		self.current_change_dist = 0
		self.prev_change_dist = 0

		self.finished = False

		while not rospy.is_shutdown():
			rospy.spin()

	def image_callback(self, img):
		self.cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

	def odom_callback(self, data):

		self.publish_wps()

		#Grab robot pos
		self.current_x = data.pose.pose.position.x
		self.current_y = data.pose.pose.position.y
		self.current_rot = data.pose.pose.orientation.z

		self.publish_robot_pos() #visualize robot pos

		rospy.loginfo("X: %f Y: %f Rot: %f" % (self.current_x, self.current_y, self.current_rot))
		
		#detect and classify sign
		self.sign_state = self.sign_finder.detect_and_classify_sign(self.cv_img)

		#PID var calc
		cte = self.calc_cte()
		t = time.time()
		dt = t - self.prev_t
		self.prev_t = t
		#rospy.loginfo("dt: %f" % dt)
		#rospy.loginfo("cte: %f" % cte)
		
		#Throttle Value
		self.thr = 5.0
		if abs(cte) > 0.5:
			#rospy.loginfo("acelerating...")
			self.thr = 6.0
		if abs(self.pid.p_error - cte) > 0.1 and abs(self.pid.p_error - cte) <= 0.2:
			#rospy.loginfo("stopped acelerating...")
			self.thr = 0.0
		elif abs(self.pid.p_error - cte) > 0.2 and self.speed > 3.2:
			#rospy.loginfo("decelerating...")
			self.thr = -4.5
		if self.speed > 6.4:
			#rospy.loginfo("normalizing at max speed...")
			self.thr = -1.6 
		if self.range < 0.8 or sign_state == 0:
			rospy.loginfo("---OBSTACLE DETECTED---" if self.range < 0.8 else "--STOP SIGN DETECTED--")
			self.thr = -20.0
		if sign_state == 1:
			rospy.loginfo("--YIELD SIGN DETECTED--")
			self.thr = -7.0
		self.speed += (self.thr/2)*dt**2		

		#Steer Value
		self.pid.updateError(cte, dt)
		self.steer_value = -self.pid.totalError()
		if self.steer_value > 1:
			self.steer_value = 1.0
		elif self.steer_value < -1:
			self.steer_value = -1.0

		self.current_dist = self.dist_to_waypoint()
		self.current_change_dist = self.current_dist - self.prev_dist

		rospy.loginfo("range: %f" % self.range)

		rospy.loginfo("dist: %f" % self.dist_to_waypoint())
		#rospy.loginfo("thr: %f steer_value: %f" % (self.thr, self.steer_value))

		#Check if passed waypoint
		if self.dist_to_waypoint() <= self.WP_CLOSE_DIST or (self.current_change_dist > 0 and self.prev_change_dist < 0):
			
			#check if over passing distance threshold
			if self.current_dist >= self.WP_PASS_THRESH:
				rospy.loginfo('UNSAFE WAYPOINT ERROR...STOPPING')
				self.speed = 0			
				self.steer_value = 0
				self.finsihed = True
			#stop if passed all waypoints
			elif self.wp_idx == len(self.wps)-1:
				self.speed = 0
				self.steer_value = 0
				rospy.loginfo("Finished following path!")
				self.finished = True
			#otherwise proceed to next waypoint
			else:
				rospy.loginfo("-----NEXT WAYPOINT-----")
				self.prev_wp = self.wps[self.wp_idx]
				self.wp_idx += 1
				self.current_wp = self.wps[self.wp_idx]

		self.prev_dist = self.current_dist
		self.prev_change_dist = self.current_change_dist

		#Publish drive vars
		ackermann_msg = AckermannDriveStamped()
		ackermann_msg.drive.speed = self.speed
		ackermann_msg.drive.steering_angle = self.steer_value
		if not self.finished: self.ackermann_pub.publish(ackermann_msg)

	def scan_callback(self, data):
		self.range = data.ranges[540]

	def calc_cte(self):
		x1 = self.prev_wp[0]
		y1 = self.prev_wp[1]

		x2 = self.current_wp[0]
		y2 = self.current_wp[1]
		
		dist = -(((y2-y1)*self.current_x - (x2-x1)*self.current_y + x2*y1 - y2*x1)/ math.sqrt((y2-y1)**2 + (x2-x1)**2))

		return dist

	def dist_to_waypoint(self):
		x1 = self.current_x
		y1 = self.current_y

		x2 = self.current_wp[0]
		y2 = self.current_wp[1]
		
		return math.sqrt((x2-x1)**2+(y2-y1)**2)

	def publish_wps(self):
		for i, wp in enumerate(self.wps):
			marker = Marker()
			marker.header.frame_id = '/map'
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.id = i
			marker.scale.x = 5.0
			marker.scale.y = 5.0
			marker.scale.z = 5.0
			marker.pose.position.x = wp[0]
			marker.pose.position.y = wp[1]
			marker.pose.position.z = 0
			marker.pose.orientation.w = 1.0
			marker.color.a = 0.3
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0

			self.marker_array.markers.append(marker)
		self.marker_pub.publish(self.marker_array)			

	def publish_robot_pos(self):
		self.pose_idx += 1
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.type = marker.SPHERE
		marker.action = marker.ADD
		marker.id = 1000+self.pose_idx
		marker.scale.x = 3
		marker.scale.y = 1.5
		marker.scale.z = 1.5
		marker.pose.position.x = self.current_x
		marker.pose.position.y = self.current_y
		marker.pose.position.z = 0
		marker.pose.orientation.z = self.current_rot
		marker.pose.orientation.w = 1.0
		marker.color.a = 1.0
		marker.color.r = 1.0
		marker.color.g = 0.0
		marker.color.b = 0.0

		self.robot_pos_pub.publish(marker)

if __name__ == '__main__':
	try:
		Autonomous()
	except rospy.ROSInterruptException:
		rospy.logerr('Could not start waypoint updater node.')
