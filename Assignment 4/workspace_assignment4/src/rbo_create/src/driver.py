#!/usr/bin/python
import roslib
import rospy
from time import sleep
from irobot import Create
from threading import Thread
from math import sin,cos,pi
from datetime import datetime

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster

from rbo_create.msg import SensorPacket
from rbo_create.srv import *

class CreateDriver:
	def __init__(self):
		port = rospy.get_param('/rbo_create/port', "/dev/ttyUSB0")
		self.create = Create(port)
		self.packetPub = rospy.Publisher('sensorPacket', SensorPacket, queue_size=1)
		self.odomPub = rospy.Publisher('odom',Odometry, queue_size=1)
		self.odomBroadcaster = TransformBroadcaster()
		self.fields = ['wheeldropCaster','wheeldropLeft','wheeldropRight','bumpLeft','bumpRight','wall','cliffLeft','cliffFronLeft','cliffFrontRight','cliffRight','virtualWall','infraredByte','advance','play','distance','angle','chargingState','voltage','current','batteryTemperature','batteryCharge','batteryCapacity','wallSignal','cliffLeftSignal','cliffFrontLeftSignal','cliffFrontRightSignal','cliffRightSignal','homeBase','internalCharger','songNumber','songPlaying']
		self.then = datetime.now()
		self.x = 0
		self.y = 0
		self.th = 0
		self.create.update = self.sense

	def start(self):
		self.create.start()
		self.then = datetime.now()

	def stop(self):
		self.create.stop()

	def sense(self):
		now = datetime.now()
		elapsed = now - self.then
		self.then = now
		elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.
		d = self.create.d_distance / 1000.
		th = self.create.d_angle*pi/180
		dx = d / elapsed
		dth = th / elapsed

		if (d != 0):
			x = cos(th)*d
			y = -sin(th)*d
			self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
			self.y = self.y + (sin(self.th)*x + cos(self.th)*y)

		if (th != 0):
			self.th = self.th + th

		quaternion = Quaternion()
		quaternion.x = 0.0
		quaternion.y = 0.0
		quaternion.z = sin(self.th/2)
		quaternion.w = cos(self.th/2)

		self.odomBroadcaster.sendTransform(
			(self.x, self.y, 0),
			(quaternion.x, quaternion.y, quaternion.z, quaternion.w),
			rospy.Time.now(),
			"base_link",
			"odom"
			)

		odom = Odometry()
		odom.header.stamp = rospy.Time.now()
		odom.header.frame_id = "odom"
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0
		odom.pose.pose.orientation = quaternion

		odom.child_frame_id = "base_link"
		odom.twist.twist.linear.x = dx
		odom.twist.twist.linear.y = 0
		odom.twist.twist.angular.z = dth

		self.odomPub.publish(odom)

		packet = SensorPacket()
		for field in self.fields:
			packet.__setattr__(field,self.create.__getattr__(field))
		self.packetPub.publish(packet)


	def leds(self,req):
		print 'leds callback'
		self.create.leds(req.advance,req.play,req.color,req.intensity)
		return LedsResponse(True)

	def tank(self,req):
		print 'tank callback'
		self.create.tank(req.left,req.right)
		return TankResponse(True)

	def reset_odom(self,req):
		print 'reset_odom callback'
		if req.reset_odom:
			self.create.clear()
			self.x = 0
			self.y = 0
			self.th = 0
		return ResetOdomResponse(True)


	def twist(self,req):
		print 'twist callback'
		x = req.linear.x*1000.
		th = req.angular.z
		if (x == 0):
			th = th*180/pi
			speed = (8*pi*th)/9
			self.create.left(int(speed))
		elif (th == 0):
			x = int(x)
			self.create.tank(x,x)
		else:
			self.create.forwardTurn(int(x),int(x/th))

if __name__ == '__main__':
	node = rospy.init_node('create')
	driver = CreateDriver()

	rospy.Service('reset_odom', ResetOdom, driver.reset_odom)
	rospy.Service('leds', Leds, driver.leds)
	rospy.Service('tank', Tank, driver.tank)
	
	rospy.Subscriber("cmd_vel", Twist, driver.twist)

	sleep(1)
	driver.start()
	sleep(1)

	rospy.spin()
	driver.stop()
