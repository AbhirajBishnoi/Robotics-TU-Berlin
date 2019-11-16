#!/usr/bin/python
import rospy
from time import sleep

from rbo_create.srv import Tank, ResetOdom
from nav_msgs.msg import Odometry

x = 0
# subscribe to odometry topic
def odom_callback(data):
	global x
	x = data.pose.pose.position.x
	
rospy.Subscriber('odom', Odometry, odom_callback)

# define tank service from driver
node = rospy.init_node('test')
print 'Waiting for create driver ...'
rospy.wait_for_service('tank')
tank = rospy.ServiceProxy('tank', Tank)
rospy.wait_for_service('reset_odom')
reset_odom = rospy.ServiceProxy('reset_odom', ResetOdom)
print 'Create driver is ready!'

# now, tank(left, right) and reset_odom(bool) can be used as functions 
#   left/right: velocities in mm/s (maximum 500)

sleep(1)
reset_odom(True)
print 'Starting the test.'
sleep(1)
print 'x: ', x

tank(-100,100)
sleep(2)
tank(0,0)
sleep(1)
print 'x: ', x

tank(-100,-100)
sleep(2)
tank(0,0)
sleep(1)
print 'x: ', x

print 'Test done.'

rospy.spin()
