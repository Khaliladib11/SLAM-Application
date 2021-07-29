#! /usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving Forward Press: W
Moving Backward Press: X
To Turn Righ Press: D
To Turn Left Press: A
TO Stop Moving Press: S

To Increase The Speed Press: Q
To Decrease The Speed Press: R
"""""


key_mapping = {
	'w': [0, 1], 'x': [0, -1], 'a': [-1 ,0], 'd': [1, 0], 's': [0, 0]
}

vel_scales = 0.5 # default to very slow

def keys_cb(msg, twist_pub):
	global vel_scales
	if msg.data == 'q':
		if vel_scales < 250:
			vel_scales = vel_scales + 50
	if msg.data == 'r':
		if vel_scales > 0:
			vel_scales = vel_scales - 50

	if len(msg.data) == 0 or msg.data[0] not in key_mapping:
		return
	vels = key_mapping[msg.data[0]]
	t = Twist()
	t.angular.z = vels[0] * vel_scales
	t.linear.x = vels[1] * vel_scales
	twist_pub.publish(t)

if __name__ == '__main__':
	print(msg)
	rospy.init_node('keys_to_twist')
	twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	rospy.Subscriber('keys', String, keys_cb, twist_pub)
	if rospy.has_param('~linear_scale'):
		vel_scales[1] = rospy.get_param('~linear_scale')
	if rospy.has_param('~angular_scale'):
		vel_scales[0] = rospy.get_param('~angular_scale')
	rospy.spin()
