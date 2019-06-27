#!/usr/bin/env python

import rospy
import math
from motor_control.msg import  motor_Values
from std_msgs.msg import Float64

"""
========================================

          0
          ^
          |
   ______ | ______
  /  hfl| 1 |hfr  \
 / / /  |   |  \ \ \
/  vfl()|___|()vfr  \
|                   |
| -1              1 | ---> 270
|        ___        |
\  vbl()|   |()vbr  /
 \ \ \  |   |  / / /
  \__hbl|-1 |hbr__/
	  |
	  |
	  v
	 180

========================================
"""


class motor:
	def __init__(self, angle, x, y):
		self.angle = angle
		self.x = x
		self.y = y
		self.thrust = 100

hfr = motor(45, 1, 1)
hfl = motor(315, -1, 1)
hbr = motor(315, 1, -1)
hbl = motor(45, -1, -1)

esc_f_min = 1550
esc_f_max = 1700
esc_r_min = 1450
esc_r_max = 1300

trans_vector = 45
trans_magnitude = 100
rotate_vector = 1 #1 clockwise, -1 counter
rotate_magnitude = 5

def scale_map(x, in_min, in_max, out_min, out_max): 
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def calc_trans_thrust(trans_vector, trans_magnitude, motor_angle):
	return (math.cos(math.radians(trans_vector-motor_angle)) * trans_magnitude)

def calc_rotational(rotate_vector, rotate_magnitude, x, y):
	combined = rotate_vector * rotate_magnitude	
	if x > 0:
		return -1 * combined
	elif x < 0:
		return combined

def normalize(max):
	if abs(max)> 100:
		return 100.0/abs(max)
	else:
		return 1

def trans_vector_callback(data):
	global trans_vector
	trans_vector = data.data

def trans_mag_callback(data):
	global trans_magnitude
	trans_magnitude = data.data

def rotate_vector_callback(data):
	global rotate_vector
	rotate_vector = data.data

def rotate_mag_callback(data):
	global rotate_magnitude
	rotate_magnitude = data.data

def main():
        motor_pub = rospy.Publisher('thruster_values', motor_Values, queue_size=10)
	motor_msg = motor_Values()
	
	rospy.Subscriber('translational_vector', Float64, trans_vector_callback)
	rospy.Subscriber('translational_magnitude', Float64, trans_mag_callback)
	rospy.Subscriber('rotational_vector', Float64, rotate_vector_callback)
	rospy.Subscriber('rotational_magnitude', Float64, rotate_mag_callback)
	
	rospy.init_node('motor_mixer', anonymous=True)

	while not rospy.is_shutdown():
		hfr.thrust = calc_trans_thrust(trans_vector, trans_magnitude, hfr.angle)
		hfr.thrust = hfr.thrust + calc_rotational(rotate_vector, rotate_magnitude, hfr.x, hfr.y)
		hfr.thrust = hfr.thrust * normalize(hfr.thrust)

		hfl.thrust = calc_trans_thrust(trans_vector, trans_magnitude, hfl.angle)
		hfl.thrust = hfl.thrust + calc_rotational(rotate_vector, rotate_magnitude, hfl.x, hfl.y)
		hfl.thrust = hfl.thrust * normalize(hfl.thrust)

		hbr.thrust = calc_trans_thrust(trans_vector, trans_magnitude, hbr.angle)
		hbr.thrust = hbr.thrust + calc_rotational(rotate_vector, rotate_magnitude, hbr.x, hbr.y)
		hbr.thrust = hbr.thrust * normalize(hbr.thrust)

		hbl.thrust = calc_trans_thrust(trans_vector, trans_magnitude, hbl.angle)
		hbl.thrust = hbl.thrust + calc_rotational(rotate_vector, rotate_magnitude, hbl.x, hbl.y)
		hbl.thrust = hbl.thrust * normalize(hbl.thrust)

			

		motor_msg.hrf = hfr.thrust
		motor_msg.hlf = hfl.thrust
		motor_msg.hrb = hbr.thrust
		motor_msg.hlb = hbl.thrust
		motor_msg.vrf = 0
		motor_msg.vlf = 0
		motor_msg.vrb = 0
		motor_msg.vlb = 0
		
		motor_pub.publish(motor_msg)

if __name__ == "__main__":
	main()
