#!/usr/bin/env python3

import rospy
import math
# import the messages for reading the joint positions and sending joint commands
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Header

# the call back function for a shorter version of the position feedback
def pose_callback(data):
	# loop through the values for 6 joints
	for joint_no in range(6):
		# read the joint name
		joint_name = data.name[joint_no]
		# read the joint position value
		joint_pos = data.position[joint_no]
		# show the results
		monitoring_messge = joint_name + ' is at %0.2f radians'
		rospy.loginfo(monitoring_messge, joint_pos)


if __name__ == '__main__':
	# initialize the node
	rospy.init_node('simple_pose_control', anonymous = True)
	# add a subscriber to it to read the position information
	rospy.Subscriber('/xarm/joint_states', JointState, pose_callback)
	# add a publisher for sending joint position commands
	pos_pub = rospy.Publisher('/xarm/xarm6_traj_controller/command', JointTrajectory, queue_size = 10)
	# set a 10Hz frequency for this loop
	loop_rate = rospy.Rate(10)

	# define a joint trajectory variable for sending the control commands
	pos_cmd = JointTrajectory()
	pos_cmd_point = JointTrajectoryPoint()
	# just a quick solution to complete the message template
	pos_cmd.joint_names.append('joint1')
	pos_cmd.joint_names.append('joint2')
	pos_cmd.joint_names.append('joint3')
	pos_cmd.joint_names.append('joint4')
	pos_cmd.joint_names.append('joint5')
	pos_cmd.joint_names.append('joint6')
	
	# initialize the position command to zero
	for joint_no in range(6):
		pos_cmd_point.positions.append(0.0)
	# set the ideal time to destination
	pos_cmd_point.time_from_start = rospy.Duration(1.0) # here one second 
	# change the value of the command for the second joint
	pos_cmd_point.positions[1] = -math.pi/4
	# change the value of the command for the third joint
	pos_cmd_point.positions[2] = -math.pi/2
	# add the trajectory point to the command
	pos_cmd.points.append(pos_cmd_point)
	# define a message header	
	header = Header()
	
	while not rospy.is_shutdown():
		# update the header with the most recent time stamp
		header.stamp = rospy.Time.now()
		# use the most recent header in the position command message
		pos_cmd.header = header
		# publish the message
		pos_pub.publish(pos_cmd)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
