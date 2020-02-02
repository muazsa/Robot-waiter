#!/usr/bin/python

import rospy
from sensor_msgs.msg import LaserScan

if __name__ == '__main__':
	rospy.init_node('nan_filter')
	pub = rospy.Publisher('/laser_without_nan', LaserScan, queue_size=1)
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		scan_final = ()
		scan = rospy.wait_for_message('/scan_filtered', LaserScan)
		for i in range(len(scan.ranges)):
			if str(scan.ranges[i]) == 'nan':
				scan_final += (3.99,)
			else:
				scan_final += (scan.ranges[i],)
		scan.ranges = scan_final
		pub.publish(scan)
		rate.sleep()
			
	

