#! /usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
# Import quaternion conversion function
from tf.transformations import quaternion_from_euler


def publish_tf():
	rospy.init_node('object_pointcloud_tf_broadcaster')

	tf_broadcaster = tf2_ros.TransformBroadcaster()
	i = 0
	while not rospy.is_shutdown():
		# Create a TransformStamped message
		transform_msg = TransformStamped()
		transform_msg.header.stamp = rospy.Time.now()
		transform_msg.header.frame_id = "PhoXi3Dscanner_sensor_MarkerSpace"  # Parent frame
		transform_msg.child_frame_id = "New_object_frame"  # Child frame

		# Set translation and rotation from your captured object position
		i += 0.001
		transform_msg.transform.translation.x = 0.3
		transform_msg.transform.translation.y = 0.1
		transform_msg.transform.translation.z = 0
		rospy.loginfo("============================================\n")
		# Calculate quaternion from Euler angles (roll, pitch, yaw)
		quaternion = quaternion_from_euler(1.5708/4, 1.5708, 0)  # Replace with actual values
		transform_msg.transform.rotation.x = quaternion[0]
		transform_msg.transform.rotation.y = quaternion[1]
		transform_msg.transform.rotation.z = quaternion[2]
		transform_msg.transform.rotation.w = quaternion[3]

		# Publish the transformation
		tf_broadcaster.sendTransform(transform_msg)
		rospy.sleep(0.1)  # Adjust sleep duration as needed


if __name__ == '__main__':
	try:
		publish_tf()
	except rospy.ROSInterruptException:
		pass


