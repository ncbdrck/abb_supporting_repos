#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from  tf.transformations import euler_from_quaternion

def main():
  rospy.init_node('transform_listener_node')
  
  tf_buf = tf2_ros.Buffer()
  tf_listener = tf2_ros.TransformListener(tf_buf)
  
  source_frame = input("enter the frame name: ")
  if not source_frame:
    source_frame = "link_tool_base"
  target_frame = "RoomComponents"
  # source_frame = "link_calib_tool"
  # target_frame = "link_6"
  rospy.loginfo(f"Frame name: {source_frame}")
  try:
    # Wait for the transformation to become available
    tf_buf.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(5.0))
    
    # Get the transformation
    transform = tf_buf.lookup_transform(target_frame, source_frame, rospy.Time())
    
    quaternion = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
    euler_angles = euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler_angles
    # Print the transformation
    print("Translation:", transform.transform.translation)
    print("Rotation:", transform.transform.rotation)
    print("Orientation in Euler:")
    print("  Roll (x):  {:.2f} degrees".format(roll * 180.0 / 3.14159265359))
    print("  Pitch (y): {:.2f} degrees".format(pitch * 180.0 / 3.14159265359))
    print("  Yaw (z):   {:.2f} degrees".format(yaw * 180.0 / 3.14159265359))
    rospy.loginfo("============================================")
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    rospy.logwarn("Transform lookup failed:", e)

if __name__ == '__main__':
  main()