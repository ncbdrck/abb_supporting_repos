#! /usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

def main():
    rospy.init_node('transform_listener_node')
    
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    
    source_frame = "New_object_frame"
    target_frame = "RoomComponents"
    # source_frame = "link_calib_tool"
    # target_frame = "link_6"

    try:
        # Wait for the transformation to become available
        tf_buf.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(5.0))
        
        # Get the transformation
        transform = tf_buf.lookup_transform(target_frame, source_frame, rospy.Time())
        
        # Print the transformation
        print("Translation:", transform.transform.translation)
        print("Rotation:", transform.transform.rotation)
        
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform lookup failed:", e)

if __name__ == '__main__':
    main()