#! /usr/bin/env python
import math
import rospy
import numpy as np
from math import pi
from computer_vision_pkg.msg import PosOriObjectsArray
from geometry_msgs.msg import Point, Quaternion
import tf2_ros
from geometry_msgs.msg import TransformStamped, Quaternion
from  tf.transformations import quaternion_from_euler, euler_from_matrix
import threading
from std_msgs.msg import Bool


aux = []
conv_speed = 0.255256266 # m/s
object_queue_flag = True

def object_data_decoder(array):
  numObj = array.numObj
  meanArray = np.array(array.meanArray).reshape(numObj, 3)
  eivecArray = np.array(array.eivecArray).reshape(numObj, 3, 3)
  pcArray = np.array(array.pcArray).reshape(numObj, 3)
  print("\n\nNumber of objects detected: ", numObj)
  print("\n\nMean values:\n", meanArray)
  print("\n\nEigenvector values:\n", eivecArray)
  print("\n\nPrincipal component values:\n", pcArray)
  obj_array = list(zip(meanArray, eivecArray, pcArray))
  return obj_array

def object_data_callback(msg):
  global aux
  obj_data_array = object_data_decoder(msg)
  if len(obj_data_array) > 0:
    aux = obj_data_array.copy()


def remove_transform(i):
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    transform_to_remove = TransformStamped()
    transform_to_remove.header.frame_id = "PhoXi3Dscanner_sensor_MarkerSpace"
    transform_to_remove.child_frame_id = "New_object_frame_" + str(i)
    rospy.sleep(1.0)
    if tf_buffer.can_transform(transform_to_remove.child_frame_id, transform_to_remove.header.frame_id, rospy.Time(0)):
        tf_buffer.clear(transform_to_remove.child_frame_id, transform_to_remove.header.frame_id)
        rospy.loginfo(f"Removed transform from '{transform_to_remove.child_frame_id}' to '{transform_to_remove.header.frame_id}'")
    else:
        rospy.logwarn(f"Transform from '{transform_to_remove.child_frame_id}' to '{transform_to_remove.header.frame_id}' does not exist.")


def pc_vector_to_angle(mean, pc):
  vector_mean_to_pc = [pc[0] - mean[0], pc[1] - mean[1], pc[2] - mean[2]] 
  magnitude = math.sqrt(vector_mean_to_pc[0]**2 + vector_mean_to_pc[1]**2 + vector_mean_to_pc[2]**2)
  normalized_vector = [vector_mean_to_pc[0] / magnitude, vector_mean_to_pc[1] / magnitude, vector_mean_to_pc[2] / magnitude]
  yaw_radians = math.asin(normalized_vector[1])
  roll_degrees = 0
  pitch_degrees = pi/2
  yaw_degrees = math.degrees(yaw_radians)
  # print("******************************* yaw_degrees:", yaw_degrees)
  return (roll_degrees, pitch_degrees, yaw_radians)



def object_tf_broadcaster():
  global aux
  global object_queue_flag
  tf_broadcaster = tf2_ros.TransformBroadcaster()
  rate = rospy.Rate(10)  # Hz
  while not rospy.is_shutdown():
    if object_queue_flag:
      if aux:
        # rospy.loginfo("Creating tfs for the %i objects detected.", len(aux))
        for i, obj in enumerate(aux):
          mean = obj[0]
          eivec = obj[1]
          pc = obj[2]
          transform_msg = TransformStamped()
          transform_msg.header.stamp = rospy.Time.now()
          transform_msg.header.frame_id = "PhoXi3Dscanner_sensor_MarkerSpace"  # Parent frame
          transform_msg.child_frame_id = "New_object_frame_" + str(i + 1)  # Child frame
          transform_msg.transform.translation = Point(mean[0], mean[1], mean[2])
          # roll, pitch, yaw = euler_from_matrix(eivec)
          roll, pitch, yaw = pc_vector_to_angle(mean, pc)
          quaternion = quaternion_from_euler(roll, pitch, yaw)
          transform_msg.transform.rotation = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
          tf_broadcaster.sendTransform(transform_msg)
      # else:
      #   rospy.loginfo("Waiting for part detection.")
    else:
      for i, obj in enumerate(aux):
        remove_transform(i+1)
      aux = []
      object_queue_flag = True
      # rospy.loginfo("Waiting for part detection.")
    rate.sleep()
    
    # # Check if the transform exists in the buffer


def object_queue_callback(msg):
  global object_queue_flag
  object_queue_flag = msg.data
  
  
def main():
  rospy.init_node('object_tf_broadcaster', anonymous=True)
  rospy.Subscriber('object_pos_array_topic', PosOriObjectsArray, object_data_callback)
  rospy.Subscriber('object_queue_topic', Bool, object_queue_callback)
  
  concurrent_thread = threading.Thread(target=object_tf_broadcaster)
  concurrent_thread.start()
  
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass