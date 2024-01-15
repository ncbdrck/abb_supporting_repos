#! /usr/bin/env python
import threading
import rospy
import sys
from std_msgs.msg import String, Bool
from phoxi_camera.srv import GetFrame, GetFrameRequest
import time
import atexit 
import signal

conv_speed = 0.245 # m/s
previous_state = "1"
current_state = ""
offset_dist = 0.4 # 40 mm
initial_state_flag = True
object_queue_flag = False

def initial_state():
  global initial_state_flag
  while current_state != "1":
    pass
  else:
    rospy.loginfo("Starting the system")
    rospy.loginfo("Conveyor ON")
    conv_pub_state.publish(True)
    rospy.loginfo("Gripper Closed")
    gripper_pub_state.publish(True)
    initial_state_flag = False

def call_get_frame_service():
  get_frame_service = rospy.ServiceProxy('/phoxi_camera/get_frame', GetFrame)
  request = GetFrameRequest()
  request.in_ = -1
  response = get_frame_service(request)
  rospy.loginfo("Received frame: %s" % response)


def sensor_value_callback(msg):
  global previous_state
  global current_state
  current_state = msg.data
  if previous_state == "1" and current_state == "0":
    time.sleep(0.6)
    rospy.loginfo("Conveyor OFF")
    conveyor_state = False 
    conv_pub_state.publish(conveyor_state)
    call_get_frame_service()
    time.sleep(1)
    conv_pub_state.publish(True)
    time.sleep(offset_dist / conv_speed)
    conv_pub_state.publish(False)
  previous_state = current_state

def object_queue_callback(msg):
  global object_queue_flag
  object_queue_flag = msg.data
  if not object_queue_flag:
    time.sleep(0.5)
    conv_pub_state.publish(True)
    rospy.loginfo("Conveyor ON")

def exit_handler(signal, frame):
  rospy.loginfo("Ctrl+C detected.")
  conv_pub_state.publish(False)
  rospy.loginfo("Conveyor OFF")
  gripper_pub_state.publish(True)
  rospy.loginfo("Gripper open")
  rospy.signal_shutdown("Exiting the program...")
  sys.exit(0)

def main():
  global conv_pub_state
  global gripper_pub_state
  rospy.init_node('main_app_node', anonymous=True)
  rospy.Subscriber("conv_sensor_state_topic", String, sensor_value_callback)
  rospy.Subscriber('object_queue_topic', Bool, object_queue_callback)
  conv_pub_state = rospy.Publisher("conveyor_state_topic", Bool, queue_size=10)
  gripper_pub_state = rospy.Publisher("gripper_state_topic", Bool, queue_size=10)
  time.sleep(1)
  if initial_state_flag == True:
    concurrent_thread = threading.Thread(target=initial_state)
    concurrent_thread.start()
  signal.signal(signal.SIGINT, exit_handler)
  rospy.spin()


if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass