#! /usr/bin/env python

import math
import numpy as np
import rospy
import moveit_commander
import geometry_msgs.msg
import copy
from moveit_msgs.msg import DisplayTrajectory
import sys
import time
import tf2_ros
from computer_vision_pkg.msg import PosOriObjectsArray
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from  tf.transformations import quaternion_from_euler, euler_from_matrix, euler_from_quaternion


numObj = 0

class robotic_arm_control(object):
  
  def __init__(self, move_group_name, end_effector_link=""):
    super(robotic_arm_control, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_" + move_group_name + "_python_interface", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander(move_group_name)
    move_group.set_max_velocity_scaling_factor(1)
    move_group.set_max_acceleration_scaling_factor(1)
    if end_effector_link:
      move_group.set_end_effector_link(end_effector_link)

    
    display_trajectory_pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)

    print("\n\n")
    print("=========== Planning frame:             %s  ===========\n" % move_group.get_planning_frame())
    print("=========== End effector link:          %s  ===========\n" % move_group.get_end_effector_link())
    group_names = robot.get_group_names()
    print("=========== Available Planning Groups:  %s ===========\n" % robot.get_group_names())
    print("\n\n")
    print("========================= Printing robot state =========================\n")
    print(robot.get_current_state())
    print("\n\n\n")

    self.box_name = ""
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_pub = display_trajectory_pub
    self.planning_frame = move_group.get_planning_frame()
    self.eef_link = move_group.get_end_effector_link()
    self.group_names = group_names


  def get_time_info(self, start_time):
    final_time = time.time() - start_time
    rospy.loginfo("Time to complete the Pick and Place task: --- %.4f seconds ---\n", final_time)
    rospy.loginfo("Parts per minute: --- %.4f ppm ---\n\n", 60 / final_time)


  def get_current_joint_state(self):
    mv = self.move_group
    current_joint_state = np.array(mv.get_current_joint_values())
    print("\n")
    rospy.loginfo("============================================\n")
    rospy.loginfo("======== Current robot joint state: ========\n")
    for i, joint in enumerate(current_joint_state):
      print("Joint " + str(i + 1) + ": " + str(joint))
    print("\n")
    rospy.loginfo("============================================")
    return current_joint_state


  def get_current_pose(self):
    mv = self.move_group
    current_pose = mv.get_current_pose().pose
    print("\n")
    rospy.loginfo("============================================\n")
    rospy.loginfo("============ Current robot pose: ===========\n")
    print(current_pose, "\n")
    quaternion = [current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]
    euler_angles = euler_from_quaternion(quaternion)
    roll, pitch, yaw = euler_angles
    print("Orientation in Euler:")
    print("  Roll (x):  {:.2f} degrees".format(roll * 180.0 / 3.14159265359))
    print("  Pitch (y): {:.2f} degrees".format(pitch * 180.0 / 3.14159265359))
    print("  Yaw (z):   {:.2f} degrees".format(yaw * 180.0 / 3.14159265359))
    rospy.loginfo("============================================")
    return current_pose


  def go_to_joint_values(self, joint_value_array):
    mv = self.move_group
    joint_goal = joint_value_array
    print("JOINT GOAL:\n", joint_goal)
    print("\n\n")
    mv.set_joint_value_target(joint_goal)
    mv.set_start_state_to_current_state()
    plan = mv.go(joint_goal, wait=True)
    if plan:
      rospy.loginfo("======== Robot successfully moved to the joint values. ========\n\n")
    else:
      rospy.loginfo("======== Failed to move the robot to the joint values. ========\n\n")
    rospy.loginfo("Robot movement completed.")
    # mv.stop()


  def go_to_pose(self, pos, ori, zoffset=0):
    mv = self.move_group
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position = geometry_msgs.msg.Point(pos[0], pos[1], pos[2] + zoffset)
    pose_goal.orientation = geometry_msgs.msg.Quaternion(ori[0], ori[1], ori[2], ori[3])
    print("POSE GOAL:\n", pose_goal)
    print("\n\n")
    mv.set_pose_target(pose_goal)
    mv.set_start_state_to_current_state()
    plan = mv.go(wait=True)
    if plan:
      rospy.loginfo("======== Robot successfully moved to the target pose. ========\n\n")
    else:
      rospy.loginfo("======== Failed to move the robot to the target pose. ========\n\n")
    rospy.loginfo("Robot movement completed.")


  def go_to_home_position(self):
    self.go_to_joint_values([0, 0, 0, 0, 0, 0])


  def go_to_safe_position(self):
    self.go_to_joint_values([0, 0.440393328666687, 0.20272229611873627, 0, 0.9276748895645142, 0.009913647547364235])


  def plan_cartesian_path(self):
    move_group = self.move_group
    waypoints = []
    wpose = move_group.get_current_pose().pose
    # Pick position + Safe pick position
    wpose.position = geometry_msgs.msg.Point(0.5770878727179302, -4.106296192679349e-05, 1.0369877233155314)
    wpose.orientation = geometry_msgs.msg.Quaternion(0.0034683330723721886, 0.7070437373201718, -0.003492266722859689, 0.7071526909056908)
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

    return plan


  def get_object_tf(self):
    tf_buf = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buf)
    object_trans, object_rot = [], []
    for i in range(numObj):
      source_frame = "New_object_frame_" + str(i + 1)
      target_frame = "RoomComponents"
      try:
        tf_buf.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(5.0))
        transform = tf_buf.lookup_transform(target_frame, source_frame, rospy.Time())
        print("Translation:", transform.transform.translation)
        print("Rotation:", transform.transform.rotation)
        translation = [transform.transform.translation.x, transform.transform.translation.y + 0.4, transform.transform.translation.z + 0.007]
        rotation = [transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w]
        object_trans.append(translation)
        object_rot.append(rotation)
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn("Transform lookup failed: %s", e)
    print("object_trans",np.array(object_trans))
    print("object_rot",np.array(object_rot))
    return [object_trans, object_rot]

  def plan_cartesian_path2(self, pos, ori):
    move_group = self.move_group
    waypoints = []
    wpose = move_group.get_current_pose().pose
    wpose.position = geometry_msgs.msg.Point(pos[0], pos[1], pos[2])
    wpose.orientation = geometry_msgs.msg.Quaternion(ori[0], ori[1], ori[2], ori[3])
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
    return plan


  def display_trajectory(self, plan):
    robot = self.robot
    display_trajectory_pub = self.display_trajectory_pub

    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    display_trajectory_pub.publish(display_trajectory)


  def execute_plan(self, plan):
    move_group = self.move_group
    move_group.execute(plan, wait=True)


def object_data_callback(msg):
  global numObj
  numObj = msg.numObj


def main():
  global numObj
  robot_interface = robotic_arm_control("robotic_arm", "link_tool_base")
  rospy.Subscriber('object_pos_array_topic', PosOriObjectsArray, object_data_callback)
  gripper_pub_state = rospy.Publisher("gripper_state_topic", Bool, queue_size=10)
  pub_object_queue = rospy.Publisher("object_queue_topic", Bool, queue_size=10)
  robot_interface.go_to_home_position()
  robot_interface.go_to_safe_position()
  is_object_array_finished = False
  x_offset = 0
  while not rospy.is_shutdown():
    if numObj > 0:
      # input()
      pub_object_queue.publish(True)
      start_time = time.time()
      obj_data = robot_interface.get_object_tf()
      for i in range(numObj):
        robot_interface.go_to_pose(obj_data[0][i], obj_data[1][i], 0.02)
        # robot_interface.go_to_pose(obj_data[0][i], obj_data[1][i])
        pick_p_plan = robot_interface.plan_cartesian_path2(obj_data[0][i], obj_data[1][i])
        robot_interface.execute_plan(pick_p_plan)
        gripper_pub_state.publish(False)
        robot_interface.go_to_pose(obj_data[0][i], obj_data[1][i], 0.02)
        numObj -= 1
        if numObj == 0:
          rospy.loginfo("======== Object queue empty. Pick and place task completed ========\n\n")
          pub_object_queue.publish(False)
        
        robot_interface.go_to_pose([0.3497924970477979, 0.2111415625092714, 0.9985210253353636], obj_data[1][i])
        
        robot_interface.go_to_pose([-0.2 + x_offset, 0.61, 0.74], [0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273], 0.02)
        robot_interface.go_to_pose([-0.2 + x_offset, 0.61, 0.74], [0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273])
        gripper_pub_state.publish(True)
        robot_interface.go_to_pose([-0.2 + x_offset, 0.61, 0.74], [0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273], 0.02)
        
        robot_interface.go_to_pose([0.3497924970477979, 0.2111415625092714, 0.9985210253353636], obj_data[1][i])
        robot_interface.get_time_info(start_time)
        x_offset += 0.03
      is_object_array_finished = True
    else:
      if is_object_array_finished:
        robot_interface.go_to_safe_position()
        is_object_array_finished = False
      
  robot_interface.go_to_home_position()
  rospy.spin()

    
if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass









# def main():
#   global numObj
#   robot_interface = robotic_arm_control("robotic_arm", "link_tool_base")
#   rospy.Subscriber('object_pos_array_topic', PosOriObjectsArray, object_data_callback)
#   pub_object_queue = rospy.Publisher("object_queue_topic", Bool, queue_size=10)
#   robot_interface.go_to_home_position()
#   # robot_interface.go_to_safe_position()
#   while not rospy.is_shutdown():
#     if numObj > 0:
#       print("ENTROOOOUUU")
#       pub_object_queue.publish(True)
#       start_time = time.time()
#       obj_data = robot_interface.get_object_tf()
#       is_first_run = True
#       for i in range(numObj):
#         pick_pos = robot_interface.plan_cartesian_path2(obj_data[0][i], obj_data[1][i], is_first_run)
#         is_first_run = False
#         robot_interface.execute_plan(pick_pos)
#         numObj -= 1
#         if numObj == 0:
#           rospy.loginfo("======== Object queue empty. Pick and place task completed ========\n\n")
#           pub_object_queue.publish(False)
#         place_pos = robot_interface.plan_cartesian_path2([-0.002057268666567913, 0.6091312095477686, 0.75], [0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273], False)
#         robot_interface.execute_plan(place_pos)
#         robot_interface.get_time_info(start_time)
#     else:
#       robot_interface.go_to_safe_position()
#   robot_interface.go_to_home_position()
#   rospy.spin()












  # def plan_cartesian_path(self):
  #   move_group = self.move_group
  #   waypoints = []
  #   wpose = move_group.get_current_pose().pose
    
  #   # Pick position + Safe pick position
  #   wpose.position = geometry_msgs.msg.Point(0.5770878727179302, -4.106296192679349e-05, 1.0369877233155314)
  #   wpose.orientation = geometry_msgs.msg.Quaternion(0.0034683330723721886, 0.7070437373201718, -0.003492266722859689, 0.7071526909056908)
  #   waypoints.append(copy.deepcopy(wpose))
  #   # wpose.position = geometry_msgs.msg.Point(0.5639554777419222, -0.2625916768118757, 0.8543393465916667)
  #   # wpose.orientation = geometry_msgs.msg.Quaternion(-0.6657379844233057, 0.22043880633506216, 0.6766989126727414, 0.22422812121831837)
  #   # waypoints.append(copy.deepcopy(wpose))
  #   # wpose.position = geometry_msgs.msg.Point(0.5770878727179302, -4.106296192679349e-05, 1.0369877233155314)
  #   # wpose.orientation = geometry_msgs.msg.Quaternion(0.0034683330723721886, 0.7070437373201718, -0.003492266722859689, 0.7071526909056908)
  #   # waypoints.append(copy.deepcopy(wpose))

  #   # Place position + safe place position
  #   # wpose.position = geometry_msgs.msg.Point(-0.002057268666567913, 0.6091312095477686, 0.91602916)
  #   # wpose.orientation = geometry_msgs.msg.Quaternion(0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273)
  #   # waypoints.append(copy.deepcopy(wpose))
  #   # wpose.position = geometry_msgs.msg.Point(-0.002057268666567913, 0.6091312095477686, 0.7360291604031564)
  #   # wpose.orientation = geometry_msgs.msg.Quaternion(0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273)
  #   # waypoints.append(copy.deepcopy(wpose))
  #   # wpose.position = geometry_msgs.msg.Point(-0.002057268666567913, 0.6091312095477686, 0.91602916)
  #   # wpose.orientation = geometry_msgs.msg.Quaternion(0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273)
  #   # waypoints.append(copy.deepcopy(wpose))

  #   # Safe pick position
  #   # wpose.position = geometry_msgs.msg.Point(0.5770878727179302, -4.106296192679349e-05, 1.0369877233155314)
  #   # wpose.orientation = geometry_msgs.msg.Quaternion(0.0034683330723721886, 0.7070437373201718, -0.003492266722859689, 0.7071526909056908)
  #   # waypoints.append(copy.deepcopy(wpose))
    
  #   (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

  #   return plan