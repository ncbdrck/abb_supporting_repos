import rospy
import moveit_commander
import geometry_msgs.msg
import copy
from moveit_msgs.msg import DisplayTrajectory
# import moveit_msgs.msg
import sys
import time


class robotic_arm_control(object):
  def __init__(self):
    super(robotic_arm_control, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("robotic_arm")
    move_group.set_max_velocity_scaling_factor(1)
    move_group.set_max_acceleration_scaling_factor(1)
    # Create a `DisplayTrajectory`_ ROS publisher which is used to display
    # trajectories in Rviz:
    display_trajectory_pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=20)

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL basic_info
    ##
    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    # END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ""
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_pub = display_trajectory_pub
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group
    tcp_offset = 0.174538 + 0.01
    # BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    # Cartesian Paths
    # ^^^^^^^^^^^^^^^
    # You can plan a Cartesian path directly by specifying a list of waypoints
    # for the end-effector to go through. If executing  interactively in a
    # Python shell, set scale = 1.0.
    ##
    waypoints = []

    move_group.set_end_effector_link("link_tool_base")
    wpose = move_group.get_current_pose().pose

    # Pick position + Safe pick position
    wpose.position = geometry_msgs.msg.Point(0.5770878727179302, -4.106296192679349e-05, 1.0369877233155314)
    wpose.orientation = geometry_msgs.msg.Quaternion(0.0034683330723721886, 0.7070437373201718, -0.003492266722859689, 0.7071526909056908)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position = geometry_msgs.msg.Point(0.5639554777419222, -0.2625916768118757, 0.8543393465916667)
    wpose.orientation = geometry_msgs.msg.Quaternion(-0.6657379844233057, 0.22043880633506216, 0.6766989126727414, 0.22422812121831837)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position = geometry_msgs.msg.Point(0.5770878727179302, -4.106296192679349e-05, 1.0369877233155314)
    wpose.orientation = geometry_msgs.msg.Quaternion(0.0034683330723721886, 0.7070437373201718, -0.003492266722859689, 0.7071526909056908)
    waypoints.append(copy.deepcopy(wpose))

    # Place position + safe place position
    wpose.position = geometry_msgs.msg.Point(-0.002057268666567913, 0.6091312095477686, 0.91602916)
    wpose.orientation = geometry_msgs.msg.Quaternion(0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position = geometry_msgs.msg.Point(-0.002057268666567913, 0.6091312095477686, 0.7360291604031564)
    wpose.orientation = geometry_msgs.msg.Quaternion(0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position = geometry_msgs.msg.Point(-0.002057268666567913, 0.6091312095477686, 0.91602916)
    wpose.orientation = geometry_msgs.msg.Quaternion(0.49289697284975587, -0.49843134939787304, -0.5040802759434219, -0.5045015753174273)
    waypoints.append(copy.deepcopy(wpose))

    # Home
    wpose.position = geometry_msgs.msg.Point(0.7075609903351935, 8.771360381956032e-06, 1.615072231768826)
    wpose.orientation = geometry_msgs.msg.Quaternion(-2.4228410042164162e-05, 1.3660089512519863e-05, 6.1984752866432076e-06, 0.9999999995939822)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_pub = self.display_trajectory_pub

    # BEGIN_SUB_TUTORIAL display_trajectorytcp_offset
    # Displaying a Trajectory
    # ^^^^^^^^^^^^^^^^^^^^^^^
    # You can ask RViz to visualize a plan (aka trajectory) for you. But the
    # group.plan() method does this automatically so this is not that useful
    # here (it just displays the same trajectory again):
    ##
    # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    # We populate the trajectory_start with our current robot state to copy over
    # any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_pub.publish(display_trajectory)


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    # BEGIN_SUB_TUTORIAL execute_plan
    ##
    # Executing a Plan
    # ^^^^^^^^^^^^^^^^
    # Use execute if you would like the robot to follow
    # the plan that has already been computed:
    move_group.execute(plan, wait=True)

    # **Note:** The robot's current joint state must be within some tolerance of the
    # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    # END_SUB_TUTORIAL


def main():
  # tcp_offset = 0.174538 + 0.01
  # rospy.init_node('move_to_multiple_poses', anonymous=True)
  # robot = moveit_commander.RobotCommander()
  # move_group = moveit_commander.MoveGroupCommander("robotic_arm")

  # # Define a list of target poses

  # while not rospy.is_shutdown():
  #   move_group.set_pose_targets(target_poses)

  #   # Plan and execute the motion for all pose targets
  #   plan = move_group.plan()
  #   if plan:
  #     rospy.loginfo("======== Motion planning successful! =======")
  #     # for i in target_poses:
  #     #   move_group.go(i)
  #   else:
  #     rospy.loginfo("========== Motion planning failed! =========")

  tutorial = robotic_arm_control()
  while not rospy.is_shutdown():
    start_time = time.time()
    cartesian_plan, fraction = tutorial.plan_cartesian_path()
    tutorial.display_trajectory(cartesian_plan)
    tutorial.execute_plan(cartesian_plan)
    print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == '__main__':
  main()
