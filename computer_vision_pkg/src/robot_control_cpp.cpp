#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

class CArmInterface
{
public:
  CArmInterface() : movegroup("robotic_arm") {}

  moveit::core::RobotState get_last_planned_robot_state()
  {
    robot_trajectory::RobotTrajectory trajectory = new_trajectory();
    trajectory.setRobotTrajectoryMsg(movegroup.getCurrentState(), single_move_plan.trajectory_);
    moveit::core::RobotState last_robot_state = trajectory.getLastWayPoint();
    return last_robot_state;
  }

  geometry_msgs::Pose get_robot_state_pose(const moveit::core::RobotState &state)
  {
    Eigen::Affine3d eef_transform = state.getGlobalLinkTransform(movegroup.getEndEffectorLink());
    geometry_msgs::Pose pose;
    tf2::convert(eef_transform, pose);
    return pose;
  }

  robot_trajectory::RobotTrajectory new_trajectory()
  {
    robot_trajectory::RobotTrajectory trajectory(movegroup.getCurrentState()->getRobotModel(), movegroup.getName());
    return trajectory;
  }

  // ... Other methods ...

private:
  moveit::planning_interface::MoveGroupInterface movegroup;
  moveit::planning_interface::MoveGroupInterface::Plan single_move_plan;
  moveit::planning_interface::MoveGroupInterface::Plan multi_move_plan;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_planning_and_execution");
  ros::NodeHandle nh;

  // Initialize MoveIt
  moveit::planning_interface::MoveGroupInterface::Options options("robotic_arm", "robot_description", nh);
  moveit::planning_interface::MoveGroupInterface move_group(options);

  // Create the CArmInterface object
  CArmInterface arm_interface;

  // Use arm_interface methods here to interact with the robot and trajectories

  return 0;
}
