#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a convenient typedef for the Action Client
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the ROS node
  ros::init(argc, argv, "husky_navigation_client");

  // Tell the action client to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait for the move_base action server to come online
  ROS_INFO("Waiting for the move_base action server to start...");
  ac.waitForServer();
  ROS_INFO("Connected to move_base server!");

  // Create the goal object
  move_base_msgs::MoveBaseGoal goal;

  // We are sending a goal relative to the global "map" frame
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // ---------------------------------------------------
  // TARGET COORDINATES (You will change these later!)
  // For now, let's drive to x = 0.0, y = 0.0 in the living room
  goal.target_pose.pose.position.x = 5.0;
  goal.target_pose.pose.position.y = 4.0;

  // Orientation is a quaternion. w=1.0 means "facing straight forward"
  goal.target_pose.pose.orientation.w = 1.0;
  // ---------------------------------------------------

  ROS_INFO("Sending target destination to Husky...");
  ac.sendGoal(goal);

  // Wait for the robot to reach the destination
  ac.waitForResult();

  // Check if it was successful
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray! The Husky successfully reached the target.");
  else
    ROS_INFO("The Husky failed to reach the target.");

  return 0;
}