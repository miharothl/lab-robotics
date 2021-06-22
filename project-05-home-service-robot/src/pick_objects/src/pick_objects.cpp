#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ros::NodeHandle n;
  ros::Publisher pub_goal_reached = n.advertise<std_msgs::UInt8>("/goalReached", 1);

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  //goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  n.getParam("/pick_up_location/tx", goal.target_pose.pose.position.x);
  n.getParam("/pick_up_location/ty", goal.target_pose.pose.position.y);
  n.getParam("/pick_up_location/qw", goal.target_pose.pose.orientation.w);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pick-up goal...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Pick-up location reached.");

    std_msgs::UInt8 goal_msg;
    goal_msg.data = 1;
    pub_goal_reached.publish(goal_msg);
  }
  else
    ROS_INFO("The robot failed reach the pick-up location.");

  ros::Duration(5).sleep(); // sleep for 5 seconds

  // Define a position and orientation for the robot to reach
  n.getParam("/drop_off_location/tx", goal.target_pose.pose.position.x);
  n.getParam("/drop_off_location/ty", goal.target_pose.pose.position.y);
  n.getParam("/drop_off_location/qw", goal.target_pose.pose.orientation.w);

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending drop-off goal...");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Drop-off location reached.");

    std_msgs::UInt8 goal_msg;
    goal_msg.data = 2;
    pub_goal_reached.publish(goal_msg);
  }
  else
    ROS_INFO("The robot failed to reach the drop-off goal.");

  ros::Duration(5).sleep(); // sleep for 5 seconds

  return 0;
}
