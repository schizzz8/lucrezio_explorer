#include <iostream>

#include <lucrezio_explorer/lucrezio_explorer.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv){

  ros::init(argc, argv, "lucrezio_explorer_node");

  LucrezioExplorer explorer;
  MoveBaseClient ac("move_base",true);

  //start execution loop
  ros::Rate loop_rate(1);
  while (ros::ok()){

    ROS_INFO("Waiting for move_base action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");
    move_base_msgs::MoveBaseGoal goal = explorer.computeNextPose();
    ac.sendGoal(goal);

    //visualize next pose
    explorer.showNextPose();

    //wait for the action server to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));
    if (finished_before_timeout){
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
