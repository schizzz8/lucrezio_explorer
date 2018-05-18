#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <frontier_detector/frontier_detector.h>

class LucrezioExplorer{

  public:

//    move_base_msgs::MoveBaseGoal computeNextPose();
    Eigen::Vector2f computeNextPose();
    void showNextPose();

  protected:

    tf::TransformListener _listener;
    FrontierDetector _detector;

    ScoredCell _next_pose;

  private:

    bool listenRobotPose(Eigen::Isometry3f &robot_pose);
    bool receiveOccupancyGridMsg(const std::string &map_topic,
                                 float duration,
                                 float &resolution,
                                 Eigen::Vector2f &origin,
                                 srrg_core::UnsignedCharImage &occupancy_grid);
};
