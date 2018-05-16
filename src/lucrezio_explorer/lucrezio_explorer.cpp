#include "lucrezio_explorer.h"

using namespace move_base_msgs;

MoveBaseGoal LucrezioExplorer::computeNextPose(){

  //listen to robot pose
  Eigen::Isometry3f robot_pose;
  if(listenRobotPose(robot_pose))
    _detector.setRobotPose(robot_pose);

  //receive current map
  float resolution;
  Eigen::Vector2f origin;
  srrg_core::UnsignedCharImage occupancy_grid;
  if(receiveOccupancyGridMsg("/map",
                             1,
                             resolution,
                             origin,
                             occupancy_grid)){
    _detector.setResolution(resolution);
    _detector.setOrigin(origin);
    _detector.setMap(occupancy_grid);
  }

  //compute goal
  _detector.init();

  _detector.computeFrontierPoints();
  _detector.computeFrontierRegions();
  _detector.computeFrontierCentroids();

  //return goal message
  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.frame_id = "/map";
  goal_msg.target_pose.header.stamp = ros::Time::now();

  Eigen::Vector2i centroid;
  goal_msg.target_pose.pose.position.x = centroid.y()*resolution + origin.x();
  goal_msg.target_pose.pose.position.y = centroid.x()*resolution + origin.y();
  goal_msg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  return goal_msg;
}

bool LucrezioExplorer::listenRobotPose(Eigen::Isometry3f &robot_pose){
  tf::StampedTransform robot_tf;
  try {
    _listener.waitForTransform("map",
                               "base_footprint",
                               ros::Time(0),
                               ros::Duration(3));
    _listener.lookupTransform("map",
                              "base_footprint",
                              ros::Time(0),
                              robot_tf);
  }
  catch(tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }

  robot_pose.setIdentity();
  robot_pose.translation().x()=robot_tf.getOrigin().x();
  robot_pose.translation().y()=robot_tf.getOrigin().y();
  robot_pose.translation().z()=robot_tf.getOrigin().z();
  Eigen::Quaternionf q;
  tf::Quaternion tq = robot_tf.getRotation();
  q.x()= tq.x();
  q.y()= tq.y();
  q.z()= tq.z();
  q.w()= tq.w();
  robot_pose.linear()=q.toRotationMatrix();

  return true;
}

bool LucrezioExplorer::receiveOccupancyGridMsg(const std::string &map_topic,
                                               float duration,
                                               float &resolution,
                                               Eigen::Vector2f &origin,
                                               srrg_core::UnsignedCharImage &occupancy_grid){

  boost::shared_ptr<nav_msgs::OccupancyGrid const> occupancy_grid_msg_ptr;
  occupancy_grid_msg_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid> (map_topic, ros::Duration (duration));

  if(occupancy_grid_msg_ptr == NULL){
    ROS_ERROR("No occupancy_grid message received!");
    return false;
  } else {

    //map received
    resolution = occupancy_grid_msg_ptr->info.resolution;
    origin << occupancy_grid_msg_ptr->info.origin.position.x,occupancy_grid_msg_ptr->info.origin.position.y;

    //convert to cv::Mat
    int width = occupancy_grid_msg_ptr->info.width;
    int height = occupancy_grid_msg_ptr->info.height;
    occupancy_grid.create(height,width);
    for (int i = 0, i_rev = height - 1; i < height; i++, i_rev--)
      for (int j = 0; j < width; j++)
        switch (occupancy_grid_msg_ptr->data[i_rev*width + j]) {
          default:
          case -1:
            occupancy_grid.data[i*width + j] = Occupancy::UNKNOWN;
            break;
          case 0:
            occupancy_grid.data[i*width + j] = Occupancy::OCCUPIED;
            break;
          case 100:
            occupancy_grid.data[i*width + j] = Occupancy::FREE;
            break;
        }

    return true;
  }
}
