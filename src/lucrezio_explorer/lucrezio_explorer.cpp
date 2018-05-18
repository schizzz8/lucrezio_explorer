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
  _detector.rankFrontierCentroids();
  _next_pose = _detector.frontierScoredCentroids().top();

  //return goal message
  move_base_msgs::MoveBaseGoal goal_msg;
  goal_msg.target_pose.header.frame_id = "/map";
  goal_msg.target_pose.header.stamp = ros::Time::now();

  goal_msg.target_pose.pose.position.x = _next_pose.cell.y()*resolution + origin.x();
  goal_msg.target_pose.pose.position.y = _next_pose.cell.x()*resolution + origin.y();
  goal_msg.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  return goal_msg;
}

void LucrezioExplorer::showNextPose(){

  //map
  srrg_core::RGBImage occupancy_rgb;
  cv::cvtColor(_detector.occupancyGrid(),occupancy_rgb,CV_GRAY2BGR);

  const float &resolution = _detector.resolution();
  const Eigen::Vector2f &grid_origin = _detector.origin();

  //map origin
  cv::Point2i origin(grid_origin.y(),grid_origin.x());
  cv::circle(occupancy_rgb,origin,4,cv::Scalar(0,255,0),2);

  //robot position
  Eigen::Vector2i robot_position = ((_detector.robotPose().translation().head(2)-grid_origin)/resolution).cast<int>();
  cv::Point2i robot(robot_position.y(),robot_position.x());
  cv::circle(occupancy_rgb,robot,4,cv::Scalar(0,0,255),2);

  //next pose
  cv::Point2i next_pose(_next_pose.cell.y(),_next_pose.cell.x());
  cv::circle(occupancy_rgb,next_pose,4,cv::Scalar(255,0,0),2);
  float score = _next_pose.score;
  std::ostringstream ss;
  ss << score;
  cv::putText(occupancy_rgb, ss.str(), next_pose, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255));

  cv::imshow("explorer",occupancy_rgb);
  cv::waitKey(10);

  std::cerr << std::endl;
  ROS_INFO("Show next pose!");
  std::cerr << std::endl;
  std::cerr << "Grid" << std::endl;
  std::cerr << "Resolution: " << resolution << std::endl;
  std::cerr << "Dimesions: " << occupancy_rgb.rows << "x" << occupancy_rgb.cols << std::endl;
  std::cerr << "Origin: " << origin << std::endl;
  std::cerr << std::endl;
  std::cerr << "Robot" << std::endl;
  std::cerr << "Position: " << robot << std::endl;

}

bool LucrezioExplorer::listenRobotPose(Eigen::Isometry3f &robot_pose){
  tf::StampedTransform robot_tf;
  try {
    _listener.waitForTransform("map",
                               "base_link",
                               ros::Time(0),
                               ros::Duration(3));
    _listener.lookupTransform("map",
                              "base_link",
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

  Eigen::Vector2f robot_position = robot_pose.translation().head(2);
  float robot_orientation = robot_pose.linear().eulerAngles(0,1,2).z();

  std::cerr << std::endl;
  ROS_INFO("Received robot pose!");
  std::cerr << "Position: " << robot_position.transpose() << std::endl;
  std::cerr << "Orientation: " << robot_orientation << std::endl;
  std::cerr << std::endl;

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
            occupancy_grid.data[i*width + j] = Occupancy::FREE;
            break;
          case 100:
            occupancy_grid.data[i*width + j] = Occupancy::OCCUPIED;
            break;
        }

    std::cerr << std::endl;
    ROS_INFO("Received occupancy grid!");
    std::cerr << "Dimension: " << height << "x" << width << std::endl;
    std::cerr << "Resolution: " << resolution << std::endl;
    std::cerr << "Origin: " << origin.transpose() << std::endl;
    std::cerr << std::endl;

    return true;
  }
}
