#include "frontier_detector.h"

using namespace srrg_core;

FrontierDetector::FrontierDetector(){
  _resolution = 0.0f;
  _origin.setZero();
}

void FrontierDetector::init(){
  assert(_resolution != 0 && "[FrontierDetector][init]: Zero grid resolution!");

  _frontier_points.clear();
  _frontier_regions.clear();
  _frontier_centroids.clear();
}

void FrontierDetector::computeFrontierPoints(){

  Eigen::Vector2i cell;
  Vector2iVector neighbors;
  Vector2iVector neighbors_of_neighbor;

  for(int r=0; r<_rows; ++r){
    const unsigned char *occupancy_ptr = _occupancy_grid.ptr<unsigned char>(r);
    for(int c=0; c<_cols; ++c, ++occupancy_ptr){
      const unsigned char &occupancy = *occupancy_ptr;

      if(occupancy != Occupancy::FREE)
        continue;

//      cell.x() = r;
//      cell.y() = c;

      cell.x() = c;
      cell.y() = r;

      getColoredNeighbors(neighbors,cell,Occupancy::UNKNOWN);

      if(neighbors.empty())
        continue;

      for(const Eigen::Vector2i &neighbor : neighbors){
        getColoredNeighbors(neighbors_of_neighbor, neighbor, Occupancy::UNKNOWN);

        if(neighbors_of_neighbor.size() >= _config.min_neighbors_threshold){
          _frontier_points.push_back(cell);
          break;
        }
      }

    }
  }
}

void FrontierDetector::computeFrontierRegions(){
  Vector2iList frontiers(_frontier_points.begin(), _frontier_points.end());

  Vector2iList::iterator it;
  for (it = frontiers.begin(); it != frontiers.end(); ++it) {
    Vector2iVector region;
    recurRegion(it, region, frontiers);
    if (region.size() >= _config.size_threshold) {
      _frontier_regions.push_back(region);
    }
    it = frontiers.begin();
  }
}

void FrontierDetector::computeFrontierCentroids(){
  for (int i = 0; i < _frontier_regions.size(); ++i) {
    Eigen::Vector2i centroid;
    centroid.setZero();

    for (int j = 0; j <_frontier_regions[i].size(); ++j)
      centroid += _frontier_regions[i][j];

    centroid /= (float)_frontier_regions[i].size();

    _frontier_centroids.push_back(centroid);
  }
}

void FrontierDetector::rankFrontierCentroids(){

  Eigen::Vector2i robot_position = ((_robot_pose.translation().head(2)-_origin)/_resolution).cast<int>();
  float robot_orientation = _robot_pose.linear().eulerAngles(0,1,2).z();

  for(size_t i=0; i<_frontier_centroids.size(); ++i){

    const Eigen::Vector2i diff = _frontier_centroids[i] - robot_position;

    //distance cost
    float distance_cost = diff.norm();
    if (distance_cost < 10)
      distance_cost = 0.1f;

    //ahead cost
//    float centroid_angle = std::atan2(diff.x(),diff.y());
    float centroid_angle = std::atan2(diff.y(),diff.x());
    const float ahead_cost = std::cos(angleDifference(robot_orientation,centroid_angle));

    //obstacle distance cost
    float obstacle_distance_cost = 1.0;
    float min_dist = _config.obstacle_radius;
    for (int r = -_config.obstacle_radius; r <= _config.obstacle_radius; ++r) {
      for (int c = -_config.obstacle_radius; c <= _config.obstacle_radius; ++c) {

        if (r == 0 && c == 0)
          continue;

//        int rr = _frontier_centroids[i].x()+r;
//        int cc = _frontier_centroids[i].y()+c;
        int rr = _frontier_centroids[i].y()+r;
        int cc = _frontier_centroids[i].x()+c;

        if ( rr < 0 || rr >= _rows || cc < 0 || cc >= _cols)
          continue;


        if (_occupancy_grid.at<unsigned char>(rr,cc) == Occupancy::OCCUPIED) {
//          float distance = (_frontier_centroids[i] - Eigen::Vector2i(rr,cc)).norm();
          float distance = (_frontier_centroids[i] - Eigen::Vector2i(cc,rr)).norm();

          if (distance < min_dist)
            min_dist = distance;

        }
      }
    }

    if (min_dist != _config.obstacle_radius && min_dist > 0) {
      const float scale_factor = 0.3;
      obstacle_distance_cost = std::exp(- scale_factor * min_dist);
    }

    ScoredCell scored_centroid;
    scored_centroid.cell = _frontier_centroids[i];
    scored_centroid.score = _config.w1*distance_cost + _config.w2*ahead_cost + _config.w3*obstacle_distance_cost;

    if(scored_centroid.score >= _config.centroid_minimum_score)
      _frontier_scored_centroids.push(scored_centroid);

  }
}

void FrontierDetector::getColoredNeighbors(Vector2iVector &neighbors,
                                           const Eigen::Vector2i &cell,
                                           const Occupancy &value){

  neighbors.clear();

  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {

      if (r == 0 && c == 0)
        continue;

//      int rr = cell.x()+r;
//      int cc = cell.y()+c;

      int rr = cell.y()+r;
      int cc = cell.x()+c;

      if ( rr < 0 || rr >= _rows ||
           cc < 0 || cc >= _cols)
        continue;


      if (_occupancy_grid.at<unsigned char>(rr,cc) == value)
//        neighbors.push_back(Eigen::Vector2i(rr, cc));
        neighbors.push_back(Eigen::Vector2i(cc,rr));

    }
  }
}

void FrontierDetector::recurRegion(const Vector2iList::iterator& frontier_it, Vector2iVector& region, Vector2iList& frontiers) {
  Eigen::Vector2i frontier = *frontier_it;
  region.push_back(frontier);
  frontiers.erase(frontier_it);

  for (int r = -1; r <= 1; ++r) {
    for (int c = -1; c <= 1; ++c) {

      if (r == 0 && c == 0)
        continue;

//      int rr = frontier.x()+r;
//      int cc = frontier.y()+c;

      int rr = frontier.y()+r;
      int cc = frontier.x()+c;

      if (rr < 0 || rr >= _rows || cc < 0 || cc >= _cols)
        continue;


//      Eigen::Vector2i n(rr, cc);
      Eigen::Vector2i n(cc,rr);
      Vector2iList::iterator it = std::find(frontiers.begin(), frontiers.end(), n);
      if (it != frontiers.end())
        recurRegion(it, region, frontiers);

    }
  }
}
