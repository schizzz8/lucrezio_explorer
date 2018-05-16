#pragma once
#include <queue>
#include <srrg_types/types.hpp>

enum Occupancy {
  FREE = 254, UNKNOWN = 205, OCCUPIED = 0
};

typedef std::vector<srrg_core::Vector2iVector, Eigen::aligned_allocator<srrg_core::Vector2iVector> > RegionVector;
typedef std::list<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iList;

struct ScoredCell {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool operator<(const ScoredCell& sc) const {
      return score < sc.score;
    }

    float score;
    Eigen::Vector2i cell;
};
typedef std::priority_queue<ScoredCell> ScoredCellQueue;

class FrontierDetector{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    struct Configuration{
        int min_neighbors_threshold = 1;
        int size_threshold = 10;
        int obstacle_radius = 10;
        float w1=1.0;
        float w2=1.0;
        float w3=1.0;
        float centroid_minimum_score = 1;
    };

    FrontierDetector();

    inline Configuration& mutableConfig(){return _config;}

    inline void setRobotPose(const Eigen::Isometry3f &robot_pose_){_robot_pose = robot_pose_;}
    inline void setResolution(float resolution_){_resolution = _resolution;}
    inline void setOrigin(const Eigen::Vector2f &origin_){_origin = origin_;}
    inline void setMap(const srrg_core::UnsignedCharImage &occupancy_grid_){occupancy_grid_.copyTo(_occupancy_grid);
                                                                            _size << occupancy_grid_.rows,occupancy_grid_.cols;}


    void init();

    void computeFrontierPoints();
    void computeFrontierRegions();
    void computeFrontierCentroids();
    void rankFrontierCentroids();

    inline const srrg_core::Vector2iVector &frontierPoints() const {return _frontier_points;}
    inline const RegionVector &frontierRegions() const {return _frontier_regions;}
    inline const srrg_core::Vector2iVector &frontierCentroids() const {return _frontier_centroids;}
    inline ScoredCellQueue &frontierScoredCentroids() {return _frontier_scored_centroids;}

  protected:

    Configuration _config;

    float _resolution;
    Eigen::Vector2i _size;
    Eigen::Vector2f _origin;

    Eigen::Isometry3f _robot_pose;
    srrg_core::UnsignedCharImage _occupancy_grid;

    srrg_core::Vector2iVector _frontier_points;
    RegionVector _frontier_regions;
    srrg_core::Vector2iVector _frontier_centroids;
    ScoredCellQueue _frontier_scored_centroids;

  private:
    void getColoredNeighbors(srrg_core::Vector2iVector &neighbors, const Eigen::Vector2i &cell, const Occupancy &value);
    void recurRegion(const Vector2iList::iterator &frontier_it, srrg_core::Vector2iVector &region, Vector2iList &frontiers);
    inline float angleDifference(float a1, float a2) {
      float diff = std::fmod(a2 - a1 + M_PI,2*M_PI) - M_PI;
      return diff < -M_PI ? diff + 2*M_PI : diff;
    }
};
