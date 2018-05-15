#include <iostream>

#include <frontier_detector/frontier_detector.h>

using namespace std;
using namespace srrg_core;

void drawFrontierPoints(RGBImage &image, const Vector2iVector &points);
void drawFrontierRegions(RGBImage &image, const RegionVector &regions);
void drawFrontierCentroids(RGBImage &image, const Vector2iVector &centroids);

int main(int argc, char **argv){

  cv::Mat occupancy_grid;
  occupancy_grid = cv::imread(argv[1],CV_LOAD_IMAGE_UNCHANGED);

  cv::imshow("input",occupancy_grid);

  RGBImage occupancy_rgb;
  cv::cvtColor(occupancy_grid,occupancy_rgb,CV_GRAY2BGR);

  float resolution = 0.05f;

  FrontierDetector detector;
  detector.setResolution(resolution);
  detector.setMap(occupancy_grid);

  detector.init();

  //compute frontier points
  detector.computeFrontierPoints();
  const Vector2iVector &points = detector.frontierPoints();
  std::cerr << "Detected " << points.size() << " frontiers" << std::endl;

  RGBImage points_image;
  occupancy_rgb.copyTo(points_image);
  drawFrontierPoints(points_image,points);
  cv::imshow("Frontier points",points_image);

  //compute frontier regions
  detector.computeFrontierRegions();
  const RegionVector &regions = detector.frontierRegions();
  std::cerr << "Frontier regions: " << regions.size() << std::endl;

  RGBImage regions_image;
  occupancy_rgb.copyTo(regions_image);
  drawFrontierRegions(regions_image,regions);
  cv::imshow("Frontier regions",regions_image);

  //compute frontier regions
  detector.computeFrontierCentroids();
  const Vector2iVector &centroids = detector.frontierCentroids();

  RGBImage centroids_image;
  occupancy_rgb.copyTo(centroids_image);
  drawFrontierCentroids(centroids_image,centroids);
  cv::imshow("Frontier centroids",centroids_image);

  cv::waitKey();

  return 0;
}

void drawFrontierPoints(RGBImage &image, const Vector2iVector &points){
  for(const Eigen::Vector2i &point : points){
    cv::Point2i cell(point.y(),point.x());
    cv::circle(image,cell,1,cv::Scalar(0,0,255));
  }
}

void drawFrontierRegions(RGBImage &image, const RegionVector &regions){
  cv::RNG rng(12345);
  for(const Vector2iVector region : regions){
    cv::Scalar color(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
    for(const Eigen::Vector2i &point : region){
      cv::Point2i cell(point.y(),point.x());
      cv::circle(image,cell,1,color);
    }
  }
}

void drawFrontierCentroids(RGBImage &image, const Vector2iVector &centroids){
  for(const Eigen::Vector2i &centroid : centroids){
    cv::Point2i cell(centroid.y(),centroid.x());
    cv::circle(image,cell,4,cv::Scalar(255,0,0),2);
  }
}
