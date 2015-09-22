// No copyright - Volker

#include <stdio.h>
#include <boost/algorithm/string.hpp>

// Include standard libraries
#include <iostream>
#include <string>
#include <typeinfo>
#include <fstream>
#include <chrono>

// Include OpenCV libraries

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"

// Set namespaces

// Global variables
std::string query_window = "Query (cam) image";
std::string map_window = "Map image";
std::string matches_window = "Matches image";

const float inlier_threshold = 100.0f; // Distance threshold to identify inliers
const float match_ratio = 0.8f;   // Nearest neighbor matching ratio


using namespace std;
using namespace std::chrono;

















#include <math.h>
#include <cmath> 

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/Geocentric.hpp>

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"

const double pi = 3.14159265358979323846;
using namespace cv;
using namespace GeographicLib;

// Dimensions of the sheets on the floor in the cyberzoo (in meters)
Point2f real_img_dim(0.297 * 9, 0.420 * 9);

// Dimensions of image on the computer
Point2i pix_img_dim(2891, 4347);

// Rotation angle (difference to north)
float rotation_angle = 54.0;

cv::Point3d datum_lla(51.805628606594915, 4.376725217558763, 0);

Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());

// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * pi / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180 / pi);
}


// Convert latitude, longitude coordinates to meter offset
Point2d lla2m(cv::Point3d lla){

  cv::Point2d m;
  m.x = (lla.x - datum_lla.x) * Constants::WGS84_a() * 60.0 * 0.3048;
  m.y = (lla.y - datum_lla.y) * Constants::WGS84_a() * 60.0 * std::cos(deg2rad(lla.x)) * 0.3048;

  return m;
}

float lla2m_g(cv::Point3d lla){

  const Geodesic& geod = Geodesic::WGS84;
  double dist;
  
  geod.Inverse(lla.x, lla.y,
               datum_lla.x, datum_lla.y,
               dist);

  return dist;
}


// Convert meter offset to latitude, longitude coordinates
cv::Point3d m2lla(cv::Point2d m){

  cv::Point3d lla;

  lla.x = m.x / (Constants::WGS84_a() * 60.0 * 0.3048) + datum_lla.x;
  lla.y = m.y / (Constants::WGS84_a() * 60 * 0.3048 * std::cos(deg2rad(datum_lla.x))) + datum_lla.y;
                 lla.z = datum_lla.z;             

  return lla;
}


cv::Point2d rotate_point(const cv::Point2d& p, const float& angle) {

    cv::Point2d out_p;

    // Convert radians to degrees
    float angle_rad = deg2rad(angle);
    
    out_p.x = std::cos(angle_rad) * p.x - std::sin(angle_rad) * p.y;
    out_p.y = std::sin(angle_rad) * p.x + std::cos(angle_rad) * p.y;
    
  return out_p;
}


Point2d m2pix_diff(const cv::Point2d& diff) {
  
  cv::Point2d pix_diff;
  pix_diff.x = diff.x * (pix_img_dim.x / real_img_dim.x);
  pix_diff.y = diff.y * (pix_img_dim.y / real_img_dim.y);

  return pix_diff;  
}

/*
  Convert an offset in pixels to an offset in meters.
 */
Point2d pix2m_diff(const cv::Point2d& diff) {

  cv::Point2d m_diff;
  m_diff.x = diff.x * (real_img_dim.x / pix_img_dim.x);
  m_diff.y = diff.y * (real_img_dim.y / pix_img_dim.y);

  return m_diff;  
  
}

cv::Point3d ecef2lla(cv::Point3d ecef) {
    
  // Reverse calculation (ECEF - lla)
  double lat, lon, h;
  earth.Reverse(ecef.x, ecef.y, ecef.z,
                 lat, lon, h);

  cv::Point3d lla(lat, lon, h);
  
  return lla;  
}


cv::Point3d lla2ecef(cv::Point3d lla) {

  // Forward calculation (lla -> ECEF)
  double ecef_x, ecef_y, ecef_z;
  earth.Forward(lla.x, lla.y, 0,
                ecef_x, ecef_y, ecef_z);

  cv::Point3d ecef(ecef_x, ecef_y, ecef_z);
  
  return ecef;
  
}


/*
  Convert a pixel position to an ECEF position
 */
Point3d pixel_pos_to_ecef_pos(const cv::Point2d& pos) {

  // Difference in meters
  cv::Point2d m_diff = pix2m_diff(pos);

  std::cout << "m_diff" << m_diff << std::endl;

  // Rotated difference
  cv::Point2d m_diff_rotated = rotate_point(m_diff, rotation_angle); 

  std::cout << "m_diff_rotated" << m_diff_rotated << std::endl;
  
  // Latitude, longitude 
  cv::Point3d lla = m2lla(m_diff_rotated);

  std::cout << "lla" << lla << std::endl;
  
  // Ecef position
  cv::Point3d ecef = lla2ecef(lla);

  std::cout << "ecef" << ecef << std::endl;
  
  return ecef;
  
}

/*
  Convert an ECEF position to a pixel position
 */
Point2d ecef_pos_to_pixel_pos(const cv::Point3d& pos) {

  std::cout << "ECEF POS TO PIXEL POS" << std::endl;
  
  // ECEF -> Latitude, longitude, altitude (LLA)
  cv::Point3d lla = ecef2lla(pos);

  std::cout << "lla" << lla << std::endl;
  
  // LLA -> Meters (m)
  cv::Point2d m = lla2m(lla);

  std::cout << "m" << m << std::endl;
  
  // Rotate meters
  cv::Point2d m_rot = rotate_point(m, - rotation_angle);

  std::cout << "m_rot" << m_rot << std::endl;

  // Get distance in pixels
  cv::Point2d p = m2pix_diff(m_rot);

  std::cout << "p" << p << std::endl;
  
  return p;
  
}






void debug_vector(std::vector<int> v) {
  for (int i = 0; i < v.size(); i++) {
    std::cout << i << ": " << v[i];
  }
}


// Pixel coordinates to ECEF coordinates



int main(int argc, char* argv[]) {

  std::cout.precision(15);
  //cv::Point2f p(5, 5);
  //cv::Point2f p_2;

  // p_2 = rotate_point(p, 35);

  //  #std::cout << "x is: " << p_2.x << "y is: " << p_2.y;

  // Init matrices for cam images and the map image
  cv::Mat query_img, query_img_gray;
  cv::Mat map_img, map_img_gray;

  // Path of map  
  std::string map_img_path = "../data/newMaze17_full.jpg";

  // Path of drone information
  std::string drone_info_path =  "../data/data_preprocessed.csv";
  std::ifstream drone_info_file(drone_info_path.c_str());

  // Homograpy
  cv::Mat homography;

  map_img = cv::imread(map_img_path, CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(map_img, map_img_gray, CV_BGR2GRAY);

  // Check for invalid input
  if (!map_img.data) {
      std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

  // Webcam usage or recorded video

  std::string video_path = "../data/output.avi";
  //int video_path = 1;

  // Load camera
  cv::VideoCapture video(video_path);

  if (!video.isOpened()) {
    std::cerr << "Capture not opened!" << std::endl;
    return -1;
  }

  // Create a window for display.
  //cv::namedWindow(query_window, cv::WINDOW_AUTOSIZE);
  //cv::namedWindow(matches_window, cv::WINDOW_AUTOSIZE);

  //-- Step 1: Detect the keypoints using SURF Detector
  std::vector<cv::KeyPoint> keypoints_query, keypoints_map;

  cv::Mat descriptors_query, descriptors_map;

  int minHessian = 400;
  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);

  detector->detectAndCompute(map_img_gray, 
			     cv::noArray(), 
			     keypoints_map,
			     descriptors_map);


  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  
  // Read video frame by frame and do calculations
  while (true) {

    t1 = high_resolution_clock::now();
    std::cout << (duration / 1000) << "\n";

    // Read video, convert it to grayscale and display it 
    video.read(query_img);
    
    cv::cvtColor(query_img, query_img_gray, cv::COLOR_BGR2GRAY);
    //cv::imshow(query_window, query_img_gray);
    
    // Safely exit the recording on pressing any key
    if(cv::waitKey(1) >= 0) break;
   
   // Calculate keypoints
   
   detector->detectAndCompute(query_img_gray, 
			      cv::noArray(), 
			      keypoints_query,
			      descriptors_query);
   
   
   cv::FlannBasedMatcher matcher;
   
   std::vector< std::vector<cv::DMatch> > matches;
   
   matcher.knnMatch(descriptors_query, 
		    descriptors_map, 
		    matches,
		   2);
   
   std::vector<cv::KeyPoint> matched_query, matched_map, inliers_query, inliers_map;
   std::vector<cv::DMatch> good_matches;

   //-- Localize the object
   std::vector<cv::Point2f> pts_query;
   std::vector<cv::Point2f> pts_map;
   
  for(size_t i = 0; i < matches.size(); i++) {
    
    cv::DMatch first = matches[i][0];
    float dist_query = matches[i][0].distance;
    float dist_map = matches[i][1].distance;
    
    if(dist_query < match_ratio * dist_map) {
      matched_query.push_back(keypoints_query[first.queryIdx]);
      matched_map.push_back(keypoints_map[first.trainIdx]);

      pts_query.push_back(keypoints_query[first.queryIdx].pt);
      pts_map.push_back(keypoints_map[first.trainIdx].pt);
      
    }
  }

  cv::Mat mask; 

  homography = cv::findHomography(pts_query, 
				  pts_map,
				  cv::RANSAC,
				  5,
				  mask);


  // Input Quadilateral or Image plane coordinates
  std::vector<cv::Point2f> centers(1), centers_transformed(1);
  cv::Point2f center(query_img.rows / 2, query_img.cols / 2);
  cv::Point2f center_transformed(query_img.rows / 2, query_img.cols / 2);
  
  centers[0] = center;
  cv::perspectiveTransform(centers, centers_transformed, homography);

  center_transformed = centers_transformed[0];
  
  std::cout << center_transformed.x << " " << center_transformed.y << std::endl;

  cv::Point3d ecef = pixel_pos_to_ecef_pos(center_transformed);

  std::cout << ecef << std::endl;
  
  //debug_vector(centers);
  //debug_vector(centers_transformed);

  for(unsigned i = 0; i < matched_query.size(); i++) {

    cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
    col.at<double>(0) = matched_query[i].pt.x;
    col.at<double>(1) = matched_map[i].pt.y;
    
    col = homography * col;
    col /= col.at<double>(2);
    double dist = sqrt( pow(col.at<double>(0) - matched_map[i].pt.x, 2) +
			pow(col.at<double>(1) - matched_map[i].pt.y, 2));
    
    if(dist < inlier_threshold) {
      int new_i = static_cast<int>(inliers_query.size());
      inliers_query.push_back(matched_query[i]);
      inliers_map.push_back(matched_map[i]);
      good_matches.push_back(cv::DMatch(new_i, new_i, 0));
   }
  }
  
  cv::Mat res, res_small;
  //cv::drawMatches(query_img_gray, inliers_query, 
		  // map_img_gray, inliers_map, 
		  // good_matches, res);
  

  //cv::Size size(640, 480);//the dst image size,e.g.100x100
  //cv::resize(res, res_small, size);
  
  //cv::imshow(matches_window, res_small);
  
  double inlier_ratio = inliers_query.size() * 1.0 / matched_query.size();
  // std::cout << "Matching Results" << std::endl;
  // std::cout << "*******************************" << std::endl;
  // std::cout << "# Keypoints 1:                        \t" << keypoints_query.size() << std::endl;
  // std::cout << "# Keypoints 2:                        \t" << keypoints_map.size() << std::endl;
  // std::cout << "# Matches:                            \t" << matched_query.size() << std::endl;
  // std::cout << "# Inliers:                            \t" << inliers_query.size() << std::endl;
  // std::cout << "# Inliers Ratio:                      \t" << inlier_ratio << std::endl;
  // std::cout << std::endl;

  t2 = high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout << "Duration" << duration << std::endl;
  
}
  
  return 0; 
  
}
