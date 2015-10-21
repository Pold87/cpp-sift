// No copyright - Volker

#include <stdio.h>

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

// Math libraries
#include <math.h>
#include <cmath>

#include "relocalize.h"

using namespace std;
using namespace cv;
const double pi = 3.14159265358979323846;

const float inlier_threshold = 100.0f; // Distance threshold to identify inliers
const float match_ratio = 0.8f;   // Nearest neighbor matching ratio

// Dimensions of the sheets on the floor in the cyberzoo (in meters)
Point2f real_img_dim(0.297 * 9, 0.420 * 9);

// Dimensions of image on the computer
Point2i pix_img_dim(2891, 4347);

// Rotation angle (difference to north)
float rotation_angle = 54.0;

cv::Point3d datum_lla(51.805628606594915, 4.376725217558763, 0);

// This function converts decimal degrees to radians
double deg2rad(double deg) {
  return (deg * pi / 180);
}

//  This function converts radians to decimal degrees
double rad2deg(double rad) {
  return (rad * 180 / pi);
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



Relocalizer::Relocalizer(std::string path) {

  ref_img_path = path;
  ref_img_c = cv::imread(ref_img_path);

  cv::cvtColor(ref_img_c,
               ref_img,
               cv::COLOR_BGR2GRAY);

  

  int minHessian = 400;
  detector = cv::xfeatures2d::SURF::create(minHessian);


  // Detect and compute keypoints of the reference image
  detector->detectAndCompute(ref_img, 
                             cv::noArray(), 
                             kp_ref,
                             des_ref);
}


cv::Point2f Relocalizer::calcLocation(cv::Mat query_img) {

    std::vector<cv::KeyPoint> kp_query; // Keypoints of the query image
    cv::Mat des_query;
    cv::Mat query_img_gray;
    
    cv::cvtColor(query_img,
                 query_img_gray,
                 cv::COLOR_BGR2GRAY);


    detector->detectAndCompute(query_img_gray, 
                               cv::noArray(), 
                               kp_query,
                               des_query);


    std::vector< std::vector<cv::DMatch> > matches;
   
    matcher.knnMatch(des_ref, 
                     des_query, 
                     matches,
                     2);


    std::vector<cv::KeyPoint> matched_query, matched_ref, inliers_query, inliers_ref;
    std::vector<cv::DMatch> good_matches;

   //-- Localize the object
    std::vector<cv::Point2f> pts_query;
    std::vector<cv::Point2f> pts_ref;
    
  for(size_t i = 0; i < matches.size(); i++) {
    
    cv::DMatch first = matches[i][0];
    float dist_query = matches[i][0].distance;
    float dist_ref = matches[i][1].distance;
    
    if (dist_query < match_ratio * dist_ref) {
      
      matched_query.push_back(kp_query[first.queryIdx]);
      matched_ref.push_back(kp_ref[first.trainIdx]);
      
      pts_query.push_back(kp_query[first.queryIdx].pt);
      pts_ref.push_back(kp_ref[first.trainIdx].pt);
      
    }
  }


  cv::Mat mask; 

  // Homograpy
  cv::Mat homography;
  
  homography = cv::findHomography(pts_query, 
				  pts_ref,
				  cv::RANSAC,
				  5,
				  mask);


  // Input Quadilateral or Image plane coordinates
  std::vector<cv::Point2f> centers(1), centers_transformed(1);

  cv::Point2f center(query_img_gray.rows / 2,
                     query_img_gray.cols / 2);
  
  cv::Point2f center_transformed(query_img.rows / 2,
                                 query_img.cols / 2);
  
  centers[0] = center; // Workaround for using perspective transform
  
  cv::perspectiveTransform(centers,
                           centers_transformed,
                           homography);

  center_transformed = centers_transformed[0];

  return center_transformed;
  
  }