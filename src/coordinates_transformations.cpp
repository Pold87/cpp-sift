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


int main() {

  Point2i p(100, 100);
  Point3d ecef = pixel_pos_to_ecef_pos(p);

  std::cout << ecef << "\n";

  Point2i p2 = ecef_pos_to_pixel_pos(ecef);
  
  std::cout << p2 << "\n";
  
}
