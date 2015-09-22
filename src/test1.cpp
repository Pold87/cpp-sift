// Small example of using the GeographicLib::Geodesic class
#include <iostream>
#include <cmath>
#include <GeographicLib/Geocentric.hpp>
using namespace std;
using namespace GeographicLib;

int main() {
  //Geocentric earth(Constants::WGS84_a(), Constants::WGS84_f());
    Geocentric earth();
    
      // Sample forward calculation (lat_lon -> ECEF)
      double lat = 27.99, lon = 86.93, h = 8820; // Mt Everest
      double ecef_x, ecef_y, ecef_z;
      earth.Forward(lat, lon, h, ecef_x, ecef_y, ecef_z);
      
  return 0;
}
