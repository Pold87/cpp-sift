/* Helper functions - Used in the classes made by Gerald
 *
 * Note: ft are always given with respect to datum, in NED frame
 *
 * Author:      Gerald van Dalen
 * Last edit:   01/21/2015
 */
#include "rmax/imageMatchPF/GHelper.h"

#include <cstdio>
#include "esim/util.h"

using namespace cv;

// Convert latitude/longitude to ft w.r.t. some datum
Point2d GHelper::latlon2ft(const double & datumLat, const double & datumLon, const double & lat, const double & lon)
{
    // Calculate distance in Northern direction
    double Xn = (lat - datumLat)*C_NM2FT*60.0;
    
    // Calculate distance in Eastern direction
    double Ye = hmodDeg(lon - datumLon)*C_NM2FT*60.0*cos(datumLat*C_DEG2RAD);
    
    return Point2d(Xn,Ye);
}


// Convert ft w.r.t. some datum to latitude/longitude
Point2d GHelper::ft2latlon(const double & datumLat, const double & datumLon, const double & Xn, const double & Ye)
{
    // Calculate latitute
    double lat = Xn/(C_NM2FT*60.0) + datumLat;
    
    // Calculate longitude
    double lon = Ye/(C_NM2FT*60.0*cos(datumLat*C_DEG2RAD)) + datumLon;
    
    return Point2d(lat, lon);
}

// Calculate pixel location in current map based on known lat/lon
Point2d GHelper::latlon2pix(const Point2d & LatLon, const Point2d & mapLatLon, const double & mapScale, const int & mapSizeX, const int & mapSizeY)
{
    // Calculate offset in Ft from datum
    Point2d XnYe = GHelper::latlon2ft(mapLatLon.x, mapLatLon.y, LatLon.x, LatLon.y);

    // Convert ft to pixels using given scale of map (in [pix/ft])
    double xPix = XnYe.y * mapScale;
    double yPix = -XnYe.x * mapScale;
    
    // Return with half map offset to get pixels w.r.t upper left corner instead of center of map
    return Point2d(xPix+mapSizeX/2, yPix+mapSizeY/2);
}


// Calculate lat/lon based on given pixel location in given map
Point2d GHelper::pix2latlon(const Point2d & pixelLoc, const Point2d & mapLatLon, const double & mapScale, const int & mapSizeX, const int & mapSizeY)
{
    // Compute xPix and yPix relative to centre instead of upper left corner
    double xPix = pixelLoc.x - mapSizeX/2;
    double yPix = pixelLoc.y - mapSizeY/2;
    
    // Calculate number of feet from upper left corner map
    double Xn = -yPix/mapScale;
    double Ye = xPix/mapScale;
    
    // Compute latitute and longitude
    return GHelper::ft2latlon(mapLatLon.x, mapLatLon.y, Xn, Ye);
}

// Calculate pixels from ft offset
Point2d GHelper::ft2pix(const Point2d & datumLatLon, const Point2d & offsetFt, const Point2d & mapLatLon, const double & mapScale, const int & mapSizeX, const int & mapSizeY) {
    Point2d latlon = GHelper::ft2latlon(datumLatLon.x, datumLatLon.y, offsetFt.x, offsetFt.y);
    return GHelper::latlon2pix(latlon, mapLatLon, mapScale, mapSizeX, mapSizeY);
}

// Calculate ft offset from pixels
Point2d GHelper::pix2ft(const Point2d & datumLatLon, const Point2d & pixelLoc, const Point2d & mapLatLon, const double & mapScale, const int & mapSizeX, const int & mapSizeY) {
    Point2d latlon = GHelper::pix2latlon(pixelLoc, mapLatLon, mapScale, mapSizeX, mapSizeY);
    return GHelper::latlon2ft(datumLatLon.x, datumLatLon.y, latlon.x, latlon.y);
}



// Calculate norm of a given point
double GHelper::pointNorm(const Point2d& p)
{
    return sqrt(p.x*p.x + p.y*p.y);
}

// Calculate absolute value of given openCV point
Point2d GHelper::pointAbs(const Point2d &p)
{
    return Point2d(sqrt(p.x*p.x), sqrt(p.y*p.y));
}


// Check whether a file exists
bool GHelper::fileExists(const string& filename)
{
    if (FILE *fh = fopen(filename.c_str(), "r")) {
        fclose(fh);
        return true;
    } else {
        return false;
    }
}


// Comperator function between two std:pair's. To be used by std:sort
bool GHelper::sortComparePair(const std::pair< double, int >& p1, const std::pair< double, int >& p2)
{
    // Compare based on first element of each pair
    return p1.first < p2.first;
}

// Function handle to make sure that opencv does not print message when exception is caught
// usage: before try-block (or begin of code) put: cv::redirectError(opencvSuppressError)
// At end of code, use: cv::redirectError(NULL) to disable again
int GHelper::opencvSuppressError(int status, const char* func_name, const char* err_msg, const char* file_name, int line, void* userdata) {
    return 0;
}




