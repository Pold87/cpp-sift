/* Helper functions - Used in the classes made by Gerald
 *
 * Note: ft are always given with respect to datum, in NED frame
 *
 * Author:      Gerald van Dalen
 * Last edit:   01/21/2015
 */
#include "opencv2/core/core.hpp"
#include <utility>

namespace GHelper {
    // Convert latitude/longitude to ft w.r.t. some datum
    cv::Point2d latlon2ft(const double & datumLat, const double & datumLon, const double & lat, const double & lon);

    // Convert ft w.r.t. some datum to latitude/longitude
    cv::Point2d ft2latlon(const double & datumLat, const double & datumLon, const double & Xn, const double & Ye);

    // Calculate pixel location in current map based on known lat/lon
    cv::Point2d latlon2pix(const cv::Point2d & LatLon, const cv::Point2d & mapLatLon,
                           const double & mapScale, const int & mapSizeX, const int & mapSizeY);

    // Calculate lat/lon based on given pixel location in given map
    cv::Point2d pix2latlon(const cv::Point2d & pixelLoc, const cv::Point2d & mapLatLon,
                           const double & mapScale, const int &mapSizeX, const int & mapSizeY);

    // Calculate pixels from ft offset
    cv::Point2d ft2pix(const cv::Point2d & datumLatLon, const cv::Point2d & offsetFt, const cv::Point2d & mapLatLon,
                       const double & mapScale, const int & mapSizeX, const int & mapSizeY);

    // Calculate ft offset from pixels
    cv::Point2d pix2ft(const cv::Point2d & datumLatLon, const cv::Point2d & pixelLoc, const cv::Point2d & mapLatLon,
                       const double & mapScale, const int & mapSizeX, const int & mapSizeY);


    // Calculate norm of a given openCV point
    double pointNorm(const cv::Point2d & p);

    // Calculate absolute value of given openCV point
    cv::Point2d pointAbs(const cv::Point2d & p);

    // Check whether a file exists
    bool fileExists(const std::string& filename);


    // Comperator function between two std:pair's. To be used by std:sort (used by GParticleFilter::updatePosEst_meanwalk() )
    bool sortComparePair(const std::pair<double,int> & p1, const std::pair<double,int> & p2);

    // Function handle to make sure that opencv does not print message when exception is caught
    // usage: before try-block (or begin of code) put: cv::redirectError(opencvSuppressError)
    // At end of code, use: cv::redirectError(NULL) to disable again
    int opencvSuppressError(int status, const char* func_name, const char* err_msg,
                            const char* file_name, int line, void* userdata);
}
