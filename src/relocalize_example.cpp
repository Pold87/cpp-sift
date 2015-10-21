#include <iostream>
#include <string>
#include "relocalize.h"

using namespace std;

int main() {

  // Construct relocalizer with reference image path
  Relocalizer relocalizer("res.png");

  // Read in query image
  cv::Mat query_img = cv::imread("res.png");

  // Get estimation (x, y) in pixels from relocalizer
  cv::Point2f estimation = relocalizer.calcLocation(query_img);

  // Print estimations
  cout << estimation.x << " " << estimation.y << endl;

  
}
