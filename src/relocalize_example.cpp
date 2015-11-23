#include <iostream>
#include <string>
#include "relocalize.h"

using namespace std;

int main(int argc, char* argv[]) {

  // Construct relocalizer with reference image path
  Relocalizer relocalizer("/home/pold/Documents/draug/img/bestnewmat.png");

  // Read in query image
  cv::Mat query_img = cv::imread(argv[1]);

  // Get estimation (x, y) in pixels from relocalizer
  cv::Point2f estimation = relocalizer.calcLocation(query_img);

  // Print estimations
  cout << estimation.x << " " << estimation.y << endl;

  
}
