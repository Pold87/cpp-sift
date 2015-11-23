#include <iostream>
#include <string>
#include "relocalize.h"
#include <ctime>

#include "opencv2/opencv_modules.hpp"

#ifdef HAVE_OPENCV_XFEATURES2D

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d/cuda.hpp"
#include "opencv2/cudaarithm.hpp"


using namespace std;

int main() {

  // Construct relocalizer with reference image path
  Relocalizer relocalizer("out.png", 5, 3);

  // Read in query image
  cv::cuda::GpuMat q;
  cv::Mat query_img;

  query_img = cv::imread("cut.png", cv::IMREAD_GRAYSCALE);

  q.upload(query_img);


  for (int i = 0; i < 100; i = i + 20){

   clock_t begin = clock();
   // Get estimation (x, y) in pixels from relocalizer
   double elapsed_time_matching = relocalizer.calcLocation_tests(q, i);
   clock_t end = clock();

   double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;


   cout << "Time matching: " << int(1000 * elapsed_time_matching) << " ms" << std::endl;
   cout << "Keypoints: " << (100 - i) << "% -- Elapsed time (total) " << int(1000 * elapsed_secs) << " ms"<< std::endl;
   cout << "Matching time / total time:  " <<  elapsed_time_matching / elapsed_secs << std::endl  << "\n";

  // Print estimations
  //cout << estimation.x << " " << estimation.y << endl;
 }
  
}


#endif
