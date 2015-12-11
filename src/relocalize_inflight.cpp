#include <iostream>
#include <string>
#include "relocalize.h"

#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"


using namespace std;
using namespace cv;

int main() {

  // Construct relocalizer with reference image path
  Relocalizer relocalizer("../../draug/img/maze_sep_xs.jpg", 3.78, 2.673);

  // Open webcam
  VideoCapture inputVideo;
  string video_path = "../../Videos/outputRelocalizer.avi";
  inputVideo.open(video_path);


  if (!inputVideo.isOpened()) {
    std::cerr << "Capture not opened!" << std::endl;
    return -1;
  }
  cv::Mat imageBackground = imread("../../draug/img/maze_sep_s.jpg");
  while(inputVideo.grab()) {

    Mat query_image;
    inputVideo.retrieve(query_image);

    // Get estimation (x, y) in pixels from relocalizer
    cv::Point3f estimation = relocalizer.calcLocation(query_image);

    // Print estimations
    cout << estimation.x << " " << estimation.y << " " << estimation.z << endl;
    cv::circle(imageBackground,Point2d(estimation.x,estimation.y),3,Scalar(200,0,0));
    cv::imshow("camera image",query_image);
    cv::imshow("on location",imageBackground);
    cv::waitKey(30);
  }
  
}
