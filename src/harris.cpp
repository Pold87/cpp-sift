#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace cv;
using namespace std;

/// Global variables
Mat src, src_gray;
int thresh = 200;
int max_thresh = 255;

char* source_window = "Source image";
char* corners_window = "Corners detected";

/// Function header
void cornerHarris_demo( int, void* );

/** @function main */
int main( int argc, char** argv ) {

  //  int video_path = 0;
  string video_path = "../data/output.avi";

VideoCapture capture(video_path);

  if(!capture.isOpened()) {
    cerr << "Capture not opened!" << endl;
    return 1;
  }

  namedWindow("TEST", CV_WINDOW_AUTOSIZE);
 
  while (true) {
    /// Load source image and convert it to gray
    capture.read(src);
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    
    /// Create a window and a trackbar
    namedWindow( source_window, WINDOW_AUTOSIZE );
    // createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );
    imshow( source_window, src );

    if (waitKey(30) > 0) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
      {
	cout << "esc key is pressed by user" << endl;
	capture.release();
	break; 
      }
    
    cornerHarris_demo( 0, 0 );

    
  }

  return(0);
}

/** @function cornerHarris_demo */
void cornerHarris_demo( int, void* )
{

  Mat dst, dst_norm, dst_norm_scaled;
  dst = Mat::zeros( src.size(), CV_32FC1 );

  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;

  /// Detecting corners
  cornerHarris( src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

  /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );

  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
              }
          }
     }
  /// Showing the result
  namedWindow( corners_window, WINDOW_AUTOSIZE );
  imshow( corners_window, dst_norm_scaled );
}
