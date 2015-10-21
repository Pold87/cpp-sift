int main(int argc, char* argv[]) {

  std::cout.precision(15);
  //cv::Point2f p(5, 5);
  //cv::Point2f p_2;

  // p_2 = rotate_point(p, 35);

  //  #std::cout << "x is: " << p_2.x << "y is: " << p_2.y;

  // Init matrices for cam images and the map image
  cv::Mat query_img, query_img_gray;
  cv::Mat map_img, map_img_gray;

  // Path of map  
  std::string map_img_path = "../data/newMaze17_full.jpg";

  // Path of drone information
  std::string drone_info_path =  "../data/data_preprocessed.csv";
  std::ifstream drone_info_file(drone_info_path.c_str());

  // Homograpy
  cv::Mat homography;

  map_img = cv::imread(map_img_path, CV_LOAD_IMAGE_COLOR);
  cv::cvtColor(map_img, map_img_gray, CV_BGR2GRAY);

  // Check for invalid input
  if (!map_img.data) {
      std::cout << "Could not open or find the image" << std::endl;
        return -1;
    }

  // Webcam usage or recorded video

  std::string video_path = "../data/output.avi";
  //int video_path = 1;

  // Load camera
  cv::VideoCapture video(video_path);

  if (!video.isOpened()) {
    std::cerr << "Capture not opened!" << std::endl;
    return -1;
  }

  // Create a window for display.
  //cv::namedWindow(query_window, cv::WINDOW_AUTOSIZE);
  //cv::namedWindow(matches_window, cv::WINDOW_AUTOSIZE);

  //-- Step 1: Detect the keypoints using SURF Detector
  std::vector<cv::KeyPoint> keypoints_query, keypoints_map;

  cv::Mat descriptors_query, descriptors_map;

  int minHessian = 400;
  //  cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
  //cv::Ptr<cv::xfeatures2d::FREAK> detector = cv::xfeatures2d::FREAK::create();
  //  cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
  cv::Ptr<cv::ORB> detector = cv::ORB::create();

  
  detector->detectAndCompute(map_img_gray, 
			     cv::noArray(), 
			     keypoints_map,
			     descriptors_map);


  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  high_resolution_clock::time_point t2 = high_resolution_clock::now();
  
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  
  // Read video frame by frame and do calculations
  while (true) {

    t1 = high_resolution_clock::now();
    std::cout << (duration / 1000) << "\n";

    // Read video, convert it to grayscale and display it 
    video.read(query_img);
    
    cv::cvtColor(query_img, query_img_gray, cv::COLOR_BGR2GRAY);
    //cv::imshow(query_window, query_img_gray);
    
    // Safely exit the recording on pressing any key
    if(cv::waitKey(1) >= 0) break;
   
   // Calculate keypoints
   
   detector->detectAndCompute(query_img_gray, 
			      cv::noArray(), 
			      keypoints_query,
			      descriptors_query);
   
   
   //   cv::FlannBasedMatcher matcher;
   cv::BFMatcher matcher;
   
   std::vector< std::vector<cv::DMatch> > matches;
   
   matcher.knnMatch(descriptors_query, 
		    descriptors_map, 
		    matches,
		   2);
   
   std::vector<cv::KeyPoint> matched_query, matched_map, inliers_query, inliers_map;
   std::vector<cv::DMatch> good_matches;

   //-- Localize the object
   std::vector<cv::Point2f> pts_query;
   std::vector<cv::Point2f> pts_map;
   
  for(size_t i = 0; i < matches.size(); i++) {
    
    cv::DMatch first = matches[i][0];
    float dist_query = matches[i][0].distance;
    float dist_map = matches[i][1].distance;
    
    if(dist_query < match_ratio * dist_map) {
      matched_query.push_back(keypoints_query[first.queryIdx]);
      matched_map.push_back(keypoints_map[first.trainIdx]);

      pts_query.push_back(keypoints_query[first.queryIdx].pt);
      pts_map.push_back(keypoints_map[first.trainIdx].pt);
      
    }
  }

  cv::Mat mask; 

  homography = cv::findHomography(pts_query, 
				  pts_map,
				  cv::RANSAC,
				  5,
				  mask);


  // Input Quadilateral or Image plane coordinates
  std::vector<cv::Point2f> centers(1), centers_transformed(1);
  cv::Point2f center(query_img.rows / 2, query_img.cols / 2);
  cv::Point2f center_transformed(query_img.rows / 2, query_img.cols / 2);
  
  centers[0] = center;
  cv::perspectiveTransform(centers, centers_transformed, homography);

  center_transformed = centers_transformed[0];
  
  std::cout << center_transformed.x << " " << center_transformed.y << std::endl;

  cv::Point3d ecef = pixel_pos_to_ecef_pos(center_transformed);

  std::cout << ecef << std::endl;
  
  //debug_vector(centers);
  //debug_vector(centers_transformed);

  for(unsigned i = 0; i < matched_query.size(); i++) {

    cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
    col.at<double>(0) = matched_query[i].pt.x;
    col.at<double>(1) = matched_map[i].pt.y;
    
    col = homography * col;
    col /= col.at<double>(2);
    double dist = sqrt( pow(col.at<double>(0) - matched_map[i].pt.x, 2) +
			pow(col.at<double>(1) - matched_map[i].pt.y, 2));
    
    if(dist < inlier_threshold) {
      int new_i = static_cast<int>(inliers_query.size());
      inliers_query.push_back(matched_query[i]);
      inliers_map.push_back(matched_map[i]);
      good_matches.push_back(cv::DMatch(new_i, new_i, 0));
   }
  }
  
  cv::Mat res, res_small;
  //cv::drawMatches(query_img_gray, inliers_query, 
		  // map_img_gray, inliers_map, 
		  // good_matches, res);
  

  //cv::Size size(640, 480);//the dst image size,e.g.100x100
  //cv::resize(res, res_small, size);
  
  //cv::imshow(matches_window, res_small);
  
  double inlier_ratio = inliers_query.size() * 1.0 / matched_query.size();
  // std::cout << "Matching Results" << std::endl;
  // std::cout << "*******************************" << std::endl;
  // std::cout << "# Keypoints 1:                        \t" << keypoints_query.size() << std::endl;
  // std::cout << "# Keypoints 2:                        \t" << keypoints_map.size() << std::endl;
  // std::cout << "# Matches:                            \t" << matched_query.size() << std::endl;
  // std::cout << "# Inliers:                            \t" << inliers_query.size() << std::endl;
  // std::cout << "# Inliers Ratio:                      \t" << inlier_ratio << std::endl;
  // std::cout << std::endl;

  t2 = high_resolution_clock::now();
  duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
  std::cout << "Duration" << duration << std::endl;
  
}
  
  return 0; 
  
}
