#include "dotDetector.h"

//ROS topics
static const std::string topicName="SemanticTopic";
static const std::string topicName2="ObservationTopic";

using namespace std;
using namespace message_filters;

dotDetector::dotDetector()
{    
  angleToSend = -100;
  
  // Initialize the saved odometry to a REALLY fake one
  lastOdometry.position.x = -100;
  lastOdometry.position.y = -100;
  lastOdometry.position.z = -100;
  lastOdometry.orientation.x = -1000;
  lastOdometry.orientation.y = -1000;
  lastOdometry.orientation.z = -1000;
  lastOdometry.orientation.w = -1000;

  lastStamp = ros::Time::now().toSec();

  status = DEACTIVATED;
  ros::NodeHandle private_handle("~");
  
  struct passwd *pw = getpwuid(getuid());
  
  private_handle.param<std::string>("log_dir_path", this->log_dir_path, string(pw->pw_dir) + "/Desktop");
  ROS_INFO("dot_detector: log_dir_path = %s", this->log_dir_path.c_str());
  
  private_handle.param<std::string>("odom_topic", this->odom_topic, "odom");
  ROS_INFO("dot_detector: odom_topic = %s", this->odom_topic.c_str());
  
  private_handle.param<std::string>("laser_topic", this->laser_topic, "front_scan");
  ROS_INFO("dot_detector: laser_topic = %s", this->laser_topic.c_str());
  
  private_handle.param<std::string>("tf_prefix", this->tf_prefix, "robot_0");
  ROS_INFO("dot_detector: tf_prefix = %s", this->tf_prefix.c_str());
  
  private_handle.param<std::string>("robot_name", this->robot_name, "robot_0");
  ROS_INFO("dot_detector: robot_name = %s", this->robot_name.c_str());

  top_camera_name = "top";
  private_handle.getParam("top_camera_name", top_camera_name);
  ROS_INFO("dot_detector: top_camera_name = %s", top_camera_name.c_str());
  
  bottom_camera_name = "bottom";
  private_handle.getParam("bottom_camera_name", bottom_camera_name);
  ROS_INFO("dot_detector: bottom_camera_name = %s", bottom_camera_name.c_str());
  
  detection_threshold = 20; //num of times
  private_handle.getParam("detection_threshold", detection_threshold);
  ROS_INFO("dot_detector: detection_threshold = %d", detection_threshold);
  
  pixel_offset = 30; //in pixels
  private_handle.getParam("pixel_offset", pixel_offset);
  ROS_INFO("dot_detector: pixel_offset = %d", pixel_offset);

  detection_range_Z = 3000; //in mm
  private_handle.getParam("detection_range_Z", detection_range_Z);
  ROS_INFO("dot_detector: detection_range_Z = %d", detection_range_Z);

  detection_range_Y = 300; //in mm
  private_handle.getParam("detection_range_Y", detection_range_Y);
  ROS_INFO("dot_detector: detection_range_Y = %d", detection_range_Y);

  max_dot_radius = 50; //in pixels
  private_handle.getParam("max_dot_radius", max_dot_radius);
  ROS_INFO("dot_detector: max_dot_radius = %d", max_dot_radius);

  min_dot_radius = 3; //in pixels
  private_handle.getParam("min_dot_radius", min_dot_radius);
  ROS_INFO("dot_detector: min_dot_radius = %d", min_dot_radius);

  ready_msg_sent = false;
  
  //laser range finder
  laser_sub_ok = false;
  laserScanSub = handle.subscribe(this->laser_topic, 1, &dotDetector::laserScanCallback, this);
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  
  //top kinect
  top_rgb_sub_ok = false;
  //top_rgbSub.subscribe(handle, top_camera_name+"/rgb/image_rect_color", 1);
//	string top_rgb_sub_string = top_camera_name+"/rgb/image_rect_color";
  string top_rgb_sub_string = top_camera_name+"/camera_middle/rgb/image_raw"; // with bananas
  top_rgbSub=handle.subscribe(top_rgb_sub_string, 1, &dotDetector::top_rgb_callback, this);
	ROS_INFO("top_rgb_subscription_string = %s", top_rgb_sub_string.c_str()); 
  //top_depthSub.subscribe(handle, top_camera_name+"/depth_registered/image_rect_raw", 1);

  top_depth_sub_ok = false;
	//string top_depth_sub_string = top_camera_name+"/depth_registered/image_raw"; //Raw image from device. Contains uint16 depths in mm. 
  string top_depth_sub_string = top_camera_name+"/camera_middle/depth/image_raw"; // with bananas
  top_depthSub= handle.subscribe(top_depth_sub_string, 1, &dotDetector::top_depth_callback, this);
	ROS_INFO("top_depth_subscription_string = %s", top_depth_sub_string.c_str()); 

  
  //bottom kinect
  bottom_sub_ok = false;
  //bottom_rgbSub.subscribe(handle, bottom_camera_name+"/rgb/image_rect_color", 1);
  //bottom_depthSub.subscribe(handle, bottom_camera_name+"/depth_registered/image_rect_raw", 1);
  //Synchronizer<MySyncPolicy> bottom_sync(MySyncPolicy(10), bottom_rgbSub, bottom_depthSub);
  //bottom_sync.registerCallback(boost::bind(&dotDetector::bottom_callback, this, _1, _2));

    
  if(bottom_sub_ok) {
	cv::namedWindow("BOTTOM KINECT - RGB View", CV_WINDOW_AUTOSIZE);
  }
  
  //odometry
  odom_sub_ok = false;
  odomSub = handle.subscribe(this->odom_topic, 1, &dotDetector::odomCallback, this);
  
  //trigger
  triggerSub = handle.subscribe("dotDetector/trigger", 10, &dotDetector::triggerCallback, this);

  laserROSPub = handle.advertise<geometry_msgs::PointStamped>("LaserPoseROS", 1024);
  semanticInfoPub = handle.advertise<dot_detector::Tag>(topicName, 1024);
  observationInfoPub = handle.advertise<dot_detector::Obs>(topicName2, 1024);
  eventPub = handle.advertise<std_msgs::String>("/" + robot_name + "/PNPConditionEvent", 1);
  sayPub = handle.advertise<std_msgs::String>("/SapienzBotDialog/say_command", 1);
  
  //BackgroundSubtraction
  top_bg_subtractor = new BackgroundSubtractorIMBS(
    5.0,        //double fps,
    15,         //unsigned int fgThreshold,
    5,          //unsigned int associationThreshold,
    100.,       //double samplingPeriod,
    2,          //unsigned int minBinHeight,
    20,         //unsigned int numSamples,
    0.65,       //double alpha,
    1.15,       //double beta,
    60,         //double tau_s,
    40,         //double tau_h,
    30,         //double minArea,
    10000,      //double persistencePeriod,
    true        //bool morphologicalFiltering
  );

  bottom_bg_subtractor = new BackgroundSubtractorIMBS(
    5.0,        //double fps,
    15,         //unsigned int fgThreshold,
    5,          //unsigned int associationThreshold,
    100.,       //double samplingPeriod,
    2,          //unsigned int minBinHeight,
    20,         //unsigned int numSamples,
    0.65,       //double alpha,
    1.15,       //double beta,
    60,         //double tau_s,
    40,         //double tau_h,
    30,         //double minArea,
    10000,      //double persistencePeriod,
    true        //bool morphologicalFiltering
  );
  
  currentScan = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  flippedScan = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  movingMessageImage = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  putText(movingMessageImage, "dot_detector Deactivated", cv::Point(30, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 255, 255));
  
  backgroundMessageImage = cv::Mat::zeros(cv::Size(640, 480), CV_8UC1);
  putText(backgroundMessageImage, "Extracting Image Background", cv::Point(30, 100), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 255, 255));
  
  take_picture = handle.advertiseService("/TakePicture", &dotDetector::takePictureHandler, this);
  
  //needed for a bug found on topic synchronization (ApproximateTime + boost)
  //ros::spin();

  activation=false;

}

dotDetector::~dotDetector()
{
  cv::destroyAllWindows();
  delete top_bg_subtractor;
  delete bottom_bg_subtractor;
}

void dotDetector::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  if(odom_sub_ok == false) {
    odom_sub_ok = true;

  }
  
  geometry_msgs::Pose newPose = msg->pose.pose;
    
  if(status != DEACTIVATED && old_status != status) {
    lastOdometry = newPose;
    lastStamp = ros::Time::now().toSec();
    ROS_INFO("ACTIVATING DOT DETECTOR...");

    activation=true;
    
    //if(top_sub_ok) cv::imshow("TOP KINECT - RGB View", backgroundMessageImage);
    
    //if(bottom_sub_ok) cv::imshow("BOTTOM KINECT - RGB View", backgroundMessageImage);
    
    cv::waitKey(5);
    
    top_background_generated = false;
    bottom_background_generated = false;
    

  }
  else if (old_status != status) {
    ROS_INFO("DEACTIVATING DOT DETECTOR...");
    

    activation=false;
    //create a new background model
    reset_mutex.lock();

    if(top_rgb_sub_ok)
            top_bg_subtractor->resetBg();
    
    if(bottom_sub_ok)
            bottom_bg_subtractor->resetBg();
    
    top_fgMask = cv::Scalar(0);
    bottom_fgMask = cv::Scalar(0);
    ready_msg_sent = false;

    
    reset_mutex.unlock();

	cv::destroyAllWindows();


  }
  
  old_status = status;
}

void dotDetector::triggerCallback(const std_msgs::String::ConstPtr& msg) {
  if(msg->data == "ACTIVE(Mem)") {
    status = MEMORIZING;
  }
  else if(msg->data == "ACTIVE(Rec)") {
    status = RECOGNIZING;
  }
  else if(msg->data == "ACTIVE(Upd)") {
    status = UPDATING;
  }
  else {
    status = DEACTIVATED;
  }
}

void dotDetector::laserScanCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  if(laser_sub_ok == false) {
    laser_sub_ok = true;
  }
  
  currentLaser.angle_increment = msg->angle_increment;
  currentLaser.angle_max = msg->angle_max;
  currentLaser.angle_min = msg->angle_min;
  currentLaser.ranges = msg->ranges;
}

//TOP KINECT
void dotDetector::top_rgb_callback(const sensor_msgs::ImageConstPtr& rgb)
{
  if(top_rgb_sub_ok == false) {
    top_rgb_sub_ok = true;
  }

  if(status != DEACTIVATED) {
    cv_bridge::CvImageConstPtr cv_ptr_rgb;
    
    try {
      cv_ptr_rgb = cv_bridge::toCvCopy(rgb, "bgr8");


    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("cv_bridge exception: %s", ex.what());
      return;
    }
    

    top_rgb_camera_mutex.lock();
    cv_ptr_rgb->image.copyTo(top_rgbMat);
    top_rgb_camera_mutex.unlock();

  }
}

void dotDetector::top_depth_callback(const sensor_msgs::ImageConstPtr& depth)
{
  if(top_depth_sub_ok == false) {
    top_depth_sub_ok = true;
  }

  if(status != DEACTIVATED) {
    cv_bridge::CvImageConstPtr cv_ptr_depth;
    
    try {
      
	cv_ptr_depth = cv_bridge::toCvCopy(depth, depth->encoding);

	if (depth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  	{
    		//ROS_INFO("Disparity image is 32-bit floating point, converting to 16UC1");
    		cv_ptr_depth->image *= 1000;
  	}

	
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("cv_bridge exception: %s", ex.what());
      return;
    }
    

    top_depth_camera_mutex.lock();
    cv_ptr_depth->image.convertTo(top_depthMat, CV_16UC1);
    top_depth_camera_mutex.unlock();
  }
}


//BOTTOM KINECT
void dotDetector::bottom_callback(const sensor_msgs::ImageConstPtr& rgb, const sensor_msgs::ImageConstPtr& depth)
{
  if(bottom_sub_ok == false) {
    bottom_sub_ok = true;
  }
  
  if(status != DEACTIVATED) {
    cv_bridge::CvImageConstPtr cv_ptr_rgb;
    cv_bridge::CvImageConstPtr cv_ptr_depth;
    
    try {
      cv_ptr_rgb = cv_bridge::toCvCopy(rgb, "bgr8");
      cv_ptr_depth = cv_bridge::toCvCopy(depth, depth->encoding);
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("cv_bridge exception: %s", ex.what());
      return;
    }
    
    bottom_camera_mutex.lock();
    cv_ptr_rgb->image.copyTo(bottom_rgbMat);
    cv_ptr_depth->image.copyTo(bottom_depthMat);
    bottom_camera_mutex.unlock();
  }
}

cv::Mat dotDetector::getDotMask(const cv::Mat &rgbFrame, const cv::Mat &fgMask) {
  //hsv conversion
  cv::cvtColor(rgbFrame, hsvMat, CV_BGR2HSV);
  cv::split(hsvMat, hsv_channels);

  cv::Mat dotMask = cv::Mat::zeros(rgbFrame.rows, rgbFrame.cols, CV_8UC1);

  for(int i = 0; i < dotMask.rows; ++i) {
    for(int j = 0; j < dotMask.cols; ++j) {
      if(hsv_channels[0].at<uchar>(i,j) < 70 && hsv_channels[0].at<uchar>(i,j) > 40 &&
        hsv_channels[1].at<uchar>(i,j) < 140 && hsv_channels[2].at<uchar>(i,j) > 210) {
          dotMask.at<uchar>(i,j) = 255;
      }
    }
  }

  // Reduce the noise so we avoid false circle detection
  for(int i = 0; i < dotMask.rows; ++i) {
    for(int j = 0; j < dotMask.cols; ++j) {
        if(dotMask.at<uchar>(i,j) == (uchar)255 && !(fgMask.at<uchar>(i,j) > 0)) {
          dotMask.at<uchar>(i,j) = 0;
      }
    }
  }

  //morphological operators
  cv::dilate(dotMask, dotMask, cv::Mat(), cv::Point(-1,-1), 2);
  cv::erode(dotMask, dotMask, cv::Mat(), cv::Point(-1,-1), 2);
  cv::dilate(dotMask, dotMask, cv::Mat(), cv::Point(-1,-1), 6);
  cv::GaussianBlur( dotMask, dotMask, cv::Size(9, 9), 2, 2 );

  return dotMask;
}

void dotDetector::getDotInMap(Eigen::Vector3f &dotToKinect, cv::Mat &rawRgbFrame, cv::Mat &rgbFrame, cv::Mat &depthFrame) {
  /*//Angle of the detected dot wrt the robot
  //get a set of points in the neighbourood of detected dot
  float angle = atan2(-dotToKinect.x(), dotToKinect.z());
  int offsetCells = lrint(angle/currentLaser.angle_increment);
  int zeroRadiansCell = (currentLaser.ranges.size()*.5) - 1;
  int lookUpCell = zeroRadiansCell + offsetCells;
  int firstPoint = lookUpCell - 20; //first point of the set
  int lastPoint = lookUpCell + 20; //last point of the set*/
  geometry_msgs::PointStamped dot3Drobot, dot3Dkinect, dot3Dmap;

  while(ros::Time::now() == ros::Time(0));
  tf::TransformListener *listener = new tf::TransformListener();

  Eigen::Vector3f dotToRobot; //the dot in robot's frame

  dot3Dkinect.header.frame_id = "/" + this->robot_name + "/top_link";
  dot3Dkinect.header.stamp = ros::Time(0); // ::now();
  dot3Dkinect.point.x = dotToKinect(0);
  dot3Dkinect.point.y = dotToKinect(1);
  dot3Dkinect.point.z = dotToKinect(2);


  dot3Drobot.header.frame_id = "/" + this->robot_name + "/base_link";
  dot3Drobot.header.stamp = ros::Time(0);

  dot3Dmap.header.frame_id = "map";
  dot3Dmap.header.stamp = ros::Time(0);

  //tf::StampedTransform kinect_transform;
  tf::StampedTransform robot_transform;
  
  try {
    listener->waitForTransform(dot3Dmap.header.frame_id, dot3Dkinect.header.frame_id, ros::Time(0), ros::Duration(3));
    listener->lookupTransform(dot3Dmap.header.frame_id, dot3Dkinect.header.frame_id, ros::Time(0), robot_transform);
    listener->transformPoint(dot3Dmap.header.frame_id, dot3Dkinect, dot3Dmap);

    //listener->waitForTransform(dot3Drobot.header.frame_id, dot3Dkinect.header.frame_id, ros::Time(0), ros::Duration(3));
    //listener->lookupTransform(dot3Drobot.header.frame_id, dot3Dkinect.header.frame_id, ros::Time(0), kinect_transform);
    listener->transformPoint(dot3Drobot.header.frame_id, dot3Dkinect, dot3Drobot);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("Errore trasformate:: %s",ex.what());
  }
  
  delete listener;

  //Angle of the detected dot wrt the robot
  //get a set of points in the neighbourood of detected dot
  float angle = atan2(dot3Drobot.point.y, dot3Drobot.point.x);
  int offsetCells = lrint(angle/currentLaser.angle_increment);
  int zeroRadiansCell = (currentLaser.ranges.size()*.5) - 1;
  int lookUpCell = zeroRadiansCell + offsetCells;
  int firstPoint = lookUpCell - 20; //first point of the set
  int lastPoint = lookUpCell + 20; //last point of the set

  cv::Vec4f line;
  currentScan = cv::Scalar(0);

  //draw the complete laser scan
  int ranges = currentLaser.ranges.size();
  for(int k = 0; k < ranges; ++k) {
    float rho = currentLaser.ranges[k];
    float angle = k*currentLaser.angle_increment;
    cv::Point2f currPoint(((rho*cos(angle))*60)+320, ((rho*sin(angle))*60)+240);
    cv::line(currentScan, currPoint, currPoint, cv::Scalar(100), 3);
  }
  
  //draw the neighbourood points
  if((firstPoint >= 0) && (lastPoint < ranges)) {
    std::vector<cv::Point2f> points;
    
    for(int i = firstPoint; i <= lastPoint; ++i) {
      float rho = currentLaser.ranges[i];
      float angle = i*currentLaser.angle_increment;
      cv::Point2f currPoint(((rho*cos(angle))*60)+320, ((rho*sin(angle))*60)+240);
      points.push_back(currPoint);
      cv::line(currentScan, currPoint, currPoint, cv::Scalar(255), 6);
    }
    
    cv::fitLine(points, line, CV_DIST_L2, 0, 0.01, 0.01);
  }

  //flip the currentScan to put the robot at the bottom of the image
  cv::flip(currentScan, flippedScan, 0);

//  cv::imshow("Scan", flippedScan);
  cv::Point2f somePoint(line[2], line[3]);

  angleToSend = atan2(-line[1], line[0]);//atan2(-line[1], -line[0])*180/M_PI;
  angleToSend = angleToSend*180/M_PI; //siamo sicuri della normale??? rivedere

  //use laser depth istead of kinect depth
  float valueToSend = (currentLaser.ranges[lookUpCell]); //*1000
  /*dotToKinect(2) = valueToSend;*/
  dot3Drobot.point.x = valueToSend;
  //std::cout << "Dot Associated Value is: " << valueToSend << std::endl;

  //Static transform from kinect's to robot's frame
  /*Eigen::Matrix3f staticTransform;
  staticTransform <<    0, 0, 1,
                        -1, 0, 0,
                        0, 1, 0;

  Eigen::Vector3f dotToRobot; //the dot in robot's frame
  dotToRobot = staticTransform*dotToKinect;*/

  //Generate dot point message wrt robot frame
  //geometry_msgs::PointStamped dot3Drobot, dot3Dkinect, dot3Dmap;
  /*dot3Dkinect.header.frame_id = "/" + this->robot_name + "/top_link";
  dot3Dkinect.header.stamp = ros::Time(0); // ::now();*/
  /*dot3Dkinect.point.x = dotToKinect(2)/1000;
  dot3Dkinect.point.z = dotToKinect(1)/1000;
  dot3Dkinect.point.y = -dotToKinect(0)/1000;*/

  /*dot3Drobot.header.frame_id = "/" + this->robot_name + "/base_link";
  dot3Drobot.header.stamp = ros::Time::now();*/
  /*dot3Drobot.point.x = dotToRobot(0)/1000; //positive forward
  dot3Drobot.point.y = dotToRobot(1)/1000; //positive left
  dot3Drobot.point.z = dotToRobot(2)/1000; //positive upward*/

  std::cout << "DOT in ROBOT frame --> x: " << dot3Drobot.point.x << 
  "  y: " << dot3Drobot.point.y << "  z: " << dot3Drobot.point.z << std::endl;

  //object segmentation
  cout << "STARTING OBJECT SEGMENTATION" << endl;

  //pubblica
  dot_detector::Obs msg;

  /*dot3Dmap.point.x /= 1000.f;
  dot3Dmap.point.y /= 1000.f;
  dot3Dmap.point.z /= 1000.f;*/
  std::cout << "Map DOT.x: " << dot3Dmap.point.x << std::endl;
  std::cout << "Map DOT.y: " << dot3Dmap.point.y << std::endl;
  std::cout << "Map DOT.z: " << dot3Dmap.point.z << std::endl;

  float x_robot = robot_transform.getOrigin().x();
  float y_robot = robot_transform.getOrigin().y();
  double roll, pitch, yaw;
  tf::Matrix3x3(robot_transform.getRotation()).getRPY(roll, pitch, yaw);
  float froll = (float) roll*180/M_PI;
  float fpitch = (float) pitch*180/M_PI;
  float theta_robot = (float) yaw*180/M_PI;

  std::cerr << "\033[95m robotX = " << x_robot << ", robotY = " << y_robot << ", robotTheta = " << theta_robot << "\033[0m" << std::endl;
  
  msg.stamp = ros::Time::now();
  msg.posx = dot3Dmap.point.x;
  msg.posy = dot3Dmap.point.y;
  msg.theta = (angleToSend + theta_robot + 180.0)*M_PI/180.f;

  std::cerr << "\033[95m posObjGlobX = " << msg.posx << ", posObjGlobY = " << msg.posy << " posObjGlobTheta = " << msg.theta<< "\033[0m" << std::endl;

  char buf[1024] = "";
  time_t t = msg.stamp.toSec();
  struct tm *tms = localtime(&t);
  strftime(buf, 1024, "%Y%m%d%H%M%S", tms);
  std::string timestring(buf);

  //Invio istogramma colore immagine (please path relativo)
//   std::string hist_H = "hist-h-" + timestring + ".txt";
//   std::string hist_S = "hist-s-" + timestring + ".txt";
//   std::string hist_V = "hist-v-" + timestring + ".txt";
// 
//   msg.properties = "objHistdata~" + hist_H + "^" + hist_S + "^" + hist_V;

  //Invio immagini log (path relativo)
  std::string rgbFilename = "rgb-" + timestring + ".png"; 
  std::string depthFilename = "depth-" + timestring + ".png";
  std::string laserFilename = "laser-" + timestring + ".png";
  std::string rawRgbFilename = "raw-" + timestring + ".png";

  msg.properties += "objImagedata~" + rgbFilename + "^" + rawRgbFilename + "^" + depthFilename + "^" + laserFilename;

  //Invio scansione laser (path relativo)
  std::string laserScan = "laserscan-" + timestring + ".txt";
  msg.properties += "~objLaserdata~" + laserScan;

  //Invio posa del robot
  std::stringstream ss;
  ss << x_robot << "^" << y_robot << "^" << theta_robot;
  std::string robotpose = ss.str();

  msg.properties += "~objSeenFrom~" + robotpose;
  
  //Invio posa del laser dot
  std::stringstream ss1;
  ss1 <<  msg.posx << "^" << msg.posy << "^" << msg.theta;
  std::string dotpose = ss1.str();

  msg.properties += "~objDotPosition~" + dotpose;

  cout << "Writing log files... " << endl;

  bool v1 = cv::imwrite(log_dir_path + "/" + rgbFilename, rgbFrame);
  if(!v1) cerr << "Unable to write " << log_dir_path + "/" + rgbFilename << endl;

  bool v2 = cv::imwrite(log_dir_path + "/" + depthFilename, depthFrame);
  if(!v2) cerr << "Unable to write " << log_dir_path + "/" + depthFilename << endl;

  bool v3 = cv::imwrite(log_dir_path + "/" + laserFilename, flippedScan);
  if(!v3) cerr << "Unable to write " << log_dir_path + "/" + laserFilename << endl;
  
  bool v5 = cv::imwrite(log_dir_path + "/" + rawRgbFilename, rawRgbFrame);
  if(!v5) cerr << "Unable to write " << log_dir_path + "/" + rawRgbFilename << endl;
  
  std::string robotPoseFilename = log_dir_path + "/" + "pose-" + timestring + ".txt";
  ofstream poseFile;
  poseFile.open(robotPoseFilename.c_str());
  
  bool v4 = poseFile.good();
  if(v4) {
    poseFile << "robotX = " << x_robot << "\nrobotY = " << y_robot << "\nfroll = " << froll << "\nfpitch = " << fpitch << "\nrobotTheta = " << theta_robot << "\n";
    v4 = poseFile.good();
    poseFile.close();
  }
  
  if(!v4) cerr << "Unable to write " << robotPoseFilename << endl;
  
  if(v1 && v2 && v3 && v4 && v5) cout << "done" << endl;
  
  std_msgs::String string_msg;
  string_msg.data = "DotDetected";
  eventPub.publish(string_msg);      //tells PNP that a dot has been detected
  string_msg.data = "Ok, hold on a second\n";
  sayPub.publish(string_msg);
  
  if (status == MEMORIZING || RECOGNIZING){
    // extract object properties by calling objects processing
    objects_processing::ExtractObjectProperties srv;
    srv.request.dotFramePath      = log_dir_path + "/" + rgbFilename;
    srv.request.rgbFramePath      = log_dir_path + "/" + rawRgbFilename;
    srv.request.depthFramePath    = log_dir_path + "/" + depthFilename;
    srv.request.poseFilePath      = robotPoseFilename;
    
    std::string obj_color = "";
    std::string possible_obj_tag = "";
    
    if (!ros::service::call("/objects_processing/extract_object_properties", srv)) {
      msg.dimx = 0.1;
      msg.dimy = 0.1;
      msg.dimz = 0.1;
      ROS_ERROR("Failed to call service ExtractObjectProperties"); 
    }
    else {
      if (srv.response.objColor != "")
        msg.properties += "~objColor~" + srv.response.objColor;
      possible_obj_tag = srv.response.possibleObjTag;
      msg.dimx = srv.response.objDimensions[0];
      msg.dimy = srv.response.objDimensions[1];
      msg.dimz = srv.response.objDimensions[2];
    }
    
    if (status == MEMORIZING) {
      //Invio dimensioni
      std::stringstream ss2;
      ss2 << msg.dimx << "^" << msg.dimy << "^" << msg.dimz;
      std::string dims = ss2.str();

      msg.properties += "~objDimensions~" + dims + "#";
      
      observationInfoPub.publish(msg);
      if (possible_obj_tag != "") {
        string_msg.data = "Categ_" + possible_obj_tag + "_It looks like a " + possible_obj_tag + ". Am I right?";      
        eventPub.publish(string_msg);
      }
      else {
        string_msg.data = "Please tell me the name of the object\n";
        sayPub.publish(string_msg);
      }
    }
    else {
      if (possible_obj_tag != "") string_msg.data = "Recognized_it looks like a " + possible_obj_tag + ". Am I right?";    
      else string_msg.data = "NotRecognized_this is hard, let me try again. Please point to the object.";
      eventPub.publish(string_msg);
    }
  }
  
  else if (status == UPDATING){
    string_msg.data = "[BEEP]500|400\n";
    sayPub.publish(string_msg);       // makes the beep sound 
  }
  
  else{
    ROS_ERROR("Not in a valid state status!");
  }
}

void dotDetector::findDot(cv::Mat &rawRgbFrame, cv::Mat &rgbFrame, cv::Mat &depthFrame, cv::Mat &dotMask, std::vector<DotObservation> &detections_history) {
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy;

  // Find contours
  cv::findContours(dotMask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

  // Approximate contours to polygons + get bounding rects and circles
  vector<vector<cv::Point> > contours_poly(contours.size());
  vector<cv::Point2f> center(contours.size());
  vector<float> radius(contours.size());

  for(unsigned int c = 0; c < contours.size(); ++c) {
    cv::approxPolyDP(cv::Mat(contours[c]), contours_poly[c], 3, true);
    cv::minEnclosingCircle( (cv::Mat)contours_poly[c], center[c], radius[c]);


    if(radius[c] > 0.f) {
	    circle( rgbFrame, center[c], radius[c], CV_RGB(255,255,255), 2);
    }

  }



  
  // dot tracking
  for(unsigned int c = 0; c < contours.size(); ++c) {
    if((int)radius[c] < max_dot_radius && (int)radius[c] > min_dot_radius) {
      bool found = false;
      for(std::vector<DotObservation>::iterator it = detections_history.begin(); it != detections_history.end(); ++it) {
        //metto tutte le osservazioni a invalido
        it->is_valid = false;
        
        if((fabs(it->dot.x - center[c].x) < pixel_offset) && (fabs(it->dot.y - center[c].y) < pixel_offset)) {
            found = true;
            it->dot = center[c];
            (it->n)++;
            it->miss = 0;
            it->is_valid = true;

		
		//circle( rgbFrame, it->dot, 6, CV_RGB(255,0,0), 2);

                break;
        }
      }
 
      if(!found) {
        DotObservation dotObs;
        dotObs.dot = center[c];
        dotObs.n = 1;
        dotObs.miss = 0;
        dotObs.is_valid = true;
        detections_history.push_back(dotObs);
      }
      
      std::vector<DotObservation> copy_detections_history;

      for(std::vector<DotObservation>::iterator it3 = detections_history.begin(); it3 != detections_history.end(); ++it3) {
        //tolgo tutte le osservazioni con id_valid false
        if(it3->is_valid) {
          copy_detections_history.push_back(*it3);
        }
        else if(it3->miss < 5) {
          (it3->miss)++;
          it3->is_valid = true;
          copy_detections_history.push_back(*it3);
        }
      }

      detections_history.clear();
      detections_history = copy_detections_history;


      bool clear_detections = false;
      for(std::vector<DotObservation>::iterator it2 = detections_history.begin(); it2 != detections_history.end(); ++it2) {
        if(it2->is_valid) {
          //dot from kinect data
          cv::Point3f dot;
          dot.x = ceil(center[c].x);
          dot.y = ceil(center[c].y);





	//circle( rgbFrame, cv::Point(ceil(center[c].x),ceil(center[c].y)), 4, CV_RGB(0,255,0), 2);
          
          //consider a 15x15 area to find z
          float sum = 0;
          float cnt = 0;

          int i_min_limit = std::max(0, (int)dot.y - 7);
          int i_max_limit = std::min((int)dot.y + 7, rgbFrame.rows - 1);

          int j_min_limit = std::max(0, (int)dot.x - 7);
          int j_max_limit = std::min((int)dot.x + 7, rgbFrame.cols - 1);
          
          for(int i = i_min_limit; i < i_max_limit; ++i) {
            for(int j = j_min_limit; j < j_max_limit; ++j) {
              
		//KINECT
		if(depthFrame.at<ushort>(i,j) > 0 && !isnan(depthFrame.at<ushort>(i,j))) {
	                sum += depthFrame.at<ushort>(i,j);
                	cnt++;
              	}

		/*//XTION
		if(depthFrame.at<float>(i,j) > 0.f && !isnan(depthFrame.at<float>(i,j))) {
	                sum += depthFrame.at<float>(i,j);
                	cnt++;
		}*/
            }
          }

          dot.z = ceil(sum/cnt);
          //dot in kinect's frame
          Eigen::Vector3f dotToKinect;
          
          //float focal_length = 525.f; //KINECT
		float focal_length = 577.3f; //KINECT
          
          //2D to 3D conversion (in m)
          dotToKinect(0) = dot.z/1000.f; //positive forward
          dotToKinect(1) = (-(dot.x - (depthFrame.cols/2))*dot.z/focal_length)/1000.f; //positive left
          dotToKinect(2) = (-(dot.y - (depthFrame.rows/2))*dot.z/focal_length)/1000.f; //positive upward

	 /*ROS_INFO("Z dotToKinect(2) %f", dotToKinect(2));
 	 ROS_INFO("Y dotToKinect(1) %f", dotToKinect(1));
	ROS_INFO("X dotToKinect(0) %f", dotToKinect(0));

	ROS_INFO("detection_range_Y %f", detection_range_Y);
	ROS_INFO("detection_range_Z %f", detection_range_Z);
	 */

	
          
          bool depth_is_valid = false;
          
          if(dotToKinect(2) > detection_range_Z || dotToKinect(1) > detection_range_Y) { //in mm
            //ROS_INFO("	DOT DETECTION: OUT OF RANGE");
            it2->is_valid = false;
          }
          else if(isnan(dot.x) || isnan(dot.y) || isnan(dot.z)) {
            //ROS_INFO("	DOT DETECTION: INVALID RANGE (NAN)");
            it2->is_valid = false;
          }
          else {
            //ROS_INFO("	DOT DETECTION: IN RANGE");

            cv::circle(rgbFrame, center[c], 3, CV_RGB(0,255,0), 3, 8, 0);

            if(it2->n > detection_threshold*0.33) {
              if(((int)radius[c]-3) > 0) {
                cv::circle(rgbFrame, center[c], (int)radius[c]-3, CV_RGB(0,0,255), 3, 8, 0);
              }
            }

            if(it2->n > detection_threshold*0.66) {
              if(((int)radius[c]-6) > 0) {
                cv::circle(rgbFrame, center[c], (int)radius[c]-6, CV_RGB(246, 172, 0), 3, 8, 0);
              }
            }

            if(it2->n > detection_threshold*0.95) {
              cv::rectangle(rgbFrame,
              cv::Point(center[c].x - (int)radius[c], center[c].y - (int)radius[c]),
              cv::Point(center[c].x + (int)radius[c], center[c].y + (int)radius[c]),
              CV_RGB(255, 0, 0), 4, 8, 0);
            }

            if((int) it2->n > detection_threshold) {
              cout << "VALID DOT DETECTION" << endl;
              depth_is_valid = true;

              cout << "DOT 3D POSITION WRT KINECT:" << endl;
              cout << "X: " << dotToKinect(0) << " m. (positive forward)" << endl;
              cout << "Y: " << dotToKinect(1) << " m. (positive left)" << endl;
              cout << "Z: " << dotToKinect(2) << " m. (positive upward)" << endl;
              
              cv::rectangle(rgbFrame, cv::Point(center[c].x - (int)radius[c] - 5, center[c].y - (int)radius[c] - 5),
              cv::Point(center[c].x + (int)radius[c] + 5, center[c].y + (int)radius[c] + 5),
              CV_RGB(0, 0, 0), 4, 8, 0);                                	
            }
          }
          if(depth_is_valid) {
            getDotInMap(dotToKinect, rawRgbFrame, rgbFrame, depthFrame);
            clear_detections = true;
            break;
          } //depth is valid
        } // > detection_threshold
      }
          
      //clear history
      if(clear_detections) {
        detections_history.clear();
        break;
      }
    }
  } //for contours
}

void dotDetector::detect()
{
  ROS_INFO("dot_detector: DETECT THREAD STARTED");
  
  while(status != DEACTIVATED) {
    if(laser_sub_ok) break;
    else ros::Duration(0.05).sleep();

    ros::spinOnce();
  }
  ROS_INFO("LASER [OK]");
  
  while(status != DEACTIVATED) {
    if(odom_sub_ok) break;
    else ros::Duration(0.05).sleep();
    ros::spinOnce();  
  }
  ROS_INFO("ODOM [OK]");

  while(status != DEACTIVATED) {
    if(top_depth_sub_ok) break;
    else {
       ros::Duration(0.05).sleep();
    }
    ros::spinOnce();
  }
  ROS_INFO("TOP DEPTH KINECT [OK]");

  while(status != DEACTIVATED) {
    if(top_rgb_sub_ok) break;
    else {
       ros::Duration(0.05).sleep();
    }
    ros::spinOnce();
  }
  ROS_INFO("TOP RGB KINECT [OK]");

    
	string window_name = "TOP KINECT - RGB View";
	cv::namedWindow(window_name, CV_WINDOW_AUTOSIZE);

  cv::Mat top_rgbFrame, top_depthFrame;                 //local copy to avoid problem with callbacks
  cv::Mat top_rawRgbFrame;                              //plain rgb frame without lines and circles for debug
  cv::Mat bottom_rgbFrame, bottom_depthFrame;           //local copy to avoid problem with callbacks
  cv::Mat bottom_rawRgbFrame;                           //plain rgb frame without lines and circles for debug
  cv::Mat top_fgMask_copy;
  cv::Mat bottom_fgMask_copy;

  

  ROS_INFO("START DETECTING CYCLE");

  while(status != DEACTIVATED) {
    ros::spinOnce();

    if(top_rgb_sub_ok && top_depth_sub_ok) {

		
      top_rgb_camera_mutex.lock();
      top_depth_camera_mutex.lock();
      
      if(!top_rgbMat.data || !top_depthMat.data) {
        top_depth_camera_mutex.unlock();
        top_rgb_camera_mutex.unlock();
        continue;
      }
      else {
        top_rgbFrame = top_rgbMat.clone();
        top_rawRgbFrame = top_rgbMat.clone();
        top_depthFrame = top_depthMat.clone();
      }
      
      top_depth_camera_mutex.unlock();
        top_rgb_camera_mutex.unlock();
      
      //get the fg mask
      reset_mutex.lock();
      top_bg_subtractor->apply(top_rgbFrame, top_fgMask, top_background_generated);

	  

      top_fgMask_copy = top_fgMask.clone();
      reset_mutex.unlock();

      if (top_background_generated){
        if (!ready_msg_sent){
          std_msgs::String event_message;
          event_message.data = "Please point at the object\n";
          sayPub.publish(event_message);
          ready_msg_sent = true;
        }
        
        //dot mask
        cv::Mat top_dotMask = getDotMask(top_rgbFrame, top_fgMask_copy);

	//cv::imshow("TOP KINECT - dot mask", top_dotMask);

        findDot(top_rawRgbFrame, top_rgbFrame, top_depthFrame, top_dotMask, top_detections_history);
        
      cv::imshow(window_name, top_rgbFrame);
      }
      
      if(bottom_sub_ok) {
        bottom_camera_mutex.lock();
      
        if(!bottom_rgbMat.data || !bottom_depthMat.data)
        {
          bottom_camera_mutex.unlock();
          continue;
        }
        else {
          bottom_rgbFrame = bottom_rgbMat.clone();
          bottom_rawRgbFrame = bottom_rgbMat.clone();
          bottom_depthFrame = bottom_depthMat.clone();
        }
      
        bottom_camera_mutex.unlock();
      
        //get the fg mask
        reset_mutex.lock();
        bottom_bg_subtractor->apply(bottom_rgbFrame, bottom_fgMask, bottom_background_generated);
        bottom_fgMask_copy = bottom_fgMask.clone();
        reset_mutex.unlock();    

        if (bottom_background_generated){
          if (!ready_msg_sent){
            std_msgs::String event_message;
            event_message.data = "[BEEP]500|400\n";
            sayPub.publish(event_message);       // makes the beep sound 
            ready_msg_sent = true;
          }
        
          //dot mask
          cv::Mat bottom_dotMask = getDotMask(bottom_rgbFrame, bottom_fgMask_copy);
          findDot(bottom_rawRgbFrame, bottom_rgbFrame, bottom_depthFrame, bottom_dotMask, bottom_detections_history);
          //cv::imshow("BOTTOM KINECT - RGB View", bottom_rgbFrame);
        }
      }


    }
    cv::waitKey(1);
  }

  if(status == DEACTIVATED) {
	 if(top_rgb_sub_ok&&top_depth_sub_ok) cv::imshow(window_name, movingMessageImage);
  //  if(bottom_sub_ok) cv::imshow("BOTTOM KINECT - RGB View", movingMessageImage);
    cv::waitKey(5);
  }
}

bool dotDetector::takePictureHandler(dot_detector::TakePicture::Request& req, dot_detector::TakePicture::Response& res) {
  ROS_INFO("Called take picture service");
  stringstream topPicture, bottomPicture, pictureName;
  
  if(top_rgb_sub_ok&&top_depth_sub_ok && top_rgbMat.data) {
    topPicture << log_dir_path + "/picture"+ top_camera_name << ros::Time::now().toSec() << ".png";
    pictureName << "/picture"+ top_camera_name << ros::Time::now().toSec();
    top_rgb_camera_mutex.lock();
    top_depth_camera_mutex.lock();
    if(cv::imwrite(topPicture.str(), top_rgbMat))
      ROS_INFO("Image taken from top camera"); 
    else
      ROS_ERROR("Could not take image from top camera");
    top_rgb_camera_mutex.unlock();
    top_depth_camera_mutex.unlock();
    res.pictureName = pictureName.str();
  }
  
  if(bottom_sub_ok && bottom_rgbMat.data) {
    bottomPicture << log_dir_path + "/picture"+ bottom_camera_name << ros::Time::now().toSec() << ".png";
    bottom_camera_mutex.lock();
    if(cv::imwrite(bottomPicture.str(), bottom_rgbMat))
      ROS_INFO("Image taken from bottom camera"); 
    else
      ROS_ERROR("Could not take image from bottom camera");; 
    bottom_camera_mutex.unlock();
  }
  
  if(!(top_rgb_sub_ok&&top_depth_sub_ok && top_rgbMat.data && bottom_sub_ok && bottom_rgbMat.data)) {
    ROS_ERROR("Kinect image not available");
    res.good = false;    
  }
  else res.good = true;
  return true;
}

void dotDetector::start()
{
  ros::Rate loop_rate(100);
  while(handle.ok())
  {
    ros::spinOnce();
    if(activation)
    {
      detect();
    }
    loop_rate.sleep();
  }
}



