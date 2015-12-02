#pragma once

#include <signal.h>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <pwd.h>

#include <eigen3/Eigen/Core>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_client.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PointStamped.h>

#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

#include <dot_detector/Tag.h>
#include <dot_detector/TakePicture.h>
#include <dot_detector/Obs.h>
#include <tf/transform_listener.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <objects_processing/RecognizeObject.h>
#include <objects_processing/ExtractObjectProperties.h>

//imbs
#include "imbs.hpp"

struct DotObservation {
  bool is_valid;          //if false the obsersation is not valid
  unsigned int n;         //number of detection for the dot
  unsigned int miss;      //number of missed detections
  cv::Point2f dot;        //image coordinates of the detected dot
};

typedef enum {
  DEACTIVATED,
  MEMORIZING,
  RECOGNIZING,
  UPDATING
} dot_detectorStatus;

class dotDetector
{
public:
  dotDetector();
  ~dotDetector();
  /*Detection function*/
	void start();

private:
  dot_detectorStatus status;
  dot_detectorStatus old_status;

  //status flag
  bool activation;

  /*Callback for Ros data aquisition*/
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr&);
  //void top_callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);
  void top_rgb_callback(const sensor_msgs::ImageConstPtr&);
  void top_depth_callback(const sensor_msgs::ImageConstPtr&);
  
  void bottom_callback(const sensor_msgs::ImageConstPtr&, const sensor_msgs::ImageConstPtr&);

  void odomCallback(const nav_msgs::OdometryConstPtr&);
  void triggerCallback(const std_msgs::String::ConstPtr&);
  
  //ROS Services
  bool takePictureHandler(dot_detector::TakePicture::Request &req, dot_detector::TakePicture::Response &res);

  cv::Mat getDotMask(const cv::Mat&, const cv::Mat&);
  void findDot(cv::Mat&, cv::Mat&, cv::Mat&, cv::Mat&, std::vector<DotObservation>&);
  void getDotInMap(Eigen::Vector3f&, cv::Mat&, cv::Mat&, cv::Mat&);
  void detect();
  

  //mutex stuff
  boost::mutex top_rgb_camera_mutex; boost::mutex top_depth_camera_mutex;
  boost::mutex bottom_camera_mutex;
  boost::mutex reset_mutex;
  
  //booleans
  bool top_rgb_sub_ok; bool top_depth_sub_ok;
  bool bottom_sub_ok;
  bool laser_sub_ok;
  bool odom_sub_ok;
  bool top_background_generated;
  bool bottom_background_generated;
  bool ready_msg_sent;

  //cv Mats
  cv::Mat movingMessageImage;
  cv::Mat backgroundMessageImage;
  cv::Mat hsvMat;
  std::vector<cv::Mat> hsv_channels;


  /*Detector specific stuff*/
  //cv::BackgroundSubtractorMOG2 top_bg_subtractor;
  BackgroundSubtractorIMBS* top_bg_subtractor;
  cv::Mat top_fgMask;
  
  //cv::BackgroundSubtractorMOG2 bottom_bg_subtractor;
  BackgroundSubtractorIMBS* bottom_bg_subtractor;
  cv::Mat bottom_fgMask;

  //top kinect
  cv::Mat top_depthMat;
  cv::Mat top_rgbMat;
  
  //bottom kinect
  cv::Mat bottom_depthMat;
  cv::Mat bottom_rgbMat;
  
  //hsv conversion
  cv::Mat top_hsvMat;
  cv::Mat bottom_hsvMat;
  std::vector<cv::Mat> top_hsv_channels;
  std::vector<cv::Mat> bottom_hsv_channels;
  
  //front laser
  cv::Mat currentScan;
  cv::Mat flippedScan;
  sensor_msgs::LaserScan currentLaser;

  //odometry
  geometry_msgs::Pose lastOdometry;
  double lastStamp;

  //strings
  std::string robot_name;
  std::string log_dir_path;
  std::string laser_topic;
  std::string odom_topic;
  std::string top_camera_name;
  std::string bottom_camera_name;
  std::string tf_prefix;
  
  //parameters for dot recognition
  int detection_threshold;
  int pixel_offset; 	        //in pixels
  int detection_range_Z; 	//in mm
  int detection_range_Y; 	//in mm
  int max_dot_radius; 		//in pixels
  int min_dot_radius; 		//in pixels

  /*Ros specific stuff*/
  ros::NodeHandle handle;

  //odometry subscriber
  ros::Subscriber odomSub;
  
  //laser subscriber
  ros::Subscriber laserScanSub;
  
  //trigger subscriber
  ros::Subscriber triggerSub;
  
  //top kinect subscriber
  ros::Subscriber top_rgbSub;
  ros::Subscriber top_depthSub;
  
  //bottom kinect subscriber
  message_filters::Subscriber<sensor_msgs::Image> bottom_rgbSub;
  message_filters::Subscriber<sensor_msgs::Image> bottom_depthSub;

  ros::Publisher laserROSPub;
  ros::Publisher semanticInfoPub;
  ros::Publisher observationInfoPub;
  ros::Publisher eventPub;
  ros::Publisher sayPub;
  
  ros::ServiceServer take_picture;
  
  std::vector<DotObservation> top_detections_history;
  std::vector<DotObservation> bottom_detections_history;

  //Specific parameters of the Kinect, not readable from ROS
  double kinect_focalDistance;          //Focal distance of the kinect in millimeters
  double kinect_pixelSize;              //pixel size in mm
  double kinect_ratio;

  float angleToSend;
};

