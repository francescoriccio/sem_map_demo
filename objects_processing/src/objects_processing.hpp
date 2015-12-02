// ---------------------------------------------------------------------------
// objects_processing.hpp
// Objects processing node handler class and main
//
// Authors: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------

#ifndef _OBJECTS_PROCESSING_NODE_HPP_
#define _OBJECTS_PROCESSING_NODE_HPP_

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include <dirent.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <pcl/visualization/cloud_viewer.h>

#include "objects_processing/ExtractObject.h"
#include "objects_processing/ExtractObjectProperties.h"
#include "objects_processing/ExtractColor.h"
#include "objects_processing/RecognizeObject.h"
#include "objects_processing/RecognizeObjectInScene.h"

class ObjectsProcessingNode {
  private:
    cv::Mat blob;
    cv::Point dotCoords;
    std::map< std::string, std::vector<cv::Scalar> > colorMap;
    
    Eigen::Vector3f readAngleFromFile(std::string nameFile);
    cv::Mat readImageFromFile(std::string nameFile, bool loadUnchanged = false);
    std::string saveImage(cv::Mat object);
    cv::Point findGreenDot(cv::Mat dotFrame);
    cv::Point3f getObjectDimensions(cv::Point, unsigned int, unsigned int, cv::Mat);
    std::string getObjectColor(cv::Mat, cv::Mat);
    void saveHistogram(cv::Mat rgbFrame);
    cv::Mat objectColorSegmentation(cv::Point greenDot, cv::Mat rgbFrame, cv::Mat depthFrame);
    cv::Mat objectDepthSegmentation(cv::Point greenDot, cv::Mat rgbFrame, cv::Mat depthFrame, cv::Mat segmented, Eigen::Vector3f camera_angles);
    cv::Mat objectBlobDetection(cv::Point greenDot, cv::Mat segmentedImage);
    cv::Mat extractObject(cv::Mat dotFrame, cv::Mat rgbFrame, cv::Mat depthFrame, Eigen::Vector3f camera_angles);
    int recognizeObject(cv::Mat mode, cv::Mat query);

  public:
    bool extractObjectHandler(objects_processing::ExtractObject::Request &req, objects_processing::ExtractObject::Response &res);
    bool extractObjectPropertiesHandler(objects_processing::ExtractObjectProperties::Request &req, objects_processing::ExtractObjectProperties::Response &res);
    bool extractColorHandler(objects_processing::ExtractColor::Request &req, objects_processing::ExtractColor::Response &res);
    bool recognizeObjectHandler(objects_processing::RecognizeObject::Request &req, objects_processing::RecognizeObject::Response &res);
    bool recognizeObjectInSceneHandler(objects_processing::RecognizeObjectInScene::Request &req, objects_processing::RecognizeObjectInScene::Response &res);
    void initColorMap();
};
#endif
