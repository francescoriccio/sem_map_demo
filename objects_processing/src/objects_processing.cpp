// ---------------------------------------------------------------------------------------------
// objects_processing.cpp
// Objects processing node handler class and main
//
// Authors: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------------------------

#include "objects_processing.hpp"

void ObjectsProcessingNode::initColorMap() {
    std::vector<cv::Scalar> rangeHSV;
    
    // Red
    rangeHSV.push_back(cv::Scalar(0., 127.5, 191.25)); //min hsv
    rangeHSV.push_back(cv::Scalar(7.5, 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["red"] = rangeHSV;
    rangeHSV.clear();

    // Orange
    rangeHSV.push_back(cv::Scalar(7.5, 178.5, 255.)); //min hsv
    rangeHSV.push_back(cv::Scalar(20., 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["orange"] = rangeHSV;
    rangeHSV.clear();
    
    // Yellow
    rangeHSV.push_back(cv::Scalar(20., 127.5, 255.)); //min hsv
    rangeHSV.push_back(cv::Scalar(32.5, 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["yellow"] = rangeHSV;
    rangeHSV.clear();
    
    // Brown
    rangeHSV.push_back(cv::Scalar(0., 255., 63.75)); //min hsv
    rangeHSV.push_back(cv::Scalar(30., 255., 127.5)); // max hsv
    ObjectsProcessingNode::colorMap["brown"] = rangeHSV;
    rangeHSV.clear();
    
    // Green
    rangeHSV.push_back(cv::Scalar(32.5, 63.75, 63.75)); //min hsv
    rangeHSV.push_back(cv::Scalar(75., 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["green"] = rangeHSV;
    rangeHSV.clear();
    
    // Light Blue
    rangeHSV.push_back(cv::Scalar(75., 51., 191.25)); //min hsv
    rangeHSV.push_back(cv::Scalar(105., 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["light blue"] = rangeHSV;
    rangeHSV.clear();
    
    // Blue
    rangeHSV.push_back(cv::Scalar(105., 102., 38.25)); //min hsv
    rangeHSV.push_back(cv::Scalar(125., 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["blue"] = rangeHSV;
    rangeHSV.clear();
    
    // Purple
    rangeHSV.push_back(cv::Scalar(125., 127.5, 51.)); //min hsv
    rangeHSV.push_back(cv::Scalar(150., 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["purple"] = rangeHSV;
    rangeHSV.clear();
    
    // Purple
    rangeHSV.push_back(cv::Scalar(150., 127.5, 51.)); //min hsv
    rangeHSV.push_back(cv::Scalar(180., 255., 255.)); // max hsv
    ObjectsProcessingNode::colorMap["pink"] = rangeHSV;
    rangeHSV.clear();
    
    // White
    rangeHSV.push_back(cv::Scalar(0., 0., 0.)); //min hsv
    rangeHSV.push_back(cv::Scalar(180., 25.5, 255.)); // max hsv
    ObjectsProcessingNode::colorMap["white"] = rangeHSV;
    rangeHSV.clear();
    
    // Black
    rangeHSV.push_back(cv::Scalar(0., 0., 0.)); //min hsv
    rangeHSV.push_back(cv::Scalar(180., 255., 10.)); // max hsv
    ObjectsProcessingNode::colorMap["black"] = rangeHSV;
    rangeHSV.clear();
}

Eigen::Vector3f ObjectsProcessingNode::readAngleFromFile(std::string nameFile) {
	std::ifstream poseFile(nameFile.c_str());
  	std::string poseLine = "";
	std::string rollString = "";
	std::string pitchString = "";
  	std::string thetaString = "";
  	
  	if(poseFile.is_open()) {
  		while(std::getline(poseFile, poseLine)) {
			std::cout << "READING: " << poseLine << std::endl;
  			//Extract froll
		  	std::string toFindR("froll = ");
  			//Extract fpitch
		  	std::string toFindP("fpitch = ");
  			//Extract thetaRobot
		  	std::string toFindY("robotTheta = ");
		  	std::size_t foundR = poseLine.find(toFindR);
		  	std::size_t foundP = poseLine.find(toFindP);
		  	std::size_t foundY = poseLine.find(toFindY);

		  	if(foundR != std::string::npos) {
		  		rollString = poseLine.substr(foundR + toFindR.length());
		  	}

		  	if(foundP != std::string::npos) {
		  		pitchString = poseLine.substr(foundP + toFindP.length());
		  	}

		  	if(foundY != std::string::npos) {
		  		thetaString = poseLine.substr(foundY + toFindY.length());
		  	}
  		}
  		
  		poseFile.close();
  	}
  	else {
  		ROS_ERROR("Unable to read file of robot pose");
  		return Eigen::Vector3f(10000, 10000, 10000);
  	}
  	
  	//transform from string to double
  	std::istringstream ssY(thetaString), ssP(pitchString), ssR(rollString);
  	float froll, fpitch, robot_theta;
  	ssY >> robot_theta;
	ssR >> froll;
	ssP >> fpitch;
	Eigen::Vector3f angles(froll, fpitch, robot_theta);
  	
  	return angles;
}

cv::Mat ObjectsProcessingNode::readImageFromFile(std::string nameFile, bool loadUnchanged) {
	cv::Mat image;
	
	if(loadUnchanged) {
		image = cv::imread(nameFile, -1);
		image.convertTo(image, CV_16UC1);
	}
	else
		image = cv::imread(nameFile);

  	if(!image.data) {
  		ROS_ERROR("Error reading image %s", nameFile.c_str());
  		return cv::Mat::zeros(1,1, CV_8UC1);
  	}
  	
  	return image;	
}

std::string saveImage(cv::Mat object){
    char buffer[80];
    std::time_t rawtime;
    std::time (&rawtime);
    struct std::tm* timeinfo = std::localtime(&rawtime);
    std::strftime(buffer, 80, "%F_%T", timeinfo);

    std::string path = ros::package::getPath("objects_processing");
    path += "/objects_extracted/";
    std::string name(buffer);
    name += ".png";
    path += name;
    
    imwrite(path, object);
    
    return path;
}



cv::Point ObjectsProcessingNode::findGreenDot(cv::Mat dotFrame) {
	ROS_INFO("Finding green dot...");
	cv::Point greenDot(-1,-1);
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::Mat blackMat = cv::Mat::zeros(dotFrame.size(), CV_8UC1);
	for(int i = 0; i < dotFrame.rows; ++i) {
		for(int j = 0; j < dotFrame.cols; ++j) {
			if(	dotFrame.at<cv::Vec3b>(i,j)[0] == 0 &&
				dotFrame.at<cv::Vec3b>(i,j)[1] == 0 &&
				dotFrame.at<cv::Vec3b>(i,j)[2] == 255 )
			{
				blackMat.at<uchar>(i,j) = 255;
			}			
		}
	}


	cv::findContours(blackMat, contours, hierarchy, 3, 2, cv::Point(0, 0));
	int sum_x = 0;
	int sum_y = 0;
	double count = 0;

	for(std::vector<std::vector<cv::Point> >::iterator c = contours.begin(); c != contours.end(); c++) {
		for(std::vector<cv::Point>::iterator d = c->begin(); d != c->end(); d++) {
			sum_x += d->x;
			sum_y += d->y;
			count++;
		}	
	}

	greenDot.x = cvRound(sum_x/count);
	greenDot.y = cvRound(sum_y/count);

	ROS_INFO("Green dot found at (%d, %d)", greenDot.x, greenDot.y);
	
	return greenDot;    
}

void ObjectsProcessingNode::saveHistogram(cv::Mat rgbFrame) {
	cv::Mat hsvMat;
	std::vector<cv::Mat> hsv_channels;
	cv::cvtColor(rgbFrame, hsvMat, 40); 
	cv::split(hsvMat, hsv_channels);

	// Establish the number of bins
	int histSize = 256;

	// Set the ranges (for B,G,R)
	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; bool accumulate = false;

	cv::Mat h_hist, s_hist, v_hist;

	/// Compute the histograms:
	cv::calcHist(&hsv_channels[0], 1, 0, cv::Mat(), h_hist, 1, &histSize, &histRange, uniform, accumulate);
	cv::calcHist(&hsv_channels[1], 1, 0, cv::Mat(), s_hist, 1, &histSize, &histRange, uniform, accumulate);
	cv::calcHist(&hsv_channels[2], 1, 0, cv::Mat(), v_hist, 1, &histSize, &histRange, uniform, accumulate);

	// Draw the histograms for B, G and R
	int hist_w = 512; int hist_h = 400;
	//int bin_w = cvRound( (double) hist_w/histSize );

	cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

	/// Normalize the result to [ 0, histImage.rows ]
	cv::normalize(h_hist, h_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(s_hist, s_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
	cv::normalize(v_hist, v_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

	std::string histFile_H = "histFile_H.png";
	std::string histFile_S = "histFile_S.png";
	std::string histFile_V = "histFile_V.png";

	cv::imwrite(histFile_H, h_hist);
	cv::imwrite(histFile_S, s_hist);
	cv::imwrite(histFile_V, v_hist);
}

cv::Mat ObjectsProcessingNode::objectColorSegmentation(cv::Point greenDot, cv::Mat rgbFrame, cv::Mat depthFrame) {
	cv::Mat channel, mask, hsvMat;
	cv::Mat dot(rgbFrame.size(), CV_32S);

	std::vector<cv::Mat> hsv_channels;
	cv::cvtColor(rgbFrame, hsvMat, 40); 
	cv::split(hsvMat, hsv_channels);
	
	channel = hsv_channels[0];
	
	int dist = 50;

	int y_init = std::max(0, greenDot.y - dist);
	int y_finish =  std::min(rgbFrame.rows - 1, greenDot.y + dist);
	int x_init = std::max(0, greenDot.x - dist);
	int x_finish =  std::min(rgbFrame.cols - 1, greenDot.x + dist);

	uchar color_lb = channel.at<uchar>(y_init, x_init);
	uchar color_ub = color_lb;

	//check the colors around the green dot
	for(int i = y_init; i <= y_finish; i++)
		for(int j = x_init; j <= x_finish; j++) {
			if(channel.at<uchar>(i, j) < color_lb && color_lb - channel.at<uchar>(i, j) < 5) color_lb--;
			if(channel.at<uchar>(i, j) > color_ub && channel.at<uchar>(i, j) - color_ub < 5) color_ub++;
		}

	//get the mask
	cv::inRange(channel, cv::Scalar(color_lb), cv::Scalar(color_ub), mask);
	
	//fill the holes
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(mask, contours, 1, 2);
	mask = cv::Mat::zeros(channel.size(), CV_8U);

	cv::drawContours(mask, contours, -1, cv::Scalar::all(255), -1);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
	cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 4);
	cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
	
	dot = cv::Scalar::all(0);
	cv::circle(dot, greenDot, 1, cv::Scalar(200), -1);

	for(int i = 0; i < mask.rows; i++)
		for(int j = 0; j < mask.cols; j++)
			if(mask.at<int>(i, j) == 0) {
				cv::circle(dot, cv::Point(j, i), 1, cv::Scalar(201), -1);
				break;
			}

	cv::cvtColor(mask, mask, 8);
	cv::watershed(mask, dot);

	cv::Mat watershed_mask(dot.size(), CV_8U);

	for(int i = 0; i < dot.rows; i++) {
		for(int j = 0; j < dot.cols; j++) {
			if(dot.at<int>(i,j) != 200)
				watershed_mask.at<uchar>(i,j) = 0;
			else
				watershed_mask.at<uchar>(i,j) = 255;
		}
	}
			
//	cv::Mat result(mask.size(), CV_16U);
//	depthFrame.copyTo(result, watershed_mask);	
	
	// WARNING!
	// result = depthFrame;
	// END WARNING

	//returns the masked depth frame
	ROS_INFO("Color segmentation: DONE!");
	return watershed_mask;
}

cv::Mat ObjectsProcessingNode::objectDepthSegmentation(cv::Point greenDot, cv::Mat rgbFrame, cv::Mat depthFrame, cv::Mat segmented,  Eigen::Vector3f camera_angles) {
	float f = 525.f;
	float cx = 319.5f;
	float cy = 239.5f;
        
	double depth_limit = 0.03;
	float robot_theta, froll, fpitch;
	froll = camera_angles(0)*CV_PI/180;
	fpitch = camera_angles(1)*CV_PI/180;	
	robot_theta = camera_angles(2);
	
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cv::Mat cloudTo2D = cv::Mat::zeros(cv::Size(segmented.cols, segmented.rows), CV_8UC3);
	
	//find the 3D position of the green dot
	double dot_Z = depthFrame.at<ushort>(greenDot.y, greenDot.x);
	double factor = dot_Z / (f);
	double dot_X = dot_Z; //positive forward
	double dot_Y = -((double)(greenDot.x - cx)*factor); //positive left
	dot_Z = -((double)(greenDot.y - cy)*factor); //positive upward
	
	//prepare the transforms for the cloud
	Eigen::Matrix4f transform1;
	transform1 << 1, 0, 0, -dot_X*0.001,
		     0, 1, 0, -dot_Y*0.001,
		     0, 0, 1, -dot_Z*0.001,
		     0, 0, 0, 1;
	
	Eigen::Matrix4f transform2;
	Eigen::Matrix4f transformR;
	transformR << 1, 0, 0, 0,
		     0, cos(froll), -sin(froll), 0,
		     0, sin(froll), cos(froll), 0,
		     0, 0, 0, 1;
	
	Eigen::Matrix4f transformP;
	transformP << cos(fpitch), 0, sin(fpitch), 0,
		     0, 1, 0, 0,
		     -sin(fpitch), 0, cos(fpitch), 0,
		     0, 0, 0, 1;

	float ref_angle;
	
	while(robot_theta < 0)
		robot_theta += 360.f;
		
	robot_theta = fmod(robot_theta, 360.f);
	
	if(robot_theta <= 45 || robot_theta > 315)
		ref_angle = 0.f;
	else if(robot_theta <= 135 && robot_theta > 45)
		ref_angle = 1.57f;
	else if(robot_theta <= 225 && robot_theta > 135)
		ref_angle = 3.14f;
	else
		ref_angle = 4.71f;
	
	std::cout << "Robot theta: " << robot_theta << " Ref angle: " << ref_angle << std::endl;
	 
	float theta = robot_theta*CV_PI/180;
	transform2 << cos(ref_angle - theta), -sin(ref_angle - theta), 0, 0,
		     sin(ref_angle - theta), cos(ref_angle - theta), 0, 0,
		     0, 0, 1, 0,
		     0, 0, 0, 1;
		     
	cloud->width = segmented.cols;
	cloud->height = segmented.rows;
	cloud->points.resize(cloud->width*cloud->height);
	
	cv::Mat nanImage = cv::Mat::zeros(cv::Size(segmented.cols, segmented.rows), CV_8UC3);
	
	//produce a 3D cloud
	for(int i = 30; i < segmented.rows - 30; ++i) {
		for(int j = 50; j < segmented.cols - 50; ++j) {
			double z = segmented.at<ushort>(i,j);
			
			if(z != 0) {
				z = z/1000;
				double factor = z/(f);
				double x = z; //positive forward
				double y = -((double)(j - cx) * factor); //positive left
				double z = -((double)(i - cy) * factor); //positive upward
			 	
				cloud->points[i * segmented.cols + j].x =  x;
				cloud->points[i * segmented.cols + j].y =  y;
				cloud->points[i * segmented.cols + j].z =  z;

				cv::Vec3b color = rgbFrame.at<cv::Vec3b>(i, j);
				uint8_t r = color[2], g = color[1], b = color[0];
				uint32_t rgb = 0;
				rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
				cloud->points[i * segmented.cols + j].rgb = *reinterpret_cast<float*>(&rgb);
			}
			else {
				if(rgbFrame.at<cv::Vec3b>(i, j) != cv::Vec3b(0, 0, 0))
					nanImage.at<cv::Vec3b>(i, j) = rgbFrame.at<cv::Vec3b>(i, j);
					
				cloud->points[i * segmented.cols + j].rgb = 0;
			}
		}
	}

	//rotate the cloud in order to normalize objects to a standard visualization
	pcl::transformPointCloud(*cloud, *cloud, transform1);
	pcl::transformPointCloud(*cloud, *cloud, transformR);
	pcl::transformPointCloud(*cloud, *cloud, transformP);
	pcl::transformPointCloud(*cloud, *cloud, transform2);

	//pcl::visualization::CloudViewer viewer("asdsad");
	//viewer.showCloud(cloud);
	//while(!viewer.wasStopped());
	ROS_INFO("Normalization of the point of view...");
	
	for(size_t i = 0; i < cloud->height; ++i) {
	    	for(size_t j = 0; j < cloud->width; ++j) {
					if(fabs(cloud->points[i * segmented.cols + j].x) > depth_limit) {
		  			cloud->points[i * segmented.cols + j].x =  0;
		 				uint32_t rgb = 0;
		  			cloud->points[i * segmented.cols + j].rgb = *reinterpret_cast<float*>(&rgb);
					}
					else {
						uint32_t rgb = *reinterpret_cast<uint32_t*>(&cloud->points[i * segmented.cols + j].rgb);

						uint32_t r = ((rgb >> 16) & 0xFF);
						uint32_t g = ((rgb >> 8) & 0xFF);
						uint32_t b = (rgb & 0xFF);
				
			  		cloudTo2D.at<cv::Vec3b>(i, j) = cv::Vec3b(b,g,r);
		  		}
	     	}
	}
	
 cloudTo2D += nanImage;
 
	ROS_INFO("Depth segmentation: DONE!");
	
	return cloudTo2D;
}

cv::Mat ObjectsProcessingNode::objectBlobDetection(cv::Point greenDot, cv::Mat segmentedImage) {
	std::vector<std::vector<cv::Point> > contours;
	cv::Mat binaryImage = cv::Mat::zeros(cv::Size(segmentedImage.cols, segmentedImage.rows), CV_8UC1);
	cv::Mat mask = cv::Mat::zeros(cv::Size(segmentedImage.cols, segmentedImage.rows), CV_8UC1);
	cv::Mat final = cv::Mat::zeros(cv::Size(segmentedImage.cols, segmentedImage.rows), CV_8UC3);
			
	for(int i = 0; i < segmentedImage.rows; ++i) {
		for(int j = 0; j < segmentedImage.cols; ++j) {
			if(segmentedImage.at<cv::Vec3b>(i, j) != cv::Vec3b(0,0,0))
			  	binaryImage.at<uchar>(i, j) = (uchar) 255;
	     	}
	}
	
	cv::findContours(binaryImage, contours, 1, 1);
	size_t objectIdx = 0;
	double min_dist = -1;
	cv::Rect boundRect;
	
	for(size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++) {
		cv::Moments moms = moments(cv::Mat(contours[contourIdx]));
		double area = moms.m00;

		if (area < 100)
		        continue;
		
		cv::Point2d center(moms.m10/moms.m00, moms.m01/moms.m00);
		std::vector<cv::Point> blob_poligon;
		
		if(min_dist < 0) {
			objectIdx = contourIdx;
			min_dist = sqrt(pow((center.x - greenDot.x), 2) + pow((center.y - greenDot.y), 2));
			cv::approxPolyDP(cv::Mat(contours[contourIdx]), blob_poligon, 3, true);
			boundRect = cv::boundingRect(cv::Mat(blob_poligon));
		}
		else {
			double dist = sqrt(pow((center.x - greenDot.x), 2) + pow((center.y - greenDot.y), 2));
			
			if(dist < min_dist) {
				objectIdx = contourIdx;
				min_dist = dist;
				cv::approxPolyDP(cv::Mat(contours[contourIdx]), blob_poligon, 3, true);
				boundRect = cv::boundingRect(cv::Mat(blob_poligon));
			}
		}
	}
	
	cv::drawContours(mask, contours, objectIdx, cv::Scalar(255), -1);
	segmentedImage.copyTo(final, mask);
        cv::Mat cropped_final(final, boundRect);
        blob = cv::Mat(mask, boundRect);
	
	ROS_INFO("Blob detection: DONE!");
	
	return cropped_final;
}

cv::Mat ObjectsProcessingNode::extractObject(cv::Mat dotFrame, cv::Mat rgbFrame, cv::Mat depthFrame, Eigen::Vector3f camera_angles) {  	
	cv::Point greenDot = this->findGreenDot(dotFrame.clone());
        dotCoords = greenDot;
	cv::Mat color_mask = this->objectColorSegmentation(greenDot, rgbFrame.clone(), depthFrame.clone());
	cv::Mat segmented_depth, segmented_rgb;
	depthFrame.copyTo(segmented_depth, color_mask);
	rgbFrame.copyTo(segmented_rgb, color_mask);
	cv::Mat depth_segmented = this->objectDepthSegmentation(greenDot, segmented_rgb.clone(), depthFrame.clone(), segmented_depth.clone(), camera_angles);
	cv::Mat final_image = this->objectBlobDetection(greenDot, depth_segmented);
	
	return final_image;
}

int ObjectsProcessingNode::recognizeObject(cv::Mat model, cv::Mat query) {
	//detect the keypoints using SURF Detector
	int minHessian = 400;

	cv::SurfFeatureDetector detector(minHessian);

	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

	detector.detect(model, keypoints_1);
	detector.detect(query, keypoints_2);

	//calculate descriptors (feature vectors)
	cv::SurfDescriptorExtractor extractor;

	cv::Mat descriptors_1, descriptors_2;

	extractor.compute(model, keypoints_1, descriptors_1);
	extractor.compute(query, keypoints_2, descriptors_2);
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	
	if (!descriptors_1.empty() && !descriptors_2.empty()) {
		//matching descriptor vectors using FLANN matcher
		matcher.match(descriptors_1, descriptors_2, matches);
	}

	//double max_dist = 0; double min_dist = 100;

	//quick calculation of max and min distances between keypoints
	//for(int i = 0; i < descriptors_1.rows; i++)
	//{
		//double dist = matches[i].distance;
		//if(dist < min_dist) min_dist = dist;
		//if(dist > max_dist) max_dist = dist;
	//}

	//std::vector<cv::DMatch> good_matches;
	
	//select only "good" matches (i.e. whose distance is less than 2*min_dist and with the same (more or less) x coord and y coord
	int counter = 0;

	for(int i = 0; i < descriptors_1.rows && i < descriptors_2.rows; i++) {
		if(matches[i].distance <= 0.1) {
			counter++;
			//good_matches.push_back(matches[i]);
		}
	}
	
        //cv::Mat img_matches;
        //cv::drawMatches(model, keypoints_1, query, keypoints_2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        
        //cv::imshow("Good Matches", img_matches);
	//cv::waitKey(0);
        
	return counter;
}

cv::Point3f ObjectsProcessingNode::getObjectDimensions(cv::Point dotPoint, unsigned int rows, unsigned int cols, cv::Mat depthFrame){
  ROS_INFO("Calculating object dimensions");
  cv::Point3f dimensions;
  
  //find the 3D dimensions of the blob
  float f = 525.f;
  std::cout << depthFrame.type() << " TIPO" << std::endl;
  dimensions.z = float(depthFrame.at<ushort>(dotPoint.y, dotPoint.x)) / 1000.f; //;
  std::cout << "Z found: " << dimensions.z << std::endl;
  double factor = dimensions.z / f;
  dimensions.y = ((double)(rows)*factor);
  dimensions.x = ((double)(cols)*factor);
  dimensions.z = 0.03;

  std::cout << "dimensions: " << dimensions << std::endl;
  return dimensions;
}

std::string ObjectsProcessingNode::getObjectColor(cv::Mat object, cv::Mat blob){
  ROS_INFO("Finding object color");
  
  // start detecting obj color
  std::string objColor;
  cv::Mat colorMask = cv::Mat::zeros(cv::Size(object.cols, object.rows), CV_8UC1);
  cv::Mat hsvMat;
  cv::cvtColor(object, hsvMat, 40); 
  
  int countMask;
  int countMaskMax = 0;
  
//   std::string path = ros::package::getPath("objects_processing");
//   path += "/objects_extracted/";
//   cv::imwrite(path+"object.png", object);

  
  typedef std::map<std::string, std::vector<cv::Scalar> >::iterator it_type;
  for(it_type iterator = ObjectsProcessingNode::colorMap.begin(); iterator != ObjectsProcessingNode::colorMap.end(); iterator++) {
    countMask = 0;
    
    cv::inRange(hsvMat, cv::Scalar(iterator->second[0]), cv::Scalar(iterator->second[1]), colorMask);
    countMask = cv::countNonZero(colorMask & blob);
    
//     cv::imwrite(path+iterator->first+".png", colorMask);
//     std::cout << iterator->first << "->" << countMask << std::endl; 
    
    if (countMask >= countMaskMax) {
      countMaskMax = countMask;
      objColor = iterator->first;
    }
  }
  
//   std::cout << "Max color found in blob: " << objColor << std::endl;
  return objColor;
}

bool ObjectsProcessingNode::extractObjectHandler(objects_processing::ExtractObject::Request &req, objects_processing::ExtractObject::Response &res) {
	ROS_INFO("Object extraction request received.");
	cv::Mat dotFrame = readImageFromFile(req.dotFramePath);
	cv::Mat rgbFrame = readImageFromFile(req.rgbFramePath);
	cv::Mat depthFrame = readImageFromFile(req.depthFramePath, true);
	
	float robot_theta, froll, fpitch;
	Eigen::Vector3f camera_angles = readAngleFromFile(req.poseFilePath);
	froll = camera_angles(0);
	fpitch = camera_angles(1);	
	robot_theta = camera_angles(2);
	
	
  	if(robot_theta == 10000 || (dotFrame.rows == 1 && dotFrame.cols == 1) || (rgbFrame.rows == 1 && rgbFrame.cols == 1) || (depthFrame.rows == 1 && depthFrame.cols == 1))
  		return false;
  		
  	cv::Mat object = this->extractObject(dotFrame, rgbFrame, depthFrame, camera_angles);
  	
  	char buffer[80];
	std::time_t rawtime;
	std::time (&rawtime);
	struct std::tm* timeinfo = std::localtime(&rawtime);
	std::strftime(buffer, 80, "%F_%T", timeinfo);

  	std::string path = ros::package::getPath("objects_processing");
	path += "/objects_extracted/";
	std::string name(buffer);
	name += ".png";
	path += name;
	
	imwrite(path, object);
	
	res.objectImagePath = path;
	
	ROS_INFO("Object extraction: DONE!");
	
  	return true;
}

bool ObjectsProcessingNode::extractObjectPropertiesHandler(objects_processing::ExtractObjectProperties::Request &req, objects_processing::ExtractObjectProperties::Response &res) {
        ROS_INFO("Object properties extraction request received.");
        cv::Mat dotFrame = readImageFromFile(req.dotFramePath);
        cv::Mat rgbFrame = readImageFromFile(req.rgbFramePath);
        cv::Mat depthFrame = readImageFromFile(req.depthFramePath, true);
        
        float robot_theta, froll, fpitch;
	Eigen::Vector3f camera_angles = readAngleFromFile(req.poseFilePath);
	froll = camera_angles(0);
	fpitch = camera_angles(1);	
	robot_theta = camera_angles(2);
        
        if(robot_theta == 10000 || (dotFrame.rows == 1 && dotFrame.cols == 1) || (rgbFrame.rows == 1 && rgbFrame.cols == 1) || (depthFrame.rows == 1 && depthFrame.cols == 1))
                return false;
                
        cv::Mat object = this->extractObject(dotFrame, rgbFrame, depthFrame, camera_angles);
        cv::Point3f dimensions = this->getObjectDimensions(dotCoords, object.rows, object.cols, depthFrame);
        std::string objColor = this->getObjectColor(object, blob);
        
        std::string path = ros::package::getPath("objects_processing");
        path += "/objects_extracted/";
        cv::imwrite(path+"object.png", object);
        
        res.objColor = objColor;
        res.objDimensions.push_back(dimensions.x);
        res.objDimensions.push_back(dimensions.y);
        res.objDimensions.push_back(dimensions.z);
        
        ROS_INFO("Object color extraction: DONE!");
        
        return true;
}

bool ObjectsProcessingNode::extractColorHandler(objects_processing::ExtractColor::Request &req, objects_processing::ExtractColor::Response &res) {
	ROS_INFO("Color extraction request received.");
	
	cv::Mat dotFrame = this->readImageFromFile(req.dotFramePath);
	cv::Mat rgbFrame = this->readImageFromFile(req.rgbFramePath);
	cv::Mat depthFrame = this->readImageFromFile(req.depthFramePath, true);
  	cv::Mat colorMask = cv::Mat::zeros(cv::Size(depthFrame.cols, depthFrame.rows), CV_8UC1);
	cv::Mat hsvMat, channel;
	std::vector<cv::Mat> hsv_channels;
	
	float robot_theta, froll, fpitch;
	Eigen::Vector3f camera_angles = readAngleFromFile(req.poseFilePath);
	froll = camera_angles(0);
	fpitch = camera_angles(1);	
	robot_theta = camera_angles(2);
	
  	if(robot_theta == 10000 || (dotFrame.rows == 1 && dotFrame.cols == 1) || (rgbFrame.rows == 1 && rgbFrame.cols == 1) || (depthFrame.rows == 1 && depthFrame.cols == 1))
  		return false;
  		
	uint8_t min_hue = req.min_hue;
	uint8_t max_hue = req.max_hue;

  	cv::Mat object = this->extractObject(dotFrame, rgbFrame, depthFrame, camera_angles);
  	
	cv::cvtColor(object, hsvMat, 40); 
	cv::split(hsvMat, hsv_channels);
	
	channel = hsv_channels[0];
	
	cv::inRange(channel, cv::Scalar(min_hue), cv::Scalar(max_hue), colorMask);
	
	cv::waitKey(20);
	int countAll = 0;
	int countMask = 0;
	
	for(int i = 0; i < channel.rows; i++) {
		for(int j = 0; j < channel.cols; j++) {
			if(channel.at<uchar>(i,j) != 0)
				countAll++;
				
			if(colorMask.at<uchar>(i,j) != 0)
				countMask++;
		}
	}
	
	//at least the 40% of pixels on the object should have that color
  	if((float)countMask/(float)countAll > 0.4)
  		res.good = true;
  	else
  		res.good = false;
  		
	ROS_INFO("Color extracted: percentage %f.", (float)countMask/(float)countAll);
	ROS_INFO("Color extraction: DONE!");
	
  	return true;
}

bool ObjectsProcessingNode::recognizeObjectHandler(objects_processing::RecognizeObject::Request &req, objects_processing::RecognizeObject::Response &res) {
        ROS_INFO("Object recognition request received.");
        
        cv::Mat dotFrame = readImageFromFile(req.dotFramePath);
        cv::Mat rgbFrame = readImageFromFile(req.rgbFramePath);
        cv::Mat depthFrame = readImageFromFile(req.depthFramePath, true);
        
        float robot_theta, froll, fpitch;
	Eigen::Vector3f camera_angles = readAngleFromFile(req.poseFilePath);
	froll = camera_angles(0);
	fpitch = camera_angles(1);	
	robot_theta = camera_angles(2);
        
        if(robot_theta == 10000 || (dotFrame.rows == 1 && dotFrame.cols == 1) || (rgbFrame.rows == 1 && rgbFrame.cols == 1) || (depthFrame.rows == 1 && depthFrame.cols == 1))
                return false;
                
        cv::Mat query = this->extractObject(dotFrame, rgbFrame, depthFrame, camera_angles);
        cv::cvtColor(query, query, 6);
        
        if(query.rows == 1 && query.cols == 1)
                return false;
                
        int max_count = 0;
        
        std::string path = ros::package::getPath("objects_processing");
        std::string name("");
        path += "/models/";
        
        DIR* dir = opendir(path.c_str());
        struct dirent* dirEntry;
        
        if (dir != NULL) {
                while((dirEntry = readdir(dir))) {
                        std::string str(dirEntry->d_name);
                        
                        if(str != "." && str != ".." && str.find(".png") != std::string::npos) {
                                cv::Mat model = this->readImageFromFile(path + str);
                                cvtColor(model, model, 6);
                                
                                if(model.rows == 1 && model.cols == 1)
                                        return false;
                                        
                                int i = this->recognizeObject(model, query);
                                
                                if(i > max_count) {
                                        max_count = 1;
                                        name = std::string(dirEntry->d_name);
                                }
                        }
                }
                
                closedir(dir);
        }
        else {
                ROS_ERROR("Could not read the model's directory");
                return false;
        }
        
        res.objectName = name;
        
        if(name.compare("") == 0)
                ROS_INFO("Object not recognized.");
        else
                ROS_INFO("Object recognized: %s", name.c_str());
        
        ROS_INFO("Object recognition: DONE!");
        
        return true;
}

bool ObjectsProcessingNode::recognizeObjectInSceneHandler(objects_processing::RecognizeObjectInScene::Request &req, objects_processing::RecognizeObjectInScene::Response &res) {
        ROS_INFO("Object recognition request received.");
        
        cv::Mat rgbFrame = readImageFromFile(req.rgbFramePath);
        cv::Mat query = rgbFrame;
        
        cv::cvtColor(query, query, 6);
        
        if(query.rows == 1 && query.cols == 1)
                return false;
                
        int max_count = 0;
        
        std::string path = ros::package::getPath("objects_processing");
        std::string name("");
        path += "/models/";
        
        DIR* dir = opendir(path.c_str());
        struct dirent* dirEntry;
        
        if (dir != NULL) {
                while((dirEntry = readdir(dir))) {
                        std::string str(dirEntry->d_name);
                        
                        if(str != "." && str != ".." && str.find(".png") != std::string::npos) {
                                cv::Mat model = this->readImageFromFile(path + str);
                                cvtColor(model, model, 6);
                                
                                if(model.rows == 1 && model.cols == 1)
                                        return false;
                                        
                                int i = this->recognizeObject(model, query);
                                
                                if(i > max_count) {
                                        max_count = i;
                                        name = std::string(dirEntry->d_name);
                                }
                        }
                }
                
                closedir(dir);
        }
        else {
                ROS_ERROR("Could not read the model's directory");
                return false;
        }
        
        res.objectName = name;
        std::cout << "Good matches: " << max_count << std::endl;
        if(name.compare("") == 0)
                ROS_INFO("Object not recognized.");
        else
                ROS_INFO("Object recognized: %s", name.c_str());
        
        ROS_INFO("Object recognition: DONE!");
        
        return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "objects_processing");
	ros::NodeHandle n("~");
	
        ros::ServiceServer extract_object;
        ros::ServiceServer extract_object_properties;
	ros::ServiceServer extract_color;
        ros::ServiceServer recognize_object;
        ros::ServiceServer recognize_object_in_scene;

	ObjectsProcessingNode* op_node = new ObjectsProcessingNode();

        extract_object = n.advertiseService("extract_object", &ObjectsProcessingNode::extractObjectHandler, op_node);
        extract_object_properties = n.advertiseService("extract_object_properties", &ObjectsProcessingNode::extractObjectPropertiesHandler, op_node);
	extract_color = n.advertiseService("extract_color", &ObjectsProcessingNode::extractColorHandler, op_node);
        recognize_object = n.advertiseService("recognize_object", &ObjectsProcessingNode::recognizeObjectHandler, op_node);
        recognize_object_in_scene = n.advertiseService("recognize_object_in_scene", &ObjectsProcessingNode::recognizeObjectInSceneHandler, op_node);
	
        op_node->initColorMap();
        
	ROS_INFO("object_processing module started");
	
	ros::spin();

	return 0;
}
