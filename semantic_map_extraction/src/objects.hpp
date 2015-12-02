// ---------------------------------------------------------------------------
// objects.hpp
// Semantic objects handler class
//
// Author: Roberto Capobianco <webrot9@gmail.com>
// ---------------------------------------------------------------------------
#ifndef _OBJECTS_HPP_
#define _OBJECTS_HPP_

#include "opencv2/highgui/highgui.hpp"


enum type {DOOR, NORMAL};
enum direction {UP, RIGHT, DOWN, LEFT};

class Objects {
	private:
		std::string name;
		cv::Point coords;
		float angle;
		direction dir;
		cv::Vec3f dims;
		std::string properties;
		type object_type;
	public:
		Objects(const std::string name, const cv::Point2i &coords, const float &angle, const cv::Vec3f &dims, const std::string &properties, const type &object_type);
		std::string getName();
		cv::Point getCoords();
		float getAngle();
		direction getDir();
		cv::Vec3f getDims();
		std::string getProperties();
		type getType();
		std::vector<cv::Point2i> getVertexCoords();
};

#endif
