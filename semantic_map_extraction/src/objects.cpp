#include "objects.hpp"


Objects::Objects(const std::string name, const cv::Point2i &coords, const float &angle, const cv::Vec3f &dims, const std::string &properties, const type &object_type) {
	this->name = name;
	this->coords = coords;
	this->angle = angle;
	
	while(this->angle < 0)
		this->angle += 2*CV_PI;
		
	this->angle = fmod(this->angle, 2*CV_PI);
		
	this->angle = 2*CV_PI - this->angle;
		
	this->angle = (angle < CV_PI/4) ? 0 : (angle < 3*CV_PI/4) ? CV_PI/2 : (angle < 5*CV_PI/4) ? CV_PI : 3*CV_PI/2;

	this->dir = (angle < CV_PI/4) ? RIGHT : (angle < 3*CV_PI/4) ? UP : (angle < 5*CV_PI/4) ? LEFT : DOWN;

	this->dims = dims;
	
	if(!name.empty()) {
		std::ostringstream oss;
		oss << "objectAngle~" << this->angle;
		std::string angle_property = oss.str();
		this->properties = angle_property;
	}
	
	if(!properties.empty())
		this->properties += "~" + properties;
	
	this->object_type = object_type;
}

std::string Objects::getName() {
	return this->name;
}

cv::Point Objects::getCoords() {
	return this->coords;
}

float Objects::getAngle() {
	return this->angle;
}

direction Objects::getDir() {
	return this->dir;
}

cv::Vec3f Objects::getDims() {
	return this->dims;
}

std::string Objects::getProperties() {
	return this->properties;
}

type Objects::getType() {
	return this->object_type;
}

std::vector<cv::Point2i> Objects::getVertexCoords() {
	std::vector<cv::Point2i> vert;

	switch(object_type) {
		default:
		case NORMAL:
			vert.push_back(cv::Point2i(this->coords.x-.5*this->dims[0]*cvRound(sin(this->angle)), this->coords.y-.5*this->dims[2]*cvRound(cos(this->angle))));
			vert.push_back(cv::Point2i(this->coords.x-.5*this->dims[0]*cvRound(sin(this->angle))+this->dims[0]*cvRound(cos(this->angle)), this->coords.y-.5*this->dims[2]*cvRound(cos(this->angle))+this->dims[2]*cvRound(sin(this->angle))));
			vert.push_back(cv::Point2i(this->coords.x+.5*this->dims[0]*cvRound(sin(this->angle))+this->dims[0]*cvRound(cos(this->angle)), this->coords.y+.5*this->dims[2]*cvRound(cos(this->angle))+this->dims[2]*cvRound(sin(this->angle))));
			vert.push_back(cv::Point2i(this->coords.x+.5*this->dims[0]*cvRound(sin(this->angle)), this->coords.y+.5*this->dims[2]*cvRound(cos(this->angle))));
			
		break;
		case DOOR:
			vert.push_back(cv::Point2i(this->coords.x-.5*this->dims[0]*cvRound(sin(this->angle)), this->coords.y-.5*this->dims[2]*cvRound(cos(this->angle))));
			vert.push_back(cv::Point2i(this->coords.x+.5*this->dims[0]*cvRound(sin(this->angle)), this->coords.y+.5*this->dims[2]*cvRound(cos(this->angle))));
		break;
	}

	return vert;
}
