#include <string>

#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)

static	tf::TransformListener* listener = NULL;

bool getRobotPose(string robotname, double &x, double &y, double &th_rad)
{
  if (listener == NULL)
    listener = new tf::TransformListener();

	tf::StampedTransform transform;
    string src_frame = "/map";
    string dest_frame = "/" + robotname + "/base_frame";
    if (robotname=="") { // local trasnformation
        src_frame = "map";
        dest_frame = "base_frame";
    }
	try {
		listener->waitForTransform(src_frame, dest_frame, ros::Time(0), ros::Duration(3));
		listener->lookupTransform(src_frame, dest_frame, ros::Time(0), transform);
	    x = transform.getOrigin().x();
	    y = transform.getOrigin().y();
	    th_rad = tf::getYaw(transform.getRotation());
	}
	catch(tf::TransformException ex) {
        th_rad = 3*M_PI;
        ROS_ERROR("Error in tf trasnform %s -> %s\n",src_frame.c_str(), dest_frame.c_str());
		    ROS_ERROR("%s", ex.what());
        return false;
	}

    return true;
}


