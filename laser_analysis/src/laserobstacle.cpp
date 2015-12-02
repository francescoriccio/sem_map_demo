#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

// BOOST
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/sample_consensus/sac_model_line.h>

#include <laser_analysis/LaserObstacle.h>
#include <laser_analysis/LaserObstacleMap.h>
#include <glocalizer/LocalizerRanges.h>

using namespace std;

// UTM-30
#define LASER_DATA_SIZE 1081
#define LASER_RANGE_MIN 20
#define LASER_ANGLE_INC (M_PI/(4*360))
#define LASER_ANGLE_MIN (-540.0*LASER_ANGLE_INC)



// Global variables

bool rosnode=true, gui=false;
string laser_frame_id="base_laser_link";
char filename[255];
static boost::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
ros::Publisher obstaclePublisher,obstacleMapPublisher;
tf::TransformListener *listener=NULL;

double obst_dx=20.0, obst_dy=0.5;



struct RobotPose {
    double x,y,th;
};

RobotPose robotpose;



void setPoint(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b, pcl::PointXYZRGB &point) {
    point.x = x;
    point.y = y;
    point.z = z;
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float*>(&rgb);
}

void cloud_add(const vector<float> &vx, const vector<float> &vy) 
{
      cloud_mutex.unlock();

      cloud->points.clear();
      pcl::PointXYZRGB point;
      for (size_t i=0; i<vx.size(); i++) {
          uint8_t r(255), g(255), b(255);
          if (fabs(vy[i])<obst_dy && (vx[i]>0 && vx[i]<obst_dx)) {
            b=0; g=0;
          }
          setPoint(vx[i],vy[i],0.0,r,g,b,point);
          cloud->points.push_back(point);
      }

      cloud_mutex.unlock();
}


void cloud_add_ranges(const vector<float> &vx, const vector<float> &vy, const vector<float> &dist)
{
      cloud_mutex.unlock();

      cloud->points.clear();
      pcl::PointXYZRGB point;
      for (size_t i=0; i<vx.size(); i++) {
          uint8_t r(255), g(255), b(255);
          if (dist[i]<0.5) {
            b=0; r=0; // green
          }
          else if (dist[i]<1.0) {
            b=0; // yellow
          }
          else {
            b=0; g=0; // red
          }
          setPoint(vx[i],vy[i],0.0,r,g,b,point);
          cloud->points.push_back(point);
      }

      cloud_mutex.unlock();
}



void getRobotPose()
{
    tf::StampedTransform transform;
    string destination_frame="/map";
    try {
        listener->waitForTransform(destination_frame, laser_frame_id, ros::Time(0), ros::Duration(1.0) );
        listener->lookupTransform(destination_frame, laser_frame_id, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("laserobstacle: %s",ex.what());
	std::cout << laser_frame_id << std::endl;
    }

    tf::Quaternion q = transform.getRotation();
    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    robotpose.x = transform.getOrigin().x();
    robotpose.y = transform.getOrigin().y();
    robotpose.th = yaw;
    /*cout << "Robot pose:  x = " << robotpose.x <<
            "   y = " << robotpose.y <<
            "  th = " << RAD2DEG(robotpose.th) << endl << endl;*/

}

double roundPI2(double a) // round angle to 0, PI/2, -PI/2, PI
{
    if ((a>=-M_PI_4 && a<=M_PI_4) || (a>=7*M_PI_4 && a<=2*M_PI)) {
        return 0;
    }
    else if (a>=M_PI_4 && a<=3*M_PI_4) {
        return M_PI_2;
    }
    else if ((a>=3*M_PI_4 && a<=5*M_PI_4) || (a>=-M_PI && a<=-3*M_PI_4)) {
        return M_PI;
    }
    else if ((a>=5*M_PI_4 && a<=7*M_PI_4) || (a>=-3*M_PI_4 && a<=-M_PI_4)) {
        return -M_PI_2;
    }
    else // should be not possible...
    	return 0;
}

void computeVectors(const sensor_msgs::LaserScan::ConstPtr& msg,
                    vector<float> &vx, vector<float> &vy)
{
    double t=msg->angle_min, r, x, y;
    double theta=roundPI2(robotpose.th)-robotpose.th; // rad
    t-=theta;
    for (size_t i=0; i<msg->ranges.size(); i++) {
        if (msg->ranges[i]<msg->range_min) {
            r = msg->range_max; x=0; y=0;
        }
        else {
            r = msg->ranges[i];
            x = r * cos(t); y = r * sin(t);
            vx.push_back(x); vy.push_back(y);
        }
        t += msg->angle_increment;
    }
}

void rotateVectors(const vector<float> &xx, const vector<float> &yy, vector<float> &vx, vector<float> &vy)
{
    float x, y;
    getRobotPose();
    float theta=roundPI2(robotpose.th)-robotpose.th; // rad
    
    for (size_t i=0; i<xx.size(); i++) {
      
	float t = atan2(yy[i],xx[i]);
	t-=theta;
      
      /*
        if (msg->ranges[i]<msg->range_min) {
            r = msg->range_max; x=0; y=0;
        }
        else { */
            float r = sqrt(xx[i]*xx[i]+yy[i]*yy[i]);
            x = r * cos(t); y = r * sin(t);
            vx.push_back(x); vy.push_back(y);
        

    }
}

void check_obstacles_ranges(const vector<float> &vx, const vector<float> &vy, const vector<float> &dist)
{
	double mx=0,my=0,varx=0,vary=0,var=0,wmx=0,wmy=0,ww=0; int count=0;
	for (size_t i=0; i<vx.size(); i++) {
		// cout << "   " << dist[i] << "   ";
		if (dist[i]>0.5) {
			mx += vx[i]; my += vy[i];
			wmx += vx[i]*dist[i]; wmy += vy[i]*dist[i];
			count++;
			ww += dist[i];
		}
	}
	// cout << endl;
	if (count>0) {
		mx /= count; my /= count;
		wmx /= ww; wmy /= ww;

		for (size_t i=0; i<vx.size(); i++) {
			if (dist[i]>0.5) {
				varx += (vx[i]-mx)*(vx[i]-mx);
				vary += (vy[i]-my)*(vy[i]-my);
			}
		}
		varx /= count; vary /= count;
		var = (varx+vary)/2;
	}

#if 0
   cout << setprecision(2);
   cout << "Obstacle_range:: count: " << count << "  ";
   cout << " obs. center: " << mx << " " << my << endl;
#endif

    laser_analysis::LaserObstacleMap omsg;
    omsg.stamp = ros::Time::now();
    omsg.npoints = count;
    omsg.mx = mx; omsg.my = my; omsg.var = var;

    obstacleMapPublisher.publish(omsg);
}

void check_obstacles(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    double t=msg->angle_min, r, x, y;
    int di=4, count=0;
    double x1=0, y1=0, x2=0, y2=0;
    bool first=true;
    double rmin=obst_dx;

    double theta=roundPI2(robotpose.th)-robotpose.th; // rad
    t-=theta;

    for (size_t i=0; i<msg->ranges.size(); i+=di) {

        if (msg->ranges[i]>msg->range_min) {
            r = msg->ranges[i];
            x = r * cos(t); y = r * sin(t);
            if (x>0 && x<obst_dx && fabs(y)<obst_dy) {
                count++;
                if (first)  {
                    x1 = x; y1 = y; first=false;
                }
                x2 = x; y2 = y;
                rmin = std::min(rmin,r);
            }
        }
        t += msg->angle_increment * di;
    }
#if 0
   cout << setprecision(2);
   cout << "robot pose th: " << robotpose.th << endl;
   cout << "count: " << count << "\trmin: " << rmin << "\ty: "<< y1 << " : " << y2 << endl << flush;
#endif

    laser_analysis::LaserObstacle omsg;
    omsg.stamp = ros::Time::now();
    omsg.npoints = count;
    omsg.mindist=rmin; omsg.y1 = y1; omsg.y2 = y2;

    obstaclePublisher.publish(omsg);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
#if 0
    if (gui) {
        vector<float> vx,vy;
        computeVectors(msg,vx,vy);
        cloud_add(vx,vy);
    }
#endif
    getRobotPose();
    check_obstacles(msg);
}

void laserRangeCallback(const glocalizer::LocalizerRanges::ConstPtr& msg)
{
	size_t n=msg->x.size();
	
	ROS_DEBUG_STREAM("LocalizerRanges " << n << " data");
#if 1
	if (gui) {
	    vector<float> vx,vy;
	    rotateVectors(msg->x,msg->y,vx,vy);
	    cloud_add_ranges(vx,vy,msg->dist);
	}
#endif
	check_obstacles_ranges(msg->x,msg->y,msg->dist);
}

void read_data(const char *filename, bool timestamp=true) 
{
    int l=strlen(filename)-3;
    bool csv = strncasecmp((filename+l),"csv",3)==0;
    ifstream f; f.open(filename); 
    int k=10; vector<float> vx,vy;
    while (f.good() && k-->0) {
	int d; char c;
	// read timestamp
	if (timestamp) {
		f >> d; if (csv) f >> c;
	}
	// read data
	double t = LASER_ANGLE_MIN, x,y;
	for (int k=0; k<=LASER_DATA_SIZE; k++) {
	    f >> d; if (csv) f >> c;
	    // cout << d << endl;

	    if (d>LASER_RANGE_MIN) {
		x = (d/1000.0) * cos(t); y = (d/1000.0) * sin(t);
		vx.push_back(x); vy.push_back(y);
	    }
	    
	    t += LASER_ANGLE_INC;
	}
	
	cloud_add(vx,vy);
    }
    f.close();
}



void display()
{
    pcl::visualization::PCLVisualizer p ("Online PointCloud2 Viewer");

    p.spinOnce (10);

    p.addCoordinateSystem(1.0);

    string name = "cloud";
    p.setCameraPosition(0,0,10,
        0,0,0,
        1,0,0
     );
    p.spinOnce (10);
    bool run=true;
    while (run)
    {
        p.removeAllShapes();
        p.removePointCloud (name);


        cloud_mutex.lock();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        p.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
        p.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
        cloud_mutex.unlock();

        double goalth=roundPI2(robotpose.th);  // rad
        double theta=robotpose.th-goalth, lsize=3;
        double lx = lsize * cos(theta), ly = lsize * sin(theta);

        Eigen::Vector3f point_on_line(0,0,0), line_direction(lx,ly,0);
        pcl::ModelCoefficients line_coeff;
        line_coeff.values.resize (6); // We need 6 values
        line_coeff.values[0] = point_on_line.x ();
        line_coeff.values[1] = point_on_line.y ();
        line_coeff.values[2] = point_on_line.z ();
        line_coeff.values[3] = line_direction.x ();
        line_coeff.values[4] = line_direction.y ();
        line_coeff.values[5] = line_direction.z ();

        p.addLine(line_coeff,string("thetarobot"),0);

        p.spinOnce (10);
        if (rosnode) {
            ros::spinOnce();
            int refresh_rate=20; // Hz
            ros::Rate rate(refresh_rate);
            rate.sleep();
            run = !ros::isShuttingDown();
        }
    }
}

void check_args(int argc, char **argv)
{
    filename[0]='\0';
    for (int i=0; i<argc; i++) {
        if (strcmp(argv[i],"-f")==0) {
            strcpy(filename,argv[i+1]); i++;
            rosnode = false;
        }
        else if (strcmp(argv[i],"-gui")==0) {
            gui = true;
        }

    }
}

int main(int argc, char **argv)
{
    check_args(argc, argv);
    if (rosnode) {
        cout << "ROS node laserobstacle init ..."<< endl;
        ros::init(argc, argv, "laserobstacle");
        ros::NodeHandle n, np("~");
        ros::Subscriber laserSubscriber =
                n.subscribe("scan", 1, laserCallback);
        ros::Subscriber localizerRangeSubscriber =
                n.subscribe("localizer_ranges", 1, laserRangeCallback);
        obstaclePublisher =
                n.advertise<laser_analysis::LaserObstacle>("laser_obstacle", 1000);

        obstacleMapPublisher =
                n.advertise<laser_analysis::LaserObstacleMap>("laser_obstacle_map", 1000);
        listener = new tf::TransformListener();

        ros::Duration(1.0).sleep();

        if (np.getParam("laser_frame_id", laser_frame_id))
            ROS_INFO("laserobstacle:: laser_frame_id = %s",laser_frame_id.c_str());
        if (np.getParam("obstacle_margin_x", obst_dx))
            ROS_INFO("laserobstacle:: obstacle_margin_x = %.3f",obst_dx);
        if (np.getParam("obstacle_margin_y", obst_dy))
            ROS_INFO("laserobstacle:: obstacle_margin_y = %.3f",obst_dy);

        if (gui) {
            cout << "start laserobstacle GUI ..." << endl;
            ros::spinOnce();
            display();
        }
        else
            ros::spin();
    }
    else {
	read_data(filename);
        display();
    }


    return 0;
}

