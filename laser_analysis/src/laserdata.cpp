#include <ros/ros.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/LaserScan.h>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>


using namespace std;

// UTM-30
#define LASER_DATA_SIZE 1081
#define LASER_RANGE_MIN 20
#define LASER_ANGLE_INC (M_PI/(4*360))
#define LASER_ANGLE_MIN (-540.0*LASER_ANGLE_INC)



// Global variables

pcl::PointCloud<pcl::PointXYZ> cloud;
double z=0, inc_z=0.1;
bool rosnode=true;
char filename[255];


void cloud_add(const vector<double> &vx, const vector<double> &vy) 
{
      
      for (size_t i=0; i<vx.size(); i++)
	  cloud.push_back(pcl::PointXYZ(vx[i],vy[i],z));
      z += inc_z;
}


void write_cloud(int id=0) 
{
      pcl::PCDWriter pw;
      char fname[32]; sprintf(fname,"cloud_%04d.pcd",id);
      pw.write<pcl::PointXYZ>(fname,cloud,true);
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // ROS_INFO("*** laser ***");
    /*
    cout << "Angle [" << msg->angle_min << "," << msg->angle_max << "]   "
	 << " inc: " << msg->angle_increment << " " 
	 << " range: [" << msg->range_min << "," << msg->range_max << "]" << endl;
*/
      cout << setprecision(4);
      double t=msg->angle_min, r, x, y;
      
      vector<double> vx,vy;
      
      for (size_t i=0; i<msg->ranges.size(); i++) {
	
		if (msg->ranges[i]<msg->range_min) {
			r = msg->range_max; x=0; y=0;
		}
		else {
			r = msg->ranges[i];
			x = r * cos(t); y = r * sin(t);
			vx.push_back(x); vy.push_back(y);
			//if (i%40==0)
			//  cout << r << "\t";
		}
	
		// cout << x << "," << y << "   ";
	
		t += msg->angle_increment;
      }

      cout << "." << flush;
      // cout << endl << flush;
      
      cloud_add(vx,vy);
      
      write_cloud();
}

void read_data(const char *filename, bool timestamp=true) 
{
    int l=strlen(filename)-3;
    bool csv = strncasecmp((filename+l),"csv",3)==0;
    ifstream f; f.open(filename); 
    int k=10; vector<double> vx,vy;
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
    write_cloud();
}

void check_args(int argc, char **argv)
{
    filename[0]='\0';
    for (int i=0; i<argc; i++) {
	if (strcmp(argv[i],"-f")==0) {
	    strcpy(filename,argv[i+1]); i++;
        rosnode = false;
	}
      
    }
}

int main(int argc, char **argv)
{
    check_args(argc, argv);
    if (rosnode) {
		ros::init(argc, argv, "laserdata");
		ros::NodeHandle n; //("~");
		ros::Subscriber laserSubscriber = n.subscribe("scan", 10, laserCallback);
		ros::spin();
    }
    else {
		read_data(filename);
    }

  return 0;
}
