#include <boost/thread.hpp> 
#include <boost/thread/mutex.hpp>


// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

// PCL specific includes
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_ros/io/pcd_io.h>
//#include <pcl_ros/point_cloud.h>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/sample_consensus/sac_model_line.h>


#define DEG(a) ((a)/M_PI*180.0)
#define RAD(a) ((a)*M_PI/180.0)

using namespace std;

// UTM-30
#define LASER_DATA_SIZE 1081
#define LASER_RANGE_MIN 0.020
#define LASER_ANGLE_INC (M_PI/(4*180))
#define LASER_ANGLE_MIN (-540.0*LASER_ANGLE_INC)


// Global variables

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
double z=0, inc_z=0.2;
static boost::mutex m;
int cnt=0;



char filename[255], fileout[255];
ofstream fout;
bool rosnode,pcd,laserlog,car3D;
bool carmen=false;
bool timestamp=false;
int current_ts;

ros::NodeHandle *n;
ros::Subscriber laserSubscriber;

void processCloudForLineFitting();

int NCLOUDS_DISPLAY=1;

void print_ts(int ts)
{
	int ms = ts%1000;
	int s = ts/1000;
	int h = s/3600;
	int m = (s-h*3600)/60;
	s = s%60;
	printf("%02d:%02d:%02d.%03d",h,m,s,ms);
}

void setPoint(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b, pcl::PointXYZRGB &point) {
    point.x = x;
    point.y = y;
    point.z = z;
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                    static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float*>(&rgb);
}

void cloud_add(const vector<double> &vx, const vector<double> &vy) 
{
    m.lock();
    if (cnt++>NCLOUDS_DISPLAY)
      cloud->points.erase (cloud->points.begin(),cloud->points.begin()+vx.size());
    double xM=0; // xm(100000),xM(0),ym(100000),yM(0);
    for (size_t i=vx.size()/2-25; i<vx.size()/2+25; i++) {
        xM=std::max(xM,vx[i]);
    }

    double z_thr1=0.30, z_thr2=0.05;
    size_t im,iM; bool green_valid=true;
    for (size_t i=vx.size()/2; i<vx.size() && green_valid; i++) {
        if ((fabs(vx[i]-xM)<z_thr1) &&
            (fabs(vx[i]-vx[i-4])<z_thr2)) { iM=i; }
        else green_valid=false;
    }
    green_valid=true;
    for (size_t i=vx.size()/2-1; i>0 && green_valid; i--) {
        if ((fabs(vx[i]-xM)<z_thr1) &&
            (fabs(vx[i]-vx[i+4])<z_thr2)) { im=i; }
        else green_valid=false;
    }

    cloud_line->points.clear();

    pcl::PointXYZRGB point;
    for (size_t i=0; i<vx.size(); i++) {
        uint8_t r(255), g(255), b(255);
        if (i>im && i<iM) { // point to use for line/plane fitting
            cloud_line->points.push_back(pcl::PointXYZ(vx[i],vy[i],0.));
            r=0; b=0; // assign green color
        }
        setPoint(vx[i],vy[i],z,r,g,b,point);
        cloud->points.push_back(point);
    }

    // cout << "Range   i: " << im << " " << iM << endl;

    processCloudForLineFitting();

    if (car3D)
        z += inc_z;
    m.unlock();
}

void saveCloudLine()
{
    ofstream of("laserline.pts");
    for (int k=0; k<cloud_line->points.size(); k++) {
        of << cloud_line->points[k].y << " " << cloud_line->points[k].x << endl;
    }
    of.flush();
    of.close();
}

void saveCloudLineAvg()
{
    ofstream of("laserlineavg.pts");
    int d=30;
    for (int k=d; k<cloud_line->points.size()-d-1; k++) {
        double avgx = 0;
        for (int j=k-d; j<=k+d; j++)
           avgx += cloud_line->points[j].x;
        avgx /= (d*2+1);
        of << cloud_line->points[k].y << " " << avgx << endl;
    }
    of.flush();
    of.close();
}


void saveInliers(std::vector< int > inliers)
{
    ofstream of("laserinliers.pts");
    for (int k=0; k<inliers.size(); k++) {
        int i = inliers[k];
        of << cloud_line->points[i].y << " " << cloud_line->points[i].x << endl;
    }
    of.flush();
    of.close();
}

double findAngle(int d, int &isize)
{
    pcl::SampleConsensusModelLine< pcl::PointXYZ > model(cloud_line);

    std::vector<int> samples;
    std::vector< int > inliers;
    Eigen::VectorXf model_coefficients;
    Eigen::VectorXf optimized_coefficients;

    samples.resize(2);
    samples[0]=d; samples[1]=cloud_line->size()-d;

    bool r = model.computeModelCoefficients(samples, model_coefficients);
    if (r) {
        // cout << "Model found " << model_coefficients << endl;

        double threshold;
        for (threshold=0.001; threshold<=0.1; threshold*=10) {
            int cd = model.countWithinDistance (model_coefficients, threshold);
            cout << "Inliers within " << threshold << "   : " << cd << endl;
        }

        threshold = 0.01;

        model.selectWithinDistance(model_coefficients, threshold, inliers);

        if (inliers.size()>100) {

            saveCloudLine();
            saveCloudLineAvg();
            // saveInliers(inliers);

            model.optimizeModelCoefficients (inliers, model_coefficients, optimized_coefficients);


            cout << "Optimized model with inliers within " << threshold << " : " << endl;
            // cout << model_coefficients << endl;

     /*  Model coefficients
            point_on_line.x : the X coordinate of a point on the line
            point_on_line.y : the Y coordinate of a point on the line
            point_on_line.z : the Z coordinate of a point on the line
            line_direction.x : the X coordinate of a line's direction
            line_direction.y : the Y coordinate of a line's direction
            line_direction.z : the Z coordinate of a line's direction
       */
            double ang = atan2(optimized_coefficients[4],optimized_coefficients[3]); // atan2(Y/X)
            double dang = DEG(ang)+90;

            cout << "Y/X Angle = " << dang << endl;

            isize=inliers.size();
            return dang;
        } // if inliers > threshold

    } // if model found

    isize=0;
    return 1001;
}





void processCloudForLineFitting()
{   // process cloud_line to fit line model

    cout << "Cloud size for line fitting: " << cloud_line->size() << endl;
    int n=10;
    int ii[n]; int dd[n];
    for (int k=0; k<n; k++) {
        dd[k] = findAngle((k+1)*5,ii[k]);
    }

    if (fout.is_open())  {
        fout << current_ts << "\t";
        for (int k=0; k<n; k++) {
            fout << ii[k] << "\t" << dd[k] << "\t";
        }
        fout << endl;
        fout.flush();
    }

}




void check_log_file_type(ifstream &f)
{
    string s;
    if (f.good()) {
        f >> s;
        if (s=="ROBOTLASER1") {
            carmen=true;
        }
        else {
            float v = atof(s.c_str());
            if (v>30000) timestamp=true;
        }
        char sbuf[10000];
        f.getline(sbuf,10000);
        cout << "Log filetype autocheck: carmen: " << carmen << " timestamp: " << timestamp << endl;
        if (!f.good())
            cout << "ERROR. File not good!!!" << endl;
    }
}

void read_data() 
{
	cout << "Reading data ... " << endl;

    int l=strlen(filename)-3;
    bool csv = strncasecmp((filename+l),"csv",3)==0;

    strcpy(fileout,filename);
    fileout[l]='\0'; strcat(fileout,"_angles.out");

    ifstream f; f.open(filename); 
    fout.open(fileout);
    fout << "#MSOrarioCPU \tLaser-street angle" << endl;

	vector<double> vx,vy;

    check_log_file_type(f);

    int cnt=100;
    while (f.good() && --cnt>0) {

        int d; double v; char c; string s;
        int laser_data_size = LASER_DATA_SIZE;

        if (carmen) {
            f >> s;
            // 7 void elements
            for (int k=0;k<8;k++) f >> v;
            laser_data_size = (int)v; // number of data
            // cout << s << "  " << laser_data_size << endl;
        }
		// read timestamp
        else if (timestamp) {
            f >> current_ts; if (csv) f >> c;
		}
        print_ts(current_ts); cout << endl;
		// read data
		vx.clear(); vy.clear();
		double t = LASER_ANGLE_MIN, x,y;
        for (int k=0; k<laser_data_size; k++) {
            double r; // range in meters
            if (carmen) {
                f >> r;
            }
            else {
                f >> d; if (csv) f >> c;
                r = d/1000.0;
            }
            // cout << r << endl;

            if (r>LASER_RANGE_MIN) {
                x = r * cos(t); y = r * sin(t);
				vx.push_back(x); vy.push_back(y);
			}
			
			t += LASER_ANGLE_INC;
		} // for k
	
		cloud_add(vx,vy);
        usleep(10000);

        char sbuf[2000];
        f.getline(sbuf,2000);
		
    } // while
}


void start_readlogfile()
{
	cout << "Reading log file " << filename << endl;

	boost::thread t(read_data);
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)

{
    cout << "*" << flush;
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

      // cout << "." << flush;
      // cout << endl << flush;
      
      cloud_add(vx,vy);
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

    while (true)
    {
        p.removePointCloud (name);
		m.lock();
		p.addPointCloud(cloud,name);
		p.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
        m.unlock();

        p.spinOnce (100);
		if (rosnode) {
			ros::spinOnce();
            int refresh_rate=10; // Hz
            ros::Rate rate(refresh_rate);
            rate.sleep();
		}
    }  
}

void display_car3D()
{
    pcl::visualization::PCLVisualizer p ("Online PointCloud2 Viewer");

    p.spinOnce (10);

    p.addCoordinateSystem(1.0);

    string name = "cloud";

    p.spinOnce (10);

    // p.getViewerPose()

    p.setCameraPosition(30,0,0, 0,0,0, 0,0,-1);

    double Cz0 = -27.1621;
    double Oz = -10.0;
    // double dcz = 0.4;
    p.setCameraPosition(
        -12.7344,0.233616, Cz0,
        0,0,Oz,
         -0.905094,-0.0311787,0.424067
    );

    while (true)
    {
        p.removePointCloud(name);
        m.lock();
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        p.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, name);
        p.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);

        // set camera pose
        double pz=0;
        if (cloud!=NULL && cloud->points.size()>0) {
            pcl::PointXYZRGB pt;
            pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud->points.end();
            pt = *(--it);
            pz=pt.z;
            // cout << "Pz = " << pt.z << "  Oz = " << Oz << endl;
        }
        m.unlock();
        double Cz = Cz0+pz; Oz = pz; //dcz;
        p.setCameraPosition(
            -12.7344,0.233616,Cz,
            0,0,Oz,
             -0.905094,-0.0311787,0.424067
        );

        p.spinOnce (100);
        if (rosnode) {
            ros::spinOnce();
            int refresh_rate=10; // Hz
            ros::Rate rate(refresh_rate);
            rate.sleep();
        }
    }
}

void check_args(int argc, char **argv)
{
    filename[0]='\0'; rosnode=true; pcd=false; laserlog=false; car3D=false;
    for (int i=0; i<argc; i++) {
		if (strcmp(argv[i],"-h")==0) {
            cout << argv[0] << " -h | -log <logfilename> | -car3D" << endl;
			cout << "          default: run as a ROS node" << endl;
			exit(1);
		}      
		else if (strcmp(argv[i],"-log")==0) {
			strcpy(filename,argv[i+1]); i++;
			laserlog=true;
            rosnode=false;
		}   
        else if (strcmp(argv[i],"-ros")==0) {

        }
        else if (strcmp(argv[i],"-car3D")==0) {
            car3D=true; NCLOUDS_DISPLAY=100;
        }

    }
}

void close()
{
    if (rosnode) {
        laserSubscriber.shutdown();
    }
    if (fout.is_open())
        fout.close();
}

int  main (int argc, char** argv)
{
   check_args(argc, argv);

	if (rosnode) { 
		cout << "ROS node" << endl;   
		ros::init(argc, argv, "laserviewer");
        n = new ros::NodeHandle();
        laserSubscriber = n->subscribe("scan", 10, laserCallback);
        display();
	}
    else if (laserlog) {
        cout << "Log file" << endl;
		start_readlogfile();
        if (car3D)
            display_car3D();
        else
            display();
	}


    return (0);
}

