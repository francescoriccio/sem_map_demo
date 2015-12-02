#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "opencv2/core/core.hpp"
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

bool GUI=false;

int w=600;
int h=600;
cv::Mat imm=cv::Mat(h,w,CV_8U);
cv::Mat immTot=cv::Mat(h,w*2+2+2,CV_8U);
cv::Mat robot_grad;
cv::Mat dist_ridotta;
cv::Mat visual_joy1(h/2,w/2,CV_8U);
cv::Mat visual_joy2(h/2,w/2,CV_8U);

int size=0;
float angle_min=0;
float angle_incr=0;
std::vector<float> ranges;
ros::Time time_stamp;

float posx;
float posy;
int indx;
int indy;

int maskSize = CV_DIST_MASK_PRECISE;
int distType = CV_DIST_L2;
cv::Mat dist;

cv::Mat grad_x, grad_y;
cv::Mat grad;
int scale = 100;
int delta = 0;
int ddepth = CV_16S;

// Default values
float resolution=.05;//.05;
float vel_angolare_max=1.0; //  1.3;
float vel_lineare_max=0.5; // 1.2;
float range_scan_min=0.001;
float range_scan_max=30;

int distanza_saturazione_cm = 65; //65;
float grandezza_robot=.8;// .35;
int force_scale_tb=500; // 500;
int momentum_scale_tb=500; // 150;


float pixel_robot, distanza_saturazione, n_pixel_sat, force_scale, momentum_scale;

float speed;

int sizematforze=round(grandezza_robot/resolution);
cv::Mat matriceForze;

geometry_msgs::Twist joy_command_vel;
geometry_msgs::Twist command_vel;


bool laser_ready=false;

void parseCmdLine(int argc, char** argv) {
    for (int k=1; k<argc; k++) {
	    if (strcmp(argv[k],"-gui")==0)
    	    GUI = true;
	}
}


/*** Callback for retrieving the laser scan ***/
void callbackSensore(const sensor_msgs::LaserScan::ConstPtr& msg)
{	
  if (!laser_ready) {
    laser_ready=true;
    std::cout << "EmergencyStop:: laser data ready!!!" << std::endl;
  }

	size=msg->ranges.size();
	angle_min=msg->angle_min;
	angle_incr=msg->angle_increment;
	time_stamp=msg->header.stamp;
	range_scan_min=msg->range_min;
	range_scan_max=msg->range_max;
	ranges=msg->ranges;
}

/*** Callback for retrieving the joystick data ***/
void callbackJoystickInput(const geometry_msgs::Twist::ConstPtr& msg)
{	
	joy_command_vel.linear=msg->linear;
	joy_command_vel.angular=msg->angular;	
}


/*** Function for building the local view image ***/
void costruisciScanImage(){
	imm = cv::Scalar(255);
	posx=0; posy=0; indx=0; indy=0;
	cv::Point oldp(-1,-1);
	cv::Point newp;
	
	for(int i = 0; i < size; i++){
		
		if(ranges[i]>range_scan_min&&ranges[i]<range_scan_max){
			posx=ranges[i]*cos(angle_min+((float)(i)*angle_incr));
			posy=ranges[i]*sin(angle_min+((float)(i)*angle_incr));
			indy=-((posy/resolution)-h/2);
			indx=(-(posx/resolution)+w/2);

			newp.x=indy; newp.y=indx;
			if(oldp.x>=0&&oldp.y>=0&&oldp.x<w&&oldp.y<h){
				if((indx!=h/2||indy!=w/2)&&indx>=0&&indx<w && indy>=0&&indy<h&&(oldp.x!=h/2||oldp.y!=w/2)){
					cv::line(imm,oldp,newp,cv::Scalar(0));
					
				}
			}
			oldp=newp;
		}else{
			oldp.x=-1; oldp.y=-1;
		}
	}
}


/*** Function for building the local view image's distance map ***/
void costruisciDistanceImage(){
	cv::Mat imm2=imm(cv::Range(h/2-(4/resolution),h/2+(4/resolution)), cv::Range(w/2-(4/resolution),w/2+(4/resolution)));
	cv::distanceTransform( imm2, dist, distType, maskSize );
	dist *= 1.f/n_pixel_sat; //1 su n pixel per la saturazione
	cv::pow(dist, .5, dist);
	for(int i=0;i<dist.rows;i+=1){
		for(int j=0;j<dist.cols;j+=1){
			if(dist.at<float>(i,j)>1.f){
				dist.at<float>(i,j)=1.f;
			}
		}
	}	
	int size=round(grandezza_robot/resolution);
	for (int i=-size/2;i<size/2;i++){
		for (int j=-size/2;j<size/2;j++){
			imm.at<uchar>(h/2+i,w/2+j)=100;
		}
	}
}


/*** Function for building gradient map ***/
void costruisciGradientImage(){

	/// Generate grad_x and grad_y
	cv::Mat abs_grad_x, abs_grad_y;

	/// Gradient X
	cv::Scharr( dist, grad_x, ddepth, 1, 0, scale, delta, cv::BORDER_DEFAULT );
	//Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	cv::convertScaleAbs( grad_x, abs_grad_x );

	/// Gradient Y
	cv::Scharr( dist, grad_y, ddepth, 0, 1, scale, delta, cv::BORDER_DEFAULT );
	//Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	cv::convertScaleAbs( grad_y, abs_grad_y );

	/// Total Gradient (approximate)
	cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );

	
	cv::Mat grad_x_rid=grad_x(cv::Range(grad_x.rows/2-(sizematforze)/2,grad_x.rows/2+(sizematforze)/2), cv::Range(grad_x.cols/2-(sizematforze)/2,grad_x.cols/2+(sizematforze)/2));
	cv::Mat grad_y_rid=grad_y(cv::Range(grad_y.rows/2-(sizematforze)/2,grad_y.rows/2+(sizematforze)/2), cv::Range(grad_y.cols/2-(sizematforze)/2,grad_y.cols/2+(sizematforze)/2));
	
	/// costruzione matrice forze
	for(int i=0;i<sizematforze;i++){
		for(int j=0;j<sizematforze;j++){
			matriceForze.at<cv::Vec3f>(i,j)=cv::Vec3f(grad_x_rid.at<short>(i,j),grad_y_rid.at<short>(i,j),0);
		}
	}
	cv::Mat gradrid=grad(cv::Range(grad.rows/2-(sizematforze)/2,grad.rows/2+(sizematforze)/2), cv::Range(grad.cols/2-(sizematforze)/2,grad.cols/2+(sizematforze)/2));	
	cv::resize(gradrid, robot_grad, cv::Size(h/2,w/2), 0, 0,cv::INTER_LINEAR);
	cv::resize(dist, dist_ridotta, cv::Size(h/2,w/2), 0, 0,cv::INTER_LINEAR);
	dist_ridotta*=255;
	dist_ridotta.convertTo(dist_ridotta,CV_8UC1);
}


/*** Function used for the visualization of the command velocity in the GUI ***/
void costruisciImmagineAssi(cv::Mat& imm, float joyspeed, float joyangular){
	imm=cv::Scalar(255);
	int lungh=imm.rows/2-10;
	
	imm(cv::Range(0,imm.rows/2-10),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(0);
	imm(cv::Range(imm.rows/2+10,imm.rows),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(0);
	imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(0,imm.cols/2-10))=cv::Scalar(0);
	imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(imm.cols/2+10,imm.cols))=cv::Scalar(0);

	if(joyspeed>0){
		if (joyspeed>vel_lineare_max){
			imm(cv::Range(0,imm.rows/2-10),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(-(lungh/vel_lineare_max)*joyspeed+lungh,imm.rows/2-11),cv::Range(imm.cols/2-4,imm.cols/2+4))=cv::Scalar(150);
	}else{
		if (joyspeed<-vel_lineare_max){
			imm(cv::Range(imm.rows/2+10,imm.rows),cv::Range(imm.cols/2-5,imm.cols/2+5))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(imm.rows/2+11,-((imm.rows/2-11)/vel_lineare_max)*joyspeed+imm.rows/2+11),cv::Range(imm.cols/2-4,imm.cols/2+4))=cv::Scalar(150);
	}
	if(joyangular>0){
		if (joyangular>vel_angolare_max){
			imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(0,imm.cols/2-10))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(imm.rows/2-4,imm.rows/2+4),cv::Range(-(((imm.cols)/2-10)/vel_angolare_max)*joyangular+imm.cols/2-10,imm.cols/2-10))=cv::Scalar(150);
	}else{
		if (joyangular<-vel_angolare_max){
			imm(cv::Range(imm.rows/2-5,imm.rows/2+5),cv::Range(imm.cols/2+10,imm.cols))=cv::Scalar(150);
			return;
		}
		imm(cv::Range(imm.rows/2-4,imm.rows/2+4),cv::Range(imm.cols/2+10,-((imm.cols/2-11)/vel_angolare_max)*joyangular+imm.cols/2+11))=cv::Scalar(150);
	}
}


void onTrackbarSaturazione( int, void* ){
	distanza_saturazione=distanza_saturazione_cm/100.f;
	n_pixel_sat=(distanza_saturazione)/resolution;
}

void onTrackbarForceScaling( int, void* ){
	force_scale=(force_scale_tb/1000.f)/(pixel_robot/2);
}

void onTrackbarMomentumScaling( int, void* ){
	momentum_scale=(momentum_scale_tb/1000.f)/(pixel_robot/2);
}


/*** Building the GUI ***/
void creaGUI(cv::Mat& imm1, cv::Mat& imm2, cv::Mat& imm3, cv::Mat& imm4, cv::Mat& imm5, cv::Mat& immris){
	immris=cv::Scalar(200);
	for (int i=0;i<imm1.rows;i++){
		for(int j=0;j<imm1.cols;j++){
			immris.at<uchar>(i,j)=imm1.at<uchar>(i,j);
		}
	}
	for (int i=0;i<imm2.rows-1;i++){
		for(int j=0;j<imm2.cols;j++){
			immris.at<uchar>(i,j+imm1.cols+2)=imm2.at<uchar>(i,j);
		}
	}
	for (int i=1;i<imm3.rows;i++){
		for(int j=0;j<imm3.cols;j++){
			immris.at<uchar>(i+imm2.rows,j+imm1.cols+2)=imm3.at<uchar>(i,j);
		}
	}
	for (int i=0;i<imm4.rows-1;i++){
		for(int j=0;j<imm4.cols;j++){
			immris.at<uchar>(i,j+imm1.cols+2+imm2.cols+2)=imm4.at<uchar>(i,j);
		}
	}
	for (int i=1;i<imm5.rows;i++){
		for(int j=0;j<imm5.cols;j++){
			immris.at<uchar>(i+imm4.rows,j+imm1.cols+2+imm2.cols+2)=imm5.at<uchar>(i,j);
		}
	}
}


void calcolaMomentoeForza(cv::Mat& forze, cv::Vec3f& momento, cv::Vec3f& forza){
	cv::Vec3f momtemp(0,0,0);
	cv::Vec3f forzatemp(0,0,0);
	cv::Vec3f f(0,0,0);
	cv::Vec3f b(0,0,0);
	for (int i=0;i<forze.cols;i++){
		for (int j=0; j<forze.rows;j++){
			f=forze.at<cv::Vec3f>(j,i);
			if(f!=cv::Vec3f(0,0,0)&&(i!=forze.rows/2 && j!=forze.rows/2)){
				if(j<=forze.rows/2){
					b[0]=-(-i+forze.rows/2); b[1]=-(j-forze.rows/2);
					momtemp+=b.cross(f);
				}
			}
			if(f!=cv::Vec3f(0,0,0)){
				forzatemp[0]+=f[0]; forzatemp[1]+=f[1];
			}
		}
	}
	momento=momtemp;
	forza=forzatemp;
} 


int main(int argc, char **argv)
{
  	ros::init(argc, argv, "emergencyStop");
  	ros::NodeHandle n;

	parseCmdLine(argc, argv);

    std::string robotname = "robot_0";
    if (ros::param::get("/robotname", robotname)) {
        ROS_INFO_STREAM("EmergencyStop:: robotname: " << robotname);

        if (robotname=="segway") {        
            grandezza_robot=.8;
            vel_angolare_max=1.3;
            vel_lineare_max=1.2;
            distanza_saturazione_cm=65;
            force_scale_tb=500;
            momentum_scale_tb=150;
        }
        else if (robotname=="sapienzbot") {
            grandezza_robot=.8;
            vel_angolare_max=1.0;
            vel_lineare_max=0.5;
            distanza_saturazione_cm=65;
            force_scale_tb=500;
            momentum_scale_tb=500;
        }
        else if (robotname.substr(0,6)=="turtle") {
            grandezza_robot=.8;
            vel_angolare_max=1.0;
            vel_lineare_max=0.5;
            distanza_saturazione_cm=65;
            force_scale_tb=500;
            momentum_scale_tb=500;
        }
        else if (robotname.substr(0,5)=="robot") {
            grandezza_robot=.8;
            vel_angolare_max=1.0;
            vel_lineare_max=0.5;
            distanza_saturazione_cm=50;
            force_scale_tb=100;
            momentum_scale_tb=100;
        }
    }

    pixel_robot=(grandezza_robot/resolution)*(grandezza_robot/resolution);
    distanza_saturazione=distanza_saturazione_cm/100.f;
    n_pixel_sat=(distanza_saturazione)/resolution;
    force_scale=(force_scale_tb/1000.f)/(pixel_robot/2);
    momentum_scale=(momentum_scale_tb/1000.f)/(pixel_robot/2);

	sizematforze=round(grandezza_robot/resolution);
	matriceForze=cv::Mat(sizematforze,sizematforze,CV_32FC3);


	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	
	ros::Subscriber sub3 = n.subscribe("desired_cmd_vel", 1, callbackJoystickInput);

	ros::Subscriber sub = n.subscribe("scan", 1, callbackSensore);

    // GUI parametro ROS
    
  if (GUI) {
	  cv::namedWindow("GUI", 1);
      // parametri ROS - letti nel ciclo
	  cv::createTrackbar("Obstacles distance influence (cm)", "GUI", &distanza_saturazione_cm, 200, onTrackbarSaturazione, 0);
	  cv::createTrackbar("Repulsive Force Scale", "GUI", &force_scale_tb, 2000, onTrackbarForceScaling, 0);
	  cv::createTrackbar("Repulsive Momentum Scale", "GUI", &momentum_scale_tb, 2000, onTrackbarMomentumScaling, 0);
  }

    // parametro ROS
	int fps=100;
	ros::Rate loop_rate(fps);
	ros::AsyncSpinner spinner(8); // n threads
	spinner.start();

	float repulsive_linear_acc=0;
	float repulsive_angular_acc=0;
	cv::Vec3f forza;
	cv::Vec3f momento;

  while (!laser_ready) {
    loop_rate.sleep();
    ROS_WARN_STREAM("[EmergencyStop]: waiting for laser...");
  }

  double current_linear_vel=0;
  double target_linear_vel=0;
  double current_ang_vel=0;
  double target_ang_vel=0;

	while(n.ok()){
		/*** Building the force field ***/
		costruisciScanImage();
		costruisciDistanceImage();
		costruisciGradientImage();
		/********************************/

		/*** Compute the velocity command and publish it ********************************/
		repulsive_linear_acc=0;
		repulsive_angular_acc=0;				
			
		command_vel=joy_command_vel;
        target_linear_vel=command_vel.linear.x;
		target_ang_vel=command_vel.angular.z;
		speed=command_vel.linear.x;
		if(speed!=0){
			calcolaMomentoeForza(matriceForze,momento,forza);
			repulsive_linear_acc=forza[1];		
			repulsive_angular_acc=momento[2];		
			if(speed>0&&forza[1]>0){	
				target_linear_vel-=force_scale*repulsive_linear_acc*.01;
				target_ang_vel+=momentum_scale*repulsive_angular_acc*.01;
			}else if(speed<0&&forza[1]<0){
				target_linear_vel-=force_scale*repulsive_linear_acc*.01;
				target_ang_vel-=momentum_scale*repulsive_angular_acc*.01;
			}
		}	
		
		if(target_ang_vel>vel_angolare_max){
			target_ang_vel=vel_angolare_max;
		}
		if(target_ang_vel<-vel_angolare_max){
			target_ang_vel=-vel_angolare_max;
		}	
		if (target_linear_vel*speed<0){
			target_linear_vel=0;
		}
		if(target_linear_vel>vel_lineare_max){
			target_linear_vel=vel_lineare_max;
		}	
		if(target_linear_vel<-vel_lineare_max){
			target_linear_vel=-vel_lineare_max;
		}


    std::string esparam; int iesparam;
    if (ros::param::get("emergency_stop", esparam))
    {
      if (esparam=="1") {
          target_linear_vel=0; target_ang_vel=0;
          //std::cout << "Emergency Stop param: " << esparam << std::endl;
      }
    }
    else if (ros::param::get("emergency_stop", iesparam))
    {
      if (iesparam==1) {
          target_linear_vel=0; target_ang_vel=0;
          //std::cout << "Emergency Stop param: " << iesparam << std::endl;
      }
    }


    current_linear_vel = target_linear_vel;
    current_ang_vel = target_ang_vel;

    command_vel.linear.x = current_linear_vel;

    command_vel.angular.z = current_ang_vel;

    pub.publish(command_vel);

    // std::cout << "send cmd_vel ... " << command_vel << std::endl;
		/********************************************************************************/

    if (GUI) {
		  /**** Create the GUI ************************************************************/	
		  costruisciImmagineAssi(visual_joy1,speed,target_ang_vel);
		  costruisciImmagineAssi(visual_joy2,command_vel.linear.x,command_vel.angular.z);
		  creaGUI(imm,dist_ridotta,robot_grad,visual_joy1,visual_joy2,immTot);

		  cv::imshow("GUI",immTot);
		  cv::waitKey(1000/fps);
		  /********************************************************************************/
    }

		loop_rate.sleep();
	}
	spinner.stop();
	return 0;
}



