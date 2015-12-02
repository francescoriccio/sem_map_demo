#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <rococo_navigation/FollowCorridorAction.h>

#define RAD(a) ((a)/180.0*M_PI)
#define DEG(a) ((a)*180.0/M_PI)


// defined in robotpose.cpp
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);


inline double norm180(double a) {
  while (a>180) a = a-360;
  while (a<=-180) a = a+360;
  return a;
}

inline double normPI(double a) {
  while (a>M_PI) a = a-2*M_PI;
  while (a<=-M_PI) a = a+2*M_PI;
  return a;
}

inline double norm360(double a) {
  while (a>=360) a = a-360;
  while (a<=-360) a = a+360;
  return a;
}

// round angle to 0, PI/2, -PI/2, PI
inline double roundPI2(double a) {
    if ((a>=-M_PI/4 && a<=M_PI/4) || (a>=7*M_PI/4 && a<=2*M_PI))
        return 0;
    else if (a>=M_PI/4 && a<=3*M_PI/4)
        return M_PI/2;
    else if ((a>=3*M_PI/4 && a<=5*M_PI/4) || (a>=-M_PI && a<=-3*M_PI/4))
        return M_PI;
    else if ((a>=5*M_PI/4 && a<=7*M_PI/4) || (a>=-3*M_PI/4 && a<=-M_PI/4))
        return -M_PI/2;
}	
	
void doFollowCorridor(std::string robot_name, ros::Publisher &cmd_vel_pub, double GX, double GY, double max_vel, bool *run)
{
    ROS_INFO_STREAM("Action follow corridor: executing ...");

    geometry_msgs::Twist cmd;
    cmd.linear.x = 0; cmd.linear.y = 0; 

    double x,y,th_rad,th_deg,target_angle;
    double max_ang_vel = RAD(15);
    double min_vel = 0.2;

    while (*run && !getRobotPose(robot_name,x,y,th_rad)) {
      ROS_ERROR_STREAM("Cannot read robot pose!!!");
      ros::Duration(0.25).sleep();
    }
    th_deg=DEG(th_rad);

    double a = atan2(GY-y,GX-x);
    printf("atan = %.3f\n",a);
    target_angle = roundPI2(a); // rad
    double GTh = DEG(target_angle); // deg
  
    bool goN=false,goE=false,goS=false,goW=false;
    double limX=1.0, limY=1.0;
    if (GTh==0 || GTh==180) { 
      limX=0.10; limY=2.0;
      if (x < GX) goE = true; else goW = true;
    }
    else if (GTh==90 or GTh==-90) {
      limX=2.0; limY=0.10;
      if (y < GY) goN = true; else goS = true;
    }
    
    printf("Robot pose: %.1f %.1f %.3f - Target: %.1f %.1f %.1f\n",x,y,th_rad,GX,GY,GTh);
    printf("DIR: %s - Limits: %.1f %.1f \n",
	   goN?"N":(goS?"S":(goE?"E":(goW?"W":"?"))), limX,limY);

    double Kp=2.0, adist=0.0;
    
    while (*run && ((goN && (y<GY-limY)) || (goS && (y>GY+limY)) || (goE && (x<GX-limX)) || (goW && (x>GX+limX))) ) {
	cmd.linear.x = max_vel;
	int d=10;
	// check the limits (different for N/S and E/W directions)
	if ( ((goN || goS) && (fabs(y-GY)<d*limY)) || ((goE || goW) && (fabs(x-GX)<d*limX)) ) {
	    
	    double dd = 1.0 - ((goN || goS)?((d*limY-fabs(y-GY))/(d*limY)):((d*limX-fabs(x-GX))/(d*limX))); // distance to the goal
	    cmd.linear.x = min_vel + dd * (max_vel-min_vel); // slow down when close to target
	    //printf("     -- x %.3f y %.3f - dd %.2f vel %.2f \n",x,y,dd,cmd.linear.x);
	    //printf("     == %.3f / %.3f \n",d*limX-fabs(x-GX),d*limX);
	}
        if (getRobotPose(robot_name,x,y,th_rad))
          adist = normPI(target_angle-th_rad); // rad
        else
          adist -= max_ang_vel/10; // just in case slow down angular vel

	if (fabs(adist)>RAD(30)) // first only turn towards the goal
	  cmd.linear.x = 0;
	
	cmd.angular.z = std::min(Kp*adist, max_ang_vel);
        // printf("  -- Robot pose: %.1f %.1f %.3f - Target: %.1f %.1f %.1f - adist: %.3f - ang z: %.3f - lim %.1f %.1f\n", x,y,th_rad,GX,GY,GTh,adist,cmd.angular.z,limX,limY);

        cmd_vel_pub.publish(cmd);
        ros::Duration(0.1).sleep(); // wait ...    
    }

    cmd.linear.x = 0;
    cmd.angular.z = 0;
    cmd_vel_pub.publish(cmd);
    ros::Duration(0.1).sleep(); // wait ...    

    ROS_INFO("Action follow corridor: finished [run=%d]",*run);
    //if (*run)
    //  *run = false;
}

// ********************    S E R V E R    ***********************


typedef actionlib::SimpleActionServer<rococo_navigation::FollowCorridorAction> FCAS;
//actionlib::ActionServer<pnp_msgs::PNPAction> PNPAS;

class FollowCorridorActionServer {

protected:

    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub;
    std::string action_name, robot_name;
    boost::mutex state_mutex;
    
    FCAS fc_server;

public:

    FollowCorridorActionServer(std::string _action_name, std::string _robot_name) : 
        action_name(_action_name), robot_name(_robot_name),
        fc_server(nh, action_name, boost::bind(&FollowCorridorActionServer::executeCB, this, _1), false)
    { 
        //set up the publisher for the cmd_vel topic
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    ~FollowCorridorActionServer() { }

    void start() {
        // Start the server
        fc_server.start();
        ROS_INFO_STREAM("FollowCorridor Action Server started!!!"); 
    }

    void executeCB(const rococo_navigation::FollowCorridorGoalConstPtr& goal)  {   
      // Run action (wait until it finishes)
      ROS_INFO_STREAM("Starting action: robot " << robot_name << " follow corridor " << 
            goal->target_X << " " << goal->target_Y << " " << goal->max_vel);
      
      bool run = true;

      //doFollowCorridor(robot_name,cmd_vel_pub,goal->target_X, goal->target_Y, goal->max_vel);

      // start in a new thread
      boost::thread t(
            boost::bind(&doFollowCorridor, _1, _2, _3, _4, _5, _6),
            robot_name,cmd_vel_pub,goal->target_X, goal->target_Y, goal->max_vel, &run);
       

      boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
      
      while (run && !t.timed_join(boost::posix_time::milliseconds(500))) {
	
	if (fc_server.isPreemptRequested() || !ros::ok()) {
	  ROS_INFO("Request to cancel the goal.");
	  run = false;
	}
	
      }
      
      ROS_INFO("Action finished.");
      // Set result
      if (run)
	fc_server.setSucceeded();
      else
	fc_server.setAborted();

      // wait for actual delivery...
      ros::Duration(1).sleep();

    }

};


// ********************    C L I E N T    ***********************

void fc_action_client(std::string robotname, double GX, double GY) {

    // Set fc topic
    std::string fc_topic = "/"+robotname+"/fc";

    // Define the action client (true: we want to spin a thread)
    actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction> ac(fc_topic, true);  

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for follow corridor action server to come up");
    }

    // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
    ac.cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec

    // Set the goal
    rococo_navigation::FollowCorridorGoal goal;
    goal.target_X = GX;  goal.target_Y = GY;   // goal
    goal.max_vel = 0.7;  // m/s

    // Send the goal
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

	  // Wait for termination
    while (!ac.waitForResult(ros::Duration(1.0))) {
	    ROS_INFO_STREAM("Running... [" << ac.getState().toString() << "]");
    }
    ROS_INFO_STREAM("Finished [" << ac.getState().toString() << "]");

    // Print result
    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("FollowCorridor successful");
    else
        ROS_INFO("FollowCorridor failed");

    // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
    ac.cancelAllGoals(); ros::Duration(1).sleep(); // wait 1 sec

}


void fc_action_client_cancel_goal(std::string robotname) {
// Set fc topic
    std::string fc_topic = "/"+robotname+"/fc";

    // Define the action client (true: we want to spin a thread)
    actionlib::SimpleActionClient<rococo_navigation::FollowCorridorAction> ac(fc_topic, true);  

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for follow corridor action server to come up");
    }

    // Cancel the goal
    ROS_INFO("Cancel goal");
    ac.cancelAllGoals();  ros::Duration(1).sleep(); // wait 1 sec
    
}

// **********************    M A I N    *************************


int main(int argc, char** argv)  {

  if ( (argc<2) || ((std::string(argv[1])=="-client") && (argc<4)) ) {
  	std::cout << "Use: " << argv[0] << " -server <robotname>" << std::endl; 
  	std::cout << "  or " << argv[0] << " -client <robotname> [<X> <Y>|stop] " << std::endl; 
    exit(-1);
  }

  if (std::string(argv[1])=="-server") {
      std::string robotname = std::string(argv[2]);
      // Init ROS node
      ros::init(argc, argv, "fc_action_server");
      // Start action server  
      FollowCorridorActionServer server("fc",robotname);
      server.start();
      ros::spin();
  }
  else if (std::string(argv[1])=="-client") {
      // Read args
      std::string robotname = std::string(argv[2]);
      if (std::string(argv[3])=="stop") {
	// Init ROS node
	std::ostringstream ss;
	ss << "fc_" << robotname << "_stop";
	std::string nodename = ss.str();
	ros::init(argc, argv, nodename);
	// Start client
	fc_action_client_cancel_goal(robotname);
      }
      else {
	double GX = atof(argv[3]), GY = atof(argv[4]);
	// Init ROS node
	std::ostringstream ss;
	ss << "fc_" << robotname << "_" << fabs(GX) << "_" << fabs(GY);
	std::string nodename = ss.str();
	ros::init(argc, argv, nodename);
	// Start client
	fc_action_client(robotname,GX,GY);
      }
  }
  return 0;
}

