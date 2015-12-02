#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19



sensor_msgs::Joy joy;
geometry_msgs::Twist vel;
ros::Publisher pub;

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
	vel.linear.x=msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
	vel.angular.z=msg->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS]*1.57;
	pub.publish(vel);
}


int main(int argc, char **argv)
{

  	ros::init(argc, argv, "joystickPS3");


  	ros::NodeHandle n;

	pub = n.advertise<geometry_msgs::Twist>("ps3joy_cmd_vel", 1);
	
	ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("/joy", 1, joyCallback);
	//ros::spin();

	int fps=100;
	ros::Rate loop_rate(fps);
	ros::AsyncSpinner spinner(1); // n threads
	spinner.start();
	while(n.ok()){
			
		pub.publish(vel);
		printf("lin= %f ang= %f\n",vel.linear.x,vel.angular.z);
		loop_rate.sleep();
	}


	return 0;
}




