#include <ros/ros.h> 
#include <geometry_msgs/Twist.h>
 #include <sensor_msgs/Joy.h> 

using namespace std;

ros::Publisher vel_pub_;
ros::Subscriber joy_sub_; 
  
double l_scale_=0.5, a_scale_=0.5; 


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) 
{ 
	geometry_msgs::Twist twist;  
	if(joy->buttons[2])
	{
		if(joy->buttons[0])
		{
			l_scale_=0.5;
			a_scale_=0.5;
		}
		else if(joy->buttons[1])
		{
			l_scale_=1;
			a_scale_=1;
		}
		else;
		twist.angular.z = a_scale_*joy->axes[0]; 
		twist.linear.x = l_scale_*joy->axes[1]; 
	}
	else
	{
		twist.angular.z = 0; 
		twist.linear.x = 0; 
	} 

	vel_pub_.publish(twist);
	//cout<<"I recived the data: "<<twist.linear.x <<"  "<<twist.angular.z <<endl;

}



int main(int argc, char** argv) 
{ 
 	
	ros::init(argc, argv, "transfor_joy");
 	ros::NodeHandle nh_; 
	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &joyCallback); 

	ros::spin();
}

