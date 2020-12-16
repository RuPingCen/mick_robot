/**
* reference from https://www.guyuehome.com/253
* 
* 读取键盘 i j k l  按键值 作为X方向和Z方向的速度值 发布到/cmd_vel 话题上
*
* @data 2020-12-15
*/ 
#include <termios.h>  
#include <signal.h>  
#include <math.h>  
#include <stdio.h>  
#include <stdlib.h>  
#include <sys/poll.h>  
  
#include <boost/thread/thread.hpp>  
#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  
  
#define KEYCODE_I 0x69  
#define KEYCODE_J 0x6a  
#define KEYCODE_K 0x6b  
#define KEYCODE_L 0x6c  

#define KEYCODE_I_CAP 0x57 
#define KEYCODE_J_CAP 0x4a 
#define KEYCODE_K_CAP 0x4b  
#define KEYCODE_L_CAP 0x4c 

 
  
class SmartCarKeyboardTeleopNode  
{  
    private:  
        double walk_vel_;  
        double run_vel_;  
        double yaw_rate_;  
        double yaw_rate_run_;  
          
        geometry_msgs::Twist cmdvel_;  
        ros::NodeHandle n_;  
        ros::Publisher pub_;  
  
    public:  
        SmartCarKeyboardTeleopNode()  
        {  
            pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  
              
            ros::NodeHandle n_private("~");  
            n_private.param("walk_vel", walk_vel_, 0.5);  
            n_private.param("run_vel", run_vel_, 1.0);  
            n_private.param("yaw_rate", yaw_rate_, 1.0);  
            n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);  
        }  
          
        ~SmartCarKeyboardTeleopNode() { }  
        void keyboardLoop();  
          
        void stopRobot()  
        {  
            cmdvel_.linear.x = 0.0;  
            cmdvel_.angular.z = 0.0;  
            pub_.publish(cmdvel_);  
        }  
};  
  
SmartCarKeyboardTeleopNode* tbk;  
int kfd = 0;  
struct termios cooked, raw;  
bool done;  
  
int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"keyboard", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
    SmartCarKeyboardTeleopNode tbk;  
      
    boost::thread t = boost::thread(boost::bind(&SmartCarKeyboardTeleopNode::keyboardLoop, &tbk));  
      
    ros::spin();  
      
    t.interrupt();  
    t.join();  
    tbk.stopRobot();  
    tcsetattr(kfd, TCSANOW, &cooked);  
      
    return(0);  
}  
  
void SmartCarKeyboardTeleopNode::keyboardLoop()  
{  
    char c;  
    double max_tv = walk_vel_;  
    double max_rv = yaw_rate_;  
    bool dirty = false;  
    int speed = 0;  
    int turn = 0;  
      
    // get the console in raw mode  
    tcgetattr(kfd, &cooked);  
    memcpy(&raw, &cooked, sizeof(struct termios));  
    raw.c_lflag &=~ (ICANON | ECHO);  
    raw.c_cc[VEOL] = 1;  
    raw.c_cc[VEOF] = 2;  
    tcsetattr(kfd, TCSANOW, &raw);  
      
    puts("Reading from keyboard");  
    puts("Use I J K L  keys to control the robot");  
    puts("Press Shift to move faster");  
      
    struct pollfd ufd;  
    ufd.fd = kfd;  
    ufd.events = POLLIN;  
      
    for(;;)  
    {  
        boost::this_thread::interruption_point();  
          
        // get the next event from the keyboard  
        int num;  
          
        if ((num = poll(&ufd, 1, 250)) < 0)  
        {  
            perror("poll():");  
            return;  
        }  
        else if(num > 0)  
        {  
            if(read(kfd, &c, 1) < 0)  
            {  
                perror("read():");  
                return;  
            }  
        }  
        else  
        {  
            if (dirty == true)  
            {  
                stopRobot();  
                dirty = false;  
            }  
              
            continue;  
        }  
        //std::cout<<"c: "<<std::hex<<(c&0xff)<<std::endl;  
        switch(c)  
        {  
            case KEYCODE_I:  
                max_tv = walk_vel_;  
                speed = 1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_K:  
                max_tv = walk_vel_;  
                speed = -1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_J:  
                max_rv = yaw_rate_;  
                speed = 0;  
                turn = 1;  
                dirty = true;  
                break;  
            case KEYCODE_L:  
                max_rv = yaw_rate_;  
                speed = 0;  
                turn = -1;  
                dirty = true;  
                break;  
                  
            case KEYCODE_I_CAP:  
                max_tv = run_vel_;  
                speed = 1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_K_CAP:  
                max_tv = run_vel_;  
                speed = -1;  
                turn = 0;  
                dirty = true;  
                break;  
            case KEYCODE_J_CAP:  
                max_rv = yaw_rate_run_;  
                speed = 0;  
                turn = 1;  
                dirty = true;  
                break;  
            case KEYCODE_L_CAP:  
                max_rv = yaw_rate_run_;  
                speed = 0;  
                turn = -1;  
                dirty = true;  
                break;                
            default:  
                max_tv = walk_vel_;  
                max_rv = yaw_rate_;  
                speed = 0;  
                turn = 0;  
                dirty = false;  
        }  
        cmdvel_.linear.x = speed * max_tv;  
        cmdvel_.angular.z = turn * max_rv;  
        pub_.publish(cmdvel_);  
    }  
}
