#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>
#include <assert.h>
#include <stdlib.h>

using namespace std;

float error_pre;
int flag=1;
int n=1;
float error=0,error1=0,error2=0,ErroI=0,thetai=0,thetad=0,errort=0,thetad_p=0,error1_pre=0,error2_pre=0, derror1=0, derror2=0;

/*Read time*/
 std::ofstream mafile;
 nav_msgs::Odometry msgs;

 ros::Publisher _pub;
 ros::Subscriber _sub; 


 void callback(const nav_msgs::Odometry &msgs)
        {
          
        float yd=4;
        float xd=0;
        double theta;
        float kit=.0001,kdt=0,kpt=.1;
        float ki=.10,kd=0.0,kp=1;

        int v=1;
                       
        tf::Quaternion q(msgs.pose.pose.orientation.x,msgs.pose.pose.orientation.y,msgs.pose.pose.orientation.z,msgs.pose.pose.orientation.w);

        tf::Matrix3x3 m(q);
        double roll,pitch,yaw;
        m.getRPY(roll,pitch,yaw);
        
        float yc,xc,I=0,D=0;
        int i=1;
        xc=msgs.pose.pose.position.x;
        yc=msgs.pose.pose.position.y;
        theta=atan2((yd-yc),(xd-xc));
        geometry_msgs::Twist msg;
        geometry_msgs::Pose pose_msg;
       
        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;
        msg.angular.x=0;
        msg.angular.y=0;
        msg.angular.z=0;

        _pub.publish(msg);

        error1 =yd-yc;
        error2 =xd-xc;

        error=sqrt(error1*error1+error2*error2);
        D=error-error_pre;
        I+=error;
		
        errort=(theta-yaw);
        thetai+=errort;
        thetad=errort-thetad_p;
		
        ros::Time last_request = ros::Time::now();

        while(ros::ok && i)
        {
    if(!((errort<.01) && (errort>-.01))&& error>.1 )
                {
                        ROS_INFO_STREAM("sendind  "<<yaw<<"THETA"<<theta<<"    error"<<errort);
                        mafile << ros::Time::now().toSec() - last_request.toSec()<< "\t" << "\t" <<errort << "\t" << "\t" << yaw << "\n" ; 
                                    mafile << "\n";
                        
						msg.linear.x=0;
	                    msg.linear.y=0;
			            msg.angular.z=kpt*errort+kit*thetai+kdt*thetad;
     		
                       _pub.publish(msg);            
                } 
    else if((error >.01))
                {
										
        ROS_INFO_STREAM("Linear X =  " << msg.linear.x  << "Linear y =  "<<   msg.linear.y);          
        ROS_INFO_STREAM("hello"<<"  x"<<msgs.pose.pose.position.x<<"  y"<<msgs.pose.pose.position.y<<"error"<<error<<"  uo   "<<u[0]<<"  u1   "<<u[1]);
                    
		msg.linear.x= (kp*error + kd*D + ki*I)*sin(yaw);     
        msg.linear.y =(kp*error + kd*D + ki*I)*cos(yaw);      
        msg.angular.z=0;

        mafile << "\t" << "\t" << ros::Time::now().toSec() - last_request.toSec()<< "\t" << "\t" <<msg.linear.x << "\t" << "\t" << msg.linear.y << "\n" ; 
                                    mafile << "\n";
                    
                  _pub.publish(msg);
                }
    else if(error<.01)
                {
                
               msg.angular.z=0;        
               msg.linear.x=0;                
               msg.linear.y=0; 
                            
              _pub.publish(msg);
                }

        i=i-1;error_pre=error;thetad_p=errort;
			        //myfile.close(); 
        }   
 }
        
int main(int argc, char **argv)
{

      ros::init(argc,argv,"goal_PID");
      ros::NodeHandle nh;
      _pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 100);   
      _sub=nh.subscribe("RosAria/pose",100,&callback);

     std::string path="/home/pioneer/";
     //std::ofstream mafile;
     path.append("text_");
     path.append(".txt");
     mafile.open(path.c_str()); 
                   
     ros::spin();
     return 0;
}




