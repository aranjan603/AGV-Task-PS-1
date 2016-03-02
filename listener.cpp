#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

using namespace cv;
using namespace std;

Mat img1 = imread("/home/ashish/catkin_ws/src/beginner_tutorials/src/ps1.jpg",1);

void changecolor(int x,int y)  
 {
   cout<<x<<" "<<y<<endl;
   img1.at<Vec3b>(x,y)[0]=255;
   img1.at<Vec3b>(x,y)[1]=0;
   img1.at<Vec3b>(x,y)[2]=0; 
   imshow("NEW",img1);
   waitKey(50);
 }


void chatterCallback(const geometry_msgs::Point::ConstPtr& pnt)  
 {                               
   changecolor(pnt->x,pnt->y);    
 }

int main(int argc, char **argv)
 { 
   cout<<"Waiting for publisher to publish message..."<<endl;
   ros::init(argc, argv, "listener");
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);   
   ros::spin();
   cout<<endl<<"ITS OVER..."<<endl;
   return 0;
}


