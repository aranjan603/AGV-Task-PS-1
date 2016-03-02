#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#define max 200
#define threshold 50

using namespace cv;
using namespace std;

int visited[max][max];
int COM[2][max];
Mat binary;
int valr,valg;
int flag=0;

void print(Mat img, String windowname)
 {
   namedWindow(windowname, CV_WINDOW_AUTOSIZE);
   imshow(windowname, img);
 }


void find(Mat img)
 {
   int cr=0,cg=0;
   for (int i=0;i<img.rows && (cr==0 || cg==0);i++)
     {
        for (int j=0;j<img.cols && (cr==0 || cg==0);j++)
           {
              if (cr==0 && img.at<Vec3b>(i, j)[0]<40 && img.at<Vec3b>(i, j)[1] <40 && img.at<Vec3b>(i, j)[2]>200 )
                 { 
                    cr++;valr=visited[i][j];
                 }
              if (cg==0 && img.at<Vec3b>(i, j)[0] <20 && img.at<Vec3b>(i, j)[1] >220 && img.at<Vec3b>(i, j)[2]<20 )
                 {
                    cg++;valg=visited[i][j];
                 }
            }
      }
 }


Mat Bin(Mat gray)
 {
   Mat binary1(gray.rows, gray.cols, CV_8UC1, 255);
   for (int i = 0; i < gray.rows; i++)
    {
      for (int j = 0; j < gray.cols; j++)
       {
          if ((int)(gray.at<uchar>(i, j)) > threshold)
                binary1.at<uchar>(i, j) = 255;
          else
                binary1.at<char>(i, j) = 0;
       }
    }   
   return binary1;
 }


void initialize()
 {
   for (int i = 0; i < max; i++)
     {
        for (int j = 0; j < max; j++)
	   {
		visited[i][j] = max;
		if (i<2)
		    COM[i][j] = 0;
	   }
      }
  }


int isvalid(Mat img, int a, int b)
 {
    if (a < 0 || b < 0 || a >= img.rows || b >= img.cols)
        return 0;
    else
        return 1;
 }


void DFSvisit(int i, int j, int count)
 {
    visited[i][j] = count;
    for (int k = i - 1; k <= i + 1; k++)
       {
           for (int l = j - 1; l <= j + 1; l++)
              {
                  if (isvalid(binary, k, l) == 1)
                     {
                         if ((visited[k][l] == max) && ((binary.at<uchar>(k, l)) == 255))
                               DFSvisit(k, l, count);
                     }
              }
       }
 }


int DFSutil()
 {
    int count = 1;
    for (int i = 0; i < binary.rows; i++)
       {
          for (int j = 0; j < binary.cols; j++)
             {
               if ((visited[i][j]==max) && ((binary.at<uchar>(i, j)) == 255))
                    {
                       DFSvisit(i, j, count);
                       count++;
                    }
             }
       }
    for (int i = 0; i < binary.rows; i++)
      {
         for (int j = 0; j < binary.cols; j++)
            {
               if (visited[i][j] == max)
                      visited[i][j] = 0;
            }
       }
    return count;
 }


void centre(int count)
 {
    int single[max];
    for (int i = 1; i < max; i++)
	   single[i] = 0;
    for (int i = 0; i < binary.rows; i++)
	{
	    for (int j = 0; j < binary.cols; j++)
		 {
		    if (visited[i][j] != 0)
			{
				int t = visited[i][j];
				COM[0][t] += j;
				COM[1][t] += i;
				single[t]++;
			}
		 }
	}
    for (int i = 0; i < 2; i++)
	{
	     for (int j = 1; j < count; j++)
		{
		    COM[i][j] /= single[j];
		}
	}
 }


void DFSvisitnew(int i, int j)
 {
    visited[i][j]=-1;
    int k,l;
    for (k = i+1 ; k >= i -1 && flag<10; k--)
       {
           for (l = j+1 ; l >=j-1 && flag<10; l--)
              {
                  if (isvalid(binary, k, l) == 1  )
                     {
                         if (visited[k][l]==1 || visited[k][l]==0 || visited[k][l]==10)
                            {
                               if (visited[k][l]==10) flag++;
                               if (k>COM[0][valg] || l>COM[1][valg]) continue;
                               DFSvisitnew(k, l);
                            }
                     }
              }   
       }
 }



void DFSutilnew()
 {
    int i,j;
    for (i = COM[0][valr]; i <=COM[0][valg] && flag<10; i++)
       {
          for (j = COM[1][valr]; j <=COM[1][valg] && flag<10; j++)
             {
               if ( visited[i][j]==1 || visited[i][j]==0 || visited[i][j]==10 )
                    {
                       if (i==COM[0][valg] && j==COM[1][valg]) flag++;
                       if (visited[i][j]==10) flag++;
                       DFSvisitnew(i, j);
                    }
             }   
       }
 }




int main(int argc, char **argv)
 {
   
   Mat img = imread("/home/ashish/catkin_ws/src/beginner_tutorials/src/ps1.jpg",1);
   Mat gray=imread("ps1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
   binary = Bin(gray);
   initialize();
   int count=DFSutil();
   centre(count);
   find(img);
   
   DFSutilnew();
   int i,j;
   
   for (i = 0; i <max; i++)
       {
          for (j =0; j < max; j++)
             {
               if (visited[i][j]==-1) { img.at<Vec3b>(i,j)[0]=255; img.at<Vec3b>(i,j)[1]=0; img.at<Vec3b>(i,j)[2]=0;}    
             }
        }  
   
   ros::init(argc, argv, "talker");
   ros::NodeHandle n;
   ros::Publisher chatter_pub = n.advertise<geometry_msgs::Point>("chatter", 1000);
   ros::Rate loop_rate(9);


 
   for (i=0;i<max;i++)
     {
        for (j=0;j<max;j++)
           {
               if (visited[i][j]==-1)
                   {
                       geometry_msgs::Point pnt;
                       pnt.x=i;
                       pnt.y=j;
                       cout<<pnt.x<< " "<<pnt.y<<endl;
                       chatter_pub.publish(pnt);
                       ros::spinOnce();
                       loop_rate.sleep();
                   }
           }
     }
   int iKey = waitKey(50);
   waitKey(0);
   return 0;
 }


