#include<iostream>
#include<stdlib.h>
#include<vector>
#include<time.h>
#include<math.h>
#include<fstream>

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_ros/transform_listener.h"

#include <opencv2/core/core.hpp>     // opencv libraries
#include <queue>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
using namespace cv;
using namespace std;

const float MapRes     = 0.3;                              //map resolution(m/cell)

const int MapHeightM   = 20;  //720 //72                            //map height in meters
const int MapWidthM    = 20; //1280                              //map width in meters

const int MapHeight    = MapHeightM/MapRes;                //map height in pixel
const int MapWidth     = MapWidthM/MapRes;                 //map width in pixel

const int CARCEN_X = MapWidth/2;        //center of car with respect to map
const int CARCEN_Y = MapWidth/2;        //center of car with respect to map

ros::Publisher path_pub;
geometry_msgs::TransformStamped transformStamped;

double start_x=0, start_y=0, end_x=10, end_y=10;
double end_p, end_q, end_r, end_s, end_theta, start_theta=0.0;
double start_p, start_q, start_r, start_s;

const double MinTR=1, pi=3.141592653589, numWP=100;

class state{
    public:
    double x, y, theta;
};

class path{
    public:
    double theta1, l, theta2;
    int type;
};

//lrl, rlr, lsl, lsr, rsl, rsr

void SetStart(const geometry_msgs::PoseStamped::ConstPtr& start_pose){
  start_x = (start_pose->pose.position.x);
  start_y = (start_pose->pose.position.y);
  start_p = start_pose->pose.orientation.x;
  start_q = start_pose->pose.orientation.y;
  start_r = start_pose->pose.orientation.z;
  start_s = start_pose->pose.orientation.w;
  start_theta=atan2((2.0*((start_s)*(start_r)+(start_p)*(start_q))), 1.0-(2.0*((start_q)*(start_q)+(start_r)*(start_r))));
  cout<<start_x<<" "<<start_y<<"\n";
}

void calcway(path a, vector<state> &n, state curr, state &curr1, double ix1, double iy1, double gx1, double gy1, double ix2, double iy2, double gx2, double gy2){
    //cout<<a.theta1<<" "<<a.theta2<<endl;
    double firstCirc[2];
    double secondCirc[2];
    int t1=0;
    int t2=0;
    if(a.type<20){
        firstCirc[0]=ix1;
        firstCirc[1]=iy1;
        if(a.type<10){
            secondCirc[0]=gx1;
            secondCirc[1]=gy1;
        }
        else{
            secondCirc[0]=gx2;
            secondCirc[1]=gy2;
        }
    }
    if(a.type>=20){
        firstCirc[0]=ix2;
        firstCirc[1]=iy2;
        if(a.type<30){
            secondCirc[0]=gx1;
            secondCirc[1]=gy1;
        }
        else{
            secondCirc[0]=gx2;
            secondCirc[1]=gy2;
        }
    }

    if(curr.theta>=0){
        if((atan2((curr.y-firstCirc[1]), (curr.x-firstCirc[0]))-curr.theta<pi/2+0.01)&&(atan2((curr.y-firstCirc[1]), (curr.x-firstCirc[0]))-curr.theta>pi/2-0.01)){
            if(a.theta1>=0){
            }
            else{
                if(curr.theta<0){
                    curr.theta+=pi;
                    t1++;
                }
                else{
                    curr.theta-=pi;
                    t1++;
                }
            }
        }
        else{
            if(a.theta1<=0){
            }
            else{
                if(curr.theta<0){
                    curr.theta+=pi;
                    t1++;
                }
                else{
                    curr.theta-=pi;
                    t1++;
                }
            }
        }
    }
    else{
        if((atan2((curr.y-firstCirc[1]), (curr.x-firstCirc[0]))-curr.theta>-pi/2-0.01)&&(atan2((curr.y-firstCirc[1]), (curr.x-firstCirc[0]))-curr.theta<-pi/2+0.01)){

            if(a.theta1>=0){
            }
            else{
                if(curr.theta<0){
                    curr.theta+=pi;
                    t1++;
                }
                else{
                    curr.theta-=pi;
                    t1++;

                }
            }
        }
        else{
            if(a.theta1<=0){
            }
            else{
                if(curr.theta<0){
                    curr.theta+=pi;
                    t1++;
                }
                else{
                    curr.theta-=pi;
                    t1++;
                }
            }
        }
    }

    for(int i=0; i<numWP; i++){
        state temp;
        temp.x=(curr.x)+2*abs(MinTR*(sin(a.theta1/(2*numWP))))*(cos(curr.theta-a.theta1/(2*numWP)));
        temp.y=(curr.y)+2*abs(MinTR*(sin(a.theta1/(2*numWP))))*(sin(curr.theta-a.theta1/(2*numWP)));
        if((curr.theta>=0)&&(curr.theta<=pi)){
            if((curr.theta-((a.theta1)/numWP))>pi)
                temp.theta=-2*pi+((curr.theta)-(a.theta1)/numWP);
            else{temp.theta=(curr.theta)-(a.theta1)/numWP;}
        }
        if((curr.theta<0)&&(curr.theta>=-pi)){
            if((curr.theta-((a.theta1)/numWP))<-pi)
                temp.theta=2*pi+((curr.theta)-(a.theta1)/numWP);
            else{temp.theta=(curr.theta)-(a.theta1)/numWP;}
        }
        n.push_back(temp);
        curr=temp;
    }

    if(t1==1&&abs(curr.theta-(atan2((secondCirc[1]+curr.y)/2.0, (secondCirc[0]+curr.x)/2.0)))){
        if(curr.theta>=0){
            curr.theta-=pi;
        }
        else{
            curr.theta+=pi;
        }
    }

    if((a.type)%10==0||(a.type)%10==1){
        for(int i=0; i<numWP; i++){
            state temp;
            temp.x=curr.x+((a.l)/numWP)*cos(atan2((secondCirc[1]-firstCirc[1]), (secondCirc[0]-firstCirc[0])));
            temp.y=curr.y+((a.l)/numWP)*sin(atan2((secondCirc[1]-firstCirc[1]), (secondCirc[0]-firstCirc[0])));
            temp.theta=curr.theta;
            curr=temp;
            n.push_back(temp);
        }
    }
    else{
        for(int i=0; i<numWP; i++){
            state temp;
            temp.x=curr.x+((a.l)/numWP)*cos(curr.theta);
            temp.y=curr.y+((a.l)/numWP)*sin(curr.theta);
            temp.theta=curr.theta;
            curr=temp;
            n.push_back(temp);
        }
    }

    if(curr.theta>0){
        if((atan2((curr.y-secondCirc[1]), (curr.x-secondCirc[0]))-curr.theta<pi/2+0.01)&&(atan2((curr.y-secondCirc[1]), (curr.x-secondCirc[0]))-curr.theta>pi/2-0.01)){

            if(a.theta2>=0){
            }
            else{
                curr.theta-=pi;
                t2++;
            }
        }
        else{
            if(a.theta2<=0){
            }
            else{
                curr.theta-=pi;
                t2++;
            }
        }
    }
    else{
        if((atan2((curr.y-secondCirc[1]), (curr.x-secondCirc[0]))-curr.theta>-pi/2-0.01)&&(atan2((curr.y-secondCirc[1]), (curr.x-secondCirc[0]))-curr.theta<-pi/2+0.01)){

            if(a.theta2<0){
            }
            else{
                curr.theta+=pi;
                t2++;
            }
        }
        else{
            if(a.theta2>=0){
            }
            else{
                curr.theta+=pi;
                t2++;
            }
        }
    }

    for(int i=0; i<numWP; i++){
        state temp;
        temp.x=(curr.x)+2*abs(MinTR*(sin(a.theta2/(2*numWP))))*(cos(curr.theta-a.theta2/(2*numWP)));
        temp.y=(curr.y)+2*abs(MinTR*(sin(a.theta2/(2*numWP))))*(sin(curr.theta-a.theta2/(2*numWP)));
        if((curr.theta>0)&&(curr.theta<=pi)){
            if((curr.theta-((a.theta2)/numWP))>pi)
                temp.theta=-2*pi+((curr.theta)-(a.theta2)/numWP);
            else{temp.theta=(curr.theta)-(a.theta2)/numWP;}
        }
        if((curr.theta<0)&&(curr.theta>=-pi)){
            if((curr.theta-((a.theta2)/numWP))<-pi)
                temp.theta=2*pi+((curr.theta)-(a.theta2)/numWP);
            else{temp.theta=(curr.theta)-(a.theta2)/numWP;}
        }
        n.push_back(temp);
        curr=temp;
    }

    /*if(t2==1){
        if(curr.theta>=0){
            curr.theta-=pi;
        }
        else{
            curr.theta+=pi;
        }
    }*/
    curr1=curr;
}


void SetGoal(const geometry_msgs::PoseStamped::ConstPtr& goal_pose){
    cout<<"afsg";
  end_x = (goal_pose->pose.position.x); 
  end_y = (goal_pose->pose.position.y);
  end_p = goal_pose->pose.orientation.x;
  end_q = goal_pose->pose.orientation.y;
  end_r = goal_pose->pose.orientation.z;
  end_s = goal_pose->pose.orientation.w;
  end_theta=atan2((2.0*((end_s)*(end_r)+(end_p)*(end_q))), 1.0-(2.0*((end_q)*(end_q)+(end_r)*(end_r))));
  cout<<end_x<<" "<<end_y<<" "<<end_theta<<"end";


    state init, final;
    int statetype=0;
    init.x=start_x;
    init.y=start_y;
    init.theta=start_theta;
    final.x=end_x;
    final.y=end_y;
    final.theta=end_theta;


    //DUBINS STARTS HERE
    vector<state> path_final;

    //case1
    double netlen[40];
    path set[40];
    double ix1=init.x+MinTR*cos(init.theta-pi/2), iy1=init.y+MinTR*sin(init.theta-pi/2), ix2=init.x+MinTR*cos(init.theta+pi/2), iy2=init.y+MinTR*sin(init.theta+pi/2);
    double gx1=final.x+MinTR*cos(final.theta-pi/2), gy1=final.y+MinTR*sin(final.theta-pi/2), gx2=final.x+MinTR*cos(final.theta+pi/2), gy2=final.y+MinTR*sin(final.theta+pi/2);
    //initial circle 1:
    //final circle 1(direct, direct, transverse, transverse)
    set[0].theta1=((init.theta-(atan2((gy1-iy1),(gx1-ix1))))); //if theta1 is positive, clockwise
    set[0].l=(sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)));
    set[0].theta2=-((final.theta-(atan2((gy1-iy1),(gx1-ix1)))));

    set[1].theta1=(set[0].theta1>0)?(-pi+(set[0].theta1)):(pi+(set[0].theta1));
    set[1].l=(sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)));
    set[1].theta2=(set[0].theta2>0)?(-pi+(set[0].theta2)):(pi+(set[0].theta2));//reversal or second direct common tangent

    set[2].theta1=init.theta-(atan2((gy1-iy1),(gx1-ix1))+asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));
    set[2].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[2].theta2=-(final.theta-(atan2((gy1-iy1),(gx1-ix1)))+asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));

    set[3].theta1=init.theta-(atan2((gy1-iy1),(gx1-ix1))+asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));
    set[3].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[3].theta2=-(final.theta-(atan2((gy1-iy1),(gx1-ix1)))-asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));

    set[4].theta1=init.theta-(atan2((gy1-iy1),(gx1-ix1))-asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));
    set[4].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[4].theta2=-(final.theta-(atan2((gy1-iy1),(gx1-ix1)))+asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));

    set[5].theta1=init.theta-(atan2((gy1-iy1),(gx1-ix1))-asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));
    set[5].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[5].theta2=-(final.theta-(atan2((gy1-iy1),(gx1-ix1)))-asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))));

    set[6].theta1=(set[2].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[2].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[2].theta1));
    set[6].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[6].theta2=(set[2].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[2].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[2].theta2));
    
    set[7].theta1=(set[3].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[3].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[3].theta1));
    set[7].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[7].theta2=(set[3].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[3].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[3].theta2));
    
    set[8].theta1=(set[4].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[4].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[4].theta1));
    set[8].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[8].theta2=(set[4].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[4].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[4].theta2));
    
    set[9].theta1=(set[5].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[5].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[5].theta1));
    set[9].l=sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1)-4*MinTR*MinTR);
    set[9].theta2=(set[5].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[5].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))+(set[5].theta2));
    
    //final circle 2 (replace g1 by g2)
    set[10].theta1=((init.theta-(atan2((gy2-iy1),(gx2-ix1)))));
    set[10].l=(sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)));
    set[10].theta2=-((final.theta-(atan2(abs(gy2-iy1),abs(gx2-ix1)))));

    set[11].theta1=(set[4].theta1>0)?(-pi+(set[4].theta1)):(pi+(set[4].theta1));
    set[11].l=(sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)));
    set[11].theta2=(set[4].theta2>0)?(-pi+(set[4].theta2)):(pi+(set[4].theta2));//reversal or second direct common tangent

    set[12].theta1=((init.theta-(atan2((gy2-iy1),(gx2-ix1)))+asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));
    set[12].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[12].theta2=-((final.theta-(atan2((gy2-iy1),(gx2-ix1)))+asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));

    set[13].theta1=((init.theta-(atan2((gy2-iy1),(gx2-ix1)))+asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));
    set[13].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[13].theta2=-((final.theta-(atan2((gy2-iy1),(gx2-ix1)))-asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));

    set[14].theta1=((init.theta-(atan2((gy2-iy1),(gx2-ix1)))-asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));
    set[14].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[14].theta2=-((final.theta-(atan2((gy2-iy1),(gx2-ix1)))+asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));

    set[15].theta1=((init.theta-(atan2((gy2-iy1),(gx2-ix1)))-asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));
    set[15].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[15].theta2=-((final.theta-(atan2((gy2-iy1),(gx2-ix1)))-asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)))))));

    set[16].theta1=(set[9].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx1-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[9].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[9].theta1));
    set[16].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[16].theta2=(set[9].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[9].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[9].theta2));
    
    set[17].theta1=(set[10].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx1-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[10].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[10].theta1));
    set[17].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[17].theta2=(set[10].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[10].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[10].theta2));
    
    set[18].theta1=(set[11].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx1-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[11].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[11].theta1));
    set[18].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[18].theta2=(set[11].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[11].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[11].theta2));
    
    set[19].theta1=(set[12].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx1-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[12].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[12].theta1));
    set[19].l=sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1)-4*MinTR*MinTR);
    set[19].theta2=(set[12].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[12].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix1)*(gx2-ix1)+(gy2-iy1)*(gy2-iy1))))))+(set[12].theta2));
    
    //initial circle 2:
    //final circle 1
    set[20].theta1=((init.theta-(atan2((gy1-iy2),(gx1-ix2)))));
    set[20].l=(sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)));
    set[20].theta2=-((final.theta-(atan2((gy1-iy2),(gx1-ix2)))));

    set[21].theta1=(set[8].theta1>0)?(-pi+(set[8].theta1)):(pi+(set[8].theta1));
    set[21].l=(sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)));
    set[21].theta2=(set[8].theta2>0)?(-pi+(set[8].theta2)):(pi+(set[8].theta2));//reversal or second direct common tangent

    set[22].theta1=((init.theta-(atan2((gy1-iy2),(gx1-ix2)))+asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));
    set[22].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[22].theta2=-((final.theta-(atan2((gy1-iy2),(gx1-ix2)))+asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));

    set[23].theta1=((init.theta-(atan2((gy1-iy2),(gx1-ix2)))+asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));
    set[23].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[23].theta2=-((final.theta-(atan2((gy1-iy2),(gx1-ix2)))-asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));

    set[24].theta1=((init.theta-(atan2((gy1-iy2),(gx1-ix2)))-asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));
    set[24].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[24].theta2=-((final.theta-(atan2((gy1-iy2),(gx1-ix2)))+asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));

    set[25].theta1=((init.theta-(atan2((gy1-iy2),(gx1-ix2)))-asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));
    set[25].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[25].theta2=-((final.theta-(atan2((gy1-iy2),(gx1-ix2)))-asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)))))));

    set[26].theta1=(set[16].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[16].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix1)+(gy1-iy2)*(gy1-iy2))))))+(set[16].theta1));
    set[26].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[26].theta2=(set[16].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[16].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[16].theta2));

    set[27].theta1=(set[17].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[17].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix1)+(gy1-iy2)*(gy1-iy2))))))+(set[17].theta1));
    set[27].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[27].theta2=(set[17].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[17].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[17].theta2));

    set[28].theta1=(set[18].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[18].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix1)+(gy1-iy2)*(gy1-iy2))))))+(set[18].theta1));
    set[28].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[28].theta2=(set[18].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[18].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[18].theta2));

    set[29].theta1=(set[19].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[19].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix1)+(gy1-iy2)*(gy1-iy2))))))+(set[19].theta1));
    set[29].l=sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2)-4*MinTR*MinTR);
    set[29].theta2=(set[19].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[19].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx1-ix2)*(gx1-ix2)+(gy1-iy2)*(gy1-iy2))))))+(set[19].theta2));

    //final circle 2 (replace gy1 by gy2)
    set[30].theta1=((init.theta-(atan2((gy2-iy2),(gx2-ix2)))));
    set[30].l=(sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)));
    set[30].theta2=-((final.theta-(atan2((gy2-iy2),(gx2-ix2)))));

    set[31].theta1=(set[12].theta1>0)?(-pi+(set[12].theta1)):(pi+(set[12].theta1));
    set[31].l=(sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)));
    set[31].theta2=(set[12].theta2>0)?(-pi+(set[12].theta2)):(pi+(set[12].theta2));//reversal or second direct common tangent

    set[32].theta1=((init.theta-(atan2((gy2-iy2),(gx2-ix2)))+asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));
    set[32].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[32].theta2=-((final.theta-(atan2((gy2-iy2),(gx2-ix2)))+asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));

    set[33].theta1=((init.theta-(atan2((gy2-iy2),(gx2-ix2)))+asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));
    set[33].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[33].theta2=-((final.theta-(atan2((gy2-iy2),(gx2-ix2)))-asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));

    set[34].theta1=((init.theta-(atan2((gy2-iy2),(gx2-ix2)))-asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));
    set[34].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[34].theta2=-((final.theta-(atan2((gy2-iy2),(gx2-ix2)))+asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));

    set[35].theta1=((init.theta-(atan2((gy2-iy2),(gx2-ix2)))-asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));
    set[35].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[35].theta2=-((final.theta-(atan2((gy2-iy2),(gx2-ix2)))-asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)))))));

    set[36].theta1=(set[23].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[23].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[23].theta1));
    set[36].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[36].theta2=(set[23].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[23].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[23].theta2));

    set[37].theta1=(set[24].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[24].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[24].theta1));
    set[37].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[37].theta2=(set[24].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[24].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[24].theta2));

    set[38].theta1=(set[25].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[25].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[25].theta1));
    set[38].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[38].theta2=(set[25].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[25].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[25].theta2));

    set[39].theta1=(set[26].theta1>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[26].theta1)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[26].theta1));
    set[39].l=sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2)-4*MinTR*MinTR);
    set[39].theta2=(set[26].theta2>0)?(-(pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[26].theta2)):((pi+2*asin(2*MinTR/((sqrt((gx2-ix2)*(gx2-ix2)+(gy2-iy2)*(gy2-iy2))))))+(set[26].theta2));

    //A common issue with some of the sets is the following: we land on the final circle with the right orientation but diametrically oppostie to the goal point which still gives us the answer

    for(int i=0; i<40; i++){
        set[i].type=i;
    }
    for(int i=0; i<40; i++){
        netlen[i]=abs(MinTR*set[i].theta1)+abs(MinTR*set[i].theta2)+set[i].l;
    }
    /*for(int i=0; i<16; i++){
        cout<<netlen[i]<<endl;
    }*/
    
    //cout<<"Ok"<<endl;
    //cout<<init.theta-(atan2(abs(gy1-iy1),abs(gx1-ix1))+asin(2*MinTR/((sqrt((gx1-ix1)*(gx1-ix1)+(gy1-iy1)*(gy1-iy1))))))<<endl;
    //cout<<ix1<<" "<<iy1<<" "<<ix2<<" "<<iy2<<" "<<gx1<<" "<<gy1<<" "<<gx2<<" "<<gy2<<endl;

    vector <state> path_temp;
    double min1=netlen[0];
    state init1=init;
    int mini=0;
    for(int i=0; i<40; i++){
        calcway(set[i], path_temp, init, init1, ix1, iy1, gx1, gy1, ix2, iy2, gx2, gy2);
        if(netlen[i]<=min1){
            if(abs((init1.x)-(final.x))<0.05&&abs((init1.y)-(final.y))<0.05&&abs((init1.theta)-(final.theta)<0.018)){
                min1=netlen[i];
                mini=i;
            }
        }
        path_temp.clear();
    }
    init1=init;
    statetype=mini;
    calcway(set[mini], path_final, init, init1, ix1, iy1, gx1, gy1, ix2, iy2, gx2, gy2);

        nav_msgs::Path path;
    path.header.frame_id = "/map";
    path.poses.clear();

    for(int i=0; i<path_final.size(); i++){
        geometry_msgs::PoseStamped vertex;
        vertex.header.stamp=ros::Time::now();
        vertex.header.frame_id="/map";
        vertex.pose.position.x= double(path_final[i].x);
        vertex.pose.position.y= double(path_final[i].y);
        vertex.pose.position.z= 0.0;
    
        vertex.pose.orientation.x= 0.0; //float(width_assign_left(Path_nodes[i].second, Path_nodes[i].first))*0.3;
        vertex.pose.orientation.y= 0.0; //float(width_assign_right(Path_nodes[i].second, Path_nodes[i].first))*0.3;
        vertex.pose.orientation.z= 0.0;
        vertex.pose.orientation.w= 0.0;

        path.poses.push_back(vertex);
    }
    cout<<path_final[path_final.size()-1].x<<path_final[path_final.size()-1].y;
    path_pub.publish(path);
  }



int main(int argc,char** argv){
    ros::init(argc,argv,"reeds_shepp");
    ros::NodeHandle n;

    //ros::Subscriber start_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, SetStart);
    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, SetGoal);
     

    path_pub= n.advertise< nav_msgs::Path >("/reeds_shep_path", 1); 
    //pub_blown_local_map = n.advertise<nav_msgs::OccupancyGrid>("/dilated_map",1);


    ros::Rate RosLoopRate(20);     // 15 in RRT*
    while(ros::ok()){
       ros::spinOnce(); 
       RosLoopRate.sleep(); 
    }
   
    return 0;
}

