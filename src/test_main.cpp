#include <ros/ros.h> 
#include <particle_filter/ParticleFilter.h>
#include <geometry_msgs/Point.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"test_main"); 
  ros::NodeHandle n;
  ParticleFilter pf(5);
  double i;
  cin>>i;
  double a = pf.numericSolver(i);
  ROS_INFO("a is %f",a);
 
  return 0; 
} 
