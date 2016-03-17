#include <ros/ros.h> 
#include <particle_filter/ParticleFilter.h>
#include <geometry_msgs/Point.h>

int main(int argc, char **argv) 
{ 
  ros::init(argc,argv,"test_main"); 
  ros::NodeHandle n;
  ParticleFilter(5);

  return 0; 
} 
