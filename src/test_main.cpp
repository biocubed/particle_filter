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
    //pf.setMotionErros();

    // particle filter iteration
    //pf.generateParticles(0.5); // generate the particles when the current in the axial coils is 500mA
    //Catheters = pf.getParticles();
    //Images = ProjectionFnc(Catheters);
    //Weights = CompareFnc(Images, Images_from_measurement);
    //pf.setWeights();
    //pf.resample();
    //pf.getMeanParticle();
    //pf.getBestPartcle();
    //pf.getVar(); 
    //sleep(); // the updating rate of particle filter 
   
    return 0; 
} 
