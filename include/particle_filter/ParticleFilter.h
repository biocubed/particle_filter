#ifndef FARTICLEFILTER_
#define FARTICLEFILTER_

#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <ros/ros.h> 

/* parameters for Bezier spline */
const double beta1 = 0.5;
const double beta2 = 0.5;


/* parameters for motion model (kg.m.s)*/
const Eigen::Vector3d B(0,3,0); // magnetic filed
const double N=130; // the number of the coils
const double A=0.0000159; // the cross area of the coil
const double l_AB=0.08; // the length of the AB
const double l_BC=0.02; // the length of the BC 
const double m=0.05;  // the mass of the coil set
const double E=10.15; // Young's modulus
const double I=0.0005; // moment of inertia wrt z axis

struct CatheterPose
{
	geometry_msgs::Point A_;
	geometry_msgs::Point B_;
	geometry_msgs::Point C_;
	geometry_msgs::Point t_;
	geometry_msgs::Point getBSpline(double s);
};

class ParticleFilter	
{
public:
	ParticleFilter(int n);
	bool setMotionErrors(std::vector<geometry_msgs::Point> vars);
	std::vector<CatheterPose> getParticles(){return Catheters_;};
	bool setWeights(std::vector<double> w);
	void generateParctiles(double i);
	void resample();
	CatheterPose getMeanParticle();
	CatheterPose getBestPartcle();
	bool getVar(double *c); 


private:
	std::vector<CatheterPose> Catheters_;
	int nm_; // the number of particles
	double i_;// current in the coil
	std::vector<double> weights_; // weights from measurement model
	std::vector<geometry_msgs::Point> vars_;// 12 variances of A, B, C and t in Motion model

	double sample(double b);
	geometry_msgs::Point sample(geometry_msgs::Point b, geometry_msgs::Point pt);
	double min_alpha(double a);
	double numericSolver(double i);

};

#endif
