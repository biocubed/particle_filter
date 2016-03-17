#ifndef FARTICLEFILTER_
#define FARTICLEFILTER_

#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>

const double beta1 = 0.5;
const double beta2 = 0.5;

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
	bool setMotionErrors(std::vector<geometry_msgs::Point> covs);
	std::vector<CatheterPose> getParticles(){return Catheters_;};
	bool setWeights(std::vector<double> w);
	void generateParctiles(double i);
	void resample();
	CatheterPose getMeanParticle();
	bool getCov(double *c); 

private:
	std::vector<CatheterPose> Catheters_;
	int nm_; // the number of particles
	double i_;// current in the coil
	std::vector<double> weights_; // weights from measurement model
	std::vector<geometry_msgs::Point> covs_;// 12 covariances of A, B, C and t in Motion model

	
};

#endif
