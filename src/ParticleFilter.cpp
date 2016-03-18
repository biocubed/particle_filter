#include <particle_filter/ParticleFilter.h>
#include <random>
#include <math.h>

ParticleFilter::ParticleFilter(int n):nm_(n)
{
	i_=0;
	Catheters_.resize(nm_);
	weights_.resize(nm_);
	covs_.resize(4);
}

bool ParticleFilter::setWeights(std::vector<double> w)
{
	if (w.size()!=Catheters_.size())
	{
		return false;
	}
	weights_=w;
	return true;
}

bool ParticleFilter::setMotionErrors(std::vector<geometry_msgs::Point> vars)
{
	if (vars.size()!=vars_.size())
	{
		return false;
	}
	vars_=vars;
	return true;
}

void ParticleFilter::generateParctiles(double i=0)
{
	i_=i; // update the current in coil
	

}

double ParticleFilter::numericSolver(double i)
{
	double alpha_z=-3.14; // initial solution is -3.14
	double alpha_z_n=-3.14;
	Eigen::Vector3d t_B;
	Eigen::Vector3d M_z;
	Eigen::Vector3d j(0,1,0);
	Eigen::vector3d g(9.8,0,0);
	double err=100;
	
	while(abs(err)>0.15)
	{
		alpha_z=alpha_z_n;
		t_B<<cos(alpha_z),sin(alpha_z),0;
		M_z=N*i*A*t_B.cross(B)-l/alpha_z*m*j.cross(g);
		alpha_z_n=M_z(2)/E/I;
		err=alpha_z-alpah_z_n;
	}
	
}

double ParticleFilter::sample(double b)
{
	double x=0, r=0;
	for(int j=0, j < 12, j++)
	{
		r=(rand()*2.0/RAND_MAX-1.0)*b;
		x=x+r;
	}
	return x;
}

geometry_msgs::Point ParticleFilter::sample(geometry_msgs::Point b, geometry_msgs::Point pt)
{
	geometry_msgs::Point pt_h;
	pt_h.x = pt.x+sample(b.x);
	pt_h.y = pt.y+sample(b.y);
	pt_h.z = pt.z+sample(b.z);
	return pt_h;
}


geometry_msgs::Point CatheterPose::getBSpline(double s)
	{
		geometry_msgs::Point b;
		b.x=-1000;
		b.y=-1000;
		b.z=-1000;
		
		if (s>1) 
			return b;
		else if (s>0.5)
		{
			s=s*2-1;
			double P1_x=B_.x;
			double P1_y=B_.y;
			double P1_z=B_.z;
			double P3_x=C_.x;
			double P3_y=C_.y;
			double P3_z=C_.z;
			double alpha2= beta2*sqrt((P3_x-P1_x)*(P3_x-P1_x)+(P3_y-P1_y)*(P3_y-P1_y)+(P3_z-P1_z)*(P3_z-P1_z));
			double P2_x=P1_x+alpha2*t_.x;
			double P2_y=P1_y+alpha2*t_.y;
			double P2_z=P1_z+alpha2*t_.z;
			b.x=A_.x*(1-s)*(1-s)+P2_x*2*(1-s)*s+P3_x*s*s;
			b.y=A_.y*(1-s)*(1-s)+P2_y*2*(1-s)*s+P3_y*s*s;
			b.z=A_.z*(1-s)*(1-s)+P2_z*2*(1-s)*s+P3_z*s*s;
			return b;
		}
		else if (s>0)
		{
			s=s*2;
			geometry_msgs::Point t_A;
			t_A.x=1;
			t_A.y=0;
			t_A.z=0;
			double P1_x=A_.x;
			double P1_y=A_.y;
			double P1_z=A_.z;
			double P4_x=B_.x;
			double P4_y=B_.y;
			double P4_z=B_.z;
			double alpha1= beta1*sqrt((P4_x-P1_x)*(P4_x-P1_x)+(P4_y-P1_y)*(P4_y-P1_y)+(P4_z-P1_z)*(P4_z-P1_z));
			double P2_x=P1_x+alpha1*t_A.x;
			double P2_y=P1_y+alpha1*t_A.y;
			double P2_z=P1_z+alpha1*t_A.z;
			double P3_x=P4_x-alpha1*t_.x;
			double P3_y=P4_y-alpha1*t_.y;
			double P3_z=P4_z-alpha1*t_.z;
			b.x=P1_x*(1-s)*(1-s)*(1-s)+P2_x*3*(1-s)*(1-s)*s+P3_x*3*(1-s)*s*s+P4_x*s*s*s;
			b.x=P1_y*(1-s)*(1-s)*(1-s)+P2_y*3*(1-s)*(1-s)*s+P3_y*3*(1-s)*s*s+P4_y*s*s*s;
			b.x=P1_z*(1-s)*(1-s)*(1-s)+P2_z*3*(1-s)*(1-s)*s+P3_z*3*(1-s)*s*s+P4_z*s*s*s;
			return b;
		}
		else 
			return b;
	} 