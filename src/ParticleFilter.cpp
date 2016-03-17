#include <particle_filter/ParticleFilter.h>


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

bool ParticleFilter::setMotionErrors(std::vector<geometry_msgs::Point> covs)
{
	if (covs.size()!=covs_.size())
	{
		return false;
	}
	covs_=covs;
	return true;
}

void ParticleFilter::generateParctiles(double i=0)
{
	i_=i; // update the current in coil
	

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