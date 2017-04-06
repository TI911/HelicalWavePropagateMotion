/*
 * helical_wave_propagate_motion.cpp
 *
 *  Created on: Feb 23, 2017
 *      Author: ubuntu-ti
 */

#include "helical_wave_propagate_motion.h"

void HelicalWavePropagateMotion::set_radius(double radius)
{
	radius_ = radius;
	ROS_INFO("> radius = %4.3f", radius_);
}

void HelicalWavePropagateMotion::set_delta(double delta)
{
	delta_ = delta;
	ROS_INFO("> delta = %4.3f", delta_);
}

void HelicalWavePropagateMotion::set_theta(double theta)
{
   flag_ ++;
   if(flag_ > 1) flag_ = 1;
}

void HelicalWavePropagateMotion::set_s(double s)
{
	s_ = s;
}

void HelicalWavePropagateMotion::HelicalWavePropagateMotionByShift(RobotSpec spec)
{
	while(s_ > pre_s_ + step_s_){

		if(flag_) phi_ += 0.1;

		if(phi_> 2*M_PI/omega_){
			phi_  = -2*M_PI/omega_;
			flag_ = 0;
		}

		while(pre_s_+step_s_ > S_T){
			t += 0.01;
			CalculateSTRelation();
		}

		CalculateCurvatureTorsionWithHyperbolic();
		ShiftControlMethod::Shift_Param(spec);
		CalculateTargetAngle(spec);

		psi_   = 0.0;
		pre_s_ = pre_s_ + step_s_;
	}
}

void HelicalWavePropagateMotion::CalculateCurvature()
{
	kappa_ = radius_ /(pow(radius_,2)+pow(delta_,2)) + 3*sin(3*theta_);
}

void HelicalWavePropagateMotion::CalculateTorsion()
{
    double first_tau = delta_ /(pow(radius_,2)+pow(delta_,2));
    //tau_ = first_tau*0.07 + tau_;
    tau_ = first_tau*pre_s_;

    psi_ = 0;	//first_tau*(pre_s_+step_s_);
}

void HelicalWavePropagateMotion::CalculateCurvatureWithHyperbolic()
{
    //t = 0;
	kappa_ = pow(pow(delta_,2)*pow((2*a_*pow(omega_,2)*cos(t)-a_*pow(cosh(phi_-omega_*t),2)*cos(t)*(pow(omega_,2)-1)
			+a_*omega_*sinh(2*phi_-2*omega_*t)*sin(t))/pow(cosh(phi_-omega_*t),3)+radius_*cos(t),2)+pow((pow(a_,2)*pow(omega_,2)
			+pow(a_,2)+(3*pow(radius_,2))/4)/pow(cosh(phi_-omega_*t),2)+(radius_*(4*a_+6*a_*pow(omega_,2)+4*a_*cosh(2*phi_
			-2*omega_*t)+radius_*cosh(3*phi_-3*omega_*t)+(-2)*a_*pow(omega_,2)*cosh(2*phi_-2*omega_*t)))
			/(4*pow(cosh(phi_-omega_*t),3)),2)+pow(delta_,2)*pow((a_*pow(cosh(phi_-omega_*t),2)*sin(t)*(pow(omega_,2)-1)
			-2*a_*pow(omega_,2)*sin(t)+a_*omega_*cos(t)*sinh(2*phi_-2*omega_*t))/pow(cosh(phi_-omega_*t),3)-radius_*sin(t),2),1/2)
			/pow((2*a_*radius_*pow(cosh(phi_-omega_*t),3)-pow(a_,2)*pow(omega_,2)+pow(a_,2)*pow(cosh(phi_-omega_*t),2)*(pow(omega_,2)+1))
			/pow(cosh(phi_-omega_*t),4)+pow(delta_,2)+pow(radius_,2),3/2);

    psi_ = 0.0; //first_tau*pre_s_;
}

void HelicalWavePropagateMotion::CalculateTorsionWithHyperbolic()
{
	/*
		double first_tau = (delta_*(4*pow(radius_,2)*cosh(2*phi_-2*omega_*t)+pow(radius_,2)*cosh(4*phi_-4*omega_*t)+4*pow(a_,2)+3*pow(radius_,2)
				+24*pow(a_,2)*pow(omega_,2)+20*pow(a_,2)*pow(omega_,4)+4*pow(a_,2)*cosh(2*phi_-2*omega_*t)+4*a_*radius_*cosh(3*phi_-3*omega_*t)
				  +12*a_*radius_*cosh(phi_-omega_*t)+8*pow(a_,2)*pow(omega_,2)*cosh(2*phi_-2*omega_*t)+4*pow(a_,2)*pow(omega_,4)*cosh(2*phi_-2*omega_*t)
				    +40*a_*pow(omega_,2)*radius_*cosh(phi_-omega_*t)+(-8)*a_*pow(omega_,2)*radius_*cosh(3*phi_-3*omega_*t)))
					  /(8*pow(cosh(phi_-omega_*t),4)*(pow(pow(radius_,2)*cosh(3*phi_-3*omega_*t)+4*a_*radius_+4*pow(a_,2)*cosh(phi_-omega_*t)
						 +3*pow(radius_,2)*cosh(phi_-omega_*t)+6*a_*pow(omega_,2)*radius_+4*a_*radius_*cosh(2*phi_-2*omega_*t)
							+4*pow(a_,2)*pow(omega_,2)*cosh(phi_-omega_*t)+(-2)*a_*pow(omega_,2)*radius_*cosh(2*phi_-2*omega_*t),2)/
								(16*pow(cosh(phi_-omega_*t),6))+(pow(delta_,2)*pow(2*a_*pow(omega_,2)*cos(t)+a_*pow(cosh(phi_-omega_*t),2)*cos(t)
									+radius_*pow(cosh(phi_-omega_*t),3)*cos(t)+(-a_)*pow(omega_,2)*pow(cosh(phi_-omega_*t),2)*cos(t)
										+a_*omega_*sinh(2*phi_-2*omega_*t)*sin(t),2))/pow(cosh(phi_-omega_*t),6)+(pow(delta_,2)*pow(2*a_*pow(omega_,2)*sin(t)
											+a_*pow(cosh(phi_-omega_*t),2)*sin(t)+radius_*pow(cosh(phi_-omega_*t),3)*sin(t)+(-a_)*pow(omega_,2)
												*pow(cosh(phi_-omega_*t),2)*sin(t)+(-a_)*omega_*cos(t)*sinh(2*phi_-2*omega_*t),2))/pow(cosh(phi_-omega_*t),6)));
	*/

    double first_tau = delta_ /(pow(radius_,2)+pow(delta_,2));
    tau_ = first_tau*step_s_ + tau_;
    //tau_ = first_tau*pre_s_;
}

void HelicalWavePropagateMotion::CalculateCurvatureTorsionWithHyperbolic()
{
	  double num   = 0,
			 denom = 0;

	  double x[3], y[3], z[3];

		x[0] = (-a_*omega_*cos(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_))
				-sin(t)*(a_*pow(cosh(omega_*t-phi_),-1)+radius_);
		y[0] = cos(t)*(a_*pow(cosh(omega_*t-phi_),-1)+radius_)
				-a_*omega_*sin(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_);
		z[0] = (M_PI*delta_)/2;

		x[1] = 2*a_*pow(omega_,2)*cos(t)*pow(cosh(omega_*t-phi_),-3)*pow(sinh(omega_*t-phi_),2)
				+2*a_*omega_*sin(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_)
				-cos(t)*(a_*pow(cosh(omega_*t-phi_),-1)+radius_)-a_*pow(omega_,2)*cos(t)*pow(cosh(omega_*t-phi_),-1);
		y[1] = 2*a_*pow(omega_,2)*sin(t)*pow(cosh(omega_*t-phi_),-3)*pow(sinh(omega_*t-phi_),2)
				-2*a_*omega_*cos(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_)
				-sin(t)*(a_*pow(cosh(omega_*t-phi_),-1)+radius_)-a_*pow(omega_,2)*sin(t)*pow(cosh(omega_*t-phi_),-1);
		z[1] = 0;

		x[2] = (-6*a_*pow(omega_,3)*cos(t)*pow(cosh(omega_*t-phi_),-4)*pow(sinh(omega_*t-phi_),3))
				-6*a_*pow(omega_,2)*sin(t)*pow(cosh(omega_*t-phi_),-3)*pow(sinh(omega_*t-phi_),2)
				+5*a_*pow(omega_,3)*cos(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_)
				+3*a_*omega_*cos(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_)+sin(t)*
			    (a_*pow(cosh(omega_*t-phi_),-1)+radius_)+3*a_*pow(omega_,2)*sin(t)*pow(cosh(omega_*t-phi_),-1);
		y[2] = (-6*a_*pow(omega_,3)*sin(t)*pow(cosh(omega_*t-phi_),-4)*pow(sinh(omega_*t-phi_),3))
				+6*a_*pow(omega_,2)*cos(t)*pow(cosh(omega_*t-phi_),-3)*pow(sinh(omega_*t-phi_),2)
				+5*a_*pow(omega_,3)*sin(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_)
				+3*a_*omega_*sin(t)*pow(cosh(omega_*t-phi_),-2)*sinh(omega_*t-phi_)-cos(t)*(a_*pow(cosh(omega_*t-phi_),-1)
				+radius_)-3*a_*pow(omega_,2)*cos(t)*pow(cosh(omega_*t-phi_),-1);
		z[2] = 0;

		num =
			sqrt(pow(y[0]*z[1] - z[0]*y[1], 2)
			   + pow(z[0]*x[1] - x[0]*z[1], 2)
			   + pow(x[0]*y[1] - y[0]*x[1], 2));

		denom =	pow(x[0]*x[0] + y[0]*y[0] + z[0]*z[0], 1.5);
		kappa_ = num/denom;

		num =
			  x[2]*(y[0]*z[1] - z[0]*y[1])
			+ y[2]*(z[0]*x[1] - x[0]*z[1])
			+ z[2]*(x[0]*y[1] - y[0]*x[1]);

		denom =
			  pow(y[0]*z[1] - z[0]*y[1], 2)
			+ pow(z[0]*x[1] - x[0]*z[1], 2)
			+ pow(x[0]*y[1] - y[0]*x[1], 2);

		double first_tau = num/denom;
	    tau_ = first_tau*step_s_ + tau_;
}

void HelicalWavePropagateMotion::CalculateSTRelation()
{
	  double x1, y1, z1,st;

	  x1 = pow(a_,2)*pow(omega_,2)*pow(cosh(omega_*t+phi_),-4)*pow(sinh(omega_*t+phi_),2);
	  y1 = pow(a_*pow(cosh(omega_*t+phi_),-1)+radius_,2);
	  z1 = (pow(M_PI, 2)*pow(delta_, 2))/4;

	  st   = sqrt(x1 + y1 + z1);
	  S_T += st;
}

void HelicalWavePropagateMotion::Test(RobotSpec spec)
{
	kappa_ = radius_/(pow(radius_,2)+pow(delta_,2));// + sin(theta_);
	tau_ = delta_/(pow(radius_,2)+pow(delta_,2));

	double NUMOFLINK = 39 ;
	for(int i=0; i<NUMOFLINK; i++){
		if(i>=1){
			snake_model_param.tau.insert(snake_model_param.tau.begin()+i,
					tau_*link_length_ + snake_model_param.tau[i-1]);
		}else{
			snake_model_param.tau.insert(snake_model_param.tau.begin()+i,
					tau_*link_length_);

		}
		snake_model_param.kappa.insert(snake_model_param.kappa.begin()+i, kappa_);
		snake_model_param.psi.insert(snake_model_param.psi.begin()+i,  0);
	}
}

void HelicalWavePropagateMotion::CalculateTargetAngle(RobotSpec spec)
{
	for(int i=0; i<num_link_; i++){
		if(i%2){ //(奇数番目)
			target_angle_ =
					-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.tau[i]+ snake_model_param.psi[i]);
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);

		}else{  //(偶数番目)
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i]*cos(snake_model_param.tau[i]+ snake_model_param.psi[i]);
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);
		}

        snake_model_param.angle.pop_back();
		snake_model_param.kappa.pop_back();
		//snake_model_param.phi.pop_back();
		snake_model_param.tau.pop_back();
		snake_model_param.psi.pop_back();
	}
	usleep(1000*10);        // 制御に時間がかかるので1秒寝て待つ
}
