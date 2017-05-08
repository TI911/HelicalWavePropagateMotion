/*
 * helical_wave_propagate_motion.h
 *
 *  Created on: Feb 23, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_HELICAL_WAVE_PROPAGATE_MOTION_H_
#define SNAKE_CONTROL_SRC_HELICAL_WAVE_PROPAGATE_MOTION_H_

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

class HelicalWavePropagateMotion: public ShiftControlMethod {
 public:
	virtual ~HelicalWavePropagateMotion(){}
	HelicalWavePropagateMotion(RobotSpec spec,

			double ds
			){

		ds_ = ds;
		radius_ = 0.10;
		delta_ = 0.10/(2*M_PI);
		theta_ = 0 ;

		num_link_ = spec.num_joint();
		target_angle_ = 0;
		//target_angle_even = 0;
		link_length_ = 0.07;
		flag_ = 0;

		a_ = 0.50;//Init(spec);
		omega_= 0.5;
		phi_0_ = -2*M_PI;
		phi_hyperbolic_ = -2*M_PI;
		pre_s_ = 0;
		step_s_= ds/50;
		s_ = 0;
		S_T = 0;
		t = 0;
		PI_= 8;
		psi_ = 0;

		Init(spec);
	}

	//--- 動作
    void HelicalWavePropagateMotionByShift(RobotSpec spec);
    void WavePropagation(RobotSpec spec);
    void init();

    void CalculateCurvature();
    void CalculateTorsion();
    void Hyperbolic();

    void CalculateCurvatureWithHyperbolic();
	void CalculateTorsionWithHyperbolic();
    void CalculateCurvatureTorsionWithHyperbolic();

    //void
    void CalculateSTRelation();
    void CalculateTargetAngle(RobotSpec spec);
    void CalculateTargetAngle2(RobotSpec spec);
    void CalculateTargetAngle3(RobotSpec spec);
    void Test(RobotSpec spec);

    virtual void InitializeShape() {}

	//--- 形状パラメータ変更
	void set_radius(double radius);
	void add_radius(double radius_add){ set_radius(radius_ + radius_add);}
	void set_delta(double delta);
	void add_delta(double delta_add){ set_delta(delta_ + delta_add/2*M_PI);}
	void set_theta(double theta);
	void add_theta(double theta_add){ set_theta(theta_add);}
	void set_s(double s);
	void add_s(double add_s){set_s(add_s + s_); }

    void set_phi(double phi);
    void add_phi(double add_phi){set_phi(add_phi + phi_hyperbolic_);};

    void set_psi(double psi);
    void add_psi(double add_psi){set_psi(add_psi + psi_);};

	double s_;
	//double v;
	//double bias;
	double pre_s_;
	double step_s_;
	double a_, omega_, phi_0_, phi_hyperbolic_, delta_, t, radius_, theta_;
	double target_angle_;
	//double target_angle_even;
	int num_link_;
	double link_length_;
	double S_T;
	int flag_;
	double PI_;



};


#endif /* SNAKE_CONTROL_SRC_HELICAL_WAVE_PROPAGATE_MOTION_H_ */
