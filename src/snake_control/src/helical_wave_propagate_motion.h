/*
 * helical_wave_propagate_motion.h
 *
 *  Created on: Feb 23, 2017
 *      Author: ubuntu-ti
 *
 *      2017/11/14 删除多余内容，如不必要的头文件，变量等
 *       		   增加注释
 *     			  => HOW TO MAKE step_s_ ???
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

		ds_ 	= ds;
		radius_ = 0.10; // [m]
		delta_ 	= 0.02; // [m]

		num_link_ 	  = spec.num_joint();
		link_length_  = spec.link_length_body();
		target_angle_ = 0;


		a_ 		= 0.01;
		omega_	= 4; //sqrt(pow(2*radius_,2)+ pow(delta_*M_PI,2))/(4*M_PI);//sqrt(pow(2*radius_,2)+pow(delta_*M_PI,2))*4*M_PI;//

		phi_hyperbolic_ = 2*M_PI;//+(t_*omega_);// -omega_/2;

		phi_0_	= -M_PI/2 ;//+(t_*omega_);//-2*M_PI;
		omega_0_= 0.4;

		s_ 		= 0;
		S_T 	= 0;
		t_ 		= 0;
		//pi_		= 4;

		pre_s_  = 0;
		step_s_ = ds/28;

		psi_ 	= 0;
		psi_hyper_ = 0;
		psi4roll_  = 0;

		ahead_psi_=2*phi_hyperbolic_;

		flag_ 	= 0;
		psi4roll_flag_ = true;

		s_mark_= 0;

		Init(spec);
	}

	//--- 動作
    void HelicalWavePropagateMotionByShift(RobotSpec spec);
    void init();

    void CalculateCurvatureTorsionWithHelicalCurveSimple();
    void CalculateTorsionWithHelicalCurveSimple();
    void CalculateTorsionWithHelicalCurve();
    void CalculateCurvatureTorsionWithHelicalCurve();
    void CalculateCurvatureTorsionWithHyperbolic();

    double RungeKutta(double x0, double t0, double tn, int n);
    double dxdt(double t, double x);

    //void
    double CalculateSTRelation(double tt);
    double CalculateIntegral(double a, double b);

    void CalculateTargetAngle(RobotSpec spec);
    void CalculateTargetAngleForRolling(RobotSpec spec);
    void CalculateTargetAngle3(RobotSpec spec);

    virtual void InitializeShape() {}

	//--- 形状パラメータ変更
	void set_radius(double radius);
	void add_radius(double radius_add){ set_radius(radius_ + radius_add);}
	void set_delta(double delta);
	void add_delta(double delta_add){ set_delta(delta_ + delta_add/2*M_PI);}

	void set_a(double a);
	void add_a(double a_add){ set_a(a_ + a_add);}
	void set_omega(double omega);
	void add_omega(double omega_add){ set_omega(omega_ + omega_add);}

	void set_s(double s);
	void add_s(double add_s){set_s(add_s + s_);}

	void set_flag_on();
	void set_flag_off();

    void set_phi(double phi);
    void add_phi(double add_phi){set_phi(add_phi + phi_hyperbolic_);};

    //void set_pi(double pi);
    //void add_pi(double add_pi){set_pi(add_pi + pi_);};
    void set_psi4roll(double psi4roll);
    void add_psi4roll(double add_psi4roll){ set_psi4roll(add_psi4roll);};

    void print_parameters();
    void compensate_G();

	double s_;
	//double v;
	//double bias;
	double pre_s_;
	double step_s_;
	double a_, omega_, phi_0_, omega_0_, phi_hyperbolic_, delta_, t_, radius_;
	double target_angle_;
	//double target_angle_even;
	int num_link_;
	double link_length_;
	double S_T;
	bool flag_;
	//double pi_;
	double psi4roll_, ahead_psi_, s_mark_;
	bool psi4roll_flag_;

};

#endif /* SNAKE_CONTROL_SRC_HELICAL_WAVE_PROPAGATE_MOTION_H_ */
