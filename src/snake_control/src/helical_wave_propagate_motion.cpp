/*
 * helical_wave_propagate_motion.cpp
 *
 *  Created on: Feb 23, 2017
 *      Author: ubuntu-ti
 */

#include "helical_wave_propagate_motion.h"

/* Set Radius For Helical Wave Propagate Motion */
void HelicalWavePropagateMotion::set_radius(double radius)
{
	radius_ = radius;
}

/* Set Pitch For Helical Wave Propagate Motion */
void HelicalWavePropagateMotion::set_delta(double delta)
{
	delta_ = delta;
}

void HelicalWavePropagateMotion::set_s(double s)
{
	s_ = s;
}

void HelicalWavePropagateMotion::set_a(double a)
{
	a_ = a;
}

void HelicalWavePropagateMotion::set_omega(double omega)
{
	omega_ = omega;
}

void HelicalWavePropagateMotion::set_phi(double phi)
{
	phi_hyperbolic_ = phi;
}

void HelicalWavePropagateMotion::set_psi4roll(double psi4roll)
{
	psi4roll_ += psi4roll;
}

void HelicalWavePropagateMotion::set_flag_on()
{
	flag_ = true;
}

void HelicalWavePropagateMotion::set_flag_off()
{
	flag_ = false;
}

void HelicalWavePropagateMotion::compensate_G()
{
	if(psi4roll_flag_)
	{
		//add_phi(s_mark_ + M_PI/2);
/*		phi_hyperbolic_ =  s_mark_ + M_PI/2;
		s_mark_ = s_;*/


		//add_phi(0.02);
		//psi_hyper_ += 0.001;
	}

}

// 必要なパラメーターをターミナルに表示する
void HelicalWavePropagateMotion::print_parameters()
{
	ROS_INFO("> RADIUS = %4.3f	*", radius_);
	ROS_INFO("> PITCH  = %4.3f	*", delta_);
	ROS_INFO(">	- -	- -	*");
	ROS_INFO("> A_     = %4.3f	*", a_);
	ROS_INFO("> omega_ = %4.3f	*", omega_);
	ROS_INFO("> phi    = %4.3f	*", phi_hyperbolic_);
	ROS_INFO(">	- -	- -	*");
	ROS_INFO("> t_     = %4.3f	*", t_);
	ROS_INFO("> psi_   = %4.3f	*", psi_);
	ROS_INFO("> s_     = %4.3f	*", s_);
	ROS_INFO("> S_T    = %4.3f	*", S_T);
	//ROS_INFO(">  psi4roll_ = %4.3f", psi4roll_);
	ROS_INFO(">  link_length_*num_link_ = %4.3f", link_length_*num_link_);
	ROS_INFO("> s_mark_    = %4.3f	*", s_mark_);
	ROS_INFO("> step_s     = %4.3f	*", step_s_);
	ROS_INFO("> kappa_     = %4.3f	*", kappa_);
	ROS_INFO("> tau_helical_ = %4.3f *", tau_helical_);
	ROS_INFO("> tau_hyperbolic_ = %4.3f *", tau_hyperbolic_);
	ROS_INFO("> ***********************");
}

/* Helical Wave Propagate Motion INIT */
void HelicalWavePropagateMotion::init()
{
	pre_s_ = 0;
	s_     = 0;
	t_     = 0;

	S_T    = 0;
	tau_   = 0;
	kappa_ = 0;
	psi_   = 0;
	psi_hyper_ = 0;
}

/* Helical Wave Propagate Motion By Shift */
void HelicalWavePropagateMotion::HelicalWavePropagateMotionByShift(RobotSpec spec)
{
	while(s_ > pre_s_ + step_s_){  //

		while(pre_s_+step_s_ > S_T){
			t_ = t_ + 0.1;
			//CalculateSTRelation(t_);
			//CalculateIntegral(t_, t_+0.1);
			S_T = RungeKutta(S_T, t_, t_+0.1, 100);	//ルンゲクッタ法(初期条件x0,  区間[t_, t_+0.1], 分割数100-> 0.1/100 ->0.001
		}

		//Cross buttonを一回押すと flag ON に
		if(flag_){

			double ss= sqrt(pow(radius_*t_, 2)+pow((delta_*t_)/(2*M_PI), 2));
			if(ss>=link_length_*num_link_){
				if(s_ - s_mark_ >=link_length_*num_link_){
					t_= 0;
					s_mark_ = s_;
				}
			}

			//compensate_G();
			//常螺旋曲線の捩率τの計算
			//CalculateTorsionWithHelicalCurveSimple();
			CalculateTorsionWithHelicalCurve();
			//CalculateCurvatureTorsionWithHelicalCurveSimple();
			//CalculateCurvatureTorsionWithHelicalCurve();

			//ハイパボリック曲線の曲率κと捩率τの計算
			CalculateCurvatureTorsionWithHyperbolic();

			//PSI　補正値
			//Helical Wave Propagate Motion のための　PSI　補正値のシフト
			//psi_hyper_ = psi_hyper_ + (tau_hyperbolic_ - tau_helical_) ;   //捻転を抑制するため
			psi_hyper_ = psi_hyper_ + (tau_helical_  - tau_hyperbolic_);  //捻転を抑制するため，

			ShiftParamPsiHyperForHelicalWave(spec);
			//Helical Wafave Propagate Motion のための　Kappa のシフト
			ShiftParamCurvatureForHelicalWave(spec);
			//ShiftControlMethod::Shift_Param_Back(spec);

		}else{
			//常螺旋曲線の捩率τの計算
			//CalculateCurvatureTorsionWithHelicalCurveSimple();
			CalculateCurvatureTorsionWithHelicalCurve();
			// PSI
			psi_  =  psi_  + tau_helical_;
			//psi_hyper_ = 0;
			//ShiftControlMethod::Shift_Param_Back(spec);
			ShiftControlMethod::Shift_Param_Forward(spec);
		}
		CalculateTargetAngle(spec);
		pre_s_ = pre_s_ + step_s_;
	}
	print_parameters();
}

void HelicalWavePropagateMotion::CalculateCurvatureTorsionWithHelicalCurveSimple()
{
	kappa_ = radius_/(pow(radius_, 2) + pow(delta_, 2));
	tau_   = delta_/(pow(radius_, 2) + pow(delta_, 2));
	  //double first_tau_helical_ = tau_;
	tau_helical_ =  tau_*step_s_;
}

void HelicalWavePropagateMotion::CalculateTorsionWithHelicalCurveSimple()
{
	//kappa_ = radius_/(pow(radius_, 2) + pow(delta_, 2));
	tau_   = delta_/(pow(radius_, 2) + pow(delta_, 2));
	tau_helical_ =  tau_*step_s_;
}

/**
 *   CalculateTorsionWithHelicalCurve()
 *   常螺旋曲線の捩率を計算するが，Helical　Wave　Curveの式を使用する．
 *   phi_hyperbolic_　＝　-2*M_PI　の場合，ハイパボリック関数が入ってこないので
 *   常螺旋曲線と等しい
 *
 * */
void HelicalWavePropagateMotion::CalculateTorsionWithHelicalCurve()
{
	  double num   = 0,
			 denom = 0;

	  double x[3], y[3], z[3];

	  x[0] = (-a_0_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_))
				-sin(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_);
	  y[0] = cos(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)
				-a_0_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_);
	  z[0] = (M_PI*delta_)/2;

	  x[1] = 2*a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+2*a_0_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				-cos(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)-a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  y[1] = 2*a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				-2*a_0_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				-sin(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)-a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  z[1] = 0;

	  x[2] = (-6*a_0_*pow(omega_,3)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-4)*pow(sinh(omega_*t_-phi_hyperbolic_),3))
				-6*a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+5*a_0_*pow(omega_,3)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				+3*a_0_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)+sin(t_)*
			    (a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)+3*a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  y[2] = (-6*a_0_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-4)*pow(sinh(omega_*t_-phi_hyperbolic_),3))
				+6*a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+5*a_0_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				+3*a_0_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)-cos(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)
				+radius_)-3*a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  z[2] = 0;

	  num   =
			  x[2]*(y[0]*z[1] - z[0]*y[1])
			+ y[2]*(z[0]*x[1] - x[0]*z[1])
			+ z[2]*(x[0]*y[1] - y[0]*x[1]);

	  denom =
			  pow(y[0]*z[1] - z[0]*y[1], 2)
			+ pow(z[0]*x[1] - x[0]*z[1], 2)
			+ pow(x[0]*y[1] - y[0]*x[1], 2);

	  double first_tau_helical_ = num/denom;
	  tau_helical_ =  first_tau_helical_*step_s_;
}

/**
 *   CalculateCurvatureTorsionWithHelicalCurve()
 *
 *   常螺旋曲線の曲率と捩率を計算するが，Helical　Wave　Curveの式を使用する．
 *   phi_hyperbolic_　＝　-2*M_PI　の場合，ハイパボリック関数が入ってこないので
 *   常螺旋曲線と等しい
 *
 * */
void HelicalWavePropagateMotion::CalculateCurvatureTorsionWithHelicalCurve()
{
	  double num   = 0,
			 denom = 0;

	  double x[3], y[3], z[3];

	  // 1 回微分
	  x[0] = (-a_0_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_))
				-sin(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_);
	  y[0] = cos(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)
				-a_0_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_);
	  z[0] = (M_PI*delta_)/2;
	  //z[0] = (M_PI*delta_)/2-(a_0_*omega_*sinh(phi_hyperbolic_+omega_*t_))/pow(cosh(phi_hyperbolic_+omega_*t_),2);

	  // 2 回微分
	  x[1] = 2*a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+2*a_0_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				-cos(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)-a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  y[1] = 2*a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				-2*a_0_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				-sin(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)-a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  z[1] = 0;
	  //z[1] = (2*a_0_*pow(omega_,2)*pow(sinh(phi_hyperbolic_+omega_*t_),2))/
	  	//	      pow(cosh(phi_hyperbolic_+omega_*t_),3)-(a_0_*pow(omega_,2))/cosh(phi_hyperbolic_+omega_*t_);

	  // 3 回微分
	  x[2] = (-6*a_0_*pow(omega_,3)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-4)*pow(sinh(omega_*t_-phi_hyperbolic_),3))
				-6*a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+5*a_0_*pow(omega_,3)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				+3*a_0_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)+sin(t_)*
			    (a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)+3*a_0_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  y[2] = (-6*a_0_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-4)*pow(sinh(omega_*t_-phi_hyperbolic_),3))
				+6*a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+5*a_0_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				+3*a_0_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)-cos(t_)*(a_0_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)
				+radius_)-3*a_0_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
	  z[2] = 0;
	  //z[2] = (5*a_0_*pow(omega_,3)*sinh(phi_hyperbolic_+omega_*t_))/
		  //    pow(cosh(phi_hyperbolic_+omega_*t_),2)-(6*a_0_*pow(omega_,3)*pow(sinh(phi_hyperbolic_+omega_*t_),3))/
		  //    pow(cosh(phi_hyperbolic_+omega_*t_),4);

	  num =
			  sqrt(pow(y[0]*z[1] - z[0]*y[1], 2)
					  + pow(z[0]*x[1] - x[0]*z[1], 2)
					  + pow(x[0]*y[1] - y[0]*x[1], 2));

	  denom = pow(x[0]*x[0] + y[0]*y[0] + z[0]*z[0], 1.5);
	  //曲率
	  kappa_ = num/denom;

	  num =
			  x[2]*(y[0]*z[1] - z[0]*y[1])
			+ y[2]*(z[0]*x[1] - x[0]*z[1])
			+ z[2]*(x[0]*y[1] - y[0]*x[1]);

	  denom =
			  pow(y[0]*z[1] - z[0]*y[1], 2)
			+ pow(z[0]*x[1] - x[0]*z[1], 2)
			+ pow(x[0]*y[1] - y[0]*x[1], 2);

	  double first_tau_helical_ = num/denom;
	  //捩率
	  tau_helical_ =  first_tau_helical_*step_s_;
}

/**
 *   CalculateCurvatureTorsionWithHyperbolic()
 *
 *   Helical Wave Propagate Motion の曲率と捩率を計算する，
 *
 *   rho = r+a*sech(omega*t_ + phi);
 *    x = rho*cos(t_)
 *    y = rho*sin(t_)
 *    z = (delta/2*pi)*t_
 *
 * */
void HelicalWavePropagateMotion::CalculateCurvatureTorsionWithHyperbolic()
{
	  double num   = 0,
			 denom = 0;

	  double x[3], y[3], z[3];

		x[0] = (-a_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_))
				-sin(t_)*(a_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_);
		y[0] = cos(t_)*(a_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)
				-a_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_);
		z[0] = (M_PI*delta_)/2;
		/*z[0] = (M_PI*delta_)/2-(a_*omega_*sinh(phi_hyperbolic_+omega_*t_))/pow(cosh(phi_hyperbolic_+omega_*t_),2);
*/

		x[1] = 2*a_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+2*a_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				-cos(t_)*(a_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)-a_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
		y[1] = 2*a_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				-2*a_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				-sin(t_)*(a_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)-a_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
		z[1] = 0;
		/*z[1] = (2*a_*pow(omega_,2)*pow(sinh(phi_hyperbolic_+omega_*t_),2))/
		      pow(cosh(phi_hyperbolic_+omega_*t_),3)-(a_*pow(omega_,2))/cosh(phi_hyperbolic_+omega_*t_);
*/
		x[2] = (-6*a_*pow(omega_,3)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-4)*pow(sinh(omega_*t_-phi_hyperbolic_),3))
				-6*a_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+5*a_*pow(omega_,3)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				+3*a_*omega_*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)+sin(t_)*
			    (a_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)+radius_)+3*a_*pow(omega_,2)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
		y[2] = (-6*a_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-4)*pow(sinh(omega_*t_-phi_hyperbolic_),3))
				+6*a_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-3)*pow(sinh(omega_*t_-phi_hyperbolic_),2)
				+5*a_*pow(omega_,3)*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)
				+3*a_*omega_*sin(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-2)*sinh(omega_*t_-phi_hyperbolic_)-cos(t_)*(a_*pow(cosh(omega_*t_-phi_hyperbolic_),-1)
				+radius_)-3*a_*pow(omega_,2)*cos(t_)*pow(cosh(omega_*t_-phi_hyperbolic_),-1);
		z[2] = 0;
		/*z[2] = (5*a_*pow(omega_,3)*sinh(phi_hyperbolic_+omega_*t_))/
		      pow(cosh(phi_hyperbolic_+omega_*t_),2)-(6*a_*pow(omega_,3)*pow(sinh(phi_hyperbolic_+omega_*t_),3))/
		      pow(cosh(phi_hyperbolic_+omega_*t_),4);
*/

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

		first_tau_ = num/denom;
	    tau_hyperbolic_ = first_tau_*step_s_;
}

/***
 *	CalculateSTRelation()
 *
 *  曲線の長さSに沿って，パラメータｔの関係
 * */
double HelicalWavePropagateMotion::CalculateSTRelation(double tt)
{
	  double x1, y1, z1,st;

/*	  x1 = pow(a_,2)*pow(omega_,2)*pow(cosh(omega_*t_+phi_hyperbolic_),-4)*pow(sinh(omega_*t_+phi_hyperbolic_),2);
	  y1 = pow(a_*pow(cosh(omega_*t_+phi_hyperbolic_),-1)+radius_,2);
	  z1 = (pow(M_PI, 2)*pow(delta_, 2))/4;*/
	  //z1 = pow((M_PI*delta_)/2-(a_*omega_*sinh(phi_hyperbolic_+omega_*t_))/pow(cosh(phi_hyperbolic_+omega_*t_),2),2);

	  x1 = (pow(a_,2)*pow(omega_,2)*pow(sinh(phi_hyperbolic_-omega_*tt),2))/pow(cosh(phi_hyperbolic_-omega_*tt),4);
	  y1 = pow(a_,2)/pow(cosh(phi_hyperbolic_-omega_*tt),2);
	  z1 = pow(delta_,2)/(4*pow(M_PI,2));

	  st = sqrt(x1 + y1 + z1);

	  ROS_INFO("> st     = %4.3f	*", st);
	  S_T += st;
	  return st;

}

double HelicalWavePropagateMotion::dxdt(double t, double x)
{

	double x1=0, y1=0, z1=0, st=0;
	double tt = t;

	x1 = (pow(a_,2)*pow(omega_,2)*pow(sinh(phi_hyperbolic_-omega_*tt),2))/pow(cosh(phi_hyperbolic_-omega_*tt),4);
	//x1 = (pow(a_,2)*pow(omega_,2)*pow(sinh(phi_hyperbolic_-omega_*tt),2))/pow(cosh(phi_hyperbolic_-omega_*tt),4);
	//y1 = pow(a_,2)/pow(cosh(phi_hyperbolic_-omega_*tt),2);
	y1 = pow(radius_+a_/cosh(phi_hyperbolic_-omega_*tt),2);
	z1 = pow(delta_,2)/(4*pow(M_PI,2));

	double j=x1 + y1 + z1;
	return sqrt(j); //st;
}

double HelicalWavePropagateMotion::RungeKutta(double x0, double t0, double tn, int n)
{
    int i;
    double x, t, h, d1, d2, d3, d4;
    x = x0;
    t = t0;
    h = (tn - t0) /n;

    // 写文件
    //ofstream outFile;
	//outFile.open("runge_kutta_(r=0.1 n=0.1 a=0.1 omega=4 phi=2pi)_2.csv", ios::out); // 打开模式可省略

    // 漸化式を計算
    for ( i=1; i <= n ; i++){
        t = t0 + i*h;
        d1 = dxdt(t,x);
        d2 = dxdt(t,x + d1*h*0.5);
        d3 = dxdt(t,x + d2*h*0.5);
        d4 = dxdt(t,x + d3*h);
        x += (d1 + 2 * d2 + 2 * d3 + d4)*(h/6.0);
        //printf("x(%f)=%f\n", t, x);
		//outFile << t << ',' << x << endl;
    }
    //outFile.close();
	return x;
}
double HelicalWavePropagateMotion::CalculateIntegral(double a, double b)
{
	double h;
	int m=10;

	double x=0;
	// １区間の幅
	h = (b - a) / m;

    // 初期化
	x = a;  // X 値を a で初期化
    double ss = 0;  // 面積初期化

    // 計算
   for (int k = 1; k <= m-1; k++) {
        x = x + h;
        ss = ss + CalculateSTRelation(x);
        ROS_INFO("> ss     = %4.3f	*", ss);
    }

    double fa = CalculateSTRelation(a);
    double fb = CalculateSTRelation(b);

    ss =  ((fa + fb)*h)/2.0 + ss;
    //ss=(fb-fa);
    S_T = ss;
    ROS_INFO("> ss     = %4.3f	*", ss);
    return ss; //S_T;
}


/**
 * 	CalculateTargetAngle(RobotSpec spec)
 * 	目標角度の計算
 *	snake_model_param.angle.clear()  が必要
 * */
void HelicalWavePropagateMotion::CalculateTargetAngle(RobotSpec spec)
{
	snake_model_param.angle.clear();
	for(int i=0; i<num_link_; i++){
		//(奇数番目)
		if(i%2){
			target_angle_ =
					-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.psi[i]  + snake_model_param.psi_hyper[i]);
		//(偶数番目)
		}else{
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i]*cos(snake_model_param.psi[i]  + snake_model_param.psi_hyper[i]);
		}
		snake_model_param.angle.push_back(target_angle_);
	}
}

void HelicalWavePropagateMotion::CalculateTargetAngleForRolling(RobotSpec spec)
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

		ROS_INFO("> snake_model_param_psi_ = %4.3f",snake_model_param.psi[i]);

        snake_model_param.angle.pop_back();
		snake_model_param.kappa.pop_back();
		//snake_model_param.phi.pop_back();
		snake_model_param.tau.pop_back();
		snake_model_param.psi.pop_back();
	}
	//usleep(1000*5);        // 制御に時間がかかるので1秒寝て待つ
}

void HelicalWavePropagateMotion::CalculateTargetAngle3(RobotSpec spec)
{
	snake_model_param.angle.clear();
	//psi_s = psi_s + tau_helical_-tau_hyperbolic_;

	for(int i=0; i<num_link_; i++){
		if(i%2){ //(奇数番目)
			target_angle_ =
					-2*link_length_*snake_model_param.kappa[i]*sin(snake_model_param.psi[i] + snake_model_param.psi_hyper[i]);

			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);

		}else{  //(偶数番目)
			target_angle_ =
					2*link_length_*snake_model_param.kappa[i]*cos(snake_model_param.psi[i] + snake_model_param.psi_hyper[i]);

			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);
		}
	}
	//usleep(1000*5);        // 制御に時間がかかるので1秒寝て待つ
}
