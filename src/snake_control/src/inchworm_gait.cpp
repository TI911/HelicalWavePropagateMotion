/*
 * inchworm_gait.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: ubuntu-ti
 */


#include "inchworm_gait.h"

void InchwormGait::set_s(double s)
{
	s_ = s;
}


void InchwormGait::InchwormGaitByShift(RobotSpec spec){


	double a = 0.11,
		   b = 0.005,
		   c = 0.05,
		   d = 1,  //d
		   e = 0.015;
	double v = 0.6;


    while(s_>(pre_s_+step_s_)){
       // ROS_INFO("************** 1 ***************");
		while(pre_s_+step_s_ > S_T){

			S_T = pow(c,-1)*(-pow(c,2)*pow(d,2)*pow(e,2)*sin(2*c*t)
					+pow(b,2)*pow(c,2)*sin(2*c*t)-pow(b,2)*sin(2*c*t)
					-16*c*d*pow(e,2)*pow(sin(c*t/2.0),2)-16*a*b*pow(sin(c*t/2.0),2)
					+2*pow(c,3)*pow(d,2)*pow(e,2)*t+4*c*pow(e,2)*t
					+2*pow(b,2)*pow(c,3)*t+2*pow(b,2)*c*t+4*pow(a,2)*c*t)/8.0;


			t += 0.01;
			//new_curve.t = t;

		}
		//修士論文
		 kappa_ = pow(pow(c,2)*pow(d,2)*pow(e,2)*pow(sin(c*t),2)-pow(b,2)*pow(c,2)*pow(sin(c*t),2)+pow(b,2)*pow(sin(c*t),2)-2*c*d*pow(e,2)*sin(c*t)-2*a*b*sin(c*t)+pow(e,2)+pow(b,2)*pow(c,2)+pow(a,2),(-3.0)/2.0)*sqrt(pow(-pow(b,2)*pow(c,2)*pow(sin(c*t),2)+pow(b,2)*pow(sin(c*t),2)-a*b*pow(c,2)*sin(c*t)-2*a*b*sin(c*t)+2*pow(b,2)*pow(c,2)+pow(a,2),2)+pow((e-c*d*e*sin(c*t))*(sin(t)*(a-b*sin(c*t))-b*pow(c,2)*sin(t)*sin(c*t)+2*b*c*cos(t)*cos(c*t))-pow(c,2)*d*e*cos(c*t)*(cos(t)*(a-b*sin(c*t))-b*c*sin(t)*cos(c*t)),2)+pow((e-c*d*e*sin(c*t))*(-cos(t)*(a-b*sin(c*t))+b*pow(c,2)*cos(t)*sin(c*t)+2*b*c*sin(t)*cos(c*t))-pow(c,2)*d*e*cos(c*t)*(sin(t)*(a-b*sin(c*t))+b*c*cos(t)*cos(c*t)),2));

		//修士論文
		double tau = (-pow(b,2)*(c-pow(c,3))*d*e*pow(sin(c*t),3)+(b*(4*a*pow(c,3)+2*a*c)*d*e+pow(b,2)*(pow(c,4)-2*pow(c,2)+1)*e)*pow(sin(c*t),2)-(d*(pow(b,2)*(2*pow(c,5)+4*pow(c,3))*e+(pow(a,2)*c-pow(a,2)*pow(c,3))*e)+b*(4*a*pow(c,2)+2*a)*e)*sin(c*t)-b*(a*pow(c,5)+2*a*pow(c,3))*d*e+pow(b,2)*(2*pow(c,4)+6*pow(c,2))*e+pow(a,2)*e)*pow(pow(-pow(b,2)*pow(c,2)*pow(sin(c*t),2)+pow(b,2)*pow(sin(c*t),2)-a*b*pow(c,2)*sin(c*t)-2*a*b*sin(c*t)+2*pow(b,2)*pow(c,2)+pow(a,2),2)+pow((e-c*d*e*sin(c*t))*(sin(t)*(a-b*sin(c*t))-b*pow(c,2)*sin(t)*sin(c*t)+2*b*c*cos(t)*cos(c*t))-pow(c,2)*d*e*cos(c*t)*(cos(t)*(a-b*sin(c*t))-b*c*sin(t)*cos(c*t)),2)+pow((e-c*d*e*sin(c*t))*(-cos(t)*(a-b*sin(c*t))+b*pow(c,2)*cos(t)*sin(c*t)+2*b*c*sin(t)*cos(c*t))-pow(c,2)*d*e*cos(c*t)*(sin(t)*(a-b*sin(c*t))+b*c*cos(t)*cos(c*t)),2),-1);

		tau_ = tau_ + (tau*step_s_);

		ShiftControlMethod::Shift_Param_Forward(spec);
		CalculateTargetAngle(spec);
		pre_s_ = pre_s_ + step_s_;

    }
}


void InchwormGait::CalculateTargetAngle(RobotSpec spec)
{

    snake_model_param.angle.clear();
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


		//snake_model_param.kappa.pop_back();
		//snake_model_param.phi.pop_back();
		//snake_model_param.tau.pop_back();
		//snake_model_param.psi.pop_back();
	}
	usleep(1000*10);        // 制御に時間がかかるので1秒寝て待つ
}
