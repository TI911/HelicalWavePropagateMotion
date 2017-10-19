/*
 * winding_gati.cpp
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 */

#include <ros/ros.h>
#include <cmath>
#include "winding_gait.h"

static int init_flag =1;

void WindingGait::set_alpha(double alpha)
{
	serpenoid_curve.alpha = alpha;

	ROS_INFO("*****************************************");
	ROS_INFO("* -->  serpenoid_curve.alpha = [%4.3f] *", serpenoid_curve.alpha);

}

void WindingGait::set_l(double l) {
	serpenoid_curve.l = l;

	ROS_INFO("* -->  serpenoid_curve.l     = [%4.3f] *", serpenoid_curve.l);
}

void WindingGait::set_v(double v){
	//if( v!=serpenoid_curve.v){
		serpenoid_curve.v = v;
	//}else{
		//serpenoid_curve.v = 0;
	//}
	ROS_INFO("* -->  serpenoid_curve.v     = [%4.3f] *", serpenoid_curve.v);
	ROS_INFO("*****************************************");
}

/*
 * @fn
 * @brief SShiftを行う．動作としては捻転するだけ
 * @param double ratio_to_max [-1.0, 1.0] 最大速さに対する割合
 * @paran double time_move [sec] 何秒分動かすか
 * @return なし
 * @detail
*/

void WindingGait::Winding(RobotSpec spec)
{

}

void WindingGait::WindingShift(RobotSpec spec)
{

    /***　先頭UNITのyaw,pitch 軸のkappa を計算する  ***/

    serpenoid_curve.alpha_s = (pre_s_ * M_PI / (2 * serpenoid_curve.l));

    double  new_target_kappa_yaw    = (M_PI * serpenoid_curve.alpha / (2 * serpenoid_curve.l)) * sin(serpenoid_curve.alpha_s);
    double new_target_kappa_pitch   = 0;
    double  new_target_bias         = 0;

    kappa_ = new_target_kappa_yaw;
    tau_   = new_target_kappa_pitch;
    ShiftControlMethod::Shift_Param(spec);

}


void WindingGait::WindingCalcAngle(RobotSpec spec)
{
	double target_angle = 0;
	//snake_model_param.angle.resize(spec.num_joint());
    for(int i=0; i<spec.num_joint(); i++){
	       //(奇数番目)
    	if(i%2){
    		target_angle = snake_model_param.bias[i] + 2*ds_ * snake_model_param.kappa[i]*sin(snake_model_param.tau[i] + snake_model_param.psi[i]);
    		snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);

    		//ROS_INFO("---------------------------------------");
    		//ROS_INFO("|   target_angle [%d] --> data[%4.3f] |", i, target_angle*(M_PI/180.0));
    		//ROS_INFO("|   snake_model_param.kappa.size[%d] --> data[%4.3f] |", snake_model_param.kappa.size(), snake_model_param.kappa[i]);

        //(偶数番目)
    	}else{
      		target_angle = snake_model_param.bias[i] + 2 * ds_ * snake_model_param.kappa[i]*cos(snake_model_param.tau[i] + snake_model_param.psi[i]);
      		snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle);

      		//ROS_INFO("---------------------------------------");
      		//ROS_INFO("|   target_angle [%d] --> data[%4.3f] |", i, target_angle*(M_PI/180.0));
      		//ROS_INFO("|   snake_model_param.kappa.size[%d] --> data[%4.3f] |", snake_model_param.kappa.size(), snake_model_param.kappa[i]);

        }
		//snake_model_param.angle.pop_back();
        snake_model_param.angle.pop_back();
		snake_model_param.kappa.pop_back();
		//snake_model_param.phi.pop_back();
		snake_model_param.tau.pop_back();
		snake_model_param.psi.pop_back();
		snake_model_param.psi_hyper.pop_back();
    }

}
