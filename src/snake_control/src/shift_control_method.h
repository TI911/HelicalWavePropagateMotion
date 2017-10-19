/*
 * shift_control_method.h
 *
 *  Created on: Feb 22, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_SHIFT_CONTROL_METHOD_H_
#define SNAKE_CONTROL_SRC_SHIFT_CONTROL_METHOD_H_

#include <ros/ros.h>
#include <vector>
#include <stdint.h>
#include <cmath>

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "robot_spec.h"

class ShiftControlMethod {

private:
	/***   サーペノイド曲線用パラメータ    ***/
   typedef struct {
	   double alpha_s;       // 生物機械工学の(3.10)式
	   double alpha;         // くねり角[rad]
	   double l;             // 曲線の1/4周期の長さ[m]
	   double v;

   }SERPENOID_CURVE;

      /***   ヘビの各関節の vector のなかのデータ,  シフトするパラメータ    ***/
   typedef struct{
	   std::vector<double> kappa_hold;
	   std::vector<double> tau_hold;
	   std::vector<double> bias_hold;
	   std::vector<double> psi_hold;
	   std::vector<double> psi_hyper_hold;

   }SHIFT_PARAM;


   typedef struct{
	   std::vector<SHIFT_PARAM> shift_param;
   }HOLD_DATA;

   /***   最終ヘビ各関節へ送るパラメータ    ***/
   typedef struct{
	   std::vector<double> angle;  /*  関節角度 */
	   std::vector<double> bias;   /*  操舵バイアス   */
	   std::vector<double> kappa;  /*  曲率  */
	   std::vector<double> tau;    /*  捩率  */
	   std::vector<double> psi;
	   std::vector<double> psi_hyper;

    }SNAKE_MODEL_PARAM;


public:
	virtual ~ShiftControlMethod(){}

	HOLD_DATA 			hold_data;
	SERPENOID_CURVE     serpenoid_curve;
	SNAKE_MODEL_PARAM   snake_model_param;

	//各种移动模式在继承shift_control_method时，将计算后的曲率，扭率等保存到这里
	double kappa_;
	double tau_helical_;
	double tau_hyperbolic_;
	double first_tau ;

	double tau_;

	double bias_;
	double psi_;
	double psi_hyper;
	//double phi_;
	double angle_;
	double ds_;

	uint8_t NUM_JOINT_;

	void Init(RobotSpec spec);
	void Shift_Param(RobotSpec spec);
	void ShiftParamCurvature(RobotSpec spec);
	void ShiftParamTorsion(RobotSpec spec);
	void ShiftParamPsi(RobotSpec spec);

	void ShiftParamPsiHyper(RobotSpec spec);
	void ShiftParamBias(RobotSpec spec);

	void Calculate_Curvature_Torsion();
	void Calculate_Torsion();
	void Calculate_ST();
	//void Calculate_Angle(RobotSpec spec);

};

#endif /* SNAKE_CONTROL_SRC_SHIFT_CONTROL_METHOD_H_ */
