/*
 * winding_gait.h
 *
 *  Created on: Sep 22, 2016
 *      Author: TI
 */

#ifndef SNAKE_CONTROL_SRC_WINDING_GAIT_H_
#define SNAKE_CONTROL_SRC_WINDING_GAIT_H_

#include <ros/ros.h>
#include <vector>
#include <stdint.h>

#include "robot_spec.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "shift_control_method.h"

class WindingGait: public ShiftControlMethod {
 public:
	virtual ~WindingGait(){}
	WindingGait(RobotSpec spec,
			double min_alpha,
			double max_alpha,
			double min_l,
			double max_l,
			double ds
			){

		ds_ = ds;

		Init(spec);
	}

	//--- 動作
    void WindingInit(RobotSpec spec);
	void Winding(RobotSpec spec);
	void WindingShift(RobotSpec spec);
	void WindingCalcAngle(RobotSpec spec);
	virtual void InitializeShape() {}

	//--- 形状パラメータ変更
	void set_alpha(double alpha);
	void add_alpha(double alpha_add){ set_alpha(serpenoid_curve.alpha + alpha_add); }
	void set_l(double l);
	void add_l(double l_add){ set_l(serpenoid_curve.l + l_add); }
	void set_v(double v);
	void add_v(double v_add){ set_v(v_add); }

};

#endif /* SNAKE_CONTROL_SRC_WINDING_GAIT_H_ */
