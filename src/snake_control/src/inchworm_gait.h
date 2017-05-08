/*
 * inchworm_gait.h
 *
 *  Created on: Apr 19, 2017
 *      Author: ubuntu-ti
 */

#ifndef SNAKE_CONTROL_SRC_INCHWORM_GAIT_H_
#define SNAKE_CONTROL_SRC_INCHWORM_GAIT_H_

#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

class InchwormGait: public ShiftControlMethod{

public:
	virtual ~InchwormGait(){}

	InchwormGait(RobotSpec spec, double ds){

		ds_ = ds;
		pre_s_ = 0;
		step_s_= ds/28;
		s_ = 0;
		S_T = 0;
		t = 0;

		num_link_ = spec.num_joint();
		target_angle_ = 0;
		link_length_ = 0.07;

		Init(spec);
	}


	void set_s(double s);
	void add_s(double add_s){set_s(add_s + s_); }

	void InchwormGaitByShift(RobotSpec spec);

	void CalculateTargetAngle(RobotSpec spec);


	double s_, S_T, t;
	double pre_s_;
	double step_s_;
	double target_angle_;
	int num_link_;
	double link_length_;

};





#endif /* SNAKE_CONTROL_SRC_INCHWORM_GAIT_H_ */
