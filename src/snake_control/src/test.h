/*
 * test.h
 *
 *  Created on: 10 05, 2017
 *      Author: ubuntu-ti
 */


#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include "robot_spec.h"
#include "shift_control_method.h"

class Test: public ShiftControlMethod{
 public:
	virtual ~Test(){}
	Test(RobotSpec spec,

			double ds){

		ds_ = ds;
		num_link_ = spec.num_joint();
		target_angle_ = 0;
		link_length_ = 0.07;
		flag_ = 0;

		Init(spec);
	}

    //--- 動作
    void CalculateTargetAngle(RobotSpec spec);
    void init();

	//--- 形状パラメータ変更
	void set_radius(double radius);
	void add_radius(double radius_add){ set_radius(radius_ + radius_add);}

	double target_angle_;

	int num_link_;
	double link_length_;
	int flag_;
	double radius_;

};

