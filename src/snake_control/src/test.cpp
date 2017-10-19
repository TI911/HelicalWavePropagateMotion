/*
 * test.cpp
 *
 *  Created on: oct 23, 2017
 *      Author: ubuntu-ti
 */

#include "test.h"
//#include <math.h>

void Test::set_radius(double radius)
{
	radius_ = radius;
	ROS_INFO("> radius = %4.3f", radius_);
}


void Test::init()
{

}


void Test::CalculateTargetAngle(RobotSpec spec)
{


	double a = 0, b = 0;
        

	for(int i=0; i<num_link_; i++){
		if(i%2){ //(奇数番目)
			if(i<15){
			a = 0.0008*(b*b) - 0.0399*b;
			target_angle_ = atan2(a,b);
			b=b+60;
			}else{target_angle_ =0;}
			
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);

		}else{  //(偶数番目)
			target_angle_ = 0;
			snake_model_param.angle.insert(snake_model_param.angle.begin()+i, target_angle_);
		}

		snake_model_param.angle.pop_back();
	}
	//usleep(1000*5);        // 制御に時間がかかるので1秒寝て待つ
	//snake_model_param.angle.pop_back();
}


