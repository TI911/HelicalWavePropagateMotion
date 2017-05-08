/*
 * shift_control_method.cpp
 *
 *  Created on: Feb 22, 2017
 *      Author: ubuntu-ti
 */
#include "shift_control_method.h"

void ShiftControlMethod::Init(RobotSpec spec)
{
	int NUM_JOINT = spec.num_joint();
	hold_data.shift_param.resize(NUM_JOINT);

	int max_hold_num = 28;     	 //角度を保持する数 角度を保持する数    //デックの大きさ

    /*  初期値0をデックに追加しておく   */
   	for(int i=0; i<NUM_JOINT; i++){
   		//hold_data.shift_param[i].bias_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].kappa_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].tau_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].psi_hold.resize(max_hold_num, 0);
   		hold_data.shift_param[i].psi_hyper_hold.resize(max_hold_num, 0);
   	}

    /***  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  ***/
   	snake_model_param.angle.resize(NUM_JOINT, 0);
   	//snake_model_param.bias.resize(NUM_JOINT, 0);
  	snake_model_param.kappa.resize(NUM_JOINT, 0);
  	snake_model_param.tau.resize(NUM_JOINT, 0);
  	snake_model_param.psi.resize(NUM_JOINT, 0);
    snake_model_param.psi_hyper.resize(NUM_JOINT, 0);
}

void ShiftControlMethod::Shift_Param(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;

	/***  kappaとtauを先頭ユニットのDEQUEの先頭に入れる  ***/
	/*** 先頭デックの最初の要素に現在κ...を追加 (デックの長さ前より＋１になる)  ***/
	//hold_data.shift_param[0].bias_hold.insert(hold_data.shift_param[0].bias_hold.begin(), bias_);
	hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), kappa_);
	hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), tau_);
	hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), psi_);
	hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper);

	/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();

	for(int i=1; i<NUMOFLINK; i++){
          hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(), hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
          hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(), hold_data.shift_param[i-1].tau_hold[hold_num-1]);
          //hold_data.shift_param[i].bias_hold.insert(hold_data.shift_param[i].bias_hold.begin(), hold_data.shift_param[i-1].bias_hold[hold_num-1]);
          hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i-1].psi_hold[hold_num-1]);
          hold_data.shift_param[i].psi_hyper_hold.insert(hold_data.shift_param[i].psi_hyper_hold.begin(), hold_data.shift_param[i-1].psi_hyper_hold[hold_num-1]);
	}

	/*  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.kappa.insert(snake_model_param.kappa.begin(), hold_data.shift_param[i].kappa_hold[hold_num-1]);
   	    snake_model_param.tau.insert(snake_model_param.tau.begin(), hold_data.shift_param[i].tau_hold[hold_num-1]);
   	    //snake_model_param.bias.insert(snake_model_param.bias.begin(), hold_data.shift_param[i].bias_hold[hold_num-1]);
   	    snake_model_param.psi.insert(snake_model_param.psi.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
        snake_model_param.psi_hyper.insert(snake_model_param.psi_hyper.begin(), hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);

		hold_data.shift_param[i].kappa_hold.pop_back();
		hold_data.shift_param[i].tau_hold.pop_back();
		//hold_data.shift_param[i].bias_hold.pop_back();
		hold_data.shift_param[i].psi_hold.pop_back();
		hold_data.shift_param[i].psi_hyper_hold.pop_back();
	}
}

void ShiftControlMethod::ShiftParamCurvature(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;
	hold_data.shift_param[0].kappa_hold.insert(hold_data.shift_param[0].kappa_hold.begin(), kappa_);
	/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].kappa_hold.insert(hold_data.shift_param[i].kappa_hold.begin(),
				hold_data.shift_param[i-1].kappa_hold[hold_num-1]);
	}
	/*  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.kappa.insert(snake_model_param.kappa.begin(),
				hold_data.shift_param[i].kappa_hold[hold_num-1]);

		hold_data.shift_param[i].kappa_hold.pop_back();
	}
}

void ShiftControlMethod::ShiftParamTorsion(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;
	/***  kappaとtauを先頭ユニットのDEQUEの先頭に入れる  ***/
	/*** 先頭デックの最初の要素に現在κ...を追加 (デックの長さ前より＋１になる)  ***/
	hold_data.shift_param[0].tau_hold.insert(hold_data.shift_param[0].tau_hold.begin(), tau_);
		/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].tau_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].tau_hold.insert(hold_data.shift_param[i].tau_hold.begin(),
        		  hold_data.shift_param[i-1].tau_hold[hold_num-1]);
	}
	/*  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.tau.insert(snake_model_param.tau.begin(),
				hold_data.shift_param[i].tau_hold[hold_num-1]);

		hold_data.shift_param[i].tau_hold.pop_back();
	}
}

void ShiftControlMethod::ShiftParamPsi(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;

	/***  kappaとtauを先頭ユニットのDEQUEの先頭に入れる  ***/
	/*** 先頭デックの最初の要素に現在κ...を追加 (デックの長さ前より＋１になる)  ***/
	hold_data.shift_param[0].psi_hold.insert(hold_data.shift_param[0].psi_hold.begin(), psi_);
	/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].kappa_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].psi_hold.insert(hold_data.shift_param[i].psi_hold.begin(), hold_data.shift_param[i-1].psi_hold[hold_num-1]);
	}
	/*  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.psi.insert(snake_model_param.psi.begin(), hold_data.shift_param[i].psi_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hold.pop_back();
	}
}

void ShiftControlMethod::ShiftParamPsiHyper(RobotSpec spec)
{
	int NUMOFLINK = spec.num_joint() ;

	/***  kappaとtauを先頭ユニットのDEQUEの先頭に入れる  ***/
	/*** 先頭デックの最初の要素に現在κ...を追加 (デックの長さ前より＋１になる)  ***/
	hold_data.shift_param[0].psi_hyper_hold.insert(hold_data.shift_param[0].psi_hyper_hold.begin(), psi_hyper);
	/***  一つ前の関節のデックの最後のものを次の関節のDEQUEの先頭に追加する  ***/
	int hold_num = (int)hold_data.shift_param[0].psi_hyper_hold.size();
	for(int i=1; i<NUMOFLINK; i++){
		hold_data.shift_param[i].psi_hyper_hold.insert(hold_data.shift_param[i].psi_hyper_hold.begin(), hold_data.shift_param[i-1].psi_hyper_hold[hold_num-1]);
	}
	/*  角度保持デックの末尾のκを読み取る, ヘビの関節に送る  */
	for(int i=0; i<NUMOFLINK; i++){
		snake_model_param.psi_hyper.insert(snake_model_param.psi_hyper.begin(), hold_data.shift_param[i].psi_hyper_hold[hold_num-1]);
		hold_data.shift_param[i].psi_hyper_hold.pop_back();
	}
}
