/*
 * dxl_control_node.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: ubuntu-ti
 */

#include "dxl_control.h"

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}

dynamixel::PortHandler *portHandler;
dynamixel::PacketHandler *packetHandler;

ros::Subscriber DynamixelControl::sub_joint_command_;
ros::Subscriber DynamixelControl::sub_joint_command4V2_;

ros::Publisher  DynamixelControl::pub_joint_data_;
ros::Publisher  DynamixelControl::pub_joint_data4V2_;


uint8_t DXL_ID =6;


snake_msgs::snake_joint_data4V2 joint_data4V2;
snake_msgs::snake_joint_data    joint_data;


void DynamixelControl::CallBackOfJointCommand(const snake_msgs::snake_joint_command joint_command)
{
/*	  snake_msgs::snake_joint_command jc = joint_command;  // 名前を短く
	  if (joint_command.joint_index >= NUM_JOINT) {        // 対象が不適
	    ROS_WARN("joint_index %d is not exist.", joint_command.joint_index);
	    return;
	  }

	  if(joint_command.ping){
		  DynamixelPing(joint_command.joint_index);
	  }

	  if(joint_command.change_mode_to_active and joint_command.target_all){
		  DynamixelTorqueEnable();
	  }
	  if(joint_command.change_mode_to_free and joint_command.target_all){
		  DynamixelTorqueDisable();
	  }


	  if(joint_command.set_position){
		  uint8_t joint_index = joint_command.joint_index;
		  float target_position = (joint_command.target_position*4095)/360+2048;

		  DynamxielGoalPosition(joint_index, target_position);
	  }

	  if(true){


	  }*/

}

void DynamixelControl::CallBackOfJointCommand4V2(const snake_msgs::snake_joint_command4V2 joint_command4V2)
{
/*	  snake_msgs::snake_joint_command4V2 jc = joint_command4V2;  // 名前を短く
	  snake_msgs::snake_joint_data4V2 joint_data4V2;
	  snake_msgs::snake_joint_data    joint_data;*/

	snake_msgs::snake_joint_command4V2 jc = joint_command4V2;  // 名前を短く

	if (joint_command4V2.joint_index >= NUM_JOINT) {        // 対象が不適
		ROS_WARN("joint_index %d is not exist.", joint_command4V2.joint_index);
	    return;
	}

    /*if(joint_command4V2.ping){
		  DynamixelPing(joint_command4V2.joint_index);
	}*/

	if(joint_command4V2.change_mode_to_active and joint_command4V2.target_all){
		DynamixelTorqueEnable();
	}
	if(joint_command4V2.change_mode_to_free and joint_command4V2.target_all){
		DynamixelTorqueDisable();
	}


	if(joint_command4V2.set_position){

		DynamxielGoalPosition(joint_command4V2);
	}


	DynamixelReadPosition(DXL_ID);
	//joint_data4V2.joint_index = DXL_ID;
	//joint_data4V2.position.push_back((present_position*360)/4096-180);  // [deg]


	DynamixelReadPresentCurrent(DXL_ID);
	//joint_data.joint_index = DXL_ID;
	//joint_data4V2.current.push_back(present_current);
	//pub_joint_data_.publish(joint_data);

	pub_joint_data4V2_.publish(joint_data4V2);
	//joint_data4V2.position.clear();

	joint_data4V2.position.clear();
	joint_data4V2.current.clear();


/*	if(DXL_ID > NUM_JOINT-2){

		pub_joint_data4V2_.publish(joint_data4V2);

		joint_data4V2.value.clear();
		DXL_ID=0;

	}*/
}


int DynamixelControl::DynamixelOpenPort()
{
	// Initialize PortHandler instance
	// Set the port path
	// Get methods and members of PortHandlerLinux or PortHandlerWindows
	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

	// Initialize PacketHandler instance
	// Set the protocol version
	// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

	// Open port
	if (portHandler->openPort())
	{
		ROS_INFO("Succeeded to open the port!\n");
	}
	else
	{
		ROS_INFO("Failed to open the port!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		ROS_INFO("Succeeded to change the baudrate!\n");
	}
	else
	{
		ROS_INFO("Failed to change the baudrate!\n");
		ROS_INFO("Press any key to terminate...\n");
		getch();
		return 0;
	}
	return 1;
}

int DynamixelControl::DynamixelTorqueEnable()
{
	int dxl_comm_result = COMM_TX_FAIL;   // Communication result
	uint8_t dxl_error = 0;                // Dynamixel error

   	// Enable Dynamixel#i Torque

   	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ADDR_XH_BROADCAST_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

   	if (dxl_comm_result != COMM_SUCCESS)
   	{
   		packetHandler->printTxRxResult(dxl_comm_result);
   		return 0;
   	}
   	else if (dxl_error != 0)
   	{
   		packetHandler->printRxPacketError(dxl_error);
   		return 0;
   	}
   	else
   	{
   		ROS_INFO("DXL has been successfully connected and TORQUE ON");
   	}
   	return 1;
}

int DynamixelControl::DynamixelTorqueDisable()
{
	int dxl_comm_result = COMM_TX_FAIL;     // Communication result
	uint8_t dxl_error = 0;                  // Dynamixel error

	// Disable Dynamixel#1 Torque
	dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ADDR_XH_BROADCAST_ID, ADDR_XH_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}else
	{
		ROS_INFO("DXL has been successfully connected and TORQUE OFF");
	}
}

int DynamixelControl::DynamixelPing(uint8_t joint_index)
{
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	uint16_t dxl_model_number;                      // Dynamixel model number

	// Try to ping the Dynamixel
	// Get Dynamixel model number

	dxl_comm_result = packetHandler->ping(portHandler, joint_index, &dxl_model_number, &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS){
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0){
		packetHandler->printRxPacketError(dxl_error);
	}

	ROS_INFO("[ID:%03d] ping Succeeded. Dynamixel model number : %d",joint_index, dxl_model_number);

	// Close port
	portHandler->closePort();

	return 0;
}


/*  モータを目標位置まで動かす, SYNC でGoal Positionのデータを送る
  *  まとめて送るではなく，一つ一つで送る
 *  id: モータのID
 *  goalPos: ゴールポposition = 0;ジション
 *
 * */
int DynamixelControl::DynamxielGoalPosition(snake_msgs::snake_joint_command4V2 joint_command4V2)
{
	// Initialize GroupSyncWrite instance
	dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_XH_GOAL_POSITION, LEN_XH_GOAL_POSITION);
	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);

	int  dxl_comm_result      = COMM_TX_FAIL;         // Communication result
	bool dxl_addparam_result  = false;                // addParam result
	uint8_t param_goal_position[4];
	float target_position = 0;

/*	if(goalPos < 1024) goalPos=1024;
	if(goalPos > 3072) goalPos=3072;

	// Allocate goal position value into byte array
	param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPos));
	param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPos));
	param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPos));
	param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPos));

	// Add parameter storage for Dynamixel#id goal position
	dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_position);
	if (dxl_addparam_result != true)
	{
		fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", id);
		return 0;
	}
	// SyncWrite goal position
	dxl_comm_result = groupSyncWrite.txPacket();

	if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

	// Clear bulkWrite parameter storage
	groupSyncWrite.clearParam();
	return 1;*/


	for(int i=0; i<joint_command4V2.target_position.size()+1; i++){
		//uint8_t joint_index = i;
		target_position = joint_command4V2.target_position[i]*4095/360+2048;

		if(target_position < 1024) target_position=1024;
		if(target_position > 3072) target_position=3072;

		// Allocate goal position value into byte array
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(target_position));
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(target_position));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(target_position));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(target_position));

		// Add parameter storage for Dynamixel#id goal position
		dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);

		if (dxl_addparam_result != true){
			fprintf(stderr, "[ID:%03d] groupBulkWrite addparam failed", i);
			///return 0;
		}
	}

	dxl_comm_result = groupSyncWrite.txPacket();

	if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);

	// Clear bulkWrite parameter storage
	groupSyncWrite.clearParam();
	return 1;
}


int32_t DynamixelControl::DynamixelReadPosition(int id)
{
	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);

	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	bool dxl_addparam_result = false;               // addParam result
	bool dxl_getdata_result = false;                // GetParam result
	int32_t dxl_present_position = 0;

	for(int8_t ID=0; ID<id; ID++){
		// Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = groupSyncRead.addParam(ID);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", ID);
			return 0;
		}
	}


	// Syncread present position
	dxl_comm_result = groupSyncRead.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);


	for(int8_t ID=0; ID<id; ID++){
		// Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);
		if (dxl_getdata_result != true)
		{
		  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ID);
		  return 0;
		}

		// Get Dynamixel#1 present position value
		dxl_present_position = groupSyncRead.getData(ID, ADDR_XH_PRESENT_POSITION, LEN_XH_PRESENT_POSITION);

		joint_data4V2.joint_index = DXL_ID;
		joint_data4V2.position.push_back((dxl_present_position*360)/4096-180);  // [deg]

	}

	return 1;
/*
	//int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	//bool dxl_addparam_result = false;                // addParam result
	//bool dxl_getdata_result = false;                 // GetParam result
	//int32_t dxl_present_position = 0;

	//snake_msgs::snake_joint_data4V2 joint_data4V2;


    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_XH_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }

    return dxl_present_position;*/

}

int32_t DynamixelControl::DynamixelReadPresentCurrent(int id)
{
	// Initialize GroupSyncWrite instance
	//dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_XH_GOAL_POSITION, LEN_XH_GOAL_POSITION);
	// Initialize Groupsyncread instance for Present Position
	dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_XH_PRESENT_CURRENT, LEN_XH_PRESENT_CURRENT);

	int8_t dxl_present_current = 0;
	int8_t  dxl_comm_result      = COMM_TX_FAIL;             // Communication result
	bool    dxl_addparam_result  = false;               // addParam result
	bool    dxl_getdata_result   = false;                // GetParam result

	for(int8_t ID=0; ID<id; ID++){
		// Add parameter storage for Dynamixel#1 present position value
		dxl_addparam_result = groupSyncRead.addParam(ID);
		if (dxl_addparam_result != true)
		{
			fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", ID);
			return 0;
		}
	}


	// Syncread present position
	dxl_comm_result = groupSyncRead.txRxPacket();
	if (dxl_comm_result != COMM_SUCCESS) packetHandler->printTxRxResult(dxl_comm_result);


	for(int8_t ID=0; ID<id; ID++){
		// Check if groupsyncread data of Dynamixel#1 is available
		dxl_getdata_result = groupSyncRead.isAvailable(ID, ADDR_XH_PRESENT_CURRENT, LEN_XH_PRESENT_CURRENT);
		if (dxl_getdata_result != true)
		{
		  fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", ID);
		  return 0;
		}

		// Get Dynamixel#1 present position value
		dxl_present_current = groupSyncRead.getData(ID, ADDR_XH_PRESENT_CURRENT, LEN_XH_PRESENT_CURRENT);

		joint_data4V2.current.push_back(dxl_present_current*2.69); //[mA]
		//joint_data4V2.joint_index = DXL_ID;
		//joint_data4V2.position.push_back((dxl_present_position*360)/4096-180);  // [deg]

	}

	groupSyncRead.clearParam();
	return 1;

/*
	int dxl_comm_result = COMM_TX_FAIL;             // Communication result
	uint8_t dxl_error = 0;                          // Dynamixel error
	//bool dxl_addparam_result = false;                // addParam result
	//bool dxl_getdata_result = false;                 // GetParam result
	int32_t dxl_present_current = 0;



    // Read present position
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_XH_PRESENT_CURRENT, (uint32_t*)&dxl_present_current, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }

    return dxl_present_current;*/

}
