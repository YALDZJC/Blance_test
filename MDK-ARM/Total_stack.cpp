#pragma once

#include "def_variable.h"
#include "Total_stack.h"
#include "cmsis_os.h"


struct SEND_GRAPHIC_QUEUE;//发送数据队列
extern SEND_GRAPHIC_QUEUE send_graphic_queue;
uint8_t send_str2[64];

/***************************函数声明*********************************/

//主跑初始化
void Total_tasks_Init();//****

//主跑函数
void Total_tasks_Run();

//急停模式控制所有pid
void control_pid_0_speed();

//清空数据数据
void Clear_ALL_Data();

//获取云台到底盘数据初始化
void Get_Gimbal_to_Chassis_Init();

//获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart);

//发送底盘3508数据
void Send_CHASSIS_3508_CAN()
{
	//发送
	RM_FDorCAN_Send(&hfdcan1,SEND_MOTOR_ID_3508,msd_CHASSIS_3508_2006.Data);
}
//发送底盘6020数据
void Send_CHASSIS_6020_CAN()
{
	//发送
	RM_FDorCAN_Send(&hfdcan1,SEND_MOTOR_ID_6020,msd_CHASSIS_6020.Data);
}

//发送云台3508数据
void Send_GIMBAL_3508_CAN()
{
	//发送
	RM_FDorCAN_Send(&hfdcan2,SEND_MOTOR_ID_3508,msd_GIMBAL_3508_2006.Data);
}
//发送云台6020数据
void Send_GIMBAL_6020_CAN()
{
	//发送
	RM_FDorCAN_Send(&hfdcan2,SEND_MOTOR_ID_6020,msd_GIMBAL_6020.Data);
}

//发送大喵电机
void Send_L_LEG_DM_CAN()
{


}

//发送大喵电机
void Send_R_LEG_DM_CAN()
{
	RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_1.send_data);//发送
}

//急停模式控制所有pid
void control_pid_0_speed()
{

}

//清空数据数据
void Clear_ALL_Data()
{ 
	
}



//主跑初始化
void Total_tasks_Init()
{
	RM_FDorCAN_Init();//can配置初始化
	
	rmClicker.Init();//遥控器串口配置初始化
				
	pm01.PM01Init();
	
	HAL_Delay(10);
	
	//记录上一次时间
	uint64_t time_adrc = HAL_GetTick();
	
	//陀螺仪初始化
//	INS_Init();
//	DWT_Init(480);
//	while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
//	{
//	  ;
//	}
//	Power_OUT1_ON;//imu初始化完成，可控电源打开，led灯亮
//	Power_OUT2_ON;
	
	//adrc收敛期
	while(1)
	{

		if(HAL_GetTick() - time_adrc > 500)
		{
			break;
		}
	}
}

//can_filo0中断接收
FDCAN_RxHeaderTypeDef CHASSIS_RxHeader;	//can接收数据
uint8_t CHASSIS_RxHeaderData[8] = { 0 };
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	//接受信息 
	HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &CHASSIS_RxHeader, CHASSIS_RxHeaderData);

	if(hfdcan == &hfdcan1)
	{
		R_joint_2.parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
		R_joint_3.parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
		R_Wheel.parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
	}
}

//can_filo1中断接收
FDCAN_RxHeaderTypeDef GIMBAL_RxHeader;	//can接收数据
uint8_t GIMBAL_RxHeaderData[8] = { 0 };
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	//接受信息 
	HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &GIMBAL_RxHeader, GIMBAL_RxHeaderData);

	if(hfdcan == &hfdcan2)
	{
		L_joint_0.parse(GIMBAL_RxHeader, GIMBAL_RxHeaderData);
		L_joint_1.parse(GIMBAL_RxHeader, GIMBAL_RxHeaderData);
		L_Wheel.parse(GIMBAL_RxHeader, GIMBAL_RxHeaderData);
	}
}

//UART空闲中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//遥控器解析
}

//UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Get_Gimbal_to_Chassis(huart);
	RM_RefereeSystem::RM_RefereeSystemParse(huart);
}

//获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart)
{
	if(huart == &Send_Gimbal_to_Chassis_Huart)
	{
		
	}
}

void Limit(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}

/*********************电机初始化*********************/
void ChassisL_Init()
{
	for(int j=0;j<10;j++)
	{
		L_joint_0.on(&hfdcan2);

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		L_joint_1.on(&hfdcan2);

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		L_Wheel.on(&hfdcan2);

	  osDelay(1);
	}
}

void ChassisR_Init()
{
	for(int j=0;j<10;j++)
	{
		R_joint_2.on(&hfdcan1);

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		R_joint_3.on(&hfdcan1);

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		R_Wheel.on(&hfdcan1);

	  osDelay(1);
	}
}

/*********************反馈值更新*********************/
void ChassisL_feedback_update()
{
	VMC_leg_L.VMC_data.phi1=pi/2.0f + L_joint_0.DM_Data.position;
	VMC_leg_L.VMC_data.phi4=pi/2.0f + L_joint_1.DM_Data.position;
	
	chassis.PithL= 0 - INS.Pitch;
	chassis.PithGyroL= 0 - INS.Gyro[0];
	
	chassis.Yaw_L = INS.YawTotalAngle;
	chassis.theta_err = 0.0f - (VMC_leg_R.VMC_data.theta + VMC_leg_L.VMC_data.theta);
//	chassis.v_filter=(L_Wheel.DM_Data.velocity-R_Wheel.DM_Data.velocity)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
//	chassis.x_filter += chassis.v_filter*((float)3/1000.0f);

//	V_speed.td_quadratic(chassis.v_filter);
}

void ChassisR_feedback_update()
{
	VMC_leg_R.VMC_data.phi1=pi/2.0f + R_joint_2.DM_Data.position;
	VMC_leg_R.VMC_data.phi4=pi/2.0f + R_joint_3.DM_Data.position;
	
	chassis.PithR = INS.Pitch;
	chassis.PithGyroR = INS.Gyro[0];
	
	chassis.leg_tar += RC_RY*rc_dc;
	chassis.v_tar = RC_LY*go_dc;
	chassis.x_tar += chassis.v_tar*0.003;
	chassis.turn_tar += RC_LX*turn_dc;
	chassis.roll_tar += RC_RX*turn_dc;
}

float diffL, diffR;
/*********************控制循环*********************/
void chassisL_control_loop()
{
	VMC_leg_L.Up_Left(INS.Pitch, INS.Gyro[0], ((float)Up_Chassis_Time)*3.0f/1000.0f);
	
	for(int i=0;i<12;i++)
	{
		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0], VMC_leg_L.VMC_data.L0);	
	}
	
	chassis.wheel_T[1] =(LQR_K[0]*(VMC_leg_L.VMC_data.theta-0.0f)
											 +LQR_K[1]*(VMC_leg_L.VMC_data.d_theta-0.0f)
											 +LQR_K[2]*(chassis.x_tar-chassis.x_filter)
											 +LQR_K[3]*(chassis.v_tar-chassis.v_filter)
											 +LQR_K[4]*(chassis.PithL-0.0f)
											 +LQR_K[5]*(chassis.PithGyroL-0.0f));
	
//右边髋关节输出力矩				
	VMC_leg_L.VMC_data.Tp=(LQR_K[6]*(VMC_leg_L.VMC_data.theta-0.0f)
												+LQR_K[7]*(VMC_leg_L.VMC_data.d_theta-0.0f)
												+LQR_K[8]*(chassis.x_tar-chassis.x_filter)
												+LQR_K[9]*(chassis.v_tar-chassis.v_filter)
												+LQR_K[10]*(chassis.PithL-0.0f)
												+LQR_K[11]*(chassis.PithGyroL-0.0f));
	
//	Turn.GetPidPos(Turn_pid, chassis.turn_tar, chassis.Yaw_L, 2);
	Turn_out = Kp * (chassis.turn_tar - chassis.Yaw_L) + Kd * (0 - INS.Gyro[2]);
	chassis.wheel_T[1] = chassis.wheel_T[1] - Turn_out;
	
	Limit(&chassis.wheel_T[1] ,-1, 1);
	Limit(&chassis.leg_tar ,6.5, 18);

	chassis.roll_f0 = roll_Kp*(chassis.roll_tar - INS.Roll) + roll_Kd*(0 - INS.Gyro[1]);
	
	L0_L.GetPidPos(L0_L_pid, chassis.leg_tar/100, VMC_leg_L.VMC_data.L0, 100);
	VMC_leg_L.VMC_data.F0 = FF/arm_cos_f32(VMC_leg_L.VMC_data.theta) + L0_L.pid.cout + chassis.roll_f0;
	VMC_leg_L.ground_detection_L();
	
	theta_err.GetPidPos(K_theta_err, 0, chassis.theta_err, 2);
	
	VMC_leg_L.VMC_data.Tp = VMC_leg_L.VMC_data.Tp + theta_err.pid.cout;
	
	VMC_leg_L.Jacobian();
	
	if(VMC_leg_L.ground_detection_L())
	{
		chassis.wheel_T[1] = 0.0f;
		chassis.x_filter = 0.0f;
		
		chassis.x_tar = chassis.x_filter;
		chassis.turn_tar = chassis.Yaw_L;
	}
}

void chassisR_control_loop()
{
		VMC_leg_R.Up_Right(INS.Pitch, INS.Gyro[0], ((float)Up_Chassis_Time)*3.0f/1000.0f);
		
		for(int i=0;i<12;i++)
		{
			LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0], VMC_leg_R.VMC_data.L0);	
		}
		
		chassis.wheel_T[0] =(LQR_K[0]*(VMC_leg_R.VMC_data.theta-0.0f)
												 +LQR_K[1]*(VMC_leg_R.VMC_data.d_theta-0.0f)
												 +LQR_K[2]*(chassis.x_filter-chassis.x_tar)
												 +LQR_K[3]*(chassis.v_filter-chassis.v_tar)
												 +LQR_K[4]*(chassis.PithR-0.0f)
												 +LQR_K[5]*(chassis.PithGyroR-0.0f));
		
	//右边髋关节输出力矩				
		VMC_leg_R.VMC_data.Tp=(LQR_K[6]*(VMC_leg_R.VMC_data.theta-0.0f)
													+LQR_K[7]*(VMC_leg_R.VMC_data.d_theta-0.0f)
													+LQR_K[8]*(chassis.x_filter-chassis.x_tar)
													+LQR_K[9]*(chassis.v_filter-chassis.v_tar)
													+LQR_K[10]*(chassis.PithR-0.0f)
													+LQR_K[11]*(chassis.PithGyroR-0.0f));
		
		chassis.wheel_T[0] = chassis.wheel_T[0] - Turn_out;
		
		Limit(&chassis.wheel_T[0] ,-1, 1);

//		roll.GetPidPos(K_roll, chassis.roll, INS.Roll, 10);

		L0_R.GetPidPos(L0_L_pid, chassis.leg_tar/100, VMC_leg_R.VMC_data.L0, 100);
		VMC_leg_R.VMC_data.F0 = FF/arm_cos_f32(VMC_leg_R.VMC_data.theta) + L0_R.pid.cout - chassis.roll_f0;
		VMC_leg_R.ground_detection_R();
		
		VMC_leg_R.VMC_data.Tp = VMC_leg_R.VMC_data.Tp + theta_err.pid.cout;
		VMC_leg_R.Jacobian();

		if(VMC_leg_R.ground_detection_R())
		{
			chassis.wheel_T[0] = 0.0f;
		}
}

/*********************左腿任务*********************/
void Chassis_Task_L()
{
  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
	ChassisL_Init();
	
	while(1)
	{	
		ChassisL_feedback_update();
		chassisL_control_loop();

		//遥控器
		if(Emergency_Stop == false)
		{
			//打开电机			
			L_joint_0.ctrl_motor(&hfdcan2, 0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[0]);
			osDelay(Up_Chassis_Time);

			L_joint_1.ctrl_motor(&hfdcan2, 0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[1]);
			osDelay(Up_Chassis_Time);

			L_Wheel.ctrl_motor(&hfdcan2, 0, 0, 0, 0, chassis.wheel_T[1]);
//			L_Wheel.ctrl_motor(&hfdcan2, 0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);
			
		}
		else if(Emergency_Stop == true)
		{
			//打开电机			
			L_joint_0.ctrl_motor(&hfdcan2, 0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);

			L_joint_1.ctrl_motor(&hfdcan2, 0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);

			L_Wheel.ctrl_motor(&hfdcan2, 0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);
			
//			chassis.turn_tar = chassis.total_yaw;
//			chassis.x_tar=chassis.x_filter;
		}
	}
}

/*********************右腿任务*********************/
void Chassis_Task_R()
{
  while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}	
	ChassisR_Init();
	
	while(1)
	{	
		ChassisR_feedback_update();
		chassisR_control_loop();
		
		//遥控器
		if(Emergency_Stop == false)
		{
			//打开电机
			R_joint_2.ctrl_motor(&hfdcan1,0, 0, 0, 0, VMC_leg_R.VMC_data.torque_set[0]);
			osDelay(Up_Chassis_Time);

			R_joint_3.ctrl_motor(&hfdcan1,0, 0, 0, 0, VMC_leg_R.VMC_data.torque_set[1]);
			osDelay(Up_Chassis_Time);

			R_Wheel.ctrl_motor(&hfdcan1,0, 0, 0, 0, chassis.wheel_T[0]);
//			R_Wheel.ctrl_motor(&hfdcan1,0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);
		}
		else if(Emergency_Stop == true)
		{
			//打开电机			
			R_joint_2.ctrl_motor(&hfdcan1,0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);

			R_joint_3.ctrl_motor(&hfdcan1,0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);

			R_Wheel.ctrl_motor(&hfdcan1,0, 0, 0, 0, 0);
			osDelay(Up_Chassis_Time);
		}
	}
}

void DM_Send_Task()
{
	 dir = RM_Clicker::ISDir();
	
		*((float*)&send_str2[0]) = chassis.wheel_T[0];
		*((float*)&send_str2[4]) = chassis.wheel_T[1];

//		*((float*)&send_str2[8]) = VMC_leg_L.VMC_data.theta;
//		*((float*)&send_str2[12]) = VMC_leg_L.VMC_data.d_theta;
//		*((float*)&send_str2[16]) = chassis.PithGyroR;
//		*((float*)&send_str2[20]) = chassis.PithGyroL;
//		*((float*)&send_str2[24]) = chassis_R.PithGyroR;
//		*((float*)&send_str2[28]) = 0;

		*((uint32_t*)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
		HAL_UART_Transmit_DMA(&Send_Usart_Data_Huart, send_str2, sizeof(float) * (7 + 1));
}

void Send_Vofa_Task()
{
//		*((float*)&send_str2[0]) = VMC_leg_L.VMC_data.L0;
//		*((float*)&send_str2[4]) = VMC_leg_L.VMC_data.d_L0;

//		*((float*)&send_str2[8]) = L_joint_0.DM_Data.position;
//		*((float*)&send_str2[12]) = L_joint_1.DM_Data.position;
//		*((float*)&send_str2[16]) = VMC_leg_L.VMC_data.phi4;
////		*((float*)&send_str2[20]) = VMC_leg_L.VMC_data.phi1;
//		*((uint32_t*)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
//		HAL_UART_Transmit_DMA(&Send_Usart_Data_Huart, send_str2, sizeof(float) * (7 + 1));
}



float vel_acc[2]; 
uint32_t OBSERVE_TIME=3;//任务周期是3ms				

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
		memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));

}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel)
{   	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void Kalman_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	static float wr,wl=0.0f;
	static float vrb,vlb=0.0f;
	static float aver_v=0.0f;
		
	xvEstimateKF_Init(&vaEstimateKF);
	
  while(1)
	{  
		wr= -R_Wheel.DM_Data.velocity - INS.Gyro[0]+VMC_leg_R.VMC_data.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb=wr*0.0603f + VMC_leg_R.VMC_data.L0 * VMC_leg_R.VMC_data.d_theta * arm_cos_f32(VMC_leg_R.VMC_data.theta)+VMC_leg_R.VMC_data.d_L0*arm_sin_f32(VMC_leg_R.VMC_data.theta);//机体b系的速度
		
		wl= -L_Wheel.DM_Data.velocity + INS.Gyro[0]+VMC_leg_L.VMC_data.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb=wl*0.0603f+VMC_leg_L.VMC_data.L0*VMC_leg_L.VMC_data.d_theta*arm_cos_f32(VMC_leg_L.VMC_data.theta)+VMC_leg_L.VMC_data.d_L0*arm_sin_f32(VMC_leg_L.VMC_data.theta);//机体b系的速度
		
		aver_v=(vrb-vlb)/2.0f;//取平均
    xvEstimateKF_Update(&vaEstimateKF,INS.MotionAccel_b[1],aver_v);
		
		//原地自转的过程中v_filter和x_filter应该都是为0
		chassis.v_filter=vel_acc[0];//得到卡尔曼滤波后的速度
		chassis.x_filter=chassis.x_filter+chassis.v_filter*((float)OBSERVE_TIME/1000.0f);
		
	//如果想直接用轮子速度，不做融合的话可以这样
	//chassis_move.v_filter=(chassis_move.wheel_motor[0].para.vel-chassis_move.wheel_motor[1].para.vel)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
	//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
		osDelay(OBSERVE_TIME);
	}
}


