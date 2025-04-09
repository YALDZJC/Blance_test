#pragma once

#include "Total_stack.h"
#include "BMMotor.hpp"
#include "cmsis_os.h"
#include "def_variable.h"

struct SEND_GRAPHIC_QUEUE; // 发送数据队列
extern SEND_GRAPHIC_QUEUE send_graphic_queue;
uint8_t send_str2[64];

/***************************函数声明*********************************/

// 主跑初始化
void Total_tasks_Init(); //****

// 主跑函数
void Total_tasks_Run();

// 急停模式控制所有pid
void control_pid_0_speed();

// 清空数据数据
void Clear_ALL_Data();

// 获取云台到底盘数据初始化
void Get_Gimbal_to_Chassis_Init();

// 获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef *huart);

// 发送底盘3508数据
void Send_CHASSIS_3508_CAN()
{
    // 发送
    RM_FDorCAN_Send(&hfdcan1, SEND_MOTOR_ID_3508, msd_CHASSIS_3508_2006.Data);
}
// 发送底盘6020数据
void Send_CHASSIS_6020_CAN()
{
    // 发送
    RM_FDorCAN_Send(&hfdcan1, SEND_MOTOR_ID_6020, msd_CHASSIS_6020.Data);
}

// 发送云台3508数据
void Send_GIMBAL_3508_CAN()
{
    // 发送
    RM_FDorCAN_Send(&hfdcan2, SEND_MOTOR_ID_3508, msd_GIMBAL_3508_2006.Data);
}
// 发送云台6020数据
void Send_GIMBAL_6020_CAN()
{
    // 发送
    RM_FDorCAN_Send(&hfdcan2, SEND_MOTOR_ID_6020, msd_GIMBAL_6020.Data);
}

// 发送大喵电机
void Send_L_LEG_DM_CAN()
{
}

// 发送大喵电机
void Send_R_LEG_DM_CAN()
{
    RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_1.send_data); // 发送
}

// 急停模式控制所有pid
void control_pid_0_speed()
{
}

// 清空数据数据
void Clear_ALL_Data()
{
}

// 主跑初始化
void Total_tasks_Init()
{
    RM_FDorCAN_Init(); // can配置初始化

    rmClicker.Init(); // 遥控器串口配置初始化

    pm01.PM01Init();

    HAL_Delay(10);

    // 记录上一次时间
    uint64_t time_adrc = HAL_GetTick();

    // 陀螺仪初始化
    //	INS_Init();
    //	DWT_Init(480);
    //	while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
    //	{
    //	  ;
    //	}
    //	Power_OUT1_ON;//imu初始化完成，可控电源打开，led灯亮
    //	Power_OUT2_ON;

    // adrc收敛期
    while (1)
    {

        if (HAL_GetTick() - time_adrc > 500)
        {
            break;
        }
    }
}

// can_filo0中断接收
FDCAN_RxHeaderTypeDef CHASSIS_RxHeader; // can接收数据
uint8_t CHASSIS_RxHeaderData[8] = {0};
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 接受信息
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CHASSIS_RxHeader, CHASSIS_RxHeaderData);

    BSP::Motor::BM::MotorP1010R.Parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
    BSP::Motor::BM::MotorP1010L.Parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);

    BSP::Motor::BM::MotorM1505R.Parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
    BSP::Motor::BM::MotorM1505L.Parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
}

// can_filo1中断接收
FDCAN_RxHeaderTypeDef GIMBAL_RxHeader; // can接收数据
uint8_t GIMBAL_RxHeaderData[8] = {0};
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // 接受信息
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &GIMBAL_RxHeader, GIMBAL_RxHeaderData);

    if (hfdcan == &hfdcan2)
    {
    }
}

// UART空闲中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    rmClicker.Parse(huart, Size); // 遥控器解析
}

// UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    Get_Gimbal_to_Chassis(huart);
    RM_RefereeSystem::RM_RefereeSystemParse(huart);
}

// 获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef *huart)
{
    if (huart == &Send_Gimbal_to_Chassis_Huart)
    {
    }
}

void Limit(float *in, float min, float max)
{
    if (*in < min)
    {
        *in = min;
    }
    else if (*in > max)
    {
        *in = max;
    }
}

/*********************电机初始化*********************/
void ChassisL_Init()
{
    BSP::Motor::BM::MotorP1010L.On(&hfdcan1);

    osDelay(1);
}

void ChassisR_Init()
{
    BSP::Motor::BM::MotorP1010R.On(&hfdcan1);

    osDelay(1);
}

float pitch_dade = 0.01;//??
/*********************反馈值更新*********************/
void ChassisL_feedback_update()
{
    VMC_leg_L.VMC_data.phi1 = pi / 2.0f + BSP::Motor::BM::MotorP1010L.getAddAngleRad(2);
    VMC_leg_L.VMC_data.phi4 = pi / 2.0f + BSP::Motor::BM::MotorP1010L.getAddAngleRad(1);

    chassis.PithL = 0.0 - (INS.Pitch - pitch_dade);
    chassis.PithGyroL = 0 - INS.Gyro[0];

    chassis.Yaw_L = INS.YawTotalAngle;
    chassis.theta_err = 0.0f - (VMC_leg_R.VMC_data.theta + VMC_leg_L.VMC_data.theta);
    //	chassis.v_filter=(L_Wheel.DM_Data.velocity-R_Wheel.DM_Data.velocity)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
    //	chassis.x_filter += chassis.v_filter*((float)3/1000.0f);

    //	V_speed.td_quadratic(chassis.v_filter);
}

void ChassisR_feedback_update()
{
    //		float forword_Rad = Zero_crossing_processing(0, BSP::Motor::BM::MotorP1010R.getAngleRad(1), 3.14);
    //		float back_Rad = Zero_crossing_processing(0, BSP::Motor::BM::MotorP1010R.getAngleRad(2), 3.14);

    VMC_leg_R.VMC_data.phi1 = pi / 2.0f + BSP::Motor::BM::MotorP1010R.getAddAngleRad(2);
    VMC_leg_R.VMC_data.phi4 = pi / 2.0f + BSP::Motor::BM::MotorP1010R.getAddAngleRad(1);

    chassis.PithR = INS.Pitch - pitch_dade;
    chassis.PithGyroR = INS.Gyro[0];
	
		chassis.roll=INS.Roll;

    chassis.leg_tar += RC_RY * 1.00000001e-07;
    Limit(&chassis.leg_tar, 0.15, 0.35);

    chassis.v_tar = RC_LY * 0.001;
    chassis.x_tar += chassis.v_tar * 0.001;

    chassis.turn_tar -= RC_LX * 0.00001;
}

float diffL, diffR;
/*********************控制循环*********************/
bool is_Tp;
float set_R_TP;
float set_L_TP;
float roll_Kp, roll_Kd;

float max_TP = 5.0f;
void chassisL_control_loop()
{
    VMC_leg_L.Up_Left(INS.Pitch, INS.Gyro[0], ((float)Up_Chassis_Time) * 1.0f / 1000.0f);

    for (int i = 0; i < 12; i++)
    {
//        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], VMC_leg_L.VMC_data.L0);
    }
//		td_theta_L.td_quadratic(VMC_leg_L.VMC_data.theta);
    chassis.wheel_T[1] =
        (LQR_K[0] * (VMC_leg_L.VMC_data.theta - 0.0f) + LQR_K[1] * (VMC_leg_L.VMC_data.d_theta - 0.0f) +
         LQR_K[2] * (chassis.x_tar - chassis.x_filter) + LQR_K[3] * (chassis.v_tar - chassis.v_filter) +
         LQR_K[4] * (chassis.PithL - 0.0f) + LQR_K[5] * (chassis.PithGyroL - 0.0f));

    // 右边髋关节输出力矩
    VMC_leg_L.VMC_data.Tp =
        (LQR_K[6] * (VMC_leg_L.VMC_data.theta - 0.0f) + LQR_K[7] * (VMC_leg_L.VMC_data.d_theta - 0.0f) +
         LQR_K[8] * (chassis.x_tar - chassis.x_filter) + LQR_K[9] * (chassis.v_tar - chassis.v_filter) +
         LQR_K[10] * (0.0f - chassis.PithL) + LQR_K[11] * (0.0f - chassis.PithGyroL));

    Turn.GetPidPos(Turn_pid, chassis.turn_tar, chassis.Yaw_L, 10);
//    Turn_out = Kp * (chassis.turn_tar - chassis.Yaw_L) + Kd * (0 - INS.Gyro[2]);
		chassis.roll_f0 = roll_Kp * (0-chassis.roll) - roll_Kd * (0 - INS.Gyro[1]);
    chassis.wheel_T[1] = chassis.wheel_T[1] - Turn.pid.cout;

    Limit(&chassis.wheel_T[1], -5, 5);

    L0_L.GetPidPos(L0_L_pid, chassis.leg_tar, VMC_leg_L.VMC_data.L0, 1000);
    VMC_leg_L.VMC_data.F0 = FF / arm_cos_f32(VMC_leg_L.VMC_data.theta) + L0_L.pid.cout - chassis.roll_f0;

    theta_err.GetPidPos(K_theta_err, 0, chassis.theta_err, 10);

    if (is_Tp == true)
        VMC_leg_L.VMC_data.Tp = VMC_leg_L.VMC_data.Tp + theta_err.pid.cout;
		else
				VMC_leg_L.VMC_data.Tp = theta_err.pid.cout;

		Limit(&VMC_leg_L.VMC_data.Tp, -max_TP, max_TP);

		
    VMC_leg_L.Jacobian();
}

float out1, out2, out3, out4 ,out5, out6;
void chassisR_control_loop()
{
    VMC_leg_R.Up_Right(INS.Pitch, INS.Gyro[0], ((float)Up_Chassis_Time) * 1.0f / 1000.0f);

    for (int i = 0; i < 12; i++)
    {
//        LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], VMC_leg_R.VMC_data.L0);
    }
//		td_theta_R.td_quadratic(VMC_leg_R.VMC_data.theta);

    chassis.wheel_T[0] =
        (LQR_K[0] * (VMC_leg_R.VMC_data.theta - 0.0f) + LQR_K[1] * (VMC_leg_R.VMC_data.d_theta - 0.0f) +
         LQR_K[2] * (chassis.x_filter - chassis.x_tar) + LQR_K[3] * (chassis.v_filter - chassis.v_tar) +
         LQR_K[4] * (chassis.PithR - 0.0f) + LQR_K[5] * (chassis.PithGyroR - 0.0f));

    // 右边髋关节输出力矩
    VMC_leg_R.VMC_data.Tp =
        (LQR_K[6] * (VMC_leg_R.VMC_data.theta - 0.0f) + LQR_K[7] * (VMC_leg_R.VMC_data.d_theta - 0.0f) +
         LQR_K[8] * (chassis.x_filter - chassis.x_tar) + LQR_K[9] * (chassis.v_filter - chassis.v_tar) +
         LQR_K[10] * (0.0f - chassis.PithR) + LQR_K[11] * (0.0f - chassis.PithGyroR));
		
		out1 = LQR_K[6] * (VMC_leg_R.VMC_data.theta - 0.0f) + LQR_K[7] * (VMC_leg_R.VMC_data.d_theta - 0.0f);
		out2 = LQR_K[8] * (chassis.x_tar - chassis.x_filter) + LQR_K[9] * (chassis.v_tar - chassis.v_filter);
		out3 = LQR_K[10] * (chassis.PithR - 0.0f) + LQR_K[11] * (chassis.PithGyroR - 0.0f);
    chassis.wheel_T[0] = chassis.wheel_T[0] - Turn.pid.cout;

    Limit(&chassis.wheel_T[0], -5, 5);

    L0_R.GetPidPos(L0_L_pid, chassis.leg_tar, VMC_leg_R.VMC_data.L0, 1000);
    VMC_leg_R.VMC_data.F0 = FF / arm_cos_f32(VMC_leg_R.VMC_data.theta) + L0_R.pid.cout + chassis.roll_f0;

		
    if (is_Tp == true)
        VMC_leg_R.VMC_data.Tp = VMC_leg_R.VMC_data.Tp + theta_err.pid.cout;
		else
				VMC_leg_R.VMC_data.Tp = theta_err.pid.cout;

		

		Limit(&VMC_leg_R.VMC_data.Tp, -max_TP, max_TP);

		
    VMC_leg_R.Jacobian();
}

/*********************左腿任务*********************/
float cur;
float angle;
bool is_on;

uint8_t send_R[8];
uint8_t send_L[8];

void setCAN(float torque, int id, uint8_t msd[])
{
    auto send_data = static_cast<int16_t>(torque);

    msd[(id - 1) * 2] = send_data >> 8;
    msd[(id - 1) * 2 + 1] = send_data << 8 >> 8;
}

bool is_wheel;
void Chassis_Task_L()
{
    while (INS.ins_flag == 0)
    { // 等待加速度收敛
        osDelay(1);
    }
    ChassisL_Init();

    while (1)
    {
        ChassisL_feedback_update();
        chassisL_control_loop();
			
        auto P1010L_torque_constant = BSP::Motor::BM::MotorP1010L.params_.torque_constant;
        auto M1505L_torque_constant = BSP::Motor::BM::MotorM1505L.params_.current_constant;
			
        // 遥控器

            // 			//打开电机
            // 			L_joint_0.ctrl_motor(&hfdcan2, 0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[0]);
            // 			osDelay(Up_Chassis_Time);

            // 			L_joint_1.ctrl_motor(&hfdcan2, 0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[1]);
            // 			osDelay(Up_Chassis_Time);

            // 			L_Wheel.ctrl_motor(&hfdcan2, 0, 0, 0, 0, chassis.wheel_T[1]);
            // //			L_Wheel.ctrl_motor(&hfdcan2, 0, 0, 0, 0, 0);
            // 			osDelay(Up_Chassis_Time);

            // BSP::Motor::BM::MotorP1010B.setCAN(VMC_leg_L.VMC_data.torque_set[0], 2);
            angle = BSP::Motor::BM::MotorP1010L.getAddAngleDeg(1);

            setCAN((VMC_leg_L.VMC_data.torque_set[1] * P1010L_torque_constant) * 100, 1, send_L);

            setCAN((VMC_leg_L.VMC_data.torque_set[0] * P1010L_torque_constant) * 100, 2, send_L);


						td_out_R.td_quadratic(chassis.wheel_T[1]);

						if(is_on == true)
							setCAN(td_out_R.x1 / M1505L_torque_constant, 4, send_L);
						else
							setCAN(0, 4, send_L);

            DM_Send_Task();
        
        if (Emergency_Stop == true)
        {
            // 打开电机
            // 打开电机
            setCAN(0, 1, send_L);
            setCAN(0, 2, send_L);
            setCAN(0, 4, send_L);

        }
				BSP::Motor::BM::MotorP1010L.sendCAN(&hfdcan1, send_L);
        osDelay(1);
    }
}

/*********************右腿任务*********************/



void Chassis_Task_R()
{
    while (INS.ins_flag == 0)
    { // 等待加速度收敛
        osDelay(1);
    }
    ChassisR_Init();

    while (1)
    {
        ChassisR_feedback_update();
        chassisR_control_loop();
			
        auto P1010R_torque_constant = BSP::Motor::BM::MotorP1010R.params_.torque_constant;
        auto M1505R_torque_constant = BSP::Motor::BM::MotorM1505R.params_.current_constant;
			
        // 遥控器

            setCAN((VMC_leg_R.VMC_data.torque_set[1] * P1010R_torque_constant) * 100, 1, send_R);

            setCAN((VMC_leg_R.VMC_data.torque_set[0] * P1010R_torque_constant) * 100, 2, send_R);
					
						td_out_L.td_quadratic(chassis.wheel_T[0]);
			
						if(is_on == true)
							setCAN(td_out_L.x1 / M1505R_torque_constant, 4, send_R);
						if(is_on == false)
							setCAN(0 / M1505R_torque_constant, 4, send_R);

        
        if (Emergency_Stop == true)
        {
            // 打开电机
            setCAN(0, 1, send_R);
            setCAN(0, 2, send_R);
            setCAN(0, 4, send_R);

						chassis.x_tar = chassis.x_filter;
						chassis.turn_tar = chassis.Yaw_L;
        }
				
				BSP::Motor::BM::MotorP1010R.sendCAN(&hfdcan1, send_R);
            osDelay(1);

    }
}

void DM_Send_Task()
{
    //    dir = RM_Clicker::ISDir();

    *((float *)&send_str2[0]) = INS.Pitch;
    *((float *)&send_str2[4]) = VMC_leg_L.VMC_data.L0;

    //		*((float*)&send_str2[8]) = VMC_leg_L.VMC_data.theta;
    //		*((float*)&send_str2[12]) = VMC_leg_L.VMC_data.d_theta;
    //		*((float*)&send_str2[16]) = chassis.PithGyroR;
    //		*((float*)&send_str2[20]) = chassis.PithGyroL;
    //		*((float*)&send_str2[24]) = chassis_R.PithGyroR;
    //		*((float*)&send_str2[28]) = 0;

    *((uint32_t *)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
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
uint32_t OBSERVE_TIME = 3; // 任务周期是3ms

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2); // 状态向量2维 没有控制量 测量向量2维

    memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF, float acc, float vel)
{
    // 卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] = vel; // 测量速度
    EstimateKF->MeasuredVector[1] = acc; // 测量加速度

    // 卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
        vel_acc[i] = EstimateKF->FilteredValue[i];
    }
}

void Kalman_task(void)
{
    while (INS.ins_flag == 0)
    { // 等待加速度收敛
        osDelay(1);
    }
    static float wr, wl = 0.0f;
    static float vrb, vlb = 0.0f;
    static float aver_v = 0.0f;

    xvEstimateKF_Init(&vaEstimateKF);

    while (1)
    {
        wr = -BSP::Motor::BM::MotorM1505R.getVelocityRads(1) - INS.Gyro[0] +
             VMC_leg_R.VMC_data.d_alpha; // 右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vrb = wr * 0.09f +
              VMC_leg_R.VMC_data.L0 * VMC_leg_R.VMC_data.d_theta * arm_cos_f32(VMC_leg_R.VMC_data.theta) +
              VMC_leg_R.VMC_data.d_L0 * arm_sin_f32(VMC_leg_R.VMC_data.theta); // 机体b系的速度

        wl = -BSP::Motor::BM::MotorM1505L.getVelocityRads(1) + INS.Gyro[0] +
             VMC_leg_L.VMC_data.d_alpha; // 左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
        vlb = wl * 0.09f +
              VMC_leg_L.VMC_data.L0 * VMC_leg_L.VMC_data.d_theta * arm_cos_f32(VMC_leg_L.VMC_data.theta) +
              VMC_leg_L.VMC_data.d_L0 * arm_sin_f32(VMC_leg_L.VMC_data.theta); // 机体b系的速度

        aver_v = (vrb - vlb) / 2.0f; // 取平均
        xvEstimateKF_Update(&vaEstimateKF, INS.MotionAccel_b[1], aver_v);

        // 原地自转的过程中v_filter和x_filter应该都是为0
        chassis.v_filter = vel_acc[0]; // 得到卡尔曼滤波后的速度
        chassis.x_filter = chassis.x_filter + chassis.v_filter * ((float)1 / 1000.0f);

        // 如果想直接用轮子速度，不做融合的话可以这样
//        chassis.v_filter=(chassis.wheel_motor[0].para.vel-chassis.wheel_motor[1].para.vel)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
//        chassis.x_filter=chassis.x_filter+chassis_move.x_filter+chassis.v_filter*((float)OBSERVE_TIME/1000.0f);

//        chassis.v_filter=(BSP::Motor::BM::MotorM1505R.getVelocityRads(1)-BSP::Motor::BM::MotorM1505L.getVelocityRads(1))*(-0.09f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
//        chassis.x_filter+=chassis.v_filter*((float)1/1000.0f);

        osDelay(1);
    }
}
