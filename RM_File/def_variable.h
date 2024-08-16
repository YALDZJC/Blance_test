#pragma once

#include "RM_StaticTime.h"//静态定时器
#include "RM_Clicker.h"//遥控器
#include "RM_PID.h"
#include "ladrc.h"
#include "RM_Motor.h"//RM电机
#include "RM_Can.h"
#include "RM_stm32fxxx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include "RM_Wheel.h"
#include "RM_PM01.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "RM_RefereeSystem.h"
#include "INS_task.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
#include "spi.h"
#include "DM.h"
#include "RM_Servos.h"
#include "RM_CHxxxGy.h"
#include "VMC.h"
#include "kalman_filter.h"

/*
														操作控制策划														

x代表无效
y代表有效
数字代表值

左手                                     右手
ch3:前后			ch2:左右			s1开关132			ch1:前后			ch0:左右			s2开关132
y             x            1             x             y            1             控底盘前后左右,不控云台,不改表俯仰角（1）
x             x            1             y             y            3             控云台yaw/pitch,不控底盘（2）
x							x						 1						 x						 x					  2							待定（3）

y             x            3             y             y            1             底盘跟随云台,俯仰角5度（4）
y             x            3             y             y            3             底盘跟随云台,俯仰角15度（5）
x             x            3             x             x            2             待定（6）

x							x						 2						 x						 x						1							待定（7）
x							x						 2						 x						 x						3							待定（8）
x							x						 2						 x						 x						2							急停（9）
*/

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define MODE1 (RM_Clicker::RC_Ctl.rc.s1 == 1)//遥控器模式
#define MODE3 (RM_Clicker::RC_Ctl.rc.s1 == 3)//遥控器模式
#define MODE2 (RM_Clicker::RC_Ctl.rc.s1 == 2)//遥控器模式

#define CONTROL_SIG 0
#if CONTROL_SIG == 0
	#define Clicker_chassis_follow_gimbal (MODE1 && RM_Clicker::RC_Ctl.rc.s2 == 1)//（1）底盘跟随云台
	#define Clicker_chassis_follow_gimbal_for (MODE1 && RM_Clicker::RC_Ctl.rc.s2 == 3)//（2）底盘跟随云台90
	#define Clicker_chassis_follow_gimbal_back (MODE1 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（3）小陀螺
	#define On_Friction_Dial_Warehouse_doors (MODE3 && RM_Clicker::RC_Ctl.rc.s2 == 1)//打开摩擦轮+仓门+拨盘
	#define Keyboard_Mouse (MODE3 && RM_Clicker::RC_Ctl.rc.s2 == 3)//（2）
	#define Combinatoria_Movement (MODE3 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（3）组合运动模式
	#define Emergency_Stop (MODE2 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（9）急停模式
#elif CONTROL_SIG == 1
	#define chassis_RC_LX (Gimbal_to_Chassis_Data.int16_RC_LX)//左手x
	#define chassis_RC_LY (Gimbal_to_Chassis_Data.int16_RC_LY)//左手y
	#define stop_mode (RM_Clicker::RC_Ctl.rc.s1 == 2 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（9）
#endif

#define YAW_E (Gimbal_to_Chassis_Data.yaw_encoder_e)//底盘跟随云台误差
#define CHASSIS_FOLLOW_GIMBAL (Gimbal_to_Chassis_Data.chassis_follow_gimbal)//底盘跟随云台模式
#define GIMBAL_HOST_CHASSIS (Gimbal_to_Chassis_Data.gimbal_host_chassis)//底盘跟随云台模式
#define CHASSIS_GYRO (Gimbal_to_Chassis_Data.chassis_gyro)//底盘跟随云台模式
#define CHASSIS_TOP (Gimbal_to_Chassis_Data.chassis_top)//底盘跟随云台模式
#define MAX_CHASSIS_SPEED 16384.0//最大速度
#define CHASSIS_SPEED_ZOOM_VW 1.0//放大因子
#define CHASSIS_SPEED_ZOOM_VXY 0.2//放大因子
#define GY_V_SET 600//小陀螺+-速度
float W_V_Init  = 1000;//小陀螺初始转速
#define YD_V_SET 0.12//平移+-速度
#define YD_V_Init 1 //平移初始转速

float L1 = 0.075, L2 = 0.14, L3 = 0.14, L4 = 0.075, L5 = 0.08;
/***************************变量声明*********************************/
//当前车体运行状态选择
enum now_chassis_mode_enum
{
	chassis_follow_gimbal = 0,
	chassis_follow_gimbal_90 = 1,
	chassis_gyro = 2
};
uint8_t now_chassis_mode = 0;

//串口打印
#define Send_Usart_Data_Huart huart10

//云台到底盘
#define Send_Gimbal_to_Chassis_Huart huart7
#define Send_Gimbal_to_Chassis_Huart_LEN 12

//底盘跟随云台旋转量
float cos_xita = 0,sin_xita = 0;

//旋转量
float vw = 0,kvw = -0.6;//kw旋转系数
float vx = 0,vy = 0;float vxy_kvw = 0;//旋转的时候降低转速保功率

//误差夹角
float yaw_e_xita = 0,yaw_e_radian = 0;

/***************************底盘*********************************/
uint8_t send_CHASSIS_motor_size;//用于3508与6020错开发送

RM_StaticTime Total_tasks_staticTime;//控制时间

/***************************云台*********************************/
#define Dead_Zone 15

#define MOUSE_X_K -0.0005f//鼠标x移动系数
#define MOUSE_Y_K -0.0008f//鼠标y移动系数

//#define YAW_ID
#define SHOOT_L_ID 0x203 
#define SHOOT_R_ID 0x204
#define GET_SHOOT_ID 0x000

#define Get_shoot_MAX_Speed -4000
#define Shoot_MAX_Speed 6000

#define MAX_ANGLE -2
#define MIN_ANGLE -70

uint8_t send_GIMBAL_motor_size;//用于3508与6020错开发送
uint32_t send_DM_motor_ms;

//达妙电机
DM L_joint_0(6, 3, 0);
DM L_joint_1(8, 4, 0);
DM L_Wheel(1, 0, 1);

DM R_joint_2(6, 3, 0);
DM R_joint_3(8, 4, 0);
DM R_Wheel(1, 0, 1);

uint8_t send_motor_ms;
bool dm_is_open = false;//达妙启动标志位
uint8_t Up_Chassis_Time = 1;

uint8_t is = 0;
//云台PID相关
typedef struct
{
	float wheel_T[2];
	
	float v_tar;//期望速度，单位是m/s
	float x_tar;//期望位置，单位是m
	float turn_tar;//期望yaw轴弧度
	float roll_tar;	//期望roll轴弧度
	float leg_tar;//期望腿长，单位是m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float PithR;
	float PithGyroR;
	float PithL;
	float PithGyroL;
	
	float Yaw_L;
	float Yaw_R;
	
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志

	uint8_t prejump_flag;//预跳跃标志
	uint8_t recover_flag;//一种情况下的倒地自起标志
	
} chassis_t;

chassis_t chassis;
//PID参数初始化
RM_PID Turn;
Kpid_t Turn_pid(1.5, 0, 0.1);

RM_PID L0_L;
Kpid_t L0_L_pid(320, 0, 10);

RM_PID L0_R;
Kpid_t L0_R_pid(0, 0, 0);

RM_PID theta_err;
Kpid_t K_theta_err(5, 0, 0.1);

RM_PID roll;
Kpid_t K_roll(0, 0, 0);
//滤波相关
TD_quadratic V_speed(150);

//舵机相关
RM_Servos servos;//仓门舵机 
float servos_angle;//舵机角度
float servos_angle_on = 1990;//初始舵机角度，开
float servos_angle_off = 850;//舵机固定角度，关

bool dir;
bool switch_DM = 1;

int16_t dianlu;
TD_quadratic dianlutd(100);
int16_t dianlupid;

VMC_leg_t VMC_leg_L( L1, L2, L3, L4, L5 );
VMC_leg_t VMC_leg_R( L1, L2, L3, L4, L5 );

float Turn_out, Kp = 3, Kd = 0.3;
float Roll_out, roll_Kp, roll_Kd;

float tar_L0, FF = 13, rc_dc = 0.0002, go_dc = 0.001,turn_dc = -0.00005;
//float Kp = -200,Kd = -40,P_out,D_out;
float tar_dc = 0.0001;
float Poly_Coefficient[12][4]={	{-88.3079710751263,	68.9068310796955,	-30.0003802287502,	-0.197774178106864},
																{1.52414598059982	,-1.09343038036609,	-2.82688593867512,	0.0281973842051861},
																{-21.8700750609220	,12.7421672466682,	-2.58779676995074	,-0.750848242540331},
																{-29.3271263750692,	17.6067629457167,	-4.23484645974363	,-1.08976980288501},
																{-147.771748892911,	94.0665615939814,	-22.5139626085997	,2.53224765312440},
																{-6.72857056332562,	4.46216499907277,	-1.14328671767927	,0.176775242328476},
																{-43.1495035855057,	35.1427890165576,	-12.7617044245710	,3.36940801739176},
																{4.14428184617563,	-2.56933858132474,	0.479050092243477	,0.248175261724735},
																{-229.898177881547	,144.949258291255	,-33.9196587052128,	3.44291788865558},
																{-329.509693153293,	207.219295206736,	-48.3799707459102	,4.952560575479143},
																{380.589246401548,	-223.660017597103	,46.1696952431268	,9.82308882692083},
																{26.1010681824798	,-15.7241310513153	,3.39175554658673	,0.278568898146322}};

float LQR_K[12]={ 
   -2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
    2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体
		
float vaEstimateKF_F[4] = {1.0f, 0.003f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.001s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.1f, 0.0f, 
                           0.0f, 0.1f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {100.0f, 0.0f, 
                            0.0f,  100.0f}; 	
														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
		