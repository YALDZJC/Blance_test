#pragma once

#include "RM_StaticTime.h"//静态定时器
#include <stdint.h>
#include "fdcan.h"
#include "RM_stm32fxxx_hal.h"
#include "RM_Can.h"

//根据上位机调节
#define L_LEG_CAN_ID 8
#define L_LEG_Master_ID 4

#define R_LEG_CAN_ID 6
#define R_LEG_Master_ID 3
//速度模式
#define DM_V_ID DM_YAW_CAN_ID + 0x200

//位置速度模式
#define DM_PV_ID DM_CAN_ID + 0x100
#define P_MIN (-12.56)
#define P_MAX (12.56)    

#define V_MIN (-30)
#define V_MAX (30)

#define KP_MIN (0.0)
#define KP_MAX (500.0)

#define KD_MIN (0.0)
#define KD_MAX (5.0)

#define T_MIN (-10)
#define T_MAX (10)

#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f

enum DM_Motor_Type
{
	DM_4310 = 0,
	DM_6215 ,
};

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
/// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min; 
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
/// Converts a float to an unsigned int, given range and number of bits///
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x-offset)*((float)((1<<bits)-1))/span);
}


typedef struct 
{
	uint8_t DM_Type;
	uint8_t Send_ID;
	uint8_t Receive_ID;
	
	uint8_t ID;
	uint8_t ERR;//故障
	float position;//位置
	float velocity;//速度
	float torque;//转矩
	uint8_t T_MOS;//MOS的平均温度
	uint8_t T_Rotor;//示电机内部线圈的平均温度
	int p_int;
	int v_int;
	int t_int;
}Data;

class DM
{
public:

	Data DM_Data;

	DM(uint8_t Send_ID, uint8_t Receive_ID, uint8_t DM_Type)
	{
		DM_Data.Send_ID = Send_ID;
		DM_Data.Receive_ID = Receive_ID;
		DM_Data.DM_Type = DM_Type;
	}

  uint8_t send_data[8];
	RM_StaticTime dir_time;
	bool dir;
  DM(/* args */);
  //初始化
  void init();
  //解析
	void parse(FDCAN_RxHeaderTypeDef RxHeader, const uint8_t* RxData);
  //设置电机数据，力矩控制a
	void ctrl_motor(FDCAN_HandleTypeDef* hcan, float _pos, float _vel,float _KP, float _KD, float _torq); 
	//位置速度
  void ctrl_motor2(FDCAN_HandleTypeDef* hcan, float _pos, float _vel);
  //速度
  void ctrl_motor3(FDCAN_HandleTypeDef* hcan, float _vel);
  //位置限幅
  float Pitch_angle_Limit(float IN,float max,float min);
  //开电机
  void on(FDCAN_HandleTypeDef* hcan);
  //关电机
  void off(FDCAN_HandleTypeDef* hcan);
  //清除电机错误
  void clear_err(FDCAN_HandleTypeDef* hcan);
	//断链
	bool is_dir(int time);
};

DM::DM(/* args */)
{
}

//void DM::init()
//{
//  this->on();  
//}

void DM::parse(FDCAN_RxHeaderTypeDef RxHeader, const uint8_t* RxData)
{
	if (RxHeader.Identifier == this->DM_Data.Receive_ID && this->DM_Data.DM_Type == DM_4310)
	{
		this->DM_Data.ID = RxData[0] & 0xF0;
		this->DM_Data.ERR = RxData[0] & 0xF;
		this->DM_Data.p_int = (RxData[1]<<8)|RxData[2];
		this->DM_Data.v_int = (RxData[3]<<4)|(RxData[4]>>4);
		this->DM_Data.t_int = ((RxData[4]&0xF)<<8)|RxData[5];
		this->DM_Data.T_MOS = RxData[6];
		this->DM_Data.T_Rotor = RxData[7];

		this->DM_Data.position = uint_to_float(this->DM_Data.p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
		this->DM_Data.velocity = uint_to_float(this->DM_Data.v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
		this->DM_Data.torque 	 = uint_to_float(this->DM_Data.t_int, T_MIN, T_MAX, 12); // (-18.0,18.0)
		
		this->dir_time.UpLastTime();
	}
	if (RxHeader.Identifier == this->DM_Data.Receive_ID && this->DM_Data.DM_Type == DM_6215)
	{
		this->DM_Data.ID = RxData[0] & 0xF0;
		this->DM_Data.ERR = RxData[0] & 0xF;
		this->DM_Data.p_int = (RxData[1]<<8)|RxData[2];
		this->DM_Data.v_int = (RxData[3]<<4)|(RxData[4]>>4);
		this->DM_Data.t_int = ((RxData[4]&0xF)<<8)|RxData[5];
		this->DM_Data.T_MOS = RxData[6];
		this->DM_Data.T_Rotor = RxData[7];

		this->DM_Data.position = uint_to_float(this->DM_Data.p_int, P_MIN2, P_MAX2, 16); // (-12.5,12.5)
		this->DM_Data.velocity = uint_to_float(this->DM_Data.v_int, V_MIN2, V_MAX2, 12); // (-45.0,45.0)
		this->DM_Data.torque 	 = uint_to_float(this->DM_Data.t_int, T_MIN2, T_MAX2, 12); // (-18.0,18.0)
		
		this->dir_time.UpLastTime();
	}
}

void DM::ctrl_motor(FDCAN_HandleTypeDef* hcan, float _pos, float _vel,float _KP, float _KD, float _torq)
{
  uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
  pos_tmp = float_to_uint(_pos, P_MIN, P_MAX, 16);
  vel_tmp = float_to_uint(_vel, V_MIN, V_MAX, 12);
  kp_tmp  = float_to_uint(_KP,  KP_MIN,KP_MAX,12);
  kd_tmp  = float_to_uint(_KD,  KD_MIN,KD_MAX,12);
  tor_tmp = float_to_uint(_torq,T_MIN, T_MAX, 12);

  this->send_data[0] = (pos_tmp >> 8);
  this->send_data[1] = (pos_tmp);
  this->send_data[2] = (vel_tmp >> 4);
  this->send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
  this->send_data[4] = kp_tmp;
  this->send_data[5] = (kd_tmp >> 4);
  this->send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
  this->send_data[7] = tor_tmp;  
	
	RM_FDorCAN_Send(hcan, this->DM_Data.Send_ID, this->send_data);//发送
}

void DM::ctrl_motor2(FDCAN_HandleTypeDef* hcan, float _pos, float _vel)
{
  uint8_t *pbuf,*vbuf;
  pbuf = (uint8_t*)&_pos;
  vbuf = (uint8_t*)&_vel;
  this->send_data[0] = *pbuf;
  this->send_data[1] = *(pbuf+1);
  this->send_data[2] = *(pbuf+2);
  this->send_data[3] = *(pbuf+3);
  this->send_data[4] = *vbuf;
  this->send_data[5] = *(vbuf+1);
  this->send_data[6] = *(vbuf+2);
  this->send_data[7] = *(vbuf+3);
	
	RM_FDorCAN_Send(hcan, this->DM_Data.Send_ID, this->send_data);//发送
}

void DM::ctrl_motor3(FDCAN_HandleTypeDef* hcan, float _vel)
{
  uint8_t *vbuf;
  vbuf = (uint8_t*)&_vel;
  this->send_data[0] = *vbuf;
  this->send_data[1] = *(vbuf+1);
  this->send_data[2] = *(vbuf+2);
  this->send_data[3] = *(vbuf+3);
	
	RM_FDorCAN_Send(hcan, this->DM_Data.Send_ID, this->send_data);//发送
}

float DM::Pitch_angle_Limit(float IN,float max,float min)
{
	float OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	
	return OUT;
}

inline void DM::on(FDCAN_HandleTypeDef* hcan)
{
  *(uint64_t*)(&this->send_data[0]) = 0xFCFFFFFFFFFFFFFF;
	RM_FDorCAN_Send(hcan, this->DM_Data.Send_ID, this->send_data);//发送

}

inline void DM::off(FDCAN_HandleTypeDef* hcan)
{
  *(uint64_t*)(&this->send_data[0]) = 0xFDFFFFFFFFFFFFFF;
	RM_FDorCAN_Send(hcan, this->DM_Data.Send_ID, this->send_data);//发送

}

inline void DM::clear_err(FDCAN_HandleTypeDef* hcan)
{
  *(uint64_t*)(&this->send_data[0]) = 0xFBFFFFFFFFFFFFFF;
	RM_FDorCAN_Send(hcan, this->DM_Data.Send_ID, this->send_data);//发送

}

inline bool DM::is_dir(int time)
{
	this->dir = this->dir_time.ISDir(time);
	return this->dir;
}