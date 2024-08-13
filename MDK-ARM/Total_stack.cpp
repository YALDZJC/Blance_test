#pragma once

#include "def_variable.h"
#include "Total_stack.h"
#include "cmsis_os.h"


struct SEND_GRAPHIC_QUEUE;//�������ݶ���
extern SEND_GRAPHIC_QUEUE send_graphic_queue;
uint8_t send_str2[64];

/***************************��������*********************************/

//���ܳ�ʼ��
void Total_tasks_Init();//****

//���ܺ���
void Total_tasks_Run();

//��ͣģʽ��������pid
void control_pid_0_speed();

//�����������
void Clear_ALL_Data();

//��ȡ��̨���������ݳ�ʼ��
void Get_Gimbal_to_Chassis_Init();

//��ȡ��̨����������
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart);

//���͵���3508����
void Send_CHASSIS_3508_CAN()
{
	//����
	RM_FDorCAN_Send(&hfdcan1,SEND_MOTOR_ID_3508,msd_CHASSIS_3508_2006.Data);
}
//���͵���6020����
void Send_CHASSIS_6020_CAN()
{
	//����
	RM_FDorCAN_Send(&hfdcan1,SEND_MOTOR_ID_6020,msd_CHASSIS_6020.Data);
}

//������̨3508����
void Send_GIMBAL_3508_CAN()
{
	//����
	RM_FDorCAN_Send(&hfdcan2,SEND_MOTOR_ID_3508,msd_GIMBAL_3508_2006.Data);
}
//������̨6020����
void Send_GIMBAL_6020_CAN()
{
	//����
	RM_FDorCAN_Send(&hfdcan2,SEND_MOTOR_ID_6020,msd_GIMBAL_6020.Data);
}

//���ʹ������
void Send_L_LEG_DM_CAN()
{


}

//���ʹ������
void Send_R_LEG_DM_CAN()
{
	RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_1.send_data);//����
}

//��ͣģʽ��������pid
void control_pid_0_speed()
{

}

//�����������
void Clear_ALL_Data()
{ 
	
}



//���ܳ�ʼ��
void Total_tasks_Init()
{
	RM_FDorCAN_Init();//can���ó�ʼ��
	
	rmClicker.Init();//ң�����������ó�ʼ��
				
	pm01.PM01Init();
	
	L_joint_0.off();//���������ʼ��	
	L_joint_1.off();
	
	HAL_Delay(10);
	
	//��¼��һ��ʱ��
	uint64_t time_adrc = HAL_GetTick();
	
	//�����ǳ�ʼ��
//	INS_Init();
//	DWT_Init(480);
//	while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
//	{
//	  ;
//	}
//	Power_OUT1_ON;//imu��ʼ����ɣ��ɿص�Դ�򿪣�led����
//	Power_OUT2_ON;
	
	//adrc������
	while(1)
	{

		if(HAL_GetTick() - time_adrc > 500)
		{
			break;
		}
	}
}

//can_filo0�жϽ���
FDCAN_RxHeaderTypeDef CHASSIS_RxHeader;	//can��������
uint8_t CHASSIS_RxHeaderData[8] = { 0 };
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	//������Ϣ 
	HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0, &CHASSIS_RxHeader, CHASSIS_RxHeaderData);

	if(hfdcan == &hfdcan1)
	{
		R_joint_2.parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
		R_joint_3.parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
		R_Wheel.parse(CHASSIS_RxHeader, CHASSIS_RxHeaderData);
	}
}

//can_filo1�жϽ���
FDCAN_RxHeaderTypeDef GIMBAL_RxHeader;	//can��������
uint8_t GIMBAL_RxHeaderData[8] = { 0 };
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	//������Ϣ 
	HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO1, &GIMBAL_RxHeader, GIMBAL_RxHeaderData);

	if(hfdcan == &hfdcan2)
	{
		L_joint_0.parse(GIMBAL_RxHeader, GIMBAL_RxHeaderData);
		L_joint_1.parse(GIMBAL_RxHeader, GIMBAL_RxHeaderData);
		L_Wheel.parse(GIMBAL_RxHeader, GIMBAL_RxHeaderData);
	}
}

//UART�����жϽ���
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//ң��������
}

//UART�ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Get_Gimbal_to_Chassis(huart);
	RM_RefereeSystem::RM_RefereeSystemParse(huart);
}

//��ȡ��̨����������
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart)
{
	if(huart == &Send_Gimbal_to_Chassis_Huart)
	{
		
	}
}

void ChassisL_Init()
{
	for(int j=0;j<10;j++)
	{
		L_joint_0.on();
		RM_FDorCAN_Send(&hfdcan2, L_joint_0.DM_Data.Send_ID, L_joint_0.send_data);//����

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		L_joint_1.on();
		RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_1.send_data);//����

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		L_Wheel.on();
		RM_FDorCAN_Send(&hfdcan2, L_Wheel.DM_Data.Send_ID, L_Wheel.send_data);//����

	  osDelay(1);
	}
}

void ChassisR_Init()
{
	for(int j=0;j<10;j++)
	{
		R_joint_2.on();
		RM_FDorCAN_Send(&hfdcan1, R_joint_2.DM_Data.Send_ID, R_joint_2.send_data);//����

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		R_joint_3.on();
		RM_FDorCAN_Send(&hfdcan1, R_joint_3.DM_Data.Send_ID, R_joint_3.send_data);//����

	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
		R_Wheel.on();
		RM_FDorCAN_Send(&hfdcan1, L_Wheel.DM_Data.Send_ID, L_Wheel.send_data);//����

	  osDelay(1);
	}
}

void ChassisL_feedback_update()
{
	VMC_leg_L.VMC_data.phi1=pi/2.0f + L_joint_0.DM_Data.position;
	VMC_leg_L.VMC_data.phi4=pi/2.0f + L_joint_1.DM_Data.position;
}

void ChassisR_feedback_update()
{
	VMC_leg_R.VMC_data.phi1=pi/2.0f + R_joint_2.DM_Data.position;
	VMC_leg_R.VMC_data.phi4=pi/2.0f + R_joint_3.DM_Data.position;
}

void Chassis_Task_L()
{
  while(INS.ins_flag==0)
	{//�ȴ����ٶ�����
	  osDelay(1);	
	}	
	ChassisL_Init();
	
	while(1)
	{	
		tar_L0 += RC_LY*tar_dc;
		if(tar_L0 > 18)
			tar_L0 = 18;
		if(tar_L0 < 5)
			tar_L0 = 5;
			
		ChassisL_feedback_update();

		VMC_leg_L.Up_Left(INS.Pitch, INS.Gyro[0], 0.001);
		
		for(int i=0;i<12;i++)
		{
			LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0], VMC_leg_L.VMC_data.L0);	
		}
		
	//�ұ��Źؽ��������				
		VMC_leg_L.VMC_data.Tp=(LQR_K[6]*(VMC_leg_L.VMC_data.theta-0.0f)
													+LQR_K[7]*(VMC_leg_L.VMC_data.d_theta-0.0f)
													+LQR_K[10]*(INS.Pitch-0.0f)
													+LQR_K[11]*(INS.Gyro[0]-0.0f));
		
		P_out = Kp*(tar_L0/100 - VMC_leg_L.VMC_data.L0);
		D_out = Kd*(0 - VMC_leg_L.VMC_data.d_L0);
		
//		LEG_F0_PID.GetPidPos(LEG_F0_Init, tar_L0/100, VMC_leg_L.VMC_data.L0, 3);
		
		VMC_leg_L.VMC_data.F0 = FF + P_out + D_out;
		
		VMC_leg_L.Jacobian();
		
		//ң����
		if(is == 0)
		{
			//�򿪵��			
			L_joint_0.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan2, L_joint_0.DM_Data.Send_ID, L_joint_0.send_data);//����
			osDelay(Up_Chassis_Time);

			L_joint_1.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_0.send_data);//����
			osDelay(Up_Chassis_Time);

			L_Wheel.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan2, L_Wheel.DM_Data.Send_ID, L_joint_0.send_data);//����
			osDelay(Up_Chassis_Time);
		}
		else
		{
			//�򿪵��			
			L_joint_0.ctrl_motor(0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[0]);
			RM_FDorCAN_Send(&hfdcan2, L_joint_0.DM_Data.Send_ID, L_joint_0.send_data);//����
			osDelay(Up_Chassis_Time);

			L_joint_1.ctrl_motor(0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[1]);
			RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_0.send_data);//����
			osDelay(Up_Chassis_Time);

			L_Wheel.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan2, L_Wheel.DM_Data.Send_ID, L_Wheel.send_data);//����
			osDelay(Up_Chassis_Time);
		}
	}
}

void Chassis_Task_R()
{
  while(INS.ins_flag==0)
	{//�ȴ����ٶ�����
	  osDelay(1);	
	}	
	ChassisR_Init();
	
	while(1)
	{	
		tar_L0 += RC_LY*tar_dc;
		if(tar_L0 > 18)
			tar_L0 = 18;
		if(tar_L0 < 5)
			tar_L0 = 5;
			
		ChassisR_feedback_update();

		VMC_leg_R.Up_Right(INS.Pitch, INS.Gyro[0], 0.001);
		
		for(int i=0;i<12;i++)
		{
			LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0], VMC_leg_L.VMC_data.L0);	
		}
		
	//�ұ��Źؽ��������				
		VMC_leg_L.VMC_data.Tp=(LQR_K[6]*(VMC_leg_L.VMC_data.theta-0.0f)
													+LQR_K[7]*(VMC_leg_L.VMC_data.d_theta-0.0f)
													+LQR_K[10]*(INS.Pitch-0.0f)
													+LQR_K[11]*(INS.Gyro[0]-0.0f));
		
		P_out = Kp*(tar_L0/100 - VMC_leg_L.VMC_data.L0);
		D_out = Kd*(0 - VMC_leg_L.VMC_data.d_L0);
		
//		LEG_F0_PID.GetPidPos(LEG_F0_Init, tar_L0/100, VMC_leg_L.VMC_data.L0, 3);
		
		VMC_leg_L.VMC_data.F0 = FF + P_out + D_out;
		
		VMC_leg_L.Jacobian();
		
		//ң����
		if(is == 0)
		{
			//�򿪵��			
			R_joint_2.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan1, R_joint_2.DM_Data.Send_ID, R_joint_2.send_data);//����
			osDelay(Up_Chassis_Time);

			R_joint_3.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan1, R_joint_3.DM_Data.Send_ID, R_joint_3.send_data);//����
			osDelay(Up_Chassis_Time);

			R_Wheel.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan1, R_Wheel.DM_Data.Send_ID, R_Wheel.send_data);//����
			osDelay(Up_Chassis_Time);
		}
		else
		{
			//�򿪵��			
			R_joint_2.ctrl_motor(0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[0]);
			RM_FDorCAN_Send(&hfdcan1, R_joint_2.DM_Data.Send_ID, R_joint_2.send_data);//����
			osDelay(Up_Chassis_Time);

			R_joint_3.ctrl_motor(0, 0, 0, 0, VMC_leg_L.VMC_data.torque_set[1]);
			RM_FDorCAN_Send(&hfdcan1, R_joint_3.DM_Data.Send_ID, R_joint_3.send_data);//����
			osDelay(Up_Chassis_Time);

			R_Wheel.ctrl_motor(0, 0, 0, 0, 0);
			RM_FDorCAN_Send(&hfdcan1, R_Wheel.DM_Data.Send_ID, R_Wheel.send_data);//����
			osDelay(Up_Chassis_Time);
		}
	}
}

void DM_Send_Task()
{
	//��������
//	if(INS.ins_flag == 1)
//	{
//		if(send_motor_ms == 0)
//		{
//			RM_FDorCAN_Send(&hfdcan2, L_joint_0.DM_Data.Send_ID, L_joint_0.send_data);//����
//		}
//		if(send_motor_ms == 1)
//		{
//			RM_FDorCAN_Send(&hfdcan2, L_joint_1.DM_Data.Send_ID, L_joint_1.send_data);//����
//		}
//		if(send_motor_ms == 2)
//		{
//			RM_FDorCAN_Send(&hfdcan2, L_Wheel.DM_Data.Send_ID, L_Wheel.send_data);//����	}
//		}
//		send_motor_ms++;
//		send_motor_ms %= 3;
//	}
}

void Send_Vofa_Task()
{
//		*((float*)&send_str2[0]) = VMC_leg_L.VMC_data.L0;
//		*((float*)&send_str2[4]) = VMC_leg_L.VMC_data.d_L0;

//		*((float*)&send_str2[8]) = L_joint_0.DM_Data.position;
//		*((float*)&send_str2[12]) = L_joint_1.DM_Data.position;
//		*((float*)&send_str2[16]) = VMC_leg_L.VMC_data.phi4;
//		*((float*)&send_str2[20]) = VMC_leg_L.VMC_data.phi1;
		*((uint32_t*)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
		HAL_UART_Transmit_DMA(&Send_Usart_Data_Huart, send_str2, sizeof(float) * (7 + 1));
}

