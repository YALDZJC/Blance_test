#pragma once

#include "stdint.h"
#include "math.h"
#include "arm_math.h"

#define pi 3.14159265352979

typedef struct
{
	/*�������ȵĹ����������̶�����*/
	float l5;//AE���� //��λΪm
	float	l1;//��λΪm
	float l2;//��λΪm
	float l3;//��λΪm
	float l4;//��λΪm
	
	float XB,YB;//B�������
	float XD,YD;//D�������
	
	float XC,YC;//C���ֱ������
	float L0,phi0;//C��ļ�����
	float alpha;
	float d_alpha;	
	
	float lBD;//BD����ľ���
	
	float d_phi0;//����C��Ƕ�phi0�ı任��
	float last_phi0;//��һ��C��Ƕȣ����ڼ���Ƕ�phi0�ı任��d_phi0

	float A0,B0,C0;//�м����
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//�ѿ����ռ������ؽڿռ�������ſɱȾ���ϵ��
	float torque_set[2];

	float F0;
	float Tp;
	
	float theta;
	float d_theta;//theta��һ�׵���
	float last_d_theta;
	float dd_theta;//theta�Ķ��׵���
	
	float d_L0;//L0��һ�׵���
	float dd_L0;//L0�Ķ��׵���
	float last_L0;
	float last_d_L0;
	
	float FN;//֧����
	
	uint8_t first_flag;
	uint8_t leg_flag;//�ȳ���ɱ�־
}VMC_t;

class VMC_leg_t
{
public:
	VMC_t VMC_data;

	VMC_leg_t(float l1, float l2, float l3, float l4, float l5)
	{
        VMC_data.l1 = l1;
        VMC_data.l2 = l2;
        VMC_data.l3 = l3;
        VMC_data.l4 = l4;
        VMC_data.l5 = l5;
	}

	void Up_Left(float pitch_Angle, float pitch_Gyro, float dt);
	void Up_Right(float pitch_Angle, float pitch_Gyro, float dt);
	void Jacobian();

  bool ground_detection_L();
  bool ground_detection_R();
};

void VMC_leg_t::Up_Left(float pitch_Angle, float pitch_Gyro, float dt)
{
    static float Pitch_L=0.0f;
	static float Pith_GyroL=0.0f;
	Pitch_L = 0.0f - pitch_Angle;
	Pith_GyroL = 0.0f - pitch_Gyro;

	this->VMC_data.YD = this->VMC_data.l4 * arm_sin_f32(this->VMC_data.phi4);//D��y����
	this->VMC_data.YB = this->VMC_data.l1 * arm_sin_f32(this->VMC_data.phi1);//B��y����
	this->VMC_data.XD = this->VMC_data.l5 + this->VMC_data.l4 * arm_cos_f32(this->VMC_data.phi4);//D��x����
	this->VMC_data.XB = this->VMC_data.l1 * arm_cos_f32(this->VMC_data.phi1); //B��x����
			
	this->VMC_data.lBD = sqrt((this->VMC_data.XD - this->VMC_data.XB)*(this->VMC_data.XD - this->VMC_data.XB) + (this->VMC_data.YD - this->VMC_data. YB) * (this->VMC_data.YD - this->VMC_data.YB));
	
	this->VMC_data.A0 = 2*this->VMC_data.l2 * (this->VMC_data.XD - this->VMC_data.XB);
	this->VMC_data.B0 = 2*this->VMC_data.l2 * (this->VMC_data.YD - this->VMC_data.YB);
	this->VMC_data.C0 = this->VMC_data.l2 * this->VMC_data.l2 + this->VMC_data.lBD*this->VMC_data.lBD - this->VMC_data.l3 * this->VMC_data.l3;
	this->VMC_data.phi2 = 2*atan2f((this->VMC_data.B0 + sqrt(this->VMC_data.A0 * this->VMC_data.A0 + this->VMC_data.B0 * this->VMC_data.B0 - this->VMC_data.C0 * this->VMC_data.C0)), this->VMC_data.A0 + this->VMC_data.C0);			
	this->VMC_data.phi3 = atan2f(this->VMC_data.YB - this->VMC_data.YD + this->VMC_data.l2 * arm_sin_f32(this->VMC_data.phi2), this->VMC_data.XB - this->VMC_data.XD + this->VMC_data.l2 * arm_cos_f32(this->VMC_data.phi2));
	//C��ֱ������
	this->VMC_data.XC = this->VMC_data.l1 * arm_cos_f32(this->VMC_data.phi1) + this->VMC_data.l2 * arm_cos_f32(this->VMC_data.phi2);
	this->VMC_data.YC = this->VMC_data.l1 * arm_sin_f32(this->VMC_data.phi1) + this->VMC_data.l2  *arm_sin_f32(this->VMC_data.phi2);
	//C�㼫����
	this->VMC_data.L0 = sqrt((this->VMC_data.XC - this->VMC_data.l5/2.0f) * (this->VMC_data.XC - this->VMC_data.l5/2.0f) + this->VMC_data.YC * this->VMC_data.YC);
		
	this->VMC_data.phi0 = atan2f(this->VMC_data.YC,(this->VMC_data.XC - this->VMC_data.l5/2.0f));//phi0���ڼ���lqr��Ҫ��theta		
	this->VMC_data.alpha = pi/2.0f-this->VMC_data.phi0 ;
		
	if(this->VMC_data.first_flag == 0)
	{
		this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
		this->VMC_data.first_flag = 1;
	}

	this->VMC_data.d_phi0 = (this->VMC_data.phi0 - this->VMC_data.last_phi0)/dt;//����phi0�仯�ʣ�d_phi0���ڼ���lqr��Ҫ��d_theta
	this->VMC_data.d_alpha = 0.0f - this->VMC_data.d_phi0 ;
		
	this->VMC_data.theta = pi/2.0f-Pitch_L - this->VMC_data.phi0;//�õ�״̬����1
	this->VMC_data.d_theta = (-Pith_GyroL - this->VMC_data.d_phi0);//�õ�״̬����2
		
	this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
    
	this->VMC_data.d_L0=(this->VMC_data.L0 - this->VMC_data.last_L0)/dt;//�ȳ�L0��һ�׵���
    this->VMC_data.dd_L0=(this->VMC_data.d_L0 - this->VMC_data.last_d_L0)/dt;//�ȳ�L0�Ķ��׵���
		
	this->VMC_data.last_d_L0 = this->VMC_data.d_L0;
	this->VMC_data.last_L0 = this->VMC_data.L0;
		
	this->VMC_data.dd_theta = (this->VMC_data.d_theta - this->VMC_data.last_d_theta)/dt;
	this->VMC_data.last_d_theta = this->VMC_data.d_theta;
}

void VMC_leg_t::Up_Right(float pitch_Angle, float pitch_Gyro, float dt)
{
    static float Pitch_R=0.0f;
	static float Pith_GyroR=0.0f;
	Pitch_R = pitch_Angle;
	Pith_GyroR = pitch_Gyro;

	this->VMC_data.YD = this->VMC_data.l4 * arm_sin_f32(this->VMC_data.phi4);//D��y����
	this->VMC_data.YB = this->VMC_data.l1 * arm_sin_f32(this->VMC_data.phi1);//B��y����
	this->VMC_data.XD = this->VMC_data.l5 + this->VMC_data.l4 * arm_cos_f32(this->VMC_data.phi4);//D��x����
	this->VMC_data.XB = this->VMC_data.l1 * arm_cos_f32(this->VMC_data.phi1); //B��x����
			
	this->VMC_data.lBD = sqrt((this->VMC_data.XD - this->VMC_data.XB)*(this->VMC_data.XD - this->VMC_data.XB) + (this->VMC_data.YD - this->VMC_data. YB) * (this->VMC_data.YD - this->VMC_data.YB));
	
	this->VMC_data.A0 = 2*this->VMC_data.l2 * (this->VMC_data.XD - this->VMC_data.XB);
	this->VMC_data.B0 = 2*this->VMC_data.l2 * (this->VMC_data.YD - this->VMC_data.YB);
	this->VMC_data.C0 = this->VMC_data.l2 * this->VMC_data.l2 + this->VMC_data.lBD*this->VMC_data.lBD - this->VMC_data.l3 * this->VMC_data.l3;
	this->VMC_data.phi2 = 2*atan2f((this->VMC_data.B0 + sqrt(this->VMC_data.A0 * this->VMC_data.A0 + this->VMC_data.B0 * this->VMC_data.B0 - this->VMC_data.C0 * this->VMC_data.C0)), this->VMC_data.A0 + this->VMC_data.C0);			
	this->VMC_data.phi3 = atan2f(this->VMC_data.YB - this->VMC_data.YD + this->VMC_data.l2 * arm_sin_f32(this->VMC_data.phi2), this->VMC_data.XB - this->VMC_data.XD + this->VMC_data.l2 * arm_cos_f32(this->VMC_data.phi2));
	//C��ֱ������
	this->VMC_data.XC = this->VMC_data.l1 * arm_cos_f32(this->VMC_data.phi1) + this->VMC_data.l2 * arm_cos_f32(this->VMC_data.phi2);
	this->VMC_data.YC = this->VMC_data.l1 * arm_sin_f32(this->VMC_data.phi1) + this->VMC_data.l2  *arm_sin_f32(this->VMC_data.phi2);
	//C�㼫����
	this->VMC_data.L0 = sqrt((this->VMC_data.XC - this->VMC_data.l5/2.0f) * (this->VMC_data.XC - this->VMC_data.l5/2.0f) + this->VMC_data.YC * this->VMC_data.YC);
		
	this->VMC_data.phi0 = atan2f(this->VMC_data.YC,(this->VMC_data.XC - this->VMC_data.l5/2.0f));//phi0���ڼ���lqr��Ҫ��theta		
	this->VMC_data.alpha = pi/2.0f-this->VMC_data.phi0 ;
		
	if(this->VMC_data.first_flag == 0)
	{
		this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
		this->VMC_data.first_flag = 1;
	}

	this->VMC_data.d_phi0 = (this->VMC_data.phi0 - this->VMC_data.last_phi0)/dt;//����phi0�仯�ʣ�d_phi0���ڼ���lqr��Ҫ��d_theta
	this->VMC_data.d_alpha = 0.0f - this->VMC_data.d_phi0 ;
		
	this->VMC_data.theta = pi/2.0f-Pitch_R - this->VMC_data.phi0;//�õ�״̬����1
	this->VMC_data.d_theta = (-Pith_GyroR - this->VMC_data.d_phi0);//�õ�״̬����2
		
	this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
    
	this->VMC_data.d_L0=(this->VMC_data.L0 - this->VMC_data.last_L0)/dt;//�ȳ�L0��һ�׵���
  this->VMC_data.dd_L0=(this->VMC_data.d_L0 - this->VMC_data.last_d_L0)/dt;//�ȳ�L0�Ķ��׵���
		
	this->VMC_data.last_d_L0 = this->VMC_data.d_L0;
	this->VMC_data.last_L0 = this->VMC_data.L0;
		
	this->VMC_data.dd_theta = (this->VMC_data.d_theta - this->VMC_data.last_d_theta)/dt;
	this->VMC_data.last_d_theta = this->VMC_data.d_theta;
}

void VMC_leg_t::Jacobian()
{
	this->VMC_data.j11 = (this->VMC_data.l1 * arm_sin_f32(this->VMC_data.phi0 - this->VMC_data.phi3) * arm_sin_f32(this->VMC_data.phi1 - this->VMC_data.phi2)) / arm_sin_f32(this->VMC_data.phi3-this->VMC_data.phi2);
	this->VMC_data.j12 = (this->VMC_data.l1 * arm_cos_f32(this->VMC_data.phi0 - this->VMC_data.phi3) * arm_sin_f32(this->VMC_data.phi1 - this->VMC_data.phi2)) / (this->VMC_data.L0 * arm_sin_f32(this->VMC_data.phi3 - this->VMC_data.phi2));
	this->VMC_data.j21 = (this->VMC_data.l4 * arm_sin_f32(this->VMC_data.phi0 - this->VMC_data.phi2) * arm_sin_f32(this->VMC_data.phi3 - this->VMC_data.phi4)) / arm_sin_f32(this->VMC_data.phi3-this->VMC_data.phi2);
	this->VMC_data.j22 = (this->VMC_data.l4 * arm_cos_f32(this->VMC_data.phi0 - this->VMC_data.phi2) * arm_sin_f32(this->VMC_data.phi3 - this->VMC_data.phi4)) / (this->VMC_data.L0 * arm_sin_f32(this->VMC_data.phi3 - this->VMC_data.phi2));
	
	this->VMC_data.torque_set[0] = this->VMC_data.j11 * this->VMC_data.F0 + this->VMC_data.j12 * this->VMC_data.Tp;//�õ�RightFront��������������أ�F0Ϊ�����˻���ĩ�����ȵ����� 
	this->VMC_data.torque_set[1] = this->VMC_data.j21 * this->VMC_data.F0 + this->VMC_data.j22 * this->VMC_data.Tp;//�õ�RightBack��������������أ�TpΪ������������� 
}

bool VMC_leg_t::ground_detection_R()
{
	this->VMC_data.FN = this->VMC_data.F0 * arm_cos_f32(this->VMC_data.theta)+this->VMC_data.Tp*arm_sin_f32(this->VMC_data.theta)/this->VMC_data.L0+6.0f;//�Ȳ���������+���������������������������*��������ֱ�����˶����ٶ�
//	vmc->FN=vmc->F0*arm_arm_cos_f32_f32(vmc->theta)+vmc->Tp*arm_arm_sin_f32_f32(vmc->theta)/vmc->L0
//+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_arm_cos_f32_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_arm_sin_f32_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_arm_sin_f32_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_arm_cos_f32_f32(vmc->theta));
 
	if(this->VMC_data.FN<5.0f)
	{
        //�����
	  return true;
	}
	else
	{
	  return false;	
	}
}

bool VMC_leg_t::ground_detection_L()
{
	this->VMC_data.FN = this->VMC_data.F0 * arm_cos_f32(this->VMC_data.theta)+this->VMC_data.Tp*arm_sin_f32(this->VMC_data.theta)/this->VMC_data.L0+6.0f;//�Ȳ���������+���������������������������*��������ֱ�����˶����ٶ�
//	vmc->FN=vmc->F0*arm_arm_cos_f32_f32(vmc->theta)+vmc->Tp*arm_arm_sin_f32_f32(vmc->theta)/vmc->L0
//+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_arm_cos_f32_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_arm_sin_f32_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_arm_sin_f32_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_arm_cos_f32_f32(vmc->theta));
 
	if(this->VMC_data.FN<5.0f)
	{
        //�����
	  return true;
	}
	else
	{
	  return false;	
	}
}

float LQR_K_calc(float *coe,float len)
{
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}
