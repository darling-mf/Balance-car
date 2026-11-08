#include "app_control.h"
#include "pid.h"
#include "task.h"
#include "app_mpu6050.h"
#include "qmath.h"
#include "app_motor.h"
#include "app_encoder.h"

//如果量不变，则声明为const
//RAM的空间十分宝贵，常量尽量放在FLASH中
static const float Pi =  3.1415927f;
static const float g = 9.8;
static const float Lp = 0.062;
static const float Rw = 0.032;

//从上到下：从外环到内环
static PID_TypeDef pid_velocity; //速度环的PI控制器
static PID_TypeDef pid_theta; //θ环的PID控制器
static PID_TypeDef pid_theta_dot; //θ导的PID控制器
static PID_TypeDef pid_turn; //转向环的PID控制器
//
// @简介：负责初始化平衡车控制系统
//
void App_Control_Init(void)
{
	PID_Init(&pid_velocity,10.0f,1.0f,0.0f);
	PID_LimitConfig(&pid_velocity,+0.5*g,-0.5*g); //速度限制在±0.5g
	
	PID_Init(&pid_theta,4.0f,0.0f,0.0f); 
	//设定每秒最多转两圈，2*2Πrad/s
	PID_LimitConfig(&pid_theta,+4*Pi,-4*Pi);
	
	PID_Init(&pid_theta_dot,10.0f,10.0f,0.0f);
	//角加速度*t =角速度，人为设定时间反推角加速度， 此处为0.1s
	PID_LimitConfig(&pid_theta,+40*Pi,-40*Pi);

	PID_Init(&pid_turn,1.0f,0.0f,0.0f);
	//转速限制在最大弧度15rad/s，即转速差在±15rad/s
	PID_LimitConfig(&pid_turn,+15.0f,-15.0f);
	
}

static float omega_ref;
static uint64_t lastTime =0;
//
// @简介：平衡车控制系统的进程函数
//
void App_Control_Proc(void)
{
	//5ms执行一次
	PERIODIC(5);
	float now = GetUs();
	
	float deltaT = (now -lastTime) * 1.0e-6f;
	
	// #-1. 改变速度环的设定值
	//PID_ChangeSP(&pid_velocity,0.0f);
	
	// #-2. 读取传感器的值(取左右轮的平均值)
	float omega = 0.5*(App_Encoder_GetSpeed_L() + App_Encoder_GetSpeed_R());
	float theta = App_MPU6050_GetPitch()/180*Pi; //读取的值是°/s，不符国际单位
	float theta_dot = App_MPU6050_GetGx()/180*Pi;
	
	// #-3. 计算速度环的反馈值
	float omega2 = -theta_dot *(Lp+Rw)/Rw;
	float omega1 = omega - omega2;
	float x_dot = omega1 * Rw;
	
	//  #-4. 速度环的PID计算
	float theta_ref = qatan(PID_Compute(&pid_velocity,x_dot)/g);
	
	
	// #1. 将外环的设定值SP设置为0
	PID_ChangeSP(&pid_theta,theta_ref);
	

	// #2. 计算外环PID的输出 
		//外环的输出值就是内环的理论输入值
	float theta_dot_ref = PID_Compute(&pid_theta,theta);
	
	// #3. 改变内环的设定值SP
	PID_ChangeSP(&pid_theta_dot,theta_dot_ref);
	
	// #4.计算内环PID的输出
	float theta_dot_dot_ref = PID_Compute(&pid_theta_dot,theta_dot);
	
	// #5. 倒立摆的逆解算
	float x_dot_dot_ref = (g * qsin(theta) - theta_dot_dot_ref*Lp)/qcos(theta);
	
	// #6. 计算轮胎转速
		//此处的积分，近似看成了矩形
	if(lastTime != 0)
	{
		omega_ref += 1.0f/Rw * x_dot_dot_ref *deltaT ;
	}
	// #7. 转向环  单位rad/s
	float gz = App_MPU6050_GetGz()/180*Pi;
	float omega_diff = PID_Compute(&pid_turn,gz);
	
	// #8.设置轮胎的转速
	App_Motor_SetOmega_L(omega_ref + omega_diff);
	App_Motor_SetOmega_R(omega_ref - omega_diff);

	lastTime = now;
}

//
// @简介：对控制系统进行复位
//
void App_Control_Reset(void)
{
	//#1. 暂存值
	lastTime = 0;
	omega_ref =0;
	
	//#2. PID控制器
	PID_Reset(&pid_velocity);
	PID_Reset(&pid_theta);
	PID_Reset(&pid_theta_dot);
}

//
// @简介：改变平衡车移动的速度
// @参数：MoveSpeed - 速度 单位m/s,最大行进速度0.7m/s
//
void App_Control_SetMoveSpeed(float MoveSpeed)
{
	PID_ChangeSP(&pid_velocity,MoveSpeed);
}
//
// @简介：改变平衡车转向的速度
// @参数：TurnSpeed - 转速 单位rad/s,最大转速为15rad/s
//
void App_Control_SetTurnSpeed(float TurnSpeed)
{
		PID_ChangeSP(&pid_turn,TurnSpeed);

}