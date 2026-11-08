#include "app_motor.h"
#include "pid.h"
#include "task.h"
#include "app_encoder.h"
#include "app_bat.h"
#include "app_pwm.h"

static PID_TypeDef pid_motor_l; //左电机调速系统的PID控制器
static PID_TypeDef pid_motor_r; //右电机调速系统的PID控制器

//
// @简介：初始化左右电机调速系统
//
void App_Motor_Init(void)
{
	PID_Init(&pid_motor_l,0.5,7,0); //PI控制器
	PID_LimitConfig(&pid_motor_l,+8.4f,-8.4f);
	PID_Init(&pid_motor_r,0.5,7,0); //PI控制器
	PID_LimitConfig(&pid_motor_r,+8.4f,-8.4f);
}

//
// @简介：电机调速系统的进程函数
//
void App_Motor_Proc(void)
{
	PERIODIC(1); //每间隔1毫秒执行一次
	
	// #1. 通过编码器获取左右电机旋转的角速度
		//实际角速度，即系统输出和值、FB
	float omega_l = App_Encoder_GetSpeed_L();
	float omega_r = App_Encoder_GetSpeed_R();
	
	// #2. 计算PID控制器的输出 (传递反馈值FB)
	float ua_l = PID_Compute(&pid_motor_l,omega_l);
	float ua_r = PID_Compute(&pid_motor_r,omega_r);
	
	// #3. 将电压ua设置到电机两端
		//获取电池电压vbat
	float vbat = App_Bat_Get();
	
	float duty_l = ua_l / vbat *100.0f;
	float duty_r = ua_r / vbat *100.0f;
	
	App_PWM_Set_L(duty_l);
	App_PWM_Set_R(duty_r);

}

//
// @简介：用来设置左右电机的转速omega的值
// @参数：Omega - 表示电机的转数 ，单位是：rad/s
//
void App_Motor_SetOmega_L(float Omega)
{
	PID_ChangeSP(&pid_motor_l,Omega);
}
void App_Motor_SetOmega_R(float Omega)
{
	PID_ChangeSP(&pid_motor_r,Omega);
}

///
// @简介：开关电机
// @参数 On:控制电机开关，0 - 关闭  非零 - 开启
//
void App_Motor_Cmd(uint8_t On)
{
	App_PWM_Cmd(On);
	//对PID控制器进行复位
	PID_Reset(&pid_motor_l);
	PID_Reset(&pid_motor_r);
}
