#include "pid.h"
#include "delay.h"
//
// @简介：初始化PID
// @参数 Kp - 比例系数
// @参数 Ki - 积分系数
// @参数 Kd - 微分系数
//
void PID_Init(PID_TypeDef *PID,float Kp,float Ki,float Kd)
{
	PID->Kp = Kp;
	PID->Ki = Ki;
	PID->Kd = Kd;
	PID->SP = 0.0f;
	
	PID->t_k_1 = 0;
	PID->err_k_1 = 0.0f;
	PID->err_int_k_1 = 0.0f;
	
	// 即初始化中，并无限制电压(-∞< x <+∞)，让用户去限制
	//如果不进行赋值，则默认为(0< x <0)
	PID->UpperLimit = 3.4e+38f; //设定初始值，此处为无穷大
	PID->LowerLimit = -3.4e+38f; //设定初始值，此处为无穷小
	
	// 积分限幅初值也设为无穷，随后由 PID_LimitConfig() 同步为输出限幅
	PID->IntUpperLimit = 3.4e+38f;
	PID->IntLowerLimit = -3.4e+38f;
}

//
// @简介：设置PID控制器门限
// @参数 Upper - 上限值
// @参数 Lower - 下限值
//
void PID_LimitConfig(PID_TypeDef *PID,float Upper,float Lower)
{
	PID->UpperLimit = Upper;
	PID->LowerLimit = Lower;
	
	// 积分限幅默认与输出限幅保持一致（保持原有手感），
	// 需要单独限制积分时再调用 PID_IntLimitConfig() 覆盖
	PID->IntUpperLimit = Upper;
	PID->IntLowerLimit = Lower;
}

//
// @简介：单独设置积分项的门限
// @参数 Upper - 积分上限
// @参数 Lower - 积分下限
// @note  积分限幅与输出限幅量纲不同（积分项是误差的时间累积），
//        因此允许独立配置，避免用输出限幅去卡积分
//
void PID_IntLimitConfig(PID_TypeDef *PID,float Upper,float Lower)
{
	PID->IntUpperLimit = Upper;
	PID->IntLowerLimit = Lower;
}

//
// @简介：改变设定值SP
// @参数 SP - 新的设定值
//
void PID_ChangeSP(PID_TypeDef *PID,float SP)
{
	PID->SP = SP;
}

//
// @简介：执行一次PID运算
// @参数 FB - 反馈值(传感器的采回值)
// @返回值：PID控制器计算的结果
//
float PID_Compute(PID_TypeDef *PID,float FB)
{
	float err = PID->SP - FB; //设定值-反馈值
	uint64_t t_k = GetUs();
	
	float deltaT = (t_k - PID->t_k_1) * 1.0e-6f;
	float err_dev = 0.0f;
	float err_int = 0.0f;
	
	if(PID->t_k_1 != 0)
	{
		err_dev = (err - PID->err_k_1)/deltaT;
		err_int = PID->err_int_k_1 + (err + PID->err_k_1)*deltaT * 0.5f;
	}
	float COp = PID->Kp * err;
	float COi = PID->Ki * err_int;
	float COd = PID->Kd * err_dev;
	float CO = COp+COi+COd;
	
	//更新
	PID->t_k_1 = t_k;
	PID->err_k_1 = err;
	PID->err_int_k_1 = err_int;
	
	//输出限幅
	if(CO > PID->UpperLimit) CO = PID->UpperLimit;
	if(CO < PID->LowerLimit) CO = PID->LowerLimit;
	
	//积分限幅（用积分专用门限，而不是输出限幅）
	if(PID->err_int_k_1 > PID->IntUpperLimit) PID->err_int_k_1 = PID->IntUpperLimit;
	if(PID->err_int_k_1 < PID->IntLowerLimit) PID->err_int_k_1 = PID->IntLowerLimit;
	
	return CO;
}

//
// @简介：对PID控制器进行复位
//
void PID_Reset(PID_TypeDef *PID)
{
	PID->t_k_1 = 0;
	PID->err_k_1 = 0;
	PID->err_int_k_1 = 0;
}