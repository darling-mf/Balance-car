#include "app_encoder.h"
#include "delay.h"

//凡是在中断会被修改的值都要加volatile
//encoder只能在本文件使用
static volatile int64_t encoder_l = 0; //左电机编码器的值
static volatile int64_t encoder_r = 0; //右电机编码器的值
static volatile int8_t direction_l =1; //左电机方向  1 - 正转 -1 -反转
static volatile int8_t direction_r =1; //右电机方向  1 - 正转 -1 -反转
static volatile uint64_t t0_l = 0,t1_l = 0; //左电机的变化时间，单位Us
static volatile uint64_t t0_r = 0,t1_r = 0; //右电机的变化时间，单位Us

static void Encoder_L_Init(void); //左编码器初始化
static void Encoder_R_Init(void); //右编码器初始化
//
// @简介：对编码器模块进行初始化
//
void App_Encoder_Init(void)
{
	Encoder_L_Init();
	Encoder_R_Init();
}

//
// @简介：读取左/右轮胎的角度，单位：度
//				计算旋转角度
//
float App_Encoder_GetPos_L(void)
{
	//轮子圈数 = 编码器转数 /22 /减速比
	//旋转角度 = 圈数 * 360°
	return encoder_l / 22.0f / (30613.0f / 1500.0f) * 360.0f;  
}
float App_Encoder_GetPos_R(void)
{
	return encoder_r / 22.0f / (30613.0f / 1500.0f) * 360.0f; 
}

//
// @简介：读取左/右轮胎旋转角速度的值(omega)，单位：rad/s
//
float App_Encoder_GetSpeed_L(void)
{
	__disable_irq(); //关闭单片机的总中断
	// 赋值一份
	int8_t direction_cpy = direction_l; 
	uint64_t t0_cpy = t0_l;
	uint64_t t1_cpy = t1_l;
	
	__enable_irq();
	
	if(direction_cpy == 2 || direction_cpy == -2)
	{
		return 0;
	}
	uint64_t now = GetUs();
	float T = 0;
	if(t0_cpy - t1_cpy > now - t0_cpy)
	{
		T = (t0_cpy - t1_cpy) * 1.0e-6f;
	}
	else
	{
		T = (now - t0_cpy) * 1.0e-6f;
	}			
	return direction_cpy/T/22.0f/(30613.0f/1500.0f) * 6.2831853f; //微秒 1e-6秒 //2*3.14
}
float App_Encoder_GetSpeed_R(void)
{
	__disable_irq(); //关闭单片机的总中断
	// 赋值一份
	int8_t direction_cpy = direction_r; 
	uint64_t t0_cpy = t0_r;
	uint64_t t1_cpy = t1_r;
	
	__enable_irq();
	if(direction_cpy == 2 || direction_cpy == -2)
	{
		return 0;
	}
	uint64_t now = GetUs();
	float T = 0;
	if(t0_cpy - t1_cpy > now - t0_cpy)
	{
		T = (t0_cpy - t1_cpy) * 1.0e-6f;
	}
	else
	{
		T = (now - t0_cpy) * 1.0e-6f;
	}			
	return direction_cpy/T/22.0f/(30613.0f/1500.0f) * 6.2831853f; //微秒 1e-6秒
}

//
// @简介：左编码器初始化
//
static void Encoder_L_Init(void)
{
	// 初始化引脚
	// PB14,PB15 - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	// EXTI 初始化
	// 让EXTI_Line14监控PB14 (引脚接口归AFIO模块管）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource14);
	
	// 配置EXTI参数
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	EXTI_InitStruct.EXTI_Line = EXTI_Line14;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	//开启EXTI中断
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; //优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0; //优先级
	NVIC_Init(&NVIC_InitStruct);
}

//
// @简介：右编码器初始化
//
static void Encoder_R_Init(void)
{
	// 初始化引脚
	// 需要重映射，先关闭JTAG，开启SWD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	// PB3,PB4 - IPU
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	// EXTI初始化
	// 让EXTI_Line3监控PB3 (引脚接口归AFIO模块管）
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource3);
	
	// 配置EXTI参数
	EXTI_InitTypeDef EXTI_InitStruct = {0};
	EXTI_InitStruct.EXTI_Line = EXTI_Line3;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	
	EXTI_Init(&EXTI_InitStruct);
	
	//开启EXTI中断
	NVIC_InitTypeDef NVIC_InitStruct = {0};
	NVIC_InitStruct.NVIC_IRQChannel = EXTI3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0; //优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0; //优先级
	NVIC_Init(&NVIC_InitStruct);
}

//
// @简介：EXTI15_10的中断响应函数，对应左编码器的A相
//				每次上/下沿都会触发一次中断，一次中断会让编码器次数±1
//				所以编码器只会一直上/下，记录两次相邻的中断时间作差即可算出▲t
//
void EXTI15_10_IRQHandler(void)
{
	// 需判断是否真是Line14的中断
	if(EXTI_GetFlagStatus(EXTI_Line14) == SET)
	{
		//对中断标志位清零 (EXTI只有一个标志位)
		EXTI_ClearFlag(EXTI_Line14);
		
		//记录中断时间，即可判断▲t
		t1_l = t0_l;
		t0_l = GetUs();
		
		uint8_t a = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);  //A相电压
		uint8_t b = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15);  //B相电压
		//中断响应函数不会立即执行，而是延迟一小段；因此此时a的电压已经是变化后的值了
		//处理换向点
		if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET)) // 现在轮胎反转
		{
			encoder_l --;
			if(direction_l >0) //之前轮胎是正转的
			{
				direction_l = -2;
			}
			else
			{
				direction_l = -1;
			}
		}
		else //现在轮胎正转
		{
			encoder_l ++;
			if(direction_l <0) //之前轮胎是反转的
			{
				direction_l = 2;
			}
			else
			{
			direction_l  = 1;
			}
		}
	}
}	

//
// @简介：EXTI3的中断响应函数，对应右编码器的A相
//				每次上/下沿都会触发一次中断，一次中断会让编码器次数±1
//				所以编码器只会一直上/下，记录两次相邻的中断时间作差即可算出▲t
//
void EXTI3_IRQHandler(void)
{
	//对中断标志位清零 (EXTI只有一个标志位)
	EXTI_ClearFlag(EXTI_Line3);
	
	//记录中断时间，即可判断▲t
	t1_r = t0_r;
	t0_r = GetUs();
	
	uint8_t a = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3);  //A相电压
	uint8_t b = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4);  //B相电压
	
	//中断响应函数不会立即执行，而是延迟一小段；因此此时a的电压已经是变化后的值了
	//处理换向点
	if((a == Bit_SET && b == Bit_RESET) || (a == Bit_RESET && b == Bit_SET)) //现在轮胎正转
	{
		encoder_r ++;
		if(direction_r <0) //之前轮胎是反转的
		{
			direction_r = 2;
		}
		else
		{
			direction_r = 1;
		}
	}
	else //现在轮胎反转
	{
		encoder_r --;
		if(direction_r >0) //之前轮胎是正转的
		{
			direction_r = -2;
		}
		else
		{
			direction_r  = -1;
		}
	}
//	if(a == Bit_SET)
//	{
//		if(b == Bit_RESET)
//		{
//			encoder_r++;
//			direction_r = 1;
//		}
//		else
//		{
//			encoder_r--;
//			direction_r = -1;
//		}
//	}
//	else//下降沿
//	{
//		if (b == Bit_RESET)
//		{
//			encoder_r--;
//			direction_r = -1;
//		}
//		else
//		{
//			encoder_r++;
//			direction_r =1;
//		}
//	}
}

