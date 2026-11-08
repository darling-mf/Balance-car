#include "app_mpu6050.h"
#include "i2c.h"
#include "delay.h"
#include "task.h"
#include "math.h"
#include "qmath.h"
static float ax,ay,az; //加速度计的结果,单位g
static float temperature; //温度计的结果,单位℃
static float gx,gy,gz; //陀螺仪的结果,单位°/s
static float yaw,pitch,roll; //欧拉角，单位°
// reg地址，value值
static void reg_write(uint8_t reg,uint8_t value);
static uint8_t reg_read(uint8_t reg);
static void reg_read_burst(uint8_t reg,uint8_t *buf,uint8_t len);
//
// @简介：对MPU6050初始化
//
void  App_MPU6050_Init(void)
{
	// #1.初始化IIC PB8 PB9 - I2C1
		// 将I2C1的引脚重映射PB8 PB9
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_I2C1,ENABLE);
	
		// 初始化PB8 PB9 - AF_OD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef GPIO_InitStruct ={0};
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	// #2.初始化I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE);
	I2C_InitTypeDef I2C_InitStruct ={0};
	I2C_InitStruct.I2C_ClockSpeed = 400000; // 通讯速度(查手册)
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 占空比
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // 模式
	I2C_Init(I2C1,&I2C_InitStruct);
	
	// #3.设置MPU6050的参数
	reg_write(0x6d,0x80); //复位
	Delay(100);
	
	reg_write(0x6d,0x00); //关闭睡眠模式
	
	reg_write(0x1b,0x18); //陀螺仪量程，±2000°/s
	reg_write(0x1c,0x00); //加速度计量程，正负2g
}

//
// @简介：MPU6050的进程函数
//
void App_MPU6050_Proc(void)
{
	PERIODIC(5); //进程函数每5ms执行一次
	//这两都封装在了PERIODIC的宏中
	//static uint32_t nxt = 0; //下次程序运行的时间
	//if(GetTick()<nxt) return;
	App_MPU6050_Update(); //更新传感器的值
	
	//通过陀螺仪的测量结果计算欧拉角
	float yaw_g = yaw + gz*0.005;
	float pitch_g	 = pitch  + gx*0.005;
	float roll_g = roll - gy*0.005;

	//通过加速度计解算欧拉角
	float yaw_a = 0.0f;
	float pitch_a = qatan2(ay,az)/3.1415927f * 180.0f;
	float roll_a = qatan2(ax,az)/3.1415927f * 180.0f; 
	
	// 使用互补滤波器对陀螺仪和加速度计的计算结果进行融合
	yaw = yaw_g;
	pitch = 0.95238 * pitch_g + (1- 0.95238)*pitch_a;
	roll = 0.95238 * roll_g + (1- 0.95238)*roll_a;
}

//
// @简介：更新MPU6050的值
//
void App_MPU6050_Update(void)
{
	// #1. 一次性从 0x3B 连续读 14 字节：AX/AY/AZ/TEMP/GX/GY/GZ
	//     相比逐寄存器读（14 次 I2C 事务），burst read 只需 1 次，减少总线占用与采样抖动
	uint8_t buf[14];
	reg_read_burst(0x3b,buf,14);
	
	int16_t ax_raw = (int16_t)((buf[0]<< 8) | buf[1]); //ax的原始数据
	int16_t ay_raw = (int16_t)((buf[2]<< 8) | buf[3]); //ay的原始数据
	int16_t az_raw = (int16_t)((buf[4]<< 8) | buf[5]); //az的原始数据
	
	ax = ax_raw/32768.0f *2.0f;
	ay = ay_raw/32768.0f *2.0f;
	az = az_raw/32768.0f *2.0f;
	
	int16_t temperature_raw = (int16_t)((buf[6]<< 8) | buf[7]); //温度计的原始数据
	
	//temperature = temperature_raw / 340.0f +36.53f; //MPU6050的计算公式
	temperature = (temperature_raw - 0) / 333.87f + 21.0f; // MPU6500
	
	int16_t gx_raw = (int16_t)((buf[8]<< 8)  | buf[9]);  //gx的原始数据
	int16_t gy_raw = (int16_t)((buf[10]<< 8) | buf[11]); //gy的原始数据
	int16_t gz_raw = (int16_t)((buf[12]<< 8) | buf[13]); //gz的原始数据
	
	gx = gx_raw/32768.0f *2000.0f;
	gy = gy_raw/32768.0f *2000.0f;
	gz = gz_raw/32768.0f *2000.0f;
}

//
// @简介：获取数据,单位g、℃、°/s
//	      跟角有关的都是°/s
//
float App_MPU6050_GetAx(void)
{
	return ax;
}
float App_MPU6050_GetAy(void)
{
	return ay;
}
float App_MPU6050_GetAz(void)
{
	return az;
}

float App_MPU6050_GetTemperature(void)
{
	return temperature;
}

float App_MPU6050_GetGx(void)
{
	return gx;
}
float App_MPU6050_GetGy(void)
{
	return gy;
}
float App_MPU6050_GetGz(void)
{
	return gz;
}
float App_MPU6050_GetYaw(void)
{
	return yaw;
}
float App_MPU6050_GetPitch(void)
{
	return pitch;
}
float App_MPU6050_GetRoll(void)
{
	return roll;
}
//
// @简介：向寄存器写值
// @参数 reg - 要写入的寄存器的地址
// @参数 value - 要写入的值
//
static void reg_write(uint8_t reg,uint8_t value)
{
	uint8_t bytesTOsend[] = {reg,value};
	My_I2C_SendBytes(I2C1,0xd0,bytesTOsend,2);
}

//
// @简介：读取寄存器写值
// @参数 reg - 要读取的寄存器的地址
// @返回值，表示要读到的值
//
static uint8_t reg_read(uint8_t reg)
{
	My_I2C_SendBytes(I2C1,0xd0,&reg,1); //发送寄存器的地址
	uint8_t regValue ;
	My_I2C_ReceiveBytes(I2C1,0xd0,&regValue,1); //读取一个字节
	
	return regValue;
}
//
// @简介：从指定寄存器开始连续读取 len 字节（burst read）
// @参数 reg - 起始寄存器地址
// @参数 buf - 输出缓冲区
// @参数 len - 读取字节数
// @note  I2C 驱动内部对前 len-1 字节回 ACK、最后一字节回 NACK 并 STOP
//
static void reg_read_burst(uint8_t reg,uint8_t *buf,uint8_t len)
{
	My_I2C_SendBytes(I2C1,0xd0,&reg,1);        //发送起始寄存器地址
	My_I2C_ReceiveBytes(I2C1,0xd0,buf,len);    //连续读取 len 字节
}