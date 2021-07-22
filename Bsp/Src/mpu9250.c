#include "mpu9250.h"
#include "i2c.h"

short Accel[3]; // 加速度计
short Gyro[3];	// 陀螺仪
short Mag[3];	// 磁力计
// short gyroX, gyroY, gyroZ;	  //三个轴陀螺仪
// short accelX, accelY, accelZ; //三个轴加速度计
// short magX, magY, magZ;		  //三个轴磁力计

// I2C 写，根据第 4 节写寄存器的时序方法，实现写一个字节到从机

// 初始化

uint8_t MPU9250_Init(void)
{
	if (I2C_Read_Reg(GYRO_ADDRESS, WHO_AM_I) == 0x71)
	{
		I2C_Write_Reg(GYRO_ADDRESS, PWR_MGMT_1, 0x00);	 //解除休眠状态
		I2C_Write_Reg(GYRO_ADDRESS, SMPLRT_DIV, 0x07);	 //采样频率 125Hz
		I2C_Write_Reg(GYRO_ADDRESS, CONFIG, 0X06);		 //低通滤波器 5Hz
		I2C_Write_Reg(GYRO_ADDRESS, GYRO_CONFIG, 0X18);	 //陀螺仪量程,正负 2000 度
		I2C_Write_Reg(GYRO_ADDRESS, ACCEL_CONFIG, 0X18); //加速度量程,正负 16g
		return 0;
	}
	return 1;
}
//读取加速度数据的函数
void MPU9250_READ_ACCEL(short *accData)
{
	uint8_t BUF[6];
	BUF[0] = I2C_Read_Reg(ACCEL_ADDRESS, ACCEL_XOUT_L); //读 X 加速度低字节
	BUF[1] = I2C_Read_Reg(ACCEL_ADDRESS, ACCEL_XOUT_H); //读 X 加速度高字节
	accelX = (BUF[1] << 8) | BUF[0];
	BUF[2] = I2C_Read_Reg(ACCEL_ADDRESS, ACCEL_YOUT_L); //读 Y 加速度低字节
	BUF[3] = I2C_Read_Reg(ACCEL_ADDRESS, ACCEL_YOUT_H); //读 Y 加速度高字节
	accelY = (BUF[3] << 8) | BUF[2];
	BUF[4] = I2C_Read_Reg(ACCEL_ADDRESS, ACCEL_ZOUT_L); //读 Z 加速度低字节
	BUF[5] = I2C_Read_Reg(ACCEL_ADDRESS, ACCEL_ZOUT_H); //读 Z 加速度高字节
	accelZ = (BUF[5] << 8) | BUF[4];
}
//读取角速度数据的函数
void MPU9250_READ_GYRO(short *gyroData)
{
	uint8_t BUF[8];
	BUF[0] = I2C_Read_Reg(GYRO_ADDRESS, GYRO_XOUT_L); //读 X 角速度低字节
	BUF[1] = I2C_Read_Reg(GYRO_ADDRESS, GYRO_XOUT_H); //读 X 角速度高字节
	gyroX = (BUF[1] << 8) | BUF[0];
	BUF[2] = I2C_Read_Reg(GYRO_ADDRESS, GYRO_YOUT_L); //读 Y 角速度低字节
	BUF[3] = I2C_Read_Reg(GYRO_ADDRESS, GYRO_YOUT_H); //读 Y 角速度高字节
	gyroY = (BUF[3] << 8) | BUF[2];
	BUF[4] = I2C_Read_Reg(GYRO_ADDRESS, GYRO_ZOUT_L); //读 Z 角速度低字节
	BUF[5] = I2C_Read_Reg(GYRO_ADDRESS, GYRO_ZOUT_H); //读 Z 角速度高字节
	gyroZ = (BUF[5] << 8) | BUF[4];
}
//读取磁力计数据的函数
void MPU9250_READ_MAG(short *magData)
{
	uint8_t BUF[6];
	I2C_Write_Reg(GYRO_ADDRESS, INT_PIN_CFG, 0x02); //turn on Bypass Mode
	delay_ms(10);
	I2C_Write_Reg(MAG_ADDRESS, 0x0A, 0x01); //用来启动单次转换,否则磁力计输出的数据不变
	delay_ms(10);
	BUF[0] = I2C_Read_Reg(MAG_ADDRESS, MAG_XOUT_L); //读 X 磁力计低字节
	BUF[1] = I2C_Read_Reg(MAG_ADDRESS, MAG_XOUT_H); //读 X 磁力计高字节
	magX = (BUF[1] << 8) | BUF[0];
	BUF[2] = I2C_Read_Reg(MAG_ADDRESS, MAG_YOUT_L); //读 Y 磁力计低字节
	BUF[3] = I2C_Read_Reg(MAG_ADDRESS, MAG_YOUT_H); //读 Y 磁力计高字节
	magY = (BUF[3] << 8) | BUF[2];
	BUF[4] = I2C_Read_Reg(MAG_ADDRESS, MAG_ZOUT_L); //读 Z 磁力计低字节
	BUF[5] = I2C_Read_Reg(MAG_ADDRESS, MAG_ZOUT_H); //读 Z 磁力计高字节
	magZ = (BUF[5] << 8) | BUF[4];
}

void readImu() //读取 IMU 数据函数
{
	MPU9250_READ_ACCEL(Accel); //读取加速度
	MPU9250_READ_GYRO(Gyro);   //读取角速度
	MPU9250_READ_MAG(Mag);	   //读取磁力计
}
