/*
	MPU6050 Interfacing with Raspberry Pi
	http://www.electronicwings.com
*/

#include <wiringPiI2C.h>
#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <unistd.h>  // 包含 usleep 函数
#include <sys/time.h>  // 包含时间戳函数

#define Device_Address 0x68	/*Device Address/Identifier for MPU6050*/

#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

int fd;

void MPU6050_Init(){
	
	wiringPiI2CWriteReg8 (fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
	wiringPiI2CWriteReg8 (fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
	wiringPiI2CWriteReg8 (fd, CONFIG, 0);		/* Write to Configuration register */
	wiringPiI2CWriteReg8 (fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
	wiringPiI2CWriteReg8 (fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */

	} 
short read_raw_data(int addr){
	short high_byte,low_byte,value;
	high_byte = wiringPiI2CReadReg8(fd, addr);
	low_byte = wiringPiI2CReadReg8(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

void ms_delay(int val){
	int i,j;
	for(i=0;i<=val;i++)
		for(j=0;j<1200;j++);
}

int main(){
	
	float Acc_x,Acc_y,Acc_z;
	float Gyro_x,Gyro_y,Gyro_z;
	float Ax=0, Ay=0, Az=0;
	float Gx=0, Gy=0, Gz=0;
	fd = wiringPiI2CSetup(Device_Address);   /*Initializes I2C with device Address*/
	MPU6050_Init();		                 /* Initializes MPU6050 */
	
	struct timeval start_time, end_time;
	double time_interval, total_time = 0;
	int read_count = 0;
	int max_read_count = 1000;  // 定义要计算读取频率的循环次数
	
	while(read_count < max_read_count)
	{
		gettimeofday(&start_time, NULL);  // 记录循环开始时间
		
		/*Read raw value of Accelerometer and gyroscope from MPU6050*/
		Acc_x = read_raw_data(ACCEL_XOUT_H);
		Acc_y = read_raw_data(ACCEL_YOUT_H);
		Acc_z = read_raw_data(ACCEL_ZOUT_H);
		
		// Gyro_x = read_raw_data(GYRO_XOUT_H);
		// Gyro_y = read_raw_data(GYRO_YOUT_H);
		// Gyro_z = read_raw_data(GYRO_ZOUT_H);
		
		// /* Divide raw value by sensitivity scale factor */
		// Ax = Acc_x/16384.0;
		// Ay = Acc_y/16384.0;
		// Az = Acc_z/16384.0;
		
		// Gx = Gyro_x/131;
		// Gy = Gyro_y/131;
		// Gz = Gyro_z/131;
		
		// printf("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n", Gx, Gy, Gz, Ax, Ay, Az);
		
		gettimeofday(&end_time, NULL);  // 记录循环结束时间
		
		// 计算当前循环的时间间隔
		time_interval = (end_time.tv_sec - start_time.tv_sec) + 
		              (end_time.tv_usec - start_time.tv_usec) / 1000000.0;
		
		// 累加时间间隔
		total_time += time_interval;
		
		// 增加读取计数
		read_count++;
		
		// 延迟 500 毫秒
		// usleep(500000);
	}
	
	// 计算平均时间间隔
	double average_time_interval = total_time / max_read_count;
	
	// 计算读取频率
	double read_frequency = 1.0 / average_time_interval;
	
	printf("平均读取时间间隔: %.4f 秒\n", average_time_interval);
	printf("读取频率: %.2f Hz\n", read_frequency);
	
	return 0;
}
