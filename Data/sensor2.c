#include <stdio.h>
#include <string>
#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>
#include "hmc5883l/hmc5883l.h"
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>
#include <time.h>
#include <sys/time.h>
//======================================================
#define Pi 3.1416
#define Gyro_Sensitivity 14.375
#define Rad2Deg 57.29578 //1 radian = 57.29578 degrees
//======================================================
int file1, file2, file3, file4;
float xAc, yAc, zAc, xGy_out, yGy_out, zGy_out, suhu_out;
signed int xAcc, yAcc, zAcc, xGyro, yGyro, zGyro, suhu;
float gyroXangle, gyroYangle, gyroZangle, accXAngle, accYAngle, accZAngle;

//==========================================================PID=====================================
float A_error, A_Integral = 0, A_Derivative, A_lasterror = 0, PID_aileron = 0, A_kp = 1.5, A_ki = 0.1, A_kd = 0.0;	
float E_error, E_Integral = 0, E_Derivative, E_lasterror = 0, PID_elevator = 0, E_kp = 1.5, E_ki = 0.1, E_kd = 0.0;
float R_error, R_Integral = 0, R_Derivative, R_lasterror = 0, PID_rudder = 0, R_kp = 1.0, R_ki = 0.0, R_kd = 0.0;

//==========================================================barom
float altitude, pressure1;
//==========================================================kompas==================================
float Xmin = 0.0, Xmax = 0.0, Ymin = 0.0, Ymax = 0.0, offX, offY, offZ, headingDegrees;		
//==========================================================filter===================================
float X_comp_angle = 0.0, Y_comp_angle = 0.0, X_kal_angle = 0.0, Y_kal_angle = 0.0;		
float pwm_roll, pwm_pitch, pwm_yaw;
//=======================================================Kalman=================================
float sudutRoll, sudutPitch;
float DTkal = 0.01;
float Q_angle  =  0.01;
float Q_gyro   =  0.0003;
float R_angle  =  0.01;
float x_bias = 0.0;
float y_bias = 0.0;
float XP_00 = 0.0, XP_01 = 0.0, XP_10 = 0.0, XP_11 = 0.0;
float YP_00 = 0.0, YP_01 = 0.0, YP_10 = 0.0, YP_11 = 0.0;
float KFangleX = 0.0;
float KFangleY = 0.0;
//========================================================Serial============
int b=0;
int serial_port;
int S_i, S_k=0, S_x;
int flag = 0;
char dat, nilai[14];
char _PA[3]={0}, _IA[3]={0}, _DA[3]={0}, _PE[3]={0}, _IE[3]={0}, _DE[3]={0};
//============================================================logging=========
FILE *LOG, *PID_A, *PID_E;
int MODE;
//====================================================time=======
struct timeval tmnow;
struct tm *tm;
char buf[30], usec_buf[6];
void I2CInit()
{
	// Create I2C bus
	char *bus = "/dev/i2c-1";
	if((file1 = open(bus, O_RDWR)) & (file2 = open(bus, O_RDWR)) & (file3 = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
}

void accInit(char Acc_Addr)
{
	ioctl(file1, I2C_SLAVE, Acc_Addr);

	//bandwidth 7.81Hz
	char Aconfig[2] = {0};
	Aconfig[0] = 0x10;
	Aconfig[1] = 0x08;
	write(file1, Aconfig, 2);
}

void gyroInit(char G_Addr)
{
	// Get I2C device
	ioctl(file2, I2C_SLAVE, G_Addr);
	
	// Select Power management register 0x3E(62)
	// 0x01(01)	Power up, PLL with X-Gyro reference
	// Power Up, Set xGyro Reference(0x01)
	char Gconfig[2] = {0};
	Gconfig[0] = 0x3E;
	Gconfig[1] = 0x01;
	write(file2, Gconfig, 2);
	
	//Select DLPF register, 0x16(22)
	// 0x18(24)	Gyro FSR of +/- 2000 dps
	// Set Full scale range of +/- 2000 deg/sec(0x18)
	char Gconfig2[2] = {0};
	Gconfig2[0] = 0x16;
	Gconfig2[1] = 0x18;
	write(file2, Gconfig2, 2);
	
	sleep(1);
}

void baromInit(char B_Addr)
{
	//Get I2C device
	ioctl(file3, I2C_SLAVE, B_Addr);
	
	// Select measurement control register(0xF4)
	// Enable temperature measurement(0x2E)
	char config[2] = {0};
	config[0] = 0xf4;
	config[1] = 0x2e;
	write(file3, config, 2);
	sleep(2);
	
	// Select measurement control register(0xf4)
	// Enable pressure measurement, OSS = 1(0x74)
	// char config2[2] = {0};
	// config2[0] = 0xf4;
	// config2[1] = 0x74;
	// write(file3, config2, 2);
	// sleep(1);
}

//struct KOMPAS!!!
//=============================//
struct{
	float x, y ,z;
} magnetic;

struct{
	float x, y, z;
} data;
//================================//

void kompas()
{
	float Xsf, Ysf, Xoff, Yoff;
	
	float Xvalue, Yvalue;
	// Initialize
	HMC5883L hmc5883l;
	if( hmc5883l_init(&hmc5883l) != HMC5883L_OKAY ) {
      fprintf(stderr, "Error: %d\n", hmc5883l._error);
      exit(1);
	}
	// Read min data && max data
	hmc5883l_read(&hmc5883l);
	if(hmc5883l._data.x > Xmax) Xmax = hmc5883l._data.x;
	if(hmc5883l._data.x < Xmin) Xmin = hmc5883l._data.x;
	if(hmc5883l._data.y > Ymax) Ymax = hmc5883l._data.y;
	if(hmc5883l._data.y < Ymin) Ymin = hmc5883l._data.y;
	//offset
	offX = (Xmax + Xmin) / 2;
	offY = (Ymax + Ymin) / 2;
	
	data.x = hmc5883l._data.x - offX;
	data.y = hmc5883l._data.y - offY;
	
	magnetic.x = data.x / _hmc5883l_Gauss_LSB_XY * HMC5883L_CONST_GAUSS2MTESLA;
	magnetic.y = data.y / _hmc5883l_Gauss_LSB_XY * HMC5883L_CONST_GAUSS2MTESLA;
	magnetic.z = data.z / _hmc5883l_Gauss_LSB_Z * HMC5883L_CONST_GAUSS2MTESLA;
	// Set declination angle on your location and fix heading
	// You can find your declination on: http://magnetic-declination.com/
	// (+) Positive or (-) for negative
	// For Bytom / Poland declination angle is 4'26E (positive)
	// Formula: (deg + (min / 60.0)) / (180 / M_PI);
	float heading = atan2(magnetic.x, magnetic.y);
	float declinationAngle = (0.0 + (37.0 / 60.0)) / (180 / M_PI);
	heading += declinationAngle;
	// Correct for heading < 0deg and heading > 360deg
	if (heading < 0)
	{
		heading += 2 * Pi;
	}
 
	if (heading > 2 * Pi)
	{
		heading -= 2 * Pi;
	}
	headingDegrees = heading * 180 / M_PI; 
	if(headingDegrees>180) headingDegrees = headingDegrees-360;
	
	//printf("X, Y:\t%0.2f | %0.2f | %0.2f | %0.2f | %0.2f | %0.2f | %0.2f\n", hmc5883l._data.x, hmc5883l._data.y, hmc5883l._data.z, offX, offY, heading, headingDegrees);
}

void accelero()
{	
	// Read 6 bytes of data from register(0x02)
	// X msb, X lsb, Y msb, Y lsb, Z msb, Z lsb
	char reg[1] = {0x02};
	write(file1, reg, 1);
	char data[6] = {0};
	if(read(file1, data, 6) != 6)
	{
		printf("Error : Input/output Error \n");
	}
	else
	{
		// Convert the values
		//xAcc = data[0];
		xAcc = ((data[1] << 6) + (data[0] >> 2)); 		//14 bit data
		if (xAcc > 10000) xAcc = (xAcc ^ 16383) * -1; 	//baca negatif, xor dengan 14 bit data 
		xAc = xAcc * 0.25 / 1000.0;						//convert satuan G datasheet
		
		yAcc = ((data[3] << 6) + (data[2] >> 2));
		if (yAcc > 10000) yAcc = (yAcc ^ 16383) * -1;
		yAc = yAcc * 0.25 / 1000.0;
		
		zAcc = ((data[5] << 6) + (data[4] >> 2));
		if (zAcc > 10000) zAcc = (zAcc ^ 16383) * -1;
		zAc = zAcc * 0.25 / 1000.0;
		
		accXAngle = (atan2(yAc, zAc) * 180) / Pi;
		accYAngle = -(atan2(xAc, zAc) * 180) / Pi;
	}
}

void gyroscope()
{
	// Read 6 bytes of data from register(0x1D)
	// X msb, X lsb, Y msb, Y lsb, Z msb, Z lsb
	char reg[1] = {0x1D}; 
	write(file2, reg, 1);
	char data[6] = {0};
	
	if(read(file2,data,6) != 6)
	{
		printf("Error : Input/output Error \n");
	}
	else
	{		
		xGyro = (data[0] << 8) + data[1]; 				//16 bit data MSB + LSB
		if(xGyro > 32767) xGyro -= 65536;	
		xGy_out = (float)xGyro / Gyro_Sensitivity; 		//convert ke degree/second
		
		yGyro = (data[2] << 8) + data[3];
		if(yGyro > 32767) yGyro -= 65536;
		yGy_out = (float)yGyro / Gyro_Sensitivity;
			
		zGyro = (data[4] << 8) + data[5];
		if(zGyro > 32767) zGyro -= 65536;
		zGy_out = (float)zGyro / Gyro_Sensitivity;
	}
}

void barom()
{
	// Calibration Cofficients stored in EEPROM of the device
	// Read 22 bytes of data from address 0xAA(170)
	
	char reg[1] = {0xAA};
	write(file3, reg, 1);
	char data[22] = {0};
	read(file3, data, 22);
	
	// Convert the data
	short AC1 = (data[0] << 8) + data[1];
	short AC2 = (data[2] << 8) + data[3];
	short AC3 = (data[4] << 8) + data[5];
	unsigned short AC4 = (data[6] << 8) + data[7];
	short AC5 = (data[8] << 8) + data[9];
	short AC6 = (data[10] << 8) + data[11];
	short B1 = (data[12] << 8) + data[13];
	short B2 = (data[14] << 8) + data[15];
	short MB = (data[16] << 8) + data[17];
	short MC = (data[18] << 8) + data[19];
	short MD = (data[20] << 8) + data[21];
	
	// //Select measurement control register(0xF4)
	// //Enable temperature measurement(0x2E)
	// char config[2] = {0};
	// config[0] = 0xf4;
	// config[1] = 0x2e;
	// write(file3, config, 2);
	// //Read Temperature
	// char reg1[2] = {0xf6};
	// write(file3, reg1, 1);
	// char data1[2] = {0};
	// if(read(file3, data1, 2) != 2){
		// printf("Error: I/O Error \n");
		// exit(0);
	// }
	// float temp = (data1[0] << 8) + data1[1];
	
	// //temperature calib
	// float X1 = (temp - AC6) * AC5 / 32768.0;
	// float X2 = (MC * 2048.0) / (X1 + MD);
	// float B5 = X1 + X2;
	// float cTemp = ((B5 + 8.0) / 16.0) / 10.0;
	// float fTemp = cTemp * 1.8 + 32.0;
	
	// Select measurement control register(0xf4)
	// Enable pressure measurement, OSS = 1(0x74)
	char config2[2] = {0};
	config2[0] = 0xf4;
	config2[1] = 0x74;
	write(file3, config2, 2);
	
	//Read Pressure
	//read 3 bit from f6, f7, f8
	char reg2[1] = {0xf6};
	write(file3, reg2, 1);
	char data3[3] = {0};
	if(read(file3, data3, 3) != 3)
	{
		printf("ERROR!");
	}
	float UP = ((data3[0] << 16) + (data3[1] << 8) + data3[2]) /128;
	float B5;
	// Calibration for Pressure
	float B6 = B5 - 4000.0;
	float X1 = (B2 * (B6 * B6 / 4096.0)) / 2048.0;
	float X2 = AC2 * B6 / 2048.0;
	float X3 = X1 + X2;
	float B3 = (((AC1 * 4.0 + X3) * 2.0) + 2.0) / 4.0;
	X1 = AC3 * B6 / 8192.0;
	X2 = (B1 * (B6 * B6 / 2048.0)) / 65536.0;
	X3 = ((X1 + X2) + 2.0) / 4.0;
	float B4 = AC4 * (X3 + 32768.0) / 32768.0;
	float B7 = ((UP - B3) * (25000.0));
	float pressure = 0.0;
	if(B7 < 2147483648.0)
	{
		pressure = (B7 * 2) / B4;
	}
	else
	{
		pressure = (B7 / B4) * 2;
	}
	X1 = (pressure / 256.0) * (pressure / 256.0);
	X1 = (X1 * 3038.0) / 65536.0;
	X2 = ((-7357) * pressure) / 65536.0;
	pressure1 = (pressure + (X1 + X2 + 3791.0) / 16.0) / 100.0;

	// Calculate Altitude
	altitude = (44330.0 * (1.0 - pow(pressure1/1013.25, 0.1903)))/100;
		
	//printf("Temperature : %.2f Celcius \t %.2f Fahrenheit \n", cTemp, fTemp);
	//printf("Altitude : %.2f m \t Pressure : %.2f hPa\n", altitude, pressure1);
}

void comp_filter()
{
	float Koef = 0.9;
	
	X_comp_angle = (Koef * (X_comp_angle + xGy_out * 0.03)) + ((1.0 - Koef) * accXAngle);
	Y_comp_angle = (Koef * (Y_comp_angle + yGy_out * 0.03)) + ((1.0 - Koef) * accYAngle);
}

float kalmanY(float AYangle, float GYangle)
{
	float y, S;
	float K_0, K_1;
	
	Y_kal_angle += DTkal * (GYangle - y_bias);
	
	YP_00 += -DTkal * (YP_10 + YP_01) + Q_angle * DTkal;
	YP_01 += -DTkal * YP_11;
	YP_10 += -DTkal * YP_11;
	YP_11 += Q_gyro * DTkal;
	
	y = AYangle - Y_kal_angle;
	S = YP_00 + R_angle;
	K_0 = YP_00 / S;
	K_1 = YP_10 / S;
	
	Y_kal_angle += K_0 * y;
	y_bias  +=  K_1 * y;
	YP_00 -= K_0 * YP_00;
	YP_01 -= K_0 * YP_01;
	YP_10 -= K_1 * YP_00;
	YP_11 -= K_1 * YP_01;
	
	return Y_kal_angle;
}

float kalmanX(float AXangle, float GXangle)
{
	float x, S;
	float K_0, K_1;
	
	X_kal_angle += DTkal * (GXangle - x_bias);
	
	XP_00 += -DTkal * (XP_10 + XP_01) + Q_angle * DTkal;
	XP_01 += -DTkal * XP_11;
	XP_10 += -DTkal * XP_11;
	XP_11 += Q_gyro * DTkal;
	
	x = AXangle - X_kal_angle;
	S = XP_00 + R_angle;
	K_0 = XP_00 / S;
	K_1 = XP_10 / S;
	
	X_kal_angle += K_0 * x;
	x_bias  +=  K_1 * x;
	XP_00 -= K_0 * XP_00;
	XP_01 -= K_0 * XP_01;
	XP_10 -= K_1 * XP_00;
	XP_11 -= K_1 * XP_01;
	
	return X_kal_angle;
}

void kalmanfilter()
{
	sudutRoll = kalmanX(accXAngle, xGy_out);
	sudutPitch = kalmanY(accYAngle, yGy_out);
}

void pwm_init()
{
	wiringPiSetupGpio();
	pinMode (18, PWM_OUTPUT);
	pinMode (6, OUTPUT);
	pinMode (13, PWM_OUTPUT);
   
	pwmSetMode (PWM_MODE_MS);
	pwmSetRange (2000);				//pwmFrequency in Hz = 19.2e6 Hz / pwmClock / pwmRange.
	pwmSetClock (192);				//pwmClock=1920 and pwmRange=200 to get pwmFrequency=50Hz:
	softPwmCreate(6, 0, 200);		//20ms cycle = 200 steps(20 ms as 200 * 100 = 20000 microseconds)
									//The 20 ms cycle is more correct for servos as it will give a nominal update of 50 Hz.
									//A 50 ms cycle will only update at 20 Hz.
}

void aktuator(float aileron, float elevator, float rudder)
{
	pwm_roll = 0.7778 * aileron + 150.0;
	if(pwm_roll < 80) pwm_roll = 80;
	if(pwm_roll > 220) pwm_roll = 220;
	
	pwm_pitch = -(0.7778 * elevator) + 150.0;
	if(pwm_pitch < 80) pwm_pitch = 80;
	if(pwm_pitch > 220) pwm_pitch = 220;
	
	pwm_yaw = 0.7778 * rudder + 150.0;
	if(pwm_yaw < 80) pwm_yaw = 80;
	if(pwm_yaw > 220) pwm_yaw = 220;
	pwm_yaw = pwm_yaw / 10;
		
	pwmWrite(18, pwm_roll);
	pwmWrite(13, pwm_pitch);
	softPwmWrite(6, pwm_yaw);
}

void kontrolPID(float DT)
{
	//roll
	A_error = 0.0 - X_comp_angle;
	A_Integral = A_Integral + (A_error * DT);
	A_Derivative = (A_error - A_lasterror) / DT;
	A_lasterror = A_error;
	PID_aileron = (A_kp * A_error) + (A_ki * A_Integral) + (A_kd * A_Derivative);
	
	//pitch
	E_error = 0.0 - Y_comp_angle;
	E_Integral = E_Integral + (E_error * DT);
	E_Derivative = (E_error - E_lasterror) / DT;
	E_lasterror = E_error;
	PID_elevator = (E_kp * E_error) + (E_ki * E_Integral) + (E_kd * E_Derivative);
	
	//yaw
	R_error = 0.0 - headingDegrees;
	R_Integral = R_Integral + (R_error * DT);
	R_Derivative = (R_error - R_lasterror) / DT;
	R_lasterror = R_error;
	PID_rudder = (R_kp * R_error) + (R_ki * R_Integral) + (R_kd * R_Derivative);
}

void serial_setup()
{
	// struct termios options;
	// tcgetattr(serial_port, &options);
	// cfsetspeed(&options, B9600);
	// options.c_cflag &= ~CSTOPB;
	// options.c_cflag |= CLOCAL;
	// options.c_cflag |= CREAD;
	// cfmakeraw(&options);
	// tcsetattr(serial_port, TCSANOW, &options);
	if ((serial_port = open ("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)	/* open serial port */
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
	}
	struct termios options;
	tcgetattr(serial_port, &options);
	cfsetspeed(&options, B9600);
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= CLOCAL;
	options.c_cflag |= CREAD;
	cfmakeraw(&options);
	tcsetattr(serial_port, TCSANOW, &options);
	
	// struct termios options;
	// tcgetattr(serial_port, &options);
	// options.c_cflag = B9600 | CS8 | CLOCAL | CREAD | ICRNL;
	// options.c_iflag = IGNPAR;
	// options.c_oflag = 0;
	// options.c_lflag = 0;
	// tcflush(serial_port, TCIFLUSH);
	// tcsetattr(serial_port, TCSANOW, &options);

	if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
	}
}

void bacaA_PID()
{
	if(serialDataAvail (serial_port) )
	{ 
		for(S_i=0;S_i<15;S_i++){
			dat = serialGetchar (serial_port);		/* receive character serially*/	
			nilai[S_i] = dat;
				//printf("nilai : %c\n", nilai[S_i]);
		}
		for(S_i=0;S_i<15;S_i++){
			if(nilai[S_i]=='A'){
				continue;
			}
			else {
				if(nilai[S_i]==','){
					flag++;
					//printf("koma %d\n", flag);
				}
				else{
					if(flag==0){
						if(S_k>3) S_k=0;
						_PA[S_k] = nilai[S_i];
						//printf("%c | %c | %d | %d\n", _PA[S_k], nilai[S_i], S_i, S_k);
					}
					if(flag==1){
						if(S_k>3) S_k=0;
						_IA[S_k] = nilai[S_i];
						//printf("%c | %c | %d | %d\n", _IA[S_k], nilai[S_i], S_i, S_k);
					}
					if(flag==2){
						if(S_k>3) S_k=0;
						_DA[S_k] = nilai[S_i];
						//printf("%c | %c | %d | %d\n", _DA[S_k], nilai[S_i], S_i, S_k);
					}
				}
				S_k++;
			}
		}
		A_kp = atof(_PA);
		A_ki = atof(_IA);
		A_kd = atof(_DA);
		//printf("%.2f,%.2f,%.2f\n", A_kp, A_ki, A_kd);
		fprintf(PID_A, "%.2f,%.2f,%.2f\n", A_kp, A_ki, A_kd);
		fclose(PID_A);
	}	
}

void bacaE_PID()
{
	if(serialDataAvail (serial_port) )
	{ 	
		for(S_i=0;S_i<15;S_i++){
			dat = serialGetchar (serial_port);		/* receive character serially*/	
			nilai[S_i] = dat;
			//printf("nilai : %c\n", nilai[S_i]);
		}
	
		//int panjang = strlen(nilai);
		for(S_i=0;S_i<15;S_i++){
			
			if(nilai[S_i]=='E'){
				continue;
			}
			else {
				if(nilai[S_i]==','){
					flag++;
					//printf("koma %d\n", flag);
				}
				else{
					if(flag==0){
						if(S_k>3) S_k=0;
						_PE[S_k] = nilai[S_i];
						//printf("%c | %c | %d | %d\n", _PA[S_k], nilai[S_i], S_i, S_k);
					}
					if(flag==1){
						if(S_k>3) S_k=0;
						_IE[S_k] = nilai[S_i];
						//printf("%c | %c | %d | %d\n", _IA[S_k], nilai[S_i], S_i, S_k);
					}
					if(flag==2){
						if(S_k>3) S_k=0;
						_DE[S_k] = nilai[S_i];
						//printf("%c | %c | %d | %d\n", _DA[S_k], nilai[S_i], S_i, S_k);
					}
				}
				S_k++;
			}			
			//printf("nilai : %c\n", nilai[S_i]);
		}
		E_kp = atof(_PE);
		E_ki = atof(_IE);
		E_kd = atof(_DE);
		fprintf(PID_E, "E%.2f,%.2f,%.2f\n", E_kp, E_ki, E_kd);
		//printf("%f\t%f\t%f\n", E_kp, E_ki, E_kd);
		fclose(PID_E);
	}	
}
int aa;
int SamplingX;
void kirimData()
{
	aa = millis();
	serial_port = open ("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);
	char a[100];
	int j = 0;
	sprintf(a,"%d, %.2f,%.2f,%.2f,%.2f\r\n", b, X_comp_angle, Y_comp_angle, sudutRoll, sudutPitch);
	
	//printf("%d %c %s", panjang, a[1], a);
	//for(j=0;j<strlen(a);j++){
	//	fflush(stdout);
		//serialPutchar(serial_port, a[j]);		//kirim data serial per karakter	
	//}
	//char buf[] = a;
	int count = write(serial_port, a, strlen(a)); 
	b++;
	if(b==100) b=0;
	serial_port = close(serial_port);
	SamplingX = millis() - aa;
}

void sinyal_setup()
{
	wiringPiSetupGpio();
	delay(500);
	pinMode(23, INPUT);
	delay(500);
}
void sinyal()
{
	if(digitalRead(23)==HIGH){
		MODE = 0;
	}
	if(digitalRead(23)==LOW){
		MODE = 1;
	}
	//printf("%d\n", MODE);
}

void time()
{
	gettimeofday(&tmnow, NULL);
    tm = localtime(&tmnow.tv_sec);
    strftime(buf,30,"%d%H:%M:%S", tm);
    strcat(buf,".");
    sprintf(usec_buf,"%d",(int)tmnow.tv_usec);
    strcat(buf,usec_buf);
    //printf("%s",buf);
}

//float p=0, q=0;
const char *logging = buf;
int main() 
{
	time();
	PID_A = fopen("PID_Aileron.txt", "r+");	
	PID_E = fopen("PID_Elevator.txt", "r+");
	LOG = fopen(logging, "w+");
	I2CInit();
	pwm_init();
	baromInit(0x77);
	accInit(0x40);
	gyroInit(0x68);
	//serial_setup();
	sinyal_setup();
	// while(1){
		// sinyal();
	// }
	while(1){
		int a = millis();
		barom();
		accelero();
		gyroscope();
		kompas();
		comp_filter();
		kalmanfilter();
		
		//p++;q++;
		
		//X_comp_angle = p;
		//Y_comp_angle = q;
		
		//kirimData();
		kontrolPID(0.01);
		PID_aileron = PID_aileron - 2.01714;
		//PID_elevator = PID_elevator + 7;
		aktuator(PID_aileron, PID_elevator, PID_rudder);
		//bacaA_PID();
		sinyal();
		//delay(30);
		int Sampling = millis() - a;
		
		printf("%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Sampling, MODE, A_kp, A_ki, A_kd, E_kp, E_ki, E_kd, X_comp_angle, Y_comp_angle, PID_aileron, PID_elevator);
		fprintf(LOG, "%d\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\r\n", MODE, A_kp, A_ki, A_kd, E_kp, E_ki, E_kd, X_comp_angle, Y_comp_angle, sudutRoll, sudutPitch, PID_aileron, PID_elevator);
		//printf("KP=%.2f \tKI=%.2f \tKD=%.2f \taccX=%.2f \taccY=%.2f \tcompX=%.2f \tcompY=%.2f \tkalX=%.2f \tkalY=%.2f \tYaw=%.2f \t\n", A_kp, A_ki, A_kd, accXAngle, accYAngle, X_comp_angle, Y_comp_angle, kalmanX(accXAngle, xGy_out), kalmanY(accYAngle, yGy_out), headingDegrees);	
		;
		
		//printf("%.2f\t%.2f\n", X_comp_angle, Y_comp_angle);
	}
	return 0;
}