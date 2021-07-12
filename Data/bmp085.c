#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>

void main()
{
	int file;
	char *bus = "/dev/i2c-1";
	if((file = open(bus, O_RDWR)) < 0)
	{
		printf("Failed OPEN bus \n");
		exit(1);
	}
	// Get I2C device, VEML6070 I2C address is 0x77(119)
	ioctl(file, I2C_SLAVE, 0x77);
	
	// Calibration Cofficients stored in EEPROM of the device
	// Read 22 bytes of data from address 0xAA(170)
	// while(1)
	// {
	while(1){
	char reg[1] = {0xAA};
	write(file, reg, 1);
	char data[22] = {0};
	read(file, data, 22);
	
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
	//sleep(1);
	
	// Select measurement control register(0xF4)
	// Enable temperature measurement(0x2E)
	char config[2] = {0};
	config[0] = 0xf4;
	config[1] = 0x2e;
	write(file, config, 2);
	//sleep(1);
	
	char reg1[2] = {0xf6};
	write(file, reg1, 1);
	char data1[2] = {0};
	if(read(file, data1, 2) != 2)
	{
		printf("Error: I/O Error \n");
		exit(0);
	}
	float temp = (data1[0] << 8) + data1[1];
	
	// Select measurement control register(0xf4)
	// Enable pressure measurement, OSS = 1(0x74)
	char config2[2] = {0};
	config2[0] = 0xf4;
	config2[1] = 0x74;
	write(file, config2, 2);
	sleep(1);
	
	//read 3 bit from f6, f7, f8
	char reg2[1] = {0xf6};
	write(file, reg2, 1);
	char data3[3] = {0};
	if(read(file, data3, 3) != 3)
	{
		printf("ERROR!");
	}
	float UP = ((data3[0] << 16) + (data3[1] << 8) + data3[2]) /128;
	
	// printf("%d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \t %d \n", data[0], data[1], AC1, AC4, AC5, AC6, B1, B2, MB, MC, MD, temp, UP);
	// }

	//temperature calib
	float X1 = (temp - AC6) * AC5 / 32768.0;
	float X2 = (MC * 2048.0) / (X1 + MD);
	float B5 = X1 + X2;
	float cTemp = ((B5 + 8.0) / 16.0) / 10.0;
	float fTemp = cTemp * 1.8 + 32.0;
	
	// Calibration for Pressure
	float B6 = B5 - 4000.0;
	X1 = (B2 * (B6 * B6 / 4096.0)) / 2048.0;
	X2 = AC2 * B6 / 2048.0;
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
	float pressure1 = (pressure + (X1 + X2 + 3791.0) / 16.0) / 100.0;

	// Calculate Altitude
	float altitude = 44330.0 * (1.0 - pow(pressure1/1013.25, 0.1903));

	// Output data to screen
	printf("Altitude : %.2f m \t", altitude);
	printf("Pressure : %.2f hPa \t", pressure1);
	printf("Temperature in Celsius : %.2f C \t", cTemp);
	printf("Temperature in Fahrenheit : %.2f F \n", fTemp);
	}
}