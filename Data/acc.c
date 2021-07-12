#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

int file;
float xAc, yAc, zAc;
int xAcc, yAcc, zAcc;

void I2Cinit()
{
	// Create I2C bus
	char *bus = "/dev/i2c-1";
	if((file = open(bus, O_RDWR)) < 0) 
	{
		printf("Failed to open the bus. \n");
		exit(1);
	}
}

void accinit(addr)
{
	// Get I2C device, BMA180 I2C address is 0x68(104)
	ioctl(file, I2C_SLAVE, addr);

	char config[2] = {0};
	config[0] = 0x10;
	config[1] = 0xB6;
	write(file, config, 2);

	// POWER
	char config2[2] = {0};
	config2[0] = 0x0D;
	config2[1] = 0x10;
	write(file, config2, 2);
	
	//bandwidth 150Hz
	char config3[2] = {0};
	config3[0] = 0x20;
	config3[1] = 0x64;
	write(file, config3, 2);
	
	
	//range -+2g
	char config4[2] = {0};
	config4[0] = 0x35;
	config4[1] = 0x02;
	write(file, config4, 2);
	sleep(1);
}

void accelero()
{
	// Read 6 bytes of data from register(0x1D)
	// X msb, X lsb, Y msb, Y lsb, Z msb, Z lsb
	char reg[1] = {0x02};
	write(file, reg, 1);
	char data[6] = {0};
	if(read(file, data, 6) != 6)
	{
		printf("Error : Input/output Error \n");
	}
	
	// Convert the values
	int xAcc = ((data[1] & 63) * 256) + data[0]; //14 bit data
	xAc = (xAcc - 8192.0) / 16384.0;
	int yAcc = ((data[3] & 63) * 256) + data[2];
	yAc = (yAcc - 8192.0) / 16384.0;
	int zAcc = ((data[5] & 63) * 256) + data[4];
	zAc = (zAcc - 8192.0) / 16384.0;
	
	// Output data to screen
	printf("X-Axis : %0.2f \t Y-Axis : %0.2f \t Z-Axis : %0.2f \n", xAc, yAc, zAc);
}
void main() 
{
	I2Cinit();
	accinit(0x40);
	while(1)
	{
		accelero();
	}
}