#include <stdio.h>

#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <math.h>
#include "hmc5883l/hmc5883l.h"
#include <wiringPi.h>
#include "hmc5883l/hmc5883l.h"

#define PI 3.1416

struct{
	float x, y ,z;
} magnetic;

struct{
	float x, y, z;
} data;

void main() {
	float minX = 0.0, minY = 0.0, minZ = 0.0;
	float maxX = 0.0, maxY = 0.0, maxZ = 0.0;
	float offX, offY, offZ;
	
	HMC5883L hmc5883l;
	float heading;
	// Initialize
	if( hmc5883l_init(&hmc5883l) != HMC5883L_OKAY ) {
		fprintf(stderr, "Error: %d\n", hmc5883l._error);
		exit(1);
	}
  
  // Read
while(1)
{
	hmc5883l_read(&hmc5883l);
	
	// Determine Min / Max values
	if (hmc5883l._data.x > maxX) maxX = hmc5883l._data.x;
	if (hmc5883l._data.y > maxY) maxY = hmc5883l._data.y;
	if (hmc5883l._data.z > maxZ) maxZ = hmc5883l._data.z;
	if (hmc5883l._data.x < minX) minX = hmc5883l._data.x;
	if (hmc5883l._data.y < minY) minY = hmc5883l._data.y;
	//if (hmc5883l._data.z < minZ) minZ = hmc5883l._data.z;
	
	offX = (maxX + minX) / 2;
	offY = (maxY + minY) / 2;
	
	data.x = hmc5883l._data.x - offX;
	data.y = hmc5883l._data.y - offY;
	
	magnetic.x = data.x / _hmc5883l_Gauss_LSB_XY * HMC5883L_CONST_GAUSS2MTESLA;
	magnetic.y = data.y / _hmc5883l_Gauss_LSB_XY * HMC5883L_CONST_GAUSS2MTESLA;
	magnetic.z = data.z / _hmc5883l_Gauss_LSB_Z * HMC5883L_CONST_GAUSS2MTESLA;
	
	heading = atan2(magnetic.x, magnetic.y);
	float declinationAngle = (0.0 + (36.0 / 60.0)) / (180 / PI);
	heading += declinationAngle;
	
	// Correct for heading < 0deg and heading > 360deg
	if (heading < 0)
	{
		heading += 2 * PI;
	}
 
	if (heading > 2 * PI)
	{
		heading -= 2 * PI;
	}
	float headingDegrees = heading * 180/PI; 
	// Print results
	
	printf("X, Y:\t%0.2f | %0.2f | %0.2f | %0.2f | %0.2f | %0.2f | %0.2f | %0.2f\n", minX, minY, minZ, maxX, maxY, maxZ, heading, headingDegrees);
}
}