#include <stdio.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringSerial.h>

int flag=0;
int serial_port;
float p, i_, d,pp;
void setupserial()
{
	if ((serial_port = serialOpen ("/dev/ttyS0", 9600)) < 0)	/* open serial port */
	{
		fprintf (stderr, "Unable to open s	erial device: %s\n", strerror (errno)) ;
		
	}
	if (wiringPiSetup () == -1)					/* initializes wiringPi setup */
	{
		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
	}
}

void baca()
{
	int S_i,k=0, x;
	
	
	char dat, nilai[S_i];
	char _P[3], _I[3], _D[3];
	while(1){
		if(serialDataAvail (serial_port) )
		{ 	
			for(S_i=0;S_i<14;S_i++){
				dat = serialGetchar (serial_port);		/* receive character serially*/	
				nilai[S_i] = dat;
				printf("nilai : %c\n", nilai[S_i]);
			}
		
			//int panjang = strlen(nilai);
			for(S_i=0;S_i<14;S_i++){
				printf("nilai : %c\n", nilai[S_i]);
				if(nilai[S_i]==','){
					printf("koma\n");
					flag++;
					
				}
				else{
					if(flag==0){
						if(k>3) k=0;
						_P[k] = nilai[S_i];
						printf("%c | %c | %d | %d\n", _P[k], nilai[S_i], S_i, k);
					}
					if(flag==1){
						if(k>3) k=0;
						_I[k] = nilai[S_i];
						printf("%c | %c | %d | %d\n", _I[k], nilai[S_i], S_i, k);
					}
					if(flag==2){
						if(k>3) k=0;
						_D[k] = nilai[S_i];
						printf("%c | %c | %d | %d\n", _D[k], nilai[S_i], S_i, k);
					}
				}
				k++;
			}
			//unsigned char y = "asd";
			p = atof(_P);
			pp = atof(_P);
			i_ = atof(_I);
 			d = atof(_D);
			//printf("%s | %s | %s | %.2f | %.2f | %.2f\n",_P, _I, _D, p, i_, d);
			printf("%f | %f | %f | %f\n", p, pp, i_, d);
		
			for(S_i=0;S_i<14;S_i++){
				fflush(stdout);
				serialPutchar(serial_port, nilai[S_i]);		/* transmit character serially on port */
			}
			
		}
	}
}
int main ()
{	
	setupserial();
	while(1){
		baca();
	}
}