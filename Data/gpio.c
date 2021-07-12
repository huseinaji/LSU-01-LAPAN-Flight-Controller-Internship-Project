#include <wiringPi.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>


int test;
int main()
{
	wiringPiSetup();
	delay(500);
	pinMode(4, INPUT);
	//pinMode(15, OUTPUT);
	delay(500);
	while(1){
		if(digitalRead(4)==HIGH){
			test=1;
		}
		if(digitalRead(4)==LOW){
			test=0;
			//digitalWrite(15, HIGH);
		}
		printf("%d\n", test);
		delay(50);
	}
	return 0;
}