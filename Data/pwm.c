#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>

void main ()
{
	int pwm = 60, pwm1, pwm2, pwm3;
	pwm1 = pwm + 80;
	pwm2 = pwm + 80;
	pwm3 = (pwm + 80) / 10;
	printf ("Raspberry Pi wiringPi test program\n");
	wiringPiSetupGpio();
	
	pinMode (18, PWM_OUTPUT);
	pinMode (4, OUTPUT);
	pinMode (13, PWM_OUTPUT);
   
	pwmSetMode (PWM_MODE_MS);
	pwmSetRange (2000);				//pwmFrequency in Hz = 19.2e6 Hz / pwmClock / pwmRange.
	pwmSetClock (192);				//pwmClock=1920 and pwmRange=200 to get pwmFrequency=50Hz:
	softPwmCreate(4, 0, 200);		//20ms cycle = 200 steps(20 ms as 200 * 100 = 20000 microseconds)
									//The 20 ms cycle is more correct for servos as it will give a nominal update of 50 Hz.
									//A 50 ms cycle will only update at 20 Hz.
	while(1)
	{
		pwm1 = pwm + 80;
		pwm2 = pwm + 80;
		pwm3 = (pwm + 80) / 10;
		// for(pwm = -50; pwm<=200;pwm++)
		// {		
			// pwm1 = pwm + 80;
			// pwm2 = pwm + 80;
			// pwm3 = (pwm + 80) / 10;
			
			// pwmWrite(18, pwm1);
			// pwmWrite(13, pwm2);
			// softPwmWrite(4, pwm3);
			// delay(10);
		// }
		// for(pwm = 200; pwm > -50; pwm--)
		// {
			// pwm1 = pwm + 80;
			// pwm2 = pwm + 80;
			// pwm3 = (pwm + 80) / 10;
			
			// pwmWrite(18, pwm1);
			// pwmWrite(13, pwm2);
			// softPwmWrite(4, pwm3);
			// delay(10);
		// }
		pwmWrite(18, 150);
		pwmWrite(13, 150);
		softPwmWrite(4, 150);
	}
}