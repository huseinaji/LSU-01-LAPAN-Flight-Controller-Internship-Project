import RPi.GPIO as GPIO # always needed with RPi.GPIO  
from time import sleep
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
GPIO.setup(12, GPIO.OUT)

pwm = GPIO.PWM(12, 50)
pwm.start(12)
sleep(1)
pwm.stop()
# try:
	# while True:
		
		
		# GPIO.output(12, 1)
		# sleep(0.0015)
		# GPIO.output(12, 0)
		
	# sleep(1)

# except KeyboardInterrupt:
	
	# GPIO.cleanup()