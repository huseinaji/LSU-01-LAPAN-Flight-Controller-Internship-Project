#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
int uart0_filestream = -1;
void serial()
{
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if(uart0_filestream == -1)
	{
		printf("ERROR, Can't open UART");
	}
	else{
		printf("serial OK");
	}
	
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL |CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
}

void main()
{
	serial();
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_buffer[0];
	*p_tx_buffer++ = 'H';
	*p_tx_buffer++ = 'e';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'l';
	*p_tx_buffer++ = 'o';
	
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}
	close(uart0_filestream);
	// while(uart0_filestream != -1)
	// {
		// unsigned char rx_buffer[256];
		// int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
		// if(rx_length < 0){
			//error occured
		// }
		// else if(rx_length == 0){
			//no data waiting
		// }
		// else{
			// rx_buffer[rx_length] = '\0';
			// printf("%i bytes read : %s\n", rx_length, rx_buffer);
		// }
	// }
}