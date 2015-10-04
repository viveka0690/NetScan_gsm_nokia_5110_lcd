/*************** uart.c ****************/
//Pin for Button PIN_NUMBER =29 [physical pin] / WiringPi = 21[gpio]

#include<stdio.h>
#include<unistd.h>
#include<termios.h>
#include<string.h>
#include<fcntl.h>
#include<stdlib.h>

//-----------------Modify : Nokia File integration ----------

#include<bcm2835.h>
#include "lcd_pcd8544.h"
#include<stdlib.h>
#include "time.h"
#include "typedefs.h"
//----------------------------------------------------------

/********************** button  ********************/
#define BT1 RPI_GPIO_P1_16
#define BT2 RPI_GPIO_P1_18
#define BT3 RPI_GPIO_P1_22

/*************************************************************/
void TransmitToUart(unsigned char *UatTempBuff);
unsigned char * ReceiveFromUart();
void print_thermal_usb();
void command_send();

void bcm_button_init();

int uart0_filestream = -1;
int num_bytes;
char SaveData[3500];
int z =0;

int main()
{
	if(!bcm2835_init())
		return 1;
	printf("Nokia 5110 Screen\n");
	Nokia_5110_Init();
	bcm_button_init();

	Nokia_5110_Set_Cursor(0, 0);
	Nokia_5110_Putstring("FACTUM: Welcom");
	Nokia_5110_Set_Cursor(1,2);
	Nokia_5110_Putstring_Inverse("NET SCAN"); 
	delay(1000);	
	printf("BREAK\n");
	
	while(1)
	{
		
		if(bcm2835_gpio_eds(BT1))
		{
			delay(500);
		//	bcm2835_gpio_set_eds(BT1);
			printf("Button 1 pressed #\n");
			printf("Netwrok Scaning ... \n");
			Nokia_5110_Clear_All();
			Nokia_5110_Set_Cursor(1,1);
			Nokia_5110_Putstring("NET SCAN pressed");
			SaveData[3500]= 0x00;
			z =0;
			num_bytes = 0;
			command_send();
			
			if(num_bytes >2000)
			{
				Nokia_5110_Clear_All();
				printf("Press PRINT button\n");
				Nokia_5110_Set_Cursor(1,2);	
				Nokia_5110_Putstring("Press PRINT button");
			//	printf("%s",SaveData);
			//	printf("Total number of Bytes : %d\n",num_bytes);
			//	num_bytes = 0;
			//	SaveData[3500]=0x00;	
			//	z=0;	
			}
			else
			{
				Nokia_5110_Clear_All();
				Nokia_5110_Set_Cursor(1,2);
				Nokia_5110_Putstring("Not Enough Data:Press Again");
				printf("Not enough Data: Press SCAN Network again\n");
			//	num_bytes = 0;
			//	SaveData[3500] = 0x00;
			//	z =0;	
			}
			delay(200);
			bcm2835_gpio_set_eds(BT1);	
		}
		else if(bcm2835_gpio_eds(BT2))
		{
			delay(500);
			bcm2835_gpio_set_eds(BT2);	
			printf("Button 2 pressed +\n");
			printf("Printing DATA...\n");
			print_thermal_usb();
			Nokia_5110_Clear_All();
			Nokia_5110_Set_Cursor(1,2);
			Nokia_5110_Putstring("Printing Data...");
			delay(500);
			Nokia_5110_Clear_All();
			Nokia_5110_Set_Cursor(0,1);
			Nokia_5110_Putstring(SaveData);
		}	
		else if(bcm2835_gpio_eds(BT3))
		{
			bcm2835_gpio_set_eds(BT3);
			printf("Button 3 pressed  Exiciting from Program& \n");
			Nokia_5110_Clear_All();
			Nokia_5110_Set_Cursor(1,2);
			Nokia_5110_Putstring("Exiting Program");
			exit(0);	
		}
		
		delay(200);	
	
	}
	Nokia_5110_Stop_SPI();
	
	return 0;

}

void command_send(void)
{	
          printf("Command Send\n");
//Setting Up The UART

	//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY );		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR ;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	
	TransmitToUart("AT+CNETSCAN\r");  // AT command for Network Scan

	if(O_NDELAY!=2048)
	printf("hey:%d\n",O_NDELAY);
	//close(uart0_filestream);
	usleep(10000);

	while(1){
	ReceiveFromUart();
		
	if(z == 1)
	{	
		close(uart0_filestream);
		return;
	}
	}
		
}

void print_thermal_usb( void)
{
printf("Thermal Print\n");
//Setting Up The UART

	//-------------------------
	//----- SETUP USART 0 -----
	//-------------------------
	//At bootup, pins 8 and 10 are already set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
	
	
	//OPEN THE UART
	//The flags (defined in fcntl.h):
	//	Access modes (use 1 of these):
	//		O_RDONLY - Open for reading only.
	//		O_RDWR - Open for reading and writing.
	//		O_WRONLY - Open for writing only.
	//
	//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
	//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
	//											immediately with a failure status if the output can't be written immediately.
	//
	//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	uart0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY );		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
	
	//CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
//	PARODD - Odd parity (else even)
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR ;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	
	//TransmitToUart(SaveData);
	TransmitToUart(SaveData);

/*	if(O_NDELAY!=2048)
	printf("hey:%d\n",O_NDELAY);
	close(uart0_filestream);
	usleep(10000);
	while(1){
	ReceiveFromUart();
	printf("Saved Data= %s\n", SaveData);
//	close(uart0_filestream);
	} */



} 

void TransmitToUart(unsigned char *UatTempBuff)
{
//Transmitting Bytes

	//----- TX BYTES -----
	unsigned char tx_buffer[20];
	unsigned char *p_tx_buffer;
	int size;
        size=strlen(UatTempBuff);
	p_tx_buffer = &tx_buffer[0];

        //*p_tx_buffer++ = 'r';

	
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, UatTempBuff, (size));		//Filestream, bytes to write, number of bytes to write
	//	printf("count = %d\n",count);
		if (count < 0)
		{
			printf("UART TX error\n");
		}
	}

}

unsigned char * ReceiveFromUart()
{
//	SaveData[3500]=0x00;
//	num_bytes =0;
//Receiving Bytes
//Because O_NDELAY has been used this will exit if there are no receive bytes waiting (non blocking read), so if you want to hold waiting for input simply put this in a while loop
//
//----- CHECK FOR ANY RX BYTES -----
//uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY );
	
	int i,j;
	char L2[3];
	if (uart0_filestream != -1)
	{
		// Read up to 255 characters from the port if they are there
		unsigned char rx_buffer[256];
		int rx_length = read(uart0_filestream, (void*)rx_buffer, 8);		//Filestream, buffer to store in, number of bytes to read (max)
		//printf("rx_length = %d\n",rx_length);
	
		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
		}
		else if (rx_length == 0)
		{
			//No data waiting
		}
		else
		{
			//Bytes received
			rx_buffer[rx_length] = '\0';
			//printf("%i bytes read : %s\n", rx_length, rx_buffer);
			//strcpy((&SaveData+num_bytes),rx_buffer);
			for(i=0;i<rx_length;i++)
			{
				j=i+num_bytes;
				if(rx_buffer[i]!=0x00)
				{
				SaveData[num_bytes+i] = rx_buffer[i];
			//	Nokia_5110_Putchar(rx_buffer[i],NOKIA_5110_PRINT_NORMAL);
				//delay(100);
				}


			}
			num_bytes+=rx_length;
			
		//	printf(" Rx_Buf :%s\n",  rx_buffer);
			
//			//rx_buffer[256] = 0;	
		SaveData[num_bytes+1]= '\0';
	//	
		i=0;
		while(i !=2)
		{
			L2[i]= SaveData[num_bytes -4 +i];
			
		i++;	
		}
		L2[i] = '\0';
		//printf("Last 2: %s\n",L2);
		if(strcmp(L2, "OK")== 0){
			
			printf("Saved Data= %s\n", SaveData);
			printf("Total num of Bytes: %d\n", num_bytes);
//Nokia 5110 used here
			z =1;
			Nokia_5110_Set_Cursor(1,2);
			Nokia_5110_Clear_All();	
			Nokia_5110_Set_Cursor(2,2);
			Nokia_5110_Putstring_Inverse("THANK YOU ! ");
	
		}
                  }
		
	}
	else
	{
	printf("File Open Error\n");
	}
	
	return 0;	
}

void bcm_button_init()
{

	bcm2835_gpio_fsel(BT1, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(BT2, BCM2835_GPIO_FSEL_INPT);
	bcm2835_gpio_fsel(BT3, BCM2835_GPIO_FSEL_INPT);
	
	bcm2835_gpio_set_pud(BT1, BCM2835_GPIO_PUD_UP);
	bcm2835_gpio_set_pud(BT2, BCM2835_GPIO_PUD_UP);
	bcm2835_gpio_set_pud(BT3, BCM2835_GPIO_PUD_UP);

	bcm2835_gpio_len(BT1);
	bcm2835_gpio_len(BT2);
	bcm2835_gpio_len(BT3);

}
