
#include "HEADER.h"

unsigned char ch,buff[500];
// return 1 -> SUCCESS
// return 0 -> FAIL

unsigned char USART6_RX_Inchar(void);

int WiFi_Response(void)
{
	int i=0;     
			for(i=0; i< 400; i++) {
		buff[i] = USART6_RX_Inchar();
			if ((buff[i]=='K') && buff[i-1]=='O')
				return 1;  
    }
			return 0;
}


void GPIOC_Initial()
{
RCC_AHB1ENR |= 0X04;	//Enable PORTC clock
while(!(RCC_AHB1ENR & 0x04))
{
	;	//Wait untill PORTC enable.
}
GPIOC_MODER &= 0xffff0fff;	//clear 15-12 bit field in GPIOC_MODER.
GPIOC_MODER |= 0x0000a000;	//load '1010' in 15-12 bit field in GPIOA_MODER.(Set alternate function for USART6- Tx(pc6) & Rx(pc7)).
GPIOC_AFRL &= 0x00ffffff;		//clear AFRL10 & AFRL9 bits
GPIOC_AFRL |= 0x88000000;		//load '1000' in AFRL7 & AFRL6(enabling alternate function for receiver and transmitter in USART)

}

void USART6_Config()
{
RCC_APB2ENR |= (0X1<<5);	//Enable USART6 clock.
while(!(RCC_APB2ENR & 0x20))
	{	
		;	//Wait untill USART6 enable
	}
	USART6_BRR = 0x08a;	//Set Baud rate 115200(Baudrate in decimal = 8.6805)
	USART6_CR1 &= ~(0x200c);	//clear 13,3,2 bits in USART_CR1
	USART6_CR1 |= (0x200c);	//set 13(USART Enable),3(Transmitter enable),2(Reciver enable) bits in USART_CR1
}

unsigned char USART6_RX_Inchar()			//BLCOKING
{
while(!(USART6_SR & 0x20))
{
	;	// wait untill Received data is ready to be read.
}	
	return (USART6_DR & 0xff);	// return data register value. (RXNE)
}

/*
unsigned char USART6_RX_Inchar_NonBlock()		//Non-Blocking function.
{
if(USART6_SR & 0x20)
{
	return (USART6_DR & 0xff);
}	
	return -1;	
}
*/

void USART_TX_Outchar(unsigned char out)
{
while(!(USART6_SR & 0x80))
{
	;
}
	USART6_DR = out;
}


void USART6_TX_Outstring(char *out)
{
	int i;
	for(i=0;out[i];i++)
		USART_TX_Outchar(out[i]);	
}

int WiFi_initial(void)
{
	int ret;
	USART6_TX_Outstring("AT\r\n");
	KM_delayms(100);
//	ret = WiFi_Response();
//		if (ret == 0)
//		{
//		;// FAIL. WIFi module not works
//			return 0;
//		}	
	USART6_TX_Outstring("AT+RST\r\n");
	KM_delayms(100);
	USART6_TX_Outstring("AT+CWMODE=3\r\n");
	KM_delayms(100);
//	ret = WiFi_Response();
//		if (ret == 0)
//		{
//		;// FAIL. WIFi module not works
//			return 0;
//		}	
	USART6_TX_Outstring("AT+CWJAP=\"KM-2.4G\",\"9963111084\"\r\n");
	KM_delayms(100);
	
//		ret = WiFi_Response();
//		if (ret == 0)
//		{
//		;// FAIL. WIFi module not works
//			return 0;
//		}	
	USART6_TX_Outstring("AT+CIFSR\r\n");
	KM_delayms(100);
//		ret = WiFi_Response();
//		if (ret == 0)
//		{
//		;// FAIL. WIFi module not works
//			return 0;
//		}	
	USART6_TX_Outstring("AT+CIPMUX=0\r\n");
		KM_delayms(100);
//		ret = WiFi_Response();
//		if (ret == 0)
//		{
//		;// FAIL. WIFi module not works
//			return 0;
//		}	
	USART6_TX_Outstring("AT+CIPMODE=0\r\n");
		KM_delayms(100);
		
//		ret = WiFi_Response();
//		if (ret == 0)
//		{
//		;// FAIL. WIFi module not works
//			return 0;
//		}	
		
		return 1;
}



int WiFi_main()
{
	//int ret=0;
	//char str[]="WiFi Failed";
	GPIOC_Initial();	//PORTC Initialization.
	STK_Init();		//SysTick Initialization.
	USART6_Config();	//USART6 configuration.
	LCD_Common();
	WiFi_initial();
	
//	ret = WiFi_initial();
//	if (ret == 0)
//	{
//		KM_LCD_Write_Str(str);
//		return -1;
//	}	
	
return 0;
}