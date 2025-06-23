
#include "HEADER.h"

void PORTB_initial(void)
{
	//PORTB initialization.
RCC_AHB1ENR |= 0x2;				//reading RCC_AHB1ENR register and setting 1st bit without disturbing others.
while(!(RCC_AHB1ENR & 0x2));  //wait untill 1st bit value 1 or clock is set.
	//STK Initialization.
STK_CTRL |= 0x05;	
STK_VAL = 0x123;
}

//PORTB Configuration.
void PORTB_config(void)
{
GPIOB_MODER &= 0xfffcf000;
GPIOB_MODER |= 0x00010555;	//PORTB configuration for PB0,PB1,PB2,PB3,PB4,PB5,PB8 set to Output mode.
GPIOB_ODR &= 0xfffffeff;	//PB8 clear (EN=0)
GPIOB_ODR &= 0xffffffef;	//PB4 cleared (RS=0) or enabling command register.
GPIOB_ODR &= 0xffffffdf;	//PB5 cleared (RW=0)
}



//high nibble function.
void write_high_nibble( unsigned char temp)
{
	GPIOB_ODR &= 0xfffffff0;	//clear first 4 bits of GPIOB_ODR.
	GPIOB_ODR |= (temp>>4);
	GPIOB_ODR |= 0x00000100;	//PB8 set (EN=1);
	KM_delayms(10);
	GPIOB_ODR &= 0xfffffeff;	//PB8 clear (EN=0)
	
}


void write_low_nibble( unsigned char temp)
{
	GPIOB_ODR &= 0xfffffff0;	//clear first 4 bits of GPIOB_ODR.
	
	GPIOB_ODR |= (temp&0x0f);	//also write like this for lower 
		
	//GPIOB_ODR |= ((temp<<4)>>4); //even logic is correct not getting correct result
	GPIOB_ODR |= 0x00000100;	//PB8 set (EN=1);
	KM_delayms(10);
	GPIOB_ODR &= 0xfffffeff;	//PB8 clear (EN=0)
}


void KM_LCD_Write_Cmd( unsigned char temp)
{
	GPIOB_ODR &= 0xffffffef;	//PB4 cleared (RS=0) or enabling command register.
	write_high_nibble(temp);
	write_low_nibble(temp);
	
}


void KM_LCD_Write_Data( unsigned char t)
{
	GPIOB_ODR |= 0x00000010;	//PB4 set (RS=1) or enabling data register.
	
	write_high_nibble(t);
	
	write_low_nibble(t);
		
	GPIOB_ODR &= 0xffffffef;	//PB4 cleared (RS=0) or enabling command register.
}
void KM_LCD_Write_Str(char *st)
{
int i=0;
for(i=0;st[i];i++)
KM_LCD_Write_Data(st[i]);
}

//LCD Initialization.
void KM_LCD_Init(void)
{
	KM_delayms(20);
	KM_LCD_Write_Cmd(0x33);
	KM_delayms(1);
	KM_LCD_Write_Cmd (0x32);
	KM_LCD_Write_Cmd (0x0C);
	KM_LCD_Write_Cmd (0x01);
}
                                                                             
void LCD_Common()
{
	PORTB_initial();
	PORTB_config();
	KM_LCD_Init();
	
}