
/*
#define RCC_AHB1ENR *((int *)0x40023830)
#define RCC_APB2ENR *((int *)0x40023844)
#define GPIOC_MODER *((int *)0x40020800)
#define STK_CTRL *((int *)0xE000E010)
#define STK_LOAD *((int *)0xE000E014)
#define STK_VAL *((int *)0xE000E018)
#define ADC_SMPR1 *((int *)0x4001200c)
#define ADC_SQR3 *((int *)0x40012034)
#define ADC_CR1 *((int *)0x40012004)
#define ADC_CR2 *((int *)0x40012008)
#define ADC_SR *((int *)0x40012000)
#define ADC_DR *((int *)0x4001204c)
	//Physical addresses for LCD
#define RCC_AHB1ENR *((int *)0x40023830)
#define GPIOB_MODER *((int *)0x40020400) 
#define GPIOB_ODR *((int *)0x40020414)
*/

#include "HEADER.h"	//user define library for all physical addresses and function declarations.

unsigned int TEMP_Value;	//global variable.
char str[20]="Temperature"; //String to display at begining.

void GPIOC_Init()
{
RCC_AHB1ENR |= 0x4;				//reading RCC_AHB1ENR register and setting 1st bit without disturbing others.
while(!(RCC_AHB1ENR & 0x4));  //wait untill 1st bit value 1 or clock is set.
}

void GPIOC_Config()
{
GPIOC_MODER |= 0x00000003;	//set bits 1&0 in GPIOC_MODER (enable PC0 in Analog direction).
}


void KM_ADC_Init(void)
{
	RCC_APB2ENR |= (0x1<<8);	//enabling ADC clock.
	while(!(RCC_APB2ENR & 0x1<<8));	//wait untill ADC clock enable.
	ADC_SMPR1 |= 0x07;	//Smapling SMP10 highest cycles. (for Temp.Sensor)
	
	//ADC_SMPR1 = 0x38;	//Smapling SMP11 highest cycles.	(for Potentiometer)
	
	ADC_SQR3 |= 0x0a;	//load '1010' in 0-4 positions to set ADC_10 as 1st conversion.
	ADC_CR1 &= ~(0x3<<24);	//load '00'in 24-25 positions to set 12th bit resolution.
	ADC_CR2 |= 0x1<<10;	//set 10th bit (EOC selection).
	ADC_CR2 |= 0x1;	//set 0th bit (ADON to turn ON ADC)
}

void KM_ADC_Read(void)
{
	int i;
	ADC_CR2 |= (0x1<<30);	//Start Conversion (SWSTRT enabling).
	while(!(ADC_SR & 0x2))
	{	
	;
	}
	//if conversion complete EOC bit will set.
	TEMP_Value = ADC_DR;	//Collect data from ADC_DR.
	TEMP_Value /= 10;	//Temperature in C = Vout/10ma
	KM_delayms(1);
	KM_LCD_Write_Cmd(0x14>>12);
	KM_LCD_Write_Data((TEMP_Value/10)+48);	
	KM_LCD_Write_Data((TEMP_Value%10)+48);
	KM_LCD_Write_Data('C');
	for(i=0;i<3;i++)
	KM_LCD_Write_Cmd(0x10);
	
	//KM_LCD_Write_Cmd(0x10);

	
	ADC_CR2 &= ~(0x1<<30);	//Clear Trigger.
	KM_delayms(500);

}

void ADC_main()
{
	GPIOC_Init();	//Initializing PORTC.
	GPIOC_Config();	//PORTC configuration.
	PORTB_initial();	//PORTB Initialization.
	PORTB_config();	//PORTB configuration.
	KM_ADC_Init();	//Initializing ADC.
	KM_LCD_Write_Str(str);
	//KM_LCD_Write_Cmd(0x14>>12);
			
}