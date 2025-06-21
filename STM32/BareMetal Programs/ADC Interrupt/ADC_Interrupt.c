
#define RCC_AHB1ENR *((int *)0x40023830)
#define RCC_APB2ENR *((int *)0x40023844)
#define GPIOC_MODER *((int *)0x40020800)
#define STK_CTRL *((int *)0xE000E010)//cortexM4
#define STK_LOAD *((int *)0xE000E014)
#define STK_VAL *((int *)0xE000E018)
#define ADC_SMPR1 *((int *)0x4001200c)
#define ADC_SQR3 *((int *)0x40012034)
#define ADC_CR1 *((int *)0x40012004)
#define ADC_CR2 *((int *)0x40012008)
#define ADC_SR *((int *)0x40012000)
#define ADC_DR *((int *)0x4001204c)
#define NVIC_ISER0 *((int *)0xE000E100)//cortexM4

int POT_Value;	//global variable.

void GPIOC_Init()
{
RCC_AHB1ENR |= 0x4;				               /*Bit 2 GPIOCEN: IO port C clock enable
																					Set and cleared by software.
																					0: IO port C clock disabled
																					1: IO port C clock enabled
																					*/
while(!(RCC_AHB1ENR & 0x4)); 
}

	//STK INITLISATION

void STK_Init()
{
STK_CTRL = 0x05;	/*Bit 2 CLKSOURCE: Clock source selection
										Selects the clock source.
										0: AHB/8
										1: Processor clock (AHB)
										Bit 1 TICKINT: SysTick exception request enable
										0: Counting down to zero does not assert the SysTick exception request
										1: Counting down to zero to asserts the SysTick exception request.
										Note: Software can use COUNTFLAG to determine if SysTick has ever counted to zero.
										Bit 0 ENABLE: Counter enable
										Enables the counter. When ENABLE is set to 1, the counter loads the RELOAD value from the 
										LOAD register and then counts down. On reaching 0, it sets the COUNTFLAG to 1 and 
										optionally asserts the SysTick depending on the value of TICKINT. It then loads the RELOAD 
										value again, and begins counting.
										0: Counter disabled
										1: Counter enable*/
STK_VAL = 0x123;	/*Bits 23:0 CURRENT: Current counter value                                     //this could be any value
										The VAL register contains the current value of the SysTick counter.
										Reads return the current value of the SysTick counter.
										A write of any value clears the field to 0, and also clears the COUNTFLAG bit in the 
										STK_CTRL register to 0*/
}

//CONFIGURATION

void GPIOC_Config()
{
GPIOC_MODER &= 0xfffffff3;	//FOR CLEAR ADC1_IN11  MODER0[1:0]
GPIOC_MODER |= 0x0000000c;	//FOR SET	ADC1_IN11  MODER0[1:0](enable PC1 in Analog direction).
}



//DELAY
void KM_delayms(int x)
{
STK_LOAD = x*16000;
while((STK_CTRL & (0X1<<16)) == 0);
} 

void KM_ADC_Init(void)
{
	RCC_APB2ENR |= (0x1<<8);	          /*Bit 8 ADC1EN: ADC1 clock enable
																				Set and cleared by software.
																				0: ADC1 clock disabled
																				1: ADC1 clock disabled*/
	
	while(!(RCC_APB2ENR & 0x1<<8));	
	
	ADC_SMPR1 |= 0x07;									/*These bits are written by software to select the sampling time individually for each channel. 
																				During sampling cycles, the channel selection bits must remain unchanged*/
																			//111: 480 cycles SMP10[2:0]
	
	ADC_SMPR1 = 0x38;									//Smapling SMP11 highest cycles.
	
	ADC_SQR3 |= 0x0b;										//Bits 14:10 SQ3[4:0]: 3rd conversion in regular sequence 
																			//load '1011' in 0-4 positions to set ADC_11 as 1st conversion.
	
	ADC_CR1 &= ~(0x3<<24);							/*Bits 25:24 RES[1:0]: Resolution
																				These bits are written by software to select the resolution of the conversion.
																-->     00: 12-bit (15 ADCCLK cycles)
																				01: 10-bit (13 ADCCLK cycles)
																				10: 8-bit (11 ADCCLK cycles)
																				11: 6-bit (9 ADCCLK cycles)*/	
																			//LOAD 24-25 "00"  FOR 12 TH BIT RESOLUTION
																			
	ADC_CR1 |= 0X1<<5;									/*Bit 5 EOCIE: Interrupt enable for EOC
																				This bit is set and cleared by software to enable/disable the end of conversion interrupt.
																				0: EOC interrupt disabled
																				1: EOC interrupt enabled. An interrupt is generated when the EOC bit is set*/
																				
	ADC_CR2 |= 0x1<<10;									/*Bit 10 EOCS: End of conversion selection
																				This bit is set and cleared by software.
																				0: The EOC bit is set at the end of each sequence of regular conversions. Overrun detection 
																				is enabled only if DMA=1.
																				1: The EOC bit is set at the end of each regular conversion. Overrun detection is enabled*/
																				
	ADC_CR2 |= 0x1;											/*Bit 0 ADON: A/D Converter ON / OFF
																				This bit is set and cleared by software.
																				Note: 0: Disable ADC conversion and go to power down mode
																				1: Enable ADC*/
																				
	NVIC_ISER0 |= (0x1<<18);						//18 25 settable ADC ADC1 global interrupts
																			//enabling NVIC_ISER0 18th bit (Enabling ADC Interrupt in NVIC).
																			// 18 25 settable ADC ADC1 global interrupts 0x0000 0088  pg:204(ref_man)
}

void KM_ADC_Read(void)
{
	ADC_CR2 |= (0x1<<30);								/*Bit 30 SWSTART: Start conversion of regular channels
																				This bit is set by software to start conversion and cleared by hardware as soon as the 
																				conversion starts.
																				0: Reset state
																				1: Starts conversion of regular channels
																				Note: This bit can be set only when ADON = 1 otherwise no conversion is launched*/
}
	
void ADC_IRQHandler(void)
{
	POT_Value = ADC_DR;									//Collect data from ADC_DR.
	ADC_CR2 &= ~(0x1<<30);							//Clear Trigger.
	KM_delayms(500);
}

int main()
{
	GPIOC_Init();	
	GPIOC_Config();
	STK_Init();
	KM_ADC_Init();	
		while(1)
		{
		KM_ADC_Read();										//read ADC o/p from Potentiometer.
		}

	
}
