
#define RCC_AHB1ENR *((int *)0x40023830)
#define GPIOB_MODER *((int *)0x40020400) 
#define GPIOB_ODR *((int *)0x40020414)
#define STK_CTRL *((int *)0xE000E010)
#define STK_LOAD *((int *)0xE000E014)
#define STK_VAL *((int *)0xE000E018)

void initial(void)
{
	//PORT B (LED)
RCC_AHB1ENR |= 0X2;											/*Bit 1 GPIOBEN: IO port B clock enable
																					Set and cleared by software.
																					0: IO port B clock disabled
																					1: IO port B clock enabled*/
while(!(RCC_AHB1ENR & 0x2));  

	//STK INITALISATION STKTIMER
	
STK_CTRL |= 0x05;	                  //Bit 0 ENABLE: Counter enable                      [1]
																					/*Selects the clock source.
																					0: AHB/8
																					1: Processor clock (AHB)*/
																		//Bit 1 TICKINT: SysTick exception request enable		[0] - [SYS INTERRUPT]
																						/*0: Counting down to zero does not assert the SysTick exception request
																						1: Counting down to zero to asserts the SysTick exception request.
																						Note: Software can use COUNTFLAG to determine if SysTick has ever counted to zero*/
																		//Bit 2 CLKSOURCE: Clock source selection						[1]
																						/*Enables the counter. When ENABLE is set to 1, the counter loads the RELOAD value from the 
																							LOAD register and then counts down. On reaching 0, it sets the COUNTFLAG to 1 and 
																							optionally asserts the SysTick depending on the value of TICKINT. It then loads the RELOAD 
																							value again, and begins counting.
																							0: Counter disabled
																							1: Counter enabled
																									*/
																				//101=5(0X05)
																				
STK_VAL = 0x123;										/*Bits 23:0 CURRENT: Current counter value
																			The VAL register contains the current value of the SysTick counter.
																			Reads return the current value of the SysTick counter.
																			A write of any value clears the field to 0, and also clears the COUNTFLAG bit in the 
																			STK_CTRL register to 0.   [JUST RANDAM VALUE]
																					*/
}	

void config(void)
{
	//Red LED
GPIOB_MODER &= 0xf3ffffff;						//CLEAR MODER14[1:0] MODER13[1:0] FOR RED AND GREEN LED( 29 28 27 26)
	/*These bits are written by software to configure the I/O direction mode.
																			00: Input (reset state)
																			01: General purpose output mode
																			10: Alternate function mode
																			11: Analog mode*/
GPIOB_MODER |= 0x04000000;						//SET MODER14[1:0] MODER13[1:0] FOR RED AND GREEN LED( 29 28 27 26)
}	

void delayms(int x)
{
	
STK_LOAD = x*16000;
while((STK_CTRL & (0X1<<16)) == 0);		//WHEN EVER [0] IT ALWAYS GO TO LOOP AND WHEN EVER [1] THEN STOPS GOING INTO LOOP
	
}

//MAX DELAY (1.04)
void delayus(int x)
{
	STK_LOAD = x*16;
	while((STK_CTRL & (0X1<<16)) == 0);
}

//MIN DELAY (0.0625)
void delaymin()
{
	STK_LOAD = 0x1;
	while((STK_CTRL & (0X1<<16)) == 0);
}

//maximum delay possible
void delaymax()
{
	STK_LOAD = 0xffffffff;
	while((STK_CTRL & (0X1<<16)) == 0);
}

int main()
{
	initial();
	config();

	while(1)
	{
	GPIOB_ODR &= ~ (0x1<<13);	//Red LED ON
	//delayms(100);
	delaymin();
	//delaymax();
		
	GPIOB_ODR |= (0x1<<13);		//Red LED OFF
	//delayms(100);
	delaymin();
	//delaymax();
	}
	
	
return 0;
}