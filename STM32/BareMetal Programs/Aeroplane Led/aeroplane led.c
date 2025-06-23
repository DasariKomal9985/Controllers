
//Toggle RED LED ON time is 50ms and RED LED OFF time is 1300ms





#define RCC_AHB1ENR *((int *) 0x40023830)  //0x40023800+0x30
#define GPIOB_ODR *((int *)0x40020414)     //0x40020400+0x14
#define GPIOB_MODER *((int *) 0x40020400)  //0x40020400+0x00

void delay(void)
{
int i=125000;		  	       	// delay loop for 100s
for(;i>0;i--);
}
void INITIALIZE(void)
{
RCC_AHB1ENR |=0X2;			// set port B in AHB1_ENR 
	while(!(RCC_AHB1ENR & 0X2));	// checking port B set or not
}
void CONFIGURATION(void)
{
GPIOB_MODER &=0XF3FFFFFF;		 // Clear 27,26 bits
GPIOB_MODER |=0X04000000; 		 // set 27,26 bits
GPIOB_MODER &=0XCFFFFFFF; 	// clear 28,29 bits
GPIOB_MODER |=0X10000000; 		 // set 28,29 bits
}
int main()
{
	int i;
	INITIALIZE();
	CONFIGURATION();
	while(1)
	{
	  GPIOB_ODR &=~(0X1<<13);	// RED LED is On
		delay();		//100ms
		GPIOB_ODR |=(0X1<<13);	// RED LED is OFF
		for(i=0;i<=26;i++)
		delay();
		GPIOB_ODR &=~(0X1<<14);	// green LED is On
		delay();		//100ms
		GPIOB_ODR |=(0X1<<14);	// green LED is OFF
		for(i=0;i<=26;i++)
		delay();
	}
}
