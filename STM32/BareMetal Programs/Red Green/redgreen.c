

#define RCC_AHB1ENR *((int *)0x40023830)
#define GPIOB_MODER *((int *)0x40020400) 
#define GPIOB_ODR *((int *)0x40020414)
void initial(void)
{
RCC_AHB1ENR |= 0X2;				//reading RCC_AHB1ENR register and setting 1st bit without disturbing others.
while(!(RCC_AHB1ENR & 0x2));  //wait untill 1st bit value 1 or clock is set.
}	

void config(void)
{
GPIOB_MODER &= 0xc3ffffff;		//clearing 29 to 26 bit positions
GPIOB_MODER |= 0x14000000;		//loading '0101' into 29 to 26 bits to set the mode of pbas output
}	

void delay(void)
{
int i=125000;
	for(;i>0;i--);
}

int main()
{
	int j;
	initial();
	config();
	
	
	//parellal blinking
	while(1)
	{
	GPIOB_ODR &= ~(0x1<<14);	//clearing 14th bit to ON green LED.	
			delay();
	GPIOB_ODR &= ~(0x1<<13);	//clearing 13th bit to ON red LED.
		delay();
	GPIOB_ODR |= (0x1<<14);		//set 14th bit to OFF green LED.
			delay();
	GPIOB_ODR |= (0x1<<13);		//set 13th bit to OFF red LED.
		delay();
	}
	
	/*
	//serial blinking
	while(1)
	{
	GPIOB_ODR &= ~(0x1<<14);	//clearing 14th bit to ON green LED.	
			delay();
	GPIOB_ODR |= (0x1<<14);		//set 14th bit to OFF green LED.
			delay();
	GPIOB_ODR &= ~(0x1<<14);	//clearing 14th bit to ON green LED.	
			delay();	
	GPIOB_ODR |= (0x1<<14);		//set 14th bit to OFF green LED.
		for(j=0;j<3;j++)	
		delay();	
			
	GPIOB_ODR &= ~(0x1<<13);	//clearing 13th bit to ON red LED.
		delay();
	GPIOB_ODR |= (0x1<<13);		//set 13th bit to OFF red LED.
		for(j=0;j<5;j++)
		delay();
	GPIOB_ODR &= ~(0x1<<13);	//clearing 13th bit to ON red LED.
		delay();	
	GPIOB_ODR |= (0x1<<13);		//set 13th bit to OFF red LED.
		for(j=0;j<9;j++)
		delay();	
		
		for(j=0;j<5;j++)
		delay();
	
	GPIOB_ODR &= ~(0x1<<14);	//clearing 14th bit to ON green LED.	
			delay();
	GPIOB_ODR &= ~(0x1<<13);	//clearing 13th bit to ON red LED.
		delay();	
		
	GPIOB_ODR |= (0x1<<14);		//set 14th bit to OFF green LED.
		for(j=0;j<3;j++)	
		delay();
	GPIOB_ODR |= (0x1<<13);		//set 13th bit to OFF red LED.
		for(j=0;j<9;j++)
		delay();	
	}
	*/
	
return 0;
}