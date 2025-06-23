

#define RCC_AHB1ENR *((int *)0x40023830)//baseaddress + offsetaddress (40023800 + 0x30 = 40023830)
#define GPIOB_MODER *((int *)0x40020400) 
#define GPIOB_ODR *((int *)0x40020414)
void initial(void)
{
RCC_AHB1ENR |= 0X2;				//reading RCC_AHB1ENR register and setting 1st bit without disturbing others.
while(!(RCC_AHB1ENR & 0x2));  //wait untill 1st bit value 1 or clock is set.
}	

void config(void)
{
GPIOB_MODER &= 0xf3ffffff;		//clearing 27 and 26 bit positions
GPIOB_MODER |= 0x04000000;		//loading 01 into 27 and 26 bits to set the mode as output
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
	while(1)
	{
	GPIOB_ODR &= ~(0x1<<13);	
		delay();
	
	GPIOB_ODR |= (0x1<<13);	
		delay();
		
	}
return 0;
}