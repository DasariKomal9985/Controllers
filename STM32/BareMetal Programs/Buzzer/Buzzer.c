





#define RCC_AHB1ENR *((int *)0x40023830)
#define GPIOB_MODER *((int *)0x40020400) 
#define GPIOB_ODR *((int *)0x40020414)
#define GPIOC_MODER *((int *)0x40020800)
#define GPIOC_PUPDR *((int *)0x4002080c)
#define GPIOC_IDR *((int *)0x40020810)
	

void initial(void)
{
RCC_AHB1ENR |= 0x2;				//reading RCC_AHB1ENR register and setting 1st bit without disturbing others.
while(!(RCC_AHB1ENR & 0x2));  //wait untill 1st bit value 1 or clock is set.

RCC_AHB1ENR |= 0x4;				//reading RCC_AHB1ENR register and setting 2nd bit without disturbing others.
while(!(RCC_AHB1ENR & 0x4));  //wait untill 2nd bit value 1 or clock is set.
}	


void config(void)
{
GPIOB_MODER &= 0xfcffffff;		//clearing 25 and 24 bit positions
GPIOB_MODER |= 0x01000000;		//loading 01 into 25 and 24 bits to set the mode as output
	

GPIOC_MODER &= 0xffcfffff;		//clearing 21 and 20 bitfield to enable portC
GPIOC_PUPDR &= 0xffcfffff;		//clearing 21 and 20 bitfield in GPIOC_PUPDR
GPIOC_PUPDR |= 0x00100000;		//set 21 and 20 bitfied to enable switch_enter in i/p direction.

}	

void delay(void)
{
int i=125000;
	for(;i>0;i--);
}

int main()
{
	int i;
	initial();
	config();
		
	while(1)
	{
		if(!(GPIOC_IDR & (0x1<<10)))	
		{
			
			for(i=0;i<5;i++)
			{
			GPIOB_ODR &= ~(0x1<<12);	//clearing 12th bit to ON buzzer.
			//delay();
			}
		}
		
			
	}
return 0;
}