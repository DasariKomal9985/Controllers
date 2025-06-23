#define RCC_AHB1ENR *(int*)0x40023830
#define GPIOB_MODER *(int*)0x40020400
#define GPIOB_ODR *(int*)0x40020414
	
//INTERNAL CLOCK ADDERS
#define STK_CTRL *(int*)0xE000E010
#define STK_LOAD *(int*)0xE000E014
#define STK_VAL *(int*)0xE000E018

//unsigned char temp=0x41;

void inphase(void)
{
	RCC_AHB1ENR|=0x2;
	while(!(RCC_AHB1ENR&0x2)){
	;
	}							
}
void conphase(void)
{
	GPIOB_MODER&=0xfffcf000;
	GPIOB_MODER|=0x00010555;
}



void km_delay(int x)
{
	STK_CTRL|=0X05;
	STK_VAL=0X111;
	STK_LOAD =x*16000;
	while(((STK_CTRL)&0x01<<16)==0);
}


	void WRITE_HIGH_Nibble(unsigned char temp)
	{
		
		GPIOB_ODR &=~(0x0f);				
		GPIOB_ODR |=temp>>4;					
		GPIOB_ODR |=(0x01<<8);					
		km_delay(10);										
		GPIOB_ODR &=~(0x01<<8);					
		
	}
	void WRITE_LOW_Nibble(unsigned char temp)
	{
		GPIOB_ODR &=~(0x0f);
		temp=temp<<4;
		temp=temp>>4;
		GPIOB_ODR |=temp;
		GPIOB_ODR |=(0x01<<8);
		km_delay(10);
		GPIOB_ODR &=~(0x01<<8);
	}


void KM_LCD_WRITE_DATA(unsigned char temp)
{
	GPIOB_ODR|=0x1<<4;    
	WRITE_HIGH_Nibble(temp);
  WRITE_LOW_Nibble(temp);
}	
void KM_LCD_WRITE_STRING(char str[])
{
	int i;
	for(i=0;str[i];i++)
	{
	KM_LCD_WRITE_DATA(str[i]);
	}
}
	
void KM_LCD_WRITE_Cmd(unsigned char temp)
{		
		GPIOB_ODR&=~(0X1<<4);
		WRITE_HIGH_Nibble(temp);
	  WRITE_LOW_Nibble(temp);
}
void KM_LCD_INIT()
{
	km_delay(10);				
	KM_LCD_WRITE_Cmd(0x33);		
	km_delay(10);
	KM_LCD_WRITE_Cmd(0x32);
	KM_LCD_WRITE_Cmd(0x0C);
	KM_LCD_WRITE_Cmd(0x01);
}

int main()
{
	inphase();					
	conphase();					
	KM_LCD_INIT();			
	KM_LCD_WRITE_STRING("komal:");	
}