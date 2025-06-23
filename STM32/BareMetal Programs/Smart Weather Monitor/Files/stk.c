
#include "HEADER.h"

void STK_Init()
{
STK_CTRL |= 0x05;	//
STK_VAL = 0x123;
}

void KM_delayms(int x)
{
	
STK_LOAD = x*16000;
while((STK_CTRL & (0X1<<16)) == 0);
	
}

//micro seconds delay.
void KM_delayus(int x)
{
	STK_LOAD = x*16;
	while((STK_CTRL & (0X1<<16)) == 0);
}