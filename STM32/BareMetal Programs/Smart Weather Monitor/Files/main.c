
#include "HEADER.h"
#include <stdio.h>

extern unsigned int TEMP_Value;
int main()
{
	int i,hum=0;
	char fetch[100]="GET /page.php?temp=00&hum=0&dev=2\r\n\r\n";
	STK_Init();	//Systick Timer Initalization.
	LCD_Common();	//LCD Initialization and configurtion.
	ADC_main();	//ADC Initialization, Configuration and Temp value print on LCD.
	WiFi_main();	//Wifi Initialization.
		
	while(1)
	{
		//WiFi_initial();
		KM_ADC_Read();	//read ADC o/p from Potentiometer.
		fetch[19]=(TEMP_Value/10)+48;
		fetch[20]=(TEMP_Value%10)+48;
		
		USART6_TX_Outstring("AT+CIPSTATUS\r\n");
		KM_delayms(1000);
		USART6_TX_Outstring("AT+CIPSTART=\"TCP\",\"142.93.218.33\",80\r\n");
		KM_delayms(1000);
		USART6_TX_Outstring("AT+CIPSEND=41\r\n");
		KM_delayms(1000);
		USART6_TX_Outstring(fetch);
		for(i=0;i<5;i++)
		KM_delayms(1000);
		//sprintf(fetch,"GET /page.php?temp=%u&hum=%d&dev=2\r\n\r\n", (TEMP_Value),hum);
	}
}