#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "LCD16x2Lib/LCD.h"
#include "MY_Keypad4x4.h"
#include <stdbool.h>
// ============================================

volatile char lcd[32];
volatile char lcd0[16];
volatile char lcd1[16];

char* waveForm[6] = {
	"Sin",
	"Square",
	"Triangle",
	"Absolute Sin",
	"Step",
	"SawTooth",
};

char* waveFormShort[6] = {
	"sin",
	"sq",
	"tri",
	"abs",
	"st",
	"saw",
};


char* selectedWaveForm="Sin";
unsigned int selectedWaveFormIndex=0 ,selectedTime=0,selectedFreq=0;
int programState=0;

float oldResult=0;
void UARTSend(char* str, unsigned int length) {

	for (unsigned int i=0 ; i < length ; i++)
	{
		while ((USART2->SR & (1<<7)) == 0);
		USART2->DR=str[i];
	}
}
// ============================================
int main(void)
{
	SystemCoreClockUpdate();
	HAL_Init();
	
	// CLK
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	
	// Pin Config
		// Input
	GPIO_InitTypeDef PinConfig;	
	PinConfig.Pin = GPIO_PIN_0;
	PinConfig.Mode = GPIO_MODE_INPUT;
	PinConfig.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	PinConfig.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	PinConfig.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	PinConfig.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	
		// output
	PinConfig.Pin = GPIO_PIN_4;
	PinConfig.Mode = GPIO_MODE_OUTPUT_PP;
	PinConfig.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	PinConfig.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	PinConfig.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	PinConfig.Pin = GPIO_PIN_7;
	HAL_GPIO_Init(GPIOA, &PinConfig);
	
		// analog
	RCC->APB2ENR |= (1<<8);//Enable the ADC clock 
	GPIOA->MODER |= 3<<2;
	ADC1->SQR3 =1;
	ADC1->SQR1 =0;
	ADC1->CR2 |= (1<<0);
	
		// UART
	GPIOA->MODER |= (2<<4);
	GPIOA->MODER |= (2<<6);
	GPIOA->AFR[0] |= (7<<8);
	GPIOA->AFR[0] |= (7<<12);
	
	RCC->APB1ENR |= (1<<17);
	
	USART2->CR1 |= (1<<13);
	USART2->BRR = 0x683;
	USART2->CR1 |= (1<<3);
	USART2->CR1 |= (1<<2);
	
	char str_tx[50] = "Test\r\n	";
	//while (1) {
	//UARTSend(str_tx,strlen(str_tx));
	//	HAL_Delay(4000);
	//}	
	LCD_Init();
	LCD_Puts(0, 0, "98242128");
	HAL_Delay(1000);
	
	
	Keypad_WiresTypeDef   myKeypadStruct;
	myKeypadStruct.IN0_Port = GPIOA;
	myKeypadStruct.IN1_Port = GPIOA;
	myKeypadStruct.IN2_Port = GPIOA;
	myKeypadStruct.IN3_Port = GPIOA;
	
	myKeypadStruct.OUT0_Port = GPIOA;
	myKeypadStruct.OUT1_Port = GPIOA;
	myKeypadStruct.OUT2_Port = GPIOA;
	myKeypadStruct.OUT3_Port = GPIOA;
	
	
	//rows => inputs
	myKeypadStruct.IN0pin = GPIO_PIN_0;
	myKeypadStruct.IN1pin = GPIO_PIN_14;
	myKeypadStruct.IN2pin = GPIO_PIN_15;
	myKeypadStruct.IN3pin = GPIO_PIN_10;
	
	//columns => outputs
	myKeypadStruct.OUT0pin = GPIO_PIN_4;
	myKeypadStruct.OUT1pin = GPIO_PIN_5;
	myKeypadStruct.OUT2pin = GPIO_PIN_6;
	myKeypadStruct.OUT3pin = GPIO_PIN_7;
	
	
	Keypad4x4_Init(&myKeypadStruct);
	bool myKeys[16];
	
	
	sprintf(lcd0
		,"1 %s 2 %s 3 %s", 
		waveFormShort[0],waveFormShort[1],waveFormShort[2]);
	sprintf(lcd1
		,"4 %s 5 %s 6 %s",
		waveFormShort[3],waveFormShort[4],waveFormShort[5]);
		
	LCD_Puts(0, 0, lcd0);
	LCD_Puts(0, 1, lcd1);
	//LCD_Puts(0, 0, "1 sin 2 sq 3 tri");
	//LCD_Puts(0, 1, "4 abs 5 st 6 saw");
	
	while (programState==0) {
		
		/*
		sprintf(lcd0
		,"%d%d%d%d %d%d%d%d %d%d%d%d %d%d%d%d"
		,myKeys[0],myKeys[1],myKeys[2],myKeys[3]
		,myKeys[4],myKeys[5],myKeys[6],myKeys[7]
		,myKeys[8],myKeys[9],myKeys[10],myKeys[11]
		,myKeys[12],myKeys[13],myKeys[14],myKeys[15]);
		LCD_Puts(0, 0, lcd0);
		*/
		//HAL_Delay(2000);
		
		Keypad4x4_ReadKeypad(myKeys);
		for (uint8_t i=0 ; i < 16 ; i++)
			if (myKeys[i])
				if (Keypad4x4_GetInt(i) >= 1 && Keypad4x4_GetInt(i) <= 6)
				{
					LCD_Clear();
					selectedWaveFormIndex=Keypad4x4_GetInt(i)-1;
					selectedWaveForm=waveForm[Keypad4x4_GetInt(i)-1];
					sprintf(lcd,"Selected wave form: %s",selectedWaveForm);
					LCD_Puts(0, 0, lcd);
					programState++;
					HAL_Delay(500);
				}
	}
	
	
	LCD_Clear();
	LCD_Puts(0, 0, "Select Time:");
	
	while (programState==1) {
		ADC1->CR2 |= (1<<30);
		while ((ADC1->SR & (1<<1)) == 0) {}
		unsigned int ADC_Result = ADC1->DR & 0x0FFF;
			// T  = aV+b
			// V: max value = 4091, min=4
			// T: max = 10,000, min=500
		float m = ((10000.0-500.0)/(4091.0-4.0));
		float b = 500.0-4.0*m;
		float Result=	(ADC_Result*m)+b;
		char str[20];
		sprintf(str,"%d ms",(int)Result);
		if ((int)oldResult != (int)Result)
			LCD_Puts(0,1,"                ");
		LCD_Puts(0,1,str);
		oldResult=Result;
		//HAL_Delay(250);
		Keypad4x4_ReadKeypad(myKeys);
		
		for (uint8_t i=0 ; i < 16 ; i++)
			if (myKeys[i])
				if (Keypad4x4_GetInt(i) >=10)
				{
					LCD_Clear();
					selectedTime=(int)Result;
					sprintf(lcd1,"%d ms",selectedTime);
					LCD_Puts(0, 0, "Selected Time:");
					LCD_Puts(0, 1, lcd1);
					programState++;
					HAL_Delay(500);
				}
	}
	LCD_Clear();
	LCD_Puts(0, 0, "Select Freq:");
	while (programState==2) {
		ADC1->CR2 |= (1<<30);
		while ((ADC1->SR & (1<<1)) == 0) {}
		unsigned int ADC_Result = ADC1->DR & 0x0FFF;
			// F  = aV+b
			// V: max value = 4091, min=4
			// F: max = 1000, min=1
		float m = ((1000.0-1.0)/(4091.0-4.0));
		float b = 1-4*m;
		float Result=	(ADC_Result*m)+b;
		char str[20];
		//sprintf(str,"%d Hz",ADC_Result);
			sprintf(str,"%d Hz",(int)Result);
		if ((int)oldResult != (int)Result)
			LCD_Puts(0,1,"                ");
		LCD_Puts(0,1,str);
		oldResult=Result;
		//HAL_Delay(250);
		Keypad4x4_ReadKeypad(myKeys);
		
		for (uint8_t i=0 ; i < 16 ; i++)
			if (myKeys[i])
				if (Keypad4x4_GetInt(i) >=10)
				{
					LCD_Clear();
					selectedFreq=(int)Result;
					sprintf(lcd1,"%d Hz",selectedFreq);
					LCD_Puts(0, 0, "Selected Freq:");
					LCD_Puts(0, 1, lcd1);
					programState++;
					HAL_Delay(500);
				}
	}
	
	LCD_Clear();
	sprintf(lcd0,"W:%s",selectedWaveForm);
	sprintf(lcd1,"T:%dms,F:%dHz",selectedTime,selectedFreq);
	LCD_Puts(0, 0, lcd0);
	LCD_Puts(0,1,lcd1);
	
	if (programState==3) {
		
		sprintf(str_tx,"%d %d %d\r\n",selectedWaveFormIndex,selectedTime,selectedFreq);
		UARTSend(str_tx,strlen(str_tx));
	}
	while (programState==3) {
	}
	return 0;

}
// ============================================
void SysTick_Handler(void)
{
  HAL_IncTick();
}
// ============================================