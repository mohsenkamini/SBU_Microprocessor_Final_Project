#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "LCD16x2Lib/LCD.h"
#include <stdbool.h>
#include <math.h>


#define TABLE_BITS  (5)
#define TABLE_SIZE  (1<<TABLE_BITS)
#define TABLE_MASK  (TABLE_SIZE-1)
 
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

typedef struct

{
	GPIO_TypeDef* OUT0_Port;	
	GPIO_TypeDef* OUT1_Port;
	GPIO_TypeDef* OUT2_Port;
	GPIO_TypeDef* OUT3_Port;
	GPIO_TypeDef* OUT4_Port;	
	GPIO_TypeDef* OUT5_Port;
	GPIO_TypeDef* OUT6_Port;
	GPIO_TypeDef* OUT7_Port;
	GPIO_TypeDef* OUT8_Port;	
	GPIO_TypeDef* OUT9_Port;
	GPIO_TypeDef* OUT10_Port;
	GPIO_TypeDef* OUT11_Port;
	GPIO_TypeDef* OUT12_Port;	
	GPIO_TypeDef* OUT13_Port;
	GPIO_TypeDef* OUT14_Port;
	GPIO_TypeDef* OUT15_Port;

	
	uint16_t	OUT0pin;
	uint16_t	OUT1pin;
	uint16_t	OUT2pin;
	uint16_t	OUT3pin;
	uint16_t	OUT4pin;
	uint16_t	OUT5pin;
	uint16_t	OUT6pin;
	uint16_t	OUT7pin;
	uint16_t	OUT8pin;
	uint16_t	OUT9pin;
	uint16_t	OUT10pin;
	uint16_t	OUT11pin;
	uint16_t	OUT12pin;
	uint16_t	OUT13pin;
	uint16_t	OUT14pin;
	uint16_t	OUT15pin;

}DACOutPins;


char* selectedWaveForm="Sin";
unsigned int selectedWaveFormIndex=0 ,selectedTime=0,selectedFreq=0;
int programState=1;

void UARTSend(char* str, unsigned int length) {

	for (unsigned int i=0 ; i < length ; i++)
	{
		while ((USART2->SR & (1<<7)) == 0);
		USART2->DR=str[i];
	}
}
void UARTReceive(char* str, unsigned int length) {
	
	//LCD_Puts(0,0,"ii");
	//UARTSend(str,strlen(str));
	str[0]=(USART2->SR & (1<<1)==0)+'0';
	str[1]=(USART2->SR & (1<<2)==0)+'0';
	str[2]=(USART2->SR & (1<<3)==0)+'0';
	str[3]=(USART2->SR & (1<<5)==0)+'0';
	//str_rx[1]=USART2->DR;
	str[4]=NULL;
	LCD_Puts(0,0,str);
	//HAL_Delay(1000);
	//LCD_Clear();
	
	//	str[0]=USART2->DR;
	//str[1]=NULL;
	for (unsigned int i=0 ; i < length ; i++)
	{
		//LCD_Puts(0,1,"nn");	
		//HAL_Delay(1000);
		//while ((USART2->SR & (1<<5)) == 0) {

		//LCD_Puts(0,1,"mm");		
		//}
		//LCD_Puts(0,1,"nn");	
		//HAL_Delay(1000);
		str[i]=USART2->DR;
		
	//LCD_Puts(0,1,"mm");
	//HAL_Delay(100);
	//LCD_Clear();
		if (str[i] == NULL )
		{
			//programState=1;
			break;
		}
	}
	//USART2->SR &= ~(1<<5);
	
	//str[length]=NULL;
}
void binOnOutput16Bit(int n,DACOutPins   DACPins) {
	int i=15;
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT15_Port, DACPins.OUT15pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT15_Port, DACPins.OUT15pin, GPIO_PIN_RESET); 
	i--;
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT14_Port, DACPins.OUT14pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT14_Port, DACPins.OUT14pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT13_Port, DACPins.OUT13pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT13_Port, DACPins.OUT13pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT12_Port, DACPins.OUT12pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT12_Port, DACPins.OUT12pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT11_Port, DACPins.OUT11pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT11_Port, DACPins.OUT11pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT10_Port, DACPins.OUT10pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT10_Port, DACPins.OUT10pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT9_Port, DACPins.OUT9pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT9_Port, DACPins.OUT9pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT8_Port, DACPins.OUT8pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT8_Port, DACPins.OUT8pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT7_Port, DACPins.OUT7pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT7_Port, DACPins.OUT7pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT6_Port, DACPins.OUT6pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT6_Port, DACPins.OUT6pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT5_Port, DACPins.OUT5pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT5_Port, DACPins.OUT5pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT4_Port, DACPins.OUT4pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT4_Port, DACPins.OUT4pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT3_Port, DACPins.OUT3pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT3_Port, DACPins.OUT3pin, GPIO_PIN_RESET); 
	i--;
	
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT2_Port, DACPins.OUT2pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT2_Port, DACPins.OUT2pin, GPIO_PIN_RESET); 
	i--;
	
	
	if ((n%(int)pow(2,i+1))-pow(2,i)>0)
		HAL_GPIO_WritePin(DACPins.OUT1_Port, DACPins.OUT1pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT1_Port, DACPins.OUT1pin, GPIO_PIN_RESET); 
	i--;
	
	if ((n%(int)pow(2,i+1))==1)
		HAL_GPIO_WritePin(DACPins.OUT0_Port, DACPins.OUT0pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(DACPins.OUT0_Port, DACPins.OUT0pin, GPIO_PIN_RESET); 
	i--;
	
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
	
	LCD_Init();
	// Pin Config
		// Input
	GPIO_InitTypeDef PinConfig;
	PinConfig.Mode = GPIO_MODE_OUTPUT_PP;
	PinConfig.Pull = GPIO_NOPULL;	
	PinConfig.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOB, &PinConfig);
	PinConfig.Pin = GPIO_PIN_13;
	HAL_GPIO_Init(GPIOB, &PinConfig);
	PinConfig.Pin = GPIO_PIN_14;
	HAL_GPIO_Init(GPIOB, &PinConfig);
	PinConfig.Pin = GPIO_PIN_15;
	HAL_GPIO_Init(GPIOB, &PinConfig);
	PinConfig.Pin = GPIO_PIN_0;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_1;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_5;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_6;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_9;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_10;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	PinConfig.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOC, &PinConfig);
	
		// set DACOutPins
	DACOutPins   DACPins;
	DACPins.OUT0_Port = GPIOB;
	DACPins.OUT1_Port = GPIOB;
	DACPins.OUT2_Port = GPIOB;
	DACPins.OUT3_Port = GPIOB;
	DACPins.OUT4_Port = GPIOC;
	DACPins.OUT5_Port = GPIOC;
	DACPins.OUT6_Port = GPIOC;
	DACPins.OUT7_Port = GPIOC;
	DACPins.OUT8_Port = GPIOC;
	DACPins.OUT9_Port = GPIOC;
	DACPins.OUT10_Port = GPIOC;
	DACPins.OUT11_Port = GPIOC;
	DACPins.OUT12_Port = GPIOC;
	DACPins.OUT13_Port = GPIOC;
	DACPins.OUT14_Port = GPIOC;
	DACPins.OUT15_Port = GPIOC;
	
	DACPins.OUT0pin = GPIO_PIN_12;
	DACPins.OUT1pin = GPIO_PIN_13;
	DACPins.OUT2pin = GPIO_PIN_14;
	DACPins.OUT3pin = GPIO_PIN_15;	
	DACPins.OUT4pin = GPIO_PIN_0;
	DACPins.OUT5pin = GPIO_PIN_1;
	DACPins.OUT6pin = GPIO_PIN_2;
	DACPins.OUT7pin = GPIO_PIN_3;	
	DACPins.OUT8pin = GPIO_PIN_4;
	DACPins.OUT9pin = GPIO_PIN_5;
	DACPins.OUT10pin = GPIO_PIN_6;
	DACPins.OUT11pin = GPIO_PIN_8;	
	DACPins.OUT12pin = GPIO_PIN_9;
	DACPins.OUT13pin = GPIO_PIN_10;
	DACPins.OUT14pin = GPIO_PIN_11;
	DACPins.OUT15pin = GPIO_PIN_12;
	
	
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
	
	
	volatile char str_rx[50] = "";
	str_rx[0]='*';
	char str_tx[50] = "TEST";	
	str_rx[0]=(USART2->SR & (1<<1))+'0';
	str_rx[1]=(USART2->SR & (1<<2))+'0';
	str_rx[2]=(USART2->SR & (1<<3))+'0';
	str_rx[3]=(USART2->SR & (1<<4))+'0';
	str_rx[4]=(USART2->SR & (1<<5))+'0';
	str_rx[5]=(USART2->SR & (1<<7));
	//str_rx[1]=USART2->DR;
	str_rx[6]=NULL;
	LCD_Puts(0,0,str_rx);
	//while (1) {		//writes to VTerminal just fine.
	//UARTSend(str_tx,strlen(str_tx));
	//}	
	//HAL_Delay(1000);
	
	USART2->SR &= ~(1<<5);	// clear RXNE
	static int sin90[TABLE_SIZE+1] = {
  0x0000,0x0647,0x0c8b,0x12c7,0x18f8,0x1f19,0x2527,0x2b1e,
  0x30fb,0x36b9,0x3c56,0x41cd,0x471c,0x4c3f,0x5133,0x55f4,
  0x5a81,0x5ed6,0x62f1,0x66ce,0x6a6c,0x6dc9,0x70e1,0x73b5,
  0x7640,0x7883,0x7a7c,0x7c29,0x7d89,0x7e9c,0x7f61,0x7fd7,
  0x7fff
};
	//unsigned int sinLookup[90];
	// lookup tables:
	// (0,0),(48,2pi)
	// Y = 48/2pi*i
	/*double j=0.0;
	for (int i=0 ; i < 90 ; i++,j+=1.0 )
	{
		//double result=((pow(2,16)*sin((j/89.0)*(2*3.14)))+(0.5*pow(2,16)));
		//sinLookup[i]=(unsigned int)result;
		
		sprintf(lcd,"sin %d: %d",i,sinLookup[i]);
		LCD_Puts(0,1,lcd);
		//HAL_Delay(100);
		//LCD_Clear();
	}*/
	
	while (programState==0) {
	
			UARTReceive(str_rx,strlen(str_rx));
			LCD_Puts(0,0,str_rx);
			//HAL_Delay(5000);
			//programState++;
		
	}
	
	
	while (programState==1) {
	/*
		binOnOutput16Bit(1,DACPins);
		HAL_Delay(100);
		binOnOutput16Bit((int)pow(2,16)-1,DACPins);
		HAL_Delay(100);
		*/
	for (int i=0 ; i < TABLE_SIZE+1 ; i++ )
	{
		binOnOutput16Bit(sin90[i]+sin90[i],DACPins);
		HAL_Delay(10);
	}
	
	for (i=TABLE_SIZE ; i >= 0 ; i-- )
	{
		binOnOutput16Bit(sin90[i]+sin90[i],DACPins);
		HAL_Delay(10);
	}
	for (i=0 ; i < TABLE_SIZE+1 ; i++ )
	{
		binOnOutput16Bit(sin90[i],DACPins);
		HAL_Delay(10);
	}
	for (i=TABLE_SIZE ; i >=0 ; i-- )
	{
		binOnOutput16Bit(sin90[i],DACPins);
		HAL_Delay(10);
	}
	
	}
	HAL_Delay(10000);
	return 0;

}
// ============================================
void SysTick_Handler(void)
{
  HAL_IncTick();
}
// ============================================