/*
 * systemmaglev.c
 *
 *  Created on: May 14, 2013
 *      Author: DVD
 */
#include "misc.c"
#include "misc.h"
#include "stm32f4xx_adc.c"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dac.c"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_tim.c"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_usart.c"
#include "stm32f4xx_usart.h"

#define MAX_ADC_DAC 4095
#define MAX_VALUE_PWM 65535
#define STM32F4_DELAY_SEC 11538462

extern u8* Dec2Str(s32 Num);
extern void Delayms(u32 ms);

void InitScopePins(void)
{
	// Scope PG1 PG3
	GPIO_InitTypeDef  GPIO_InitStructure;
	// GPIOG Peripheral clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	// Configure PG6 and PG8 in output push or pull mode
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

}

void InitLed(void)
{	//LED PC13
	GPIO_InitTypeDef  GPIO_InitStructure;
	// GPIOG Peripheral clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// Configure PG6 and PG8 in output push or pull mode
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIOC->BSRRL = GPIO_Pin_13;
}

void SetLed(u8 state)
{
	if(state){	GPIOC->BSRRH = GPIO_Pin_13; }
	else	 {	GPIOC->BSRRL = GPIO_Pin_13; }
}

void InitButton(void)
{	//Button PA0
	GPIO_InitTypeDef  GPIO_InitStructure;
	// GPIOA Peripheral clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	// Configure PG6 and PG8 in output push or pull mode
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
u8 GetButton(void)
{
	if(GPIOA->IDR & 1){ return 0xff; } else { return 0; }
}

void WaitForButton(void)
{
	while(!GetButton());
}

void InitADC(void)
{
	//Structure for ADC configuration
	 ADC_InitTypeDef ADC_Init_structure;
	 ADC_CommonInitTypeDef ADC_CommonInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	 GPIO_InitTypeDef GPIO_InitStructre;

	 //ADC structure configuration
	 ADC_DeInit();

	 //Clock configuration
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

	 //Clock for the ADC port!!
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	 //Analog pin configuration
	 //The ADC3 channel 4 is connected to PF6 Photo sensor
	 //The ADC1 channel 6 is connected to PA6 External Trim Reference
	 GPIO_InitStructre.GPIO_Pin = GPIO_Pin_6;
	 //The PF6 pin is configured in analog mode
	 GPIO_InitStructre.GPIO_Mode = GPIO_Mode_AN;
	 //We don't need any pull up or pull down
	 GPIO_InitStructre.GPIO_PuPd = GPIO_PuPd_NOPULL;
	 //Affecting the port with the initialization structure configuration
	 GPIO_InitStructre.GPIO_Speed = GPIO_Speed_50MHz;

	 // Init ADC3 and ADC1 pin
	 GPIO_Init(GPIOF,&GPIO_InitStructre);
	 GPIO_Init(GPIOA,&GPIO_InitStructre);

	 // Configure All ADCs
	 ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	 ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	 ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	 ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	 ADC_CommonInit(&ADC_CommonInitStructure);

	 //data converted will be shifted to right
	 ADC_Init_structure.ADC_DataAlign = ADC_DataAlign_Right;
	 //Input voltage is converted into a 12bit number giving a maximum value of 4095
	 ADC_Init_structure.ADC_Resolution = ADC_Resolution_12b;
	 //the conversion is not continuous, the input data is converted once
	 ADC_Init_structure.ADC_ContinuousConvMode = DISABLE;
	 // conversion is synchronous with TIM2
	 ADC_Init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_CC2;
	 ///No trigger for the conversion
	 ADC_Init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	 //One conversion at a time ...
	 ADC_Init_structure.ADC_NbrOfConversion = 1;
	 //The scan is configured in one channel
	 ADC_Init_structure.ADC_ScanConvMode = DISABLE;
	 //Initialize ADC with the previous configuration
	 ADC_Init(ADC3,&ADC_Init_structure);
	 ADC_Init(ADC1,&ADC_Init_structure);

	 //Select the channel to be read from
	 ADC_RegularChannelConfig(ADC3,ADC_Channel_4,1,ADC_SampleTime_3Cycles);
	 ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_3Cycles);

	 //Enable ADC interrupt
	 NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	 // ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);
	 // ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	 NVIC_Init(&NVIC_InitStructure);

	 ADC_Cmd(ADC3,ENABLE);
	 ADC_Cmd(ADC1,ENABLE);
}

s32 GetADC3(void)
{
	 ADC_SoftwareStartConv(ADC3);//Start the conversion
	 while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));//Processing the conversion
	 return ADC_GetConversionValue(ADC3); //Return the converted data
}

s32 GetADC1(void)
{
	 ADC_SoftwareStartConv(ADC1);//Start the conversion
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));//Processing the conversion
	 return ADC_GetConversionValue(ADC1); //Return the converted data
}

void InitDAC(void)
{
	// DAC 1 OUT  PA4 D10
	// DAC 2 OUT  PA5 D13
	DAC_InitTypeDef  DAC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	DAC_DeInit();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

	/* Once the DAC channel is enabled, the corresponding GPIO pin is automatically
	   connected to the DAC converter. In order to avoid parasitic consumption,
	   the GPIO pin should be configured in analog */

	GPIO_InitStructure.GPIO_Pin  =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	DAC_InitStructure.DAC_Trigger 	   = DAC_Trigger_None ;
	DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	/* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
    automatically connected to the DAC converter. */
	DAC_Cmd(DAC_Channel_1, ENABLE);
	DAC_SetChannel1Data(DAC_Align_12b_R, 0);
	return;
}

void SetDAC(u32 Data)
{
	DAC_SetChannel1Data(DAC_Align_12b_R, Data);
	return;
}

void InitUSART(u32 baudrate)
{
	  GPIO_InitTypeDef GPIO_InitStructure;
	  USART_InitTypeDef USART_InitStructure;
	  USART_ClockInitTypeDef USART_ClockInitstructure;
	  NVIC_InitTypeDef NVIC_InitStructure;

	  /* Configure USART as alternate function  */
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	                                   //  Tx           Rx          Clk           Ct             Rt
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

      // USART Init Data
	  USART_InitStructure.USART_BaudRate   = baudrate;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits   = USART_StopBits_1;
	  USART_InitStructure.USART_Parity     = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode       =  USART_Mode_Rx | USART_Mode_Tx;

	/* USART Clock Initialization  */
	  USART_ClockInitstructure.USART_Clock   = USART_Clock_Disable ;
	  USART_ClockInitstructure.USART_CPOL    = USART_CPOL_Low ;
	  USART_ClockInitstructure.USART_LastBit = USART_LastBit_Enable;
	  USART_ClockInitstructure.USART_CPHA    = USART_CPHA_1Edge;

// USART IRQ init data
	  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;

	  /* USART configuration */

	  /* Enable GPIOD clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Enable USART3 clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);

	  /* Connect PXx to USARTx_Rx*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

	  /* Connect PXx to USARTx_Ck*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_USART3);

	  /* Connect PXx to USARTx_Cts*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource11, GPIO_AF_USART3);

	  /* Connect PXx to USARTx_Rts*/
	  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_USART3);

	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  // RX, TX interrupts enable
	  USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	  // USART_ITConfig(USART3, USART_IT_TXE, ENABLE);

	  USART_Init(USART3, &USART_InitStructure);
	  USART_ClockInit(USART3, &USART_ClockInitstructure);
	  NVIC_Init(&NVIC_InitStructure);
	  /* Enable USART */
	  USART_Cmd(USART3, ENABLE);
}
void TxUSART(u8 byte)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3, (uint8_t)byte);
}

u8 RxUSART(void)
{
	return USART_ReceiveData(USART3);
}

void TxCR(void)
{
	TxUSART(13);
}

void TxTAB(void)
{
	TxUSART(9);
}

void TxCharN(u8 Char, u32 Len)
{
	while(Len){
		// wait until data register is empty
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
		USART_SendData(USART3, Char);
		Len--;
	}
}

void PutsUSART(u8* s){

	u16 i=0;
	while(s[i]){
		// wait until data register is empty
		while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
		USART_SendData(USART3, s[i]);
		i++;
	}
}

void PutsDecUSART(u16* Dump, u32 Length){

	u16 i;
	for(i=0;i<Length;i++)
	{
		PutsUSART(Dec2Str(Dump[i]));
		TxUSART(13);
	}
}

u16* PutsDecColUSART(u16* Dump, u32 Length, u8 Ncol)
{
	u8  Col = Ncol;
	while(Length)
	{
		PutsUSART(Dec2Str(*Dump));
		Dump++;
		Col--;
		if(!Col)
		{
			TxUSART(13);
			Col = Ncol;
		}else{
			TxUSART(' ');
		}
		Length--;
		Delayms(50);
	}
	return Dump;
}

void InitUSBHSCDC(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  /* Configure USB HS as alternate function  */
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	                                  //  ID           Vbus          DM           DP
	  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	  /* USB HS configuration */

	  /* Enable GPIOB clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	  /* Enable USB HS clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_OTG_HS, ENABLE);

	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_OTG_HS);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_OTG_HS);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_OTG_HS);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_OTG_HS);

	  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void InitTimer2IRQ(u32 uSeconds)
{
	NVIC_InitTypeDef        NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = uSeconds - 1; // 1 MHz down to 1 KHz (1 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* TIM IT enable */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	/* Enable the TIM2 global Interrupt */
	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_Cmd(TIM2, ENABLE);
}

void InitTimer1PWM(u32 usPeriod)
{
//	https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2Fcortex_mx_stm32%2FSTM32F4%20TIM1%20PWM%20missing%20ch1.3&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=734#{7EDAA66A-D536-4737-A954-3479E6DFD025}

	GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

	// Timer1 1 CH: PE9   Con 12
	//		  2 CH: PE11  Con 14
	//		  3 CH: PE13  Con 16
	//		  4 CH: PE14  Con 17

    // Reset Timer 1
    TIM_DeInit(TIM1);

    GPIO_InitStructure.GPIO_Pin   = (GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14);
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = (usPeriod-1);
    TIM_TimeBaseStructure.TIM_Prescaler     = ((uint16_t)((SystemCoreClock /2) / 25000000) - 1);
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    /* PWM1 Mode configuration */
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode 		 = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = usPeriod/4;
    TIM_OCInitStructure.TIM_OCPolarity 	 = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	// Clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// GPIO Pins function as TIM1 pins
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    // Timer Channel Init
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    //required for timers 1 or 8
    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    /* TIM1 enable counter */
    TIM_Cmd(TIM1, ENABLE);
}

void SetDutyTimer1PWM(u8 Channel, u32 usPeriod)
{
   switch(Channel){
   case 1:{
	   TIM1->CCR1 = usPeriod;
	   break;
   }
   case 2:{
	   TIM1->CCR2 = usPeriod;
	   break;
   }
   case 3:{
	   TIM1->CCR3 = usPeriod;
	   break;
   }
   case 4:{
	   TIM1->CCR4 = usPeriod;
	   break;
   }
   }
}


