/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/*
 * Olimex + Eclipse Helios + OpenOCD project
 *
 *    board: STM32-H407
 *  project: Blinking LED (FLASH)
 *
 *
 */

/* Includes ------------------------------------------------------------------*/

#define USE_STDPERIPH_DRIVER

#include "main.h"
#include "systemmaglev.c"
#include "common.c"
#include "PID.c"


/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define INFO1  "       System Initialization Successful !                                 "
#define INFO10 "       1. ADC input should be 3.2V and Op-Amp output should be 8.0V.      "
#define INFO11 "       2. Turn the Trim potentiometer to initialize the reference         "
#define INFO12 "       3. Push Wk-Up button to continue !                                 "
#define INFO13 "       4. Stick the ball to the coil to get the ADC offset.               "
#define INFO14 "       5. Push Wk-Up button to start the controller sequence !            "
#define INFO3  "       Value = "
#define INFO4  "       Setting Up PV Offset   "
#define INFO5  "       Setting Up Reference  "
#define INFO7  "       PID Control Loop Initiated !  "
#define INFO8  "       The control is set to     "
#define INFO9  "       Data Gathered trending stopped !     "

/* Private macro -------------------------------------------------------------*/

#define BITS_FOR_FRACTIONAL_PART 16
#define FILL_MAX_32B 0xffffffff
#define TREND_BUFFER_SIZE 10000
#define MAX_STRLEN_USART 1024
#define SCOPE_CHANNEL1 GPIO_Pin_3
#define SCOPE_CHANNEL2 GPIO_Pin_1
#define REF_INTERP_BUFFER_LEN 4095
#define TIMER_SCALE_INTERVAL 200
// Keep in mind always less or equal to 900 !!!!
#define MAX_REFERENCE 900


/* Private variables ---------------------------------------------------------*/

MYPID ST;
static u32 TimerScaler  = TIMER_SCALE_INTERVAL;
static u8  StartControl = 0;
static s32 Con = 0;
static s32 Cnt = 0;
static s32 RefSum = 0;
static s32 RefARR[REF_INTERP_BUFFER_LEN];
static s32 RefInd = 0;


/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

void TIM2_IRQHandler(void)
{	// All IRQs
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{ // Our PID :)
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		if(StartControl == 0xff)
		{
			// Get new Smoothed reference sum
			if(RefInd < REF_INTERP_BUFFER_LEN)
			{
				RefSum = RefSum - RefARR[RefInd];
				RefARR[RefInd] = GetADC1();
				RefSum = RefSum + RefARR[RefInd];
				RefInd++;
			}else{ RefInd = 0; }
			// Read the reference
			ST.R = FracNum((RefSum / REF_INTERP_BUFFER_LEN),MAX_REFERENCE,MAX_ADC_DAC);
			// Toggle timer State for monitoring
			GPIO_ToggleBits(GPIOG,SCOPE_CHANNEL1);
			//Read the position from ADC
			ST.YRow = GetADC3();
			// Get Equilibrium DAC Value
			ST.ConOffset = QuadraticEquation(ST.YRow);
			// Calculate our PID
			PIDDo(&ST);
			//Offset it and scale it
			Con = (ST.Control + ST.ConOffset) / 6249;
			if(Con < 0){ Con = 0; }
			if(Con > MAX_VALUE_PWM){ Con = MAX_VALUE_PWM; }
			// Send it to PWM
			SetDutyTimer1PWM(3,Con);
			// Toggle timer State for monitoring
			GPIO_ToggleBits(GPIOG,SCOPE_CHANNEL2);
		}
		// Cave Johnson we'r done here ...
	}
}


s32 InitRef(void)
{
	s32 AdcFracked = 0;
	u8 RefDigLen = 0;
	RefDigLen = DigitsCount(MAX_REFERENCE,10);
	TxTAB();
	TxCharN(95,RefDigLen);
		do
		{
			TxCharN(8,RefDigLen);
			AdcFracked = FracNum(GetADC1(),MAX_REFERENCE,MAX_ADC_DAC);
			PutsUSART(Dec2NcharsStr(AdcFracked,RefDigLen));
			Delayms(80);
		}while(!GetButton());
	return AdcFracked;
}

s32 InitValue(void)
{
	s32 Adc;
	s32 Count = MAX_ADC_DAC;
	u8 Btn;
		do
		{
			Btn = GetButton();
			Adc = GetADC3();
			Count--;
			if(!Count){ Count = MAX_ADC_DAC; }
			if(Count <= Adc){  SetLed(1); }else{ SetLed(0); }
		}while(Btn == 0);
	return Adc;
}

int main(void){
  PIDMaglevInit(&ST);
  InitButton();
  InitADC();
  InitDAC();
  InitLed();
  InitScopePins();
  InitUSART(115200);
  InitTimer2IRQ(40);
  InitTimer1PWM(20000);
  TxCharN(8,DigitsCount(MAX_REFERENCE,10)+1);
  TxCR();
  PutsUSART((u8*)INFO1); TxCR();
  PutsUSART((u8*)INFO10); TxCR();
  PutsUSART((u8*)INFO11); TxCR();
  PutsUSART((u8*)INFO12); TxCR();
  PutsUSART((u8*)INFO13); TxCR();
  PutsUSART((u8*)INFO14); TxCR();
  PutsUSART((u8*)INFO5);
  PutsUSART((u8*)INFO3);
  ST.R = InitRef();
  Delayms(2000);
  TxUSART(' ');  TxUSART('!'); TxUSART(13);
  SetDAC(MAX_ADC_DAC);
  SetDutyTimer1PWM(3,MAX_VALUE_PWM);
  SetDutyTimer1PWM(4,MAX_VALUE_PWM);
  PutsUSART((u8*)INFO4);
  ST.YOffset = (InitValue() - 200);
  PutsUSART((u8*)INFO3);
  PutsUSART(Dec2Str(ST.YOffset));
  TxUSART(' ');  TxUSART('!');  TxUSART(13);
  PutsUSART((u8*)INFO7);
  TxCR();
  Cnt = 0;

  FillArr(RefARR,REF_INTERP_BUFFER_LEN,ST.R);
  RefSum = SumArr(RefARR,REF_INTERP_BUFFER_LEN);
  Delayms(3000);

  TxUSART(27);
  StartControl = 0xff;

  while(1)
  {
	Delayms(1);
	TimerScaler--;
	SetDutyTimer1PWM(4,Con);
	if(!TimerScaler)
	{
		TimerScaler = TIMER_SCALE_INTERVAL;
		TxCR();
		PutsUSART(Dec2Str(Cnt++));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.YRow));
		TxTAB();
		PutsUSART(PWM2Str(Con));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.ConOffset));
		TxTAB();
		PutsUSART(Dec2Str(ST.R));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.Control + ST.ConOffset));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.Err));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.vP));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.vI));
		TxTAB();
		PutsUSART(FloatDec2Str(ST.vD));
	}
	if(ST.Err > 0){ SetLed(1); }else{ SetLed(0); }
  }

return 0;
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
