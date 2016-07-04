// ----------------------------------------------------------------------------
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "resources.h"

#include <math.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_adc.h>

// diodes
#define GreenLED GPIO_Pin_9
#define BlueLED GPIO_Pin_8
#define LEDGPIO GPIOC

//Define Push button
#define PushButton_Pin GPIO_Pin_0
#define PushButton_GPIO GPIOA

//Initialization structs

GPIO_InitTypeDef			GPIO_InitStructure;
GPIO_InitTypeDef         	LEDs;
GPIO_InitTypeDef         	Buttons;
GPIO_InitTypeDef         	Pins;
TIM_TimeBaseInitTypeDef 	TTB;
DAC_InitTypeDef         	DAC_InitStructure;
NVIC_InitTypeDef         	DACNVIC_InitStructure;
NVIC_InitTypeDef         	ADCNVIC_InitStructure;
ADC_InitTypeDef 			ADC_InitStructure;

uint32_t lutIndex =0;
uint32_t lutStep =15;
uint8_t  lutStepADC =1;
int handleTIM =1;
int usingLeds =0;
int usingDAC=1;
int usingADC=1;

uint16_t outputDAC=4095;


uint32_t accumulator=0;
uint32_t accumulatorStep=0;
uint16_t CurrentTimerVal = 0;

#define ONE ((int)(2*256*256*256))

uint32_t  feedback_i = ONE*0.25/32768;
uint32_t  accumulator_i = ONE*1;
uint32_t  accumulatorDelta_i = ONE*0.005;


uint16_t ADC1ConvertedValue =0;
uint16_t ADC1ConvertedPreviousValue =0;

// configure board
void  InitBoard()
{


        SystemInit();
        SystemCoreClockUpdate();

        //Enable GPIO Clock
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        // enable Timer
        // TIM3 for DAC
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);
        // TIM1 for ADC
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

        //Initialize LEDs
        GPIO_StructInit(&LEDs);
        LEDs.GPIO_Pin = GreenLED | BlueLED;
        LEDs.GPIO_Mode = GPIO_Mode_OUT;
        LEDs.GPIO_OType = GPIO_OType_PP;
        LEDs.GPIO_PuPd = GPIO_PuPd_NOPULL;
        LEDs.GPIO_Speed = GPIO_Speed_Level_3; //50MHz
        GPIO_Init(LEDGPIO, &LEDs);
        // turn on green  and blue Leds
        GPIO_SetBits(LEDGPIO, GreenLED);
        GPIO_ResetBits(LEDGPIO, BlueLED);


}
// configure DAC
void InitDAC(void)
{

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* DAC Periph clock enable */

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);



  // Configure PA.04/05 (DAC) as output -------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Fill DAC InitStructure */


  	  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T3_TRGO;
 	  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;//DAC_OutputBuffer_Enable;


 	  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

 	  //(+) Enable the DAC channel using DAC_Cmd()
 	  DAC_Cmd(DAC_Channel_1, ENABLE);


  /* DAC channel1 Configuration */
 	  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
     automatically connected to the DAC converter. */
 	  DAC_Cmd(DAC_Channel_2, ENABLE);



}


/*
 * Example of ADC with IRQ
 *
 * https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fSTM32Discovery%2fstm32f0%20interrupt&FolderCTID=0x01200200770978C69A1141439FE559EB459D75800084C20D8867EAD444A5987D47BE638E0F&currentviews=494
*/

void InitADC()
{


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOC, ENABLE);

	  // adc pins configuration

	GPIO_StructInit(&Pins);
	Pins.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	Pins.GPIO_Mode = GPIO_Mode_AN;
	Pins.GPIO_OType = GPIO_OType_PP;
	Pins.GPIO_PuPd = GPIO_PuPd_NOPULL;
	Pins.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &Pins);


	ADC_DeInit(ADC1);

	// timer, and interupt for ADC
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
	 /* Conversions are 12 bit - put them in the lower 12 bits of the result */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;



	ADC_Init(ADC1, &ADC_InitStructure);

	/* Convert the ADC1 input with 55.5 Cycles as sampling time */
	//ADC_ChannelConfig(ADC1, 1, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1,2,ADC_SampleTime_55_5Cycles); // PA1 PIN


	/* ADC Calibration */
	 ADC_GetCalibrationFactor(ADC1);

	/* Enable ADC1 */
	 ADC_Cmd(ADC1, ENABLE);


	 /* Wait the ADCEN falg */
	 while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));

	   /* ADC1 regular Software Start Conv */
	   	ADC_StartOfConversion(ADC1);
	   	//*  TODO WHY THIS IS NOT WORKING
	   	/*
	   	ADCNVIC_InitStructure.NVIC_IRQChannel = ADC1_COMP_IRQn;
	  	ADCNVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	   	ADCNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&ADCNVIC_InitStructure);
		*/
	     // Enable EOC interrupt
	    ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	    // Lower Priority
	    //NVIC_SetPriority(ADC1_COMP_IRQn, 1);


}


void InitTimer()
{

	 	 	SystemInit(); //Ensure CPU is running at correctly set clock speed
	     	SystemCoreClockUpdate(); //Update SystemCoreClock variable to current clock speed
	     	//SysTick_Config(SystemCoreClock/1000); //Set up a systick interrupt every millisecond

            TTB.TIM_CounterMode = TIM_CounterMode_Up;
            TTB.TIM_Prescaler = 100-1; //  4800 kHz
            TTB.TIM_Period = 10-1; //1Hz;
            TTB.TIM_RepetitionCounter = 0;
            TIM_TimeBaseInit(TIM3, &TTB);
            TIM_Cmd(TIM3, ENABLE);



            /* http://visualgdb.com/tutorials/arm/stm32/timers/ */
            TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
            TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
            /* http://forbot.pl/blog/artykuly/programowanie/kurs-stm32-7-liczniki-timery-w-praktyce-pwm-id8459 */



            DACNVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
            DACNVIC_InitStructure.NVIC_IRQChannelPriority = 0;
            DACNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&DACNVIC_InitStructure);
            // Higher Priority
            NVIC_SetPriority(TIM3_IRQn, 0);

}




void TIM3_IRQHandler()
{



    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {


    	if (usingLeds)
    	{

        if (GPIO_ReadOutputDataBit(LEDGPIO, BlueLED))
                    GPIO_ResetBits(LEDGPIO, BlueLED);
                else
                    GPIO_SetBits(LEDGPIO, BlueLED);

        if (GPIO_ReadOutputDataBit(LEDGPIO, GreenLED))
                    GPIO_ResetBits(LEDGPIO, GreenLED);
                else
                    GPIO_SetBits(LEDGPIO, GreenLED);

    	}


    	if (usingDAC)
    	{
    	// index modulo number of samples

    	if (lutIndex>=1024)
    	{
    		//lutIndex= lutIndex%1024;
    		lutIndex-=1024; // instead of zero
    	}

			DAC_SetChannel1Data(DAC_Align_12b_R,(uint16_t) (0.98*Sine1024_12bit[lutIndex]) );
			lutIndex+=lutStep;
    	}

    	/*
    	 http://stackoverflow.com/questions/16889426/fm-synthesis-using-phase-accumulator
    	 http://blog.thelonepole.com/2011/07/numerically-controlled-oscillators/
    	 */



       TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    }

}
// handler for ADC sampling with interrupt ...
void ADC1_COMP_IRQHandler(void)
{
 if(ADC_GetITStatus(ADC1, ADC_IT_EOC) != RESET)
 {

	 ADC1ConvertedValue=ADC_GetConversionValue(ADC1);
	 if (ADC1ConvertedValue > 2000)
	     {

	       GPIO_ResetBits(GPIOC, BlueLED);
	       GPIO_SetBits(GPIOC, GreenLED);
	       lutStep=15;
	     }
	     else
	     {

	       GPIO_SetBits(GPIOC, BlueLED);
	       GPIO_ResetBits(GPIOC, GreenLED);
	       lutStep=5;
	     }

	// lutStep=1+(uint16_t)(511/4095)*ADC1ConvertedValue;
	ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

 }
}

int main(int argc, char* argv[])
{


    InitBoard();

    if (usingDAC)
    {
    	InitDAC();
    }

    if (usingADC)
    {
    	InitADC();
    }

    InitTimer();


  // Infinite loop
  while (1)

  {

	  ADC1ConvertedValue=ADC_GetConversionValue(ADC1);


	  if (ADC1ConvertedValue < 1024)
	  	     {

		  	  GPIO_ResetBits(GPIOC, BlueLED);
		  	  GPIO_ResetBits(GPIOC, GreenLED);
		  	  lutStep=5;

	  	     }
	  if (ADC1ConvertedValue < 2048 && ADC1ConvertedValue>=1000)
	  	     {

	  	       GPIO_ResetBits(GPIOC, BlueLED);
	  	  	   GPIO_SetBits(GPIOC, GreenLED);
	  	  	   lutStep=10;
	  	     }
	  if (ADC1ConvertedValue < 3036 && ADC1ConvertedValue>=2000)
	  	  	     {

	  	  	       GPIO_SetBits(GPIOC, BlueLED);
	  	  	       GPIO_ResetBits(GPIOC, GreenLED);
	  	  	  	   lutStep=15;
	  	  	     }

	  if (ADC1ConvertedValue < 4000 && ADC1ConvertedValue>=3000)
	  	  	     {

	  	  	       GPIO_SetBits(GPIOC, BlueLED);
	  	  	       GPIO_SetBits(GPIOC, GreenLED);
	  	  	  	   lutStep=20;
	  	  	     }

	  if (ADC1ConvertedValue < 5000 && ADC1ConvertedValue>=4000)
	  	  	     {

		  	  	  GPIO_SetBits(GPIOC, BlueLED);
		  	  	  GPIO_SetBits(GPIOC, GreenLED);
	  	  	  	  lutStep=25;
	  	  	     }



	  lutStepADC=(int)(ADC1ConvertedValue/1000);
	  //ADC1ConvertedPreviousValue=ADC1ConvertedValue;
	  lutStep=5+5*lutStepADC;

	  //lutStep=1+(uint16_t)(512/4095)*(ADC1ConvertedValue);

  }

  return 0;
}




#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  while (1) {
  }
}
#endif

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
