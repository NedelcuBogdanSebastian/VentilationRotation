/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern volatile uint32_t TimingDelay;

extern volatile uint8_t sec;
extern volatile uint8_t min;
extern volatile uint8_t hour;

extern volatile uint8_t TEST_COIL_FLAG;

extern volatile uint8_t GR[4];
extern volatile uint8_t GR1_STATE_FLAG;
extern volatile uint8_t GR2_STATE_FLAG;
extern volatile uint8_t GR3_STATE_FLAG;
extern volatile uint8_t GR4_STATE_FLAG;

volatile uint32_t GR_TIME[4] = {0, 0, 0, 0};


/* Private function prototypes -----------------------------------------------*/

extern void prvvTIMERExpiredISR( void );
extern void prvvUARTTxReadyISR(void);
extern void prvvUARTRxISR(void);


/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    if (TimingDelay != 0x00) TimingDelay--;	
}



/*******************************************************************************
* Function Name  : RTC_IRQHandler
* Description    : This function handles RTC global interrupt request.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

void swap32(uint32_t *a, uint32_t *b)
{
	uint32_t temp = *a; *a = *b; *b = temp;
}
void swap8(uint8_t *a, uint8_t *b)
{
	uint8_t temp = *a; *a = *b; *b = temp;
}

void Sort_GR_TIME_and_GR(void)
{
	// we sort ascending the GR_TIME[], and also swap the GR[]
	// on GR_TIME[0] will be the smallest working time in seconds
	// also on GR[0] we will have which ventilation block has the smallest working time
	//
	// we use Bose-Nelson sorting networks
	// generated from the web page:
	// http://pages.ripco.net/%7Ejgamble/nw.html
	// for 3 variables, there are 3 comparators in this network, (3 ventilation blocks)
	// [1,2], [0,2], [0,1]
	// for four variables, there are 5 comparators in this network, (4 ventilation blocks)
	// [0,1], [2,3], [0,2], [1,3], [1,2]

	if (GR_TIME[0] > GR_TIME[1]) {
		swap32(&GR_TIME[0], &GR_TIME[1]);
		swap8(&GR[0], &GR[1]);
	}
	if (GR_TIME[2] > GR_TIME[3]) {
		swap32(&GR_TIME[2], &GR_TIME[3]);
		swap8(&GR[2], &GR[3]);
	}
	if (GR_TIME[0] > GR_TIME[2]) {
		swap32(&GR_TIME[0], &GR_TIME[2]);
		swap8(&GR[0], &GR[2]);
	}
	if (GR_TIME[1] > GR_TIME[3]) {
		swap32(&GR_TIME[1], &GR_TIME[3]);
		swap8(&GR[1], &GR[3]);
	}
	if (GR_TIME[1] > GR_TIME[2]) {
		swap32(&GR_TIME[1], &GR_TIME[2]);
		swap8(&GR[1], &GR[2]);
	}

	// if the third ventilation block, the one with the highest working time,
	// reach 0x7FFFFFFF, approx. 68year * 365DAYS * 24H * 3600SEC,
	// then reset all time counters and ventilation blocks order
	if (GR_TIME[3] > 0x7FFFFFFF)
	{
		GR[0] = 1;
		GR[1] = 2;
		GR[2] = 3;
		GR[3] = 4;
		GR_TIME[0] = 0;
		GR_TIME[1] = 0;
		GR_TIME[2] = 0;
		GR_TIME[3] = 0;
	}
}


void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)
    {
        // clear the RTC second interrupt
        RTC_ClearITPendingBit(RTC_IT_SEC);

        // blink LED as seconds indicator
	    GPIOB->ODR ^= GPIO_Pin_0;

        sec++;

	    // increment working time for active ventilation blocks
		if (GR1_STATE_FLAG == 1) GR_TIME[0]++;
		if (GR2_STATE_FLAG == 1) GR_TIME[1]++;
		if (GR3_STATE_FLAG == 1) GR_TIME[2]++;
		if (GR4_STATE_FLAG == 1) GR_TIME[3]++;

        if (sec == 60)
        {
            sec = 0;

    		// if testing ventilation blocks rotation is active
    		// then we also make the sorting here (EACH MINUTE), we don't wait until 10 hours
    		if (TEST_COIL_FLAG == 1) {
    			Sort_GR_TIME_and_GR();
    		}

            min++;
            if (min == 60)
            {
                min = 0;
                hour++;
                
                if (hour == 9) {
                    hour = 0;

                    Sort_GR_TIME_and_GR();
                }
            }
        }
    }
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/


void TIM4_IRQHandler(void)
{
		if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
		{
				/* Clear TIM4 Capture Compare1 interrupt pending bit*/
				TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
				prvvTIMERExpiredISR( );
		}
}



void USART1_IRQHandler(void)
{
		if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
		{
				prvvUARTRxISR();
				USART_ClearITPendingBit(USART1, USART_IT_RXNE);		
		}

		if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
		{
				prvvUARTTxReadyISR();
				USART_ClearITPendingBit(USART1, USART_IT_TXE);
		}

		if(USART_GetFlagStatus(USART1,USART_FLAG_ORE)==SET)
		{
				USART_ClearFlag(USART1,USART_FLAG_ORE);
				USART_ReceiveData(USART1);
		}
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
