/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file is responsible to offer board support package and is 
  *          configurable by user.
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

/* Includes ------------------------------------------------------------------*/
#include "usb_bsp.h"
#include "usbd_conf.h"
#include "nvic.h"

/**
* @brief  USB_OTG_BSP_Init
*         Initilizes BSP configurations
* @param  None
* @retval None
*/

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
	/* Enable GPIO clocks */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; /* Enable GPIOA clock */

	/* USB VBUS, ID and DM/DP pin configuration */
	GPIOA->MODER   &= ~(uint32_t)0x03FC0000;	/* unselect mode */
	GPIOA->MODER   |=  (uint32_t)0x02A00000;	/* select AF mode (ID,DM,DP) / input mode (VBUS) */
	GPIOA->OSPEEDR |=  (uint32_t)0x03FC0000;	/* set speed to 100 MHz */
	GPIOA->OTYPER  &= ~(uint32_t)0x00001800;  /* select push-pull (DM,DP) */
	GPIOA->OTYPER  |=  (uint32_t)0x00000600;  /* select open drain (VBUS,ID) */
	GPIOA->PUPDR   &= ~(uint32_t)0x03C00000;  /* no pull-up/-down (VBUS,DM,DP) */
	GPIOA->PUPDR   |=  (uint32_t)0x00100000;  /* pull-up (ID) */
	
	/* USB ID and DM/DP AF 10 configuration */
  GPIOA->AFR[1] &= ~(uint32_t)0x000FFF00;
	GPIOA->AFR[1] |=  (uint32_t)0x000AAA00;

	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
}
/**
* @brief  USB_OTG_BSP_EnableInterrupt
*         Enabele USB Global interrupt
* @param  None
* @retval None
*/
void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
	IRQ_init_enable(OTG_FS_IRQn,1,3);
}
/**
* @brief  USB_OTG_BSP_uDelay
*         This function provides delay time in micro sec
* @param  usec : Value of delay required in micro sec
* @retval None
*/
void USB_OTG_BSP_uDelay (const uint32_t usec)
{
  uint32_t count = 0;
  const uint32_t utime = (120 * usec / 7);
  do
  {
    if ( ++count > utime )
    {
      return ;
    }
  }
  while (1);
}


/**
* @brief  USB_OTG_BSP_mDelay
*          This function provides delay time in milli sec
* @param  msec : Value of delay required in milli sec
* @retval None
*/
void USB_OTG_BSP_mDelay (const uint32_t msec)
{
  USB_OTG_BSP_uDelay(msec * 1000);   
}
