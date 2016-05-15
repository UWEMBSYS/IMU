/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/6DOrientation/Src/main.c
 * @author  CL
 * @version V2.1.0
 * @date    4-April-2016
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <string.h> /* strlen */
#include <stdio.h>  /* sprintf */
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A1_Examples
 * @{
 */

/** @addtogroup ORIENTATION
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define ORIENTATION_CHANGE_INDICATION_DELAY  100  /* LED is ON for this period [ms]. */



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile uint8_t mems_int1_detected       = 0;
static volatile uint8_t send_orientation_request = 0;
static char dataOut[256];

static void *LSM6DS3_X_0_handle = NULL;



/* Private function prototypes -----------------------------------------------*/
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void sendOrientation( void );



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to use sensor expansion board to find out the 6D orientation and send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like TeraTerm.
 *         After connection has been established:
 *         - the user can rotate the board to change the 6D orientation and then view the data using an hyper terminal
 * @param  None
 * @retval Integer
 */
int main( void )
{

  uint8_t status = 0;
  
  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize LED */
  BSP_LED_Init( LED2 );
  
  /* Initialize button */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#endif
  
#if (defined (USE_STM32L1XX_NUCLEO))
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
#endif
  
  /* Initialize UART */
  USARTConfig();
  
  /* Initialize all sensors */
  initializeAllSensors();
  /* Enable all sensors */
  enableAllSensors();
  
  /* Enable 6D orientation */
  BSP_ACCELERO_Enable_6D_Orientation_Ext( LSM6DS3_X_0_handle );
  
  while (1)
  {
  
    if ( mems_int1_detected != 0 )
    {
      if ( BSP_ACCELERO_Get_6D_Orientation_Status_Ext( LSM6DS3_X_0_handle, &status ) == COMPONENT_OK )
      {
        if ( status != 0 )
        {
          sendOrientation();
          BSP_LED_On( LED2 );
          HAL_Delay( ORIENTATION_CHANGE_INDICATION_DELAY );
          BSP_LED_Off( LED2 );
        }
      }
      mems_int1_detected = 0;
    }
    
    if ( send_orientation_request != 0 )
    {
      sendOrientation();
      send_orientation_request = 0;
    }
  }
}



/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{
  
  if(BSP_ACCELERO_Init( LSM6DS3_X_0, &LSM6DS3_X_0_handle ) == COMPONENT_ERROR)
  {
    /* LSM6DS3 not detected, switch on LED2 and go to infinity loop */
    BSP_LED_On( LED2 );
    while (1)
    {}
  }
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{

  BSP_ACCELERO_Sensor_Enable( LSM6DS3_X_0_handle );
}



/**
 * @brief  Send actual 6D orientation to UART
 * @param  None
 * @retval None
 */
static void sendOrientation( void )
{

  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  uint8_t instance;
  
  BSP_ACCELERO_Get_Instance( LSM6DS3_X_0_handle, &instance );
  
  if ( BSP_ACCELERO_Get_6D_Orientation_XL_Ext( LSM6DS3_X_0_handle, &xl ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting 6D orientation XL axis from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_XH_Ext( LSM6DS3_X_0_handle, &xh ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting 6D orientation XH axis from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_YL_Ext( LSM6DS3_X_0_handle, &yl ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting 6D orientation YL axis from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_YH_Ext( LSM6DS3_X_0_handle, &yh ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting 6D orientation YH axis from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_ZL_Ext( LSM6DS3_X_0_handle, &zl ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting 6D orientation ZL axis from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  if ( BSP_ACCELERO_Get_6D_Orientation_ZH_Ext( LSM6DS3_X_0_handle, &zh ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting 6D orientation ZH axis from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  
  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 )
  {
    sprintf( dataOut, "\n  ________________  " \
                      "\n |                | " \
                      "\n |  *             | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |________________| \n" );
  }
  
  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( dataOut, "\n  ________________  " \
                      "\n |                | " \
                      "\n |             *  | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |________________| \n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 )
  {
    sprintf( dataOut, "\n  ________________  " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |  *             | " \
                      "\n |________________| \n" );
  }
  
  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( dataOut, "\n  ________________  " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |                | " \
                      "\n |             *  | " \
                      "\n |________________| \n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 )
  {
    sprintf( dataOut, "\n  __*_____________  " \
                      "\n |________________| \n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 )
  {
    sprintf( dataOut, "\n  ________________  " \
                      "\n |________________| " \
                      "\n    *               \n" );
  }
  
  else
  {
    sprintf( dataOut, "None of the 6D orientation axes is set in LSM6DS3 - accelerometer[%d].\n", instance );
  }
  
  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
}



/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{

  /* User button. */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  if(GPIO_Pin == KEY_BUTTON_PIN)
#endif
#if (defined (USE_STM32L1XX_NUCLEO))
  if(GPIO_Pin == USER_BUTTON_PIN)
#endif
  {
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
    if ( BSP_PB_GetState( BUTTON_KEY ) == GPIO_PIN_RESET )
#endif
#if (defined (USE_STM32L1XX_NUCLEO))
    if ( BSP_PB_GetState( BUTTON_USER ) == GPIO_PIN_RESET )
#endif
#if (defined (USE_STM32L0XX_NUCLEO))
    if ( BSP_PB_GetState( BUTTON_KEY ) == GPIO_PIN_SET )
#endif
    {
      /* Request to send actual 6D orientation to UART (available only for LSM6DS3 sensor). */
      send_orientation_request = 1;
    }
  }
    
  /* 6D orientation (available only for LSM6DS3 sensor). */
  else if ( GPIO_Pin == M_INT1_PIN )
  {
    mems_int1_detected = 1;
  }
}



/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler( void )
{

  while (1)
  {}
}



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t *file, uint32_t line )
{

  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {}
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
