/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/SingleDoubleTap/Src/main.c
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

#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A1_Examples
  * @{
  */

/** @addtogroup SINGLE_DOUBLE_TAP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
typedef enum
{
  MODE_DEFAULT    = 0,
  MODE_SINGLE_TAP = 1,
  MODE_DOUBLE_TAP = 2
} Mode_t;



/* Private define ------------------------------------------------------------*/

#define SINGLE_TAP_INDICATION_DELAY  100  /* LED is ON for this period [ms]. */
#define DOUBLE_TAP_INDICATION_DELAY  100  /* LED is ON for this period [ms]. */
#define INDICATION_WAIT_DELAY        100  /* LED is locked OFF for this period [ms]. */



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile uint8_t mems_int1_detected  = 0;

static volatile uint8_t single_tap_mode_request = 1;
static volatile uint8_t double_tap_mode_request = 0;
static volatile uint8_t default_mode_request    = 0;

static Mode_t mode = MODE_DEFAULT;

static void *LSM6DS3_X_0_handle = NULL;



/* Private function prototypes -----------------------------------------------*/
static void initializeAllSensors( void );
static void enableAllSensors( void );



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to detect the single and double tap events using the sensor expansion board and a STM32 Nucleo board.
 *         After application is started, the user can try to tap the STM32 Nucleo board; when the single tap is detected,
 *         the LED is switched on for a while. Press the user button to pass from the single tap detection to the double tap detection feature.
 *         When the double tap is detected, the LED is switched on twice for a while. Press again the user button to disable the single/double tap detection feature.
 *         Press the user button to enable again the single tap detection feature.
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
  
  /* Initialize all sensors */
  initializeAllSensors();
  /* Enable all sensors */
  enableAllSensors();
  
  while (1)
  {
  
    if ( mems_int1_detected == 1 )
    {
      if(mode == MODE_SINGLE_TAP)
      {
        if ( BSP_ACCELERO_Get_Single_Tap_Detection_Status_Ext( LSM6DS3_X_0_handle, &status ) == COMPONENT_OK )
        {
          if ( status != 0 )
          {
            BSP_LED_On( LED2 );
            HAL_Delay( SINGLE_TAP_INDICATION_DELAY );
            BSP_LED_Off( LED2 );
            HAL_Delay( INDICATION_WAIT_DELAY );
          }
        }
      }
      else if(mode == MODE_DOUBLE_TAP)
      {
        if ( BSP_ACCELERO_Get_Double_Tap_Detection_Status_Ext( LSM6DS3_X_0_handle, &status ) == COMPONENT_OK )
        {
          if ( status != 0 )
          {
            BSP_LED_On( LED2 );
            HAL_Delay( DOUBLE_TAP_INDICATION_DELAY );
            BSP_LED_Off( LED2 );
            HAL_Delay( INDICATION_WAIT_DELAY );
            BSP_LED_On( LED2 );
            HAL_Delay( DOUBLE_TAP_INDICATION_DELAY );
            BSP_LED_Off( LED2 );
            HAL_Delay( INDICATION_WAIT_DELAY );
          }
        }
      }
      
      mems_int1_detected = 0;
    }
    
    if ( single_tap_mode_request == 1 )
    {
      if ( BSP_ACCELERO_Enable_Single_Tap_Detection_Ext( LSM6DS3_X_0_handle ) == COMPONENT_OK )
      {
        mode = MODE_SINGLE_TAP;
        single_tap_mode_request = 0;
      }
    }
    
    if ( double_tap_mode_request == 1 )
    {
      if ( BSP_ACCELERO_Disable_Single_Tap_Detection_Ext( LSM6DS3_X_0_handle ) == COMPONENT_OK )
      {
        if ( BSP_ACCELERO_Enable_Double_Tap_Detection_Ext( LSM6DS3_X_0_handle ) == COMPONENT_OK )
        {
          mode = MODE_DOUBLE_TAP;
          double_tap_mode_request = 0;
        }
      }
    }
    
    if ( default_mode_request == 1 )
    {
      if ( BSP_ACCELERO_Disable_Double_Tap_Detection_Ext( LSM6DS3_X_0_handle ) == COMPONENT_OK )
      {
        mode = MODE_DEFAULT;
        default_mode_request = 0;
      }
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
      if ( mode == MODE_DEFAULT )
      {
        single_tap_mode_request = 1;
      }
          
      else if ( mode == MODE_SINGLE_TAP )
      {
        double_tap_mode_request = 1;
      }
          
      else if ( mode == MODE_DOUBLE_TAP )
      {
        default_mode_request = 1;
      }
          
      /* ERROR. */
      else
      {
        Error_Handler();
      }
    }
  }
    
  /* Single tap or double tap detection (available only for LSM6DS3 sensor). */
  else if ( GPIO_Pin == M_INT1_PIN )
  {
    mems_int1_detected = 1;
  }
    
  /* ERROR. */
  else
  {
    Error_Handler();
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
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed( uint8_t* file, uint32_t line )
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
