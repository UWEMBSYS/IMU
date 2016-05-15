/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/Pedometer/Src/main.c
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

/** @addtogroup PEDOMETER
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define STEP_INDICATION_DELAY     100  /* LED is ON for this period [ms]. */
#define SEND_STEP_COUNT_TIMEOUT  3000  /* Send step count to UART after this timeout [ms]. */



/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile uint8_t mems_int1_detected       = 0;
static volatile uint8_t step_count_reset_request = 0;
static char dataOut[256];
static RTC_HandleTypeDef RtcHandle;
static uint16_t step_count                       = 0;

static void *LSM6DS3_X_0_handle = NULL;



/* Private function prototypes -----------------------------------------------*/

static void RTC_Config( void );
static void RTC_TimeStampConfig( void );
static void RTC_Handler( void );
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void Send_Step_Count( void );



/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main function is to show how to use sensor expansion board to count steps and send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like TeraTerm.
 *         After connection has been established:
 *         - the user can shake the board to simulate the steps and then view the data using an hyper terminal.
 * @param  None
 * @retval Integer
 */
int main( void )
{

  uint8_t status         = 0;
  uint32_t previous_tick = 0;
  
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
  
  /* Initialize RTC */
  RTC_Config();
  RTC_TimeStampConfig();
  
  /* Initialize all sensors */
  initializeAllSensors();
  /* Enable all sensors */
  enableAllSensors();
  
  /* Enable pedometer */
  BSP_ACCELERO_Enable_Pedometer_Ext( LSM6DS3_X_0_handle );
  
  /* Store current tick. */
  previous_tick = user_currentTimeGetTick();
  
  while (1)
  {
  
    if ( mems_int1_detected != 0 )
    {
      if ( BSP_ACCELERO_Get_Pedometer_Status_Ext( LSM6DS3_X_0_handle, &status ) == COMPONENT_OK )
      {
        if ( status != 0 )
        {
          Send_Step_Count();
          BSP_LED_On( LED2 );
          HAL_Delay( STEP_INDICATION_DELAY );
          BSP_LED_Off( LED2 );
        }
      }
      mems_int1_detected = 0;
    }
    
    if ( step_count_reset_request != 0 )
    {
      if ( BSP_ACCELERO_Reset_Step_Counter_Ext( LSM6DS3_X_0_handle ) == COMPONENT_OK )
      {
        step_count_reset_request = 0;
      }
    }
    
    /* After defined timeout send time stamp and step count to UART. */
    if ( user_currentTimeGetElapsedMS( previous_tick ) >= SEND_STEP_COUNT_TIMEOUT )
    {
      Send_Step_Count();
      previous_tick = user_currentTimeGetTick();
    }
  }
}



/**
 * @brief  Handles the time+date getting/sending
 * @param  None
 * @retval None
 */
static void RTC_Handler( void )
{

  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;
  
  HAL_RTC_GetTime( &RtcHandle, &stimestructure, FORMAT_BIN );
  HAL_RTC_GetDate( &RtcHandle, &sdatestructureget, FORMAT_BIN );
  subSec = (((((( int )RTC_SYNCH_PREDIV) - (( int )stimestructure.SubSeconds)) * 100) / ( RTC_SYNCH_PREDIV + 1 )) & \
            0xff );
            
  /* First send the extra line separately to clean the UART line (better results). */
  sprintf( dataOut, "\n" );
  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
  
  sprintf( dataOut, "Time stamp: %02d:%02d:%02d.%02d\n", stimestructure.Hours, stimestructure.Minutes, \
           stimestructure.Seconds, subSec );
  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
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
 * @brief  Send time stamp and step count to UART
 * @param  None
 * @retval None
 */
static void Send_Step_Count( void )
{
  uint8_t instance;
  
  RTC_Handler();
  
  BSP_ACCELERO_Get_Instance( LSM6DS3_X_0_handle, &instance );
  
  if ( BSP_ACCELERO_Get_Step_Count_Ext( LSM6DS3_X_0_handle, &step_count ) == COMPONENT_ERROR )
  {
    sprintf( dataOut, "Error getting step count from LSM6DS3 - accelerometer[%d].\n", instance );
  }
  else
  {
    sprintf( dataOut, "Step count: %d\n", step_count );
  }
  
  HAL_UART_Transmit( &UartHandle, ( uint8_t* )dataOut, strlen( dataOut ), 5000 );
}



/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config( void )
{

  /*##-1- Configure the RTC peripheral #######################################*/
  RtcHandle.Instance = RTC;
  
  /* Configure RTC prescaler and RTC data registers */
  /* RTC configured as follow:
     - Hour Format    = Format 12
     - Asynch Prediv  = Value according to source clock
     - Synch Prediv   = Value according to source clock
     - OutPut         = Output Disable
     - OutPutPolarity = High Polarity
     - OutPutType     = Open Drain */
  RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if ( HAL_RTC_Init( &RtcHandle ) != HAL_OK )
  {
  
    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
static void RTC_TimeStampConfig( void )
{

  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  
  /*##-3- Configure the Date using BCD format ################################*/
  /* Set Date: Monday January 1st 2000 */
  sdatestructure.Year    = 0x00;
  sdatestructure.Month   = RTC_MONTH_JANUARY;
  sdatestructure.Date    = 0x01;
  sdatestructure.WeekDay = RTC_WEEKDAY_MONDAY;
  
  if ( HAL_RTC_SetDate( &RtcHandle, &sdatestructure, FORMAT_BCD ) != HAL_OK )
  {
  
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-4- Configure the Time using BCD format#################################*/
  /* Set Time: 00:00:00 */
  stimestructure.Hours          = 0x00;
  stimestructure.Minutes        = 0x00;
  stimestructure.Seconds        = 0x00;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BCD ) != HAL_OK )
  {
  
    /* Initialization Error */
    Error_Handler();
  }
}



/**
 * @brief  Configures the current time and date
 * @param  None
 * @retval None
 */
void RTC_TimeRegulate( uint8_t hh, uint8_t mm, uint8_t ss )
{

  RTC_TimeTypeDef stimestructure;
  
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if ( HAL_RTC_SetTime( &RtcHandle, &stimestructure, FORMAT_BIN ) != HAL_OK )
  {
  
    /* Initialization Error */
    Error_Handler();
  }
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
      /* Request step count reset (available only for LSM6DS3 sensor). */
      step_count_reset_request = 1;
    }
  }
    
  /* Pedometer (available only for LSM6DS3 sensor). */
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



/**
 * @brief  Provides SysTick time in milliseconds
 * @param  None
 * @retval SysTick time
 */
uint32_t user_currentTimeGetTick( void )
{

  return HAL_GetTick();
}



/**
 * @brief  Provides a SysTick time elapsed from 'Tick1' start value in milliseconds
 * @param  Tick1  Elapsed time start value
 * @retval SysTick time elapsed
 */
uint32_t user_currentTimeGetElapsedMS( uint32_t Tick1 )
{

  uint32_t Delta, Tick2;
  
  Tick2 = HAL_GetTick();
  Delta = Tick2 - Tick1;
  return Delta;
}



#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.0
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
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
