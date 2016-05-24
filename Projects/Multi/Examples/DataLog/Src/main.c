/**
  ******************************************************************************
  * @file    Projects/Multi/Examples/DataLog/Src/main.c
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
#include "com.h"
#include <string.h> // strlen
#include <stdio.h>  // sprintf
#include <math.h>   // trunc
#include "DemoSerial.h"

/** @addtogroup X_NUCLEO_IKS01A1_Examples
  * @{
  */

/** @addtogroup DATALOG
  * @{
  */

/* Extern variables ----------------------------------------------------------*/
extern volatile uint8_t DataLoggerActive; /*!< DataLogger Flag */
extern UART_HandleTypeDef UartHandle;     /*!< UART HANDLE */

/* Private variables ---------------------------------------------------------*/
char dataOut[256];                       /*!< DataOut Frame */
RTC_HandleTypeDef RtcHandle;             /*!< RTC HANDLE */
volatile uint32_t Sensors_Enabled = 0;   /*!< Enable Sensor Flag */
volatile uint32_t DataTxPeriod = 1000;   /*!< TX DATA Period */
volatile uint8_t AutoInit = 0;           /*!< Auto Init */
SensorAxes_t ACC_Value;                  /*!< Acceleration Value */
SensorAxes_t GYR_Value;                  /*!< Gyroscope Value */
SensorAxes_t MAG_Value;                  /*!< Magnetometer Value */
float PRESSURE_Value;           /*!< Pressure Value */
float HUMIDITY_Value;           /*!< Humidity Value */
float TEMPERATURE_Value;        /*!< Temperature Value */
volatile uint32_t Int_Current_Time1 = 0; /*!< Int_Current_Time1 Value */
volatile uint32_t Int_Current_Time2 = 0; /*!< Int_Current_Time2 Value */
void *ACCELERO_handle = NULL;
void *GYRO_handle = NULL;
void *MAGNETO_handle = NULL;
void *HUMIDITY_handle = NULL;
void *TEMPERATURE_handle = NULL;
void *PRESSURE_handle = NULL;


/* Private function prototypes -----------------------------------------------*/
static void RTC_Config(void);
static void RTC_TimeStampConfig(void);

static void initializeAllSensors(void);
static void enableAllSensors(void);
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
static void RTC_Handler(TMsg *Msg);
static void Accelero_Sensor_Handler(TMsg *Msg);
static void Gyro_Sensor_Handler(TMsg *Msg);
static void Magneto_Sensor_Handler(TMsg *Msg);

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use X_NUCLEO_IKS01A1 expansion board to send data from a Nucleo board
 *         using UART to a connected PC or Desktop and display it on generic applications like
 *         TeraTerm and specific application like Sensors_DataLog, which is developed by STMicroelectronics
 *         and provided with this package.
 *
 *         After connection has been established:
 *         - the user can view the data from various on-board environment sensors like Temperature, Humidity, and Pressure
 *         - the user can also view data from various on-board MEMS sensors as well like Accelerometer, Gyrometer, and Magnetometer
 *         - the user can also visualize this data as graphs using Sensors_DataLog application provided with this package
 * @param  None
 * @retval Integer
 */
int main(void)
{
  TMsg Msg;
  
  /* STM32F4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
   */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();
  
  /* Initialize LEDs */
  BSP_LED_Init(LED2);
  BSP_LED_Off(LED2);
  
  /* Initialize Button */
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
  initializeAllSensors();
  enableAllSensors();
  
  while(1)
  {
  
    if (UART_ReceivedMSG((TMsg*) &Msg))
    {
      if (Msg.Data[0] == DEV_ADDR)
      {
        HandleMSG((TMsg*) &Msg);
        if ( DataLoggerActive )
        {
          AutoInit = 0;
        }
      }
    }
    
    RTC_Handler(&Msg);
    
    Accelero_Sensor_Handler(&Msg);
    
    Gyro_Sensor_Handler(&Msg);
    
    Magneto_Sensor_Handler(&Msg);
    
    if ( DataLoggerActive || AutoInit )
    {
      BSP_LED_Toggle(LED2);
    }
    
    else
    {
      BSP_LED_Off(LED2);
    }
    
    if(DataLoggerActive)
    {
      INIT_STREAMING_HEADER(&Msg);
      Msg.Len = STREAMING_MSG_LENGTH;
      UART_SendMsg(&Msg);
      HAL_Delay(DataTxPeriod);
    }
    
    if ( AutoInit )
    {
      HAL_Delay(500);
    }
  }
}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors(void)
{
  /* Try to use LSM6DS3 DIL24 if present, otherwise use LSM6DS0 on board */
  BSP_ACCELERO_Init( ACCELERO_SENSORS_AUTO, &ACCELERO_handle );
  /* Try to use LSM6DS3 if present, otherwise use LSM6DS0 */
  BSP_GYRO_Init( GYRO_SENSORS_AUTO, &GYRO_handle );
  /* Force to use LIS3MDL */
  BSP_MAGNETO_Init( LIS3MDL_0, &MAGNETO_handle );
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors(void)
{
  BSP_ACCELERO_Sensor_Enable( ACCELERO_handle );
  BSP_GYRO_Sensor_Enable( GYRO_handle );
  BSP_MAGNETO_Sensor_Enable( MAGNETO_handle );
}

/**
 * @brief  Splits a float into two integer values.
 * @param  in the float value as input
 * @param  out_int the pointer to the integer part as output
 * @param  out_dec the pointer to the decimal part as output
 * @param  dec_prec the decimal precision to be used
 * @retval None
 */
static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec)
{
  *out_int = (int32_t)in;
  if(in >= 0.0f)
  {
    in = in - (float)(*out_int);
  }
  else
  {
    in = (float)(*out_int) - in;
  }
  *out_dec = (int32_t)trunc(in * pow(10, dec_prec));
}

/**
 * @brief  Handles the time+date getting/sending
 * @param  Msg the time+date part of the stream
 * @retval None
 */
static void RTC_Handler(TMsg *Msg)
{
  uint8_t subSec = 0;
  RTC_DateTypeDef sdatestructureget;
  RTC_TimeTypeDef stimestructure;
  
  if(DataLoggerActive || AutoInit)
  {
    HAL_RTC_GetTime(&RtcHandle, &stimestructure, FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructureget, FORMAT_BIN);
    subSec = ((((((int) RTC_SYNCH_PREDIV) - ((int) stimestructure.SubSeconds)) * 100) / (RTC_SYNCH_PREDIV + 1)) & 0xff);
  }
  
  if(DataLoggerActive)
  {
    Msg->Data[3] = (uint8_t)stimestructure.Hours;
    Msg->Data[4] = (uint8_t)stimestructure.Minutes;
    Msg->Data[5] = (uint8_t)stimestructure.Seconds;
    Msg->Data[6] = subSec;
  }
  
  else if(AutoInit)
  {
    sprintf(dataOut, "TimeStamp: %d:%d:%d.%d\n", stimestructure.Hours, stimestructure.Minutes, stimestructure.Seconds, subSec);
            
    HAL_UART_Transmit(&UartHandle, (uint8_t*)dataOut, strlen(dataOut), 5000);
  }
}


/**
 * @brief  Handles the ACCELERO axes data getting/sending
 * @param  Msg the ACCELERO part of the stream
 * @retval None
 */
static void Accelero_Sensor_Handler(TMsg *Msg)
{
  int32_t data[6];
  uint8_t status = 0;
  
  if(BSP_ACCELERO_IsInitialized(ACCELERO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_ACCELERO_Get_Axes(ACCELERO_handle, &ACC_Value);
    
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & ACCELEROMETER_SENSOR)
      {
        Serialize_s32(&Msg->Data[15], ACC_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[19], ACC_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[23], ACC_Value.AXIS_Z, 4);
      }
    }
    
    else if ( AutoInit )
    {
      data[0] = ACC_Value.AXIS_X;
      data[1] = ACC_Value.AXIS_Y;
      data[2] = ACC_Value.AXIS_Z;
      
      sprintf(dataOut, "ACC_X: %d, ACC_Y: %d, ACC_Z: %d\n", (int)data[0], (int)data[1], (int)data[2]);
      HAL_UART_Transmit(&UartHandle, (uint8_t*)dataOut, strlen(dataOut), 5000);
    }
  }
}


/**
 * @brief  Handles the GYRO axes data getting/sending
 * @param  Msg the GYRO part of the stream
 * @retval None
 */
static void Gyro_Sensor_Handler(TMsg *Msg)
{
  int32_t data[6];
  uint8_t status = 0;
  
  if(BSP_GYRO_IsInitialized(GYRO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_GYRO_Get_Axes(GYRO_handle, &GYR_Value);
    
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & GYROSCOPE_SENSOR)
      {
        Serialize_s32(&Msg->Data[27], GYR_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[31], GYR_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[35], GYR_Value.AXIS_Z, 4);
      }
    }
    
    else if ( AutoInit )
    {
      data[3] = GYR_Value.AXIS_X;
      data[4] = GYR_Value.AXIS_Y;
      data[5] = GYR_Value.AXIS_Z;
      
      sprintf(dataOut, "GYR_X: %d, GYR_Y: %d, GYR_Z: %d\n", (int)data[3], (int)data[4], (int)data[5]);
      HAL_UART_Transmit(&UartHandle, (uint8_t*)dataOut, strlen(dataOut), 5000);
    }
  }
}


/**
 * @brief  Handles the MAGNETO axes data getting/sending
 * @param  Msg the MAGNETO part of the stream
 * @retval None
 */
static void Magneto_Sensor_Handler(TMsg *Msg)
{
  int32_t data[3];
  uint8_t status = 0;
  
  if(BSP_MAGNETO_IsInitialized(MAGNETO_handle, &status) == COMPONENT_OK && status == 1)
  {
    BSP_MAGNETO_Get_Axes(MAGNETO_handle, &MAG_Value);
    
    if ( DataLoggerActive )
    {
      if(Sensors_Enabled & MAGNETIC_SENSOR)
      {
        Serialize_s32(&Msg->Data[39], (int32_t)MAG_Value.AXIS_X, 4);
        Serialize_s32(&Msg->Data[43], (int32_t)MAG_Value.AXIS_Y, 4);
        Serialize_s32(&Msg->Data[47], (int32_t)MAG_Value.AXIS_Z, 4);
      }
    }
    
    else if ( AutoInit )
    {
      data[0] = MAG_Value.AXIS_X;
      data[1] = MAG_Value.AXIS_Y;
      data[2] = MAG_Value.AXIS_Z;
      
      sprintf(dataOut, "MAG_X: %d, MAG_Y: %d, MAG_Z: %d\n", (int)data[0], (int)data[1], (int)data[2]);
      HAL_UART_Transmit(&UartHandle, (uint8_t*)dataOut, strlen(dataOut), 5000);
    }
  }
}



/**
 * @brief  Configures the RTC
 * @param  None
 * @retval None
 */
static void RTC_Config(void)
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
  RtcHandle.Init.HourFormat = RTC_HOURFORMAT_12;
  RtcHandle.Init.AsynchPrediv = RTC_ASYNCH_PREDIV;
  RtcHandle.Init.SynchPrediv = RTC_SYNCH_PREDIV;
  RtcHandle.Init.OutPut = RTC_OUTPUT_DISABLE;
  RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  RtcHandle.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  
  if(HAL_RTC_Init(&RtcHandle) != HAL_OK)
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
static void RTC_TimeStampConfig(void)
{
  RTC_DateTypeDef sdatestructure;
  RTC_TimeTypeDef stimestructure;
  
  /*##-3- Configure the Date #################################################*/
  /* Set Date: Tuesday February 18th 2014 */
  sdatestructure.Year = 0x14;
  sdatestructure.Month = RTC_MONTH_FEBRUARY;
  sdatestructure.Date = 0x18;
  sdatestructure.WeekDay = RTC_WEEKDAY_TUESDAY;
  
  if(HAL_RTC_SetDate(&RtcHandle, &sdatestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /*##-4- Configure the Time #################################################*/
  /* Set Time: 08:10:00 */
  stimestructure.Hours = 0x08;
  stimestructure.Minutes = 0x10;
  stimestructure.Seconds = 0x00;
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BCD) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin the pin connected to EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  if(GPIO_Pin == KEY_BUTTON_PIN)
#endif
  
#if (defined (USE_STM32L1XX_NUCLEO))
  if(GPIO_Pin == USER_BUTTON_PIN)
#endif
  {
    /* Manage software debouncing*/
    int doOperation = 0;
      
    if(Int_Current_Time1 == 0 && Int_Current_Time2 == 0)
    {
      Int_Current_Time1 = user_currentTimeGetTick();
      doOperation = 1;
    }
    else
    {
      int i2;
      Int_Current_Time2 = user_currentTimeGetTick();
      i2 = Int_Current_Time2;
        
      /* If I receive a button interrupt after more than 300 ms from the first one I get it, otherwise I discard it */
      if((i2 - Int_Current_Time1)  > 300)
      {
        Int_Current_Time1 = Int_Current_Time2;
        doOperation = 1;
      }
    }
      
    if(doOperation)
    {
      if ( DataLoggerActive )
      {
        AutoInit = 0;                       // always off
      }
      else
      {
        AutoInit = ( AutoInit ) ? 0 : 1;    // toggle on each button pressed
      }
    }
  }
}

/**
 * @brief  Configures the current time and date
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{

  RTC_TimeTypeDef stimestructure;
  
  stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
  stimestructure.Hours = hh;
  stimestructure.Minutes = mm;
  stimestructure.Seconds = ss;
  stimestructure.SubSeconds = 0;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;
  
  if(HAL_RTC_SetTime(&RtcHandle, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
  while(1)
  {}
}

/**
 * @brief  Get the current tick value in millisecond
 * @param  None
 * @retval The tick value
 */
uint32_t user_currentTimeGetTick(void)
{
  return HAL_GetTick();
}

/**
 * @brief  Get the delta tick value in millisecond from Tick1 to the current tick
 * @param  Tick1 the reference tick used to compute the delta
 * @retval The delta tick value
 */
uint32_t user_currentTimeGetElapsedMS(uint32_t Tick1)
{
  volatile uint32_t Delta, Tick2;
  
  Tick2 = HAL_GetTick();
  
  /* Capture computation */
  Delta = Tick2 - Tick1;
  return Delta;
}


#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number where the assert_param error has occurred
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
