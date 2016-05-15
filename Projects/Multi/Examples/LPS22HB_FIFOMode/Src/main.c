/**
 ******************************************************************************
 * @file    Projects/Multi/Examples/LPS22HB_FIFOMode/Src/main.c
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
#include <math.h>   /* trunc, pow */
#include "main.h"

/** @addtogroup X_NUCLEO_IKS01A1_Examples
 * @{
 */

/** @addtogroup LPS22HB_FIFO_MODE
 * @{
 */
/* Private typedef -----------------------------------------------------------*/

/* FIFO Interrupt type */
typedef enum {
  
  FIFO_INTERRUPT_THRESHOLD      = 0,
  FIFO_INTERRUPT_FULL           = 1,
  FIFO_INTERRUPT_OVERRUN        = 2
    
} FIFO_INTERRUPT_TYPE;

/* Handle DEMO State Machine */
typedef enum {
  
  STATUS_IDLE                   = 0,
  STATUS_SET_FIFO_MODE          = 1,
  STATUS_FIFO_RUN               = 2,
  STATUS_FIFO_DOWNLOAD          = 3,
  STATUS_SET_BYPASS_MODE        = 4
    
} DEMO_FIFO_STATUS;

/* Private define ------------------------------------------------------------*/

/* FIFO INTERRUPT event type (see FIFO_INTERRUPT_TYPE typedef) */
#define FIFO_INTERRUPT          FIFO_INTERRUPT_THRESHOLD 

/* FIFO size limit */
#define FIFO_WATERMARK          5                

/* It defines LPS22HB ODR (ODR_LOW, ODR_MID_LOW, ODR_MID, ODR_MID_HIGH, ODR_HIGH) */
#define SAMPLE_ODR              ODR_LOW  

#define MAX_FIFO_SAMPLES        32
#define UART_TRANSMIT_TIMEOUT   5000
#define FIFO_INDICATION_DELAY   100


/* Private variables ---------------------------------------------------------*/
/* This variable MUST be volatile because it could change into a ISR */
static volatile uint8_t memsIntDetected = 0;

static char dataOut[256];
static void *LPS22HB_P_0_handle = NULL;
static void *LPS22HB_T_0_handle = NULL;

uint8_t fifoFlag = 0; 
uint8_t samplesInFIFO = 0, oldSamplesInFIFO = 0;
/* This variable MUST be volatile because it could change into a ISR */
static volatile DEMO_FIFO_STATUS demoFifoStatus = STATUS_SET_BYPASS_MODE;

/* Private function prototypes -----------------------------------------------*/
static DrvStatusTypeDef Init_All_Sensors(void);
static DrvStatusTypeDef Enable_All_Sensors(void);

static DrvStatusTypeDef LPS22HB_FIFO_Set_Bypass_Mode(void);
static DrvStatusTypeDef LPS22HB_FIFO_Set_FIFO_Mode(void);

static DrvStatusTypeDef LPS22HB_Read_All_FIFO_Data(void);

static void floatToInt(float in, int32_t *out_int, int32_t *out_dec, int32_t dec_prec);
static DrvStatusTypeDef LPS22HB_FIFO_Demo_Config(void);
static void printConfiguration(void);


/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Main function is to show how to use sensor expansion board to run the
 *         LPS22HB FIFO in FIFO Mode
 * @param  None
 * @retval Integer
 */
int main(void)
{
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
  BSP_LED_Init(LED2);
  
  /* Initialize button */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
#elif (defined (USE_STM32L1XX_NUCLEO))
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
#endif
  
  /* Initialize UART */
  USARTConfig();
  
  if (Init_All_Sensors() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }
  
  if (Enable_All_Sensors() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }
  
  /* Configure LPS22HB Sensor for the DEMO application */
  if (LPS22HB_FIFO_Demo_Config() == COMPONENT_ERROR)
  {
    Error_Handler(__func__);
  }
  
  sprintf(dataOut, "\r\n------ LPS22HB FIFO Mode DEMO ------\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  
  while (1)
  {
    switch(demoFifoStatus)
    {
    case STATUS_IDLE:
      break;
      
    case STATUS_SET_FIFO_MODE:
      printConfiguration();
      
      if (LPS22HB_FIFO_Set_FIFO_Mode() == COMPONENT_ERROR)
      {
        Error_Handler(__func__);
      }
      demoFifoStatus = STATUS_FIFO_RUN;
      break;
      
    case STATUS_FIFO_RUN:
      /* Get num of unread FIFO samples before reading data */
      if (BSP_PRESSURE_FIFO_Get_Num_Of_Samples_Ext(LPS22HB_P_0_handle, &samplesInFIFO) == COMPONENT_ERROR)
      {
        return COMPONENT_ERROR;
      }
      
      /* Print realtime data stored in FIFO when new data is stored */
      if (samplesInFIFO != oldSamplesInFIFO) 
      {
        oldSamplesInFIFO = samplesInFIFO;
        sprintf(dataOut, ".");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
        
        if (memsIntDetected)
          demoFifoStatus = STATUS_FIFO_DOWNLOAD;
      }
      
      /* This IF handles overrun event for the 33th sample */
      if ((samplesInFIFO == MAX_FIFO_SAMPLES) && (memsIntDetected) && (FIFO_INTERRUPT == FIFO_INTERRUPT_OVERRUN))
      {
        sprintf(dataOut, ".(overrun)");
        HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
        demoFifoStatus = STATUS_FIFO_DOWNLOAD;
      }
      
      break;
      
    case STATUS_FIFO_DOWNLOAD:
      /* Error Handler if INT1 and FIFO STATUS Register aren't coherent */
      if (((FIFO_INTERRUPT == FIFO_INTERRUPT_FULL) && (BSP_PRESSURE_FIFO_Get_Full_Status_Ext(LPS22HB_P_0_handle, &fifoFlag) == COMPONENT_ERROR)) ||
          ((FIFO_INTERRUPT == FIFO_INTERRUPT_OVERRUN) && (BSP_PRESSURE_FIFO_Get_Ovr_Status_Ext(LPS22HB_P_0_handle, &fifoFlag) == COMPONENT_ERROR)) ||
            ((FIFO_INTERRUPT == FIFO_INTERRUPT_THRESHOLD) && (BSP_PRESSURE_FIFO_Get_Fth_Status_Ext(LPS22HB_P_0_handle, &fifoFlag) == COMPONENT_ERROR)))
      {
        Error_Handler(__func__);
      }
      
      /* fifoFlag is setted in the previous if. It is 1 only if the flag in 
      LPS22HB status register and the FIFO_INTERRUPT type event selected 
      are coherent */
      if (fifoFlag)
      {
        BSP_LED_On(LED2);
        
        if (LPS22HB_Read_All_FIFO_Data() == COMPONENT_ERROR)
        {
          Error_Handler(__func__);
        }
        
        BSP_LED_Off(LED2);
        
        demoFifoStatus = STATUS_SET_BYPASS_MODE;          
      }
      break;
      
    case STATUS_SET_BYPASS_MODE:
      if (LPS22HB_FIFO_Set_Bypass_Mode() == COMPONENT_ERROR)
      {
        Error_Handler(__func__);
      }
      
      memsIntDetected = 0;
      samplesInFIFO = 0;
      oldSamplesInFIFO = 0;
      demoFifoStatus = STATUS_IDLE;   
      break;        
    }
  }
}



/**
 * @brief  Print current configuration
 * @param  None
 * @retval None
 * @retval None
 */
static void printConfiguration(void)
{
  switch(FIFO_INTERRUPT)
  {
  case FIFO_INTERRUPT_FULL:
    sprintf(dataOut, "FIFO_INTERRUPT_TYPE: FULL\r\n");
    break;
  case FIFO_INTERRUPT_THRESHOLD:
    sprintf(dataOut, "FIFO_INTERRUPT_TYPE: THRESHOLD\r\n");
    break;
  case FIFO_INTERRUPT_OVERRUN:
    sprintf(dataOut, "FIFO_INTERRUPT_TYPE: OVERRUN\r\n");
    break;
  }
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
}




/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Init_All_Sensors(void)
{
  if (BSP_PRESSURE_Init(LPS22HB_P_0, &LPS22HB_P_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (BSP_TEMPERATURE_Init(LPS22HB_T_0, &LPS22HB_T_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Enable all sensors
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef Enable_All_Sensors(void)
{
  if (BSP_PRESSURE_Sensor_Enable(LPS22HB_P_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  if (BSP_TEMPERATURE_Sensor_Enable(LPS22HB_T_0_handle) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}


/**
 * @brief  Configure FIFO
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LPS22HB_FIFO_Demo_Config(void)
{
  /* Set LPS22HB ODR */
  if (BSP_PRESSURE_Set_ODR(LPS22HB_P_0_handle, SAMPLE_ODR) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Reset FIFO MODE register (in order to avoid multiple event interrupt configured) */
  if (BSP_PRESSURE_FIFO_Reset_Interrupt_Ext(LPS22HB_P_0_handle, FIFO_INTERRUPT_THRESHOLD) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }
  if (BSP_PRESSURE_FIFO_Reset_Interrupt_Ext(LPS22HB_P_0_handle, FIFO_INTERRUPT_FULL) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }
  if (BSP_PRESSURE_FIFO_Reset_Interrupt_Ext(LPS22HB_P_0_handle, FIFO_INTERRUPT_OVERRUN) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }
  
  /* Set FIFO INTERRUPT TYPE on INT1 */
  if (BSP_PRESSURE_FIFO_Set_Interrupt_Ext(LPS22HB_P_0_handle, FIFO_INTERRUPT) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Set FIFO WATERMARK. It has effect only if interrupt is configured on FIFO THRESHOLD event */
  if (BSP_PRESSURE_FIFO_Set_Watermark_Level_Ext(LPS22HB_P_0_handle, FIFO_WATERMARK) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }
  
  return COMPONENT_OK;
}



/**
 * @brief  Set FIFO bypass mode
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LPS22HB_FIFO_Set_Bypass_Mode(void)
{
  if (BSP_PRESSURE_FIFO_Set_Mode_Ext(LPS22HB_P_0_handle, LPS22HB_FIFO_BYPASS_MODE) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  sprintf(dataOut, "Press USER button to start the DEMO...\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  return COMPONENT_OK;
}



/**
 * @brief  Set FIFO mode
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LPS22HB_FIFO_Set_FIFO_Mode(void)
{
  sprintf(dataOut, "\r\nLPS22HB starts to store the data into FIFO...\r\n\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  
  HAL_Delay(1000);
  
  /* Set FIFO mode to FIFO */
  if (BSP_PRESSURE_FIFO_Set_Mode_Ext(LPS22HB_P_0_handle, LPS22HB_FIFO_MODE) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  /* Enable FIFO */
  if (BSP_PRESSURE_FIFO_Usage_Ext(LPS22HB_P_0_handle, LPS22HB_ENABLE) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  return COMPONENT_OK;
}



/**
 * @brief  Read all unread FIFO data in cycle
 * @param  None
 * @retval COMPONENT_OK
 * @retval COMPONENT_ERROR
 */
static DrvStatusTypeDef LPS22HB_Read_All_FIFO_Data(void)
{
  uint8_t samplesToRead = 0;
  float pressure = 0;
  float temperature = 0;
  int32_t d1, d2;
  int i;

  /* Get num of unread FIFO samples before reading data */
  if (BSP_PRESSURE_FIFO_Get_Num_Of_Samples_Ext(LPS22HB_P_0_handle, &samplesToRead) == COMPONENT_ERROR)
  {
    return COMPONENT_ERROR;
  }

  sprintf(dataOut, "\r\n%d samples in FIFO.\r\n\nStart to download data from FIFO...\r\n\n", samplesInFIFO);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  
  HAL_Delay(1000);
  sprintf(dataOut, "[DATA ##]  PRESS   TEMP\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  
  for (i = 0; i < samplesToRead; i++)
  {
    /* Read single FIFO data (pressure and temperature) */
    if (BSP_PRESSURE_FIFO_Get_Data_Ext(LPS22HB_P_0_handle, &pressure, &temperature) == COMPONENT_ERROR)
    {
      return COMPONENT_ERROR;
    }

    floatToInt(pressure, &d1, &d2, 2);
    sprintf(dataOut, "[DATA %2d]  %d.%02d", i + 1, (int)d1, (int)d2);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

    floatToInt(temperature, &d1, &d2, 2);
    sprintf(dataOut, "  %d.%02d\r\n", (int)d1, (int)d2);
    HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);
  }

  sprintf(dataOut, "\r\nFIFO download completed.\r\n\r\n");
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  return COMPONENT_OK;
}



/**
 * @brief Splits a float into two integer values
 * @param in the float value as input
 * @param out_int the pointer to the integer part as output
 * @param out_dec the pointer to the decimal part as output
 * @param dec_prec the decimal precision to be used
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
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* User button pressed */
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L0XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
  if(GPIO_Pin == KEY_BUTTON_PIN)
#elif (defined (USE_STM32L1XX_NUCLEO))
  if(GPIO_Pin == USER_BUTTON_PIN)
#endif
  {
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_RESET)
#elif (defined (USE_STM32L1XX_NUCLEO))
    if (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET)
#elif (defined (USE_STM32L0XX_NUCLEO))
    if (BSP_PB_GetState(BUTTON_KEY) == GPIO_PIN_SET)
#endif
    {
      /* Change this variable only if demoFifoStatus is STATUS_IDLE */
      if(demoFifoStatus == STATUS_IDLE)
      {
        demoFifoStatus = STATUS_SET_FIFO_MODE;
      }
    }
  }

  /* FIFO full (available only for LPS22HB sensor) */
  else if (GPIO_Pin == M_INT1_PIN)
  {
    memsIntDetected = 1;
  }

  /* ERROR */
  else
  {
    Error_Handler(__func__);
  }
}



/**
 * @brief  This function is executed in case of error occurrence, turns LED2 ON and ends in infinite loop
 * @param  None
 * @retval None
 */
void Error_Handler(const char *function_name)
{
  sprintf(dataOut, "\r\nError in '%s' function.\r\n", function_name);
  HAL_UART_Transmit(&UartHandle, (uint8_t *)dataOut, strlen(dataOut), UART_TRANSMIT_TIMEOUT);

  while (1)
  {
    BSP_LED_On(LED2);
    HAL_Delay(FIFO_INDICATION_DELAY);
    BSP_LED_Off(LED2);
    HAL_Delay(FIFO_INDICATION_DELAY);
  }
}



#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred
 * @param  file pointer to the source file name
 * @param  line assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
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
