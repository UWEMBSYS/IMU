/**
  *******************************************************************************
  * @file    Projects/Multi/Examples/DataLog/Inc/Serial_CMD.h
  * @author  CL
  * @version V2.1.0
  * @date    4-April-2016
  * @brief   This file contains serial commands code
  *******************************************************************************
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
  ********************************************************************************
  */



/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERIAL_CMD_H
#define __SERIAL_CMD_H

/* Exported constants --------------------------------------------------------*/

#define DATALOG_MODE         0
#define DATALOG_FUSION_MODE  1
#define DATALOG_AR_MODE      2
#define DATALOG_CP_MODE      3
#define DATALOG_GR_MODE      4


/**********  GENERIC  CMD  (0x00 - 0x0F)  **********/
#define CMD_Ping                                                                0x01
#define CMD_Read_PresString                                                     0x02
#define CMD_NACK                                                                0x03
#define CMD_CheckModeSupport                                                    0x04
#define CMD_UploadAR                                                            0x05
#define CMD_UploadCP                                                            0x06
#define CMD_Start_Data_Streaming                                                0x08
#define CMD_Stop_Data_Streaming                                                 0x09
#define CMD_StartDemo                                                           0x0A
#define CMD_Sleep_Sec                                                           0x0B
#define CMD_Set_DateTime                                                        0x0C
#define CMD_Get_DateTime                                                        0x0D
#define CMD_Enter_DFU_Mode                                                      0x0E
#define CMD_Reset                                                               0x0F

#define CMD_Reply_Add                                                           0x80

/****************************************************/


/******** ENVIRONMENTAL  CMD  (0x60 - 0x6F)  ********/

#define CMD_LPS25HB_Init                                                        0x60
#define CMD_LPS25HB_Read                                                        0x61
#define CMD_HTS221_Init                                                         0x62
#define CMD_HTS221_Read                                                         0x63

/****************************************************/


/******** INERTIAL  CMD  (0x70 - 0x/7F)  ********/

#define CMD_LSM9DS1_Init                                                        0x70
#define CMD_LSM9DS1_9AXES_Read                                                  0x71
#define CMD_LSM9DS1_AHRS_Read                                                   0x72
#define CMD_LSM9DS1_ACC_Read                                                    0x73
#define CMD_LSM9DS1_GYR_Read                                                    0x74
#define CMD_LSM9DS1_MAG_Read                                                    0x75
#define CMD_LSM6DSO_Init                                                        0x76
#define CMD_LSM6DSO_ACC_Read                                                    0x77
#define CMD_LSM6DSO_GYR_Read                                                    0x78
#define CMD_LSM6DS0_6AXES_Read                                                  0x79
#define CMD_LIS3MDL_Init                                                        0x7A
#define CMD_LIS3MDL_Read                                                        0x7B
#define CMD_SF_Init                                                             0x7C
#define CMD_SF_Data                                                             0x7D

/****************************************************/

#endif /* __SERIAL_CMD_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
