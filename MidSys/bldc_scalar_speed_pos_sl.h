/**
 * @file bldc_scalar_speed_pos_sl.h
 * @brief Speed and position interface using sensorless technique. This uses floating prescaler feature of CCU4 for speed capture
 * @date 2016-09-08

 ***********************************************************************************************************************
 * BLDC_SCALAR_CONTROL v1.0.2 - BLDC motor control using block commutation
 * Copyright (c) 2015-2016, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * To improve the quality of the software, users are encouraged to share modifications, enhancements or bug fixes
 * with Infineon Technologies AG (dave@infineon.com).
 ***********************************************************************************************************************
 *
 * Change History
 * --------------
 *
 *
 * 2016-09-08:
 *     - Initial version
 *
 * @endcond
 *
 */
/**
 * @addtogroup BLDC_SCALAR BLDC Motor Control
 * @{
 */

/**
 * @addtogroup MidSys
 * @{
 */
#ifndef BLDC_SCALAR_SPEED_POS_SL_H_
#define BLDC_SCALAR_SPEED_POS_SL_H_

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../MCUInit/ccu4.h"
#include "../MCUInit/posif.h"

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/**********************************************************************************************************************
* MACROS
**********************************************************************************************************************/
/** This defines speed accumulation limit value*/
#define BLDC_SCALAR_SPEED_POS_SL_SPEEDACCUMLIMITCHECK             (5U)
/** Maximum period value of CCU4 slice */
#define BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL                     (0xFFFFU)
/** Capture register number */
#define BLDC_SCALAR_HALL_CAPTURE_REGITSER                         (3U)

/**********************************************************************************************************************
* ENUMS
**********************************************************************************************************************/
/**
 * @brief return value of API
 */
typedef enum BLDC_SCALAR_SPEED_POS_SL_STATUS
{
  BLDC_SCALAR_SPEED_POS_SL_STATUS_SUCCESS = 0U,   /*!< API execution is successful */
  BLDC_SCALAR_SPEED_POS_SL_STATUS_FAILURE        /*!< API execution is failed */

} BLDC_SCALAR_SPEED_POS_SL_STATUS_t;

/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
/**
 * @brief structure to get the position and speed
 */
typedef struct BLDC_SCALAR_SPEED_POS_SL
{
  uint32_t captureval[6];      /*!< captured time value between two correct hall events */
  uint32_t capval;             /*!< Captured time Value */
  uint32_t speedcheck;         /*!< whether motor speed can be calculated */
  uint32_t speedaccum;         /*!< accumulated speed of the motor for 6 samples */
  uint32_t speed_constant;     /*!< constant value used for speed calculation */
  uint8_t index;               /*!< index of an array of hall pattern */
  uint8_t speedindex;          /*!< index of an array of the speed capture variables */
} BLDC_SCALAR_SPEED_POS_SL_t;


/**********************************************************************************************************************
* EXTERN
**********************************************************************************************************************/
extern BLDC_SCALAR_SPEED_POS_SL_t Motor0_BLDC_SCALAR_SPEED_POS_SL;
extern uint32_t BLDC_SCALAR_SPEED_POS_SL_Cap_Array[16];

#ifdef __cplusplus
   extern "C" {
#endif
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/

/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Initializes the peripheral required for position and speed identification.
 * This initializes the POSIF and CCU4 module.
 */
void Motor0_BLDC_SCALAR_SPEED_POS_SL_Init(void);

/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 *  Starts CCU4 and POSIF modules required for position and speed detection.
 *  It also reinitializes the speed calculation related variables.
 */
void Motor0_BLDC_SCALAR_SPEED_POS_SL_Start(void);

/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 *  Stops CCU4 and POSIF modules required for position and speed detection.
 */
void Motor0_BLDC_SCALAR_SPEED_POS_SL_Stop(void);

/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 *  Starts POSIF module.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_POSIF_Start(void)
{
  XMC_POSIF_Start(MOTOR0_BLDC_SCALAR_POSIF_MODULE);
}
/**
 * @return none <br>
 *
 * \par<b>Description:</b><br>
 * Resets variables related to speed calculation.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_ClearSpeedFilter(void)
{
  Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.speedcheck = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[0] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[1] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[2] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[3] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[4] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[5] = 0U;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.capval = Motor0_BLDC_SCALAR_SPEED_POS_SL.speed_constant;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.speedaccum = 0U;
}

/**
 * @param capval captured value calculated from captured timer and prescaler
 * @return None
 *  <br>
 *
 * \par<b>Description:</b><br>
 *  Calculates the captured time from timer value and current prescaler value in capture register
 *
 * \par<b>Execution Time:</b><br>
 * <b>1.7 uSec </b>using O3 optimization level
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_ReadCaptureValue(uint32_t *capval, uint32_t *psc)
{
  uint32_t temp_capval; /* capture register value */
  uint32_t curr_psc;    /* prescaler value */

  temp_capval = (uint32_t)XMC_CCU4_SLICE_GetTimerValue(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);

  curr_psc = ((uint32_t)(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE->FPC & CCU4_CC4_FPC_PVAL_Msk) >> (uint32_t)CCU4_CC4_FPC_PVAL_Pos);
  Motor0_BLDC_SCALAR_SPEED_POS_SL.capval = (uint32_t)(BLDC_SCALAR_SPEED_POS_SL_Cap_Array[(curr_psc - MOTOR0_BLDC_SCALAR_CCU4_PRESCALER)] +
            (((uint32_t)1 << (curr_psc - MOTOR0_BLDC_SCALAR_CCU4_PRESCALER)) * (uint32_t)temp_capval));
  *capval = (uint32_t)Motor0_BLDC_SCALAR_SPEED_POS_SL.capval;
  *psc    = (uint32_t)curr_psc;
}

/**
 * @param capval time between two hall events (60 degrees)
 * @param speed Calculated electrical speed in RPM
 * @return None
 *  <br>
 *
 * \par<b>Description:</b><br>
 * Calculates the speed based upon the captured time value between two zero crossings.
 * It uses the floating prescaler for better resolution and low speed value.
 *
 */
/*This function will calculate the speed based upon the captured time values.*/
RAM_ATTRIBUTE __STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_SpeedCalculation(uint32_t capval, uint32_t* speed)
{
  /* Moving average to calculate the speed */
  Motor0_BLDC_SCALAR_SPEED_POS_SL.speedaccum -= Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex];
  Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex] = capval;
  Motor0_BLDC_SCALAR_SPEED_POS_SL.speedaccum += Motor0_BLDC_SCALAR_SPEED_POS_SL.captureval[Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex];

  Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex++;

  /* Speed calculation starts after 6 hall events */
  if (Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex > BLDC_SCALAR_SPEED_POS_SL_SPEEDACCUMLIMITCHECK)
  {
    Motor0_BLDC_SCALAR_SPEED_POS_SL.speedindex = 0U;
    Motor0_BLDC_SCALAR_SPEED_POS_SL.speedcheck = 1U;
  }

  if (Motor0_BLDC_SCALAR_SPEED_POS_SL.speedcheck == 1U)
  {
    if (Motor0_BLDC_SCALAR_SPEED_POS_SL.speedaccum > 0U)
    {
      __disable_irq();
      *speed = (Motor0_BLDC_SCALAR_SPEED_POS_SL.speed_constant) / (Motor0_BLDC_SCALAR_SPEED_POS_SL.speedaccum);
      __enable_irq();
    }
  }
  else
  {
    *speed = 0U;
  }
}
/**
 * @param pattern The 16b multi-channel pattern [0-65535]
 * @retval None
 *
 * \par<b>Description</b><br>
 * Configures \a MCSM register with Multi-Channel Pattern.\n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(uint16_t pattern)
{
  XMC_POSIF_MCM_SetMultiChannelPattern(MOTOR0_BLDC_SCALAR_POSIF_MODULE, pattern);
}
/**
 * @retval None <br>
 *
 * \par<b>Description</b><br>
 * Performs shadow transfer of the Multi-Channel Pattern register by configuring.\n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern(void)
{
  XMC_POSIF_MCM_UpdateMultiChannelPattern(MOTOR0_BLDC_SCALAR_POSIF_MODULE);
}
/**
 * @retval uint16_t Returns configured multi channel pattern <br>
 *
 * \par<b>Description</b><br>
 * Returns configured multi channel pattern of \a peripheral. \n
 */
__STATIC_INLINE uint16_t Motor0_BLDC_SCALAR_SPEED_POS_SL_GetMultiChannelPattern(void)
{
  uint16_t pattern;
  pattern = XMC_POSIF_MCM_GetMultiChannelPattern(MOTOR0_BLDC_SCALAR_POSIF_MODULE);
  return (pattern);
}

/**
 * @param None
 * @retval uint8_t Returns multi-channel shadow transfer event status <br>
 *
 * \par<b>Description</b><br>
 * Returns multi-channel shadow transfer event status. \n
 */
__STATIC_INLINE uint8_t Motor0_BLDC_SCALAR_SPEED_POS_SL_GetMCMTxEventStatus(void)
{
  uint8_t evnt_status;
  evnt_status = XMC_POSIF_GetEventStatus(MOTOR0_BLDC_SCALAR_POSIF_MODULE, XMC_POSIF_IRQ_EVENT_MCP_SHADOW_TRANSFER);
  return (evnt_status);
}

/**
 * @param None
 * @retval None
 *
 * \par<b>Description</b><br>
 * Clears multi-channel shadow transfer event status. \n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_ClearMCMTxEvent(void)
{
  XMC_POSIF_ClearEvent(MOTOR0_BLDC_SCALAR_POSIF_MODULE, XMC_POSIF_IRQ_EVENT_MCP_SHADOW_TRANSFER);
}

/**
 * @param None
 * @retval None
 *
 * \par<b>Description</b><br>
 * Clears multi-channel shadow transfer event status. \n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMCMTxEvent(void)
{
  XMC_POSIF_SetEvent(MOTOR0_BLDC_SCALAR_POSIF_MODULE, XMC_POSIF_IRQ_EVENT_MCP_SHADOW_TRANSFER);
}
/**
 * @retval uint16_t Returns configured multi channel pattern present in shadow transfer register <br>
 *
 * \par<b>Description</b><br>
 * Returns configured multi channel pattern in shadow register of \a peripheral. \n
 */
__STATIC_INLINE uint16_t Motor0_BLDC_SCALAR_SPEED_POS_SL_GetShadowMultiChannelPattern(void)
{
  uint16_t pattern;
  pattern = XMC_POSIF_MCM_GetShadowMultiChannelPattern(MOTOR0_BLDC_SCALAR_POSIF_MODULE);
  return(pattern);
}
#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
/**
 * @retval uint16_t None <br>
 *
 * \par<b>Description</b><br>
 * Starts ccu4 trigger slice timer
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_StartTriggerTimer(void)
{
  XMC_CCU4_SLICE_StartTimer(MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE);
}
#endif
/**
 * @retval None <br>
 *
 * \par<b>Description</b><br>
 * Reset the capture timer prescaler value.\n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_ResetCaptureTimePrescaler(void)
{
  /*
   * Stop timer and prescaler.
   * initialize timer and prescaler value and restarts the timer
   */
  XMC_CCU4_SLICE_StopTimer(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);
  XMC_CCU4_StopPrescaler(MOTOR0_BLDC_SCALAR_CCU4_MODULE);
  XMC_CCU4_SLICE_SetTimerValue(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE, 0U);
  XMC_CCU4_SLICE_SetPrescaler(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE, MOTOR0_BLDC_SCALAR_CCU4_PRESCALER);
  XMC_CCU4_StartPrescaler(MOTOR0_BLDC_SCALAR_CCU4_MODULE);
  XMC_CCU4_SLICE_StartTimer(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);
}

/**
 * @retval None <br>
 *
 * \par<b>Description</b><br>
 * Updates commutation timer period value and prescaler value.\n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateCommutationTimerPeriod(uint32_t prescaler, uint32_t period_count)
{
  /*
   * Stop timer and prescalar.
   * initialize prescaler value, update period match and restart the timer
   */
  XMC_CCU4_SLICE_StopTimer(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE);
  XMC_CCU4_StopPrescaler(MOTOR0_BLDC_SCALAR_CCU4_MODULE);
  XMC_CCU4_SLICE_SetTimerValue(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, 0U);
  XMC_CCU4_SLICE_SetPrescaler(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, prescaler);
  XMC_CCU4_StartPrescaler(MOTOR0_BLDC_SCALAR_CCU4_MODULE);
  XMC_CCU4_SLICE_SetTimerPeriodMatch(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, period_count);
  /*enable shadow transfer for period value*/
  XMC_CCU4_EnableShadowTransfer(MOTOR0_BLDC_SCALAR_CCU4_MODULE,MOTOR0_CCU4_COMMUTATION_TIMER_SHADOWTRANSFER);
  XMC_CCU4_SLICE_StartTimer(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE);
}

#endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif
#endif /* BLDC_SCALAR_SPEED_POS_SL_H_ */

