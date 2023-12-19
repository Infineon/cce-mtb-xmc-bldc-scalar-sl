/**
 * @file ccu4.c
 * @brief CCU4 slices configuration
 * -# CCU4 slice for hall signal blanking time
 * -# CCU4 slice for speed capture is used.
 * -# CCU4 slice is optionally used (Motor0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4) for
 * multi-channel pattern synchronization instead of CCU8 period match to avoid the PWM time delay.
 * @date 2016-09-08
 *
  **********************************************************************************************************************
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
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
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
 * 2016-05-25:
 * - Initial version
 * 2016-09-08:
 *     - Updated for sensorless support
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "ccu4.h"

/**********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/**
 *  This slice is used in timer mode for measuring the time between commutations and
 *  it provides an interrupt for phase advance.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t Motor0_CCU4_Sensorless_commutation_timer =
{
  .timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot            = 1U,
  .shadow_xfer_clear   = 0U,
  .dither_timer_period = 0U,
  .dither_duty_cycle   = 0U,
  .prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable          = 0U,
  .prescaler_initval   = MOTOR0_BLDC_SCALAR_CCU4_PRESCALER,
  .float_limit         = 0U,
  .dither_limit        = 0U,
  .passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation = 0U
};
/****************************Start: Delay Timer Slice *****************************************************************/
#if(MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
/*
 * CCU4 used for providing trigger signal for VADC measurement.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t Motor0_CCU4_Trigger_Config =
{
  .timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot            = 0U,
  .shadow_xfer_clear   = 1U,
  .dither_timer_period = 0U,
  .dither_duty_cycle   = 0U,
  .prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable          = 0U,
  .prescaler_initval   = 0U,
  .float_limit         = 0U,
  .dither_limit        = 0U,
  .passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation = 0U
};

uint16_t motor0_ccu4_period_match = MOTOR0_BLDC_SCALAR_SL_CCU4_TRIGGER;

#endif /*end of #if(MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)*/
/*
 * CCU4 used for speed capture.
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t Motor0_CCU4_Speed_Config =
{
  .timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_CA,
  .monoshot            = 0U,
  .shadow_xfer_clear   = 1U,
  .dither_timer_period = 0U,
  .dither_duty_cycle   = 0U,
  .prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable          = 0U,
  .prescaler_initval   = MOTOR0_BLDC_SCALAR_CCU4_PRESCALER,
  .float_limit         = 15U,
  .dither_limit        = 0U,
  .passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation = 0U
};
#endif /*end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)*/

/****************************Start: Fast Synchronization Slice ***************************/
#if(1U == MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4)

/**
 * CCU4 used for providing synchronization signal for multi channel pattern transfer
 * to the output lines.
 * Compare mode, period value in the range if 2-5 us
 */
XMC_CCU4_SLICE_COMPARE_CONFIG_t Motor0_BLDC_SCALAR_CCU4_FastSynch_Config =
{
  .timer_mode          = XMC_CCU4_SLICE_TIMER_COUNT_MODE_EA,
  .monoshot            = 0U,
  .shadow_xfer_clear   = 1U,
  .dither_timer_period = 0U,
  .dither_duty_cycle   = 0U,
  .prescaler_mode      = XMC_CCU4_SLICE_PRESCALER_MODE_NORMAL,
  .mcm_enable          = 0U,
  .prescaler_initval   = 0U,
  .float_limit         = 0U,
  .dither_limit        = 0U,
  .passive_level       = XMC_CCU4_SLICE_OUTPUT_PASSIVE_LEVEL_LOW,
  .timer_concatenation = 0U
};
#endif
/****************************End: Fast Synchronization Slice ***************************/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
#if(1U == MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4)
/* Fast synchronization slice initialization */
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Initializes synchronization CCU4 slice for fast synchronization of MCM update after HALL event.
 */
void Motor0_BLDC_SCALAR_CCU4_MCMSync_Init(void)
{
  /*Timer initiation*/
  XMC_CCU4_SLICE_CompareInit(MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SLICE, &Motor0_BLDC_SCALAR_CCU4_FastSynch_Config);
  /*Disable single shot mode by setting repeat mode*/
  XMC_CCU4_SLICE_SetTimerRepeatMode(MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SLICE, XMC_CCU4_SLICE_TIMER_REPEAT_MODE_REPEAT);
  /*set period value*/
  XMC_CCU4_SLICE_SetTimerPeriodMatch(MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SLICE,
      (uint16_t)MOTOR0_BLDC_SCALAR_MCM_SYNCTRANSFER_PERIOD);
  /*Enable clock - Idle mode clear*/
  XMC_CCU4_EnableClock(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SLICE_NUM);
  /*Enable shadow transfer*/
  XMC_CCU4_EnableShadowTransfer(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SHADOWTRANSFER);
  XMC_CCU4_SLICE_StartTimer(MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SLICE);
}
#endif  /* end of #if(1U == MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4) */
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/*
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * CCU4 slices initialization for sensorless configuration.
 */
void Motor0_CCU4_SL_Init(void)
{
  /* Enable CCU4 module */
  XMC_CCU4_Init(MOTOR0_BLDC_SCALAR_CCU4_MODULE, XMC_CCU4_SLICE_MCMS_ACTION_TRANSFER_PR_CR);
  /* Enable commutation timer */
  Motor0_BLDC_SCALAR_CCU4_SL_CommutationTimer_Init();
  /*Fast sync slice initialization*/
  #if(1U == MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4)
  Motor0_BLDC_SCALAR_CCU4_MCMSync_Init();
  #endif
  /*Continuous trigger initialization*/
#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
  Motor0_BLDC_SCALAR_CCU4_SL_VADCTrigger_Init();
#endif
  Motor0_BLDC_SCALAR_CCU4_SL_SpeedCapture_Init();


}
#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
/*
 * CCU4 slices initialization for generating trigger for VADC conversion
 */
void Motor0_BLDC_SCALAR_CCU4_SL_VADCTrigger_Init(void)
{
  /*Phase delay slice compare timer initiation*/
  XMC_CCU4_SLICE_CompareInit(MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE, &Motor0_CCU4_Trigger_Config);
  /* Clear IDLE mode.*/
  XMC_CCU4_EnableClock(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE_NUM);

  XMC_CCU4_SLICE_SetTimerPeriodMatch(MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE, motor0_ccu4_period_match);

  /*enable shadow transfer for period value*/
  XMC_CCU4_EnableShadowTransfer(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SHADOWTRANSFER);
  /* Clear Timer Period Match Flag */
  XMC_CCU4_SLICE_ClearEvent(MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  /* Bind the interrupts to, SR2 - Period match(ISR)) */
  XMC_CCU4_SLICE_EnableEvent(MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  /* Set interrupt nodes */
  XMC_CCU4_SLICE_SetInterruptNode(MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE,XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_PM_SR);

}

#endif /* #if(MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)*/
/*
 * CCU4 slices initialization for speed capture.
 */
void Motor0_BLDC_SCALAR_CCU4_SL_SpeedCapture_Init(void)
{
  /*Speed Capture slice compare timer initiation*/
  XMC_CCU4_SLICE_CompareInit(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE, &Motor0_CCU4_Speed_Config);
  /* Clear IDLE mode.*/
  XMC_CCU4_EnableClock(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE_NUM);
  /*Enable floating prescaler unit*/
  XMC_CCU4_SLICE_EnableFloatingPrescaler(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);
  /*Set period value*/
  XMC_CCU4_SLICE_SetTimerPeriodMatch(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE, (uint16_t)0xFFFF);
  /*enable shadow transfer for period value*/
  XMC_CCU4_EnableShadowTransfer(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SHADOWTRANSFER);

}
/*
 * This slice is used as commutation timer. Period match is connected to STALL detection ISR.
 * Compare match is used as phase delay to update next MCM pattern.
 */
void Motor0_BLDC_SCALAR_CCU4_SL_CommutationTimer_Init(void)
{
  /* Clear IDLE mode.*/
  XMC_CCU4_EnableClock(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE_NUM);

  /* Phase delay slice compare timer initiation*/
  XMC_CCU4_SLICE_CompareInit(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, &Motor0_CCU4_Sensorless_commutation_timer);

  /*Phase delay slice period value(phase delay timing) configuration*/
  XMC_CCU4_SLICE_SetTimerPeriodMatch(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, (uint16_t)0xFFFE);
  XMC_CCU4_SLICE_SetTimerCompareMatch(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, (uint16_t)0xFFFF);

  XMC_CCU4_SLICE_EnableEvent(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH);
  XMC_CCU4_SLICE_SetInterruptNode(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE, XMC_CCU4_SLICE_IRQ_ID_PERIOD_MATCH, MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_SR);
  /*enable shadow transfer for period value*/
  XMC_CCU4_EnableShadowTransfer(MOTOR0_BLDC_SCALAR_CCU4_MODULE, MOTOR0_CCU4_COMMUTATION_TIMER_SHADOWTRANSFER);

}

#endif/* end of #if(MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
