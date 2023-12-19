/**
 * @file bldc_scalar_bemf_zero_cross.c
 * @brief VADC channel event is used to detect the BEMF zero cross point.
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
 * 2016-09-08:
 *     - Initial version
 *
 */
/**
 * @addtogroup BLDC_SCALAR BLDC Motor Control
 * @{
 */

/**
 * @addtogroup Interrupt Interrupts
 * @brief  Interrupt Service Routines  <br>
 * @{
 */
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../ControlModule/bldc_scalar_control_scheme.h"

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#include "../ControlModule/bldc_scalar_control_sensorless.h"
#endif

/**********************************************************************************************************************
* MACROS
**********************************************************************************************************************/

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/**
 * @param None
 * @return none <br>
 *
 * \par<b>Description:</b><br>
 * This is VADC channel event configured for in-bound event with global boundary \n
 * 1. Zero cross counter to avoid false zero cross point \n
 * 2. Speed calculation and commutation  \n
 * -# Read the time between two zero cross events from CCU4 speed slice and restart timer
 * -# Start commutation timer with (PR = time / 2) i.e time for 30 degrees
 * -# Speed calculation
 * 3. VADC configurations \n
 * -# Enable BEMF VADC channel to be measured
 * -# Configure boundaries based on rising or falling edge
 */

RAM_ATTRIBUTE void Motor0_BLDC_SCALAR_BEMF_ZeroCross_ISR(void)
{
  uint32_t prescaler;                       /* Pre-scaler value at the time of capture */
  uint32_t time_30degree;                   /* time for next commutation */
  uint32_t capval;                          /* Time between two zero cross events */
  uint32_t single_trig_compensation = 0U;   /* Compensation time between two zero cross events */
  uint32_t speed = 0U;                      /* electrical speed of the motor */
  uint16_t mcmval;                          /*current multi-channel pattern */
  uint16_t mcmvals;                         /*shadow multi-channel pattern */
  uint8_t direction = (uint8_t)Motor0_BLDC_SCALAR.motor_set_direction & 8U;  /* intended direction */

  /* Check if MCM shadow transfer is over */
  if (Motor0_BLDC_SCALAR_SPEED_POS_SL_GetMCMTxEventStatus() == 1U)
  {
    #if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
    if (Motor0_BLDC_SCALAR_SL.demagnetization_skip_pwm_counter >= Motor0_BLDC_SCALAR_SL.demagnetization_skip_pwm_count)
    {
    #endif
      Motor0_BLDC_SCALAR_SL.bemf_zero_cross_counter++;

      if (Motor0_BLDC_SCALAR_SL.bemf_zero_cross_counter >= 2U)
      {
        Motor0_BLDC_SCALAR_SL.bemf_zero_cross_counter = 0U;

        /* Multi-channel update */
        Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_NextHallPat[Motor0_BLDC_SCALAR_SL.rotor_pos + direction];

        /* Load the next MCM pattern based upon the sensed position and direction */
        Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(Motor0_BLDC_SCALAR_OutPat[Motor0_BLDC_SCALAR_SL.rotor_pos + direction]);
        /*Get the shadow multi-channel pattern value and apply the PWM modulation.*/
        mcmval = (uint16_t)Motor0_BLDC_SCALAR_SPEED_POS_SL_GetMultiChannelPattern();
        mcmvals = (uint16_t)Motor0_BLDC_SCALAR_SPEED_POS_SL_GetShadowMultiChannelPattern();
        Motor0_BLDC_SCALAR_PWM_BC.shadow_modulation_ptr(mcmval, mcmvals);
        Motor0_BLDC_SCALAR_SPEED_POS_SL_ClearMCMTxEvent();

        /* Get the time between two zero cross events */
        Motor0_BLDC_SCALAR_SPEED_POS_SL_ReadCaptureValue(&capval, &prescaler);

        #if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER)
        /* Skip the compensation for first zero cross event */
        single_trig_compensation = Motor0_BLDC_SCALAR_SL.single_trigger_compensation  >> (prescaler - MOTOR0_BLDC_SCALAR_CCU4_PRESCALER);
        #endif /* end of #if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER)*/

        /* Calculate time for 30 degrees and start the commutation timer */
        time_30degree = (((capval >> (uint32_t)1) >> (prescaler - MOTOR0_BLDC_SCALAR_CCU4_PRESCALER)) - single_trig_compensation);
        Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateCommutationTimerPeriod(prescaler, time_30degree);

        /* Restart capture timer */
        Motor0_BLDC_SCALAR_SPEED_POS_SL_ResetCaptureTimePrescaler();

        /* Speed calculation */
        Motor0_BLDC_SCALAR_SPEED_POS_SL_SpeedCalculation(capval, &speed);
        Motor0_BLDC_SCALAR.motor_speed = ((Motor0_BLDC_SCALAR.actual_motor_direction * (int32_t)speed *
            (int32_t)Motor0_BLDC_SCALAR.speed_mech_scale) >> BLDC_SCALAR_SPEED_SCALE_RES);

        /* Remove all the phase voltage channels from scan request source */
        #if (VADC_SCAN_3PH_SAME_GROUP == 1U)
        Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, 0U);
        #else
        Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(VADC_G0, 0U);
        Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(VADC_G1, 0U);
        #endif

        /* Add open phase channel in scan request source for conversion for next 60 degrees */
        Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(Motor0_BLDC_SCALAR_SL.open_phase_grp[Motor0_BLDC_SCALAR_SL.rotor_pos + direction],
                                 (uint32_t)Motor0_BLDC_SCALAR_SL.open_phase[Motor0_BLDC_SCALAR_SL.rotor_pos + direction]);

        /* Clear the Channel Event interrupt flags */
        Motor0_BLDC_SCALAR_VOLT_3PHASE_ChannelClearEvent();
      } /* end of if (Motor0_BLDC_SCALAR_SL.bemf_zero_cross_counter >= 2U) */
    #if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
    }/* end of if (demag_skip_counter >= Motor0_BLDC_SCALAR_SL.demagnetization_skip_pwm_count) */
    #endif
  }/* end of if (mcmval == mcmvals) */
  else
  {
    Motor0_BLDC_SCALAR_SL.bemf_zero_cross_counter = 0U;
  }
}

#endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
/**
 * @}
 */
/**
 * @}
 */
