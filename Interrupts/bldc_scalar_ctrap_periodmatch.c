/**
 * @file bldc_scalar_ctrap.c
 * @brief Ctrap event of CCU8
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
 * 2016-05-25:
 *     - Initial version
 * 2016-09-08:
 *     - Updated for sensorless support
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

#include "../bldc_scalar_user_interface.h"
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#include "../ControlModule/bldc_scalar_control_sensorless.h"
#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
#include "../ControlModule/bldc_scalar_inductive_sensing.h"
#endif
#endif

/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

/**
 * @param none
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Trap event mapped to the Phase U (CCU8) event 2 \n
 * PWM is stopped by the hardware when this event is detected.
 * This ISR, stops the motor and clears the state of the control if trap event occurs.
 *
 * This ISR is also mapped to the phase V period match to handle startup algorithm and fast control loop execution
 * rate for sensor-less configuration.
 */

RAM_ATTRIBUTE void Motor0_BLDC_SCALAR_Trap_PeriodMatch_ISR(void)
{
  #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
  int32_t current,error; /* ADC value of direct current in Q14 */
  uint32_t duty_cycle; /* CCU8 duty cycle in Q14 */
  #endif

#if (MOTOR0_BLDC_SCALAR_ENABLE_CTRAP == 1U)
  /* CTRAP event check */
  if(Motor0_BLDC_SCALAR_PWM_BC_GetEvent(Motor0_BLDC_SCALAR_PWM_BC.ccu8_handle_ptr->phase_ptr[0], XMC_CCU8_SLICE_IRQ_ID_TRAP) == true)
  {
    /* Update the error status and stop the motor in case of trap event */
    Motor0_BLDC_SCALAR.error_status |= (uint32_t)1 << (uint32_t)BLDC_SCALAR_EID_CTRAP_ERROR;
    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_ERROR;
    Motor0_BLDC_SCALAR_MotorStop();
  }
#endif /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_CTRAP == 1U) */

  /* Check if phase V period match event is occurred to execute startup algorithm */
  if (Motor0_BLDC_SCALAR_PWM_BC_GetEvent(Motor0_BLDC_SCALAR_PWM_BC.ccu8_handle_ptr->phase_ptr[1], XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH) == true)
  {
    #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
    /******************************STARTUP Algorithm - START **********************************/
    if (Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION)
    {
      if ((Motor0_BLDC_SCALAR_SL.braking_status == BLDC_SCALAR_BRAKING_STATUS_COMPLETED) &&
          (Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status == BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_IN_PROGRESS))
      {
        #if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
        Motor0_BLDC_SCALAR_IndSenseRotorPosIdentification();
        #endif
        /* If startup technique is align and go then rotor pre-alignment function will be called
         * from state machine during this time.
         */
      }
      else
      {
        /* Braking is in progress */
        BLDC_SCALAR_BrakeMotor();
      }
    }
    else if ((Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_TRANSITION) &&
             (Motor0_BLDC_SCALAR_SL.transition_status != BLDC_SCALAR_TRANSITION_STATUS_COMPLETED))
    {
      /* Transition state machine */
      Motor0_BLDC_SCALAR_TransitionStateMachine();

      /* Duty cycle will be controlled by the current control during transition */
      /***************** Current Reading ***************************************/
  #if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U)
      /* Get the IDC Link instantaneous current value */
      Motor0_BLDC_SCALAR_GetCurrentValue(&current);
      Motor0_BLDC_SCALAR.motor_current = Motor0_BLDC_SCALAR.motor_set_direction * current;
  #endif  /* end of #if(MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U) */

  #if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 1U) || (MOTOR0_BLDC_SCALAR_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK == 1U))
      int32_t avg_current;      /* ADC reading of average current in Q14 */
      /* Get the IDC Link average current value */
      Motor0_BLDC_SCALAR_GetAverageCurrentValue(&avg_current);
      Motor0_BLDC_SCALAR.motor_average_current = Motor0_BLDC_SCALAR.motor_set_direction * avg_current;
  #endif /* end of #if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 1U) || (MOTOR0_BLDC_SCALAR_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK == 1U))*/

      /* Current control */
      error = (int32_t)(Motor0_BLDC_SCALAR_StartupCurrentControl.user_current_set) - (int32_t)(current);

      /*Current Control based on PI technique*/
      Motor0_BLDC_SCALAR_PI_Controller(&Motor0_BLDC_SCALAR_StartupCurrentControl_PI,(int32_t)error);
      duty_cycle = Motor0_BLDC_SCALAR_StartupCurrentControl_PI.uk;

      /* max_amplitude is decided by MOTOR0_BLDC_SCALAR_MAX_AMPLITUDE */
      if (duty_cycle > Motor0_BLDC_SCALAR.max_amplitude)
      {
        duty_cycle = Motor0_BLDC_SCALAR.max_amplitude;
      }
      /* min_amplitude is decided by MOTOR0_BLDC_SCALAR_MIN_AMPLITUDE */
      if (duty_cycle < Motor0_BLDC_SCALAR_SL.startup_min_amplitude)
      {
        duty_cycle = Motor0_BLDC_SCALAR_SL.startup_min_amplitude;
      }
      Motor0_BLDC_SCALAR.amplitude = duty_cycle;
      /* Update the CCU8 compare values */
      Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate((uint16_t)Motor0_BLDC_SCALAR.amplitude);
    }
    else
    {
      /* Switched to Normal operation */
  #if(MOTOR0_BLDC_SCALAR_CONTROL_LOOP_EXECUTION_RATE > 1U)
      if(Motor0_BLDC_SCALAR.control_loop_exec_count >= Motor0_BLDC_SCALAR.control_loop_exec_rate)
      {
        Motor0_BLDC_SCALAR_PWM_BC_EnableEvent(Motor0_BLDC_SCALAR_CCU8_PWM_Config.phase_ptr[0U], XMC_CCU8_SLICE_IRQ_ID_ONE_MATCH);
      }
      Motor0_BLDC_SCALAR.control_loop_exec_count++;
  #endif
    }
    /******************************STARTUP Algorithm - END **********************************/
  #endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
  } /* end of if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_ERROR) */
}

/**
 * @}
 */
/**
 * @}
 */
