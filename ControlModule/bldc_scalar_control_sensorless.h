/**
 * @file bldc_scalar_control_sensorless.h
 * @brief Hall sensor feedback control algorithm
 * @date 2016-09-08
 * @cond
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
 * @endcond
 *
 */
#ifndef BLDC_SCALAR_CTRL_SENSORLESS_H
#define BLDC_SCALAR_CTRL_SENSORLESS_H

/**
 * @addtogroup BLDC_SCALAR BLDC Motor Control
 * @{
 */

/**
 * @addtogroup ControlModule
 * @brief  Control algorithm for BLDC motor control <br>
 * @{
 */


/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

#include "bldc_scalar_control_scheme.h"

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/***********************************************************************************************************************
 * MACRO
 **********************************************************************************************************************/
/** BEMF slope is rising */
#define MOTOR0_BLDC_SCALAR_BEMF_RISE                         (1U)
/** BEMF slope is falling */
#define MOTOR0_BLDC_SCALAR_BEMF_FALL                         (0U)
/** Boundary offset for VADC limit check */
#define MOTOR0_BLDC_SCALAR_VADC_BOUNDARY                     (100U)

/** Channel mask for phase voltage U */
#define MOTOR0_BLDC_SCALAR_SCAN_PH_U                         ((uint32_t)1 << MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM)
/** Channel mask for phase voltage V */
#define MOTOR0_BLDC_SCALAR_SCAN_PH_V                         ((uint32_t)1 << MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM)
/** Channel mask for phase voltage W */
#define MOTOR0_BLDC_SCALAR_SCAN_PH_W                         ((uint32_t)1 << MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM)

#define MOTOR0_BLDC_SCALAR_HS_U                       ((uint16_t)HIGH_ON << (uint16_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_U_SLICE_NUM))
#define MOTOR0_BLDC_SCALAR_LS_U                       ((uint16_t)LOW_ON << (uint16_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_U_SLICE_NUM))
#define MOTOR0_BLDC_SCALAR_HS_V                       ((uint16_t)HIGH_ON << (uint16_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_V_SLICE_NUM))
#define MOTOR0_BLDC_SCALAR_LS_V                       ((uint16_t)LOW_ON << (uint16_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_V_SLICE_NUM))
#define MOTOR0_BLDC_SCALAR_HS_W                       ((uint16_t)HIGH_ON << (uint16_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_W_SLICE_NUM))
#define MOTOR0_BLDC_SCALAR_LS_W                       ((uint16_t)LOW_ON << (uint16_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_W_SLICE_NUM))


/***********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * States of the transition state control algorithm
 */
typedef enum BLDC_SCALAR_TRANSITION_STATE_MACHINE
{
  BLDC_SCALAR_TRANSITION_SM_FIRST_KICK = 0U, /*!< First kick from motor standstill position */
  BLDC_SCALAR_TRANSITION_SM_COAST_SLOW = 1U, /*!< Motor running slowly,BEMF zero crossing not possible */
  BLDC_SCALAR_TRANSITION_SM_COAST_FAST = 2U, /*!< Motor running fast enough to enter in close loop */
  BLDC_SCALAR_TRANSITION_SM_WRONG_DIR = 3U   /*!< Motor spinning in wrong direction */

} BLDC_SCALAR_TRANSITION_STATE_MACHINE_t;

/**
 * This enumerates the status of motor direction identification
 */
typedef enum BLDC_SCALAR_DIR_IDENTIFICATION_STATUS
{
  BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_UNKNOWN = 0U,     /*!< Actual motor direction unknown */
  BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_IN_PROGRESS = 1U, /*!< Actual motor direction identification is in progress */
  BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_COMPLETED = 2U    /*!< Actual motor direction identified */

} BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_t;

/**
 * This enumerates the braking status to stop the motor.
 */
typedef enum BLDC_SCALAR_BRAKING_STATUS
{
  BLDC_SCALAR_BRAKING_STATUS_INIT = 0U,        /*!< Braking related PWM configuration done and braking started */
  BLDC_SCALAR_BRAKING_STATUS_IN_PROGRESS = 1U, /*!< Braking is in progress (Lower switches will be turned as per configuration */
  BLDC_SCALAR_BRAKING_STATUS_COMPLETED = 2U    /*!< Braking cycle completed */

} BLDC_SCALAR_BRAKING_STATUS_t;

/**
 * This enumerates the rotor initial position identification status
 */
typedef enum BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS
{
  BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_UNKNOWN = 0U,       /*!< Rotor initial position unknown */
  BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_IN_PROGRESS = 1U,   /*!< Rotor initial position identification is in progress */
  BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_SUCCESS = 2U,       /*!< Rotor initial position identified successfully*/
  BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_STATUS_FAILED = 3U, /*!< Rotor initial position identification failed */

} BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_t;

/**
 * This enumerates the internal states of transition state in MSM
 */
typedef enum BLDC_SCALAR_TRANSITION_STATUS
{
  BLDC_SCALAR_TRANSITION_STATUS_UNKNOWN = 0U,     /*!< Not in transition state */
  BLDC_SCALAR_TRANSITION_STATUS_FAILED = 1U,      /*!< Transition failed due to less BEMF/Wrong direction */
  BLDC_SCALAR_TRANSITION_STATUS_COMPLETED = 2U,   /*!< Transition completed. Switch to close loop */
  BLDC_SCALAR_TRANSITION_STATUS_WRONG_DIR = 3U,   /*!< Motor spinning in wrong direction */
  BLDC_SCALAR_TRANSITION_STATUS_CORRECT_DIR = 4U  /*!< Motor spinning in intended direction */

} BLDC_SCALAR_TRANSITION_STATUS_t;
/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/**
 * @brief control parameters of the sensor-less algorithm.
 */
typedef struct BLDC_SCALAR_SL
{
  BLDC_SCALAR_TRANSITION_STATE_MACHINE_t transition_sm_state; /*!< Motor transition state machine */
  BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_t rotor_init_pos_identifcation_status;/*!< Rotor initial position identification status */
  BLDC_SCALAR_TRANSITION_STATUS_t transition_status; /*!< Motor transition state internal status */
  BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_t dir_identification_status; /*!< Motor actual direction identification */
  BLDC_SCALAR_BRAKING_STATUS_t braking_status;          /*!< Braking status */
  XMC_VADC_GROUP_t *open_phase_grp[16];                 /*!< Open phase for BEMF sensing */
  int32_t motor_sensed_direction;                       /*!< motor sensed direction */
  uint32_t commutation_time_raw;                        /*!< Commutation timer */
  uint32_t first_kick_time_count;                       /*!< First kick time in-terms of PWM count */
  uint32_t first_kick_time_counter;                     /*!< First kick time tracker in-terms of PWM count */
  uint32_t current_decay_time_count;                    /*!< Current decay time in-terms of PWM count */
  uint32_t current_decay_time_counter;                  /*!< Current decay time tracker in-terms of PWM count */
  uint32_t phase_energizing_time_count;                 /*!< Phase energizing  time in-terms of PWM count */
  uint32_t phase_energizing_time_counter;               /*!< Phase energizing  time tracker in-terms of PWM count */
  uint32_t braking_time_count;                          /*!< Braking time in-terms of PWM count */
  uint32_t braking_time_counter;                        /*!< Braking time tracker in-terms of PWM count */
#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_ALIGN)
  uint32_t align_counter;                               /*!< Align time tracker in terms of systick count */
  uint32_t align_count;                                 /*!< Align time in terms of systick count */
  int32_t align_duty;                                   /*!< alignment voltage */
#endif
  uint32_t min_threshold;                               /*!< Minimum threshold below which motor stops and chnages the state to IDLE */
  uint32_t bemf_to_dc_link_voltage_ratio;               /*!< BEMF to DC link voltage divider ratio. */
  uint16_t phaseU_voltage;                              /*!< Phase U voltage in Q14 format */
  uint16_t phaseV_voltage;                              /*!< Phase V voltage in Q14 format */
  uint16_t phaseW_voltage;                              /*!< Phase W voltage in Q14 format */
  uint16_t bemf_amplitude;                              /*!< (Max BEMF - Min BEMF) */
  uint16_t bemf_min_emf;                                /*!< Minimum threshold BEMF below which motor will considered as stop. */
  uint16_t bemf_transition_emf;                         /*!< Minimum threshold BEMF to enter into close loop */
  uint16_t startup_commutation_counter;                 /*!< Startup commutation counter to keep a track */
  uint16_t startup_min_amplitude;                       /*!< During transition state, this is the minimum duty will be applied */
  uint16_t configured_braking_amplitude_cmp_val;        /*!< Configured duty cycle for braking */
  uint16_t braking_amplitude_cmp_val;                   /*!< Lower all three switches will be turned on by this duty cycle */
#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER)
  uint16_t single_trigger_compensation;                 /*!< 30 degree time compensation for single trigger */
#else
  uint16_t demagnetization_skip_pwm_count;              /*!< De-magnetization skip time in terms of PWM count */
  uint16_t demagnetization_skip_pwm_counter;            /*!< De-magnetization skip time counter for tracking */
#endif
  uint8_t rotor_init_pos;                               /*!< Identified initial rotor position */
  uint8_t rotor_pos;                                    /*!< Rotor position while running in transition. */
  uint8_t rotor_last_pos;                               /*!< Rotor last position while running in transition. */
  uint8_t new_commutation;                              /*!< New commutation occurred */
  uint8_t bemf_amplification_enabled;                   /*!< BEMF amplification enable/disable control by VADC on chip gain */
  uint8_t bemf_zero_cross_counter;                      /*!< Keeps the track of zero crossing count. */
  uint8_t bemf_rise[16];                                /*!< BEMF zero crossing detection to be done on falling/rising edge.*/
  uint8_t open_phase[16];                               /*!< Next open phase to check BEMF for zero crossing detection */
  uint8_t trigger_skip_cnt;                             /*!< Single trigger BEMF sensing compensation skip count for first event */
  uint8_t startup_failure_counter;                      /*!< Keeps a track of startup failure. For multiple failures it goes into error state */
} BLDC_SCALAR_SL_t;

/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/
extern BLDC_SCALAR_SL_t Motor0_BLDC_SCALAR_SL;
extern const uint8_t Motor0_BLDC_SCALAR_NextHallPat[];
extern uint16_t Motor0_BLDC_SCALAR_OutPat[];
extern const uint8_t Motor0_BLDC_SCALAR_VirtualHall[];

#ifdef __cplusplus
extern "C"
{
#endif
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * @param output Output of the control scheme
 * @param amplitude Absolute value of the amplitude
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This returns the absolute value of the amplitude to be applied to PWM
 *
 * \par<b>Execution Time:</b><br>
 * using O3 optimization level \n
 * without bi-direction control: <b>0.120 uSec</b> \n
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_DirectionControl(int32_t output, uint32_t *amplitude)
{
  /* Absolute value of the output voltage */

  if (output < 0)
  {
    output = -1 * output;
  }
  *amplitude = (uint32_t) output;

}

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function senses all three BEMF's and determines whether motor is spinning or not.
 * If motor is spinning with sufficient BEMF then identify the rotor position at that moment.
 */
void Motor0_BLDC_SCALAR_State_Identification(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * A. Turns OFF all switches. \n
 * B. Turns ON all three lower switches simultaneously with duty cycle as braking amplitude
 *    for configured braking cycle time.
 */
void BLDC_SCALAR_BrakeMotor(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function is called from COAST_FAST mode and at the end of the COAST_SLOW mode
 * when the motor is spinning fast enough to go into zero crossing detection mode. It should
 * be called only after a new commutation time is detected.
 * It checks the motor direction, measures the commutation time and sets up the ADC
 * for zero crossing detection.
 */
void BLDC_SCALAR_PrepZeroCrossingMode(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This is state machine used during transition to close loop time.
 * This state machine make sure the motor will be spinning with sufficient BEMF
 * before transition to close loop.
 */
void Motor0_BLDC_SCALAR_TransitionStateMachine(void);


/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif
#endif //end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#endif //end of #ifndef BLDC_SCALAR_CTRL_SENSORLESS_H
