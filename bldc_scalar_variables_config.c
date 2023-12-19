/**
 * @file bldc_scalar_variables_config.c
 * @brief Data structure initialization as per user configurations
 * @date 2016-08-09
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
 *     - Initial version
 * 2016-09-08:
 *     - Updated for sensorless support
 *
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../ControlModule/bldc_scalar_control_scheme.h"
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#include "../ControlModule/bldc_scalar_control_sensorless.h"
#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
#include "../ControlModule/bldc_scalar_inductive_sensing.h"
#endif
#endif


/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)

#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
/************************START : Inductive Sensing  ***********************************/
BLDC_SCALAR_INDUCTIVE_SENSING_t Motor0_BLDC_SCALAR_InductiveSensing =
{
  .status = BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_UNKNOWN,
  .rotor_initial_pos = (uint8_t)0xFF,
  .ind_sense_pat = {
                      MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_A,
                      MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_B,
                      MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_C,
                      MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_D,
                      MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_E,
                      MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_F
                    },
  .ind_sense_pulse_width = (uint32_t)MOTOR0_BLDC_SCALAR_SL_IND_SENSE_CCU8_PULSE_WIDTH_COUNT,
  .ind_sense_current_decay = (uint32_t)MOTOR0_BLDC_SCALAR_SL_IND_SENSE_CCU8_CURRENT_DECAY_COUNT,
  .ind_sense_adc_result = {0,0,0,0,0,0},
  .ind_sense_pat_index = (uint8_t)0,
  .read_current_enable = (uint8_t)0,
  .status = BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_UNKNOWN
};
#endif /* end of #if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING) */
/************************END : Inductive Sensing  ************************************/

/************************START : Current Control during startup ***********************************/
/** Current control PI */
BLDC_SCALAR_PI_CONTROLLER_t Motor0_BLDC_SCALAR_StartupCurrentControl_PI =
{
  .uk_limit_min = 0,
  .uk_limit_max = (int32_t)MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_PI_OP,
  .kp           = (uint16_t)MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_KP,
  .ki           = (uint16_t)MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_KI,
  .scale_kpki   = (uint8_t)MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_PI_SCALE,
  .sat_state    = 1U
};

BLDC_SCALAR_CURRENT_CONTROL_t Motor0_BLDC_SCALAR_StartupCurrentControl =
{
  .user_current_set             = (int32_t)MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_SET_T,
  .ref_current                  = 0,
  .fdbk_current                 = 0
};
/************************END : Current Control during startup***********************************/

#if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_ALIGN)
BLDC_SCALAR_RAMP_t Motor0_BLDC_SCALAR_Align_Ramp  =
{
  .input_value           = (uint32_t)MOTOR0_BLDC_SCALAR_SL_ALIGN_DUTY_CYCLE,
  .set_value             = (uint32_t)0,
  .ramp_rate          = (uint32_t)(MOTOR0_BLDC_SCALAR_SL_ALIGN_RAMP_RATE_T),
};
#endif
/***********************Sensor-less Control **********************************************/
BLDC_SCALAR_SL_t Motor0_BLDC_SCALAR_SL =
{
  .dir_identification_status                 = BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_UNKNOWN,
  .braking_status                            = BLDC_SCALAR_BRAKING_STATUS_INIT,
  .bemf_amplification_enabled                = 0U,
  .braking_time_count                        = (uint32_t)MOTOR0_BLDC_SCALAR_SL_BRAKING_CYCLE_COUNT,
#if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_ALIGN)
  .align_count                               = MOTOR0_BLDC_SCALAR_SL_ALIGN_COUNT,
  .align_duty                                = MOTOR0_BLDC_SCALAR_SL_ALIGN_DUTY_CYCLE,
#endif
  .configured_braking_amplitude_cmp_val      = (uint16_t)MOTOR0_BLDC_SCALAR_SL_BRAKING_CCU8_COMP_REG,
  .braking_amplitude_cmp_val                 = (uint16_t)MOTOR0_BLDC_SCALAR_SL_BRAKING_CCU8_COMP_REG,
  .startup_min_amplitude                     = (uint16_t)MOTOR0_BLDC_SCALAR_SL_STARTUP_MIN_DUTY_CYCLE,
  .first_kick_time_count                     = (uint32_t)MOTOR0_BLDC_SCALAR_SL_FIRST_KICK_COUNT,
  .current_decay_time_count                  = (uint32_t)MOTOR0_BLDC_SCALAR_SL_TR_CURRENT_DECAY_COUNT,
  .phase_energizing_time_count               = (uint32_t)MOTOR0_BLDC_SCALAR_SL_TR_PHASE_ENERGIZATION_COUNT,
  .bemf_to_dc_link_voltage_ratio             = (uint32_t)MOTOR0_BLDC_SCALAR_SL_BEMF_TO_DC_LINK_RATIO,
  .bemf_min_emf                              = (uint16_t)MOTOR0_BLDC_SCALAR_SL_MIN_BEMF_VOLTAGE_PP_T,
  .bemf_transition_emf                       = (uint16_t)MOTOR0_BLDC_SCALAR_SL_TRANSITION_BEMF_VOLTAGE_PP_T,
  .commutation_time_raw                      = 0U,
  .min_threshold                             = (uint32_t)MOTOR0_BLDC_SCALAR_SL_MIN_THRESHOLD_T,
  .transition_sm_state                       = BLDC_SCALAR_TRANSITION_SM_FIRST_KICK,
  .rotor_init_pos_identifcation_status       = BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_UNKNOWN,
#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER)
  .single_trigger_compensation               = (uint16_t)(((uint32_t)MOTOR0_BLDC_SCALAR_CCU8_PERIOD_REG + (uint32_t)1) << (uint32_t)1) >> MOTOR0_BLDC_SCALAR_CCU4_PRESCALER,
#else
  .demagnetization_skip_pwm_count            = (uint16_t)MOTOR0_BLDC_SCALAR_SL_DEMAGNETIZATION_BLANKING_PWM_COUNT,
#endif
  .bemf_rise = { (uint8_t)0,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_FALL,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_FALL,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_RISE,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_FALL,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_RISE,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_RISE,
      (uint8_t)0,
      (uint8_t)0,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_FALL,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_FALL,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_RISE,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_FALL,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_RISE,
      (uint8_t)MOTOR0_BLDC_SCALAR_BEMF_RISE,
      (uint8_t)0},

  .open_phase = { (uint8_t)0,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_U,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_V,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_W,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_W,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_V,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_U,
      (uint8_t)0,
      (uint8_t)0,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_U,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_V,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_W,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_W,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_V,
      (uint8_t)MOTOR0_BLDC_SCALAR_SCAN_PH_U,
      (uint8_t)0},

 .open_phase_grp = { 0,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,
                         0,
                         0,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,
                     MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,
                     0
                    }

 };
#endif /*end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)*/
/************************START : Speed Control ***********************************/
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
/** Speed control PI */
BLDC_SCALAR_PI_CONTROLLER_t Motor0_BLDC_SCALAR_SpeedControl_PI =
{
  .uk_limit_min = -((int32_t)MOTOR0_BLDC_SCALAR_SPEED_PI_OP),
  .uk_limit_max = (int32_t)MOTOR0_BLDC_SCALAR_SPEED_PI_OP,
  .kp           = (uint16_t)MOTOR0_BLDC_SCALAR_SPEED_KP,
  .ki           = (uint16_t)MOTOR0_BLDC_SCALAR_SPEED_KI,
  .scale_kpki   = (uint8_t)MOTOR0_BLDC_SCALAR_SPEED_PI_SCALE,
  .sat_state    = 1U
};
#endif   /* end of #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL)) */
/************************END : Speed Control ***********************************/

/************************START : Current Control ***********************************/
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
/** Current control PI */
BLDC_SCALAR_PI_CONTROLLER_t Motor0_BLDC_SCALAR_CurrentControl_PI =
{
  .uk_limit_min = -((int32_t)MOTOR0_BLDC_SCALAR_CURRENT_PI_OP),
  .uk_limit_max = (int32_t)MOTOR0_BLDC_SCALAR_CURRENT_PI_OP,
  .kp           = (uint16_t)MOTOR0_BLDC_SCALAR_CURRENT_KP,
  .ki           = (uint16_t)MOTOR0_BLDC_SCALAR_CURRENT_KI,
  .scale_kpki   = (uint8_t)MOTOR0_BLDC_SCALAR_CURRENT_PI_SCALE,
  .sat_state    = 1U
};
#endif   /* end of #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL)) */
/************************END : Current Control ***********************************/

/************************START : Voltage Compensation Filter ***********************************/
#if ((MOTOR0_BLDC_SCALAR_ENABLE_VOLT_COMPENSATION == 1U))
#if ((MOTOR0_BLDC_SCALAR_ENABLE_VOLT_COMPENSATION_FILTER == 1U))
/** Voltage compensation low pass filter*/
BLDC_SCALAR_PT1_FILTER_t  Motor0_BLDC_SCALAR_PT1_VoltageComp =
{
  .z1      = (int32_t)MOTOR0_BLDC_SCALAR_VOLTCOMP_FILTER_Z,
  .y_max   = (int32_t)BLDC_SCALAR_PT1_VOLT_COMP_MAX_LIMIT,
  .y_min   = (int32_t)BLDC_SCALAR_PT1_VOLT_COMP_MIN_LIMIT,
  .pt1_buf = (int32_t)BLDC_SCALAR_PT1_VOLT_COMP_BUF_VAL
};
#endif
#endif   /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_VOLT_COMPENSATION == 1U) */
/************************END : Voltage Compensation Filter ***********************************/

/************************START : Direct current filter ***********************************/
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U))
#if (MOTOR0_BLDC_SCALAR_ENABLE_IDC_LINK_CURRENT_FILTER == 1U)
/** Direct current low pass filter */
BLDC_SCALAR_PT1_FILTER_t  Motor0_BLDC_SCALAR_PT1_DirectCurrent =
{
  .z1      = (int32_t)MOTOR0_BLDC_SCALAR_CURRENT_FILTER_Z,
  .y_max   = (int32_t)BLDC_SCALAR_PT1_LIMIT,
  .y_min   = (int32_t)0,
  .pt1_buf = (int32_t)0
};
#endif
#endif    /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_IDC_LINK_CURRENT_FILTER == 1U) */

/************************END : Direct current filter ***********************************/

/************************START : Average current filter ***********************************/
#if (((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 1U) || (MOTOR0_BLDC_SCALAR_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK == 1U)))
#if (MOTOR0_BLDC_SCALAR_ENABLE_IDC_AVERAGE_CURRENT_FILTER == 1U)
/** Average current low pass filter */
BLDC_SCALAR_PT1_FILTER_t  Motor0_BLDC_SCALAR_PT1_AverageCurrent =
{
  .z1      = (int32_t)MOTOR0_BLDC_SCALAR_IDC_AVERAGE_CURRENT_FILTER_Z,
  .y_max   = (int32_t)BLDC_SCALAR_PT1_LIMIT,
  .y_min   = (int32_t)-BLDC_SCALAR_PT1_LIMIT,
  .pt1_buf = (int32_t)0
};
#endif
#endif
/************************END : Average current filter ***********************************/


#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U))
#if (MOTOR0_BLDC_SCALAR_POTENTIOMETER_PT1_FILTER_ENABLE == 1U)
/** Potentiometer low pass filter */
BLDC_SCALAR_PT1_FILTER_t Motor0_BLDC_SCALAR_PT1_Potentiometer =
{
  .z1      = (int32_t)MOTOR0_BLDC_SCALAR_POTENTIOMETER_FILTER_Z,
  .y_max   = (int32_t)BLDC_SCALAR_PT1_LIMIT,
  .y_min   = (int32_t)-BLDC_SCALAR_PT1_LIMIT,
  .pt1_buf = (int32_t)0
};
#endif
#endif

/************************START : Ramp Generator ***********************************/
#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U)
/** Linear Ramp */
BLDC_SCALAR_RAMP_t Motor0_BLDC_SCALAR_Ramp =
{
  .set_value             = 0,
  .ramp_up_rate          = (uint32_t)(MOTOR0_BLDC_SCALAR_RAMP_UP_RATE_T),
  .ramp_down_rate        = (uint32_t)(MOTOR0_BLDC_SCALAR_RAMP_DOWN_RATE_T),
};
#endif   /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U) */
/************************END : Ramp Generator ***********************************/

/************************START : Block commutation PWM generator ***********************************/
/** Data structure initialization of PWM_BC module */
BLDC_SCALAR_PWM_BC_t Motor0_BLDC_SCALAR_PWM_BC =
{
  .ccu8_handle_ptr = &Motor0_BLDC_SCALAR_CCU8_PWM_Config,
#if (MOTOR0_BLDC_SCALAR_MODULATION == BLDC_SCALAR_PWM_HIGHSIDE)
  .shadow_modulation_ptr    = Motor0_BLDC_SCALAR_PWM_BC_HsShadowMod,
#elif (MOTOR0_BLDC_SCALAR_MODULATION == BLDC_SCALAR_PWM_LOWSIDE)
  .shadow_modulation_ptr    = Motor0_BLDC_SCALAR_PWM_BC_LsShadowMod,
#elif (MOTOR0_BLDC_SCALAR_MODULATION == BLDC_SCALAR_PWM_HIGHSIDE_SYNCHRECTI)
  .shadow_modulation_ptr    = Motor0_BLDC_SCALAR_PWM_BC_HsShadowMod_SyncRect,
#endif

#if (MOTOR0_BLDC_SCALAR_MODULATION == BLDC_SCALAR_PWM_HIGHSIDE)
  .immediate_modulation_ptr    = Motor0_BLDC_SCALAR_PWM_BC_HsImmediateMod,
#elif (MOTOR0_BLDC_SCALAR_MODULATION == BLDC_SCALAR_PWM_LOWSIDE)
  .immediate_modulation_ptr    = Motor0_BLDC_SCALAR_PWM_BC_LsImmediateMod,
#elif (MOTOR0_BLDC_SCALAR_MODULATION == BLDC_SCALAR_PWM_HIGHSIDE_SYNCHRECTI)
  .immediate_modulation_ptr    = Motor0_BLDC_SCALAR_PWM_BC_HsImmediateMod_SyncRect,
#endif

  .modulation_type        = (PWM_BC_MOD_t)MOTOR0_BLDC_SCALAR_MODULATION,

  .ph_cmpval              = {0U},
  .ph_mcpatt_compmask     = {(uint16_t)((uint32_t)0x1U << (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_U_SLICE_NUM)),
      (uint16_t)((uint32_t)0x1U << (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_V_SLICE_NUM)) ,
      (uint16_t)((uint32_t)0x1U << (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_W_SLICE_NUM))},

  .ph_bothside_compmask   = { (uint16_t)((uint32_t)0x3U << (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_U_SLICE_NUM)),
      (uint16_t)((uint32_t)0x3U << (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_V_SLICE_NUM)) ,
      (uint16_t)((uint32_t)0x3U << (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU8_PH_W_SLICE_NUM)) },

  .amplitude_scale        = ((uint16_t)MOTOR0_BLDC_SCALAR_AMPLITUDE_SCALE) + 1U,
  .inverter_pin           = (PWM_BC_INVERTERPINLEVEL_t)MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF
};
/************************END : Block commutation PWM generator ***********************************/

/************************START : Speed and Position ***********************************/
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/** Data structure initialization of SPEED_POS_HALL module */
BLDC_SCALAR_SPEED_POS_SL_t Motor0_BLDC_SCALAR_SPEED_POS_SL =
{
  .index                  = 0U,
  .speedindex             = 0U,
  .capval                 = (uint32_t)MOTOR0_BLDC_SCALAR_SPEED_CONSTANT_AVG,
  .speedcheck             = 0U,
  .captureval             = {0U,0U,0U,0U,0U,0U},
  .speedaccum             = 0U,
  .speed_constant         = (uint32_t) MOTOR0_BLDC_SCALAR_SPEED_CONSTANT_AVG,
};
#endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
/************************END : Speed and Position ***********************************/

/************************START : Current Measurement ***********************************/
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U) ||(MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 1U))
/** DC link current measurement */
BLDC_SCALAR_CURRENT_MEASUREMENT_t Motor0_BLDC_SCALAR_CurrentMeasurement =
{
  .direct_dc_amplifier_offset     = (int32_t)MOTOR0_BLDC_SCALAR_AMPL_OFFSET_T,
  .avg_dc_amplifier_offset        = (int32_t)MOTOR0_BLDC_SCALAR_AMPL_OFFSET_T,
  .current_adc_scale              = (uint32_t)MOTOR0_BLDC_SCALAR_CURRENT_ADC_SCALE,
#if (MOTOR0_BLDC_SCALAR_ENABLE_OVER_CURRENT == 1U)
  .short_circuit_current          = (int32_t)MOTOR0_BLDC_SCALAR_SHORTCIRCUIT_CURRENT_T,
#endif
  .amplitude                      = &Motor0_BLDC_SCALAR.amplitude,
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U))
#if (MOTOR0_BLDC_SCALAR_ENABLE_DEMAGNET_BLANKING == 1U)
  .demagnetization_blanking_count = (uint32_t)MOTOR0_BLDC_SCALAR_DEMAGNETIZATION_TIME_COUNT,
#endif /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_DEMAGNET_BLANKING == 1U) */
#endif /* #if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U)) */
};
#endif
/************************END : Current Measurement ***********************************/

/************************START : Control Schemes ***********************************/

#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_VOLTAGE_CTRL))
/** Voltage control */
BLDC_SCALAR_VOLTAGE_CONTROL_t Motor0_BLDC_SCALAR_VoltageControl =
{
  .user_start_voltage_set       = (int32_t)MOTOR0_BLDC_SCALAR_USER_RAMP_INITIAL_VOLTAGE_T,
  .user_voltage_set             = (int32_t)MOTOR0_BLDC_SCALAR_USER_VOLTAGE_SET_T
};
#endif

/***********************************************************/
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
/** Speed control */
BLDC_SCALAR_SPEED_CONTROL_t Motor0_BLDC_SCALAR_SpeedControl =
{
  .user_start_speed_set         = (int32_t)MOTOR0_BLDC_SCALAR_USER_RAMP_INITIAL_SPEED_T,
  .user_speed_set               = (int32_t)MOTOR0_BLDC_SCALAR_USER_SPEED_SET_T
};
#endif

/***********************************************************/
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
/** Current control */
BLDC_SCALAR_CURRENT_CONTROL_t Motor0_BLDC_SCALAR_CurrentControl =
{
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
  .user_start_current_set       = (int32_t)MOTOR0_BLDC_SCALAR_USER_RAMP_INITIAL_CURRENT_T,
  .user_current_set             = (int32_t)MOTOR0_BLDC_SCALAR_USER_CURRENT_SET_T,
#endif
  .ref_current                  = 0,
  .fdbk_current                 = 0
};
#endif

/***********************************************************/
/************************END : Control Schemes ***********************************/

/************************START : BLDC SCALAR ***********************************/
/** BLDC_SCALAR handle - contains the control parameters */
BLDC_SCALAR_t Motor0_BLDC_SCALAR =
{
  .msm_state                    = BLDC_SCALAR_MSM_STOP,
  .error_status                 = (uint32_t)0,

  .period                       = (uint16_t)MOTOR0_BLDC_SCALAR_CCU8_PERIOD_REG,
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U))
  .motor_set_direction          = MOTOR0_BLDC_SCALAR_MOTOR_DIRECTION,
#else
  .motor_set_direction          = BLDC_SCALAR_POSITIVE_DIR,
#endif
#endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  .min_analogip_val             = (int32_t)MOTOR0_BLDC_SCALAR_MIN_ANALOG_INPUT_LOW_LIMIT_T,
#endif
  .amplitude_high_threshold     = (uint32_t)MOTOR0_BLDC_SCALAR_AMPLITUDE_HIGH_THRESHOLD_T,
  .min_amplitude_configured     = (uint32_t)MOTOR0_BLDC_SCALAR_MIN_AMPLITUDE_T,
  .min_amplitude                = (uint32_t)MOTOR0_BLDC_SCALAR_MIN_AMPLITUDE_T,
  .max_amplitude                = (uint32_t)MOTOR0_BLDC_SCALAR_MAX_AMPLITUDE_T,
#if(MOTOR0_BLDC_SCALAR_CONTROL_LOOP_EXECUTION_RATE > 1U)
  .control_loop_exec_rate       = (uint32_t)MOTOR0_BLDC_SCALAR_CONTROL_LOOP_EXECUTION_RATE,
  .control_loop_exec_count      = 1,
#endif

#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP_DOWN_VOLTAGE_CLAMPING == 1U)
  .max_dc_link_voltage          = (((uint32_t)MOTOR0_BLDC_SCALAR_RAMP_DOWN_VOLTAGE_LIMIT_T * (uint32_t)MOTOR0_BLDC_SCALAR_VOLTAGE_ADC_SCALE) >> BLDC_SCALAR_SHIFT_14),
#endif
  .nominal_dc_link_voltage      = (uint32_t)MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT_T,
#if (MOTOR0_BLDC_SCALAR_ENABLE_OVER_CURRENT == 1U)
  .overcurrent_count            = (uint32_t)MOTOR0_BLDC_SCALAR_CURRENT_PROTECTION_COUNT,
#endif
#if (MOTOR0_BLDC_SCALAR_ENABLE_UNDER_OVER_VOLTAGE == 1U)
  .over_under_voltage_count     = (uint32_t)MOTOR0_BLDC_SCALAR_VOLTAGE_PROTECTION_COUNT,
#endif

#if (MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U)
  .bootstrap_count              = (uint32_t)MOTOR0_BLDC_SCALAR_BOOTSTRAP_COUNT,
#endif
#if (MOTOR0_BLDC_SCALAR_ENABLE_UNDER_OVER_VOLTAGE == 1U)
  .over_voltage_limit           = (int32_t)MOTOR0_BLDC_SCALAR_MAX_DC_LINK_VOLTAGE_T,
  .under_voltage_limit          = (int32_t)MOTOR0_BLDC_SCALAR_MIN_DC_LINK_VOLTAGE_T,
#endif

  .speed_scale                  = (uint32_t)MOTOR0_BLDC_SCALAR_SPEED_SCALE,
  .current_scale                = (uint32_t)MOTOR0_BLDC_SCALAR_CURRENT_SCALE,
  .voltage_scale                = (uint32_t)MOTOR0_BLDC_SCALAR_VOLTAGE_SCALE,
  .speed_mech_scale             = (uint32_t)MOTOR0_BLDC_SCALAR_SPEED_MECH_SCALE,

#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  .speedcontrol_rate           = MOTOR0_BLDC_SCALAR_SPEED_CTRL_EXECUTION_RATE,
#endif

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
  .mc_pattern             = {
                             /* high side synchronous rectification modulation - patterns are initialized in Init function*/
                             {
                               0x0U
                             },
                             /* high side modulation */
                             {
                                 (uint16_t)0x0U,                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_A, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_E, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_F,
                                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_C, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_B, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_D, (uint16_t)0x00U,
                                 (uint16_t)0x0U,                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_D, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_B, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_C,
                                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_F, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_E, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_A, (uint16_t)0x00U
                             },
                             /* low side modulation */
                             {
                                 (uint16_t)0x0U,                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_A, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_E, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_F,
                                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_C, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_B, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_D, (uint16_t)0x00U,
                                 (uint16_t)0x0U,                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_D, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_B, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_C,
                                 (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_F, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_E, (uint16_t)MOTOR0_BLDC_SCALAR_MC_PAT_A, (uint16_t)0x00U
                             }
            },
#endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
};

/************************END : BLDC SCALAR ***********************************/
