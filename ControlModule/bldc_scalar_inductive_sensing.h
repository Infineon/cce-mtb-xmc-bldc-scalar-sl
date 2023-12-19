/**
 * @file bldc_scalar_inductive_sensing.h
 * @brief Inductive sensing based rotor initial position detection control algorithm
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


#ifndef BLDC_SCALAR_INDUCTIVE_SENSING_H
#define BLDC_SCALAR_INDUCTIVE_SENSING_H

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
#include "bldc_scalar_control_sensorless.h"
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)


/***********************************************************************************************************************
 * MACRO
 **********************************************************************************************************************/
#define BLDC_SCALAR_INDUCTIVE_SENSING_PAT_NUM (6U)

#define   MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_A            (WOFF_VL_UH)  /*!< Phase pattern corresponding to U+,V- */
#define   MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_B            (WOFF_VH_UL)  /*!< Phase pattern corresponding to U-,V+ */
#define   MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_C            (WL_VH_UOFF)  /*!< Phase pattern corresponding to V+,W- */
#define   MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_D            (WH_VL_UOFF)  /*!< Phase pattern corresponding to V-,W+ */
#define   MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_E            (WH_VOFF_UL)  /*!< Phase pattern corresponding to W+,U- */
#define   MOTOR0_BLDC_SCALAR_IND_SENSE_MC_PAT_F            (WL_VOFF_UH)  /*!< Phase pattern corresponding to W-,U+ */
/***********************************************************************************************************************
 * ENUMS
 **********************************************************************************************************************/
/**
 * This enumerates the inductive sensing initial rotor identification status
 */
typedef enum BLDC_SCALAR_INDUCTIVE_SENSING_STATUS
{
  BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_UNKNOWN        = 0U,       /*!< Inductive sensing not executed */
  BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_IN_PROGRESS    = 1U,       /*!< Inductive sensing in progress */
  BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_SUCCESS        = 2U,       /*!< Inductive sensing completed successfully*/
  BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_FAILED         = 3U,       /*!< Inductive sensing failed to identify initial rotor position */
} BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_t;

/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
/**  @brief structure inductive sensing */
typedef struct BLDC_SCALAR_INDUCTIVE_SENSING
{
  BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_t status;                        /*!< Inductive sensing status information */
  uint32_t ind_sense_pat[BLDC_SCALAR_INDUCTIVE_SENSING_PAT_NUM];        /*!< During inductive sensing this patterns are applied */
  uint32_t ind_sense_pulse_width;                                       /*!< Inductive sense applied MCM pulse width */
  uint32_t ind_sense_current_decay;                                     /*!< After inductive sense pattern applied wait for current to decay */
  int16_t ind_sense_adc_result[BLDC_SCALAR_INDUCTIVE_SENSING_PAT_NUM];  /*!< Captured VADC results after each inductive sense pattern applied */
  uint8_t ind_sense_pat_index;                                          /*!< Inductive sensing applied pattern index */
  uint8_t read_current_enable;                                          /*!< Flag to indicate to read the current */
  uint8_t rotor_initial_pos;                                            /*!< Identified rotor initial position */
} BLDC_SCALAR_INDUCTIVE_SENSING_t;
/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/
extern BLDC_SCALAR_INDUCTIVE_SENSING_t Motor0_BLDC_SCALAR_InductiveSensing;
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
/*
 * Finds the rotor position based upon the current sensed from six applied patterns during inductive sensing.
 * Determines the patterns to be applied for rotating motor in intended direction.
 */
__STATIC_INLINE BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_t BLDC_SCALAR_CalcIndSenPos(void)
{
  int16_t delta[3U];
  BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_t status;
  uint8_t count;
  uint32_t pos;
  int16_t near_edge_thresh;
  int16_t *curr = &Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_adc_result[0U];

  /* Calculate the deltas between measured current readings */
  for (count = 0U; count < 6U; count += 2U)
  {
    delta[(uint8_t)(count >>1U) ] = curr[count] - curr[count + 1U];
  }

  /* Finds rotor position and determines voltage vector that will cause proper rotation */
  if ((delta[0U] >= delta[1U]) && ((delta[0U] >= delta[2U])))
  {
    /* delta[0] the biggest */
    if (delta[1U] >= delta[2U])
    {
      /* delta[0] > delta[1] > delta[2] */
      /* Angle between 330 degree and 30 degree */
      near_edge_thresh = ((int16_t)((delta[0U] - delta[2U]) >> 1) + delta[2U]); /* dead center point */
      if (delta[1U] > near_edge_thresh)
      {
        /* Near to 30 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 3U; /* Apply (V+,U-) pattern 4*/
        }
        else
        {
          pos = 1U; /* Apply (W+,V-) pattern 6*/
        }
      }
      else
      {
        /* Near to 330 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 1U; /* Apply (V+,W-) pattern 6*/
        }
        else
        {
          pos = 5U; /* Apply (W+,U-) pattern 2*/
        }
      }
    }
    else
    {
      /* delta[0] > delta[2] > delta[1] */
      /* Angle between 270 degree and 330 degree */
      near_edge_thresh = ((int16_t)((delta[0U] - delta[1U]) >> 1) + delta[1U]);
      if (delta[2U] > near_edge_thresh)
      {
        /* Near to 270 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 5U; /* Apply (U+,W-) pattern 2*/
        }
        else
        {
          pos = 4U; /* Apply (V+,U-) pattern 3*/
        }
      }
      else
      {
        /* Near to 330 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 1U; /* Apply (V+,W-) pattern 6*/
        }
        else
        {
          pos = 5U; /* Apply (W+,U-) pattern 2*/
        }
      }
    }
  }
  else if ((delta[1U] >= delta[0U]) && ((delta[1U] >= delta[2U])))
  {
    /* delta[1] the biggest */
    if(delta[0U] >= delta[2U])
    {
      /* delta[1] > delta[0] > delta[2] */
      /* Angle between 30 degree and 90 degree */
      near_edge_thresh =  ((int16_t)((delta[1U] - delta[2U]) >> 1) + delta[2U]); // dead center point
      if (delta[0U] > near_edge_thresh)
      {
        /* Near to 30 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 3U; /* Apply (V+,U-) pattern 4*/
        }
        else
        {
          pos = 1U; /* Apply (W+,V-) pattern 6*/
        }
      }
      else
      {
        /* Near to 90 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 2U; /* Apply (W+,U-) pattern 5*/
        }
        else
        {
          pos = 3U; /* Apply (U+,V-) pattern 4*/
        }
      }
    }
    else
    {
      /* delta[1] > delta[2] > delta[0] */
      /* Angle between 90 degree and 150 degree */
      near_edge_thresh = ((int16_t)((delta[1U] - delta[0U]) >> 1) + delta[0U]);
      if (delta[2U] > near_edge_thresh)
      {
        /* Near to 150 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 6U; /* Apply (W+,V-) pattern 1*/
        }
        else
        {
          pos = 2U; /* Apply (U+,W-) pattern 5*/
        }
      }
      else
      {
        /* Near to 90 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 2U; /* Apply (W+,U-) pattern 5*/
        }
        else
        {
          pos = 3U; /* Apply (U+,V-) pattern 4*/
        }
      }
    }
  }
  else
  {
    /* delta[2] is the biggest */
    if (delta[0U] >= delta[1U])
    {
      /* delta[2] > delta[0] > delta[1] */
      /* Angle between 210 degree and 270 degree */
      near_edge_thresh =  ((int16_t)((delta[2U] - delta[1U]) >> 1) + delta[1U]); // dead center point
      if (delta[0U] > near_edge_thresh)
      {
        /* Near to 270 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 5U; /* Apply (U+,W-) pattern 2*/
        }
        else
        {
          pos = 4U; /* Apply (V+,U-) pattern 3*/
        }
      }
      else
      {
        /* Near to 210 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 4U; /* Apply (U+,V-) pattern 3*/
        }
        else
        {
          pos = 6U; /* Apply (V+,W-) pattern 1*/
        }
      }
    }
    else
    {
      /* delta[2] > delta[1] > delta[0] */
      /* Angle between 150 degree and 210 degree */
      near_edge_thresh = ((int16_t)((delta[2U] - delta[0U]) >> 1) + delta[0U]);
      if (delta[1U] > near_edge_thresh)
      {
        /* Near to 150 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 6U; /* Apply (W+,V-) pattern 1*/
        }
        else
        {
          pos = 2U; /* Apply (U+,W-) pattern 5*/
        }
      }
      else
      {
        /* Near to 210 degree edge */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          pos = 4U; /* Apply (U+,V-) pattern 3*/
        }
        else
        {
          pos = 6U; /* Apply (V+,W-) pattern 1*/
        }
      }
    }
  }

  /* Rotor initial position identified successfully */
  Motor0_BLDC_SCALAR_InductiveSensing.rotor_initial_pos = (uint8_t)pos;
  status = BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_SUCCESS;
  return status;
}

/* This function setups CCU8 for inductance sensing. */
void Motor0_BLDC_SCALAR_InductiveSensing_Init(void);

__STATIC_INLINE void Motor0_BLDC_SCALAR_IndSenseRotorPosIdentification(void)
{
  volatile uint32_t result_temp = 0U;

  XMC_VADC_GROUP_t * DC_Link_Group = (XMC_VADC_GROUP_t *)MOTOR0_BLDC_SCALAR_VADC_IDC_LINK_GRP;

  if(Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pat_index < BLDC_SCALAR_INDUCTIVE_SENSING_PAT_NUM)
  {
    /* Stop timers (just clear the start bit so they can restart on the rising edge) */
    Motor0_BLDC_SCALAR_CCU8_PWM_Stop();

    Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern((uint16_t)Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pat[Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pat_index]);
    Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();

    if(Motor0_BLDC_SCALAR_InductiveSensing.read_current_enable == 1U)
    {
      /* Wait for ADC result */
      while ((result_temp & VADC_G_RES_VF_Msk) == 0U)
      {
        result_temp = Motor0_BLDC_SCALAR_Current_Motor_GetDetailedResult((XMC_VADC_GROUP_t *)MOTOR0_BLDC_SCALAR_VADC_IDC_LINK_GRP,
            MOTOR0_BLDC_SCALAR_VADC_IDC_LINK_RES_REG_NUM);
      }
      Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_adc_result[Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pat_index++] = (int16_t)(result_temp & 0x0000FFFFU);
      Motor0_BLDC_SCALAR_InductiveSensing.read_current_enable = 0U;

      /* Turn off all switches */
      Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0U);
      Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();

      /* Update period value to produce configured delay */
      Motor0_BLDC_SCALAR_PWM_BC_SetTimerPeriodMatch(Motor0_BLDC_SCALAR_PWM_BC.ccu8_handle_ptr->phase_ptr[1U],
                                                    (uint16_t)Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_current_decay);
      /*Enable CCU8 shadow transfer*/
      Motor0_BLDC_SCALAR_PWM_BC_EnableShadowTransfer();
    }
    else
    {
      Motor0_BLDC_SCALAR_InductiveSensing.read_current_enable = 1U;
      /* Update period value to produce configured delay */
      Motor0_BLDC_SCALAR_PWM_BC_SetTimerPeriodMatch(Motor0_BLDC_SCALAR_PWM_BC.ccu8_handle_ptr->phase_ptr[1U],
                                                    (uint16_t)Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pulse_width);
      /*Enable CCU8 shadow transfer*/
      Motor0_BLDC_SCALAR_PWM_BC_EnableShadowTransfer();
      /* Clear the ADC Valid flag for instantaneous DC link current measurement.*/
      DC_Link_Group->VFR = (uint32_t)(1<<MOTOR0_BLDC_SCALAR_VADC_IDC_LINK_RES_REG_NUM);
      result_temp = 0U;
    }

    /* Start Timers */
    Motor0_BLDC_SCALAR_CCU8_PWM_Start();
  }
  else
  {
    /* Inductive sensing completed */
    Motor0_BLDC_SCALAR_InductiveSensing.status = BLDC_SCALAR_CalcIndSenPos();
    Motor0_BLDC_SCALAR_SL.rotor_init_pos = Motor0_BLDC_SCALAR_InductiveSensing.rotor_initial_pos;
    Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status = BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_SUCCESS;
  }
}

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif


#endif /*end of #if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING) */
#endif /*end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */

#endif
