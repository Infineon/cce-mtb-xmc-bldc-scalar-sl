/**
 * @file ccu4.h
 * @brief CCU4 slices configuration
 * -# CCU4 slice for hall signal blanking time
 * -# CCU4 slice for speed capture is used.
 * -# CCU4 slice is optionally used (Motor0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4) for
 * multi-channel pattern synchronization instead of CCU8 period match to avoid the PWM time delay.
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
 * @endcond
 *
 */
/**
 * @addtogroup BLDC_SCALAR BLDC Motor Control
 * @{
 */

/**
 * @addtogroup MCUInit
 * @brief  MCU peripheral initialization <br>
 * @{
 */
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#ifndef MCUINIT_CCU4_H_
#define MCUINIT_CCU4_H_

#include "../Configuration/bldc_scalar_derived_parameter.h"
#include "../Configuration/bldc_scalar_mcuhw_config.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4 == 1U)
/** Shadow transfer mask for fast sync slice */
#define MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SHADOWTRANSFER      ((uint32_t)1 << \
    (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU4_MCMSYNC_SLICE_NUM))
#endif
#if(MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#define MOTOR0_CCU4_COMMUTATION_TIMER_SHADOWTRANSFER        ((uint32_t)1 << \
    (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE_NUM))

#define MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SHADOWTRANSFER      ((uint32_t)1 << \
    (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU4_TRIGGER_SLICE_NUM))

#define MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SHADOWTRANSFER      ((uint32_t)1 << \
    (uint32_t)(4U * MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE_NUM))

#endif
/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
/**
 * @brief Structure for the phase delay slice configurations
 */
typedef struct CCU4_PH_DELAY_CONFIG
{
  uint16_t blanking_time;  /*!< Time delay between hall edge detection and re-sampling. Compare value for phase delay slice */
  uint16_t phase_delay;    /*!< Period value for phase delay slice Trigger to the multi-channel pattern update */
} CCU4_PH_DELAY_CONFIG_t;

#ifdef __cplusplus
   extern "C" {
#endif
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * CCU4 slices initialization for sensorless configuration.
 */
void Motor0_CCU4_SL_Init(void);
#if(MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * CCU4 slices initialization for generating trigger for VADC conversion.
 */
void Motor0_BLDC_SCALAR_CCU4_SL_VADCTrigger_Init(void);
#endif
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * CCU4 slices initialization for speed capture.
 */
void Motor0_BLDC_SCALAR_CCU4_SL_SpeedCapture_Init(void);
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This slice is used to measure commutation time.This slice is also used for speed measurement.
 */
void Motor0_BLDC_SCALAR_CCU4_SL_CommutationTimer_Init(void);
#endif

#if(1U == MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4)
/**
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function initializes the MCM pattern sync slice with configured value of 1us as period value.
 * The slice is free running timer of having configurable period.The period acts as MCM pattern sync signal.
 */
void Motor0_BLDC_SCALAR_CCU4_MCMSync_Init(void);
#endif

/***********************************************************************************************************************
 * EXTERN
 **********************************************************************************************************************/
extern CCU4_PH_DELAY_CONFIG_t Motor0_BLDC_SCALAR_CCU4_3Hall_PhaseDelay_Config;
/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif
#endif /* MCUINIT_CCU4_H_ */
