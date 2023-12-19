/**
 * @file posif.c
 * @brief POSIF in hall sensor configuration for BLDC motor control with hall sensor feedback.
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
 *     - Initial version
 * 2016-09-08:
 *     - Updated for sensorless support
 * @endcond
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "posif.h"

/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/**
 * POSIF configured with multichannel mode
 */
const XMC_POSIF_CONFIG_t Motor0_BLDC_SCALAR_POSIF_GLOBAL_Config =
{
  .mode   = XMC_POSIF_MODE_MCM                     /**< POSIF Operational mode */
};
#endif
/**
 * POSIF multi-channel configurations
 * Multi-channel pattern update signal is connected to CCU4 blanking signal period match or
 * CCU4 fast synchronization period match
 */
const XMC_POSIF_MCM_CONFIG_t Motor0_BLDC_SCALAR_POSIF_MCM_Config =
{
  .pattern_sw_update      = 0U,
  .pattern_update_trigger = MOTOR0_BLDC_SCALAR_POSIF_PATTERN_UPDATE_SEL,
  .pattern_trigger_edge   = XMC_POSIF_HSC_TRIGGER_EDGE_RISING,
  .pwm_sync               = MOTOR0_BLDC_SCALAR_POSIF_PWM_SYNC_SIGNAL_SEL
};



/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/*
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function initializes the POSIF peripheral in multichannel mode.
 * Enables below Multi-channel pattern shadow transfer event and connects to interrupt node
 */
void Motor0_BLDC_SCALAR_POSIF_SL_Init()
{
  XMC_POSIF_Init(MOTOR0_BLDC_SCALAR_POSIF_MODULE, &Motor0_BLDC_SCALAR_POSIF_GLOBAL_Config);
  XMC_POSIF_MCM_Init(MOTOR0_BLDC_SCALAR_POSIF_MODULE, &Motor0_BLDC_SCALAR_POSIF_MCM_Config);
}
#endif
