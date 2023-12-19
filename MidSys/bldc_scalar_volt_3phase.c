/**
 * @file bldc_scalar_volt_3phase.c
 * @brief Phase voltage measurement
 * @date 2016-09-08
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
 * 2016-09-08:
 *     - Initial version
 *
 * @endcond
 *
 */
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "bldc_scalar_volt_3phase.h"
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/***********************************************************************************************************************
 * GLOBAL DATA
 **********************************************************************************************************************/
uint32_t motor0_phasevoltage_adc_scale = (uint32_t)MOTOR0_BLDC_SCALAR_PHASE_VOLTAGE_ADC_SCALE;
/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/* Phase Voltage Measurement */

/**
 * @brief Initializes channel and result configuration 3 phase voltage measurement.
 * @param void
 * @return void
 */
void Motor0_BLDC_SCALAR_3PhVoltageMeasurment_Init(void)
{
  /* Initialize VADC channel for 3 phases */
  XMC_VADC_GROUP_ChannelInit(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM, &Motor0_BLDC_SCALAR_VADC_PhaseU_Voltage_CH_handle);
  XMC_VADC_GROUP_ChannelInit(MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM, &Motor0_BLDC_SCALAR_VADC_PhaseV_Voltage_CH_handle);
  XMC_VADC_GROUP_ChannelInit(MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM, &Motor0_BLDC_SCALAR_VADC_PhaseW_Voltage_CH_handle);

  /* Initialize VADC result for 3 phase voltage measurement */
  XMC_VADC_GROUP_ResultInit(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_RES_REG_NUM, &Motor0_BLDC_SCALAR_VADC_Phase_Voltage_Res_handle);
  XMC_VADC_GROUP_ResultInit(MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_RES_REG_NUM, &Motor0_BLDC_SCALAR_VADC_Phase_Voltage_Res_handle);
  XMC_VADC_GROUP_ResultInit(MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_RES_REG_NUM, &Motor0_BLDC_SCALAR_VADC_Phase_Voltage_Res_handle);

  /* Bind the channel event to shared service request line */
  XMC_VADC_GROUP_ChannelSetEventInterruptNode(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM,
    		MOTOR0_BLDC_SCALAR_ZERO_CROSS_SR);
  XMC_VADC_GROUP_ChannelSetEventInterruptNode(MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM,
    		MOTOR0_BLDC_SCALAR_ZERO_CROSS_SR);
  XMC_VADC_GROUP_ChannelSetEventInterruptNode(MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM,
    		MOTOR0_BLDC_SCALAR_ZERO_CROSS_SR);

  /* Add 3 phase channels to scan sequence*/
  XMC_VADC_GROUP_ScanAddChannelToSequence(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM);
  XMC_VADC_GROUP_ScanAddChannelToSequence(MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM);
  XMC_VADC_GROUP_ScanAddChannelToSequence(MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM);

}
#endif
