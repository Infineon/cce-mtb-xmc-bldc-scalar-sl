/**
 * @file bldc_scalar_volt_3phase.h
 * @brief Three Phase voltage measurement
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

#ifndef BLDC_SCALAR_VOLT_3PHASE_H_
#define BLDC_SCALAR_VOLT_3PHASE_H_
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "../MCUInit/vadc.h"
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define BLDC_SCALAR_MEASUREMENT_SHIFT_14 (14U)       /*!< Shift by 14 bits */

/***********************************************************************************************************************
 * EXTERN DATA STRUCTURES
 **********************************************************************************************************************/
extern uint32_t  motor0_phasevoltage_adc_scale;

/***********************************************************************************************************************
 * DATA STRUCTURES
 **********************************************************************************************************************/
#ifdef __cplusplus
   extern "C" {
#endif
/***********************************************************************************************************************
 * API Prototypes
 **********************************************************************************************************************/
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * initializes VADC group, channel, result for 3 phase voltage measurement.
 */
 void Motor0_BLDC_SCALAR_3PhVoltageMeasurment_Init(void);

/**
 * @param dc_bus_voltage VADC conversion result for 3 phase voltage measurement
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Returns VADC conversion result for 3 phase voltage measurement
 */

__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_Get3PhVoltage(uint32_t * PhU_Voltage, uint32_t * PhV_Voltage, uint32_t * PhW_Voltage)
{
  uint32_t phu_voltage;
  uint32_t phv_voltage;
  uint32_t phw_voltage;
  phu_voltage = VADC_GetResult((VADC_G_TypeDef*)MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_RES_REG_NUM);
  *PhU_Voltage = (uint32_t)((phu_voltage * motor0_phasevoltage_adc_scale) >> BLDC_SCALAR_MEASUREMENT_SHIFT_14);
  phv_voltage = VADC_GetResult((VADC_G_TypeDef*)MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_RES_REG_NUM);
  *PhV_Voltage = (uint32_t)((phv_voltage * motor0_phasevoltage_adc_scale) >> BLDC_SCALAR_MEASUREMENT_SHIFT_14);
  phw_voltage = VADC_GetResult((VADC_G_TypeDef*)MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_RES_REG_NUM);
  *PhW_Voltage = (uint32_t)((phw_voltage * motor0_phasevoltage_adc_scale) >> BLDC_SCALAR_MEASUREMENT_SHIFT_14);
}
/**
 * @param group pointer and channel mask
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Adds required channels into scan sequence
 */

__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(XMC_VADC_GROUP_t *const group_ptr, const uint32_t ch_mask)
{
  XMC_VADC_GROUP_ScanAddMultipleChannels(group_ptr,ch_mask);
}
/**
 * @param group pointer and channel number
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Adds required channel into scan sequence
 */

__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanAddChannelToSequence(XMC_VADC_GROUP_t *const group_ptr, const uint32_t channel)
{
  XMC_VADC_GROUP_ScanAddChannelToSequence(group_ptr, channel);
}
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * clears the channel event
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_ChannelClearEvent(void)
{
  XMC_VADC_GROUP_ChannelClearEvent((VADC_G_TypeDef*)MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM);
  XMC_VADC_GROUP_ChannelClearEvent((VADC_G_TypeDef*)MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM);
  XMC_VADC_GROUP_ChannelClearEvent((VADC_G_TypeDef*)MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP,MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM);
}
/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Enables VADC on chip gain for BEMF channels. BEMF amplification used only during startup
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_BemfAmplificationEnable(void)
{
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0,MOTOR0_BLDC_SCALAR_VADC_ON_CHIP_GAIN_BEMF,MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP_NUM,
                                    MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0,MOTOR0_BLDC_SCALAR_VADC_ON_CHIP_GAIN_BEMF,MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP_NUM,
                                    MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0,MOTOR0_BLDC_SCALAR_VADC_ON_CHIP_GAIN_BEMF,MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP_NUM,
                                    MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM);
}

/**
 * @param None <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Disables VADC on chip gain for BEMF channels.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_BemfAmplificationDisable(void)
{
  /* Disable on chip gain for normal operation */
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0,0U,MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP_NUM,
                                    MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0,0U,MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP_NUM,
                                    MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM);
  XMC_VADC_GLOBAL_SHS_SetGainFactor(SHS0,0U,MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP_NUM,
                                    MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM);
}

/**
 * @param group pointer <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Generates conversion request (Software initiated conversion).
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanTriggerConversion(XMC_VADC_GROUP_t *const group_ptr)
{
  /* Disable on chip gain for normal operation */
  XMC_VADC_GROUP_ScanTriggerConversion(group_ptr);
}

/**
 * @param group pointer <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Generates conversion request (Software initiated conversion).
 */
__STATIC_INLINE uint8_t Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanGetReqSrcEventStatus(XMC_VADC_GROUP_t *const group_ptr)
{
  uint8_t evt_status;
  /* Disable on chip gain for normal operation */
  evt_status = XMC_VADC_GROUP_ScanGetReqSrcEventStatus(group_ptr);
  return (evt_status);
}
/**
 * @param group pointer,boundary0 and boundary1 <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * configures the VADC boundary value.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_SetBoundaries(XMC_VADC_GLOBAL_t *const global_ptr,
                                                                     const uint32_t boundary0,
                                                                     const uint32_t boundary1)
{
  XMC_VADC_GLOBAL_SetBoundaries(global_ptr, boundary0, boundary1);
}

/**
 * @param group pointer,channel number and criteria <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * configures the channel event criteria.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_ChannelEventCriteria(XMC_VADC_GROUP_t *const group_ptr,
                                                                     const uint32_t boundary0,
                                                                     const uint32_t boundary1)
{
  XMC_VADC_GROUP_ChannelTriggerEventGenCriteria(group_ptr, boundary0, boundary1);
}
/**
 * @param group pointer,gating mode <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * configures the gating mode for BEMF channels.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_GatingModeSel(XMC_VADC_GROUP_t *const group_ptr,
		                                                                 XMC_VADC_GATEMODE_t mode_sel)
{
  XMC_VADC_GROUP_ScanSetGatingMode(group_ptr, mode_sel);
}

/**
 * @param group pointer,trigger signal selection <br>
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * configures the trigger signal selection for BEMF channels.
 */
__STATIC_INLINE void Motor0_BLDC_SCALAR_VOLT_3PHASE_TriggerSignalSel(XMC_VADC_GROUP_t *const group_ptr,
		                                                             XMC_VADC_TRIGGER_INPUT_SELECT_t trigger_input)
{
  XMC_VADC_GROUP_ScanSelectTrigger(group_ptr, trigger_input);
}


#endif /*end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)*/
#endif /*end of BLDC_SCALAR_VOLT_3PHASE_H_*/
/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif
