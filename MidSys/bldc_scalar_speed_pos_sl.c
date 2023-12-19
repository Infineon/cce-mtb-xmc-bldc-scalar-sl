/**
 * @file bldc_scalar_speed_pos_hall.c
 * @brief Speed and position interface using 3 hall sensor feedback. This uses floating prescaler feature of CCU4 for speed capture
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
 * 2016-07-25:
 *     - Initial version
 *
 * @endcond
 *
 */


/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

#include "../MidSys/bldc_scalar_speed_pos_sl.h"
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define BLDC_SCALAR_SPEED_POS_SL_MASK    (0x7U)
#define BLDC_SCALAR_SPEED_POS_SL_2_POS   (2U)
/***********************************************************************************************************************
 * DATA
 **********************************************************************************************************************/
/**
 * Speed calculation constant used for floating prescaler values
 */
uint32_t BLDC_SCALAR_SPEED_POS_SL_Cap_Array[16] = {
                                         0U,
                                         1U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         3U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         7U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         15U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         31U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         63U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         127U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         255U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         511U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         1023U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         2047U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         4095U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         8191U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         16383U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
                                         32767U * BLDC_SCALAR_SPEED_POS_SL_CAP_COMP_VAL,
};


/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/

/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/*
 * Initialize POSIF and CCU4 peripherals
 */
void Motor0_BLDC_SCALAR_SPEED_POS_SL_Init()
{
  Motor0_BLDC_SCALAR_POSIF_SL_Init();
  Motor0_CCU4_SL_Init();
}

/*
 * Start POSIF and CCU4 timer.
 */
void Motor0_BLDC_SCALAR_SPEED_POS_SL_Start()
{
  /*
   * Set RUN bit of the POSIF and
   * CC41 First slice will be started on external start trigger
   */
  XMC_POSIF_Start(MOTOR0_BLDC_SCALAR_POSIF_MODULE);
  /*Clear CCU4 timers*/
  XMC_CCU4_SLICE_ClearTimer(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE);
  XMC_CCU4_SLICE_ClearTimer(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);
}

/*
 * Stop POSIF and CCU4 timer.
 */
void Motor0_BLDC_SCALAR_SPEED_POS_SL_Stop()
{
  Motor0_BLDC_SCALAR_SPEED_POS_SL_ClearSpeedFilter();
  XMC_POSIF_Stop(MOTOR0_BLDC_SCALAR_POSIF_MODULE);
  XMC_CCU4_SLICE_StopTimer(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);
  XMC_CCU4_SLICE_StopTimer(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE);
}

#endif  /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)*/

