/**
 * @file bldc_scalar_control_sensorless.c
 * @brief Sensor-less feedback control algorithm
 * @date 2016-09-08
 *
 * @cond
 ***********************************************************************************************************************
 * BLDC_SCALAR_CONTROL v1.0.2 - BLDC motor control using block commutation
 * Copyright (c) 2015, Infineon Technologies AG
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
#include "bldc_scalar_control_sensorless.h"

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * DATA
 **********************************************************************************************************************/
/** Table used to determine next virtual hall pattern in commanded direction to determine next pattern to be energized. */
const uint8_t Motor0_BLDC_SCALAR_NextHallPat[] = { 1U, 3U, 6U, 2U, 5U, 1U, 4U, 1U,
                                                   1U, 5U, 3U, 1U, 6U, 4U, 2U, 1U };

/**
 *  Table used to determine next pattern to be energized. Virtual hall position is used as an index. First 8 patterns
 * are for one direction and next 8 for reverse direction
 */
uint16_t Motor0_BLDC_SCALAR_OutPat[]     = {  (uint16_t)0x00,
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_W) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_V)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_U) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_W)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_U) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_V)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_V) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_U)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_W) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_U)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_V) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_W)),
                                                    (uint16_t)0x00,
                                                    (uint16_t)0x00,
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_V) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_W)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_W) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_U)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_V) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_U)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_U) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_V)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_U) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_W)),
                                                    ((uint16_t)(MOTOR0_BLDC_SCALAR_LS_W) | (uint16_t)(MOTOR0_BLDC_SCALAR_HS_V)),
                                                    (uint16_t)0x00
                                                 };

/**
 * Table used to determine virtual hall position to energizes next pattern. First 8 virtual halls
 * are for one direction and next 8 for reverse direction.This table is used to determine rotor position
 * based upon BEMF measured.
 */
const uint8_t Motor0_BLDC_SCALAR_VirtualHall[] = { 0U,
                                                   5U, /* Vu>Vv>Vw (MOTOR0_BLDC_SCALAR_HS_U, MOTOR0_BLDC_SCALAR_LS_W)*/
                                                   4U, /* Vu>Vw>Vv (MOTOR0_BLDC_SCALAR_HS_U, MOTOR0_BLDC_SCALAR_LS_V)*/
                                                   3U, /* Vv>Vw>Vu (MOTOR0_BLDC_SCALAR_HS_V, MOTOR0_BLDC_SCALAR_LS_U)*/
                                                   1U, /* Vv>Vu>Vw (MOTOR0_BLDC_SCALAR_HS_V, MOTOR0_BLDC_SCALAR_LS_W)*/
                                                   6U, /* Vw>Vu>Vv (MOTOR0_BLDC_SCALAR_HS_W, MOTOR0_BLDC_SCALAR_LS_V)*/
                                                   2U, /* Vw>Vv>Vu (MOTOR0_BLDC_SCALAR_HS_W, MOTOR0_BLDC_SCALAR_LS_U)*/
                                                   0U,
                                                   0U,
                                                   2U, /* Vu>Vv>Vw (MOTOR0_BLDC_SCALAR_HS_U, MOTOR0_BLDC_SCALAR_LS_W)*/
                                                   3U, /* Vu>Vw>Vv (MOTOR0_BLDC_SCALAR_HS_U, MOTOR0_BLDC_SCALAR_LS_V)*/
                                                   4U, /* Vv>Vw>Vu (MOTOR0_BLDC_SCALAR_HS_V, MOTOR0_BLDC_SCALAR_LS_U)*/
                                                   6U, /* Vv>Vu>Vw (MOTOR0_BLDC_SCALAR_HS_V, MOTOR0_BLDC_SCALAR_LS_W)*/
                                                   1U, /* Vw>Vu>Vv (MOTOR0_BLDC_SCALAR_HS_W, MOTOR0_BLDC_SCALAR_LS_V)*/
                                                   5U, /* Vw>Vv>Vu (MOTOR0_BLDC_SCALAR_HS_W, MOTOR0_BLDC_SCALAR_LS_U)*/
                                                   0U
                                                 };

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * A. Turns OFF all switches.
 * B. Sense all three BEMF's by triggering VADC manually.
 * C. Average of two BEMF samples are taken.
 */
static void Motor0_BLDC_SCALAR_SenseBEMF3(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function calculates the motor position based on the 3 back emf voltages
 * It also calculates the magnitude (DeltaV) of the phase voltages which is used
 * to determine the motor speed.
 */
static void Motor0_BLDC_SCALAR_CalcPosDelta(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function checks the direction that the motor is spinning.During the first time through this function
 * it stores the Hall state and then waits to see if Hall State +2 is reached before Hall State -2.
 * If Hall State + 2 (the next next hall state) is reach first, then motor is spinning in intended direction.
 * If Hall State -2 is reached first, then motor is spinning in wrong direction.
 */
static void Motor0_BLDC_SCALAR_CheckDirection(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function calculates the speed based upon transition time between two commutations.
 * Determines the motor direction based upon the new rotor position.
 */
static void BLDC_SCALAR_CalcSpdAndChkDir(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * This function sets up the scan request source to sample the open phase voltage.
 * once every PWM cycle (at the end of the cycle) in case of single trigger.
 * It also sets up the limit checker so that an interrupt occurs if the ADC result is
 * inside the configured boundary limits for zero crossing detection.
 * The limit checker also filters out the de-magnetization spikes.
 */
static void BLDC_SCALAR_EnableZeroCrossingDetectViaADC(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * After Inductances Sensing/Rotor alignment we know the position of the rotor.
 * Energize the right 2 phases to get to motor to spin in our desired direction.
 * The phases are energized with duty cycle controlled by current control
 * output and minimum duty is limited(e.g. 10%) for a fixed amount of time(e.g. 12ms).
 */
static void BLDC_SCALAR_TRANSITION_SM_FIRST_KICK_Fun(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * The motor is spinning slow enough for the special low speed mode to operate, but not fast enough
 * for normal zero crossing detection.In low speed mode we energize 2 phases for a fixed
 * time (e.g. 500usec) and then turn all phases off,wait a fixed time (e.g. 100 usec) for the current
 * to stop, then measure the voltage on all 3 phases.From the phase voltages we can find the rotor position
 * and approximate speed.
 */
static void BLDC_SCALAR_TRANSITION_SM_COAST_SLOW_Fun(void);


/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * The motor is spinning fast enough for normal zero crossing detection.This function verifies the
 * BEMF is still sufficient to go to close loop operation and motor is running in intended direction.
 * A. Motor running in wrong direction with BEMF greater than threshold BEMF then it will go to ERROR state.
 * B. Motor running in intended direction with sufficient BEMF then prepare for zero crossing and switch to
 *    close loop.
 * C. Spinning slowly then go to coast slow mode.
 * D. Motor spinning very slowly/stopped.
 */
static void BLDC_SCALAR_TRANSITION_SM_COAST_FAST_Fun(void);

/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * The motor is spinning in the wrong direction so turn on all three
 * low side MOSFETs with the configured braking duty cycle (e.g. 50%)
 * for configured braking cycle time. Once braking cycle over then it
 * verifies that motor is stopped. If motor still spinning then it again
 * applies the braking cycle.
 */
static void BLDC_SCALAR_TRANSITION_SM_WRONG_DIR_Fun(void);
/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/

void BLDC_SCALAR_BrakeMotor(void)
{
  if (Motor0_BLDC_SCALAR_SL.braking_status == BLDC_SCALAR_BRAKING_STATUS_INIT)
  {
    Motor0_BLDC_SCALAR_PWM_BC_BrakeInit((uint16_t) Motor0_BLDC_SCALAR_SL.braking_amplitude_cmp_val);
    Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;
    Motor0_BLDC_SCALAR_SL.braking_status = BLDC_SCALAR_BRAKING_STATUS_IN_PROGRESS;
  }
  else if (Motor0_BLDC_SCALAR_SL.braking_status == BLDC_SCALAR_BRAKING_STATUS_IN_PROGRESS)
  {
    /* Braking cycle is in progress */
    if (Motor0_BLDC_SCALAR_SL.braking_time_counter == Motor0_BLDC_SCALAR_SL.braking_time_count)
    {
      /* Braking cycle completed.Turn off all switches */
      Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0x0U);
      Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();
      Motor0_BLDC_SCALAR_SL.braking_status = BLDC_SCALAR_BRAKING_STATUS_COMPLETED;

      /* Reset braking counter for next cycle */
      Motor0_BLDC_SCALAR_SL.braking_time_counter = 0U;
    }
    else
    {
      Motor0_BLDC_SCALAR_SL.braking_time_counter++;
    }
  }
  else
  {
    /* Braking cycle is completed.*/
  }
}

static void Motor0_BLDC_SCALAR_SenseBEMF3(void)
{
  uint32_t Vu_temp[2] = { 0U, 0U };
  uint32_t Vv_temp[2] = { 0U, 0U };
  uint32_t Vw_temp[2] = { 0U, 0U };
  uint32_t adc_index;
  uint8_t vadc_conversion_status;

  /* Stop the timers */
  Motor0_BLDC_SCALAR_CCU8_PWM_Stop();

  /* Turn off all switches.*/
  Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0U);
  Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();

  /* Reset all compare values */
  Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(0U);

  for (adc_index = 0U; adc_index < 2U; adc_index++)
  {
#if(VADC_SCAN_3PH_SAME_GROUP == 1U)
    /* Trigger the Group 1 Scan via SW. */
    Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanTriggerConversion(
           (XMC_VADC_GROUP_t *) (void*) MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP);

    /* Wait for all BEMF conversions to complete */
    do
    {
      vadc_conversion_status = Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanGetReqSrcEventStatus(
          (XMC_VADC_GROUP_t *) (void*) MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP);
    } while (vadc_conversion_status == 0U);
#else
    Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanTriggerConversion((XMC_VADC_GROUP_t *)VADC_SCAN_GROUP1); // Trigger the Group 1 Scan via SW
    Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanTriggerConversion((XMC_VADC_GROUP_t *)VADC_SCAN_GROUP2);

    /* Wait for all BEMF conversions to complete */
    do
    {
      vadc_conversion_status = Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanGetReqSrcEventStatus((XMC_VADC_GROUP_t *)VADC_SCAN_GROUP1);
    }
    while(vadc_conversion_status == 0U);

    vadc_conversion_status = 0U;
    /* Wait for all BEMF conversions to complete */
    do
    {
      vadc_conversion_status = Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanGetReqSrcEventStatus((XMC_VADC_GROUP_t *)VADC_SCAN_GROUP2);
    }
    while(vadc_conversion_status == 0U);
#endif
    Motor0_BLDC_SCALAR_VOLT_3PHASE_Get3PhVoltage(&Vu_temp[adc_index], &Vv_temp[adc_index], &Vw_temp[adc_index]);
  }
  Motor0_BLDC_SCALAR_SL.phaseU_voltage = (uint16_t) (Vu_temp[0] + Vu_temp[1]) >> 1U;
  Motor0_BLDC_SCALAR_SL.phaseV_voltage = (uint16_t) (Vv_temp[0] + Vv_temp[1]) >> 1U;
  Motor0_BLDC_SCALAR_SL.phaseW_voltage = (uint16_t) (Vw_temp[0] + Vw_temp[1]) >> 1U;
}


static void Motor0_BLDC_SCALAR_CalcPosDelta(void)
{
  uint8_t direction = (uint8_t) Motor0_BLDC_SCALAR.motor_set_direction & 8U;

  if ((Motor0_BLDC_SCALAR_SL.phaseU_voltage >= Motor0_BLDC_SCALAR_SL.phaseV_voltage) &&
      (Motor0_BLDC_SCALAR_SL.phaseV_voltage >= Motor0_BLDC_SCALAR_SL.phaseW_voltage))
  {
    /* U >= V >= W */
    Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_VirtualHall[1U + direction];
    Motor0_BLDC_SCALAR_SL.bemf_amplitude = Motor0_BLDC_SCALAR_SL.phaseU_voltage -
                                           Motor0_BLDC_SCALAR_SL.phaseW_voltage;
  }
  else if ((Motor0_BLDC_SCALAR_SL.phaseU_voltage >= Motor0_BLDC_SCALAR_SL.phaseW_voltage) &&
           (Motor0_BLDC_SCALAR_SL.phaseW_voltage >= Motor0_BLDC_SCALAR_SL.phaseV_voltage))
  {
    /* U >= W >= V */
    Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_VirtualHall[2U + direction];
    Motor0_BLDC_SCALAR_SL.bemf_amplitude = Motor0_BLDC_SCALAR_SL.phaseU_voltage -
                                           Motor0_BLDC_SCALAR_SL.phaseV_voltage;
  }
  else if ((Motor0_BLDC_SCALAR_SL.phaseV_voltage >= Motor0_BLDC_SCALAR_SL.phaseW_voltage) &&
           (Motor0_BLDC_SCALAR_SL.phaseW_voltage >= Motor0_BLDC_SCALAR_SL.phaseU_voltage))
  {
    /* V >= W >= U */
    Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_VirtualHall[3U + direction];
    Motor0_BLDC_SCALAR_SL.bemf_amplitude = Motor0_BLDC_SCALAR_SL.phaseV_voltage -
                                           Motor0_BLDC_SCALAR_SL.phaseU_voltage;
  }
  else if ((Motor0_BLDC_SCALAR_SL.phaseV_voltage >= Motor0_BLDC_SCALAR_SL.phaseU_voltage) &&
           (Motor0_BLDC_SCALAR_SL.phaseU_voltage >= Motor0_BLDC_SCALAR_SL.phaseW_voltage))
  {
    /* V >= U >= W */
    Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_VirtualHall[4U + direction];
    Motor0_BLDC_SCALAR_SL.bemf_amplitude = Motor0_BLDC_SCALAR_SL.phaseV_voltage -
                                           Motor0_BLDC_SCALAR_SL.phaseW_voltage;
  }
  else if ((Motor0_BLDC_SCALAR_SL.phaseW_voltage >= Motor0_BLDC_SCALAR_SL.phaseU_voltage) &&
           (Motor0_BLDC_SCALAR_SL.phaseU_voltage >= Motor0_BLDC_SCALAR_SL.phaseV_voltage))
  {
    /* W >= U >= V */
    Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_VirtualHall[5U + direction];
    Motor0_BLDC_SCALAR_SL.bemf_amplitude = Motor0_BLDC_SCALAR_SL.phaseW_voltage -
                                           Motor0_BLDC_SCALAR_SL.phaseV_voltage;
  }
  else if ((Motor0_BLDC_SCALAR_SL.phaseW_voltage >= Motor0_BLDC_SCALAR_SL.phaseV_voltage) &&
           (Motor0_BLDC_SCALAR_SL.phaseV_voltage >= Motor0_BLDC_SCALAR_SL.phaseU_voltage))
  {
    /* W >= V >= U */
    Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_VirtualHall[6U + direction];
    Motor0_BLDC_SCALAR_SL.bemf_amplitude = Motor0_BLDC_SCALAR_SL.phaseW_voltage -
                                           Motor0_BLDC_SCALAR_SL.phaseU_voltage;
  }
  else
  {
    /* Not really possible */
  }

  /* Update rotor last and next position on new commutation */
  if ((Motor0_BLDC_SCALAR_SL.rotor_pos != Motor0_BLDC_SCALAR_SL.rotor_last_pos) &&
      (Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_min_emf))
  {
    Motor0_BLDC_SCALAR_SL.new_commutation = 1U;
    Motor0_BLDC_SCALAR_SL.rotor_last_pos = Motor0_BLDC_SCALAR_SL.rotor_pos;
  }
}

void Motor0_BLDC_SCALAR_State_Identification(void)
{
  /* Sense all 3 BEMF's */
  Motor0_BLDC_SCALAR_SenseBEMF3();
  /* Determines rotor position and BEMF amplitude */
  Motor0_BLDC_SCALAR_CalcPosDelta();
}


static void Motor0_BLDC_SCALAR_CheckDirection(void)
{
  static uint8_t NextNextHallState = 0U;
  static uint8_t LastLastHallState = 0U;

  uint8_t direction = (uint8_t) Motor0_BLDC_SCALAR.motor_set_direction & 8U;

  if (Motor0_BLDC_SCALAR_SL.dir_identification_status == BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_UNKNOWN)
  {
     NextNextHallState = Motor0_BLDC_SCALAR_NextHallPat[Motor0_BLDC_SCALAR_NextHallPat[Motor0_BLDC_SCALAR_SL.rotor_pos +
                        direction] + direction];

    LastLastHallState = Motor0_BLDC_SCALAR_NextHallPat[Motor0_BLDC_SCALAR_NextHallPat[NextNextHallState + direction] +
                        direction];
     Motor0_BLDC_SCALAR_SL.dir_identification_status = BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_IN_PROGRESS;
  }
  else if (Motor0_BLDC_SCALAR_SL.dir_identification_status == BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_IN_PROGRESS)
  {
    if ( (Motor0_BLDC_SCALAR_SL.rotor_pos == NextNextHallState))
    {
      /* Motor spinning in intended direction */
      Motor0_BLDC_SCALAR.actual_motor_direction = Motor0_BLDC_SCALAR.motor_set_direction;

      /* Motor direction identified successfully */
      Motor0_BLDC_SCALAR_SL.dir_identification_status = BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_COMPLETED;
    }
    else if ((Motor0_BLDC_SCALAR_SL.rotor_pos == LastLastHallState))
    {
      /* Motor spinning in reverse direction */
      if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_NEGATIVE_DIR)
      {
        Motor0_BLDC_SCALAR.actual_motor_direction = BLDC_SCALAR_POSITIVE_DIR;
      }
      else
      {
        Motor0_BLDC_SCALAR.actual_motor_direction = BLDC_SCALAR_NEGATIVE_DIR;
      }
      /* Motor direction identified successfully */
      Motor0_BLDC_SCALAR_SL.dir_identification_status = BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_COMPLETED;
    }
    else
    {
      /* Motor direction identification is in progress */
    }
  }
  else
  {
    /* Direction identification completed */
  }
}

static void BLDC_SCALAR_CalcSpdAndChkDir(void)
{
  uint32_t capval;
  uint32_t prescaler;
  uint32_t speed = 0U;

  if (Motor0_BLDC_SCALAR_SL.startup_commutation_counter <= MOTOR0_BLDC_SCALAR_SL_STARTUP_COMMUTATION_COUNT)
  {
    Motor0_BLDC_SCALAR_SL.startup_commutation_counter++;
  }

  Motor0_BLDC_SCALAR_SL.new_commutation = 0U;

  /* Determines actual motor direction */
  Motor0_BLDC_SCALAR_CheckDirection();

  if (Motor0_BLDC_SCALAR_SL.dir_identification_status == BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_COMPLETED)
  {
    if (Motor0_BLDC_SCALAR.actual_motor_direction != Motor0_BLDC_SCALAR.motor_set_direction)
    {
      /* Spinning in the wrong direction */
      Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_WRONG_DIR;
    }
    else
    {
      Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_CORRECT_DIR;
      /* Get the time between two zero cross events */
      Motor0_BLDC_SCALAR_SPEED_POS_SL_ReadCaptureValue(&capval, &prescaler);
      /* Restart capture timer */
      Motor0_BLDC_SCALAR_SPEED_POS_SL_ResetCaptureTimePrescaler();
      /* Speed calculation */
      Motor0_BLDC_SCALAR_SPEED_POS_SL_SpeedCalculation(capval, &speed);
        Motor0_BLDC_SCALAR.motor_speed = ((Motor0_BLDC_SCALAR.actual_motor_direction * (int32_t) speed *
                                          (int32_t) Motor0_BLDC_SCALAR.speed_mech_scale) >> BLDC_SCALAR_SPEED_SCALE_RES);
    }
    /* Reset the direction identification status to unknown for next iteration */
    Motor0_BLDC_SCALAR_SL.dir_identification_status = BLDC_SCALAR_DIR_IDENTIFICATION_STATUS_UNKNOWN;
  }
  else
  {
    /* Direction identification is in progress */
  }
}

/*
 * This function sets up the scan request source to sample the open phase voltage.
 * once every PWM cycle (at the end of the cycle) in case of single trigger.
 * It also sets up the limit checker so that an interrupt occurs if the ADC result is
 * inside the configured boundary limits for zero crossing detection.
 * The limit checker also filters out the de-magnetization spikes.
 */
static void BLDC_SCALAR_EnableZeroCrossingDetectViaADC(void)
{
  uint32_t dc_link_voltage;
  uint32_t dc_link_voltage_in_bemf_scale;
  uint8_t direction = (uint8_t) Motor0_BLDC_SCALAR.motor_set_direction & 8U; /* intended direction */
  dc_link_voltage = Motor0_BLDC_SCALAR_VOLT_DCBUS_GetRawDCLinkVoltage();
  dc_link_voltage_in_bemf_scale = (uint32_t)((dc_link_voltage * Motor0_BLDC_SCALAR_SL.bemf_to_dc_link_voltage_ratio) >> BLDC_SCALAR_TEN);

  /* VADC configurations */
  if (Motor0_BLDC_SCALAR_SL.bemf_rise[Motor0_BLDC_SCALAR_SL.rotor_pos + direction] == MOTOR0_BLDC_SCALAR_BEMF_RISE)
  {
    /*
     * back emf is rising, so boundary 0 is the zero crossing voltage (Vdc / 2) and Boundary 1 is (Vdc - a little)
     */
    Motor0_BLDC_SCALAR_VOLT_3PHASE_SetBoundaries((XMC_VADC_GLOBAL_t *)(void*)VADC, (dc_link_voltage_in_bemf_scale >>
                                         (uint32_t)1), (dc_link_voltage_in_bemf_scale - MOTOR0_BLDC_SCALAR_VADC_BOUNDARY));
  }
  else
  {
    /*
     * back emf is falling, so boundary 0 is (GND + a little) and Boundary 1 is the zero crossing voltage (Vdc / 2)
     */
    Motor0_BLDC_SCALAR_VOLT_3PHASE_SetBoundaries((XMC_VADC_GLOBAL_t *)(void*)VADC, (MOTOR0_BLDC_SCALAR_VADC_BOUNDARY),
                                                 (dc_link_voltage_in_bemf_scale >> (uint32_t)1));
  }

  /* Remove all the phase voltage channels from scan request source */
#if (VADC_SCAN_3PH_SAME_GROUP == 1U)
  Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, 0U);
#else
  Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(VADC_G0, 0U);
  Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry(VADC_G1, 0U);
#endif

  /* Add open phase channel in scan request source for conversion for next 60 degrees */
  Motor0_BLDC_SCALAR_VOLT_3PHASE_AddScanEntry((XMC_VADC_GROUP_t *)(void*)
      Motor0_BLDC_SCALAR_SL.open_phase_grp[Motor0_BLDC_SCALAR_SL.rotor_pos + direction],
      (uint32_t)Motor0_BLDC_SCALAR_SL.open_phase[Motor0_BLDC_SCALAR_SL.rotor_pos + direction]);
}


RAM_ATTRIBUTE void BLDC_SCALAR_PrepZeroCrossingMode(void)
{
  uint8_t direction = (uint8_t) Motor0_BLDC_SCALAR.motor_set_direction & 8U;
  uint32_t capval;     /* Time between two zero cross events */
  uint32_t prescaler;  /* Prescaler value at the time of capture */
  uint32_t speed = 0U; /* electrical speed of the motor */
  int32_t end_value;
  int32_t temp_amplitude;

  /* If the back emf is high enough and the motor has had enough commutations and is spinning in the right direction, then
   go into closed loop zero crossing mode. */
  if ((Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_transition_emf) &&
        (Motor0_BLDC_SCALAR_SL.startup_commutation_counter > MOTOR0_BLDC_SCALAR_SL_STARTUP_COMMUTATION_COUNT) &&
        (Motor0_BLDC_SCALAR.motor_set_direction == Motor0_BLDC_SCALAR.actual_motor_direction) &&
        (Motor0_BLDC_SCALAR_SL.bemf_amplification_enabled == 0U))
  {
    /****************************************************************
     * Setup for closed loop zero crossing
     * 1. Set the commutation time to 1/2 the last commutation time.  After this
     *     commutation times will be measured as the time between zero crossings.
     * 2. Setup the ADC Limit Checker: one boundary is 1/2 the VBAT voltage (BATT_20V)
     *     the other boundary is either VBAT - a small threshold, or GND (0V) + a small
     *     threshold.  The boundaries are used to detect the zero crossing and filter out
     *     de-magnetization spikes.
     * 3. Start the ADC measuring the correct phase voltage at the end of each PWM pulse.
     */
    /* Turn off all switches.*/
    Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0U);
    Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();


    /* 1.  Setup the commutation timer   */
    /* Get the time between two zero cross events */
    Motor0_BLDC_SCALAR_SPEED_POS_SL_ReadCaptureValue(&capval, &prescaler);
    /* Restart capture timer */
    Motor0_BLDC_SCALAR_SPEED_POS_SL_ResetCaptureTimePrescaler();
    /* Speed calculation */
    Motor0_BLDC_SCALAR_SPEED_POS_SL_SpeedCalculation(capval, &speed);
    Motor0_BLDC_SCALAR.motor_speed = ((Motor0_BLDC_SCALAR.actual_motor_direction * (int32_t) speed *
        (int32_t) Motor0_BLDC_SCALAR.speed_mech_scale) >> BLDC_SCALAR_SPEED_SCALE_RES);

#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
    end_value = Motor0_BLDC_SCALAR.analogip_val;
#else
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
    end_value = Motor0_BLDC_SCALAR_SpeedControl.user_speed_set;
#elif ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
    end_value = Motor0_BLDC_SCALAR_CurrentControl.user_current_set;
#elif ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_VOLTAGE_CTRL))
    end_value = Motor0_BLDC_SCALAR_VoltageControl.user_voltage_set;
#endif
#endif

    temp_amplitude = end_value * Motor0_BLDC_SCALAR.motor_set_direction;

#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U)
    if ((int32_t)Motor0_BLDC_SCALAR.amplitude < temp_amplitude)
    {
      Motor0_BLDC_SCALAR_Ramp.set_value = ((int32_t)Motor0_BLDC_SCALAR.amplitude * Motor0_BLDC_SCALAR.motor_set_direction);
    }
    else
    {
      Motor0_BLDC_SCALAR_Ramp.set_value = end_value;
    }
#endif

#if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL)
    Motor0_BLDC_SCALAR_SpeedControl_PI.ik = (Motor0_BLDC_SCALAR.amplitude << (Motor0_BLDC_SCALAR_SpeedControl_PI.scale_kpki - 1U)) *
                                              Motor0_BLDC_SCALAR.motor_set_direction;
#endif

#if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL)
    Motor0_BLDC_SCALAR_CurrentControl_PI.ik = (Motor0_BLDC_SCALAR.amplitude << (Motor0_BLDC_SCALAR_CurrentControl_PI.scale_kpki - 1U)) *
                                              Motor0_BLDC_SCALAR.motor_set_direction;
#endif

    /* Multi-channel update */
      Motor0_BLDC_SCALAR_SL.rotor_pos = Motor0_BLDC_SCALAR_NextHallPat[Motor0_BLDC_SCALAR_SL.rotor_pos + direction];

    Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(
        Motor0_BLDC_SCALAR_OutPat[Motor0_BLDC_SCALAR_SL.rotor_pos + direction]);
    Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();
    Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMCMTxEvent();

    Motor0_BLDC_SCALAR_PWM_BC.immediate_modulation_ptr(Motor0_BLDC_SCALAR_OutPat[Motor0_BLDC_SCALAR_SL.rotor_pos + direction]);

#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
#if (VADC_SCAN_3PH_SAME_GROUP == 1U)
    Motor0_BLDC_SCALAR_VOLT_3PHASE_GatingModeSel(MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, (XMC_VADC_GATEMODE_t)MOTOR0_BLDC_SCALAR_VADC_SCAN_0_GATING);
#else
    Motor0_BLDC_SCALAR_VOLT_3PHASE_GatingModeSel(VADC_G0, (XMC_VADC_GATEMODE_t)MOTOR0_BLDC_SCALAR_VADC_SCAN_0_GATING);
    Motor0_BLDC_SCALAR_VOLT_3PHASE_GatingModeSel(VADC_G1, (XMC_VADC_GATEMODE_t)MOTOR0_BLDC_SCALAR_VADC_SCAN_0_GATING);
#endif
    Motor0_BLDC_SCALAR_SPEED_POS_SL_StartTriggerTimer();
#endif
#if (VADC_SCAN_3PH_SAME_GROUP == 1U)
    Motor0_BLDC_SCALAR_VOLT_3PHASE_TriggerSignalSel((XMC_VADC_GROUP_t *)(void *)MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_SCAN_0_TRIGGER_SIGNAL);
#else
    Motor0_BLDC_SCALAR_VOLT_3PHASE_TriggerSignalSel((XMC_VADC_GROUP_t *)(void *)VADC_G0, MOTOR0_BLDC_SCALAR_VADC_SCAN_0_TRIGGER_SIGNAL);
    Motor0_BLDC_SCALAR_VOLT_3PHASE_TriggerSignalSel((XMC_VADC_GROUP_t *)(void *)VADC_G1, MOTOR0_BLDC_SCALAR_VADC_SCAN_0_TRIGGER_SIGNAL);
#endif
    /* Limit minimum amplitude for normal operation */
    Motor0_BLDC_SCALAR.min_amplitude = Motor0_BLDC_SCALAR.min_amplitude_configured;

    /* 2.Set the ADC Group specific boundaries
     *  The ADC channel control registers for the phase voltages (VADC_G1->CHCTRy)
     *  are set so that the upper boundary is always G1 boundary 1.  The lower
     *  boundary is always G1 boundary 0.
     *  Boundaries are used for limit checking.  They can be changed on the fly.
     */
    BLDC_SCALAR_EnableZeroCrossingDetectViaADC();
  }
}

/*
 * After Inductances Sensing/Rotor alignment we know the position of the rotor.
 * Energize the right 2 phases to get to motor to spin in our desired direction.
 * The phases are energized with duty cycle controlled by current control
 * output and minimum duty is limited(e.g. 10%) for a fixed amount of time(e.g. 12ms).
 */
static void BLDC_SCALAR_TRANSITION_SM_FIRST_KICK_Fun(void)
{
  uint8_t direction = (uint8_t) Motor0_BLDC_SCALAR.motor_set_direction & 8U;
  uint16_t mcmval;

  if (Motor0_BLDC_SCALAR_SL.first_kick_time_counter == 0U)
  {
    /* Stop Timers */
    Motor0_BLDC_SCALAR_CCU8_PWM_Stop();

    /* Load the next MCM pattern based upon the sensed position and direction */
    Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(
        Motor0_BLDC_SCALAR_OutPat[Motor0_BLDC_SCALAR_SL.rotor_init_pos + direction]);
    Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();

    mcmval = (uint16_t) XMC_POSIF_MCM_GetMultiChannelPattern((XMC_POSIF_t *)(void*)MOTOR0_BLDC_SCALAR_POSIF_MODULE);
    Motor0_BLDC_SCALAR_PWM_BC.immediate_modulation_ptr(mcmval);
    Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(Motor0_BLDC_SCALAR_SL.startup_min_amplitude); /* Min. Duty applied */
    Motor0_BLDC_SCALAR_SL.first_kick_time_counter++;

    /* Start Timers */
    Motor0_BLDC_SCALAR_CCU8_PWM_Start();
  }
  else
  {
    if (Motor0_BLDC_SCALAR_SL.first_kick_time_counter == Motor0_BLDC_SCALAR_SL.first_kick_time_count)
    {
      if (Motor0_BLDC_SCALAR_SL.current_decay_time_counter == 0U)
      {
        /* Turn off all switches.*/
        Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0x0U);
        Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();
      }

      if (Motor0_BLDC_SCALAR_SL.current_decay_time_counter == Motor0_BLDC_SCALAR_SL.current_decay_time_count)
      {
        /* Turn off the phases and read the back emf on 3 phases and determine rotor position*/
        Motor0_BLDC_SCALAR_SenseBEMF3();

        /* Start the timers */
        Motor0_BLDC_SCALAR_CCU8_PWM_Start();

        /* Reset the counter for the next iteration */
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;
        Motor0_BLDC_SCALAR_SL.first_kick_time_counter = 0U;

        /* Calculate rotor position */
        Motor0_BLDC_SCALAR_CalcPosDelta();

        /*Check if BEMF is sufficient */
        if (Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_min_emf)
        {
          /* Reset variables for COAST_SLOW mode */
          Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter = 0U;
          Motor0_BLDC_SCALAR_SL.new_commutation = 0U;
          /* BEMF is sufficient,go to coast slow mode. */
          Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_COAST_SLOW;
          Motor0_BLDC_SCALAR_SL.rotor_last_pos = Motor0_BLDC_SCALAR_SL.rotor_pos;
        }
        else
        {
          /* BEMF is not sufficient,repeat rotor initial position identification. */
          Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_FAILED;
        }
      }
      else
      {
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter++;
      }
    }
    else
    {
      Motor0_BLDC_SCALAR_SL.first_kick_time_counter++;
    }
  }
}

/*
 * The motor is spinning slow enough for the special low speed mode to operate, but not fast enough
 * for normal zero crossing detection.In low speed mode we energize 2 phases for a fixed
 * time (e.g. 500usec) and then turn all phases off,wait a fixed time (e.g. 100 usec) for the current
 * to stop, then measure the voltage on all 3 phases.From the phase voltages we can find the rotor position
 * and approximate speed.
 */
static void BLDC_SCALAR_TRANSITION_SM_COAST_SLOW_Fun(void)
{
  uint8_t direction = (uint8_t) Motor0_BLDC_SCALAR.motor_set_direction & 8U;
  uint16_t mcmval;

  if (Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter == 0U)
  {
    /* Stop Timers */
    Motor0_BLDC_SCALAR_CCU8_PWM_Stop();

    /* New commutation occurred,check speed and direction */
    if (Motor0_BLDC_SCALAR_SL.new_commutation == 1U)
    {
      BLDC_SCALAR_CalcSpdAndChkDir();
    }

    if (Motor0_BLDC_SCALAR_SL.transition_status != BLDC_SCALAR_TRANSITION_STATUS_WRONG_DIR)
    {
      Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(
          Motor0_BLDC_SCALAR_OutPat[Motor0_BLDC_SCALAR_SL.rotor_pos + direction]);
      Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();

      mcmval = (uint16_t) XMC_POSIF_MCM_GetMultiChannelPattern((XMC_POSIF_t *)(void*)POSIF0);
      Motor0_BLDC_SCALAR_PWM_BC.immediate_modulation_ptr(mcmval);

      /* Min. Duty applied */
      Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate(Motor0_BLDC_SCALAR_SL.startup_min_amplitude);
      Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter++;

      /* Start the timers */
      Motor0_BLDC_SCALAR_CCU8_PWM_Start();
    }
    else
    {
      /* Motor spinning in wrong direction. */
      Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_WRONG_DIR;
      Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;
      /* Start the timers */
      Motor0_BLDC_SCALAR_CCU8_PWM_Start();
    }
  }
  else
  {
    if (Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter == Motor0_BLDC_SCALAR_SL.phase_energizing_time_count)
    {
      if (Motor0_BLDC_SCALAR_SL.current_decay_time_counter == 0U)
      {
        /* Turn off all switches.*/
        Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0x0U);
        Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();
      }

      if (Motor0_BLDC_SCALAR_SL.current_decay_time_counter == Motor0_BLDC_SCALAR_SL.current_decay_time_count)
      {
        /* Turn off the phases and read the back emf on 3 phases and determine rotor position*/
        Motor0_BLDC_SCALAR_SenseBEMF3();

        /* Reset counters for next iteration */
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;
        Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter = 0U;

        /* Determines rotor position and BEMF amplitude */
        Motor0_BLDC_SCALAR_CalcPosDelta();

        if (Motor0_BLDC_SCALAR_SL.bemf_amplitude < Motor0_BLDC_SCALAR_SL.bemf_min_emf)
        {
          /* BEMF is less. Configure for ADC on-chip gain. */
          Motor0_BLDC_SCALAR_SL.bemf_amplification_enabled = 1U;
          Motor0_BLDC_SCALAR_VOLT_3PHASE_BemfAmplificationEnable();

          /* Read the back emf on 3 phases */
          Motor0_BLDC_SCALAR_SenseBEMF3();

          /* Disable VADC on chip gain */
          Motor0_BLDC_SCALAR_VOLT_3PHASE_BemfAmplificationDisable();

          /* Determines rotor position and BEMF amplitude */
          Motor0_BLDC_SCALAR_CalcPosDelta();
        }
        else
        {
          /* BEMF is sufficient. */
          Motor0_BLDC_SCALAR_SL.bemf_amplification_enabled = 0U;
        }

        /*Check if BEMF is sufficient */
        if (Motor0_BLDC_SCALAR_SL.bemf_amplitude <= Motor0_BLDC_SCALAR_SL.bemf_min_emf)
        {
          /* BEMF is not sufficient,repeat initial position detection. */
          Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_FAILED;
        }
        else
        {
          /* Check if BEMF is sufficient and ready to go in close loop operation */
          if ((Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_transition_emf) &&
              (Motor0_BLDC_SCALAR_SL.startup_commutation_counter > MOTOR0_BLDC_SCALAR_SL_STARTUP_COMMUTATION_COUNT))
          {
            /* Transition completed successfully */
            /* Enable VADC for detecting BEMF zero crossing */
            BLDC_SCALAR_PrepZeroCrossingMode();
            Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_COMPLETED;
          }
        }

        /* Start the timers */
        Motor0_BLDC_SCALAR_CCU8_PWM_Start();
        /* Reset the counter for the next iteration */
        Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter = 0U;
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;
      }
      else
      {
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter++;
      }
    }
    else
    {
      Motor0_BLDC_SCALAR_SL.phase_energizing_time_counter++;
    }
  }
}

/*
 * The motor is spinning fast enough for normal zero crossing detection.This function verifies the
 * BEMF is still sufficient to go to close loop operation and motor is running in intended direction.
 * A. Motor running in wrong direction with BEMF greater than threshold BEMF then it will go to ERROR state.
 * B. Motor running in intended direction with sufficient BEMF then prepare for zero crossing and switch to
 *    close loop.
 * C. Spinning slowly then go to coast slow mode.
 * D. Motor spinning very slowly/stopped.
 */
static void BLDC_SCALAR_TRANSITION_SM_COAST_FAST_Fun(void)
{
  Motor0_BLDC_SCALAR_SL.bemf_amplification_enabled = 0U;

  /* Read the back emf on 3 phases to make sure sufficient BEMF is available */
  Motor0_BLDC_SCALAR_SenseBEMF3();

  /* Determines rotor position and BEMF amplitude */
  Motor0_BLDC_SCALAR_CalcPosDelta();

    /* Start the timers */
    Motor0_BLDC_SCALAR_CCU8_PWM_Start();
  if (Motor0_BLDC_SCALAR_SL.bemf_amplitude <= Motor0_BLDC_SCALAR_SL.bemf_min_emf)
  {
    /* D. Motor spinning very slowly/stopped. */
    /* BEMF is not sufficient,repeat initial position detection. */
    Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_FAILED;
  }
  else if (Motor0_BLDC_SCALAR_SL.bemf_amplitude < Motor0_BLDC_SCALAR_SL.bemf_transition_emf)
  {
    /*  C. Spinning slowly then go to coast slow mode.*/
    Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_COAST_SLOW;
  }
  else if (Motor0_BLDC_SCALAR_SL.new_commutation == 1U)
  {
    /* Motor spinning with sufficient BEMF */
    BLDC_SCALAR_CalcSpdAndChkDir();
    if(Motor0_BLDC_SCALAR_SL.startup_commutation_counter >= 2U)
    {
      if (Motor0_BLDC_SCALAR_SL.transition_status == BLDC_SCALAR_TRANSITION_STATUS_CORRECT_DIR)
      {
        /*  B. Motor running in intended direction with sufficient BEMF then prepare for zero
         *     crossing and switch to close loop.
         */
          Motor0_BLDC_SCALAR_SL.startup_commutation_counter = MOTOR0_BLDC_SCALAR_SL_STARTUP_COMMUTATION_COUNT + 1U;
         /* Transition completed */
          /* Enable VADC for detecting BEMF zero crossing */
          BLDC_SCALAR_PrepZeroCrossingMode();
         Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_COMPLETED;
       }
       else
       {
         /* A. Motor running in wrong direction with BEMF greater than threshold BEMF
          *    then go to ERROR state.
          */
         Motor0_BLDC_SCALAR.error_status |= (uint32_t)1 << (uint32_t)BLDC_SCALAR_EID_MOTOR_FREE_REV_RUNNING;
       }
    }
    else
    {
      /* Waiting for new commutation to determine the direction. */
    }
  }
  else
  {
    /* Waiting for new commutation to determine the direction. */
  }
}

/*
 * The motor is spinning in the wrong direction so turn on all three
 * low side MOSFETs with the configured braking duty cycle (e.g. 50%)
 * for configured braking cycle time. Once braking cycle over then it
 * verifies that motor is stopped. If motor still spinning then it again
 * applies the braking cycle.
 */
static void BLDC_SCALAR_TRANSITION_SM_WRONG_DIR_Fun(void)
{
  if (Motor0_BLDC_SCALAR_SL.transition_status == BLDC_SCALAR_TRANSITION_STATUS_WRONG_DIR)
  {
    if (Motor0_BLDC_SCALAR_SL.braking_status != BLDC_SCALAR_BRAKING_STATUS_COMPLETED)
    {
      BLDC_SCALAR_BrakeMotor();
    }
    else if (Motor0_BLDC_SCALAR_SL.braking_status == BLDC_SCALAR_BRAKING_STATUS_COMPLETED)
    {
      /* Braking cycle is completed */
      if (Motor0_BLDC_SCALAR_SL.current_decay_time_counter == 0U)
      {
        /* Turn off all switches.*/
        Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0x0U);
        Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();
      }

      if (Motor0_BLDC_SCALAR_SL.current_decay_time_counter == Motor0_BLDC_SCALAR_SL.current_decay_time_count)
      {
        /* Disable BEMF amplification */
        Motor0_BLDC_SCALAR_SL.bemf_amplification_enabled = 0U;

        /* Turn off the phases and read the back emf on 3 phases and determine rotor position*/
        Motor0_BLDC_SCALAR_SenseBEMF3();

        /* Reset counters for next iteration */
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;

        /* Determines rotor position and BEMF amplitude */
        Motor0_BLDC_SCALAR_CalcPosDelta();

        /* Check if motor is still spinning */
        if (Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_min_emf)
        {
          /* Apply next braking cycle */
          Motor0_BLDC_SCALAR_SL.braking_status = BLDC_SCALAR_BRAKING_STATUS_INIT;
        }
        else
        {
          /* Motor stopped,now repeat rotor initial position identification. */
          Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_FAILED;
        }

        /* Start the timers to go for initial rotor identification*/
        Motor0_BLDC_SCALAR_CCU8_PWM_Start();
      }
      else
      {
        /* Wait for current decay */
        Motor0_BLDC_SCALAR_SL.current_decay_time_counter++;
      }
    }
    else
    {
      /* Wait for braking cycle to complete. */
    }
  }
}


RAM_ATTRIBUTE void Motor0_BLDC_SCALAR_TransitionStateMachine(void)
{
  switch (Motor0_BLDC_SCALAR_SL.transition_sm_state)
  {
    case BLDC_SCALAR_TRANSITION_SM_FIRST_KICK:
      BLDC_SCALAR_TRANSITION_SM_FIRST_KICK_Fun();
      break;

    case BLDC_SCALAR_TRANSITION_SM_COAST_SLOW:
      BLDC_SCALAR_TRANSITION_SM_COAST_SLOW_Fun();
      break;

    case BLDC_SCALAR_TRANSITION_SM_COAST_FAST:
      BLDC_SCALAR_TRANSITION_SM_COAST_FAST_Fun();
      break;

    case BLDC_SCALAR_TRANSITION_SM_WRONG_DIR:
      BLDC_SCALAR_TRANSITION_SM_WRONG_DIR_Fun();
      break;

    default:
      break;
  }
}



#endif /*end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */
