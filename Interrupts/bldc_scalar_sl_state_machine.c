/**
 * @file bldc_scalar_state_machine.c
 * @brief system timer event used for state machine
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
 * 2016-09-08:
 *     - Initial version
 *
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/

#include "../ControlModule/bldc_scalar_control_scheme.h"
#include "../ControlModule/bldc_scalar_control_sensorless.h"
#include "../ControlModule/bldc_scalar_inductive_sensing.h"
#if(MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1)
#include "../uCProbe/uCProbe.h"
#endif
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)

/*********************************************************************************************************************
 * MACROS
 * *******************************************************************************************************************/
#define BLDC_SCALAR_BOOTSTRAP_CMP_VAL           (0xFFFFU)
#define BLDC_SCALAR_ALIGN_POS_POSITIVE_DIR       (3U)
#define BLDC_SCALAR_ALIGN_POS_NEGATIVE_DIR       (10U)
#define BLDC_SCALAR_STARTUP_FAILURE_MAX_COUNT    (20U)
/*********************************************************************************************************************
 * Local API's
 * ******************************************************************************************************/
/* Function for the state - START */
static void Motor0_BLDC_SCALAR_MSM_START_Func(void);
/* Exit Function for the state - START */
static void Motor0_BLDC_SCALAR_MSM_START_Exit_Func(void);


/* Entry Function for the state - IDLE */
static void Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func(void);
/* Function for the state - IDLE */
static void Motor0_BLDC_SCALAR_MSM_IDLE_Func(void);
/* Exit Function for the state - IDLE */
static void Motor0_BLDC_SCALAR_MSM_IDLE_Exit_Func(void);


/* Entry Function for the state - NORMAL_OPERATION */
static void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Entry_Func(void);
/* Function for the state - NORMAL_OPERATION */
static void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Func(void);


static void Motor0_BLDC_SCALAR_MSM_ERROR_Func(void);

/* Entry Function for the state - STATE_IDENTIFICATION */
static void Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Entry_Func(void);
/* Function for the state - STATE_IDENTIFICATION */
static void Motor0_BLDC_SCALAR_MSM_MOTOR_STATE_IDENTIFICATION_Func(void);
/* Exit Function for the state - STATE_IDENTIFICATION */
static void Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Exit_Func(void);


/* Entry Function for the state - ROTOR_POSITION_IDENTIFICATION */
static void Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Entry_Func(void);
/* Function for the state - ROTOR_POSITION_IDENTIFICATION */
static void Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Func(void);
/* Exit Function for the state - ROTOR_POSITION_IDENTIFICATION */
static void Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Exit_Func(void);

/* Entry Function for the state - TRANSITION */
static void Motor0_BLDC_SCALAR_MSM_TRANSITION_Entry_Func(void);
/* Function for the state - TRANSITION */
static void Motor0_BLDC_SCALAR_MSM_TRANSITION_Func(void);
/* Exit Function for the state - TRANSITION */
static void Motor0_BLDC_SCALAR_MSM_TRANSITION_Exit_Func(void);

#if (MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U)
/* Entry Function for the state - BOOTSTRAP */
static void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func(void);
/* Function for the state - BOOTSTRAP */
static void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Func(void);
/* Exit Function for the state - BOOTSTRAP */
static void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func(void);
#endif


/**********************************************************************************************************************
* API IMPLEMENTATION
**********************************************************************************************************************/
/**
 * @addtogroup BLDC_SCALAR BLDC Motor Control
 * @{
 */

/**
 * @addtogroup Interrupt Interrupts
 * @brief  Interrupt Service Routines  <br>
 * @{
 */
/**
 * @param None
 * @return None <br>
 *
 * \par<b>Description:</b><br>
 * Systick event handler for motor state machine \n
 * This is the lowest priority interrupt which handles the state transitions and also
 * performs less time critical tasks like ramp, potentiometer reading.\n
 */
void SysTick_Handler(void)
{
  /* Call motor control state machine */
  Motor0_BLDC_SCALAR_MSM();
}

/**
 * @}
 */
/**
 * @}
 */

RAM_ATTRIBUTE void Motor0_BLDC_SCALAR_MSM(void)
{
  switch (Motor0_BLDC_SCALAR.msm_state)
  {
    case BLDC_SCALAR_MSM_NORMAL_OPERATION:
      Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Func();
      break;

    case BLDC_SCALAR_MSM_IDLE:
      Motor0_BLDC_SCALAR_MSM_IDLE_Func();
      break;

    case BLDC_SCALAR_MSM_MOTOR_STATE_IDENTIFICATION:
      Motor0_BLDC_SCALAR_MSM_MOTOR_STATE_IDENTIFICATION_Func();
      break;

    case BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION:
      Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Func();
      break;

    case BLDC_SCALAR_MSM_TRANSITION:
    	Motor0_BLDC_SCALAR_MSM_TRANSITION_Func();
      break;

#if (MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U)
    case BLDC_SCALAR_MSM_BOOTSTRAP:
      Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Func();
      break;
#endif

    case BLDC_SCALAR_MSM_ERROR:
      Motor0_BLDC_SCALAR_MSM_ERROR_Func();
      break;

    case BLDC_SCALAR_MSM_START:
      Motor0_BLDC_SCALAR_MSM_START_Func();
      break;

    case BLDC_SCALAR_MSM_STOP:
      Motor0_BLDC_SCALAR_MotorStop();
      break;

    default:
      break;

  }

  if (Motor0_BLDC_SCALAR.error_status != 0U)
  {
    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_ERROR;
  }
#if (MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE == 1)
Motor0_BLDC_SCALAR_uCProbe_Scheduler();
#endif
}

/****************************start: BLDC_SCALAR_MSM_START ***********************/
/**
 * START state:
 * Initialize Motor control variables
 * Enable inverter
 * Enable interrupt for ctrap and protection
 */
static void Motor0_BLDC_SCALAR_MSM_START_Func(void)
{
  /*Initialize all run time parameters*/
  Motor0_BLDC_SCALAR_MotorParamInit();
#if (MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF != BLDC_SCALAR_INV_DISABLED)
  /*Enable inverter*/
  Motor0_BLDC_SCALAR_PWM_BC_InverterEnable();
#endif

#if (MOTOR0_BLDC_SCALAR_ENABLE_CTRAP == 1U)
  /* Clear if any pending trap events and enable the trap */
  Motor0_BLDC_SCALAR_PWM_BC_ClearTrapEvent();
  Motor0_BLDC_SCALAR_PWM_BC_EnableTrap();
#endif
  /* ISR Init for trap and startup algorithm */
  BLDC_SCALAR_NVIC_NodeInit(MOTOR0_BLDC_SCALAR_CTRAP_NODE, MOTOR0_BLDC_SCALAR_TRAP_NVIC_PRIO);

#if ((MOTOR0_BLDC_SCALAR_ENABLE_OVER_CURRENT == 1U))
  /* ISR Init for VADC channel event handler*/
  BLDC_SCALAR_NVIC_NodeInit(MOTOR0_BLDC_SCALAR_VADC_PROTECTION_NODE, MOTOR0_BLDC_SCALAR_VADC_PROTECTION_NVIC_PRIO);
#endif

  Motor0_BLDC_SCALAR_MSM_START_Exit_Func();
}

/*
 * START exit:
 * Next state
 * STATE_IDENTIFICATION
 */
static void Motor0_BLDC_SCALAR_MSM_START_Exit_Func(void)
{
  int32_t reference_set;
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  reference_set = Motor0_BLDC_SCALAR.analogip_val * Motor0_BLDC_SCALAR.motor_set_direction;

#else /* Potentiometer is not enabled */
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  reference_set = Motor0_BLDC_SCALAR_SpeedControl.user_speed_set * Motor0_BLDC_SCALAR.motor_set_direction;
  #endif

  #if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL)
  reference_set = Motor0_BLDC_SCALAR_CurrentControl.user_current_set * Motor0_BLDC_SCALAR.motor_set_direction;
  #endif

  #if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_VOLTAGE_CTRL)
  reference_set = Motor0_BLDC_SCALAR_VoltageControl.user_voltage_set * Motor0_BLDC_SCALAR.motor_set_direction;
  #endif
#endif /* end of #if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U) */

  if (reference_set > (int32_t)Motor0_BLDC_SCALAR_SL.min_threshold)
  {
    Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Entry_Func();
  }
  else
  {
    /* Start CCU8 for potentiometer measurement*/
    Motor0_BLDC_SCALAR_CCU8_PWM_Start();
    Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func();
  }

}

/****************************end: BLDC_SCALAR_MSM_START ***********************/

/****************************start: BLDC_SCALAR_MSM_NORMAL_OPERATION ***********************/
/*
 * NORMAL_OPERATION entry:
 * Prepare VADC for BEMF zero crossing event detection
 * Start the CCU8 for PWM
 * Disable Phase V period match event
 * Enable CCU8 one match event
 */

static void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Entry_Func(void)
{
#if(MOTOR0_BLDC_SCALAR_CONTROL_LOOP_EXECUTION_RATE == 1U)
  /* Disable slice event */
  Motor0_BLDC_SCALAR_PWM_BC_DisableEvent(Motor0_BLDC_SCALAR_CCU8_PWM_Config.phase_ptr[1U], XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
  XMC_CCU8_SLICE_ClearEvent(Motor0_BLDC_SCALAR_CCU8_PWM_Config.phase_ptr[1U],XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH);
#endif
  /* Start POSIF */
  Motor0_BLDC_SCALAR_SPEED_POS_SL_POSIF_Start();
  /*Clear CCU4 timers*/
  XMC_CCU4_SLICE_ClearTimer(MOTOR0_BLDC_SCALAR_CCU4_COMMUTATION_TMR_SLICE);
  XMC_CCU4_SLICE_ClearTimer(MOTOR0_BLDC_SCALAR_CCU4_SPEEDCAP_SLICE);

  NVIC_ClearPendingIRQ(MOTOR0_BLDC_SCALAR_CTRL_LOOP_NODE);
  NVIC_ClearPendingIRQ(MOTOR0_BLDC_SCALAR_CTRAP_NODE);
  XMC_CCU8_SLICE_ClearEvent(Motor0_BLDC_SCALAR_CCU8_PWM_Config.phase_ptr[0U],XMC_CCU8_SLICE_IRQ_ID_ONE_MATCH);
  /* ISR Init for control loop */
  BLDC_SCALAR_NVIC_NodeInit(MOTOR0_BLDC_SCALAR_CTRL_LOOP_NODE, MOTOR0_BLDC_SCALAR_CTRL_LOOP_NVIC_PRIO);

  /* Clear the ADC NVIC node*/
  NVIC_ClearPendingIRQ(MOTOR0_BLDC_SCALAR_ZERO_CROSS_NODE);
  /* Enable the ADC channel interrupt   */
  BLDC_SCALAR_NVIC_NodeInit(MOTOR0_BLDC_SCALAR_ZERO_CROSS_NODE, MOTOR0_BLDC_SCALAR_ZERO_CROSS_NVIC_PRIO);
  /* Reset startup failure counter for next iteration. */
  Motor0_BLDC_SCALAR_SL.startup_failure_counter = 0U;

  Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_NORMAL_OPERATION;

  /* Start CCU8 */
  Motor0_BLDC_SCALAR_CCU8_PWM_Start();
}

/*
 * NORMAL_OPERATION state:
 * Potentiometer measurement
 * Ramp
 * Stall detection
 */

static void Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Func(void)
{
  int32_t temp_set_val;
#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U)
  int32_t setval_diff;          /* difference between ramp input and output value */
#endif

#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  /* Potentiometer measurement */
  Motor0_BLDC_SCALAR_AnalogIpMeasurement();
#endif

  /************************** Ramp start**************************/
#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U)
  Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_Ramp.set_value;
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR.analogip_val;
#else  /* potentiometer disabled */
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /* Speed control */
    Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR_SpeedControl.user_speed_set;
  #elif ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
    /* Current control */
    Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR_CurrentControl.user_current_set;
  #else
    /* Voltage control */
    Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR_VoltageControl.user_voltage_set;
  #endif

#endif /* end of #if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U) */

  setval_diff = ((Motor0_BLDC_SCALAR_Ramp.input_value) - Motor0_BLDC_SCALAR_Ramp.set_value) *
                 Motor0_BLDC_SCALAR.motor_set_direction;

  /* Find whether ramp up or down is required based on set value */
  if (setval_diff < 0)
  {
    Motor0_BLDC_SCALAR_Ramp.ramp_rate = Motor0_BLDC_SCALAR_Ramp.ramp_down_rate;
  }
  else
  {
    Motor0_BLDC_SCALAR_Ramp.ramp_rate = Motor0_BLDC_SCALAR_Ramp.ramp_up_rate;
  }

  #if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_VDC_LINK_MEASUREMENT == 1U) && (MOTOR0_BLDC_SCALAR_ENABLE_RAMP_DOWN_VOLTAGE_CLAMPING == 1U))
  if (setval_diff < 0)
  {
    /* Ramp down - If DC Link voltage within threshold limit then only ramp down */
    if (Motor0_BLDC_SCALAR.dclink_voltage < Motor0_BLDC_SCALAR.max_dc_link_voltage)
    {
      Motor0_BLDC_SCALAR_Ramp_Linear();
    }
  }
  else
  {
    Motor0_BLDC_SCALAR_Ramp_Linear();
  }
  #else /* #if(MOTOR0_BLDC_SCALAR_VADC_ENABLE_VDC_LINK_MEASUREMENT == 1U) */

  Motor0_BLDC_SCALAR_Ramp_Linear();
  #endif
#else /*#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U) */
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR.analogip_val;
#else  /* potentiometer disabled */
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /* Speed control */
    Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_SpeedControl.user_speed_set;
  #elif ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
    /* Current control */
    Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_CurrentControl.user_current_set;
  #else
    /* Voltage control */
    Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_VoltageControl.user_voltage_set;
  #endif

#endif /* end of #if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U) */
#endif /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U) */

  /************************** Ramp end**************************/
  /*************************IDLE condition check*************************/
  /* Switch off all the PWM outputs when reference and duty is less than threshold */
  if (Motor0_BLDC_SCALAR.set_val < 0)
  {
    temp_set_val = (-Motor0_BLDC_SCALAR.set_val);
  }
  else
  {
    temp_set_val = Motor0_BLDC_SCALAR.set_val;
  }
  if (((uint32_t)temp_set_val < Motor0_BLDC_SCALAR_SL.min_threshold))
  {
    if (Motor0_BLDC_SCALAR.amplitude < Motor0_BLDC_SCALAR_SL.min_threshold)
    {
      Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func();
    }
  }
}
/****************************end: BLDC_SCALAR_MSM_NORMAL_OPERATION ***********************/



/****************************start: BLDC_SCALAR_MSM_STATE_IDENTIFICATION****************/
/*
 * STATE_IDENTIFICATION entry:
 * Add BEMF phase voltage channels to scan request source
 * Start POSIF
 */

static void Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Entry_Func(void)
{
  /* Start POSIF and clear speed capture timer */
  Motor0_BLDC_SCALAR_SPEED_POS_SL_Start();

  /* Add 3 phase channels to scan sequence*/
  Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanAddChannelToSequence((XMC_VADC_GROUP_t *)(void*)MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEU_VOLTAGE_CH_NUM);
  Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanAddChannelToSequence((XMC_VADC_GROUP_t *)(void*)MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEV_VOLTAGE_CH_NUM);
  Motor0_BLDC_SCALAR_VOLT_3PHASE_ScanAddChannelToSequence((XMC_VADC_GROUP_t *)(void*)MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_GRP, MOTOR0_BLDC_SCALAR_VADC_PHASEW_VOLTAGE_CH_NUM);

  /*Change Motor Control State to motor state identification */
  Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_MOTOR_STATE_IDENTIFICATION;
}

/*
 * STATE_IDENTIFICATION state:
 * Measure 3 phase BEMF voltages
 */
static void Motor0_BLDC_SCALAR_MSM_MOTOR_STATE_IDENTIFICATION_Func(void)
{
  Motor0_BLDC_SCALAR_State_Identification();

  /* Initial state identification completed */
  Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Exit_Func();
}

/*
 * STATE_IDENTIFICATION exit:
 * Next state:
 * TRANSITION: If motor is spinning with sufficient BEMF
 * BOOTSTRAP: If motor is slowly running with BEMF not more than ZERO_BEMF and bootstrap is enabled
 * ROTOR_POSITION_IDENTIFICATION : If motor not spinning.
 */
static void Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Exit_Func(void)
{
  if (Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_transition_emf)
  {
    /* Motor spinning with sufficient BEMF. Go to transition state COAST_FAST to determine direction
     * and prepare for zero crossing detection */
     Motor0_BLDC_SCALAR_MSM_TRANSITION_Entry_Func();
     Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_COAST_FAST;
  }
  else if (Motor0_BLDC_SCALAR_SL.bemf_amplitude > Motor0_BLDC_SCALAR_SL.bemf_min_emf)
  {
  /* Motor spinning with less BEMF. Go to transition state COAST_SLOW to determine direction.
   * Build the sufficient BEMF to go into close loop */
   Motor0_BLDC_SCALAR_MSM_TRANSITION_Entry_Func();
   Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_COAST_SLOW;
  }
  else
  {
    /* Motor spinning very slowly/stopped */
    /* Current amplifier bias voltage calibration*/
    #if (MOTOR0_BLDC_SCALAR_ENABLE_AMPLIFIER_OFFSET_CALIBRATION == 1U)
    Motor0_BLDC_SCALAR_AmpBiasVoltCalibration();
    #endif

    #if (MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U)
    /*Bootstrap Configuration */
    Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func();
    #else
    Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Entry_Func();
    #endif
  }
}

/****************************end: BLDC_SCALAR_MSM_STATE_IDENTIFICATION****************/

/****************************Start: BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION****************/
/*
 * ROTOR_POSITION_IDENTIFICATION entry:
 * Stop CCU8 PWM
 * Enable Phase V period match event
 * Start CCu8 PWM
 */
static void Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Entry_Func(void)
{
  /* Stop the timers if running */
  Motor0_BLDC_SCALAR_CCU8_PWM_Stop();

  /*
   * Initialize Phase-V period match event and Bind Phase-U period match event SR(service request)
   * to interrupt node for trap.
   */
  BLDC_SCALAR_CCU8_Event_Init(Motor0_BLDC_SCALAR_CCU8_PWM_Config.phase_ptr[1U],
  (XMC_CCU8_SLICE_IRQ_ID_t)XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, (XMC_CCU8_SLICE_SR_ID_t)MOTOR0_BLDC_SCALAR_CCU8_CTRAP_EVT2_SR);

  Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION;
  Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_FIRST_KICK;
  Motor0_BLDC_SCALAR_SL.braking_status = BLDC_SCALAR_BRAKING_STATUS_INIT;

  /* Braking with 100% duty by turning on all 3 bottom switches */
  Motor0_BLDC_SCALAR_SL.braking_amplitude_cmp_val = Motor0_BLDC_SCALAR.period + 1U;

  #if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
  /* Reset inductive sensing related variables */
  Motor0_BLDC_SCALAR_InductiveSensing.status = BLDC_SCALAR_INDUCTIVE_SENSING_STATUS_UNKNOWN;
  Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pat_index = 0U;
  Motor0_BLDC_SCALAR_InductiveSensing.read_current_enable = 0U;
  #endif

  Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status = BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_UNKNOWN;
  Motor0_BLDC_SCALAR.amplitude = (uint32_t)0;

  /* Start Timers */
  Motor0_BLDC_SCALAR_PWM_BC_Start();
}

/*
 * ROTOR_POSITION_IDENTIFICATION:
 * Apply brake before starting rotor position identification or alignment
 * Inductive sensing to identify the rotor position OR align to V+W- position
 *
 */
static void Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Func()
{
#if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_ALIGN)
  uint16_t mcmval;            /* multi-channel pattern */
#endif

#if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
  /* Apply Brake before going for initial rotor position identification */
  if ((Motor0_BLDC_SCALAR_SL.braking_status == BLDC_SCALAR_BRAKING_STATUS_COMPLETED) &&
      (Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status == BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_UNKNOWN))
  {
    /* Setup CCU8 for inductance sensing */
    Motor0_BLDC_SCALAR_InductiveSensing_Init();
  }
  else if(BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_SUCCESS == Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status)
  {
    Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Exit_Func();
  }
  else
  {
    /* Braking / rotor initial position identification is in progress */
  }
#endif  /* end of #if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING) */

#if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_ALIGN)
  if ((Motor0_BLDC_SCALAR_SL.braking_status == BLDC_SCALAR_BRAKING_STATUS_COMPLETED) &&
      (Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status == BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_UNKNOWN))
  {
    /* Load fixed multi-channel pattern V+W- for alignment */
    Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(Motor0_BLDC_SCALAR_OutPat[0x1U]);
    Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();
    mcmval = (uint16_t)Motor0_BLDC_SCALAR_SPEED_POS_SL_GetMultiChannelPattern();
    Motor0_BLDC_SCALAR_PWM_BC.immediate_modulation_ptr(mcmval);
    Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status = BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_IN_PROGRESS;
  }
  else if(BLDC_SCALAR_ROTOR_INIT_POS_IDENTIFICATION_STATUS_IN_PROGRESS == Motor0_BLDC_SCALAR_SL.rotor_init_pos_identifcation_status)
  {
    /* Ramp to the align voltage as per configured ramp rate */
    if (Motor0_BLDC_SCALAR_Align_Ramp.set_value < Motor0_BLDC_SCALAR_SL.align_duty)
    {
      Motor0_BLDC_SCALAR_Align_Ramp_Linear();
      /* Update the CCU8 compare values */
      Motor0_BLDC_SCALAR_PWM_BC_DutyCycleUpdate((uint16_t)Motor0_BLDC_SCALAR_Align_Ramp.set_value);
    }
    else
    {
      /* Align the rotor for configured time */
      Motor0_BLDC_SCALAR_SL.align_counter++;
      if (Motor0_BLDC_SCALAR_SL.align_counter >= Motor0_BLDC_SCALAR_SL.align_count)
      {
        Motor0_BLDC_SCALAR_SL.align_counter = 0U;
        Motor0_BLDC_SCALAR_Align_Ramp.set_value = 0;
        /* Update the rotor initial position as per aligned pattern V+W- */
        if (Motor0_BLDC_SCALAR.motor_set_direction == BLDC_SCALAR_POSITIVE_DIR)
        {
          Motor0_BLDC_SCALAR_SL.rotor_init_pos = Motor0_BLDC_SCALAR_NextHallPat[BLDC_SCALAR_ALIGN_POS_POSITIVE_DIR];
        }
        else
        {
          Motor0_BLDC_SCALAR_SL.rotor_init_pos = Motor0_BLDC_SCALAR_NextHallPat[BLDC_SCALAR_ALIGN_POS_NEGATIVE_DIR];
        }
        Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Exit_Func();
      }
    }
  }
#endif  /* end of #if(MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_ALIGN) */
}

/*
 * ROTOR_POSITION_IDENTIFICATION exit:
 * Next state:
 * TRANSITION: with sub-state as FIRST-KICK
 */
static void Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Exit_Func()
{
  /* Go to transition state */
  Motor0_BLDC_SCALAR_MSM_TRANSITION_Entry_Func();

  /* First kick in transition state machine to be applied */
  Motor0_BLDC_SCALAR_SL.transition_sm_state = BLDC_SCALAR_TRANSITION_SM_FIRST_KICK;
}

/****************************end: BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION****************/

/****************************Start: BLDC_SCALAR_MSM_TRANSITION ********************************/
/*
 * TRANSITION entry:
 * Stop CCU8 and initialize for closed loop operation
 * Enable phase V period match event
 * initialize first kick and startup variables
 */
static void Motor0_BLDC_SCALAR_MSM_TRANSITION_Entry_Func()
{
  /* Stop the PWM */
  Motor0_BLDC_SCALAR_CCU8_PWM_Stop();

  /* Re - initialize PWM timer slices for normal operation */
  Motor0_BLDC_SCALAR_CCU8_PWM_Init();

#if (MOTOR0_BLDC_SCALAR_ENABLE_CTRAP == 1U)
  Motor0_BLDC_SCALAR_PWM_BC_EnableTrap();
#endif

  NVIC_ClearPendingIRQ(MOTOR0_BLDC_SCALAR_CTRAP_NODE);

  /*
   * Initialize Phase-V period match event and Bind Phase-V period match event SR(service request)
   * to interrupt node for trap.
   */
  BLDC_SCALAR_CCU8_Event_Init(Motor0_BLDC_SCALAR_CCU8_PWM_Config.phase_ptr[1U],
  (XMC_CCU8_SLICE_IRQ_ID_t)XMC_CCU8_SLICE_IRQ_ID_PERIOD_MATCH, (XMC_CCU8_SLICE_SR_ID_t)MOTOR0_BLDC_SCALAR_CCU8_CTRAP_EVT2_SR);

  /* Limit the minimum duty for transition */
  Motor0_BLDC_SCALAR.min_amplitude = Motor0_BLDC_SCALAR_SL.startup_min_amplitude;
  Motor0_BLDC_SCALAR_SL.braking_amplitude_cmp_val = Motor0_BLDC_SCALAR_SL.configured_braking_amplitude_cmp_val;

  /* Reset counters for next iteration */
  Motor0_BLDC_SCALAR_SL.first_kick_time_counter = 0U;
  Motor0_BLDC_SCALAR_SL.current_decay_time_counter = 0U;

  /* Reset current control variables */
  Motor0_BLDC_SCALAR_StartupCurrentControl_PI.ik = 0;
  Motor0_BLDC_SCALAR_StartupCurrentControl_PI.uk = 0;

  /* Change state to transition */
  Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_TRANSITION;
  Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_UNKNOWN;

  /* Start the PWM */
  Motor0_BLDC_SCALAR_CCU8_PWM_Start();
}

/*
 * TRANSITION:
 * Wait for the first kick and startup to finish and check the status
 */
static void Motor0_BLDC_SCALAR_MSM_TRANSITION_Func()
{
  if (Motor0_BLDC_SCALAR_SL.transition_status == BLDC_SCALAR_TRANSITION_STATUS_COMPLETED)
  {
    /* Transition completed successfully.Ready to go to normal operation */
    Motor0_BLDC_SCALAR_MSM_TRANSITION_Exit_Func();
  }
  else if (Motor0_BLDC_SCALAR_SL.transition_status == BLDC_SCALAR_TRANSITION_STATUS_FAILED)
  {
    if (Motor0_BLDC_SCALAR.error_status == ((uint32_t)1 << BLDC_SCALAR_EID_MOTOR_FREE_REV_RUNNING))
    {
      /* Motor freely running in wrong direction with speed greater than threshold.
       * Change MSM state to ERROR */
      Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_ERROR;
      Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_UNKNOWN;
    }
    else
    {
      /* Transition failed due to insufficient BEMF,Repeat initial rotor position identification */
      Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Entry_Func();
      if (Motor0_BLDC_SCALAR_SL.startup_failure_counter > BLDC_SCALAR_STARTUP_FAILURE_MAX_COUNT)
      {
        /* Startup failure for multiple times.
         * Change MSM state to ERROR */
        Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_ERROR;
        Motor0_BLDC_SCALAR.error_status = ((uint32_t)1 << BLDC_SCALAR_EID_STARTUP_FAILURE);
        Motor0_BLDC_SCALAR_SL.transition_status = BLDC_SCALAR_TRANSITION_STATUS_UNKNOWN;
        Motor0_BLDC_SCALAR_MotorStop();
      }
      else
      {
        Motor0_BLDC_SCALAR_SL.startup_failure_counter++;
      }
    }
  }
  else
  {
    /* Transition is in progress */
  }
}

/*
 * TRANSITION exit:
 * Next state:
 * NORMAL_OPERATION: if transition is successful
 */
static void Motor0_BLDC_SCALAR_MSM_TRANSITION_Exit_Func()
{
  Motor0_BLDC_SCALAR_MSM_NORMAL_OPERATION_Entry_Func();
}
/****************************End: BLDC_SCALAR_MSM_TRANSITION **********************************/


/****************************start: BLDC_SCALAR_MSM_BOOTSTRAP****************/
/*
 * BOOTSTRAP entry:
 * Compare value = 0xFFFF
 * Stop POSIF
 * Disable hall event
 * Start CCU8
 */
#if (MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U)

/* Initialize CCU8 slices for bootstrap */
static void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Entry_Func(void)
{
  /* Update channel 1 compare value (CR1) of slice */
  Motor0_BLDC_SCALAR_PWM_BC_SetCompareValPhaseU((uint16_t)BLDC_SCALAR_BOOTSTRAP_CMP_VAL);
  /* Update channel 1 compare value (CR1) of slice */
  Motor0_BLDC_SCALAR_PWM_BC_SetCompareValPhaseV((uint16_t)BLDC_SCALAR_BOOTSTRAP_CMP_VAL);
  /* Update channel 1 compare value (CR1) of slice */
  Motor0_BLDC_SCALAR_PWM_BC_SetCompareValPhaseW((uint16_t)BLDC_SCALAR_BOOTSTRAP_CMP_VAL);

  /*Enable the shadow transfer for all three consumed slice*/
  Motor0_BLDC_SCALAR_PWM_BC_EnableShadowTransfer();

  if (Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_MOTOR_STATE_IDENTIFICATION)
  {
    /* Stop POSIF */
    XMC_POSIF_Stop((XMC_POSIF_t *)(void*)MOTOR0_BLDC_SCALAR_POSIF_MODULE);
    /* ISR Init for wrong hall event and correct hall event handler*/
  }

  Motor0_BLDC_SCALAR_CCU8_PWM_Start();

  /*Change Motor Control State Machine to Boot Strap*/
  Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_BOOTSTRAP;

}

/*
 * BOOTSTRAP state:
 * Wait for the bootstrap time
 */
static void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Func(void)
{
  if (BLDC_SCALAR_BOOTSTRAP_COMPLETED == Motor0_BLDC_SCALAR_Bootstrap())
  {
    Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func();
  }
}

/*
 * BOOTSTRAP exit:
 * Next state:
 * ROTOR_POSITION_IDENTIFICATION: After bootstrapping completion
 */
static void Motor0_BLDC_SCALAR_MSM_BOOTSTRAP_Exit_Func(void)
{
  /* Change to initial rotor identification state */
  Motor0_BLDC_SCALAR_MSM_ROTOR_POSITION_IDENTIFICATION_Entry_Func();
}
#endif
/****************************end: BLDC_SCALAR_MSM_BOOTSTRAP****************/

/****************************start: BLDC_SCALAR_MSM_ERROR****************/
/*
 * ERROR state:
 * Change the state to STOP when all the errors are cleared
 */
static void Motor0_BLDC_SCALAR_MSM_ERROR_Func(void)
{
/* Change the state to STOP when all the errors are cleared */
  if (Motor0_BLDC_SCALAR.error_status == 0U)
  {
    Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_STOP;
  }
}

/****************************end: BLDC_SCALAR_MSM_ERROR****************/

/****************************start: BLDC_SCALAR_MSM_IDLE****************/
/*
 * IDLE entry:
 * Multi-channel pattern as 0x00
 * Stop the POSIF
 * Disable VADC channel event
 * Disable CCU8 one match event
 */
static void Motor0_BLDC_SCALAR_MSM_IDLE_Entry_Func(void)
{
  Motor0_BLDC_SCALAR.motor_speed = 0;
  Motor0_BLDC_SCALAR.motor_current = 0;
  Motor0_BLDC_SCALAR.motor_average_current = 0;

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
  /* Disable CCU8 outputs by applying multi-channel pattern as 0 */
  Motor0_BLDC_SCALAR_SPEED_POS_SL_SetMultiChannelPattern(0x0U);
  /*Immediate Shadow transfer update of multi-channel pattern*/
  Motor0_BLDC_SCALAR_SPEED_POS_SL_UpdateMultiChannelPattern();

  /* Stop POSIF */
  Motor0_BLDC_SCALAR_SPEED_POS_SL_Stop();
#endif
  /*Disable Inverter*/
#if (MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF != BLDC_SCALAR_INV_DISABLED)
  Motor0_BLDC_SCALAR_PWM_BC_InverterDisable();
#endif

  NVIC_ClearPendingIRQ(MOTOR0_BLDC_SCALAR_ZERO_CROSS_NODE);
  /* ISR disable for wrong hall event and correct hall event handler*/
  NVIC_DisableIRQ(MOTOR0_BLDC_SCALAR_ZERO_CROSS_NODE);

  NVIC_DisableIRQ(MOTOR0_BLDC_SCALAR_CTRL_LOOP_NODE);

  Motor0_BLDC_SCALAR.msm_state = BLDC_SCALAR_MSM_IDLE;

}

/*
 * IDLE state:
 * Potentiometer measurement to update the ref
 * Ramp to update the ref
 */
static void Motor0_BLDC_SCALAR_MSM_IDLE_Func(void)
{
  int32_t temp_set_val;
#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U)
  int32_t setval_diff;          /* difference between ramp input and output value */
#endif
  Motor0_BLDC_SCALAR.motor_speed = 0;
  Motor0_BLDC_SCALAR.motor_current = 0;
  Motor0_BLDC_SCALAR.motor_average_current = 0;
  Motor0_BLDC_SCALAR.amplitude = 0U;

#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  Motor0_BLDC_SCALAR_AnalogIpMeasurement();
#endif

  /************************** Ramp start**************************/
#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U)
  Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_Ramp.set_value;
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR.analogip_val;
#else  /* potentiometer disabled */
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /* Speed control */
    Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR_SpeedControl.user_speed_set;
  #elif ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
    /* Current control */
    Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR_CurrentControl.user_current_set;
  #else
    /* Voltage control */
    Motor0_BLDC_SCALAR_Ramp.input_value = Motor0_BLDC_SCALAR_VoltageControl.user_voltage_set;
  #endif

#endif /* end of #if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U) */

  setval_diff =  ((Motor0_BLDC_SCALAR_Ramp.input_value) - Motor0_BLDC_SCALAR_Ramp.set_value) * Motor0_BLDC_SCALAR.motor_set_direction;

  /* Find whether ramp up or down is required based on set value */
  if (setval_diff < 0)
  {
    Motor0_BLDC_SCALAR_Ramp.ramp_rate = Motor0_BLDC_SCALAR_Ramp.ramp_down_rate;
  }
  else
  {
    Motor0_BLDC_SCALAR_Ramp.ramp_rate = Motor0_BLDC_SCALAR_Ramp.ramp_up_rate;
  }

  #if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_VDC_LINK_MEASUREMENT == 1U) && (MOTOR0_BLDC_SCALAR_ENABLE_RAMP_DOWN_VOLTAGE_CLAMPING == 1U))
  if (setval_diff < 0)
  {
    /* Ramp down - If DC Link voltage within threshold limit then only ramp down */
    if (Motor0_BLDC_SCALAR.dclink_voltage < Motor0_BLDC_SCALAR.max_dc_link_voltage)
    {
      Motor0_BLDC_SCALAR_Ramp_Linear();
    }
  }
  else
  {
    Motor0_BLDC_SCALAR_Ramp_Linear();
  }
  #else /* #if(MOTOR0_BLDC_SCALAR_VADC_ENABLE_VDC_LINK_MEASUREMENT == 1U) */
  Motor0_BLDC_SCALAR_Ramp_Linear();
  #endif
#else /*#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U) */
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
  Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR.analogip_val;
#else  /* potentiometer disabled */
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /* Speed control */
    Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_SpeedControl.user_speed_set;
  #elif ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
    /* Current control */
    Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_CurrentControl.user_current_set;
  #else
    /* Voltage control */
    Motor0_BLDC_SCALAR.set_val = Motor0_BLDC_SCALAR_VoltageControl.user_voltage_set;
  #endif

#endif /* end of #if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U) */
#endif /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP == 1U) */
  /************************** Ramp end**************************/

  if (Motor0_BLDC_SCALAR.set_val < 0)
  {
    temp_set_val = (- Motor0_BLDC_SCALAR.set_val);
  }
  else
  {
    temp_set_val = Motor0_BLDC_SCALAR.set_val;
  }
  if (((uint32_t)temp_set_val > Motor0_BLDC_SCALAR_SL.min_threshold))
  {
    Motor0_BLDC_SCALAR_MSM_IDLE_Exit_Func();
  }

}

/*
 * IDLE exit:
 * Reset speed calculation related variables
 * Reset PI buffer
 * Next state:
 * NORMAL_OPERATION
 */
static void Motor0_BLDC_SCALAR_MSM_IDLE_Exit_Func(void)
{

  /*Initialize all run time parameters*/
  Motor0_BLDC_SCALAR_MotorParamInit();

  /* Reset speed calculation related variables */
  Motor0_BLDC_SCALAR_SPEED_POS_SL_ClearSpeedFilter();
  Motor0_BLDC_SCALAR_StartupCurrentControl_PI.ik = 0;
  Motor0_BLDC_SCALAR_StartupCurrentControl_PI.sat_state = 1U;

  /* Reset the PI integral buffer */
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  Motor0_BLDC_SCALAR_SpeedControl_PI.ik = 0;
  Motor0_BLDC_SCALAR_SpeedControl_PI.sat_state = 1U;
#endif

#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  Motor0_BLDC_SCALAR_CurrentControl_PI.ik = 0;
  Motor0_BLDC_SCALAR_CurrentControl_PI.sat_state = 1U;
#endif

#if (MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF != BLDC_SCALAR_INV_DISABLED)
  /*Enable inverter*/
    Motor0_BLDC_SCALAR_PWM_BC_InverterEnable();
#endif

  /* Change the state to state identification */
  Motor0_BLDC_SCALAR_MSM_STATE_IDENTIFICATION_Entry_Func();
}


/****************************end: BLDC_SCALAR_MSM_IDLE****************/
#endif
