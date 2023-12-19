/**
 * @file bldc_scalar_user_config.h
 * @brief Control algorithm parameters which user need to configure.
 * By default, these are configured for Infineon XMC13 low voltage motor control kit KIT_XMC1X_AK_MOTOR_001 with Maxon motor part no 267121.
 * @date 2016-09-08
 *
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
 * @addtogroup Configuration
 * @brief Control algorithm and MCU peripheral configurations <br>
 *
 * @{
 */

/*
 *  Configurations are grouped under \n
 *  BASIC
 * -# Motor and power board selection
 * -# Control scheme configurations - Select the control scheme and configure required parameters of the scheme
 * -# ADC measurement selection  - Select VADC measurements (current, voltage, potentiometer)
 * -# Protection - Current protection, Voltage protection, Stall protection and Ctrap configurations
 * -# Hall sensor configurations - Hall pattern and phase excitation pattern
 * -# Power board configurations - Power board configurations like inverter configurations, current amplifier gain, voltage divider ratio, dead time
 * ADVANCED
 * -# ADC measurement configurations - Current trigger, Filter configurations
 * -# Control algorithm feature selection and configurations - Advanced configurations to enable or disable features like seamless bi-directional control, voltage
 * compensation, hall pattern learning
 */

/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#ifndef BLDC_SCALAR_USER_CONFIG_H_
#define BLDC_SCALAR_USER_CONFIG_H_

#include "xmc_common.h"
#include "bldc_scalar_common.h"

/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/

/***********************************************************************************************************************
 * CONFIGURATIONS
 **********************************************************************************************************************/
/**
 * @addtogroup SW_Configurations
 * @brief Configurations of control algorithm
 *
 *  Configurations are grouped under \n
 *  BASIC
 * -# Motor and power board selection
 * -# Control scheme configurations - Select the control scheme and configure required parameters of the scheme
 * -# ADC measurement selection  - Select VADC measurements (current, voltage, potentiometer)
 * -# Protection - Current protection, Voltage protection, Stall protection and Ctrap configurations
 * -# Hall sensor configurations - Hall pattern and phase excitation pattern
 * -# Power board configurations - Power board configurations like inverter configurations, current amplifier gain, voltage divider ratio, dead time
 * ADVANCED
 * -# ADC measurement configurations - Current trigger, Filter configurations
 * -# Control algorithm feature selection and configurations - Advanced configurations to enable or disable features like seamless bi-directional control, voltage
 * compensation, hall pattern learning
 * @{
 */
/*********************************************************************************************************************
 *                                      BASIC CONFIGURATIONS
 ********************************************************************************************************************/
/*********************************************************************************************************************
 * Motor and power board selection
 ********************************************************************************************************************/
/* Board and motor selection */
/**
 * Motor control kit
 * Options - KIT_XMC1X_AK_MOTOR_001, KIT_XMC750WATT_MC_AK_V1, KIT_CUSTOM
 */
#define   MOTOR0_BLDC_SCALAR_BOARD                         (KIT_XMC1X_AK_MOTOR_001)
/**
 * BLDC motor
 * Options - MOTOR_EC_MAXON_267121, MOTOR_CUSTOM
 */
#define   MOTOR0_BLDC_SCALAR_MOTOR                         (MOTOR_EC_MAXON_267121)

/* Motor Parameters */
#if (MOTOR0_BLDC_SCALAR_MOTOR == MOTOR_EC_MAXON_267121)
#define   MOTOR0_BLDC_SCALAR_MOTOR_NO_LOAD_SPEED            (4530U)    /*!< No load speed of the motor in RPM */
#define   MOTOR0_BLDC_SCALAR_MOTOR_POLE_PAIRS               (4U)       /*!< Pole pairs */

#elif (MOTOR0_BLDC_SCALAR_MOTOR == MOTOR_CUSTOM)
#define   MOTOR0_BLDC_SCALAR_MOTOR_NO_LOAD_SPEED          (6200U)      /*!< No load speed of the motor in RPM */
#define   MOTOR0_BLDC_SCALAR_MOTOR_POLE_PAIRS             (4U)         /*!< Pole pairs */
#endif

/*********************************************************************************************************************
 * Control scheme configurations
 ********************************************************************************************************************/
/*********************  Control & PWM Modulation Scheme  *******************************/
/**
 * Control scheme selection:
 * Options - BLDC_SCALAR_VOLTAGE_CTRL, BLDC_SCALAR_SPEED_CTRL, BLDC_SCALAR_CURRENT_CTRL, BLDC_SCALAR_SPEEDCURRENT_CTRL
 */
#define   MOTOR0_BLDC_SCALAR_CTRL_SCHEME            (BLDC_SCALAR_SPEED_CTRL)
/**
 * PWM modulation scheme selection:
 * Options - BLDC_SCALAR_PWM_HIGHSIDE, BLDC_SCALAR_PWM_LOWSIDE, BLDC_SCALAR_PWM_HIGHSIDE_SYNCHRECTI
 */
#define   MOTOR0_BLDC_SCALAR_MODULATION             (BLDC_SCALAR_PWM_HIGHSIDE)

/************************* PWM Frequency and scheduler settings *****************************************/
/* CPU clock (mclk) is configured to 32MHz and CCU peripheral clock(pclk) to 64 MHz (double clock) */
/** PWM switching frequency. Range: 1000 to 100000 Hz*/
#define   MOTOR0_BLDC_SCALAR_PWM_FREQ                     (20000.0F)
/** Scheduler interrupt period. Range: 1000 to 10000 us*/
#define   MOTOR0_BLDC_SCALAR_SYSTICK_PERIOD               (1000.0F)
/**
 * Rate of execution of control loop. This rate is based on
 * PWM frequency which tells how many times once the control loop should execute
 */
#define MOTOR0_BLDC_SCALAR_CONTROL_LOOP_EXECUTION_RATE       (1U)

/*********************  Control Parameters based upon control scheme  ******************/
#define     MOTOR0_BLDC_SCALAR_ENABLE_RAMP                    (1U)      /*!< Enable (1)/disable(0) linear ramp */

/* Speed control */
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
#define   MOTOR0_BLDC_SCALAR_SPEED_SET            (2000.0F) /*!< speed in rpm, sign represents the motor direction +ve or -ve.
                                                      Motor runs with this set value. This configuration gets overwritten if VADC potentiometer measurement is enabled. */

#define   MOTOR0_BLDC_SCALAR_RAMP_INITIAL_VALUE   (200.0F)     /*!< speed in rpm, sign represents the motor direction +ve or -ve.
                                                                    Motor starts with this value when ramp is enabled and ramps up to set value. */
#define   MOTOR0_BLDC_SCALAR_RAMP_UP_RATE         (500.0F)     /*!< Unit is rpm/s. Range: 1 to MOTOR0_BLDC_SCALAR_MOTOR_NO_LOAD_SPEED*/
#define   MOTOR0_BLDC_SCALAR_RAMP_DOWN_RATE       (500.0F)     /*!< Unit is rpm/s. Range: 1 to MOTOR0_BLDC_SCALAR_MOTOR_NO_LOAD_SPEED*/

#define   MOTOR0_BLDC_SCALAR_SPEED_KP             (20972U)   /*!< Proportional gain of speed PI in Q15. Range: 0 to 32767*/
#define   MOTOR0_BLDC_SCALAR_SPEED_KI             (2U)       /*!< Integral gain of speed PI in Q15. Range: 0 to 32767 */
#define   MOTOR0_BLDC_SCALAR_SPEED_PI_SCALE       (15U)       /*!< Scale of Kp and Ki parameters.*/
#define   MOTOR0_BLDC_SCALAR_SPEED_PI_LIMIT       (100.0F)   /*!< in % with respect to base current in speed inner current control mode
                                                                  or base voltage in speed control mode   */
#define   MOTOR0_BLDC_SCALAR_SPEED_CTRL_EXECUTION_RATE   (1U)   /*!< Speed control loop is executed in PWM interrupt based on execution rate. Range: 1 to 255
                                                                 if execution rate is 2, speed control loop is executed once in two PWM interrupts.*/
#endif

/* Current Control */
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
/**
 * Proportional gain of current PI. Range: 0 to 32767
 * Kp = (2* 3.14 * Ls * (MOTOR0_BLDC_SCALAR_BASE_CURRENT / MOTOR0_BLDC_SCALAR_BASE_VOLTAGE) * Bandwidth * 2^scale)
 * For MOTOR_EC_MAXON_267121, Kp = 1.517 for Ls = 3865 uH and bandwidth = 500 Hz
 */
#define   MOTOR0_BLDC_SCALAR_CURRENT_KP            (24853U)
/**
 * Integral gain of current PI. Range: 0 to 32767
 * Ki = (2* 3.14 * Rs * (MOTOR0_BLDC_SCALAR_BASE_CURRENT / MOTOR0_BLDC_SCALAR_BASE_VOLTAGE) * Bandwidth * MOTOR0_BLDC_SCALAR_CCU8_PERIOD_uS * 2^scale)
 * For MOTOR_EC_MAXON_267121, Ki = 0.134 for Rs = 6.85 ohms and bandwidth = 500 Hz
 */
#define   MOTOR0_BLDC_SCALAR_CURRENT_KI            (2202U)
#define   MOTOR0_BLDC_SCALAR_CURRENT_PI_SCALE      (14U)       /*!< Scale of Kp and Ki parameters.*/
#define   MOTOR0_BLDC_SCALAR_CURRENT_PI_LIMIT      (100.0F)    /*!< in % with respect to base voltage */
/**
 * Feedback for current PI
 * Options - BLDC_SCALAR_IDC_DIRECT_CURRENT, BLDC_SCALAR_IDC_AVG_CURRENT
 * Average current can be directly measured from VADC or can be calculated from DC link current MOTOR0_BLDC_SCALAR_VADC_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK
 */
#define   MOTOR0_BLDC_SCALAR_CURRENT_FEEDBACK      (BLDC_SCALAR_IDC_DIRECT_CURRENT)

#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL))
#define   MOTOR0_BLDC_SCALAR_CURRENT_SET           (0.3F)     /*!< Current is in Amps, sign represents the direction +ve or -ve
                                                                   Motor runs with this set value. This configuration gets overwritten if VADC potentiometer measurement is enabled. */

#define   MOTOR0_BLDC_SCALAR_RAMP_INITIAL_VALUE    (0.2F)   /*!< Current is in Amps, sign represents the direction +ve or -ve
                                                                 Motor starts with this value when ramp is enabled and ramps up to set value. */
#define   MOTOR0_BLDC_SCALAR_RAMP_UP_RATE          (0.01F)    /*!< Unit is A/s. Range: 0.001 to 1000 * MOTOR0_BLDC_SCALAR_MAX_CURRENT*/
#define   MOTOR0_BLDC_SCALAR_RAMP_DOWN_RATE        (0.01F)    /*!< Unit is A/s. Range: 0.001 to 1000 * MOTOR0_BLDC_SCALAR_MAX_CURRENT*/
#endif /* end of #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL)) */

#endif /* #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL)) */

/* Voltage control */
#if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_VOLTAGE_CTRL)
#define   MOTOR0_BLDC_SCALAR_VOLTAGE_SET           (12.0F)  /*!< Voltage is in volts, sign represents the direction +ve or -ve
                                                                 Motor runs with this set value. This configuration gets overwritten if VADC potentiometer measurement is enabled.*/
#define   MOTOR0_BLDC_SCALAR_RAMP_INITIAL_VALUE    (2.0F)  /*!< Voltage is in volts, sign represents the direction +ve or -ve
                                                      Motor starts with this value when ramp is enabled and ramps up to set value. */
#define   MOTOR0_BLDC_SCALAR_RAMP_UP_RATE          (1.0F)  /*!< Unit is V/s. Range: 0.1 to MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT */
#define   MOTOR0_BLDC_SCALAR_RAMP_DOWN_RATE        (1.0F)  /*!< Unit is V/s. Range: 0.1 to MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT*/
#endif

/* output voltage limits */
#define   MOTOR0_BLDC_SCALAR_MIN_AMPLITUDE         (0.0F)   /*!< Minimum amplitude in % */
#define   MOTOR0_BLDC_SCALAR_MAX_AMPLITUDE         (98.0F)  /*!< Maximum amplitude in % */

/*********************************************************************************************************************
 * ADC measurement selection
 ********************************************************************************************************************/

/************************* Configuration Macros for enabling VADC measurements. *****************************************/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT            (1U) /*!< Enable (1)/disable (0) DC link current measurement.*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT         (0U) /*!< Enable (1)/disable (0) IDC average current measurement.*/
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U) && (MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 0U))
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK   (0U)  /*!< Enable (1)/disable (0) average current calculation from DC link current. Average current
                                                                             is calculated in SW from the measured DC link current */
#endif
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_VDC_LINK_MEASUREMENT            (1U) /*!< Enable (1)/disable (0) VDC link current measurement.*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT       (0U) /*!< Enable (1)/disable (0) pot measurement.*/

#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF1_MEASUREMENT           (0U) /*!< Enable (1)/disable (0)user defined measurement
                                                                          Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF2_MEASUREMENT           (0U) /*!< Enable (1)/disable (0)user defined measurement
                                                                          Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF3_MEASUREMENT           (0U) /*!< Enable (1)/disable (0)user defined measurement
                                                                          Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/
#define MOTOR0_BLDC_SCALAR_VADC_ENABLE_USER_DEF4_MEASUREMENT           (0U) /*!< Enable (1)/disable (0)user defined measurement
                                                                          Configure ADC channel number and group number in the bldc_scalar_mcuhw_config.h file*/


/*********************************************************************************************************************
 * Protection Schemes
 ********************************************************************************************************************/
/* Current protection */
#define   MOTOR0_BLDC_SCALAR_ENABLE_OVER_CURRENT           (1U)        /*!< Enable (1) /disable (0) over current and short-circuit protection */
#if (MOTOR0_BLDC_SCALAR_ENABLE_OVER_CURRENT == 1U)
/**
 * Current source for over current protection:
 * Options - BLDC_SCALAR_IDC_DIRECT_CURRENT, BLDC_SCALAR_IDC_AVG_CURRENT
 */
#define   MOTOR0_BLDC_SCALAR_OVER_CURRENT_SOURCE           (BLDC_SCALAR_IDC_DIRECT_CURRENT)
#define   MOTOR0_BLDC_SCALAR_OVER_CURRENT                  (2.7F)       /*!< Current is in Amps Max Range: MOTOR0_BLDC_SCALAR_MAX_CURRENT*/
#define   MOTOR0_BLDC_SCALAR_CURRENT_PROTECTION_TIME       (50.0F)       /*!< Detection time for over current protection in mSec*/

#define   MOTOR0_BLDC_SCALAR_SHORTCIRCUIT_CURRENT          (2.9F)    /*!< Current is in Amps. Motor is stopped immediately when current is above this limit
                                                                 Max range: MOTOR0_BLDC_SCALAR_MAX_CURRENT*/
#endif

#define   MOTOR0_BLDC_SCALAR_ENABLE_UNDER_OVER_VOLTAGE     (0U)    /*!< Enable (1) /disable (0) over/under voltage protection */
#if (MOTOR0_BLDC_SCALAR_ENABLE_UNDER_OVER_VOLTAGE == 1U)
#define   MOTOR0_BLDC_SCALAR_MIN_DC_LINK_VOLTAGE           (80.0F)   /*!< in % with respect to base voltage MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT*/
#define   MOTOR0_BLDC_SCALAR_MAX_DC_LINK_VOLTAGE           (120.0F)  /* in % with respect to base voltage */
#define   MOTOR0_BLDC_SCALAR_VOLTAGE_PROTECTION_TIME       (50.0F)     /*!< Detection time for voltage protection in mSec*/
#endif

/* Ctrap protection */
#define   MOTOR0_BLDC_SCALAR_ENABLE_CTRAP                   (1U)     /*!< Enable (1)/disable (0) ctrap protection */


/*********************************************************************************************************************
 * Power board configurations
 ********************************************************************************************************************/
/************************ Power board parameters **************************************************************/
#if (MOTOR0_BLDC_SCALAR_BOARD == KIT_XMC1X_AK_MOTOR_001)
/* Power Inverter parameters */
#define   MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT         (24.0F)     /*!< DC link voltage */
#define   MOTOR0_BLDC_SCALAR_RISING_DEAD_TIME             (0.75F)     /*!< Dead time for rising edge in uSec*/
#define   MOTOR0_BLDC_SCALAR_FALLING_DEAD_TIME            (0.75F)     /*!< Dead time for falling edge in uSec*/
#define   MOTOR0_BLDC_SCALAR_SWITCH_DELAY                 (0.75F)     /*!< Switch delay in uSec*/

#define   MOTOR0_BLDC_SCALAR_HS_SWITCH_ACTIVE_LEVEL        (BLDC_SCALAR_ACTIVE_HIGH)       /*!< Active level of the high side switch. Options: BLDC_SCALAR_ACTIVE_HIGH, BLDC_SCALAR_ACTIVE_LOW */
#define   MOTOR0_BLDC_SCALAR_LS_SWITCH_ACTIVE_LEVEL        (BLDC_SCALAR_ACTIVE_HIGH)       /*!< Active level of the low side switch. Options: BLDC_SCALAR_ACTIVE_HIGH, BLDC_SCALAR_ACTIVE_LOW */
#define   MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF          (BLDC_SCALAR_INV_ACTIVE_HIGH)   /*!< Active level of inverter enable. Options: BLDC_SCALAR_INV_DISABLED,  BLDC_SCALAR_INV_ACTIVE_HIGH, BLDC_SCALAR_INV_ACTIVE_LOW*/

/* ADC Measurement  parameters */
#define   MOTOR0_BLDC_SCALAR_VADC_REF_VOLTAGE              (5.0F)         /*!< Reference voltage of VADC conversion */
#define   MOTOR0_BLDC_SCALAR_CURRENT_AMPLIFIER_OFFSET      (2.5F)       /*!< Amplifier offset voltage  */
#define   MOTOR0_BLDC_SCALAR_CURRENT_RSHUNT                (50.0F)       /*!< Current amplifier shunt resistor value in mOhms */
#define   MOTOR0_BLDC_SCALAR_CURRENT_AMPLIFIER_GAIN        (16.4F)        /*!< Current amplifier gain */
#define   MOTOR0_BLDC_SCALAR_VOLTAGE_DIVIDER_RATIO         (9.79F)      /*!< Voltage divider ratio in % for DC link voltage measurement */
#define   MOTOR0_BLDC_SCALAR_BEMF_DIVIDER_RATIO            (9.79F)      /*!< Voltage divider ratio in % for phase voltage measurement */
/* end of #if (MOTOR0_BLDC_SCALAR_BOARD == KIT_XMC1X_AK_MOTOR_001) */

#elif  (MOTOR0_BLDC_SCALAR_BOARD == KIT_XMC750WATT_MC_AK_V1)
/* Power Inverter parameters */
#define   MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT         (320.0F)     /*!< DC link voltage */
#define   MOTOR0_BLDC_SCALAR_RISING_DEAD_TIME             (0.75F)     /*!< Dead time for rising edge in uSec*/
#define   MOTOR0_BLDC_SCALAR_FALLING_DEAD_TIME            (0.75F)     /*!< Dead time for falling edge in uSec*/
#define   MOTOR0_BLDC_SCALAR_SWITCH_DELAY                 (0.75F)     /*!< Switch delay in uSec*/

#define   MOTOR0_BLDC_SCALAR_HS_SWITCH_ACTIVE_LEVEL        (BLDC_SCALAR_ACTIVE_LOW)       /*!< Active level of the high side switch. Options: BLDC_SCALAR_ACTIVE_HIGH, BLDC_SCALAR_ACTIVE_LOW */
#define   MOTOR0_BLDC_SCALAR_LS_SWITCH_ACTIVE_LEVEL        (BLDC_SCALAR_ACTIVE_LOW)       /*!< Active level of the low side switch. Options: BLDC_SCALAR_ACTIVE_HIGH, BLDC_SCALAR_ACTIVE_LOW */
#define   MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF          (BLDC_SCALAR_INV_ACTIVE_LOW)   /*!< Active level of inverter enable. Options: BLDC_SCALAR_INV_DISABLED,  BLDC_SCALAR_INV_ACTIVE_HIGH, BLDC_SCALAR_INV_ACTIVE_LOW*/

/* ADC Measurement  parameters */
#define   MOTOR0_BLDC_SCALAR_VADC_REF_VOLTAGE              (5U)         /*!< Reference voltage of VADC conversion */
#define   MOTOR0_BLDC_SCALAR_CURRENT_AMPLIFIER_OFFSET      (2.5F)       /*!< Amplifier offset voltage */
#define   MOTOR0_BLDC_SCALAR_CURRENT_RSHUNT                (50.0F)      /*!< Current amplifier shunt resistor value in mOhms */
#define   MOTOR0_BLDC_SCALAR_CURRENT_AMPLIFIER_GAIN        (12.0F)      /*!< Current amplifier gain */
#define   MOTOR0_BLDC_SCALAR_VOLTAGE_DIVIDER_RATIO         (1.0F)      /*!< Voltage divider ratio in % for DC link voltage measurement */
#define   MOTOR0_BLDC_SCALAR_BEMF_DIVIDER_RATIO            (1.1976F)    /*!< Voltage divider ratio in % for phase voltage measurement */
/*end of if  (MOTOR0_BLDC_SCALAR_BOARD == KIT_XMC750WATT_MC_AK_V1) */

#elif  (MOTOR0_BLDC_SCALAR_BOARD == KIT_CUSTOM)
/* Power Inverter parameters */
#define   MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT         (24.0F)     /*!< DC link voltage */
#define   MOTOR0_BLDC_SCALAR_RISING_DEAD_TIME             (0.75F)     /*!< Dead time for rising edge in uSec*/
#define   MOTOR0_BLDC_SCALAR_FALLING_DEAD_TIME            (0.75F)     /*!< Dead time for falling edge in uSec*/
#define   MOTOR0_BLDC_SCALAR_SWITCH_DELAY                 (0.75F)     /*!< Switch delay in uSec*/

#define   MOTOR0_BLDC_SCALAR_HS_SWITCH_ACTIVE_LEVEL        (BLDC_SCALAR_ACTIVE_HIGH)       /*!< Active level of the high side switch. Options: BLDC_SCALAR_ACTIVE_HIGH, BLDC_SCALAR_ACTIVE_LOW */
#define   MOTOR0_BLDC_SCALAR_LS_SWITCH_ACTIVE_LEVEL        (BLDC_SCALAR_ACTIVE_HIGH)       /*!< Active level of the low side switch. Options: BLDC_SCALAR_ACTIVE_HIGH, BLDC_SCALAR_ACTIVE_LOW */
#define   MOTOR0_BLDC_SCALAR_INVERTER_ENABLE_CONF          (BLDC_SCALAR_INV_ACTIVE_HIGH)   /*!< Active level of inverter enable. Options: BLDC_SCALAR_INV_DISABLED,  BLDC_SCALAR_INV_ACTIVE_HIGH, BLDC_SCALAR_INV_ACTIVE_LOW*/

/* ADC Measurement  parameters */
#define   MOTOR0_BLDC_SCALAR_VADC_REF_VOLTAGE              (5.0F)         /*!< Reference voltage of VADC conversion */
#define   MOTOR0_BLDC_SCALAR_CURRENT_AMPLIFIER_OFFSET      (2.5F)       /*!< Amplifier offset voltage  */
#define   MOTOR0_BLDC_SCALAR_CURRENT_RSHUNT                (50.0F)       /*!< Current amplifier shunt resistor value in mOhms */
#define   MOTOR0_BLDC_SCALAR_CURRENT_AMPLIFIER_GAIN        (16.4F)        /*!< Current amplifier gain */
#define   MOTOR0_BLDC_SCALAR_VOLTAGE_DIVIDER_RATIO         (9.79F)      /*!< Voltage divider ratio in % for DC link voltage measurement */
#define   MOTOR0_BLDC_SCALAR_BEMF_DIVIDER_RATIO            (9.79F)      /*!< Voltage divider ratio in % for phase voltage measurement */
/* end of if  (MOTOR0_BLDC_SCALAR_BOARD == KIT_CUSTOM) */
#endif

#define   MOTOR0_BLDC_SCALAR_AMPLITUDE_HIGH_THRESHOLD         (100.0F)   /*!< in %. Max amplitude threshold above which maximum duty cycle is applied to avoid bootstrap capacitor discharge
                                                                             Range: 95 to 100% */

/*********************************************************************************************************************
 *                                      SENSORLESS CONFIGURATIONS
 ********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)

/*********************************************************************************************************************
 *                                   INITIAL POSITION CONFIGURATIONS
 ********************************************************************************************************************/
/**
 * Sensorless startup method selection
 * Options - BLDC_SCALAR_SL_INDUCTIVE_SENSING, BLDC_SCALAR_SL_ALIGN
 */
#define MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE       (BLDC_SCALAR_SL_INDUCTIVE_SENSING)

#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING)
#define MOTOR0_BLDC_SCALAR_SL_IND_SENSE_PATTERN_ONTIME        (0.2F)     /*!< CCU8 pulse width in mSec. Phase patterns are energized in
                                                                              sequence for this time. Range: 0.0005 to 32 mSec */
#define MOTOR0_BLDC_SCALAR_SL_IND_SENSE_CURRENT_DECAY_TIME    (0.2F)     /*!< Time in mSec for current to decay after pulse is applied. Range: 0.0005 to 32 mSec */

#else  /* Alignment */
#define MOTOR0_BLDC_SCALAR_SL_ALIGN_VOLTAGE                  (15.0F)        /*!< in % with respect to  MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT Range: 1 to 50 */
#define MOTOR0_BLDC_SCALAR_SL_ALIGN_RAMP_RATE                (50.0F)         /*!< in volt/sec. Voltage is ramped to MOTOR0_BLDC_SCALAR_SL_ALIGN_VOLTAGE and
                                                                                  then maintained for MOTOR0_BLDC_SCALAR_SL_ALIGN_TIME. */
#define MOTOR0_BLDC_SCALAR_SL_ALIGN_TIME                     (100.0F)       /*!< Alignment time in mSec. Min range: MOTOR0_BLDC_SCALAR_SYSTICK_PERIOD */

#endif /* end of #if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE == BLDC_SCALAR_SL_INDUCTIVE_SENSING) */

/*********************************************************************************************************************
 *                                   STARTUP CONFIGURATIONS
 ********************************************************************************************************************/
#define MOTOR0_BLDC_SCALAR_SL_STARTUP_MIN_AMPLITUDE          (10.0F)        /*!< in %. Minimum Duty cycle to be applied for sensorless startup.
                                                                                Duty cycle is then controlled by current control loop.
                                                                                Range: 0.1 to 10 */
#define MOTOR0_BLDC_SCALAR_SL_FIRST_KICK_TIME                (4.0F)        /*!< First kick time in mSec. Phase pattern corresponding to identified position
                                                                                is energized for this time.
                                                                                Min Range: 1/MOTOR0_BLDC_SCALAR_PWM_FREQ */

#define MOTOR0_BLDC_SCALAR_SL_TR_PHASE_ENERGIZATION_TIME     (0.8F)        /*!< in mSec. Phases are energized in sequence for this time
                                                                                during transition state from first kick to closed loop.
                                                                                Min Range: 1/MOTOR0_BLDC_SCALAR_PWM_FREQ */
#define MOTOR0_BLDC_SCALAR_SL_TR_CURRENT_DECAY_TIME          (0.8F)        /*!< in mSec. Wait time for current to decay.
                                                                                Min Range: 1/MOTOR0_BLDC_SCALAR_PWM_FREQ */

#define MOTOR0_BLDC_SCALAR_SL_MIN_BEMF_VOLTAGE               (2.0F)       /*!< in % with respect to MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT
                                                                               if measured BEMF voltage is less than this voltage during startup, position identification is again performed. */
#define MOTOR0_BLDC_SCALAR_SL_TRANSITION_BEMF_VOLTAGE        (10.0F)      /*!< in % with respect to MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT
                                                                               Motor switches to closed loop when measured BEMF voltage is greater than this voltage. */
#define MOTOR0_BLDC_SCALAR_SL_STARTUP_COMMUTATION_COUNT      (12U)        /*!< Startup commutation counts before control switches to closed loop. Range: 6 to 255 */

/*********************************************************************************************************************
 *                                   STARTUP CURRENT CONTROL
 ********************************************************************************************************************/
#define   MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_SET           (0.4F)     /*!< Current is in Amps.
                                                                   Motor runs in current control during transition phase. */
/**
 * Proportional gain of startup current PI. Range: 0 to 32767
 */
#define   MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_KP            (24853U)
/**
 * Integral gain of startup current PI. Range: 0 to 32767
 */
#define   MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_KI            (2202U)
#define   MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_PI_SCALE      (14U)       /*!< Scale of Kp and Ki parameters.*/
#define   MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_PI_LIMIT      (30.0F)    /*!< in % with respect to base voltage */
/**
 * Feedback for current PI
 * Options - BLDC_SCALAR_IDC_DIRECT_CURRENT, BLDC_SCALAR_IDC_AVG_CURRENT
 * Average current can be directly measured from VADC or can be calculated from DC link current MOTOR0_BLDC_SCALAR_VADC_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK
 */
#define   MOTOR0_BLDC_SCALAR_SL_STARTUP_CURRENT_FEEDBACK      (BLDC_SCALAR_IDC_DIRECT_CURRENT)

/*********************************************************************************************************************
 *                                   BEMF MEASUREMENT
 ********************************************************************************************************************/
/**
 * Trigger method for BEMF measurement selection
 * Options - BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER, BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER
 */
#define MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER                   (BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER)


#define MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER_OFFSET                (0.5F)     /*!< BEMF measurement trigger time in uSec. BEMF is measured at
                                                                              (End of PWM ON time - configured time).
                                                                              This time should be greater than sampling time of VADC (0.1 uSec) */
#if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_CONTINUOUS_TRIGGER)
#define MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER_TIME               (3.0F)     /*!< BEMF measurement trigger time in uSec. BEMF is measured continuously at this configured time
                                                                             during PWM ON time. This time should be greater than VADC conversion time (1.0 uSec) multiplied by
                                                                             number of channels in the same group. */
#define MOTOR0_BLDC_SCALAR_SL_RINGING_BLANKING_TIME           (15.0F)     /*!< in uSec. BEMF is ignored for this time every PWM ON time. */
#define MOTOR0_BLDC_SCALAR_SL_DEMAGNETIZATION_BLANKING_TIME   (100.0F)    /*!< in uSec. BEMF is ignored for this time to avoid de-magnetization pulse. */
#endif /* end of #if (MOTOR0_BLDC_SCALAR_SL_BEMF_TRIGGER == BLDC_SCALAR_SL_BEMF_SINGLE_TRIGGER) */
/*********************************************************************************************************************
 *                                   LOW SPEED
 ********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL)
#define MOTOR0_BLDC_SCALAR_SL_MIN_THRESHOLD                   (2.0F)    /*!< in %. Below this set value, motor control goes to IDLE state in current control. */
#else
#define MOTOR0_BLDC_SCALAR_SL_MIN_THRESHOLD                   (10.0F)   /*!< in %. Below this set value, motor control goes to IDLE state. */
#endif
#define MOTOR0_BLDC_SCALAR_SL_BRAKING_VOLTAGE                 (50.0F)   /*!< Braking DC voltage in terms of %. This duty cycle is applied to low
                                                                              side switches during braking */
#endif /* end of #if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS) */

/*********************************************************************************************************************
 *                                      ADVANCED CONFIGURATIONS
 ********************************************************************************************************************/

/*********************************************************************************************************************
 * ADC configurations
 ********************************************************************************************************************/
#define MOTOR0_BLDC_SCALAR_ENABLE_VADC_STARTUP_CALIBRATION            (1U)          /*!< Enable (1)/disable (0) VADC startup calibration. */

/*********************************************************************************************************************
 * Current measurement configurations
 ********************************************************************************************************************/
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U) || (MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 1U))
#define MOTOR0_BLDC_SCALAR_ENABLE_AMPLIFIER_OFFSET_CALIBRATION        (1U)          /*!< Enable (1)/disable (0) current amplifier offset voltage calibration */
#endif

#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U)
#define MOTOR0_BLDC_SCALAR_ENABLE_IDC_LINK_CURRENT_FILTER             (0U)       /*!< Enable(1)/ disable (0) SW filter for direct DC link current */
#define MOTOR0_BLDC_SCALAR_IDC_LINK_CURRENT_FILTER_TIMECONST          (0.0005F)    /*!< PT1 filter time constant in sec*/
#endif

#if ( (MOTOR0_BLDC_SCALAR_VADC_ENABLE_AVERAGE_CURRENT_USING_IDC_LINK == 1U) || (MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_AVERAGE_MEASUREMENT == 1U))
#define MOTOR0_BLDC_SCALAR_ENABLE_IDC_AVERAGE_CURRENT_FILTER          (0U)        /*!< Enable (1) / disable (0) SW filter for average DC link current */
#define MOTOR0_BLDC_SCALAR_IDC_AVERAGE_CURRENT_FILTER_TIMECONST       (0.0005F)    /*!< PT1 filter time constant in sec*/
#endif

#define MOTOR0_BLDC_SCALAR_CURRENT_TRIGGER_OFFSET                  (MOTOR0_BLDC_SCALAR_SWITCH_DELAY)  /*!< By default, current trigger offset is switch delay from PWM midpoint.
                                                                                                   Enter time in uSec if offset other than switch delay is required */
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_IDC_LINK_MEASUREMENT == 1U))
#define MOTOR0_BLDC_SCALAR_ENABLE_DEMAGNET_BLANKING                      (0U)      /*!< Enable (1)/ disable (0) demagnetization blanking for DC link current measurement */
#if (MOTOR0_BLDC_SCALAR_ENABLE_DEMAGNET_BLANKING == 1U)
#define MOTOR0_BLDC_SCALAR_DEMAGNET_BLANKING_TIME                        (100.0F)  /*!< Blanking time in uSec. Direct DC link current measurement is skipped
                                                                                      for this time to avoid wrong values during de-magnetization time */
#endif
#endif

/*********************************************************************************************************************
 * Potentiometer configurations
 ********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U)
#define     MOTOR0_BLDC_SCALAR_MIN_ANALOG_INPUT_LOW_LIMIT     (5.0F)    /*!< in % of the analog input value. Analog value is considered as 0 below this limit.
                                                                      Range: 1 to 5% */
#define MOTOR0_BLDC_SCALAR_POTENTIOMETER_PT1_FILTER_ENABLE      (0U)     /*!< Enable (1)/disable (0) SW filter on potentiometer values */
#define MOTOR0_BLDC_SCALAR_POTENTIOMETER_FILTER_TIMECONST      (0.001F)    /* PT1 filter time constant in sec*/
#endif

/*********************************************************************************************************************
 * Control algorithm feature Selection
 ********************************************************************************************************************/
#define     MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4         (0U)      /*!< Enable (1)/disable (0) multi-channel update synchronization with CCU4 */
#if (MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4 == 1U)
#define     MOTOR0_BLDC_SCALAR_MCM_SYNCTRANSFER_TIME         (2.0F)      /*!< Multi-channel pattern update is synchronized with this time (uSec). Range: 1 to 5 uSec */
#endif /* end of #if (MOTOR0_BLDC_SCALAR_ENABLE_FAST_SYNCH_CCU4 == 1U) */

#if (MOTOR0_BLDC_SCALAR_FEEDBACK == BLDC_SCALAR_SENSORLESS)
#if ((MOTOR0_BLDC_SCALAR_VADC_ENABLE_POTENTIOMETER_MEASUREMENT == 1U))
#define   MOTOR0_BLDC_SCALAR_MOTOR_DIRECTION        (BLDC_SCALAR_NEGATIVE_DIR)      /*!< Select the direction of rotation. In other cases, direction is determined from the
                                                                                ramp initial value and set value */
#endif
#endif

#define     MOTOR0_BLDC_SCALAR_ENABLE_VOLT_COMPENSATION       (0U)      /*!< Enable/disable DC link voltage ripple compensation */
#if (MOTOR0_BLDC_SCALAR_ENABLE_VOLT_COMPENSATION == 1U)
#define     MOTOR0_BLDC_SCALAR_ENABLE_VOLT_COMPENSATION_FILTER (0U)     /*!< Enable (1)/disable (0) PT1 filter on DC link voltage for compensation */
#define     MOTOR0_BLDC_SCALAR_VOLTCOMP_FILTER_TIMECONST        (0.06F)    /* PT1 filter time constant in sec */
#endif

#define     MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP               (1U)      /*!< Bootstrap enabled/disable */
#if (MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U)
#define   MOTOR0_BLDC_SCALAR_BOOTSTRAP_TIME                   (20.0F)      /*!< bootstrap capacitor charging time for each leg in mSec */
#endif  /* if(MOTOR0_BLDC_SCALAR_ENABLE_BOOTSTRAP == 1U) */

#define     MOTOR0_BLDC_SCALAR_ENABLE_RAMP_DOWN_VOLTAGE_CLAMPING  (0U)      /*!< Enable (1)/disable (0) DC bus voltage clamping during fast braking */
#if (MOTOR0_BLDC_SCALAR_ENABLE_RAMP_DOWN_VOLTAGE_CLAMPING == 1U)
#define      MOTOR0_BLDC_SCALAR_RAMP_DOWN_VOLTAGE_LIMIT            (105.0F)   /* in % with respect to base voltage MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT */
#endif

#define MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE (1U)

/**
 * @}
 */

/**
 * @}
 */
/**
 * @}
 */
#endif /* BLDC_SCALAR_USER_CONFIG_H_ */
