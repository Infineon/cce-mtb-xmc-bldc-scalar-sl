
/***********************************************************************************************************************
 * HEADER FILES
 **********************************************************************************************************************/
#include "bldc_scalar_user_interface.h"
#include  "../ControlModule/bldc_scalar_inductive_sensing.h"
#include "uCProbe.h"
#include "xmc1_flash.h"
/***********************************************************************************************************************
 * MACROS
 **********************************************************************************************************************/
#define BLDC_SCALAR_MCM_PATTERN_SIZE (15U)                                            /*Multi-channel pattern array size*/

/***********************************************************************************************************************
 * LOCAL DATA
 **********************************************************************************************************************/
#if (MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1U)
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME != BLDC_SCALAR_VOLTAGE_CTRL))
BLDC_SCALAR_VOLTAGE_CONTROL_t Motor0_BLDC_SCALAR_VoltageControl;
#endif
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME != BLDC_SCALAR_SPEED_CTRL) && (MOTOR0_BLDC_SCALAR_CTRL_SCHEME != BLDC_SCALAR_SPEEDCURRENT_CTRL))
BLDC_SCALAR_SPEED_CONTROL_t Motor0_BLDC_SCALAR_SpeedControl;
BLDC_SCALAR_PI_CONTROLLER_t Motor0_BLDC_SCALAR_SpeedControl_PI;
#endif
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME != BLDC_SCALAR_CURRENT_CTRL))
BLDC_SCALAR_CURRENT_CONTROL_t Motor0_BLDC_SCALAR_CurrentControl;
#endif
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME != BLDC_SCALAR_CURRENT_CTRL) && (MOTOR0_BLDC_SCALAR_CTRL_SCHEME != BLDC_SCALAR_SPEEDCURRENT_CTRL))
BLDC_SCALAR_PI_CONTROLLER_t Motor0_BLDC_SCALAR_CurrentControl_PI;
#endif
#if (MOTOR0_BLDC_SCALAR_SL_STARTUP_TECHNIQUE != BLDC_SCALAR_SL_INDUCTIVE_SENSING)
BLDC_SCALAR_INDUCTIVE_SENSING_t Motor0_BLDC_SCALAR_InductiveSensing;
#endif

#endif /*End of #if (MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1U)*/
/**********************************************************************************************************************
* DATA STRUCTURES
**********************************************************************************************************************/
BLDC_SCALAR_UCPROBE_t Motor0_BLDC_SCALAR_ucprobe;
/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
/*Read all flash variables, default values and store it in local array*/
void Motor0_BLDC_SCALAR_uCProbe_Read_Variable(void);
/*Read all flash variables, from flash*/
void Motor0_BLDC_SCALAR_uCProbe_Read_Flash(void);
/*Write all flash variables to  flash*/
void Motor0_BLDC_SCALAR_uCProbe_Write_Flash(void);
/*Write all flash variables*/
void Motor0_BLDC_SCALAR_uCProbe_Write_Variable(void);
/*Store all the default value of flash variable into a local array*/
void Motor0_BLDC_SCALAR_Store_Default_value(void);
/*Write all the default value of flash variable from a local array*/
void Motor0_BLDC_SCALAR_Write_Default_value(void);
/*Update all the UCPROBE UI variables*/
void Motor0_BLDC_SCALAR_uCPorbe_Update_UI_Var(void);
/*Read Speed control PI variable from the variable*/
void Motor0_BLDC_SCALAR_uCProbe_Read_Variable_SpeedPI(void);

/*Read Current control PI variable from the variable*/
void Motor0_BLDC_SCALAR_uCProbe_Read_Variable_CurrentPI(void);

void Motor0_BLDC_SCALAR_uCProbe_Read_Variable_Sensorless(void);



void Motor0_BLDC_SCALAR_uCProbe_SpeedPI_Reset(void);
void Motor0_BLDC_SCALAR_uCProbe_CurrentPI_Reset(void);
void Motor0_BLDC_SCALAR_uCProbe_Sensorless_Reset(void);
/**********************************************************************************************************************
 * API IMPLEMENTATION
 **********************************************************************************************************************/
#if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))
/*UCproBE scheduler function to handle ucprobe comments from UI */
void Motor0_BLDC_SCALAR_uCProbe_Scheduler(void)
{
  switch(Motor0_BLDC_SCALAR_ucprobe.control_word)
  {
    case 1:  /* Start the motor */
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      Motor0_BLDC_SCALAR_MotorStart();
      break;

    case 2: /*Stop the motor*/
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      Motor0_BLDC_SCALAR_MotorStop();
      break;

    case 3: /*Clear Error state*/
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      Motor0_BLDC_SCALAR_ClearErrorState();
      break;

    case 4:  /*Clear flash and load defualt value into flash*/
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      Motor0_BLDC_SCALAR_ucprobe.user_config[0] =0;
      Motor0_BLDC_SCALAR_Write_Default_value();
      Motor0_BLDC_SCALAR_uCProbe_Write_Flash();
      break;

    case 5: /*Write PI into flash*/
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      Motor0_BLDC_SCALAR_ucprobe.user_config[0] &=0xFFFF0000;
      Motor0_BLDC_SCALAR_ucprobe.user_config[0] |=MOTOR0_BLDC_SCALAR_UCPROBE_FLASH_VALID_ID;
      #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
      Motor0_BLDC_SCALAR_ucprobe.user_config[0] |=0x20000;
      #endif
      #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
      Motor0_BLDC_SCALAR_ucprobe.user_config[0] |=0x10000;
      #endif
      Motor0_BLDC_SCALAR_uCProbe_Read_Variable();
      Motor0_BLDC_SCALAR_uCProbe_Write_Flash();
      break;

    case 6:
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      break;

    case 7:
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      break;

    case 8:
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      break;

    case 9:  /*Direction change*/
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;

      Motor0_BLDC_SCALAR_SetDirection(Motor0_BLDC_SCALAR.motor_set_direction*-1);


      break;

    default:
      Motor0_BLDC_SCALAR_ucprobe.control_word=0;
      break;
  } /*End of switch(Motor0_BLDC_SCALAR_ucprobe.control_word)*/
  Motor0_BLDC_SCALAR_uCPorbe_Update_UI_Var();
}
#endif /* End of #if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))*/
/***********************************************************************************************************************/
/* Handling flash variable, if flash contain any valid data, write to actual variable from flash*/
void Motor0_BLDC_SCALAR_Flash_Var_Init(void)
{
#if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))
	Motor0_BLDC_SCALAR_Store_Default_value();
#endif
  /*Check flash contain any data*/
  if (((*MOTOR0_BLDC_SCALAR_UCPROBE_CONFIG_ADDR) & 0xFFFF)== MOTOR0_BLDC_SCALAR_UCPROBE_FLASH_VALID_ID)
  {
    /*Read from flash*/
    Motor0_BLDC_SCALAR_uCProbe_Read_Flash();
    /*Write flash value into variables*/
    Motor0_BLDC_SCALAR_uCProbe_Write_Variable();
  }
  else
  {
    /*Read from flash*/
    Motor0_BLDC_SCALAR_uCProbe_Read_Flash();
    Motor0_BLDC_SCALAR_uCProbe_Read_Variable();
    Motor0_BLDC_SCALAR_ucprobe.user_config[0] =0;
    /*Write to flash*/
    Motor0_BLDC_SCALAR_uCProbe_Write_Flash();
  }
}
/***********************************************************************************************************************/
#if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))
/* UCPROBE variable initialization */
void Motor0_BLDC_SCALAR_uCProbe_Init(void)
{
  Motor0_BLDC_SCALAR_ucprobe.max_speed_RPM = MOTOR0_BLDC_SCALAR_MOTOR_NO_LOAD_SPEED;
  Motor0_BLDC_SCALAR_ucprobe.max_current_mA = (uint32_t)(MOTOR0_BLDC_SCALAR_MAX_CURRENT*1000);
  Motor0_BLDC_SCALAR_ucprobe.max_voltage_Volt = (uint32_t)MOTOR0_BLDC_SCALAR_NOMINAL_DC_LINK_VOLT;

  #if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_OSC_ENABLE ==1) && (MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))
  ProbeScope_Init(20000);
  #endif

}
#endif /*#if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))*/

/***********************************************************************************************************************
 * LOCAL ROUTINES
 **********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_Read_Variable(void)
{
  Motor0_BLDC_SCALAR_uCProbe_Read_Variable_SpeedPI();/* Array index from 1 to 5*/
  Motor0_BLDC_SCALAR_uCProbe_Read_Variable_CurrentPI();/* Array index from 6 to 10*/
  Motor0_BLDC_SCALAR_uCProbe_Read_Variable_Sensorless();/* Array index from 11 to 18 and 20*/
  Motor0_BLDC_SCALAR_ucprobe.user_config[19] = Motor0_BLDC_SCALAR.motor_set_direction;
}

/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_Read_Flash(void)
{
  uint32_t count=0;
  uint32_t *motor_conf_addr = MOTOR0_BLDC_SCALAR_UCPROBE_CONFIG_ADDR;

  for(count=0;count<(BLDC_SCALAR_UCPROBE_MAX_PARAMETER-1);count++)
  {
    Motor0_BLDC_SCALAR_ucprobe.user_config[count] =*(motor_conf_addr+count);
  }
}

/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_Write_Flash(void)
{
  uint32_t *motor_conf_addr = MOTOR0_BLDC_SCALAR_UCPROBE_CONFIG_ADDR;
  uint32_t *user_config_addr = &Motor0_BLDC_SCALAR_ucprobe.user_config[0];
  if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_STOP)
  {
    Motor0_BLDC_SCALAR_MotorStop();
  }
  /*Erase and write 256 byte of data*/
  XMC_FLASH_ProgramVerifyPage(motor_conf_addr,user_config_addr); /*Address, data*/
}
/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_Write_Variable(void)
{

  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  if((Motor0_BLDC_SCALAR_ucprobe.user_config[0]&0x10000)!= 0)
  {
  /** Speed control PI */
    Motor0_BLDC_SCALAR_SpeedControl_PI.kp=Motor0_BLDC_SCALAR_ucprobe.user_config[1];
    Motor0_BLDC_SCALAR_SpeedControl_PI.ki= Motor0_BLDC_SCALAR_ucprobe.user_config[2];
    Motor0_BLDC_SCALAR_SpeedControl_PI.scale_kpki=Motor0_BLDC_SCALAR_ucprobe.user_config[3];
    Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_min=Motor0_BLDC_SCALAR_ucprobe.user_config[4];
    Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_max=Motor0_BLDC_SCALAR_ucprobe.user_config[5];
  }
  #endif

  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  if((Motor0_BLDC_SCALAR_ucprobe.user_config[0]&0x20000)!= 0)
  {
    /** Current control PI */
    Motor0_BLDC_SCALAR_CurrentControl_PI.kp=Motor0_BLDC_SCALAR_ucprobe.user_config[6];
    Motor0_BLDC_SCALAR_CurrentControl_PI.ki= Motor0_BLDC_SCALAR_ucprobe.user_config[7];
    Motor0_BLDC_SCALAR_CurrentControl_PI.scale_kpki=Motor0_BLDC_SCALAR_ucprobe.user_config[8];
    Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_min=Motor0_BLDC_SCALAR_ucprobe.user_config[9];
    Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_max=Motor0_BLDC_SCALAR_ucprobe.user_config[10];
  }
  #endif

  Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pulse_width = Motor0_BLDC_SCALAR_ucprobe.user_config[11];
  Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_current_decay = Motor0_BLDC_SCALAR_ucprobe.user_config[12];
  Motor0_BLDC_SCALAR_SL.startup_min_amplitude = Motor0_BLDC_SCALAR_ucprobe.user_config[13];
  Motor0_BLDC_SCALAR_SL.first_kick_time_count = Motor0_BLDC_SCALAR_ucprobe.user_config[14];
  Motor0_BLDC_SCALAR_SL.phase_energizing_time_count = Motor0_BLDC_SCALAR_ucprobe.user_config[15];
  Motor0_BLDC_SCALAR_StartupCurrentControl.user_current_set = Motor0_BLDC_SCALAR_ucprobe.user_config[16];
  Motor0_BLDC_SCALAR_SL.current_decay_time_count = Motor0_BLDC_SCALAR_ucprobe.user_config[17];
  Motor0_BLDC_SCALAR_SL.bemf_transition_emf = Motor0_BLDC_SCALAR_ucprobe.user_config[18];
  Motor0_BLDC_SCALAR_SL.configured_braking_amplitude_cmp_val = Motor0_BLDC_SCALAR_ucprobe.user_config[20];

  #if(MOTOR0_BLDC_SCALAR_ENABLE_BIDIRECTIONAL_CTRL == 0U)
  Motor0_BLDC_SCALAR_SetDirection(Motor0_BLDC_SCALAR_ucprobe.user_config[19]);
  #endif

}
/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_Read_Variable_SpeedPI(void)
{
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /** Speed control PI */
  Motor0_BLDC_SCALAR_ucprobe.user_config[1]= Motor0_BLDC_SCALAR_SpeedControl_PI.kp;
  Motor0_BLDC_SCALAR_ucprobe.user_config[2]= Motor0_BLDC_SCALAR_SpeedControl_PI.ki;
  Motor0_BLDC_SCALAR_ucprobe.user_config[3]= Motor0_BLDC_SCALAR_SpeedControl_PI.scale_kpki;
  Motor0_BLDC_SCALAR_ucprobe.user_config[4]= Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_min;
  Motor0_BLDC_SCALAR_ucprobe.user_config[5]= Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_max;
  #endif

}
/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_Read_Variable_CurrentPI(void)
{
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /** current control PI */
  Motor0_BLDC_SCALAR_ucprobe.user_config[6]= Motor0_BLDC_SCALAR_CurrentControl_PI.kp;
  Motor0_BLDC_SCALAR_ucprobe.user_config[7]= Motor0_BLDC_SCALAR_CurrentControl_PI.ki;
  Motor0_BLDC_SCALAR_ucprobe.user_config[8]= Motor0_BLDC_SCALAR_CurrentControl_PI.scale_kpki;
  Motor0_BLDC_SCALAR_ucprobe.user_config[9]= Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_min;
  Motor0_BLDC_SCALAR_ucprobe.user_config[10]= Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_max;
  #endif
}

void Motor0_BLDC_SCALAR_uCProbe_Read_Variable_Sensorless(void)
{
  Motor0_BLDC_SCALAR_ucprobe.user_config[11]= Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pulse_width;
  Motor0_BLDC_SCALAR_ucprobe.user_config[12]= Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_current_decay;
  Motor0_BLDC_SCALAR_ucprobe.user_config[13]= Motor0_BLDC_SCALAR_SL.startup_min_amplitude;
  Motor0_BLDC_SCALAR_ucprobe.user_config[14]= Motor0_BLDC_SCALAR_SL.first_kick_time_count;
  Motor0_BLDC_SCALAR_ucprobe.user_config[15]= Motor0_BLDC_SCALAR_SL.phase_energizing_time_count;
  Motor0_BLDC_SCALAR_ucprobe.user_config[16]= Motor0_BLDC_SCALAR_StartupCurrentControl.user_current_set;
  Motor0_BLDC_SCALAR_ucprobe.user_config[17]= Motor0_BLDC_SCALAR_SL.current_decay_time_count;
  Motor0_BLDC_SCALAR_ucprobe.user_config[18]= Motor0_BLDC_SCALAR_SL.bemf_transition_emf;
  Motor0_BLDC_SCALAR_ucprobe.user_config[20]= Motor0_BLDC_SCALAR_SL.configured_braking_amplitude_cmp_val;
}
/***********************************************************************************************************************/

#if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))
void Motor0_BLDC_SCALAR_Store_Default_value(void)
{
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /** Speed control PI */
  Motor0_BLDC_SCALAR_ucprobe.default_config[1]= Motor0_BLDC_SCALAR_SpeedControl_PI.kp;
  Motor0_BLDC_SCALAR_ucprobe.default_config[2]= Motor0_BLDC_SCALAR_SpeedControl_PI.ki;
  Motor0_BLDC_SCALAR_ucprobe.default_config[3]= Motor0_BLDC_SCALAR_SpeedControl_PI.scale_kpki;
  Motor0_BLDC_SCALAR_ucprobe.default_config[4]= Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_min;
  Motor0_BLDC_SCALAR_ucprobe.default_config[5]= Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_max;
#endif
#if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /** Speed control PI */
  Motor0_BLDC_SCALAR_ucprobe.default_config[6]= Motor0_BLDC_SCALAR_CurrentControl_PI.kp;
  Motor0_BLDC_SCALAR_ucprobe.default_config[7]= Motor0_BLDC_SCALAR_CurrentControl_PI.ki;
  Motor0_BLDC_SCALAR_ucprobe.default_config[8]= Motor0_BLDC_SCALAR_CurrentControl_PI.scale_kpki;
  Motor0_BLDC_SCALAR_ucprobe.default_config[9]= Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_min;
  Motor0_BLDC_SCALAR_ucprobe.default_config[10]= Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_max;

#endif

  Motor0_BLDC_SCALAR_ucprobe.default_config[11]= Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pulse_width;
  Motor0_BLDC_SCALAR_ucprobe.default_config[12]= Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_current_decay;
  Motor0_BLDC_SCALAR_ucprobe.default_config[13]= Motor0_BLDC_SCALAR_SL.startup_min_amplitude;
  Motor0_BLDC_SCALAR_ucprobe.default_config[14]= Motor0_BLDC_SCALAR_SL.first_kick_time_count;
  Motor0_BLDC_SCALAR_ucprobe.default_config[15]= Motor0_BLDC_SCALAR_SL.phase_energizing_time_count;
  Motor0_BLDC_SCALAR_ucprobe.default_config[16]= Motor0_BLDC_SCALAR_StartupCurrentControl.user_current_set;
  Motor0_BLDC_SCALAR_ucprobe.default_config[17]= Motor0_BLDC_SCALAR_SL.current_decay_time_count;
  Motor0_BLDC_SCALAR_ucprobe.default_config[18]= Motor0_BLDC_SCALAR_SL.bemf_transition_emf;
  Motor0_BLDC_SCALAR_ucprobe.default_config[20]= Motor0_BLDC_SCALAR_SL.configured_braking_amplitude_cmp_val;

  Motor0_BLDC_SCALAR_ucprobe.default_config[19] = Motor0_BLDC_SCALAR.motor_set_direction;

}
/***********************************************************************************************************************/


/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_SpeedPI_Reset(void)
{
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  /** Speed control PI */
   Motor0_BLDC_SCALAR_SpeedControl_PI.kp=Motor0_BLDC_SCALAR_ucprobe.default_config[1];
   Motor0_BLDC_SCALAR_SpeedControl_PI.ki= Motor0_BLDC_SCALAR_ucprobe.default_config[2];
   Motor0_BLDC_SCALAR_SpeedControl_PI.scale_kpki=Motor0_BLDC_SCALAR_ucprobe.default_config[3];
   Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_min=Motor0_BLDC_SCALAR_ucprobe.default_config[4];
   Motor0_BLDC_SCALAR_SpeedControl_PI.uk_limit_max=Motor0_BLDC_SCALAR_ucprobe.default_config[5];
#endif
}
/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCProbe_CurrentPI_Reset(void)
{
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))

  /** Current control PI */
  Motor0_BLDC_SCALAR_CurrentControl_PI.kp=Motor0_BLDC_SCALAR_ucprobe.default_config[6];
  Motor0_BLDC_SCALAR_CurrentControl_PI.ki= Motor0_BLDC_SCALAR_ucprobe.default_config[7];
  Motor0_BLDC_SCALAR_CurrentControl_PI.scale_kpki=Motor0_BLDC_SCALAR_ucprobe.default_config[8];
  Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_min=Motor0_BLDC_SCALAR_ucprobe.default_config[9];
  Motor0_BLDC_SCALAR_CurrentControl_PI.uk_limit_max=Motor0_BLDC_SCALAR_ucprobe.default_config[10];
  #endif
}
void Motor0_BLDC_SCALAR_uCProbe_Sensorless_Reset(void)
{
	  Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_pulse_width = Motor0_BLDC_SCALAR_ucprobe.default_config[11];
	  Motor0_BLDC_SCALAR_InductiveSensing.ind_sense_current_decay = Motor0_BLDC_SCALAR_ucprobe.default_config[12];
	  Motor0_BLDC_SCALAR_SL.startup_min_amplitude = Motor0_BLDC_SCALAR_ucprobe.default_config[13];
	  Motor0_BLDC_SCALAR_SL.first_kick_time_count = Motor0_BLDC_SCALAR_ucprobe.default_config[14];
	  Motor0_BLDC_SCALAR_SL.phase_energizing_time_count = Motor0_BLDC_SCALAR_ucprobe.default_config[15];
	  Motor0_BLDC_SCALAR_StartupCurrentControl.user_current_set = Motor0_BLDC_SCALAR_ucprobe.default_config[16];
	  Motor0_BLDC_SCALAR_SL.current_decay_time_count = Motor0_BLDC_SCALAR_ucprobe.default_config[17];
	  Motor0_BLDC_SCALAR_SL.bemf_transition_emf = Motor0_BLDC_SCALAR_ucprobe.default_config[18];
	  Motor0_BLDC_SCALAR_SL.configured_braking_amplitude_cmp_val = Motor0_BLDC_SCALAR_ucprobe.default_config[20];
}
/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_Write_Default_value(void)
{

  Motor0_BLDC_SCALAR_uCProbe_SpeedPI_Reset();
  Motor0_BLDC_SCALAR_uCProbe_CurrentPI_Reset();
  Motor0_BLDC_SCALAR_uCProbe_Sensorless_Reset();
  #if(MOTOR0_BLDC_SCALAR_ENABLE_BIDIRECTIONAL_CTRL == 0U)
  Motor0_BLDC_SCALAR_SetDirection(Motor0_BLDC_SCALAR_ucprobe.default_config[19]);
  #endif
}
/***********************************************************************************************************************/
void Motor0_BLDC_SCALAR_uCPorbe_Update_UI_Var(void)
{
  Motor0_BLDC_SCALAR_ucprobe.dclinkgvoltage_Volt = ((int32_t)(Motor0_BLDC_SCALAR.dclink_voltage *MOTOR0_BLDC_SCALAR_BASE_VOLTAGE*10) >>14);
  Motor0_BLDC_SCALAR_ucprobe.motor_current_mA =	((int32_t)(Motor0_BLDC_SCALAR.motor_current*MOTOR0_BLDC_SCALAR_BASE_CURRENT*1000)>>14);
  Motor0_BLDC_SCALAR_ucprobe.motor_speed_RPM =  ((int32_t)(Motor0_BLDC_SCALAR.motor_speed*MOTOR0_BLDC_SCALAR_BASE_SPEED_MECH_RPM)>>14);


  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_STOP)
  {
    Motor0_BLDC_SCALAR_ucprobe.speed_pi_error = (int32_t)(Motor0_BLDC_SCALAR_SpeedControl.ref_speed) - (int32_t)(Motor0_BLDC_SCALAR_SpeedControl.fdbk_speed);
    Motor0_BLDC_SCALAR_ucprobe.speed_set_RPM = ((int32_t)(Motor0_BLDC_SCALAR_SpeedControl.ref_speed*MOTOR0_BLDC_SCALAR_BASE_SPEED_MECH_RPM)>>14);
  }
  else
  {
    Motor0_BLDC_SCALAR_ucprobe.speed_pi_error=0;
    Motor0_BLDC_SCALAR_ucprobe.speed_set_RPM=0;
  }
  #endif
  #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
  if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_STOP)
  {
    Motor0_BLDC_SCALAR_ucprobe.current_pi_error=(int32_t)(Motor0_BLDC_SCALAR_CurrentControl.ref_current) - (int32_t)(Motor0_BLDC_SCALAR_CurrentControl.fdbk_current);
  }
  else
  {
    Motor0_BLDC_SCALAR_ucprobe.current_pi_error=0;
  }
  #endif

  #if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL)
  if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_STOP)
  {
    Motor0_BLDC_SCALAR_ucprobe.current_set_mA = ((int32_t)(Motor0_BLDC_SCALAR_CurrentControl.ref_current*MOTOR0_BLDC_SCALAR_BASE_CURRENT*1000)>>14);
  }
  else
  {
    Motor0_BLDC_SCALAR_ucprobe.current_set_mA =0;
 }
  #endif

  #if (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_VOLTAGE_CTRL)
  if (Motor0_BLDC_SCALAR.msm_state != BLDC_SCALAR_MSM_STOP)
  {
    Motor0_BLDC_SCALAR_ucprobe.voltage_set_Volt = ((int32_t)(Motor0_BLDC_SCALAR_VoltageControl.ref_voltage *MOTOR0_BLDC_SCALAR_BASE_VOLTAGE*10) >>14);
  }
  else
  {
    Motor0_BLDC_SCALAR_ucprobe.voltage_set_Volt =0;
  }
	#endif

  if(Motor0_BLDC_SCALAR.msm_state == BLDC_SCALAR_MSM_STOP)
  {
    #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEED_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
    Motor0_BLDC_SCALAR_ucprobe.speed_pi_error = 0;
    Motor0_BLDC_SCALAR_SpeedControl_PI.uk=0;
    Motor0_BLDC_SCALAR_ucprobe.speed_set_RPM=0;
    #endif
    #if ((MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_CURRENT_CTRL) || (MOTOR0_BLDC_SCALAR_CTRL_SCHEME == BLDC_SCALAR_SPEEDCURRENT_CTRL))
    Motor0_BLDC_SCALAR_ucprobe.current_pi_error=0;
    Motor0_BLDC_SCALAR_CurrentControl_PI.uk=0;
    #endif

  }
}

#endif /*#if ((MOTOR0_BLDC_SCALAR_CTRL_UCPROBE_ENABLE==1))*/
