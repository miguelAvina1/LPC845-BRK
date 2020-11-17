/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v8.0
processor: LPC845
package_id: LPC845M301JBD48
mcu_data: ksdk2_0
processor_version: 8.0.2
board: LPC845BREAKOUT
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: f445b125-4879-4fb9-b5d6-4049aadbd03d
  called_from_default_init: true
  selectedCore: core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * ADC0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'ADC0'
- type: 'lpc_adc'
- mode: 'ADC'
- custom_name_enabled: 'false'
- type_id: 'lpc_adc_d74172b5bd0591c0d32a6c93c043a67f'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'ADC0'
- config_sets:
  - fsl_adc:
    - doSelfCalibration: 'true'
    - adcConfig:
      - clockMode: 'kADC_ClockSynchronousMode'
      - clockSourceFreq: 'BOARD_BootClockFRO18M'
      - clockDividerNumber: '1'
      - enableLowPowerMode: 'false'
      - voltageRange: 'kADC_HighVoltageRange'
    - thresholdConfig:
      - ADC_SetThreshold0: 'false'
      - thresholdSettingsPair0:
        - lowValue: '0'
        - highValue: '0'
      - ADC_SetThreshold1: 'false'
      - thresholdSettingsPair1:
        - lowValue: '0'
        - highValue: '0'
    - enableSeqConfigurationA: 'true'
    - adcConvSeqConfigA:
      - enableHighPriority: 'true'
      - adcConvSeqConfig:
        - triggerMaskM: '0U'
        - triggerPolarity: 'kADC_TriggerPolarityPositiveEdge'
        - enableSyncBypass: 'false'
        - enableSingleStep: 'false'
        - interruptMode: 'kADC_InterruptForEachConversion'
    - enableSeqConfigurationB: 'false'
    - adcConvSeqConfigB:
      - enableHighPriority: 'false'
      - adcConvSeqConfig:
        - triggerMaskM: '0U'
        - triggerPolarity: 'kADC_TriggerPolarityNegativeEdge'
        - enableSyncBypass: 'false'
        - enableSingleStep: 'false'
        - interruptMode: 'kADC_InterruptForEachConversion'
    - channels:
      - 0:
        - channelNumber: 'CH.0'
        - channelThresholdPair: 'thresholdPair0'
        - thresholdInterruptMode: 'kADC_ThresholdInterruptDisabled'
        - conversion_sequence: 'a'
    - interrupt_sel: ''
    - enable_irq_seqA: 'false'
    - adc_interrupt_seqA:
      - IRQn: 'ADC0_SEQA_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - enable_irq_seqB: 'false'
    - adc_interrupt_seqB:
      - IRQn: 'ADC0_SEQB_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - enable_irq_thcmp: 'false'
    - adc_interrupt_thcmp:
      - IRQn: 'ADC0_THCMP_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
    - enable_irq_ovr: 'false'
    - adc_interrupt_ovr:
      - IRQn: 'ADC0_OVR_IRQn'
      - enable_interrrupt: 'enabled'
      - enable_priority: 'false'
      - priority: '0'
      - enable_custom_name: 'false'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const adc_config_t ADC0configStruct = {
  .clockMode = kADC_ClockSynchronousMode,
  .clockDividerNumber = 1UL,
  .enableLowPowerMode = false,
  .voltageRange = kADC_HighVoltageRange,
};
/* Conversion sequence A configuration structure */
const adc_conv_seq_config_t ADC0ConvSeqAConfigStruct = {
  .channelMask = 1U,
  .triggerMask = 0U,
  .triggerPolarity = kADC_TriggerPolarityPositiveEdge,
  .enableSyncBypass = false,
  .enableSingleStep = false,
  .interruptMode = kADC_InterruptForEachConversion
};

static void ADC0_init(void) {
  /* Perform self calibration */
  ADC_DoSelfCalibration(ADC0_PERIPHERAL, ADC0_CLK_FREQ);
  /* Initialize ADC0 peripheral */
  ADC_Init(ADC0_PERIPHERAL, &ADC0configStruct);
  /* Configure priority for sequence A */
  ADC_SetConvSeqAHighPriority(ADC0_PERIPHERAL);
  /* Configure the conversion sequence A */
  ADC_SetConvSeqAConfig(ADC0_PERIPHERAL, &ADC0ConvSeqAConfigStruct);
  /* Enable the conversion sequence A */
  ADC_EnableConvSeqA(ADC0_PERIPHERAL, true);
  /* Configure threshold compare interrupt on channel 0 */
  ADC_EnableThresholdCompareInterrupt(ADC0_PERIPHERAL, 0U, kADC_ThresholdInterruptDisabled);
}

/***********************************************************************************************************************
 * I2C0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'I2C0'
- type: 'lpc_i2c'
- mode: 'I2C_Polling'
- custom_name_enabled: 'false'
- type_id: 'lpc_i2c_f5051a0134792729f1007113ec6ddccd'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'I2C0'
- config_sets:
  - fsl_i2c:
    - i2c_mode: 'kI2C_Slave'
    - clockSource: 'FunctionClock'
    - clockSourceFreq: 'BOARD_BootClockFRO18M'
    - i2c_slave_config:
      - enableSlave: 'true'
      - address0:
        - address: '0x7E'
        - addressDisable: 'false'
        - qualMode: 'kI2C_QualModeMask'
        - qualAddress: '0'
      - address1:
        - address: '0'
        - addressDisable: 'true'
      - address2:
        - address: '0'
        - addressDisable: 'true'
      - address3:
        - address: '0'
        - addressDisable: 'true'
      - busSpeed: 'kI2C_SlaveStandardMode'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const i2c_slave_config_t I2C0_config = {
  .enableSlave = true,
  .address0 = {
    .address = 0x7EU,
    .addressDisable = false
  },
  .qualMode = kI2C_QualModeMask,
  .qualAddress = 0,
  .address1 = {
    .address = 0U,
    .addressDisable = true
  },
  .address2 = {
    .address = 0U,
    .addressDisable = true
  },
  .address3 = {
    .address = 0U,
    .addressDisable = true
  },
  .busSpeed = kI2C_SlaveStandardMode
};

static void I2C0_init(void) {
  /* Initialization function */
  I2C_SlaveInit(I2C0_PERIPHERAL, &I2C0_config, I2C0_CLOCK_SOURCE);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  ADC0_init();
  I2C0_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
