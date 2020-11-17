/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    LPC845_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC845.h"
#include "fsl_debug_console.h"
#include "fsl_power.h"
#include "fsl_acomp.h"
#include "fsl_capt.h"

#define DEMO_ACOMP_BASE            ACOMP
#define DEMO_ACOMP_CAPT_CHANNEL    5U
#define DEMO_CAPT_BASE             CAPT
#define DEMO_CAPT_NOISE            0U
#define DEMO_CAPT_IRQ_NUMBER       CMP_CAPT_IRQn
#define DEMO_CAPT_IRQ_HANDLER_FUNC CMP_CAPT_DriverIRQHandler
#define DEMO_CAPT_ENABLE_PINS                                                                                       \
		kCAPT_X0Pin | kCAPT_X1Pin | kCAPT_X2Pin | kCAPT_X3Pin | kCAPT_X4Pin | kCAPT_X5Pin | kCAPT_X6Pin | kCAPT_X7Pin | \
		kCAPT_X8Pin

#define DEMO_CAPT_FILTER_NUM 10U



#define I2C_DATA_LENGTH            3U
#define I2C_SLAVE_PAYLOAD_LENGTH            4U
#define I2C_MASTER_PAYLOAD_LENGTH            4U

#ifdef __GNUC__
#define PACKED  __attribute__((__packed__))
#endif

/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */


typedef union {		// Payload from slave to master
	struct PACKED {
		uint16_t adc_val;
		uint8_t tactile_switch;
		uint8_t touch_button;
	} data;
	uint8_t payload_arr[4];
} i2c_payload_slave_t;


typedef union {
	struct PACKED leds {
		unsigned red_led : 1;
		unsigned green_led : 1;
		unsigned blue_led : 1;
		unsigned reserved : 5;
	} led_state_bit;
	uint8_t leds_state;
} leds_state_t;

typedef struct {
	uint8_t rgb_red;
	uint8_t rgb_green;
	uint8_t rgb_blue;
} rgb_info_t;

typedef union {		// Payload from slave to master
	struct PACKED {
		leds_state_t leds_state;	// 1 byte. Each bit is Active/Inactive
		rgb_info_t rgb_info;  // Not implemented yet
	} data;
	uint8_t payload_arr[4];
} i2c_payload_master_t;


i2c_payload_slave_t I2C_Payload_slave;
i2c_payload_master_t I2C_payload_master;


uint8_t touch_button_active = false;

uint32_t g_value[16] = {0U};
capt_touch_data_t g_data;
volatile bool g_YesTouchFlag;

static uint32_t CAPT_DoCalibration(CAPT_Type *base);
static void ACOMP_Configuration(void);

void fill_slave_I2C_Payload(uint32_t adc_val, bool tactile_switch, bool touch_button) {
	I2C_Payload_slave.data.adc_val = adc_val;
	I2C_Payload_slave.data.tactile_switch = tactile_switch;
	I2C_Payload_slave.data.touch_button = touch_button;


}

void BOARD_LED_ON(uint32_t index)
{
	switch (index)
	{
	case 0U:
		LED_GREEN_ON();
		break;
	default:
		break;
	}
}


void BOARD_InitLED(void)
{

	LED_GREEN_INIT(LOGIC_LED_OFF);
	/* BSP does not provide macro for configuring all LEDs. Calling each LED Init separately
	 * resets the port losing previous pin-specific configuration*/
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, \
			&(gpio_pin_config_t){kGPIO_DigitalOutput, (LOGIC_LED_OFF)});
	GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, \
			&(gpio_pin_config_t){kGPIO_DigitalOutput, (LOGIC_LED_OFF)});

    GPIO_PortInit(GPIO, 0);

    GPIO_PinInit(BOARD_LED_GREEN_GPIO, 0, 4, \
    			&(gpio_pin_config_t){kGPIO_DigitalInput, (0)});
}

void BOARD_LED_OFF(void)
{
	LED_GREEN_OFF();
}


void DEMO_CAPT_IRQ_HANDLER_FUNC(void)
{
	uint32_t n, mask, largest = 0U;
	static uint32_t filter_index;
	static uint8_t largests[DEMO_CAPT_FILTER_NUM];

	CAPT_GetTouchData(DEMO_CAPT_BASE, &g_data);
	g_value[g_data.XpinsIndex] = g_data.count;
	mask                       = CAPT_GetInterruptStatusFlags(DEMO_CAPT_BASE);
	CAPT_ClearInterruptStatusFlags(DEMO_CAPT_BASE, mask);
	if (mask & kCAPT_InterruptOfYesTouchEnable)
	{
		g_YesTouchFlag = true;
	}
	if (mask & kCAPT_InterruptOfPollDoneEnable)
	{
		if (g_YesTouchFlag)
		{
			for (n = 0; n < CAPT_GET_XMAX_NUMBER(DEMO_CAPT_BASE->STATUS); n++)
			{
				if (g_value[n + 1] > g_value[largest])
				{
					largest = n + 1;
				}
			}
			largests[filter_index++] = largest;
			if (filter_index == DEMO_CAPT_FILTER_NUM)
			{
				filter_index = 0U;
			}
			for (n = 0; n < DEMO_CAPT_FILTER_NUM - 1; n++)
			{
				if (largests[n] != largests[n + 1])
				{
					g_YesTouchFlag = false;
					break;
				}
			}
			if (g_YesTouchFlag)
			{
				//BOARD_LED_ON(largest);
				touch_button_active = 1U;
				g_YesTouchFlag = false;
			}
			else
			{
				//BOARD_LED_OFF();
				touch_button_active = 0U;
			}
		}
		else
		{
			for (n = 0; n < CAPT_GET_XMAX_NUMBER(DEMO_CAPT_BASE->STATUS); n++)
			{
				largests[n] = n;
			}
			//BOARD_LED_OFF();
			touch_button_active = 0U;
		}
	}
}

void delay() {
	volatile uint32_t i = 0;
	for (i = 0; i < 100000; i++) {
		asm("NOP");
	}

}
/*
 * @brief   Application entry point.
 */
int main(void) {

	adc_result_info_t adcResultInfoStruct;
	//uint8_t g_slave_buff[I2C_DATA_LENGTH];
	status_t reVal = kStatus_Fail;
	uint32_t thresold;
	capt_config_t captConfig;
	uint8_t tactile_switch_state;
	uint8_t prev_led_status;

	POWER_DisablePD(kPDRUNCFG_PD_ADC0);
	POWER_DisablePD(kPDRUNCFG_PD_ACMP);
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();
	BOARD_InitLED();


#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
	/* Init FSL debug console. */
	BOARD_InitDebugConsole();
#endif

	PRINTF("Hello World\n");

	ACOMP_Configuration();
	/* Initialize CAPT module. */
	CAPT_GetDefaultConfig(&captConfig);
	captConfig.triggerMode = kCAPT_ComparatorTriggerMode;
	captConfig.XpinsMode   = kCAPT_InactiveXpinsHighZMode;
	/* Calculate the clock divider to make sure CAPT work in 2Mhz fclk. */
	captConfig.clockDivider = (CLOCK_GetFroFreq() / 2000000U - 1U);
	captConfig.enableXpins  = DEMO_CAPT_ENABLE_PINS;
	CAPT_Init(DEMO_CAPT_BASE, &captConfig);

	/* Do the self-calibration. */
	thresold = CAPT_DoCalibration(DEMO_CAPT_BASE);
	CAPT_SetThreshold(DEMO_CAPT_BASE, thresold - DEMO_CAPT_NOISE);
	PRINTF("Calibration has finished.\r\n");
	BOARD_LED_OFF();
	/* Enable the interrupts. */
	CAPT_EnableInterrupts(DEMO_CAPT_BASE, kCAPT_InterruptOfYesTouchEnable | kCAPT_InterruptOfNoTouchEnable |
			kCAPT_InterruptOfPollDoneEnable);
	NVIC_EnableIRQ(DEMO_CAPT_IRQ_NUMBER);

	/* Set polling mode and start poll. */
	CAPT_SetPollMode(DEMO_CAPT_BASE, kCAPT_PollContinuousMode);

	//memset(g_slave_buff, 0, sizeof(I2C_payload_master.payload_arr));

	I2C_SlaveEnable(I2C0_PERIPHERAL, true);

	/* Enter an infinite loop, just incrementing a counter. */

	I2C_payload_master.data.leds_state.led_state_bit.red_led = 0U;

	while(1) {


		reVal = I2C_SlaveReadBlocking(I2C0_PERIPHERAL, I2C_payload_master.payload_arr, I2C_MASTER_PAYLOAD_LENGTH);

		if (reVal != kStatus_Success)
		{
			// return -1;
		}


		if (I2C_payload_master.data.leds_state.leds_state  != prev_led_status) {
			if (I2C_payload_master.data.leds_state.led_state_bit.red_led) {
				LED_RED_ON();
			} else {
				LED_RED_OFF();
			}
			if (I2C_payload_master.data.leds_state.led_state_bit.green_led) {
				LED_GREEN_ON();
			} else {
				LED_GREEN_OFF();
			}
			if (I2C_payload_master.data.leds_state.led_state_bit.blue_led) {
				LED_BLUE_ON();
			} else {
				LED_BLUE_OFF();
			}
			prev_led_status = I2C_payload_master.data.leds_state.leds_state;
		}

		ADC_DoSoftwareTriggerConvSeqA(ADC0_PERIPHERAL);
		while (!ADC_GetConvSeqAGlobalConversionResult(ADC0_PERIPHERAL, &adcResultInfoStruct))
		{
		}

		tactile_switch_state = !GPIO_PinRead(GPIO, 0 , 4);	// 1 -> SW is pressed


		fill_slave_I2C_Payload(adcResultInfoStruct.result, tactile_switch_state, touch_button_active);

		reVal = I2C_SlaveWriteBlocking(I2C0_PERIPHERAL, I2C_Payload_slave.payload_arr, I2C_SLAVE_PAYLOAD_LENGTH);
		//reVal = I2C_SlaveWriteBlocking(I2C0_PERIPHERAL, &g_slave_buff[0], 1);

		if (reVal != kStatus_Success)
		{
			//return -1;
		}


	}
	return 0 ;
}

static void ACOMP_Configuration(void)
{
	acomp_config_t acompConfig;
	acomp_ladder_config_t acompLadderConfig;

	acompConfig.enableSyncToBusClk  = false;
	acompConfig.hysteresisSelection = kACOMP_Hysteresis20MVSelection;
	ACOMP_Init(ACOMP, &acompConfig);

	ACOMP_EnableInterrupts(ACOMP, kACOMP_InterruptsDisable);

	ACOMP_SetInputChannel(ACOMP, DEMO_ACOMP_CAPT_CHANNEL, 0U);

	acompLadderConfig.ladderValue      = 0x10U;
	acompLadderConfig.referenceVoltage = kACOMP_LadderRefVoltagePinVDD;
	ACOMP_SetLadderConfig(ACOMP, &acompLadderConfig);
}

static uint32_t CAPT_DoCalibration(CAPT_Type *base)
{
	uint16_t xpinIndex, repeatCounter;
	uint32_t temp_count, temp_count_high, temp_count_low;
	uint32_t temp_xPins, max_xpins;
	uint32_t average_count[16] = {0U};

	max_xpins  = CAPT_GET_XMAX_NUMBER(CAPT_GetStatusFlags(base));
	temp_xPins = base->CTRL & CAPT_CTRL_XPINSEL_MASK;
	if (base->POLL_TCNT & CAPT_POLL_TCNT_TCHLOW_ER_MASK)
	{
		temp_count = 0xFFFU;
	}
	else
	{
		temp_count = 0U;
	}
	for (xpinIndex = 0U; xpinIndex <= max_xpins; xpinIndex++)
	{
		if (temp_xPins & (1U << (CAPT_CTRL_XPINSEL_SHIFT + xpinIndex)))
		{
			/* Before writing into CTRL register, INCHANGE(bit 15)should equal '0'. */
			while (CAPT_CTRL_INCHANGE_MASK == (CAPT_CTRL_INCHANGE_MASK & base->CTRL))
			{
			}
			base->CTRL = (base->CTRL & ~CAPT_CTRL_XPINSEL_MASK) | 1U << (CAPT_CTRL_XPINSEL_SHIFT + xpinIndex);
			for (repeatCounter = 0U; repeatCounter < 100U; ++repeatCounter)
			{
				/* Before writing into CTRL register, INCHANGE(bit 15)should equal '0'. */
				while (CAPT_CTRL_INCHANGE_MASK == (CAPT_CTRL_INCHANGE_MASK & base->CTRL))
				{
				}
				/* Start poll-now mode. */
				base->CTRL &= ~CAPT_CTRL_POLLMODE_MASK;
				base->CTRL |= CAPT_CTRL_POLLMODE(kCAPT_PollNowMode);
				/* Wait for poll-now done. */
				while (!(base->STATUS & CAPT_STATUS_POLLDONE_MASK))
				{
				}
				/* Clear the status flags. */
				base->STATUS |= base->STATUS;
				average_count[xpinIndex] += ((base->TOUCH & CAPT_TOUCH_COUNT_MASK) >> CAPT_TOUCH_COUNT_SHIFT);
			}
			average_count[xpinIndex] /= 100U;
		}
	}
	/* Restore the xpins. */
	base->CTRL = (base->CTRL & ~CAPT_CTRL_XPINSEL_MASK) | temp_xPins;

	temp_count_high = 0U;
	temp_count_low  = 0xFFFU;
	for (xpinIndex = 0U; xpinIndex <= max_xpins; xpinIndex++)
	{
		if (temp_xPins & (1U << (CAPT_CTRL_XPINSEL_SHIFT + xpinIndex)))
		{
			if (temp_count_high < average_count[xpinIndex])
			{
				temp_count_high = average_count[xpinIndex];
			}
			if (temp_count_low > average_count[xpinIndex])
			{
				temp_count_low = average_count[xpinIndex];
			}
		}
	}
	/* For touchlower mode.*/
	if (base->POLL_TCNT & CAPT_POLL_TCNT_TCHLOW_ER_MASK)
	{
		temp_count = 2 * temp_count_low - temp_count_high;
	}
	else /* For touchhiger mode. */
	{
		temp_count = 2 * temp_count_high - temp_count_low;
	}

	return temp_count;
}
