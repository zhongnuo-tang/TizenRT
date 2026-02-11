/** mbed Microcontroller Library
  ******************************************************************************
  * @file    sys_api.c
  * @author
  * @version V1.0.0
  * @date    2016-08-01
  * @brief   This file provides following mbed system API:
  *				-JTAG OFF
  *				-LOGUART ON/OFF
  *				-OTA image switch
  *				-System Reset
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

#include "cmsis.h"
#include "sys_api.h"
#include "flash_api.h"
#include "log.h"
//#define printf					printf

static const char *const TAG = "SYS";

#define RSIP_REMAP_REGION_ADDR_SHIFT	12

/** @addtogroup Ameba_Mbed_API
  * @{
  */

/** @addtogroup MBED_SYSAPI
 *  @brief      MBED_SYSAPI driver modules.
 *  @{
 */

/** @defgroup MBED_SYSAPI_Exported_Functions MBED_SYSAPI Exported Functions
  * @{
  */

/**
  * @brief  Turn off the JTAG function.
  * @retval none
  */
void sys_jtag_off(void)
{
	Pinmux_Swdoff();
}

/**
  * @brief  Open log uart.
  * @retval none
  */
void sys_log_uart_on(void)
{
	/* Just Support S0 */
	Pinmux_UartLogCtrl(PINMUX_S0, ON);

	LOGUART_INTConfig(LOGUART_DEV, RUART_BIT_ERBI, ENABLE);
	LOGUART_RxCmd(LOGUART_DEV, ENABLE);
}

/**
  * @brief  Close log uart.
  * @retval none
  */
void sys_log_uart_off(void)
{
	LOGUART_INTConfig(LOGUART_DEV, RUART_BIT_ERBI, DISABLE);
	LOGUART_RxCmd(LOGUART_DEV, DISABLE);

	/* Just Support S0 */
	Pinmux_UartLogCtrl(PINMUX_S0, OFF);
}

/**
  * @brief  Store or load ADC calibration parameter.
  * @param  write: This parameter can be one of the following values:
  *	   @arg 0: Load ADC calibration parameter a & b & c.
  *    @arg 1: Store ADC calibration parameter a & b & c.
  * @param  a: Pointer to ADC parameter a.
  * @param  b: Pointer to ADC parameter b.
  * @param  c: Pointer to ADC parameter c.
  * @retval none
  */
void sys_adc_calibration(u8 write, u16 *a, u16 *b, u16 *c)
{
	/* To avoid gcc warnings */
	(void) write;
	(void) a;
	(void) b;
	(void) c;

	RTK_LOGA(TAG, "ADC calibration is finished in FT test. Calibration parameters can be found in EFUSE." \
			 "Please refer to Battery Measurement chapter in Application Note to get calibration parameters.\n");

	assert_param(0);
}

/**
  * @brief  System software reset.
  * @retval none
  */
void sys_reset(void)
{
	System_Reset();
}

/**
  * @brief Vector reset.
  * @retval none
  */
void sys_cpu_reset(void)
{
	RTK_LOGA(TAG, "AmebaGreen2 not support sys_cpu_reset function!\n");
	assert_param(0);
}
/** @} */
/** @} */
/** @} */


/******************* (C) COPYRIGHT 2016 Realtek Semiconductor *****END OF FILE****/
