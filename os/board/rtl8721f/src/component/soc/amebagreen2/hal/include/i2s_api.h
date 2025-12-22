/*******************************************************************************
 * @file    i2s_api.h
 * @author
 * @version V1.0.0
 * @brief   This file provides following mbed I2S API
 ******************************************************************************
 * @attention
 *
 * Copyright(c) 2025 Realtek Corporation. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

#ifndef MBED_EXT_I2S_API_EXT_H
#define MBED_EXT_I2S_API_EXT_H

#include "device.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#ifdef CONFIG_ARCH_CHIP_AMEBAGREEN2
enum {
	SR_8KHZ      = SP_8K,
	SR_12KHZ     = SP_12K,
	SR_16KHZ     = SP_16K,
	SR_24KHZ     = SP_24K,
	SR_32KHZ     = SP_32K,
	SR_48KHZ     = SP_48K,
	SR_96KHZ     = SP_96K,
	SR_192KHZ    = SP_192K,
	SR_384KHZ    = SP_384K,
	SR_11p02PKHZ = SP_11P025K,
	SR_22P05KHZ  = SP_22P05K,
	SR_44P1KHZ   = SP_44P1K,
	SR_88P2KHZ   = SP_88P2K,
	SR_176P4KHZ  = SP_176P4K
};

enum {
	CH_STEREO = SP_CH_STEREO,
	CH_MONO   = SP_CH_MONO
};

enum {
	WL_16b    = SP_TXWL_16,
	WL_24b    = SP_TXWL_24,
	WL_32b    = SP_TXWL_32,
	WL_RX_16b = SP_RXWL_16,
	WL_RX_24b = SP_RXWL_24,
	WL_RX_32b = SP_RXWL_32
};

enum {
	CL_16b    = SP_TXCL_16,
	CL_24b    = SP_TXCL_24,
	CL_32b    = SP_TXCL_32,
	CL_RX_16b = SP_RXCL_16,
	CL_RX_24b = SP_RXCL_24,
	CL_RX_32b = SP_RXCL_32
};

enum {
	I2S_IDX1 = I2S0,
	I2S_IDX2 = I2S1
};

enum {
	I2S_DIR_RX   = SP_DIR_RX,   // Rx Only
	I2S_DIR_TX   = SP_DIR_TX,   // Tx Only
};
#endif /* CONFIG_ARCH_CHIP_AMEBAGREEN2 */

typedef void (*i2s_irq_handler)(uint32_t id, char *pbuf);

typedef struct i2s_s i2s_t;

/****************************************************************************
 * Public Function Declarations
 ****************************************************************************/

/**
 * @brief  Initialize the I2S device, including clock, function, interrupt and I2S registers.
 * @param  obj: I2S object defined in application software.
 * @param  sck: Serial clock PinName according to pinmux spec.
 * @param  ws: Word select PinName according to pinmux spec.
 * @param  sd_tx: Tx PinName according to pinmux spec.
 * @param  sd_rx: Rx PinName according to pinmux spec.
 * @param  mck: Master clock PinName according to pinmux spec.
 * @retval none
 */
void i2s_init(i2s_t *obj, PinName sck, PinName ws, PinName sd_tx, PinName sd_rx, PinName mck);

/**
 * @brief  Set I2S channel number, sample rate and word length.
 * @param  obj: I2S object defined in application software.
 * @param  channel_num: This parameter can be one of the following values:
 * @arg CH_STEREO: Stereo channel.
 * @arg CH_MONO: Mono channel.
 * @param  rate: This parameter can be one of the following values:
 * @arg SR_8KHZ: Sample rate is 8kHz.
 * @arg SR_12KHZ: Sample rate is 12kHz.
 * @arg SR_16KHZ: Sample rate is 16kHz.
 * @arg SR_24KHZ: Sample rate is 24kHz.
 * @arg SR_32KHZ: Sample rate is 32kHz.
 * @arg SR_48KHZ: Sample rate is 48kHz.
 * @arg SR_64KHZ: Sample rate is 64kHz.
 * @arg SR_96KHZ: Sample rate is 96kHz.
 * @arg SR_192KHZ: Sample rate is 192kHz.
 * @arg SR_384KHZ: Sample rate is 384kHz.
 * @arg SR_11p025KHZ: Sample rate is 11.025kHz.
 * @arg SR_22p05KHZ: Sample rate is 22.05kHz.
 * @arg SR_44p1KHZ: Sample rate is 44.1kHz.
 * @arg SR_88p2KHZ: Sample rate is 88.2kHz.
 * @arg SR_176p4KHZ: Sample rate is 176.4kHz.
 * @param  word_len: This parameter can be one of the following values:
 * @arg WL_16b: Sample bit is 16 bit.
 * @arg WL_24b: Sample bit is 24 bit.
 * @arg WL_32b: Sample bit is 32 bit.
 * @retval none
 */
void i2s_set_param(i2s_t *obj, uint8_t channel_num, uint32_t sampling_rate, uint8_t word_length);

/**
 * @brief  Deinitialize the I2S device, including function, interrupt and I2S registers.
 * @param  obj: I2S object defined in application software.
 * @retval none
 */
void i2s_deinit(i2s_t *obj);

/**
 * @brief  Get current tx page address.
 * @param  obj: I2S object defined in application software.
 * @return Address of current tx page if it is owned by CPU or NULL if it is owned by I2S.
 */
int *i2s_get_tx_page(i2s_t *obj);

/**
 * @brief  Set current tx page owned by I2S.
 * @retval none
 */
void i2s_send_page(void);

/**
 * @brief  Set current rx page owned by I2S.
 * @param  obj: I2S object defined in application software.
 * @retval none
 */
void i2s_recv_page(i2s_t *obj);

/**
 * @brief  Enable I2S interrupt and function.
 * @param  obj: I2S object defined in application software.
 * @retval none
 */
void i2s_enable(i2s_t *obj);

/**
 * @brief  Disable I2S interrupt and function.
 * @param  obj: I2S object defined in application software.
 * @retval none
 */
void i2s_disable(i2s_t *obj, bool is_suspend);

/**
 * @brief  Pause I2S interrupt and function.
 * @param  obj: I2S object defined in application software.
 * @retval none
 */
void ameba_i2s_pause(i2s_t *obj);

/**
 * @brief  Resume I2S interrupt and function.
 * @param  obj: I2S object defined in application software.
 * @retval none
 */
void ameba_i2s_resume(i2s_t *obj);

/**
 * @brief  Register RX interrupt handler.
 * @param  obj: I2S object defined in application software.
 * @param  handler: RX interrupt callback function.
 * @param  id: RX interrupt callback parameter.
 * @retval none
 */
void i2s_rx_irq_handler(i2s_t *obj, i2s_irq_handler handler, uint32_t id);

/**
 * @brief  Check whether tx dma is done.
 * @param  page_num: Page number.
 * @retval OK: tx dma is done. -1: tx dma is not done.
 */
int i2s_dma_tx_done(uint8_t page_num);

/**
 * @brief  Set I2S data transfer direction.
 * @param  obj: I2S object defined in application software.
 * @param  trx_type: Transfer direction. This parameter can be one of the following values:
 * @arg I2S_DIR_RX: Rx receive direction.
 * @arg I2S_DIR_TX: Tx transmission direction.
 * @retval none
 */
void i2s_set_direction(i2s_t *obj, int trx_type);

/**
 * @brief  Register TX interrupt handler.
 * @param  obj: I2S object defined in application software.
 * @param  handler: TX interrupt callback function.
 * @param  id: TX interrupt callback parameter.
 * @retval none
 */
void i2s_tx_irq_handler(i2s_t *obj, i2s_irq_handler handler, uint32_t id);

/**
 * @brief  Set page number, page size and page address.
 * @param  obj: I2S object defined in application software.
 * @param  tx_buf: Pointer to the start address of Tx page.
 * @param  rx_buf: Pointer to the start address of Rx page.
 * @param  page_num: Page number. This parameter must be set to a value in the 2~8 range.
 * @param  page_size: Page size. This parameter must be set to a value in the 2~32768 bytes range.
 * @retval none
 */
void i2s_set_dma_buffer(i2s_t *obj, char *tx_buf, char *rx_buf, uint32_t page_num, uint32_t page_size);

#endif /* MBED_EXT_I2S_API_EXT_H */

#ifdef __cplusplus
}
#endif /* __cplusplus */
