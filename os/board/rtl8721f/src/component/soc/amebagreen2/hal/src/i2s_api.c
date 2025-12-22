/******************************************************************************
 * @file    i2s_api.c
 * @author
 * @version V1.0.0
 * @date    2025-12-29
 * @brief   This file provides mbed API for I2S.
 ******************************************************************************
 * @attention
 *
 * This module is a confidential and proprietary property of RealTek and
 * possession or use of this module requires written permission of RealTek.
 *
 * Copyright(c) 2025, Realtek Semiconductor Corporation. All rights reserved.
 ******************************************************************************
 */
#include "i2s_api.h"
#include "ameba_audio_clock.h"
#include "ameba_soc.h"
#include "amebagreen2_i2s.h"
#include "objects.h"

/****************************************************************************
 * Private Macro Definitions
 ****************************************************************************/ 

#define SP_MAX_DMA_PAGE_NUM 8
#define I2S_CODEC_MULTIPLIER 256

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/ 

typedef struct
{
	void (*TxCCB)(uint32_t id, char *pbuf);
	uint32_t TxCBId;
	void (*RxCCB)(uint32_t id, char *pbuf);
	uint32_t RxCBId;
} I2S_USER_CB;

typedef struct
{
	uint8_t tx_gdma_own;
	uint32_t tx_addr;
} TX_BLOCK, *pTX_BLOCK;

typedef struct
{
	TX_BLOCK tx_block[SP_MAX_DMA_PAGE_NUM];
	TX_BLOCK tx_zero_block;
	uint8_t tx_gdma_cnt;
	uint8_t tx_usr_cnt;
	uint8_t tx_empty_flag;
	uint8_t tx_page_num;
	uint16_t tx_page_size;
} SP_TX_INFO, *pSP_TX_INFO;

typedef struct
{
	uint8_t rx_gdma_own;
	uint32_t rx_addr;
} RX_BLOCK, *pRX_BLOCK;

typedef struct
{
	RX_BLOCK rx_block[SP_MAX_DMA_PAGE_NUM];
	RX_BLOCK rx_full_block;
	uint8_t rx_gdma_cnt;
	uint8_t rx_usr_cnt;
	uint8_t rx_full_flag;
	uint8_t rx_page_num;
	uint16_t rx_page_size;
} SP_RX_INFO, *pSP_RX_INFO;

typedef struct
{
	GDMA_InitTypeDef SpTxGdmaInitStruct; /* Pointer to GDMA_InitTypeDef */
	GDMA_InitTypeDef SpRxGdmaInitStruct; /* Pointer to GDMA_InitTypeDef */
	uint8_t i2s_idx;
} SP_GDMA_STRUCT;

static SP_InitTypeDef SP_InitStruct;
static SP_GDMA_STRUCT SPGdmaStruct;
static I2S_USER_CB I2SUserCB; /* Pointer to I2S User Callback */

#ifdef CONFIG_AMEBAGREEN2_I2S_TX
static SP_TX_INFO sp_tx_info;
static struct GDMA_CH_LLI LliTx[SP_MAX_DMA_PAGE_NUM];
#endif /* CONFIG_AMEBAGREEN2_I2S_TX */

#ifdef CONFIG_AMEBAGREEN2_I2S_RX
static SP_RX_INFO sp_rx_info;
static struct GDMA_CH_LLI LliRx[SP_MAX_DMA_PAGE_NUM];
#endif /* CONFIG_AMEBAGREEN2_I2S_RX */


/****************************************************************************
 * Private Function Declarations
 ****************************************************************************/

/**
 * @brief  Set I2S clock select
 * @param  obj: I2S object defined in application software.
 * @retval clock_mode： Clock mode
 */
static uint32_t i2s_clock_select(i2s_t *obj);

/**
 * @brief  Initialize I2S parameter structure.
 * @param  obj: I2S object defined in application software.
 * @param  clock_mode: I2S clock mode.
 * @retval none
 */
static void i2s_param_init(i2s_t *obj, uint32_t clock_mode);

/**
 * @brief  Initialize I2S clock.
 * @param  none
 * @retval none
 */
static void i2s_clock_init(void);

/**
 * @brief  Initialize I2S pinmux.
 * @param  obj: I2S object defined in application software.
 * @param  sck: I2S SCK pin name.
 * @param  ws: I2S WS pin name.
 * @param  sd_tx: I2S SD TX pin name.
 * @param  sd_rx: I2S SD RX pin name.
 * @param  mck: I2S MCK pin name.
 * @retval none
 */
static void i2s_pin_init(i2s_t *obj, PinName sck, PinName ws, PinName sd_tx, PinName sd_rx, PinName mck);
static void i2s_tx_isr(void *sp_data);
static void i2s_rx_isr(void *sp_data);
static uint32_t *i2s_get_ready_tx_page(uint8_t i2s_index);
static void i2s_release_rx_page(uint8_t i2s_index);
static void i2s_release_tx_page(uint8_t i2s_index);
static void ameba_audio_set_sp_data_out(uint32_t index);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

 void i2s_init(i2s_t *obj, PinName sck, PinName ws, PinName sd_tx, PinName sd_rx, PinName mck)
{
	uint32_t index = obj->i2s_idx;
	assert_param(IS_SP_SEL_I2S(obj->i2s_idx));
	i2s_clock_init();
	i2s_pin_init(obj, sck, ws, sd_tx, sd_rx, mck);
	i2s_set_param(obj, obj->channel_num, obj->sampling_rate, obj->word_length);
}

void i2s_set_param(i2s_t *obj, uint8_t channel_num, uint32_t sampling_rate, uint8_t word_length)
{
	uint32_t clock_mode = i2s_clock_select(obj);
	obj->channel_num = channel_num;
	obj->sampling_rate = sampling_rate;
	obj->word_length = word_length;

	i2s_param_init(obj, clock_mode);
	AUDIO_SP_Reset(obj->i2s_idx);
	AUDIO_SP_SetMclk(obj->i2s_idx, ENABLE);
	AUDIO_SP_Init(obj->i2s_idx, obj->direction, &SP_InitStruct);
	AUDIO_SP_SetMasterSlave(obj->i2s_idx, obj->role);
}

void i2s_deinit(i2s_t *obj)
{
	SP_GDMA_STRUCT *l_SPGdmaStruct = &SPGdmaStruct;

	if (obj->i2s_idx == I2S0) {
		RCC_PeriphClockCmd(APBPeriph_SPORT, APBPeriph_SPORT_CLOCK, DISABLE);
	}

	GDMA_ClearINT(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum);
	GDMA_Cmd(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum, DISABLE);
	GDMA_ChnlFree(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum);
	AUDIO_SP_Unregister(obj->i2s_idx, obj->direction);
	AUDIO_SP_Deinit(obj->i2s_idx, obj->direction);
}

void i2s_enable(i2s_t *obj)
{
	AUDIO_SP_DmaCmd(obj->i2s_idx, ENABLE);
	if (obj->direction == I2S_DIR_TX) {
		AUDIO_SP_TXStart(obj->i2s_idx, ENABLE);
	} else {
		AUDIO_SP_RXStart(obj->i2s_idx, ENABLE);
	}
}

void i2s_disable(i2s_t *obj, bool is_suspend)
{
	SP_GDMA_STRUCT *l_SPGdmaStruct = &SPGdmaStruct;

	if (obj->direction == I2S_DIR_TX) {
		GDMA_ClearINT(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum);
		GDMA_Abort(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum);
		GDMA_ChnlFree(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum);
		AUDIO_SP_DmaCmd(obj->i2s_idx, DISABLE);
		AUDIO_SP_TXStart(obj->i2s_idx, DISABLE);
		if (is_suspend) {
			AUDIO_SP_Deinit(obj->i2s_idx, obj->direction);
			/* deinit the peripheral clock when suspending to avoid invalid state */
			if (obj->i2s_idx == I2S0) {
				RCC_PeriphClockCmd(APBPeriph_SPORT, APBPeriph_SPORT_CLOCK, DISABLE);
			}
		}
	} else {
		GDMA_ClearINT(l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_ChNum);
		GDMA_Cmd(l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_ChNum, DISABLE);

		AUDIO_SP_DmaCmd(obj->i2s_idx, DISABLE);
		AUDIO_SP_RXStart(obj->i2s_idx, DISABLE);
	}
}

void i2s_set_direction(i2s_t *obj, int trx_type)
{
	obj->direction = trx_type;

	AUDIO_SP_Init(obj->i2s_idx, obj->direction, &SP_InitStruct);
}

void i2s_set_dma_buffer(i2s_t *obj, char *tx_buf, char *rx_buf, uint32_t page_num, uint32_t page_size)
{
	assert_param(IS_SP_SEL_I2S(obj->i2s_idx));

#if defined(CONFIG_AMEBAGREEN2_I2S_TX) || defined(CONFIG_AMEBAGREEN2_I2S_RX)
	uint32_t i, j;
#endif /* CONFIG_AMEBAGREEN2_I2S_TX || CONFIG_AMEBAGREEN2_I2S_RX */

	if ((page_num < 2) || (page_num > 8) || (page_size < 8)) {
		DBG_PRINTF(MODULE_I2S, LEVEL_INFO, "%s: PageNum(%d) valid value is 2~8; PageSize(%d must > 8)\r\n", __FUNCTION__, page_num, page_size);
		return;
	}
#ifdef CONFIG_AMEBAGREEN2_I2S_TX
	if (obj->direction == I2S_DIR_TX) {

		sp_tx_info.tx_gdma_cnt = 0;
		sp_tx_info.tx_usr_cnt = 0;
		sp_tx_info.tx_empty_flag = 0;

		_memset(tx_buf, 0, (page_num + 1) * page_size);
		DCache_CleanInvalidate((u32)tx_buf, (page_num + 1) * page_size);

		for (i = 0; i < page_num; i++) {
			sp_tx_info.tx_block[i].tx_gdma_own = 0;
			sp_tx_info.tx_block[i].tx_addr = (uint32_t)tx_buf + i * page_size;
		}

		sp_tx_info.tx_zero_block.tx_addr = (uint32_t)tx_buf + page_num * page_size;

		for (i = 0; i < page_size; i++) {
			((uint8_t *)(sp_tx_info.tx_zero_block.tx_addr))[i] = 0;
		}

		sp_tx_info.tx_page_size = page_size;
		sp_tx_info.tx_page_num = page_num;

		for (j = 0; j < page_num + 1; j++) {
			if (j == 0) {
				LliTx[j].LliEle.Sarx = (uint32_t)tx_buf + (page_num)*page_size;
			} else {
				LliTx[j].LliEle.Sarx = (uint32_t)tx_buf + (j - 1) * page_size;
			}

			if (j == page_num) {
				LliTx[j].pNextLli = &LliTx[1];
			} else {
				LliTx[j].pNextLli = &LliTx[j + 1];
			}
		}
	}
#endif /* CONFIG_AMEBAGREEN2_I2S_TX */
#ifdef CONFIG_AMEBAGREEN2_I2S_RX
	else {
		sp_rx_info.rx_gdma_cnt = 0;
		sp_rx_info.rx_usr_cnt = 0;
		sp_rx_info.rx_full_flag = 0;

		for (i = 0; i < page_num; i++) {
			sp_rx_info.rx_block[i].rx_gdma_own = 1;
			sp_rx_info.rx_block[i].rx_addr = (uint32_t)rx_buf + i * page_size;
		}

		sp_rx_info.rx_full_block.rx_addr = (uint32_t)rx_buf + page_num * page_size;

		for (i = 0; i < page_size; i++) {
			((uint8_t *)(sp_rx_info.rx_full_block.rx_addr))[i] = 0;
		}

		sp_rx_info.rx_page_size = page_size;
		sp_rx_info.rx_page_num = page_num;

		for (j = 0; j < page_num; j++) {
			LliRx[j].LliEle.Darx = (uint32_t)rx_buf + j * page_size;
			if (j == page_num - 1) {
				LliRx[j].pNextLli = &LliRx[0];
			} else {
				LliRx[j].pNextLli = &LliRx[j + 1];
			}
		}
	}
#endif /* CONFIG_AMEBAGREEN2_I2S_RX */
}

void ameba_i2s_pause(i2s_t *obj)
{

	SP_GDMA_STRUCT *l_SPGdmaStruct = &SPGdmaStruct;
	if (obj->direction == I2S_DIR_TX) {
		GDMA_ClearINT(l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpTxGdmaInitStruct.GDMA_ChNum);
		AUDIO_SP_DmaCmd(obj->i2s_idx, DISABLE);
		AUDIO_SP_TXStart(obj->i2s_idx, DISABLE);
	} else {
		GDMA_Suspend(l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_ChNum);
	}
}

void ameba_i2s_resume(i2s_t *obj)
{

	SP_GDMA_STRUCT *l_SPGdmaStruct = &SPGdmaStruct;
	if (obj->direction == I2S_DIR_TX) {
		AUDIO_SP_DmaCmd(obj->i2s_idx, ENABLE);
		AUDIO_SP_TXStart(obj->i2s_idx, ENABLE);
	} else {
		GDMA_Resume(l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_Index, l_SPGdmaStruct->SpRxGdmaInitStruct.GDMA_ChNum);
	}
}

#ifdef CONFIG_AMEBAGREEN2_I2S_RX
void i2s_recv_page(i2s_t *obj)
{
	uint8_t i2s_index = obj->i2s_idx;

	pRX_BLOCK prx_block = &(sp_rx_info.rx_block[sp_rx_info.rx_usr_cnt]);

	prx_block->rx_gdma_own = 1;
	sp_rx_info.rx_usr_cnt++;
	if (sp_rx_info.rx_usr_cnt == sp_rx_info.rx_page_num) {
		sp_rx_info.rx_usr_cnt = 0;
	}
}

void i2s_rx_irq_handler(i2s_t *obj, i2s_irq_handler handler, uint32_t id)
{
	uint8_t i2s_index = obj->i2s_idx;
	SP_GDMA_STRUCT *sp_str = &SPGdmaStruct;
	sp_str->i2s_idx = i2s_index; /* Store I2S index */

	I2SUserCB.RxCCB = handler;
	I2SUserCB.RxCBId = id;

	i2s_get_free_rx_page(i2s_index);
	AUDIO_SP_LLPRXGDMA_Init(i2s_index,
							GDMA_INT,
							&sp_str->SpRxGdmaInitStruct,
							sp_str,
							(IRQ_FUN)i2s_rx_isr,
							sp_rx_info.rx_page_size,
							sp_rx_info.rx_page_num,
							LliRx);
}
#endif

#ifdef CONFIG_AMEBAGREEN2_I2S_TX
int *i2s_get_tx_page(i2s_t *obj)
{
	pTX_BLOCK ptx_block = &(sp_tx_info.tx_block[sp_tx_info.tx_usr_cnt]);

	if (ptx_block->tx_gdma_own) {
		return NULL;
	} else {
		return (int *)ptx_block->tx_addr;
	}
}

void i2s_send_page(void)
{
	pTX_BLOCK ptx_block = &(sp_tx_info.tx_block[sp_tx_info.tx_usr_cnt]);

	DCache_CleanInvalidate((uint32_t)ptx_block->tx_addr, sp_tx_info.tx_page_size);
	ptx_block->tx_gdma_own = 1;
	sp_tx_info.tx_usr_cnt++;
	if (sp_tx_info.tx_usr_cnt == sp_tx_info.tx_page_num) {
		sp_tx_info.tx_usr_cnt = 0;
	}
}

int i2s_dma_tx_done(uint8_t page_num)
{
	(void)page_num;
	for (int i = 0; i < sp_tx_info.tx_page_num; i++) {
		pTX_BLOCK ptx_block = &(sp_tx_info.tx_block[i]);
		if (ptx_block->tx_gdma_own) {
			return -1;
		}
	}
	return OK;
}

void i2s_tx_irq_handler(i2s_t *obj, i2s_irq_handler handler, uint32_t id)
{
	assert_param(IS_SP_SEL_I2S(obj->i2s_idx));

	uint8_t i2s_index = obj->i2s_idx;
	SP_GDMA_STRUCT *sp_str = &SPGdmaStruct;

	sp_str->i2s_idx = i2s_index; /* Store I2S index */

	I2SUserCB.TxCCB = handler;
	I2SUserCB.TxCBId = id;

	i2s_get_ready_tx_page(i2s_index);
	AUDIO_SP_LLPTXGDMA_Init(i2s_index, GDMA_INT,
							&sp_str->SpTxGdmaInitStruct,
							sp_str,
							(IRQ_FUN)i2s_tx_isr,
							sp_tx_info.tx_page_size,
							sp_tx_info.tx_page_num + 1, LliTx);
}
#endif /* CONFIG_AMEBAGREEN2_I2S_TX */

/****************************************************************************
 * Private Function Definitions
 ****************************************************************************/

#ifdef CONFIG_AMEBAGREEN2_I2S_TX
static uint32_t i2s_clock_select(i2s_t *obj)
{
	uint32_t clock_mode;
	AUDIO_InitParams Init_Params;
	AUDIO_ClockParams Clock_Params;

	Init_Params.chn_len = SP_CL_32;
	Init_Params.chn_cnt = obj->channel_num;
	Init_Params.sr = obj->sampling_rate;
	Init_Params.codec_multiplier_with_rate = I2S_CODEC_MULTIPLIER;
	Init_Params.sport_mclk_fixed_max = 0U;
	Audio_Clock_Choose(PLL_CLK, &Init_Params, &Clock_Params);
	obj->clock = Clock_Params.Clock;

	switch (Clock_Params.Clock) {
	case PLL_CLOCK_45P1584M:
		RCC_PeriphClockSourceSet(I2S, SYS_PLL);
		RCC_PeriphClockDividerSet(SYS_PLL_I2S, 8);
		RCC_PeriphClockDividerFENSet(SYS_PLL_I2S, ENABLE);
		clock_mode = PLL_CLOCK_45P1584M;
		break;
	case PLL_CLOCK_98P304M:
		clock_mode = PLL_CLOCK_98P304M;
		RCC_PeriphClockSourceSet(I2S, SYS_PLL);
		RCC_PeriphClockDividerSet(SYS_PLL_I2S, 4);
		RCC_PeriphClockDividerFENSet(SYS_PLL_I2S, ENABLE);
		break;
	case I2S_CLOCK_XTAL40M:
		RCC_PeriphClockSourceSet(I2S, XTAL);
		clock_mode = I2S_CLOCK_XTAL40M;
		break;
	}
	return clock_mode;
}

static void i2s_param_init(i2s_t *obj, uint32_t clock_mode)
{
	AUDIO_SP_StructInit(&SP_InitStruct);
	SP_InitStruct.SP_SelFIFO = obj->fifo_num;
	SP_InitStruct.SP_SelChLen = obj->channel_length;
	SP_InitStruct.SP_SelWordLen = obj->word_length;
	SP_InitStruct.SP_SetMultiIO = obj->mode;
	SP_InitStruct.SP_SR = obj->sampling_rate;
	SP_InitStruct.SP_SelClk = clock_mode;
}

static void i2s_clock_init(void)
{
	RCC_PeriphClockCmd(APBPeriph_SPORT, APBPeriph_SPORT_CLOCK, ENABLE);
	RCC_PeriphClockCmd(APBPeriph_AC, APBPeriph_AC_CLOCK, ENABLE);
	RCC_PeriphClockSourceSet(I2S, XTAL);
}

static void i2s_pin_init(i2s_t *obj, PinName sck, PinName ws, PinName sd_tx, PinName sd_rx, PinName mck)
{
	if (mck) {
		Pinmux_Config(mck, PINMUX_FUNCTION_I2S0_MCLK);
	}
	Pinmux_Config(sck, PINMUX_FUNCTION_I2S0_BCLK);
	Pinmux_Config(ws, PINMUX_FUNCTION_I2S0_WS);
	ameba_audio_set_sp_data_out(0);
	Pinmux_Config(sd_tx, PINMUX_FUNCTION_I2S0_DIO3);
}

static void i2s_tx_isr(void *sp_data)
{
	u32 *pbuf;
	SP_GDMA_STRUCT *gs = sp_data;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &(gs->SpTxGdmaInitStruct);

	uint8_t i2s_index = gs->i2s_idx;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);

	i2s_release_tx_page(i2s_index);
	pbuf = i2s_get_ready_tx_page(i2s_index);
	I2SUserCB.TxCCB(I2SUserCB.TxCBId, (char *)pbuf);
}

static uint32_t *i2s_get_ready_tx_page(uint8_t i2s_index)
{
	pTX_BLOCK ptx_block = &(sp_tx_info.tx_block[sp_tx_info.tx_gdma_cnt]);

	if (ptx_block->tx_gdma_own) {
		sp_tx_info.tx_empty_flag = 0;
		return (uint32_t *)ptx_block->tx_addr;
	} else {
		sp_tx_info.tx_empty_flag = 1;
		return (uint32_t *)sp_tx_info.tx_zero_block.tx_addr; // for audio buffer empty case
	}
}

static void i2s_release_tx_page(uint8_t i2s_index)
{
	pTX_BLOCK ptx_block = &(sp_tx_info.tx_block[sp_tx_info.tx_gdma_cnt]);

	if (sp_tx_info.tx_empty_flag) {
		/* Do nothing */
	} else {
		memcpy((void *)ptx_block->tx_addr, (const void *)sp_tx_info.tx_zero_block.tx_addr, sp_tx_info.tx_page_size);
		DCache_CleanInvalidate((uint32_t)ptx_block->tx_addr, sp_tx_info.tx_page_size);
		ptx_block->tx_gdma_own = 0;
		sp_tx_info.tx_gdma_cnt++;
		if (sp_tx_info.tx_gdma_cnt == sp_tx_info.tx_page_num) {
			sp_tx_info.tx_gdma_cnt = 0;
		}
	}
}
#endif /* CONFIG_AMEBAGREEN2_I2S_TX */

static void ameba_audio_set_sp_data_out(uint32_t index)
{
	(void)index;
	uint32_t tmp;
	tmp = HAL_READ32(PINMUX_REG_BASE, REG_I2S_CTRL);
	tmp |= PAD_BIT_SP0_DIO0_MUXSEL | PAD_BIT_SP0_DIO1_MUXSEL | PAD_BIT_SP0_DIO2_MUXSEL | PAD_BIT_SP0_DIO3_MUXSEL;

	HAL_WRITE32(PINMUX_REG_BASE, REG_I2S_CTRL, tmp);
}

#ifdef CONFIG_AMEBAGREEN2_I2S_RX
static void i2s_rx_isr(void *sp_data)
{
	SP_GDMA_STRUCT *gs = sp_data;
	PGDMA_InitTypeDef GDMA_InitStruct;
	GDMA_InitStruct = &(gs->SpRxGdmaInitStruct);

	uint8_t i2s_index = gs->i2s_idx;

	/* Clear Pending ISR */
	GDMA_ClearINT(GDMA_InitStruct->GDMA_Index, GDMA_InitStruct->GDMA_ChNum);
	i2s_release_rx_page(i2s_index);

	/* Read data */
	pRX_BLOCK prx_block = &(sp_rx_info.rx_block[sp_rx_info.rx_usr_cnt]);
	DCache_CleanInvalidate((uint32_t)prx_block->rx_addr, sp_rx_info.rx_page_size);
	I2SUserCB.RxCCB((uint32_t)NULL, (void *)(uint32_t)prx_block->rx_addr);
	i2s_get_free_rx_page(i2s_index);
}

static void i2s_release_rx_page(uint8_t i2s_index)
{
	pRX_BLOCK prx_block = &(sp_rx_info.rx_block[sp_rx_info.rx_gdma_cnt]);

	if (sp_rx_info.rx_full_flag) {
	} else {
		prx_block->rx_gdma_own = 0;
		sp_rx_info.rx_gdma_cnt++;
		if (sp_rx_info.rx_gdma_cnt == sp_rx_info.rx_page_num) {
			sp_rx_info.rx_gdma_cnt = 0;
		}
	}
}

static uint32_t *i2s_get_free_rx_page(uint8_t i2s_index)
{
	pRX_BLOCK prx_block = &(sp_rx_info.rx_block[sp_rx_info.rx_gdma_cnt]);

	if (prx_block->rx_gdma_own) {
		sp_rx_info.rx_full_flag = 0;
		return (uint32_t *)prx_block->rx_addr;
	} else {
		sp_rx_info.rx_full_flag = 1;
		return (uint32_t *)sp_rx_info.rx_full_block.rx_addr; // for audio buffer full case
	}
}
#endif /* CONFIG_AMEBAGREEN2_I2S_RX */
