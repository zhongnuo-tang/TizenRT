/****************************************************************************
 *
 * Copyright 2025 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <tinyara/config.h>

#define NR_IRQS (16 + 80)   /* Total number of interrupts

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 */

/* Processor Exceptions (vectors 0-15) */

#define AMEBAGREEN2_IRQ_RESERVED       (0) /* Reserved vector (only used with CONFIG_DEBUG) */
                                     /* Vector  0: Reset stack pointer value */
                                     /* Vector  1: Reset (not handler as an IRQ) */
#define AMEBAGREEN2_IRQ_NMI            (2) /* Vector  2: Non-Maskable Interrupt (NMI) */
#define AMEBAGREEN2_IRQ_HARDFAULT      (3) /* Vector  3: Hard fault */
#define AMEBAGREEN2_IRQ_MEMFAULT       (4) /* Vector  4: Memory management (MPU) */
#define AMEBAGREEN2_IRQ_BUSFAULT       (5) /* Vector  5: Bus fault */
#define AMEBAGREEN2_IRQ_USAGEFAULT     (6) /* Vector  6: Usage fault */
                                     /* Vectors 7-10: Reserved */
#define AMEBAGREEN2_IRQ_SVCALL        (11) /* Vector 11: SVC call */
#define AMEBAGREEN2_IRQ_DBGMONITOR    (12) /* Vector 12: Debug Monitor */
                                     /* Vector 13: Reserved */
#define AMEBAGREEN2_IRQ_PENDSV        (14) /* Vector 14: Pendable system service request */
#define AMEBAGREEN2_IRQ_SYSTICK       (15) /* Vector 15: System tick */

/* External interrupts (vectors >= 16).  These definitions are chip-specific */

#define AMEBAGREEN2_IRQ_FIRST         (16) /* Vector number of the first external interrupt */


#define AMEBAGREEN2_IRQ_WIFI_FISR_FESR_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 0)
#define AMEBAGREEN2_IRQ_WIFI_FTSR_MAILBOX_IRQ			(AMEBAGREEN2_IRQ_FIRST + 1)
#define AMEBAGREEN2_IRQ_WL_DMA_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 2)
#define AMEBAGREEN2_IRQ_WL_PROTOCOL_IRQ     			(AMEBAGREEN2_IRQ_FIRST + 3)
#define AMEBAGREEN2_IRQ_AP_WAKE_IRQ         			(AMEBAGREEN2_IRQ_FIRST + 4)
#define AMEBAGREEN2_IRQ_IPC_CPU0_IRQ        			(AMEBAGREEN2_IRQ_FIRST + 5)
#define AMEBAGREEN2_IRQ_IWDG_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 6)
#define AMEBAGREEN2_IRQ_TIMER0_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 7)
#define AMEBAGREEN2_IRQ_TIMER1_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 8)
#define AMEBAGREEN2_IRQ_TIMER2_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 9)
#define AMEBAGREEN2_IRQ_TIMER3_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 10)
#define AMEBAGREEN2_IRQ_TIMER4_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 11)
#define AMEBAGREEN2_IRQ_TIMER5_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 12)
#define AMEBAGREEN2_IRQ_TIMER6_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 13)
#define AMEBAGREEN2_IRQ_TIMER7_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 14)
#define AMEBAGREEN2_IRQ_TIMER8_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 15)
#define AMEBAGREEN2_IRQ_COEX_MAILBOX_IRQ    			(AMEBAGREEN2_IRQ_FIRST + 16)
#define AMEBAGREEN2_IRQ_RSVD_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 17)
#define AMEBAGREEN2_IRQ_PMC_TIMER0_IRQ      			(AMEBAGREEN2_IRQ_FIRST + 18)
#define AMEBAGREEN2_IRQ_PMC_TIMER1_IRQ      			(AMEBAGREEN2_IRQ_FIRST + 19)
#define AMEBAGREEN2_IRQ_UART0_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 20)
#define AMEBAGREEN2_IRQ_UART1_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 21)
#define AMEBAGREEN2_IRQ_UART2_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 22)
#define AMEBAGREEN2_IRQ_UART3_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 23)
#define AMEBAGREEN2_IRQ_UART_LOG_IRQ        			(AMEBAGREEN2_IRQ_FIRST + 24)
#define AMEBAGREEN2_IRQ_GPIOA_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 25)
#define AMEBAGREEN2_IRQ_GPIOB_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 26)
#define AMEBAGREEN2_IRQ_GPIOC_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 27)
#define AMEBAGREEN2_IRQ_I2C0_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 28)
#define AMEBAGREEN2_IRQ_I2C1_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 29)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL0_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 30)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL1_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 31)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL2_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 32)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL3_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 33)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL4_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 34)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL5_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 35)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL6_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 36)
#define AMEBAGREEN2_IRQ_GDMA0_CHANNEL7_IRQ  			(AMEBAGREEN2_IRQ_FIRST + 37)
#define AMEBAGREEN2_IRQ_SPI0_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 38)
#define AMEBAGREEN2_IRQ_SPI1_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 39)
#define AMEBAGREEN2_IRQ_SPORT0_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 40)
#define AMEBAGREEN2_IRQ_RTC_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 41)
#define AMEBAGREEN2_IRQ_ADC_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 42)
#define AMEBAGREEN2_IRQ_ADC_COMP_IRQ        			(AMEBAGREEN2_IRQ_FIRST + 43)
#define AMEBAGREEN2_IRQ_CAPTOUCH_IRQ        			(AMEBAGREEN2_IRQ_FIRST + 44)
#define AMEBAGREEN2_IRQ_THERMAL_IRQ         			(AMEBAGREEN2_IRQ_FIRST + 45)
#define AMEBAGREEN2_IRQ_BOR_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 46)
#define AMEBAGREEN2_IRQ_PWR_DOWN_IRQ        			(AMEBAGREEN2_IRQ_FIRST + 47)
#define AMEBAGREEN2_IRQ_RMII_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 48)
#define AMEBAGREEN2_IRQ_LCDC_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 49)
#define AMEBAGREEN2_IRQ_MJPEG_IRQ           			(AMEBAGREEN2_IRQ_FIRST + 50)
#define AMEBAGREEN2_IRQ_PPE_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 51)
#define AMEBAGREEN2_IRQ_PKE_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 52)
#define AMEBAGREEN2_IRQ_TRNG_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 53)
#define AMEBAGREEN2_IRQ_AON_TIM_IRQ         			(AMEBAGREEN2_IRQ_FIRST + 54)
#define AMEBAGREEN2_IRQ_AON_WAKEPIN_IRQ     			(AMEBAGREEN2_IRQ_FIRST + 55)
#define AMEBAGREEN2_IRQ_SDIO_WIFI_IRQ       			(AMEBAGREEN2_IRQ_FIRST + 56)
#define AMEBAGREEN2_IRQ_SDIO_BT_IRQ         			(AMEBAGREEN2_IRQ_FIRST + 57)
#define AMEBAGREEN2_IRQ_SDIO_HOST_IRQ       			(AMEBAGREEN2_IRQ_FIRST + 58)
#define AMEBAGREEN2_IRQ_USB_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 59)
#define AMEBAGREEN2_IRQ_A2C0_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 60)
#define AMEBAGREEN2_IRQ_A2C1_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 61)
#define AMEBAGREEN2_IRQ_IR_IRQ              			(AMEBAGREEN2_IRQ_FIRST + 62)
#define AMEBAGREEN2_IRQ_RXI300_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 63)
#define AMEBAGREEN2_IRQ_PSRAMC_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 64)
#define AMEBAGREEN2_IRQ_SPI_FLASH_IRQ       			(AMEBAGREEN2_IRQ_FIRST + 65)
#define AMEBAGREEN2_IRQ_RSIP_IRQ            			(AMEBAGREEN2_IRQ_FIRST + 66)
#define AMEBAGREEN2_IRQ_AES_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 67)
#define AMEBAGREEN2_IRQ_SHA_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 68)
#define AMEBAGREEN2_IRQ_CPU0_NS_WDG_IRQ     			(AMEBAGREEN2_IRQ_FIRST + 69)
#define AMEBAGREEN2_IRQ_CPU0_S_WDG_IRQ      			(AMEBAGREEN2_IRQ_FIRST + 70)
#define AMEBAGREEN2_IRQ_OCP_IRQ             			(AMEBAGREEN2_IRQ_FIRST + 71)
#define AMEBAGREEN2_IRQ_SPIC_ECC_IRQ        			(AMEBAGREEN2_IRQ_FIRST + 72)
#define AMEBAGREEN2_IRQ_UVC_DEC_IRQ         			(AMEBAGREEN2_IRQ_FIRST + 73)
#define AMEBAGREEN2_IRQ_RTC_DET_IRQ         			(AMEBAGREEN2_IRQ_FIRST + 74)
#define AMEBAGREEN2_IRQ_BT_MAILBOX_IRQ      			(AMEBAGREEN2_IRQ_FIRST + 75)
#define AMEBAGREEN2_IRQ_BT_SCB_IRQ          			(AMEBAGREEN2_IRQ_FIRST + 76)
#define AMEBAGREEN2_IRQ_BT_WAKE_HOST_IRQ    			(AMEBAGREEN2_IRQ_FIRST + 77)
#define AMEBAGREEN2_IRQ_CPU1_WDG_RST_IRQ    			(AMEBAGREEN2_IRQ_FIRST + 78)
