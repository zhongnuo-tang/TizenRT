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

#define SECURE_ADDR_OFFSET           (0x10000000)
#define KBYTES(x)                    ((x) * 1024)
#define IMAGE_HEADER_LEN             (0x20)
#define KM4TZ_KM4NS_RAM_SIZE         KBYTES(20)
#define SRAM_BASE                    (0x20000000)
#define SRAM_END                     (0x20080000)
#define PSRAM_BASE                   (0x60000000)
#define PSRAM_END                    (0x60400000)

/* layout configuration */
#define TZ_NSC_SIZE                  (4)
#define TZ_ENTRY_SIZE                (16)
#define TZ_S_SIZE                    (44)
#if defined(CONFIG_IMG3_FLASH)
#define TZ_S_BSS_SIZE                (16)
#endif

#if defined(CONFIG_IMG3_SRAM)
#define RAM_TZ_NSC_SIZE              KBYTES(TZ_NSC_SIZE)
#define RAM_TZ_ENTRY_SIZE            KBYTES(TZ_ENTRY_SIZE)
#define RAM_KM4TZ_IMG3_SIZE          KBYTES(TZ_S_SIZE)
#define PSRAM_TZ_NSC_SIZE            KBYTES(0)
#define PSRAM_TZ_ENTRY_SIZE          KBYTES(0)
#define PSRAM_KM4TZ_IMG3_SIZE        KBYTES(0)
#elif defined(CONFIG_IMG3_PSRAM)
#define RAM_TZ_NSC_SIZE              KBYTES(0)
#define RAM_TZ_ENTRY_SIZE            KBYTES(0)
#define RAM_KM4TZ_IMG3_SIZE          KBYTES(0)
#define PSRAM_TZ_NSC_SIZE            KBYTES(TZ_NSC_SIZE)
#define PSRAM_TZ_ENTRY_SIZE          KBYTES(TZ_ENTRY_SIZE)
#define PSRAM_KM4TZ_IMG3_SIZE        KBYTES(TZ_S_SIZE)
#elif defined(CONFIG_IMG3_FLASH)
#define RAM_TZ_NSC_SIZE              KBYTES(TZ_NSC_SIZE)
#define RAM_TZ_ENTRY_SIZE            KBYTES(TZ_ENTRY_SIZE)
#define RAM_KM4TZ_IMG3_SIZE          KBYTES(TZ_S_BSS_SIZE)
#define PSRAM_TZ_NSC_SIZE            KBYTES(0)
#define PSRAM_TZ_ENTRY_SIZE          KBYTES(0)
#define PSRAM_KM4TZ_IMG3_SIZE        KBYTES(0)
#else /* undefine trustzone */
#define RAM_TZ_NSC_SIZE              KBYTES(0)
#define RAM_TZ_ENTRY_SIZE            KBYTES(0)
#define RAM_KM4TZ_IMG3_SIZE          KBYTES(0)
#define PSRAM_TZ_NSC_SIZE            KBYTES(0)
#define PSRAM_TZ_ENTRY_SIZE          KBYTES(0)
#define PSRAM_KM4TZ_IMG3_SIZE        KBYTES(0)
#endif

#if defined(CONFIG_IMG1_FLASH)
#define KM4TZ_IMG1_SIZE              KBYTES(4)
#else
#define KM4TZ_IMG1_SIZE              KBYTES(32)
#endif

#define RAM_KM4NS_IMG2_SIZE          KBYTES(192)
/*
                  amebagreen2 SRAM Layout (512K)
0x20000000 ──────────►┌──────────────────────┐
                      │     KM4TZ/KM4NS RAM  │
                      │       (20K)          │
                      │    <Realtek Fixed>   │
                      │                      │
fix addr              │                      │
0x20005000 ──────────►├──────────────────────┼──┐
                      │KM4TZ Bootloader(IMG1)│  ├───────►  KM4TZ_IMG1_SIZE
                      │                      │  │
                   ┌──┼──────────────────────┼──┴─┐
                   │  │      Trustzone(IMG3) │    │
If Trustzone off,  │  │      TZ_NSC(4K)      │    │        If Trustzone on,
              ◄────┤  │     TZ_ENTRY(16K)    │    ├─────►  IMG3 = RAM_TZ_NSC_SIZE + RAM_TZ_ENTRY_SIZE + RAM_KM4TZ_IMG3_SIZE
IMG3 size is 0.    │  │   KM4TZ_IMG3(44K)    │    │
                   └──┼──────────────────────┼────┴─┐
                      │                      │      │
                      │                      │      │
                      │    KM4TZ IMG2 RAM    │      │
                      │                      │      ├───►  KM4TZ_IMG2_SIZE = KM4NS_IMG2_RAM_START - KM4TZ_IMG2_RAM_START
                      │                      │      │
                      │                      │      │
                      ├──────────────────────┼──────┴─┐
                      │                      │        │
                      │    KM4NS IMG2 RAM    │        │
                      │                      │        ├─►  RAM_KM4NS_IMG2_SIZE
                      │                      │        │
                      │                      │        │
                      │                      │        │
 0x20080000──────────►└──────────────────────┴────────┘
*/

/* BOOT(IMG1)/IMG3/IMG2 layout*/                                                            /* Trstzone On (IMG1 On Flash) | Trustzone off (IMG1 On Flash) */
#define KM4TZ_IMG1_RAM_SATRT          (SRAM_BASE + KM4TZ_KM4NS_RAM_SIZE)                    /* IMG1 On SRAM: 0x20005000 ~ 0x2000D000 ; IMG1 On Flash: 0x20005000 ~ 0x20006000*/
#define KM4TZ_RAM_TZ_NSC_START        (KM4TZ_IMG1_RAM_SATRT + KM4TZ_IMG1_SIZE )             /* 0x20006000 ~ 0x20007000 | unavailable */
#define KM4TZ_RAM_TZ_ENTRY_START      (KM4TZ_RAM_TZ_NSC_START + RAM_TZ_NSC_SIZE )           /* 0x20007000 ~ 0x2000B000 | unavailable */
#define KM4TZ_RAM_TZ_SECURE_START     (KM4TZ_RAM_TZ_ENTRY_START + RAM_TZ_ENTRY_SIZE )       /* 0x2000B000 ~ 0x20016000 | unavailable */
#define KM4TZ_IMG2_RAM_START          (KM4TZ_RAM_TZ_SECURE_START + RAM_KM4TZ_IMG3_SIZE)     /* 0x20016000 ~ 0x20050000 | 0x20006000 ~ 0x20050000*/
#define KM4NS_IMG2_RAM_START          (SRAM_END - RAM_KM4NS_IMG2_SIZE)                      /* 0x20050000 ~ 0x20080000 | 0x20050000 ~ 0x20080000*/

/* IMG3 On PSRAM */
#define KM4TZ_PSRAM_TZ_NSC_START      (PSRAM_BASE)                                          /* 0x60000000 ~ 0x60001000 | unavailable */
#define KM4TZ_PSRAM_TZ_ENTRY_START    (KM4TZ_PSRAM_TZ_NSC_START + PSRAM_TZ_NSC_SIZE )       /* 0x60001000 ~ 0x60005000 | unavailable */
#define KM4TZ_PSRAM_TZ_SECURE_START   (KM4TZ_PSRAM_TZ_ENTRY_START + PSRAM_TZ_ENTRY_SIZE)    /* 0x60005000 ~ 0x60025000 | unavailable */

#define PSRAM_KM4TZ_IMG2_SIZE         (KBYTES(3220) - (PSRAM_TZ_NSC_SIZE + PSRAM_TZ_ENTRY_SIZE + PSRAM_KM4TZ_IMG3_SIZE))
#define KM4TZ_IMG2_PSRAM_START        (KM4TZ_PSRAM_TZ_SECURE_START + PSRAM_KM4TZ_IMG3_SIZE)    /* 0x60025000 ~ 0x60325000 | 0x60000000 ~ 0x60325000*/
#define KM4NS_IMG2_PSRAM_START        (KM4TZ_IMG2_PSRAM_START + PSRAM_KM4TZ_IMG2_SIZE)         /* 0x60325000 ~ 0x60400000 | 0x60325000 ~ 0x60400000*/
#define PSRAM_KM4NS_IMG2_SIZE         (PSRAM_END - KM4NS_IMG2_PSRAM_START)

MEMORY
{
    EMPTY_SECTION (rwx) :                  ORIGIN = 0x00000000, LENGTH = 0x00000000 - 0x00000000    /* This region shall be empty */

    /* KM4TZ(CPU0) ROM */
    KM4TZ_IROM (rx) :                      ORIGIN = 0x00000000, LENGTH = 0x0001D000 - 0x00000000    /* KM4TZ IROM_S: 116K */
    KM4TZ_DROM (rx) :                      ORIGIN = 0x0001D000, LENGTH = 0x0001F000 - 0x0001D000    /* KM4TZ DROM_S: 8k */
    KM4TZ_ONLY_ROM (rx) :                  ORIGIN = 0x0001F000, LENGTH = 0x00020000 - 0x0001F000    /* KM4TZ_ONLY_ROM: 4k */

    /* SHARE ROM */
    KM4NS_SYS_ROM (rx) :                   ORIGIN = 0x00100000, LENGTH = 0x00101000 - 0x00100000    /* KM4NS NP_SYS_ROM: 4k */
    SHARE_IROM (rx) :                      ORIGIN = 0x00101000, LENGTH = 0x0010C000 - 0x00101000    /* SHARE IROM: 44k */
    SHARE_DROM (rx) :                      ORIGIN = 0x0010C000, LENGTH = 0x0010F000 - 0x0010C000    /* SHARE DROM: 12k */
    OS_ROM (rx) :                          ORIGIN = 0x0010F000, LENGTH = 0x00114000 - 0x0010F000    /* OS ROM: 20k */
    WIFI_ROM (rx) :                        ORIGIN = 0x00114000, LENGTH = 0x00147800 - 0x00114000    /* WIFI ROM: 206k */
    DELAY_ROM (rx) :                       ORIGIN = 0x00147800, LENGTH = 0x00148000 - 0x00147800    /* DELAY ROM: 2k */

    /* KM4TZ Secure RAM, BIT(28)=1 */
    KM4TZ_ROMBSS_RAM_S (rw) :              ORIGIN = 0x30000000, LENGTH = 0x30000600 - 0x30000000    /* KM4TZ ROM BSS RAM S: 1.5k */
    KM4TZ_MSP_RAM_S (rw) :                 ORIGIN = 0x30000600, LENGTH = 0x30001000 - 0x30000600    /* KM4TZ MSP_S RAM: 2.5k */

    /* KM4TZ Non-secure RAM */
    KM4TZ_ROMBSS_RAM_NS (rw) :             ORIGIN = 0x20001000, LENGTH = 0x20002400 - 0x20001000    /* KM4TZ ROM BSS NS RAM: 5k */
    KM4TZ_MSP_RAM_NS (rw) :                ORIGIN = 0x20002400, LENGTH = 0x20003000 - 0x20002400    /* KM4TZ MSP_NS RAM: 3k */

    /* KM4NS RAM */
    KM4NS_ROMBSS_MMU (rw) :                ORIGIN = 0x20001000, LENGTH = 0x20002400 - 0x20001000    /* Remap to AP_ROMBSS_RAM_NS */
    KM4NS_ROMBSS_RAM (rw) :                ORIGIN = 0x20003000, LENGTH = 0x20004400 - 0x20003000    /* KM4NS ROM BSS RAM: 5k */
    KM4NS_MSP_RAM (rw) :                   ORIGIN = 0x20004600, LENGTH = 0x20005000 - 0x20004600    /* KM4NS MSP RAM: 3k */

    /* KM4NS_IPC_RAM need to be delete */
    KM4NS_IPC_RAM (rwx) :                  ORIGIN = 0x20004400, LENGTH = 0x20004600 - 0x20004400    /* KM4TZ PMC RAM: 512B */

    /* FULLMAC */
    FULLMAC_IMG2 (rwx) :                   ORIGIN = 0x20003020, LENGTH = 0x2006F000 - 0x20003020    /* FULLMAC IMAGE2: 432k, 0x2006F000 ~ 0x20070000 for MANIFEST */
    FULLMAC_TCM_RAM (rwx) :                ORIGIN = 0x000F0000, LENGTH = 0x00100000 - 0x000F0000    /* FULLMAC TCM RAM: TZ+NS=64k */
    FULLMAC_PG_LOADER_RAM (rwx) :          ORIGIN = 0x30000040, LENGTH = 0x300004C0 - 0x30000040    /* FULLMAC PG Loader RAM: AP_ROMBSS_RAM_S, shall after _fih_cfi_ctr */

    /* KM4TZ Secure RAM, BIT(28)=1 */
    KM4TZ_BOOTLOADER_RAM_S (rwx) :         ORIGIN = KM4TZ_IMG1_RAM_SATRT + SECURE_ADDR_OFFSET + IMAGE_HEADER_LEN, LENGTH = KM4TZ_IMG1_SIZE  - IMAGE_HEADER_LEN        /* KM4TZ BOOT Loader RAM: 4K or 32k */
#ifndef CONFIG_FULLMAC_PG_LOADER
    KM4TZ_FLOADER_RAM_S (rwx) :            ORIGIN = 0x30070020, LENGTH = 0x3007F000 - 0x30070020    /* Floader / FULLMAC bootloader: 60k, 0x3007F000 ~ 0x30080000 for MANIFEST */
#else
    KM4TZ_FLOADER_RAM_S (rwx) :            ORIGIN = ORIGIN(FULLMAC_PG_LOADER_RAM), LENGTH = LENGTH(FULLMAC_PG_LOADER_RAM)
#endif

    /* IMG2 RAM */
#if defined (CONFIG_WIFI_HOST_CONTROL)
    KM4TZ_BD_RAM (rwx) :                   ORIGIN = ORIGIN(FULLMAC_IMG2), LENGTH = LENGTH(FULLMAC_IMG2)
#else
    KM4TZ_BD_RAM_TZ_NSC (rwx) :            ORIGIN = KM4TZ_RAM_TZ_NSC_START + IMAGE_HEADER_LEN,     LENGTH = RAM_TZ_NSC_SIZE - IMAGE_HEADER_LEN                                   /* KM4TZ BD RAM NSC: 4k */
    KM4TZ_BD_RAM_TZ_ENTRY (rwx) :          ORIGIN = KM4TZ_RAM_TZ_ENTRY_START,  LENGTH = RAM_TZ_ENTRY_SIZE                                                                        /* KM4TZ BD RAM ENTRY: 16k */
    KM4TZ_BD_RAM_TZ_S (rwx) :              ORIGIN = KM4TZ_RAM_TZ_SECURE_START + SECURE_ADDR_OFFSET  + IMAGE_HEADER_LEN, LENGTH = RAM_KM4TZ_IMG3_SIZE - IMAGE_HEADER_LEN          /* KM4TZ BD RAM S: 44k */
    KM4TZ_BD_RAM (rwx) :                   ORIGIN = KM4TZ_IMG2_RAM_START + IMAGE_HEADER_LEN,   LENGTH = KM4NS_IMG2_RAM_START - KM4TZ_IMG2_RAM_START - IMAGE_HEADER_LEN           /* KM4TZ BD RAM: TZ On, 212KB; Tz off, 288K */
    KM4NS_BD_RAM (rwx) :                   ORIGIN = KM4NS_IMG2_RAM_START + IMAGE_HEADER_LEN,   LENGTH = RAM_KM4NS_IMG2_SIZE - IMAGE_HEADER_LEN                                   /* KM4NS BD RAM: 192KB */
#endif /* CONFIG_TRUSTZONE_EN */


    KM4TZ_BD_PSRAM_TZ_NSC (rwx) :          ORIGIN = KM4TZ_PSRAM_TZ_NSC_START + IMAGE_HEADER_LEN, LENGTH = PSRAM_TZ_NSC_SIZE - IMAGE_HEADER_LEN                                   /* KM4TZ BD PSRAM NSC: 4k */
    KM4TZ_BD_PSRAM_TZ_ENTRY (rwx) :        ORIGIN = KM4TZ_PSRAM_TZ_ENTRY_START, LENGTH = PSRAM_TZ_ENTRY_SIZE                                                                     /* KM4TZ BD PSRAM ENTRY: 16k */
    KM4TZ_BD_PSRAM_TZ_S (rwx) :            ORIGIN = KM4TZ_PSRAM_TZ_SECURE_START + SECURE_ADDR_OFFSET  + IMAGE_HEADER_LEN, LENGTH = PSRAM_KM4TZ_IMG3_SIZE - IMAGE_HEADER_LEN      /* KM4TZ BD PSRAM S:  128k */
    /* KM4TZ Non-secure PSRAM */
    KM4TZ_BD_PSRAM (rwx) :                 ORIGIN = KM4TZ_IMG2_PSRAM_START + IMAGE_HEADER_LEN, LENGTH = PSRAM_KM4TZ_IMG2_SIZE - IMAGE_HEADER_LEN                                 /* KM4TZ BD PSRAM : Tz off,3220K; TZ on, 3072K */
    KM4NS_BD_PSRAM (rwx) :                 ORIGIN = KM4NS_IMG2_PSRAM_START + IMAGE_HEADER_LEN, LENGTH = PSRAM_KM4NS_IMG2_SIZE - IMAGE_HEADER_LEN                                 /* KM4NS BD PSRAM: 876K */

    KM4TZ_PSRAM_HEAP_EXT (rwx) :           ORIGIN = 0x6FFFFFFF, LENGTH = 0x6FFFFFFF - 0x6FFFFFFF    /* KM4TZ PSRAM HEAP EXT */
    KM4NS_PSRAM_HEAP_EXT (rwx) :           ORIGIN = 0x6FFFFFFF, LENGTH = 0x6FFFFFFF - 0x6FFFFFFF    /* KM4NS PSRAM HEAP EXT */

    /* Retention RAM */
    RETENTION_RAM (rwx) :                  ORIGIN = 0x4080AA00, LENGTH = 0x4080AC00 - 0x4080AA00    /* KM4NS Retention SRAM: 512B */

    /* Flash */
    KM4TZ_BOOT_XIP (rx) :                  ORIGIN = 0x10400020, LENGTH = 0x10C00000 - 0x10400020    /* KM4TZ IMG1 Virtual address: 8M */
    KM4TZ_IMG3_XIP (rx) :                  ORIGIN = 0x10C00020, LENGTH = 0x11000000 - 0x10C00020    /* KM4TZ IMG3 Virtual address: 4M */
    KM4TZ_IMG2_XIP (rx) :                  ORIGIN = 0x02000020, LENGTH = 0x04000000 - 0x02000020    /* KM4TZ IMG2 Virtual address: 32M */
    KM4NS_IMG2_XIP (rx) :                  ORIGIN = 0x04000020, LENGTH = 0x06000000 - 0x04000020    /* KM4NS IMG2 Virtual address: 32M */
    BTRACE (rx) :                          ORIGIN = 0xC8800000, LENGTH = 0xC9000000 - 0xC8800000    /* BT LOG Virtual address: 8192k */
    CTRACE (rx) :                          ORIGIN = 0xCA000000, LENGTH = 0xCA7FFFFF - 0xCA000000    /* COEX LOG Virtual address: 8192k */
}

__ipc_memory_start__ = ORIGIN(KM4NS_IPC_RAM);
__km4ns_flash_text_start__ = ORIGIN(KM4NS_IMG2_XIP);
__km4ns_flash_text_end__ = ORIGIN(KM4NS_IMG2_XIP) + LENGTH(KM4NS_IMG2_XIP);
__km4tz_flash_text_start__ = ORIGIN(KM4TZ_IMG2_XIP);
__km4tz_flash_text_end__ = ORIGIN(KM4TZ_IMG2_XIP) + LENGTH(KM4TZ_IMG2_XIP);
__km4tz_boot_text_start__ = ORIGIN(KM4TZ_BOOT_XIP);
__km4tz_boot_text_end__ = ORIGIN(KM4TZ_BOOT_XIP) + LENGTH(KM4TZ_BOOT_XIP);
__km4tz_img3_text_start__ = ORIGIN(KM4TZ_IMG3_XIP);
__km4tz_img3_text_end__ = ORIGIN(KM4TZ_IMG3_XIP) + LENGTH(KM4TZ_IMG3_XIP);


#if defined (CONFIG_TRUSTZONE_EN)
__km4tz_tz_nsc_start__ = ORIGIN(KM4TZ_BD_RAM_TZ_NSC) - 0x20;
__km4tz_tz_entry_start__ = ORIGIN(KM4TZ_BD_RAM_TZ_ENTRY);
__km4tz_tz_entry_end__ = ORIGIN(KM4TZ_BD_RAM_TZ_ENTRY) + LENGTH(KM4TZ_BD_RAM_TZ_ENTRY);

__km4tz_psram_tz_nsc_start__ = ORIGIN(KM4TZ_BD_PSRAM_TZ_NSC) - 0x20;
__km4tz_psram_tz_entry_start__ = ORIGIN(KM4TZ_BD_PSRAM_TZ_ENTRY) ;
__km4tz_psram_tz_entry_end__ = ORIGIN(KM4TZ_BD_PSRAM_TZ_ENTRY) + LENGTH(KM4TZ_BD_PSRAM_TZ_ENTRY);
#endif
/*SRAM Layout boundary*/
__km4tz_bd_ram_start__ = ORIGIN(KM4TZ_BD_RAM) - 0x20;

/*PSRAM Layout boundary*/
__km4tz_bd_psram_start__ = ORIGIN(KM4TZ_BD_PSRAM) - 0x20;
__non_secure_psram_end__ = PSRAM_END;


SECTIONS
{
	/* xip part */
	.xip_image2.text :
	{
		__flash_text_start__ = .;
		FLASH_INPUT_TEXT_SECTION
		*(.xip.only.text*)
		*(.xip.only.rodata*)
		. = ALIGN(32);
		__flash_text_end__ = .;
	} > KM4TZ_IMG2_XIP

	/* sram part */
	.sram_image2.text.data ORIGIN(KM4TZ_BD_RAM):
	{
		__sram_image2_start__ = .;
		/* fixed location. do not move to other location */
		__image2_entry_func__ = .;
		KEEP(*(SORT(.image2.entry.data*)))
		__image2_validate_code__ = .;
		KEEP(*(.image2.validate.rodata*))
		. = ALIGN(32);

		SRAM_INPUT_TEXT_SECTION
		*(.sram.only.text*)
		*(.sram.only.data*)
		__sram_image2_end__ = .;
		. = ALIGN(32);
	} > KM4TZ_BD_RAM

	/* image2: normal image start */
	.psram_image2.text.data ORIGIN(KM4TZ_BD_PSRAM):
	{
		__psram_image2_start__ = .;

		PSRAM_INPUT_TEXT_SECTION

		__psram_image2_end__ = .;
	} > KM4TZ_BD_PSRAM

	/* text section */
	OUT_TEXT_SECTION :
	{
		__image2_backtrace_start__ = .;

		*(.img2_custom_signature*)
		*(.image2.net.ram.text*)
		*(.timesensitive.text*)

		*(.text*)
		*(.rodata*)

		. = ALIGN (4);
		__cmd_table_start__ = .;
		KEEP(*(.cmd.table.data*))
		__cmd_table_end__ = .;
		. = ALIGN (4);
		__ipc_table_start__ = .;
		KEEP(*(*.ipc.table.data*))
		__ipc_table_end__ = .;
		. = ALIGN (4);
		__git_ver_table_start__ = .;
		KEEP(*(.git.ver.data))
		__git_ver_table_end__ = .;

		/* Add This for C++ support */
		. = ALIGN(4);
		__c_plus_plus_support_start__ = .;
#if !defined(CONFIG_WIFI_HOST_CONTROL)
		__preinit_array_start = .;
		KEEP(*(.preinit_array))
		__preinit_array_end = .;
		. = ALIGN(4);
		__init_array_start = .;
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		__init_array_end = .;
		. = ALIGN(4);
		__fini_array_start = .;
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		__fini_array_end = .;
#endif
		__c_plus_plus_support_end__ = .;
		/*-----------------*/
		__image2_backtrace_end__ = .;

		. = ALIGN (32);
	} > IMG2_TEXT_LMA_REGION

	/* C++ .ARM.extab/.ARM.exidx address shall same as *(.text*) */
	.ARM.extab  :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)

	} > ARM_EXCEPTION_SECTION

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > ARM_EXCEPTION_SECTION

	OUT_DATA_SECTION :
	{
		*(.timesensitive.data*)
		*(.data*)

		/*cache line size aligned in order to avoid MMU remap err*/
		. = ALIGN(32);
	} > IMG2_DATA_LMA_REGION

	/*-----------------*/
	.ram_image2.bss (NOLOAD):
	{
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		__bss_end__ = .;
	} > IMG2_DATA_LMA_REGION

	.ram_image2.bd.data (NOLOAD):
	{
		. = ALIGN (32);
		__ram_bd_start__ = .;
		*(.bdsram.data*)
		. = ALIGN (32);
		__ram_bd_end__ = .;
	} > IMG2_DATA_LMA_REGION

	.ram_image2.nocache.data (NOLOAD):
	{
		. = ALIGN (32);
		__ram_nocache_start__ = .;
		*(.nocache.data*)
		. = ALIGN (32);
		__ram_nocache_end__ = .;
	} > IMG2_DATA_LMA_REGION

	/* put in the end of sram part */
	.bdram_heap.start (NOLOAD):
	{
		__bdram_heap_buffer_start__ = .;
	} > KM4TZ_BD_RAM

	/* put in the end of psram part */
	.psram_heap.start (NOLOAD):
	{
		__psram_heap_buffer_start__ = .;
	} > KM4TZ_BD_PSRAM

	.bluetooth_trace.text :
	{
		__btrace_start__ = .;
		*(.BTTRACE)
		__btrace_end__ = .;
	} > BTRACE

	.coex_trace.text :
	{
		__ctrace_start__ = .;
		*(.COEXTRACE)
		__ctrace_end__ = .;
	} > CTRACE
}

ASSERT(__image2_entry_func__ == ORIGIN(KM4TZ_BD_RAM), "__image2_entry_func__ is not KM4TZ_BD_RAM.")
ASSERT(__sram_image2_start__ == ORIGIN(KM4TZ_BD_RAM), "__sram_image2_start__ is changed.")
ASSERT(__flash_text_start__ == ORIGIN(KM4TZ_IMG2_XIP), "__flash_text_start__ is changed.")
ASSERT(__psram_image2_start__ == ORIGIN(KM4TZ_BD_PSRAM), "__psram_image2_start__ is changed.")

#if defined(CONFIG_DATA_HEAP_SRAM)
	#if defined(CONFIG_WIFI_HOST_CONTROL)
		//ASSERT(__ipc_table_start__ == __ipc_table_end__, "fullmac does not require ipc table!")
		ASSERT(__c_plus_plus_support_start__ == __c_plus_plus_support_end__, "fullmac does not support C++!")

		__bdram_heap_buffer_size__ = SRAM_END - __bdram_heap_buffer_start__;
		__tcm_heap_buffer_start__ = ORIGIN(FULLMAC_TCM_RAM);
		__tcm_heap_buffer_size__ = LENGTH(FULLMAC_TCM_RAM);
		// remember heap: [__retrieve_ap_buffer_start__, __retrieve_ap_buffer_size__)
	#else
		__bdram_heap_buffer_size__ = ORIGIN(KM4TZ_BD_RAM) + LENGTH(KM4TZ_BD_RAM) - __bdram_heap_buffer_start__;
	#endif
#else /* CONFIG_DATA_HEAP_PSRAM */
	__bdram_heap_buffer_size__ = 0;
#endif

#if defined(CONFIG_PSRAM_ALL_FOR_AP_HEAP) || defined(CONFIG_DISABLE_PSRAM)
__psram_heap_buffer_start__ = .;
__psram_heap_buffer_size__ = 0;

__psram_heap_extend_start__ = .;
__psram_heap_extend_size__ = 0;

#else
__psram_heap_buffer_size__ = ORIGIN(KM4TZ_BD_PSRAM) + LENGTH(KM4TZ_BD_PSRAM) - __psram_heap_buffer_start__;

__psram_heap_extend_start__ = ORIGIN(KM4TZ_PSRAM_HEAP_EXT);
__psram_heap_extend_size__ = LENGTH(KM4TZ_PSRAM_HEAP_EXT);
#endif
