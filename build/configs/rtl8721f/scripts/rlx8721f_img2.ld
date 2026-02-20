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

OUTPUT_ARCH(arm)
ENTRY(app_start)
 
MEMORY
{
    EMPTY_SECTION (rwx) : ORIGIN = 0x00000000, LENGTH = 0x00000000 - 0x00000000
    KM4TZ_IROM (rx) : ORIGIN = 0x00000000, LENGTH = 0x0001D000 - 0x00000000
    KM4TZ_DROM (rx) : ORIGIN = 0x0001D000, LENGTH = 0x0001F000 - 0x0001D000
    KM4TZ_ONLY_ROM (rx) : ORIGIN = 0x0001F000, LENGTH = 0x00020000 - 0x0001F000
    KM4NS_SYS_ROM (rx) : ORIGIN = 0x00100000, LENGTH = 0x00101000 - 0x00100000
    SHARE_IROM (rx) : ORIGIN = 0x00101000, LENGTH = 0x0010C000 - 0x00101000
    SHARE_DROM (rx) : ORIGIN = 0x0010C000, LENGTH = 0x0010F000 - 0x0010C000
    OS_ROM (rx) : ORIGIN = 0x0010F000, LENGTH = 0x00114000 - 0x0010F000
    WIFI_ROM (rx) : ORIGIN = 0x00114000, LENGTH = 0x00147800 - 0x00114000
    DELAY_ROM (rx) : ORIGIN = 0x00147800, LENGTH = 0x00148000 - 0x00147800
    KM4TZ_ROMBSS_RAM_S (rw) : ORIGIN = 0x30000000, LENGTH = 0x30000600 - 0x30000000
    KM4TZ_MSP_RAM_S (rw) : ORIGIN = 0x30000600, LENGTH = 0x30001000 - 0x30000600
    KM4TZ_ROMBSS_RAM_NS (rw) : ORIGIN = 0x20001000, LENGTH = 0x20002400 - 0x20001000
    KM4TZ_MSP_RAM_NS (rw) : ORIGIN = 0x20002400, LENGTH = 0x20003000 - 0x20002400
    KM4NS_ROMBSS_MMU (rw) : ORIGIN = 0x20001000, LENGTH = 0x20002400 - 0x20001000
    KM4NS_ROMBSS_RAM (rw) : ORIGIN = 0x20003000, LENGTH = 0x20004400 - 0x20003000
    KM4NS_MSP_RAM (rw) : ORIGIN = 0x20004600, LENGTH = 0x20005000 - 0x20004600
    KM4NS_IPC_RAM (rwx) : ORIGIN = 0x20004400, LENGTH = 0x20004600 - 0x20004400
    FULLMAC_IMG2 (rwx) : ORIGIN = 0x20003020, LENGTH = 0x2006F000 - 0x20003020
    FULLMAC_TCM_RAM (rwx) : ORIGIN = 0x000F0000, LENGTH = 0x00100000 - 0x000F0000
    FULLMAC_PG_LOADER_RAM (rwx) : ORIGIN = 0x30000040, LENGTH = 0x300004C0 - 0x30000040
    KM4TZ_BOOTLOADER_RAM_S (rwx) : ORIGIN = ((0x20000000) + ((20) * 1024)) + (0x10000000) + (0x20), LENGTH = ((4) * 1024) - (0x20)
    KM4TZ_FLOADER_RAM_S (rwx) : ORIGIN = 0x30070020, LENGTH = 0x3007F000 - 0x30070020
    KM4TZ_BD_RAM_TZ_NSC (rwx) : ORIGIN = (((0x20000000) + ((20) * 1024)) + ((4) * 1024) ) + (0x20), LENGTH = (((4)) * 1024) - (0x20)
    KM4TZ_BD_RAM_TZ_ENTRY (rwx) : ORIGIN = ((((0x20000000) + ((20) * 1024)) + ((4) * 1024) ) + (((4)) * 1024) ), LENGTH = (((16)) * 1024)
    KM4TZ_BD_RAM_TZ_S (rwx) : ORIGIN = (((((0x20000000) + ((20) * 1024)) + ((4) * 1024) ) + (((4)) * 1024) ) + (((16)) * 1024) ) + (0x10000000) + (0x20), LENGTH = (((44)) * 1024) - (0x20)
    KM4TZ_BD_RAM (rwx) : ORIGIN = ((((((0x20000000) + ((20) * 1024)) + ((4) * 1024) ) + (((4)) * 1024) ) + (((16)) * 1024) ) + (((44)) * 1024)) + (0x20), LENGTH = ((0x20080000) - ((96) * 1024)) - ((((((0x20000000) + ((20) * 1024)) + ((4) * 1024) ) + (((4)) * 1024) ) + (((16)) * 1024) ) + (((44)) * 1024)) - (0x20)
    KM4NS_BD_RAM (rwx) : ORIGIN = ((0x20080000) - ((96) * 1024)) + (0x20), LENGTH = ((96) * 1024) - (0x20)
    KM4TZ_BD_PSRAM_TZ_NSC (rwx) : ORIGIN = ((0x60000000)) + (0x20), LENGTH = ((0) * 1024) - (0x20)
    KM4TZ_BD_PSRAM_TZ_ENTRY (rwx) : ORIGIN = (((0x60000000)) + ((0) * 1024) ), LENGTH = ((0) * 1024)
    KM4TZ_BD_PSRAM_TZ_S (rwx) : ORIGIN = ((((0x60000000)) + ((0) * 1024) ) + ((0) * 1024)) + (0x10000000) + (0x20), LENGTH = ((0) * 1024) - (0x20)
    KM4TZ_BD_PSRAM (rwx) : ORIGIN = (((((0x60000000)) + ((0) * 1024) ) + ((0) * 1024)) + ((0) * 1024)) + (0x20), LENGTH = (((3220) * 1024) - (((0) * 1024) + ((0) * 1024) + ((0) * 1024))) - (0x20)
    KM4NS_BD_PSRAM (rwx) : ORIGIN = ((((((0x60000000)) + ((0) * 1024) ) + ((0) * 1024)) + ((0) * 1024)) + (((3220) * 1024) - (((0) * 1024) + ((0) * 1024) + ((0) * 1024)))) + (0x20), LENGTH = ((0x60400000) - ((((((0x60000000)) + ((0) * 1024) ) + ((0) * 1024)) + ((0) * 1024)) + (((3220) * 1024) - (((0) * 1024) + ((0) * 1024) + ((0) * 1024))))) - (0x20)
    KM4TZ_PSRAM_HEAP_EXT (rwx) : ORIGIN = 0x6FFFFFFF, LENGTH = 0x6FFFFFFF - 0x6FFFFFFF
    KM4NS_PSRAM_HEAP_EXT (rwx) : ORIGIN = 0x6FFFFFFF, LENGTH = 0x6FFFFFFF - 0x6FFFFFFF
    RETENTION_RAM (rwx) : ORIGIN = 0x4080AA00, LENGTH = 0x4080AC00 - 0x4080AA00
    KM4TZ_BOOT_XIP (rx) : ORIGIN = 0x10400020, LENGTH = 0x10C00000 - 0x10400020
    KM4TZ_IMG3_XIP (rx) : ORIGIN = 0x10C00020, LENGTH = 0x11000000 - 0x10C00020
    KM4NS_IMG2_XIP (rx) : ORIGIN = 0x02000020, LENGTH = 0x04000000 - 0x02000020
    KM4TZ_IMG2_XIP (rx) : ORIGIN = 0x04000020, LENGTH = 0x06000000 - 0x04000020
    BTRACE (rx) : ORIGIN = 0xC8800000, LENGTH = 0xC9000000 - 0xC8800000
    CTRACE (rx) : ORIGIN = 0xCA000000, LENGTH = 0xCA7FFFFF - 0xCA000000
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
__km4tz_tz_nsc_start__ = ORIGIN(KM4TZ_BD_RAM_TZ_NSC) - 0x20;
__km4tz_tz_entry_start__ = ORIGIN(KM4TZ_BD_RAM_TZ_ENTRY);
__km4tz_tz_entry_end__ = ORIGIN(KM4TZ_BD_RAM_TZ_ENTRY) + LENGTH(KM4TZ_BD_RAM_TZ_ENTRY);
__km4tz_psram_tz_nsc_start__ = ORIGIN(KM4TZ_BD_PSRAM_TZ_NSC) - 0x20;
__km4tz_psram_tz_entry_start__ = ORIGIN(KM4TZ_BD_PSRAM_TZ_ENTRY) ;
__km4tz_psram_tz_entry_end__ = ORIGIN(KM4TZ_BD_PSRAM_TZ_ENTRY) + LENGTH(KM4TZ_BD_PSRAM_TZ_ENTRY);
__km4tz_bd_ram_start__ = ORIGIN(KM4TZ_BD_RAM) - 0x20;
__km4tz_bd_psram_start__ = ORIGIN(KM4TZ_BD_PSRAM) - 0x20;
__non_secure_psram_end__ = (0x60400000);
 
SECTIONS
{
	/* xip part */
	.xip_image2.text :
	{
		__flash_text_start__ = .;
		*(.non.dram.text*)
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

		*(.sramdram.only.text*)
        *(.wlan.critical.text*)
		*(.sram.only.text*)
		*(.sram.only.data*)
		__sram_image2_end__ = .;
		. = ALIGN(32);
	} > KM4TZ_BD_RAM
 
	/* image2: normal image start */
	.psram_image2.text.data ORIGIN(KM4TZ_BD_PSRAM):
	{
		__psram_image2_start__ = .;

		__psram_image2_end__ = .;
	} > KM4TZ_BD_PSRAM
 
    /* text section */
    .xip_image2.text :
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
		__c_plus_plus_support_end__ = .;
		/*-----------------*/
		__image2_backtrace_end__ = .;

		. = ALIGN (32);
	} > KM4TZ_IMG2_XIP
 
	/* C++ .ARM.extab/.ARM.exidx address shall same as *(.text*) */
	.ARM.extab  :
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)

	} > KM4TZ_BD_RAM

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} > KM4TZ_BD_RAM
 
	.sram_image2.text.data :
	{
		*(.timesensitive.data*)
		*(.data*)

		/*cache line size aligned in order to avoid MMU remap err*/
		. = ALIGN(32);
	} > KM4TZ_BD_RAM

    /*-----------------*/
	.ram_image2.bss (NOLOAD):
	{
        _sbss = ABSOLUTE(.);
		__bss_start__ = .;
		*(.bss*)
		*(COMMON)
		__bss_end__ = .;
	} > KM4TZ_BD_RAM

	.ram_image2.bd.data (NOLOAD):
	{
        _sdata = ABSOLUTE(.);
		. = ALIGN (32);
		__ram_bd_start__ = .;
		*(.bdsram.data*)
        _edata = ABSOLUTE(.);
		. = ALIGN (32);
		__ram_bd_end__ = .;
	} > KM4TZ_BD_RAM
	.ram_image2.nocache.data (NOLOAD):
	{
		. = ALIGN (32);
		__ram_nocache_start__ = .;
		*(.nocache.data*)
		. = ALIGN (32);
		__ram_nocache_end__ = .;
	} > KM4TZ_BD_RAM

	/* put in the end of sram part */
	.bdram_heap.start (NOLOAD):
	{
		__bdram_heap_buffer_start__ = .;
        _ebss = ABSOLUTE(.);
		/* Heap start address in internal RAM */
		_sint_heap = ABSOLUTE(.);
	} > KM4TZ_BD_RAM
 
	/* put in the end of psram part */
	.psram_heap.start (NOLOAD):
	{
		__psram_heap_buffer_start__ = .;
        _sext_heap = ABSOLUTE(.);
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
 
    .stack_heap_limits_psram (NOLOAD):
    {
        __PsramStackLimit = ORIGIN(KM4TZ_BD_PSRAM) + LENGTH(KM4TZ_BD_PSRAM);
    } > KM4TZ_BD_PSRAM
	_sidle_stack = ABSOLUTE(ORIGIN(KM4TZ_MSP_RAM_NS) + LENGTH(KM4TZ_MSP_RAM_NS)) - CONFIG_IDLETHREAD_STACKSIZE;
	__StackLimit = ABSOLUTE(ORIGIN(KM4TZ_BD_RAM) + LENGTH(KM4TZ_BD_RAM));
}

/* Assertions */
ASSERT(__image2_entry_func__ == ORIGIN(KM4TZ_BD_RAM), "__image2_entry_func__ is not KM4TZ_BD_RAM.")
ASSERT(__sram_image2_start__ == ORIGIN(KM4TZ_BD_RAM), "__sram_image2_start__ is changed.")
ASSERT(__flash_text_start__ == ORIGIN(KM4TZ_IMG2_XIP), "__flash_text_start__ is changed.")
ASSERT(__psram_image2_start__ == ORIGIN(KM4TZ_BD_PSRAM), "__psram_image2_start__ is changed.")
ASSERT(__StackLimit >= _sint_heap, "region data overflowed with stack and heap")
/*__psram_heap_buffer_size__ = ORIGIN(KM4TZ_BD_PSRAM) + LENGTH(KM4TZ_BD_PSRAM) - __psram_heap_buffer_start__;*/

SECTIONS
{
    PROVIDE(__retrieve_ap_buffer_size__ = 0xf40);
    PROVIDE(INT_HardFault = 0x1f001);
    PROVIDE(INT_MemManage = 0x1f00b);
    PROVIDE(INT_BusFault = 0x1f015);
    PROVIDE(INT_UsageFault = 0x1f01f);
    PROVIDE(INT_SecureFault = 0x1f029);
    PROVIDE(INT_NMI = 0x1f033);
    PROVIDE(INT_SVCall = 0x1f049);
    PROVIDE(INT_PendSV = 0x1f069);
    PROVIDE(INT_SysTick = 0x1f089);
    PROVIDE(INT_HardFault_C = 0x1f245);
    PROVIDE(FaultHandler = 0x1f2b1);
    PROVIDE(PreFaultPatch_register = 0x1f2c1);
    PROVIDE(PostFaultPatch_register = 0x1f2cd);
    PROVIDE(SVCall_irqfunc_set = 0x1f2d9);
    PROVIDE(SVCall_irqfunc_get = 0x1f2e5);
    PROVIDE(PendSV_irqfunc_set = 0x1f2f1);
    PROVIDE(PendSV_irqfunc_get = 0x1f2fd);
    PROVIDE(Systick_irqfunc_set = 0x1f309);
    PROVIDE(Systick_irqfunc_get = 0x1f315);
    PROVIDE(RSIP_WAIT_IDLE = 0x1f321);
    PROVIDE(RSIP_IV_Set = 0x1f34d);
    PROVIDE(RSIP_OTF_init = 0x1f365);
    PROVIDE(RSIP_OTF_Set_GCM_Tag = 0x1f381);
    PROVIDE(RSIP_OTF_Cmd = 0x1f3b5);
    PROVIDE(RSIP_OTF_Enable = 0x1f3cd);
    PROVIDE(RSIP_MMU_Config = 0x1f449);
    PROVIDE(RSIP_MMU_Cmd = 0x1f47f);
    PROVIDE(RSIP_MMU_Cache_Clean = 0x1f4a1);
    PROVIDE(otf_hw_crypto_set_iv_sw = 0x1f4f1);
    PROVIDE(otf_hw_crypto = 0x1f529);
    PROVIDE(otf_hw_crypto_gcm_set_iv_sw = 0x1f675);
    PROVIDE(otf_hw_crypto_gcm = 0x1f699);
    PROVIDE(irq_enable = 0x101001);
    PROVIDE(irq_disable = 0x10101d);
    PROVIDE(irq_set_priority = 0x101041);
    PROVIDE(irq_get_priority = 0x101091);
    PROVIDE(irq_set_pending = 0x1010b5);
    PROVIDE(irq_get_pending = 0x1010d1);
    PROVIDE(irq_clear_pending = 0x1010f5);
    PROVIDE(irq_register = 0x101111);
    PROVIDE(irq_unregister = 0x101139);
    PROVIDE(_char2num = 0x101155);
    PROVIDE(_2char2dec = 0x101181);
    PROVIDE(_2char2hex = 0x101201);
    PROVIDE(_memchr = 0x101271);
    PROVIDE(_memcmp = 0x1012f9);
    PROVIDE(_memcpy = 0x101365);
    PROVIDE(_memmove = 0x10140d);
    PROVIDE(_memset = 0x1014f9);
    PROVIDE(_vsscanf_minimal = 0x101659);
    PROVIDE(_sscanf_ss = 0x101af1);
    PROVIDE(_strcmp = 0x101b0d);
    PROVIDE(_strcpy = 0x101b75);
    PROVIDE(_stricmp = 0x101bc1);
    PROVIDE(_strlen = 0x101c1d);
    PROVIDE(_strncpy = 0x101c69);
    PROVIDE(_strtol = 0x101cf5);
    PROVIDE(_strtoul = 0x101e0d);
    PROVIDE(_strupr = 0x101f4d);
    PROVIDE(__wrap_memset = 0x101f69);
    PROVIDE(__wrap_memcpy = 0x101f71);
    PROVIDE(__wrap_memcmp = 0x101f79);
    PROVIDE(__wrap_memchr = 0x101f81);
    PROVIDE(__wrap_memmove = 0x101f89);
    PROVIDE(__wrap_strlen = 0x101f91);
    PROVIDE(__wrap_strcmp = 0x101f99);
    PROVIDE(__wrap_stricmp = 0x101fa1);
    PROVIDE(__wrap_strupr = 0x101fa9);
    PROVIDE(__wrap_strncpy = 0x101fb1);
    PROVIDE(__wrap_strtoul = 0x101fb9);
    PROVIDE(__wrap_strcpy = 0x101fc1);
    PROVIDE(__wrap_strtol = 0x101fc9);
    PROVIDE(__wrap_printf = 0x101fd1);
    PROVIDE(__wrap_sprintf = 0x101fed);
    PROVIDE(AES_Get_AES_Addr = 0x102009);
    PROVIDE(AES_get_km_aes_addr = 0x102029);
    PROVIDE(AES_Get_DMA_Addr = 0x102049);
    PROVIDE(AES_Disable_SCA_Mask = 0x102069);
    PROVIDE(AES_Enable_SCA_Mask = 0x10207d);
    PROVIDE(AES_Set_SCA_Mask = 0x102091);
    PROVIDE(AES_DMA_Init = 0x1020bd);
    PROVIDE(AES_StructInit = 0x1020e9);
    PROVIDE(AES_Init = 0x1020f9);
    PROVIDE(AES_Cmd = 0x102161);
    PROVIDE(AES_GetIv = 0x102181);
    PROVIDE(AES_SetIv = 0x1021a1);
    PROVIDE(AES_GetTag = 0x1021c1);
    PROVIDE(AES_InitTag = 0x102209);
    PROVIDE(AES_ProcessDma = 0x102229);
    PROVIDE(AES_GCM_Set_Iv = 0x102589);
    PROVIDE(AES_lock_mutex = 0x102681);
    PROVIDE(AES_unlock_mutex = 0x1026d5);
    PROVIDE(AES_set_swap_setting = 0x1026e9);
    PROVIDE(AES_get_swap_setting = 0x102715);
    PROVIDE(AES_load_key = 0x102739);
    PROVIDE(crypto_aes_set_sw_key = 0x102839);
    PROVIDE(crypto_aes_share_secure_key = 0x1029cd);
    PROVIDE(crypto_aes_slave_start = 0x102a4d);
    PROVIDE(crypto_aes_ecb_slave = 0x102b91);
    PROVIDE(crypto_aes_ctr_slave = 0x102bb1);
    PROVIDE(crypto_aes_start = 0x102bd3);
    PROVIDE(crypto_aes_ecb = 0x102cb5);
    PROVIDE(crypto_aes_cbc = 0x102ce1);
    PROVIDE(crypto_aes_cfb = 0x102d0f);
    PROVIDE(crypto_aes_ofb = 0x102d31);
    PROVIDE(crypto_aes_ctr = 0x102d53);
    PROVIDE(crypto_aes_xts = 0x102d75);
    PROVIDE(crypto_cmac = 0x102db5);
    PROVIDE(crypto_gcm_encrypt_and_tag = 0x102e39);
    PROVIDE(crypto_gcm_auth_decrypt = 0x103019);
    PROVIDE(SHA2_Get_SHA_Addr = 0x103135);
    PROVIDE(SHA2_get_km_hmac_addr = 0x103155);
    PROVIDE(SHA2_Get_DMA_Addr = 0x103175);
    PROVIDE(SHA2_Restore = 0x103195);
    PROVIDE(SHA2_Save = 0x103231);
    PROVIDE(SHA2_Get_Final_Digest = 0x103281);
    PROVIDE(SHA_StructInit = 0x103399);
    PROVIDE(SHA2_Init = 0x1033a7);
    PROVIDE(SHA2_DMA_Init = 0x103455);
    PROVIDE(SHA2_SlaveEnd = 0x10349d);
    PROVIDE(SHA2_DmaStart = 0x1034d9);
    PROVIDE(SHA2_ProcessDma = 0x103769);
    PROVIDE(SHA2_lock_mutex = 0x1037dd);
    PROVIDE(SHA2_unlock_mutex = 0x103831);
    PROVIDE(SHA2_set_swap_setting = 0x103845);
    PROVIDE(SHA2_get_swap_setting = 0x103879);
    PROVIDE(SHA2_hmac_load_key = 0x10389d);
    PROVIDE(crypto_sha2_init = 0x10394d);
    PROVIDE(crypto_sha2_update = 0x1039d1);
    PROVIDE(crypto_sha2_final = 0x103b71);
    PROVIDE(crypto_hmac_sha2_set_sw_key = 0x103c39);
    PROVIDE(crypto_hmac_sha2_share_secure_key = 0x103d4d);
    PROVIDE(crypto_hmac_sha2_init = 0x103dc1);
    PROVIDE(crypto_hmac_sha2_update = 0x103e81);
    PROVIDE(crypto_hmac_sha2_final = 0x103e8d);
    PROVIDE(CRYPTO_Init = 0x103e99);
    PROVIDE(CRYPTO_Addr_Not_Alined_Warning = 0x103ee1);
    PROVIDE(CRYPTO_Length_Not_Alined_Warning = 0x103f11);
    PROVIDE(pke_ecp_group_init_in_rom = 0x103f41);
    PROVIDE(pke_ecp_set_mmem_curve_parameter = 0x1042f5);
    PROVIDE(pke_ecp_set_tmem_curve_parameter = 0x10437d);
    PROVIDE(pke_ecp_point_init = 0x10440d);
    PROVIDE(pke_ecp_point_init_base_point = 0x104425);
    PROVIDE(pke_ecp_mul = 0x104451);
    PROVIDE(pke_ecdsa_lalu_hash_process = 0x1045cd);
    PROVIDE(pke_ecdsa_set_key = 0x104635);
    PROVIDE(pke_ecdsa_genkey = 0x104695);
    PROVIDE(pke_ecdsa_write_signature = 0x10476d);
    PROVIDE(pke_ecdsa_read_signature = 0x1048a5);
    PROVIDE(pke_eddsa_genkey = 0x1049f5);
    PROVIDE(pke_eddsa_write_signature_R = 0x104ab1);
    PROVIDE(pke_eddsa_write_signature_S = 0x104b75);
    PROVIDE(pke_eddsa_read_signature = 0x104c3d);
    PROVIDE(pke_eddsa_ed25519_get_r_value = 0x104d61);
    PROVIDE(pke_eddsa_ed25519_get_k_value = 0x104df9);
    PROVIDE(pke_eddsa_ed25519_genkey = 0x104ea9);
    PROVIDE(pke_eddsa_ed25519_write_signature = 0x104f1d);
    PROVIDE(pke_eddsa_ed25519_read_signature = 0x104ffd);
    PROVIDE(pke_rsa_witness = 0x105039);
    PROVIDE(pke_rsa_exp_mod = 0x1050d5);
    PROVIDE(pke_get_pke_addr = 0x105175);
    PROVIDE(pke_lock_mutex = 0x105195);
    PROVIDE(pke_unlock_mutex = 0x10521d);
    PROVIDE(pke_engine_init = 0x10522d);
    PROVIDE(pke_engine_check_status = 0x105265);
    PROVIDE(pke_engine_write = 0x105301);
    PROVIDE(pke_function_set_entry = 0x105329);
    PROVIDE(pke_read_string = 0x105351);
    PROVIDE(pke_reverse_array = 0x10540d);
    PROVIDE(io_assert_failed = 0x105435);
    PROVIDE(FLASH_UserMode_Enter = 0x105451);
    PROVIDE(FLASH_UserMode_Exit = 0x105471);
    PROVIDE(FLASH_Config = 0x10548d);
    PROVIDE(FLASH_SetSpiMode = 0x105521);
    PROVIDE(FLASH_RxCmd_InUserMode = 0x1055fd);
    PROVIDE(FLASH_RxCmd = 0x10563d);
    PROVIDE(FLASH_WaitBusy_InUserMode = 0x105653);
    PROVIDE(FLASH_RxBasic_InUserMode = 0x1056c5);
    PROVIDE(FLASH_RxData = 0x1057c9);
    PROVIDE(FLASH_TxBasic_InUserMode = 0x1057f5);
    PROVIDE(FLASH_TxCmd_InUserMode = 0x1058d5);
    PROVIDE(FLASH_WriteEn_InUserMode = 0x1058fd);
    PROVIDE(FLASH_TxBasic = 0x105921);
    PROVIDE(FLASH_TxData = 0x10595d);
    PROVIDE(FLASH_SetStatus = 0x1059bd);
    PROVIDE(FLASH_Erase = 0x1059e5);
    PROVIDE(FLASH_Addr4ByteEn = 0x105a61);
    PROVIDE(FLASH_Addr4ByteDis = 0x105a83);
    PROVIDE(FLASH_DeepPowerDown = 0x105aa5);
    PROVIDE(FLASH_SetStatusBits = 0x105afd);
    PROVIDE(FLASH_RxBasic = 0x105ba5);
    PROVIDE(FLASH_StructInit = 0x105bf1);
    PROVIDE(FLASH_StructInit_Micron = 0x105c39);
    PROVIDE(FLASH_StructInit_MXIC = 0x105c5d);
    PROVIDE(FLASH_StructInit_GD = 0x105c75);
    PROVIDE(FLASH_Init = 0x105c81);
    PROVIDE(FLASH_Wait_Boot_Finish = 0x105ca1);
    PROVIDE(FLASH_DeepSleep_PDEX = 0x105cc5);
    PROVIDE(OTP_Read8_Rom = 0x105e49);
    PROVIDE(OTP_ProgramMarginRead8 = 0x105e4d);
    PROVIDE(_OTP_Write8 = 0x105e51);
    PROVIDE(OTP_Write8_Rom = 0x105eed);
    PROVIDE(OTPGetCRC = 0x105fc5);
    PROVIDE(OTP_Read8 = 0x105fe1);
    PROVIDE(OTP_Read32 = 0x106025);
    PROVIDE(otp_logical_remain = 0x1060a1);
    PROVIDE(OTPSetCRC = 0x1060bb);
    PROVIDE(OTP_LogicalMap_Read = 0x1061c9);
    PROVIDE(OTP_Write8 = 0x1062c9);
    PROVIDE(OTP_LogicalMap_Write = 0x106415);
    PROVIDE(shell_array_init = 0x1065d5);
    PROVIDE(shell_get_argc = 0x1065e3);
    PROVIDE(shell_get_argv = 0x10660f);
    PROVIDE(shell_cmd_chk = 0x1066d1);
    PROVIDE(shell_uart_irq_rom = 0x1067bd);
    PROVIDE(shell_init_rom = 0x10686d);
    PROVIDE(shell_task_rom = 0x1068dd);
    PROVIDE(shell_rom = 0x106905);
    PROVIDE(RTIM_TimeBaseStructInit = 0x106925);
    PROVIDE(RTIM_Cmd = 0x10693d);
    PROVIDE(RTIM_GetCount = 0x106961);
    PROVIDE(RTIM_INTConfig = 0x106965);
    PROVIDE(RTIM_INTClear = 0x106999);
    PROVIDE(RTIM_TimeBaseInit = 0x1069af);
    PROVIDE(RTIM_DeInit = 0x106ad5);
    PROVIDE(RTIM_INTClearPendingBit = 0x106af5);
    PROVIDE(RTIM_GetFlagStatus = 0x106b2d);
    PROVIDE(RTIM_GetINTStatus = 0x106b75);
    PROVIDE(BKUP_Write = 0x106bc5);
    PROVIDE(BKUP_Read = 0x106be9);
    PROVIDE(BKUP_Set = 0x106c0d);
    PROVIDE(BKUP_Clear = 0x106c35);
    PROVIDE(GPIO_INTMode = 0x106c99);
    PROVIDE(GPIO_INTConfig = 0x106d19);
    PROVIDE(GPIO_DeInit = 0x106d5d);
    PROVIDE(GPIO_ReadDataBit = 0x106dad);
    PROVIDE(GPIO_PortDirection = 0x106de5);
    PROVIDE(GPIO_PortRead = 0x106e2d);
    PROVIDE(GPIO_PortWrite = 0x106e69);
    PROVIDE(GPIO_DebounceClock = 0x106ead);
    PROVIDE(GPIO_LevelSync = 0x106f01);
    PROVIDE(GPIO_INTStatusGet = 0x106f41);
    PROVIDE(GPIO_INTStatusClearEdge = 0x106f4b);
    PROVIDE(GPIO_SetDWWrapper = 0x106f57);
    PROVIDE(GPIO_GetDWWrapper = 0x106f7d);
    PROVIDE(GPIO_Direction = 0x106f8d);
    PROVIDE(GPIO_Init = 0x106ff5);
    PROVIDE(GPIO_ODWriteBit = 0x107081);
    PROVIDE(GPIO_WriteBit = 0x107095);
    PROVIDE(GPIO_ODInit = 0x1070fd);
    PROVIDE(PAD_DrvStrength = 0x10712d);
    PROVIDE(PAD_SchmitCtrl = 0x10714d);
    PROVIDE(PAD_SlewRateCtrl = 0x10716d);
    PROVIDE(PAD_PullCtrl = 0x10718d);
    PROVIDE(PAD_SleepPullCtrl = 0x10727d);
    PROVIDE(PAD_ResistorCtrl = 0x10736d);
    PROVIDE(PAD_InputCtrl = 0x10738d);
    PROVIDE(PAD_CMD = 0x1073ad);
    PROVIDE(PAD_SpicCtrl = 0x1073cd);
    PROVIDE(Pinmux_Config = 0x107425);
    PROVIDE(Pinmux_ConfigGet = 0x1074ab);
    PROVIDE(Pinmux_UartLogCtrl = 0x1074b9);
    PROVIDE(Pinmux_BootSpicCtrl = 0x1074e5);
    PROVIDE(Pinmux_ComboSpicCtrl = 0x1074e9);
    PROVIDE(Pinmux_Swdoff = 0x1074f3);
    PROVIDE(RCC_PeriphClockCmd = 0x107509);
    PROVIDE(SPIC_Config = 0x1075d5);
    PROVIDE(NAND_SetSpiMode = 0x1076a9);
    PROVIDE(NAND_RxCmd = 0x107739);
    PROVIDE(NAND_GetStatus = 0x1077f5);
    PROVIDE(NAND_WaitBusy = 0x107821);
    PROVIDE(FLASH_UserModeEn = 0x1078a1);
    PROVIDE(NAND_UserModeEn = 0x1078c5);
    PROVIDE(NAND_RxData = 0x1078c9);
    PROVIDE(NAND_Page_Read_FromCache = 0x1079b5);
    PROVIDE(NAND_TxCmd = 0x1079c1);
    PROVIDE(NAND_WriteEn = 0x107a69);
    PROVIDE(NAND_TxData = 0x107a91);
    PROVIDE(NAND_Page_Write_Data_Xfer = 0x107b8d);
    PROVIDE(NAND_SetStatus = 0x107bad);
    PROVIDE(NAND_Erase = 0x107bdd);
    PROVIDE(NAND_Page_Read_ArrayToCache = 0x107c1d);
    PROVIDE(NAND_Page_Read = 0x107c51);
    PROVIDE(NAND_CheckBadBlock = 0x107c75);
    PROVIDE(NAND_Page_Write_Program_Execute = 0x107cc5);
    PROVIDE(NAND_Page_Write = 0x107d01);
    PROVIDE(NAND_SetStatusBits = 0x107d3d);
    PROVIDE(NAND_GetPageSize = 0x107d65);
    PROVIDE(NAND_StructInit = 0x107e0d);
    PROVIDE(NAND_StructInit_Micron = 0x107e61);
    PROVIDE(NAND_StructInit_MXIC = 0x107e77);
    PROVIDE(NAND_StructInit_GD = 0x107e85);
    PROVIDE(NAND_Reset = 0x107e95);
    PROVIDE(NAND_Init = 0x107ee5);
    PROVIDE(TRNG_Random_Delay = 0x107f05);
    PROVIDE(TRNG_Security_Only = 0x107f75);
    PROVIDE(TRNG_INTConfig = 0x107fa9);
    PROVIDE(TRNG_ClearINT = 0x107fd5);
    PROVIDE(TRNG_GetINT = 0x107ffd);
    PROVIDE(TRNG_Init = 0x108021);
    PROVIDE(TRNG_GetFIFOLevel = 0x108039);
    PROVIDE(_rand = 0x10805d);
    PROVIDE(__wrap_rand = 0x1080a1);
    PROVIDE(TRNG_get_random_bytes = 0x1080a5);
    PROVIDE(TRNG_get_random_bytes_f_rng = 0x10816d);
    PROVIDE(EFUSE_GetChipVersion = 0x108175);
    PROVIDE(EFUSE_GetBDNum = 0x108195);
    PROVIDE(EFUSE_GetESFlag = 0x1081b5);
    PROVIDE(EFUSE_GetUUID = 0x1081d5);
    PROVIDE(EFUSE_Get_Info = 0x10821d);
    PROVIDE(EFUSE_GetChipInfo = 0x10828d);
    PROVIDE(LOGUART_StructInit = 0x1082a9);
    PROVIDE(LOGUART_SetBaud = 0x1082d9);
    PROVIDE(LOGUART_RxCmd = 0x108381);
    PROVIDE(LOGUART_INTCoreConfig = 0x108393);
    PROVIDE(LOGUART_INT_NP2AP = 0x1083a3);
    PROVIDE(LOGUART_INT_AP2NP = 0x1083c1);
    PROVIDE(LOGUART_AGGCmd = 0x1083dd);
    PROVIDE(LOGUART_AGGPathCmd = 0x1083ef);
    PROVIDE(LOGUART_AGGSetTimeOut = 0x108459);
    PROVIDE(LOGUART_AGGSetThreshold = 0x108475);
    PROVIDE(LOGUART_AGGSetSyncPattern = 0x108499);
    PROVIDE(LOGUART_AGGPathAllCmd = 0x1084a7);
    PROVIDE(LOGUART_WaitTx = 0x1084c5);
    PROVIDE(LOGUART_PutChar = 0x1084fd);
    PROVIDE(LOGUART_GetChar = 0x10853d);
    PROVIDE(LOGUART_PutStr = 0x108551);
    PROVIDE(LOGUART_Writable = 0x1085a9);
    PROVIDE(LOGUART_Readable = 0x1085d5);
    PROVIDE(LOGUART_GetRxCount = 0x1085e5);
    PROVIDE(LOGUART_GetIMR = 0x1085f1);
    PROVIDE(LOGUART_SetIMR = 0x1085fd);
    PROVIDE(LOGUART_INTConfig = 0x108609);
    PROVIDE(LOGUART_INTClear = 0x108647);
    PROVIDE(LOGUART_GetStatus = 0x10864f);
    PROVIDE(LOGUART_ClearRxFifo = 0x108653);
    PROVIDE(_LOGUART_Init = 0x10866f);
    PROVIDE(LOGUART_Init = 0x1087c9);
    PROVIDE(LOGUART_WaitBusy = 0x108829);
    PROVIDE(LOGUART_WaitTxComplete = 0x108861);
    PROVIDE(LOGUART_BT_SendData = 0x10888d);
    PROVIDE(LOGUART_Relay_StructInit = 0x1088d1);
    PROVIDE(LOGUART_Relay_SetFormat = 0x1088e3);
    PROVIDE(LOGUART_Relay_SetBaud = 0x108929);
    PROVIDE(LOGUART_Relay_ClearRxFifo = 0x10897d);
    PROVIDE(LOGUART_Relay_RxCmd = 0x10899f);
    PROVIDE(LOGUART_MonitorParaConfig = 0x1089b1);
    PROVIDE(LOGUART_LPBaudSet = 0x1089ed);
    PROVIDE(LOGUART_Relay_LPSetBaud = 0x108a91);
    PROVIDE(LOGUART_RxMonitorCmd = 0x108aed);
    PROVIDE(LOGUART_RxMonBaudCtrlRegGet = 0x108aff);
    PROVIDE(LOGUART_RxMonitorSatusGet = 0x108b03);
    PROVIDE(LOGUART_DiagInit = 0x108b07);
    PROVIDE(SDIO_StructInit = 0x108b95);
    PROVIDE(SDIO_Init = 0x108bb1);
    PROVIDE(SDIO_INTClear = 0x108c5d);
    PROVIDE(SDIO_INTClearAll = 0x108c63);
    PROVIDE(SDIO_INTConfig = 0x108c6f);
    PROVIDE(SDIO_INTStatus = 0x108c8b);
    PROVIDE(SDIO_RPWM1_Get = 0x108c93);
    PROVIDE(SDIO_RPWM2_Get = 0x108c9b);
    PROVIDE(SDIO_CPWM1_Set = 0x108ca3);
    PROVIDE(SDIO_CPWM2_Set = 0x108cc5);
    PROVIDE(SDIO_H2C_MSG_Get = 0x108ced);
    PROVIDE(SDIO_H2C_MSG_EXT_Get = 0x108cf3);
    PROVIDE(SDIO_C2H_MSG_Set = 0x108cf9);
    PROVIDE(SDIO_C2H_MSG_EXT_Set = 0x108d07);
    PROVIDE(SDIO_Tx_AGG_Set = 0x108d0d);
    PROVIDE(SDIO_RXBD_RPTR_Get = 0x108d25);
    PROVIDE(SDIO_RXBD_WPTR_Set = 0x108d2d);
    PROVIDE(SDIO_TXBD_WPTR_Get = 0x108d33);
    PROVIDE(SDIO_TXBD_RPTR_Set = 0x108d3b);
    PROVIDE(SDIO_TXBD_RPTR_Get = 0x108d41);
    PROVIDE(SDIO_DMA_Reset = 0x108d49);
    PROVIDE(SDIO_DMA_CTRL_Get = 0x108d51);
    PROVIDE(SDIO_SetReady = 0x108d57);
    PROVIDE(SDIO_RxReq = 0x108d6f);
    PROVIDE(SYSCFG_TRP_UARTImage = 0x108d79);
    PROVIDE(SYSCFG_TRP_ICFG = 0x108d89);
    PROVIDE(SYSCFG_TRP_TestMode = 0x108da1);
    PROVIDE(SYSCFG_TRP_OTPBYP = 0x108db1);
    PROVIDE(SYSCFG_TRP_Boot_Int = 0x108dc1);
    PROVIDE(SYSCFG_TRP_Boot_Nor = 0x108dd1);
    PROVIDE(SYSCFG_CHIPType_Get = 0x108de1);
    PROVIDE(SYSCFG_RLVersion = 0x108df1);
    PROVIDE(SYSCFG_GetRLNum = 0x108e15);
    PROVIDE(SYSCFG_OTP_BOOTSEL = 0x108e39);
    PROVIDE(SYSCFG_OTP_BTFUNC = 0x108e47);
    PROVIDE(SYSCFG_OTP_SDIO_PINMUX = 0x108e55);
    PROVIDE(SYSCFG_OTP_BootMCMFlash = 0x108e63);
    PROVIDE(SYSCFG_OTP_FlashDSleepEn = 0x108e93);
    PROVIDE(SYSCFG_OTP_SPICAddr4ByteEn = 0x108ea1);
    PROVIDE(SYSCFG_OTP_SPICBootSpeedUpDis = 0x108eaf);
    PROVIDE(SYSCFG_OTP_BootFlashBaud = 0x108ebd);
    PROVIDE(SYSCFG_OTP_BigPageNand = 0x108ecb);
    PROVIDE(SYSCFG_OTP_BOOT_CNT_LMT_EN = 0x108ed9);
    PROVIDE(SYSCFG_OTP_BootFromNor = 0x108ee7);
    PROVIDE(SYSCFG_OTP_DisBootLog = 0x108f11);
    PROVIDE(SYSCFG_OTP_LowBaudLog = 0x108f1f);
    PROVIDE(SYSCFG_OTP_SBootEn = 0x108f2d);
    PROVIDE(SYSCFG_OTP_RDPEn = 0x108f4b);
    PROVIDE(SYSCFG_OTP_RSIPEn = 0x108f69);
    PROVIDE(SYSCFG_OTP_UsbDownloadEn = 0x108f87);
    PROVIDE(SYSCFG_OTP_RandDelayEn = 0x108f93);
    PROVIDE(SYSCFG_OTP_AntiRollback = 0x108fa5);
    PROVIDE(SYSCFG_OTP_FaultLogDis = 0x108fb7);
    PROVIDE(SYSCFG_OTP_RSIPMode = 0x108fc9);
    PROVIDE(SYSCFG_OTP_HUKDerivEn = 0x108fe1);
    PROVIDE(SYSCFG_OTP_PKID2 = 0x108ff3);
    PROVIDE(SYSCFG_HWTrigDownload = 0x109005);
    PROVIDE(SYSCFG_SWTrigDownload = 0x109035);
    PROVIDE(SYSCFG_OTP_AuthAlg = 0x10905d);
    PROVIDE(SYSCFG_OTP_HashAlg = 0x10906b);
    PROVIDE(SYSCFG_OTP_OTAAddr = 0x109077);
    PROVIDE(SYSCFG_OTP_BootVer = 0x109083);
    PROVIDE(SYSCFG_OTP_RMAMode = 0x1090b9);
    PROVIDE(SYSCFG_OTP_RomPatchEn = 0x1090e9);
    PROVIDE(UART_DeInit = 0x109101);
    PROVIDE(UART_StructInit = 0x109107);
    PROVIDE(UART_BaudParaGetFull = 0x109123);
    PROVIDE(UART_BaudParaGet = 0x109159);
    PROVIDE(UART_SetBaud = 0x109179);
    PROVIDE(UART_SetBaudExt = 0x109221);
    PROVIDE(UART_SetRxLevel = 0x1092ad);
    PROVIDE(UART_RxCmd = 0x1092bd);
    PROVIDE(UART_Writable = 0x1092cf);
    PROVIDE(UART_Readable = 0x1092d7);
    PROVIDE(UART_CharPut = 0x1092df);
    PROVIDE(UART_CharGet = 0x1092e3);
    PROVIDE(UART_ReceiveData = 0x1092e9);
    PROVIDE(UART_SendData = 0x10930d);
    PROVIDE(UART_ReceiveDataTO = 0x109331);
    PROVIDE(UART_SendDataTO = 0x10936d);
    PROVIDE(UART_RxByteCntClear = 0x1093a9);
    PROVIDE(UART_RxByteCntGet = 0x1093b3);
    PROVIDE(UART_BreakCtl = 0x1093b9);
    PROVIDE(UART_ClearRxFifo = 0x1093cd);
    PROVIDE(UART_ClearTxFifo = 0x1093e9);
    PROVIDE(UART_Init = 0x1093f3);
    PROVIDE(UART_INTConfig = 0x1094a1);
    PROVIDE(UART_INT_Clear = 0x1094af);
    PROVIDE(UART_IntStatus = 0x1094b7);
    PROVIDE(UART_ModemStatusGet = 0x1094bb);
    PROVIDE(UART_LineStatusGet = 0x1094bf);
    PROVIDE(UART_WaitBusy = 0x1094c3);
    PROVIDE(XTAL_ClkGet = 0x1094e5);
    PROVIDE(XTAL_Check_READY = 0x1094ed);
    PROVIDE(cmd_rom_table = 0x10968d);
    PROVIDE(IPC_SEMAInit = 0x109699);
    PROVIDE(IPC_SEMTake = 0x1096b9);
    PROVIDE(IPC_SEMFree = 0x109701);
    PROVIDE(IPC_SEMDelayStub = 0x109725);
    PROVIDE(SYSTIMER_Init = 0x109731);
    PROVIDE(SYSTIMER_TickGet = 0x10977d);
    PROVIDE(SYSTIMER_GetPassTick = 0x10978d);
    PROVIDE(SYSTIMER_GetPassTime = 0x1097ad);
    PROVIDE(WDG_Wait_Busy = 0x1097dd);
    PROVIDE(WDG_StructInit = 0x1097fd);
    PROVIDE(WDG_Init = 0x10980d);
    PROVIDE(WDG_Enable = 0x10984d);
    PROVIDE(WDG_Timeout = 0x109891);
    PROVIDE(WDG_Refresh = 0x1098c1);
    PROVIDE(WDG_INTConfig = 0x1098d1);
    PROVIDE(WDG_ClearINT = 0x1098f9);
    PROVIDE(IWDG_LP_Enable = 0x109917);
    PROVIDE(DiagFVSNprintf = 0x109c85);
    PROVIDE(DiagFVSNprintfNano = 0x10a03f);
    PROVIDE(DiagVSNprintf = 0x10a1b7);
    PROVIDE(DiagVSprintf = 0x10a1d1);
    PROVIDE(DiagVprintf = 0x10a1dd);
    PROVIDE(DiagPrintf = 0x10a1e9);
    PROVIDE(DiagVSNprintfNano = 0x10a203);
    PROVIDE(DiagVprintfNano = 0x10a21d);
    PROVIDE(DiagPrintfNano = 0x10a229);
    PROVIDE(DiagSPrintf = 0x10a243);
    PROVIDE(DiagSnPrintf = 0x10a25d);
    PROVIDE(rtk_log_array_clear = 0x10a279);
    PROVIDE(rtk_log_level_get = 0x10a295);
    PROVIDE(rtk_log_isprint = 0x10a2e5);
    PROVIDE(rtk_log_write = 0x10a2f5);
    PROVIDE(rtk_log_write_nano = 0x10a335);
    PROVIDE(rtk_log_array_print = 0x10a375);
    PROVIDE(rtk_log_level_set = 0x10a41d);
    PROVIDE(rtk_log_memory_dump_word = 0x10a491);
    PROVIDE(rtk_log_memory_dump_byte = 0x10a511);
    PROVIDE(rtk_log_memory_dump2char = 0x10a589);
    PROVIDE(crash_GetExStackContent = 0x10a671);
    PROVIDE(crash_GetExStackDepth = 0x10a75d);
    PROVIDE(crash_SetExStackDepth = 0x10a769);
    PROVIDE(crash_Diagnosis = 0x10a781);
    PROVIDE(crash_GetExStack = 0x10aab9);
    PROVIDE(crash_GetFaultInfo = 0x10ab01);
    PROVIDE(crash_GetGeneralReg = 0x10ab71);
    PROVIDE(crash_GetSysStatus = 0x10abcd);
    PROVIDE(crash_GetExReason = 0x10ae31);
    PROVIDE(crash_Main = 0x10ae85);
    PROVIDE(pke_ecdsa_write_signature_patched = 0x10aef1);
    PROVIDE(pke_ecdsa_read_signature_patched = 0x10b029);
    PROVIDE(__drom_common_start__ = 0x10c000);
    PROVIDE(SPIC_CALIB_PATTERN = 0x10ced4);
    PROVIDE(GPIO_PORTx_S = 0x10d194);
    PROVIDE(GPIO_PORTx = 0x10d1a0);
    PROVIDE(_ctype_ = 0x10d512);
    PROVIDE(crash_reg_name = 0x10e254);
    PROVIDE(crash_print_info = 0x10e298);
    PROVIDE(__drom_bank_start__ = 0x10e358);
    PROVIDE(__drom_common_end__ = 0x10e358);
    PROVIDE(RomVectorTable = 0x10e600);
    PROVIDE(AA__os_share_rom_start__ = 0x10f000);
    PROVIDE(AA__os_share_rom_end__ = 0x113636);
    PROVIDE(AA__wifi_share_rom_start__ = 0x114000);
    PROVIDE(AA__wifi_share_rom_end__ = 0x140bbc);
    PROVIDE(DelayNop = 0x147801);
    PROVIDE(DelayUs = 0x14780d);
    PROVIDE(DelayMs = 0x14786d);
    PROVIDE(DelayClkUpdate = 0x147881);
    PROVIDE(__km4tz_flash_text_start__ = 0x2000020);
    PROVIDE(__km4tz_flash_text_end__ = 0x4000000);
    PROVIDE(__km4ns_flash_text_start__ = 0x4000020);
    PROVIDE(__km4ns_flash_text_end__ = 0x6000000);
    PROVIDE(__km4tz_boot_text_start__ = 0x10400020);
    PROVIDE(__km4tz_boot_text_end__ = 0x10c00000);
    PROVIDE(__km4tz_img3_text_start__ = 0x10c00020);
    PROVIDE(__km4tz_img3_text_end__ = 0x11000000);
    PROVIDE(__rom_bss_start__ = 0x20001000);
    PROVIDE(UserIrqFunTable = 0x20001000);
    PROVIDE(UserIrqDataTable = 0x20001140);
    PROVIDE(flash_init_para = 0x20001280);
    PROVIDE(shell_ctl = 0x200012b4);
    PROVIDE(shell_argv_array = 0x200012d0);
    PROVIDE(shell_buf = 0x20001350);
    PROVIDE(shell_buf_array = 0x20001358);
    PROVIDE(shell_rxbuf = 0x200013d8);
    PROVIDE(shell_rxbuf_array = 0x200013e0);
    PROVIDE(rtk_log_tag_array = 0x20001460);
    PROVIDE(rtk_log_entry_count = 0x2000148c);
    PROVIDE(rtk_log_default_level = 0x20001490);
    PROVIDE(DiagFunc = 0x20001494);
    PROVIDE(crash_dump_depth = 0x20001498);
    PROVIDE(INT_SVCall_patch = 0x2000149c);
    PROVIDE(INT_PendSV_patch = 0x200014a0);
    PROVIDE(INT_SysTick_patch = 0x200014a4);
    PROVIDE(PrePatchHandler = 0x200014a8);
    PROVIDE(PostPatchHandler = 0x200014ac);
    PROVIDE(__rom_bss_chk__ = 0x200014b0);
    PROVIDE(__rom_bss_end__ = 0x200014b0);
    PROVIDE(__rom_bss_start_ns__ = 0x200014b0);
    PROVIDE(RBSS_UDELAY_DIV = 0x200014b0);
    PROVIDE(RBSS_UDELAY_CLK = 0x200014b4);
    PROVIDE(ipc_delay_func = 0x200014b8);
    PROVIDE(CrashDumpPatchHandler = 0x200014bc);
    PROVIDE(AA__wifi_share_bss_start__ = 0x200014c0);
    PROVIDE(AA__wifi_share_bss_end__ = 0x20001ced);
    PROVIDE(AA__os_share_bss_start__ = 0x20001cee);
    PROVIDE(AA__os_share_bss_end__ = 0x200020a4);
    PROVIDE(__retrieve_ap_buffer_start__ = 0x200020c0);
    PROVIDE(__rom_bss_end_ns__ = 0x20002400);
    PROVIDE(__ipc_memory_start__ = 0x20004400);
    PROVIDE(__km4tz_bd_ram_start__ = 0x2000b000);
    PROVIDE(__km4tz_bd_psram_start__ = 0x60000000);
}
SECTIONS
{
    PROVIDE(irq_clear_pending = 0x1010f5);
    PROVIDE(_memcmp = 0x1012f9);
    PROVIDE(_memcpy = 0x101365);
    PROVIDE(_memset = 0x1014f9);
    PROVIDE(_strlen = 0x101c1d);
    PROVIDE(__wrap_memset = 0x101f69);
    PROVIDE(__wrap_memcpy = 0x101f71);
    PROVIDE(__wrap_memcmp = 0x101f79);
    PROVIDE(__wrap_strlen = 0x101f91);
    PROVIDE(io_assert_failed = 0x105435);
    PROVIDE(SYSCFG_RLVersion = 0x108df1);
    PROVIDE(DiagPrintf = 0x10a1e9);
    PROVIDE(rtk_log_write = 0x10a2f5);
    PROVIDE(rtk_log_write_nano = 0x10a335);
    PROVIDE(AA__wifi_share_rom_start__ = 0x114000);
    PROVIDE(aes_set_key = 0x114039);
    PROVIDE(aes_encrypt = 0x114409);
    PROVIDE(aes_decrypt = 0x115291);
    PROVIDE(AES_WRAP = 0x116181);
    PROVIDE(AES_UnWRAP = 0x116335);
    PROVIDE(wifi_rom_omac1_aes_128_vector = 0x1164a1);
    PROVIDE(crc32_get = 0x116695);
    PROVIDE(arc4_byte = 0x1166c5);
    PROVIDE(rt_arc4_init = 0x1166f1);
    PROVIDE(rt_arc4_crypt = 0x116751);
    PROVIDE(rt_md5_init = 0x116f6d);
    PROVIDE(rt_md5_append = 0x116f99);
    PROVIDE(rt_md5_final = 0x117025);
    PROVIDE(rt_md5_hmac = 0x117081);
    PROVIDE(RC4 = 0x11721d);
    PROVIDE(RC4_set_key = 0x11754d);
    PROVIDE(dh_groups_get = 0x117631);
    PROVIDE(wifi_rom_mpi_init = 0x117659);
    PROVIDE(wifi_rom_mpi_free = 0x117665);
    PROVIDE(wifi_rom_mpi_grow = 0x117695);
    PROVIDE(wifi_rom_mpi_shrink = 0x1176e9);
    PROVIDE(wifi_rom_mpi_copy = 0x117771);
    PROVIDE(wifi_rom_mpi_safe_cond_assign = 0x1177ed);
    PROVIDE(wifi_rom_mpi_lset = 0x117871);
    PROVIDE(wifi_rom_mpi_get_bit = 0x1178e5);
    PROVIDE(wifi_rom_mpi_set_bit = 0x117905);
    PROVIDE(wifi_rom_mpi_lsb = 0x117955);
    PROVIDE(wifi_rom_mpi_bitlen = 0x117999);
    PROVIDE(wifi_rom_mpi_size = 0x1179f5);
    PROVIDE(wifi_rom_mpi_read_binary = 0x117a01);
    PROVIDE(wifi_rom_mpi_write_binary = 0x117a9d);
    PROVIDE(wifi_rom_mpi_shift_l = 0x117b29);
    PROVIDE(wifi_rom_mpi_shift_r = 0x117bed);
    PROVIDE(wifi_rom_mpi_cmp_abs = 0x117ca1);
    PROVIDE(wifi_rom_mpi_cmp_mpi = 0x117d31);
    PROVIDE(wifi_rom_mpi_cmp_int = 0x117ddd);
    PROVIDE(wifi_rom_mpi_add_abs = 0x117e0d);
    PROVIDE(wifi_rom_mpi_sub_abs = 0x117ee5);
    PROVIDE(wifi_rom_mpi_add_mpi = 0x117fe1);
    PROVIDE(wifi_rom_mpi_sub_mpi = 0x118031);
    PROVIDE(wifi_rom_mpi_add_int = 0x118081);
    PROVIDE(wifi_rom_mpi_sub_int = 0x1180f1);
    PROVIDE(wifi_rom_mpi_mul_hlp = 0x118161);
    PROVIDE(wifi_rom_mpi_mul_mpi = 0x118ad5);
    PROVIDE(wifi_rom_mpi_mul_int = 0x118c0d);
    PROVIDE(wifi_rom_mpi_div_mpi = 0x118c29);
    PROVIDE(wifi_rom_mpi_mod_mpi = 0x119139);
    PROVIDE(wifi_rom_mpi_exp_mod = 0x1191c9);
    PROVIDE(wifi_rom_mpi_gcd = 0x11971d);
    PROVIDE(wifi_rom_mpi_fill_random = 0x119989);
    PROVIDE(wifi_rom_mpi_inv_mod = 0x119a21);
    PROVIDE(wifi_rom_ecp_point_init = 0x11a985);
    PROVIDE(wifi_rom_ecp_group_init = 0x11a9a1);
    PROVIDE(wifi_rom_ecp_point_free = 0x11a9f5);
    PROVIDE(wifi_rom_ecp_group_free = 0x11aa15);
    PROVIDE(wifi_rom_ecp_copy = 0x11aaad);
    PROVIDE(wifi_rom_ecp_set_zero = 0x11aadd);
    PROVIDE(wifi_rom_ecp_is_zero = 0x11b181);
    PROVIDE(wifi_rom_ecp_point_cmp = 0x11b195);
    PROVIDE(wifi_rom_ecp_point_write_binary = 0x11b1c9);
    PROVIDE(wifi_rom_ecp_point_read_binary = 0x11b26d);
    PROVIDE(wifi_rom_ecp_mul_restartable = 0x11b301);
    PROVIDE(wifi_rom_ecp_mul = 0x11b7cd);
    PROVIDE(wifi_rom_ecp_muladd_restartable = 0x11b7e9);
    PROVIDE(wifi_rom_ecp_muladd = 0x11b881);
    PROVIDE(wifi_rom_ecp_check_pubkey = 0x11b89d);
    PROVIDE(wifi_rom_ecp_check_privkey = 0x11b8d5);
    PROVIDE(wifi_rom_mpi_add_mod = 0x11b915);
    PROVIDE(wifi_rom_mpi_mul_mod = 0x11b951);
    PROVIDE(wifi_rom_mpi_sub_int_mod = 0x11b971);
    PROVIDE(wifi_rom_ecp_sw_rhs = 0x11b9b1);
    PROVIDE(wifi_rom_ecp_sw_derive_y = 0x11ba79);
    PROVIDE(wifi_rom_ecp_group_load = 0x11c769);
    PROVIDE(wifi_rom_platform_zeroize = 0x11c8c5);
    PROVIDE(wep_80211_encrypt = 0x11c9f5);
    PROVIDE(wep_80211_decrypt = 0x11ca5d);
    PROVIDE(tkip_micappendbyte = 0x11caf1);
    PROVIDE(rtw_secmicsetkey = 0x11cb4d);
    PROVIDE(rtw_secmicappend = 0x11cb99);
    PROVIDE(rtw_secgetmic = 0x11cbfd);
    PROVIDE(rtw_seccalctkipmic = 0x11ccb5);
    PROVIDE(tkip_phase1 = 0x11cd89);
    PROVIDE(tkip_phase2 = 0x11cee1);
    PROVIDE(tkip_80211_encrypt = 0x11d0ad);
    PROVIDE(tkip_80211_decrypt = 0x11d135);
    PROVIDE(aes1_encrypt = 0x11d1e9);
    PROVIDE(aesccmp_construct_mic_iv = 0x11d4dd);
    PROVIDE(aesccmp_construct_mic_header1 = 0x11d541);
    PROVIDE(aesccmp_construct_mic_header2 = 0x11d5ad);
    PROVIDE(aesccmp_construct_ctr_preload = 0x11d649);
    PROVIDE(aes_80211_encrypt = 0x11d6d9);
    PROVIDE(aes_80211_decrypt = 0x11db15);
    PROVIDE(gcmp_80211_encrypt = 0x11ead9);
    PROVIDE(gcmp_80211_decrypt = 0x11ebbd);
    PROVIDE(bip_gcmp_protect = 0x11ed45);
    PROVIDE(wifi_rom_sha256_vector = 0x11ef39);
    PROVIDE(wifi_rom_hmac_sha256_vector = 0x11f149);
    PROVIDE(wifi_rom_hmac_sha256_kdf = 0x11f251);
    PROVIDE(wifi_rom_sha256_prf_bits = 0x11f329);
    PROVIDE(wifi_rom_sha256_prf = 0x11f429);
    PROVIDE(wifi_rom_hmac_sha384_vector = 0x11fa25);
    PROVIDE(wifi_rom_sha384_prf = 0x11fb31);
    PROVIDE(rt_sha1_init = 0x11fecd);
    PROVIDE(rt_sha1_update = 0x11ff1d);
    PROVIDE(rt_sha1_finish = 0x11ff85);
    PROVIDE(rt_hmac_sha1 = 0x11ff95);
    PROVIDE(wifi_rom_enc_gtk = 0x120305);
    PROVIDE(wifi_rom_gen_nonce = 0x120361);
    PROVIDE(wifi_rom_wpa_key_mgmt_sae = 0x1203f9);
    PROVIDE(wifi_rom_wpa_key_mgmt_sha256 = 0x120405);
    PROVIDE(wifi_rom_wpa_key_mgmt_sha384 = 0x120411);
    PROVIDE(wifi_rom_ap_constructEAPOL_1Of4Way = 0x120419);
    PROVIDE(wifi_rom_wpa_parse_rsnxe = 0x120559);
    PROVIDE(wifi_rom_wpa_append_rsnxe = 0x1205ad);
    PROVIDE(wifi_rom_check_append_rsnxe = 0x1205d9);
    PROVIDE(wifi_rom_set_playercounter = 0x12062d);
    PROVIDE(wifi_rom_equal_playercounter = 0x120635);
    PROVIDE(wifi_rom_fill_eapol_header = 0x120651);
    PROVIDE(wifi_rom_check_playercounter = 0x1206ad);
    PROVIDE(wifi_rom_set_group_cipher = 0x1206d1);
    PROVIDE(wifi_rom_parse_group_kde = 0x1206fd);
    PROVIDE(wifi_rom_psk_ieee80211w_kde_add = 0x120831);
    PROVIDE(wifi_rom_ap_constructEAPOL_1Of2Way = 0x120891);
    PROVIDE(wifi_rom_ap_fill_keydata = 0x120a79);
    PROVIDE(wifi_rom_ap_constructEAPOL_3Of4Way = 0x120b55);
    PROVIDE(strtopsk = 0x120e3d);
    PROVIDE(rom_psk_PasswordHash = 0x120efd);
    PROVIDE(rom_psk_CalcGTK = 0x120f39);
    PROVIDE(rom_psk_CalcPTK = 0x120ff5);
    PROVIDE(_ntohs_rom = 0x12114d);
    PROVIDE(wifi_rom_message_replaycounter_oc2li = 0x121155);
    PROVIDE(wifi_rom_message_equalreplaycounter = 0x121171);
    PROVIDE(wifi_rom_message_smallerequalreplaycounter = 0x1211a5);
    PROVIDE(wifi_rom_message_smallerreplaycounter = 0x1211e1);
    PROVIDE(wifi_rom_message_setreplaycounter = 0x12121d);
    PROVIDE(wifi_rom_inc_largeinteger = 0x121239);
    PROVIDE(wifi_rom_inc_octet16_integer = 0x121255);
    PROVIDE(wifi_rom_inc_octet32_integer = 0x1212c5);
    PROVIDE(wifi_rom_set_eapolkeyiv = 0x121331);
    PROVIDE(wifi_rom_check_mic = 0x1213c5);
    PROVIDE(wifi_rom_calc_mic = 0x1214e5);
    PROVIDE(wifi_rom_decrypt_WPA2keydata = 0x1215bd);
    PROVIDE(wifi_rom_decrypt_GTK = 0x1216e9);
    PROVIDE(wifi_rom_client_constructEAPOL_2Of4Way = 0x1217d5);
    PROVIDE(wifi_rom_client_constructEAPOL_4Of4Way = 0x1218e5);
    PROVIDE(wifi_rom_client_constructEAPOL_2Of2Way = 0x1219bd);
    PROVIDE(wifi_rom_client_constructEAPOL_MICof2Way = 0x121aad);
    PROVIDE(wifi_rom_eapol_check_rsne_pmkid = 0x121b8d);
    PROVIDE(wifi_rom_sae_octect_buf_alloc = 0x1220d9);
    PROVIDE(wifi_rom_sae_octect_buf_clear = 0x12211d);
    PROVIDE(wifi_rom_sae_set_state = 0x122149);
    PROVIDE(wifi_rom_sae_get_rand_mask = 0x1221bd);
    PROVIDE(wifi_rom_sae_ecc_point_invert = 0x12234d);
    PROVIDE(wifi_rom_sae_clear_data = 0x1223a1);
    PROVIDE(wifi_rom_sae_check_group = 0x1223a9);
    PROVIDE(wifi_rom_sae_blind_select_group = 0x12241d);
    PROVIDE(wifi_rom_sae_target_select_group = 0x122471);
    PROVIDE(wifi_rom_sae_derive_pt = 0x1224e5);
    PROVIDE(wifi_rom_sae_prepare_commit = 0x122521);
    PROVIDE(wifi_rom_sae_prepare_confirm = 0x1226d5);
    PROVIDE(wifi_rom_sae_del_event = 0x1227d9);
    PROVIDE(wifi_rom_sae_parse_peer_commit = 0x122805);
    PROVIDE(wifi_rom_sae_process_peer_commit = 0x122b3d);
    PROVIDE(wifi_rom_sae_check_peer_confirm = 0x122ba1);
    PROVIDE(wifi_rom_sae_check_big_sync = 0x122c5d);
    PROVIDE(wifi_rom_sae_generate_commit_element_ecc = 0x122c69);
    PROVIDE(wifi_rom_sae_generate_commit_element_ffc = 0x122cc9);
    PROVIDE(wifi_rom_is_duplicate_sae_auth = 0x122d21);
    PROVIDE(wifi_rom_dump_bignum = 0x122fa9);
    PROVIDE(wifi_rom_dump_octect_string = 0x12304d);
    PROVIDE(wifi_rom_ecc_point_read_bignum = 0x1230d1);
    PROVIDE(wifi_rom_ecc_point_write_bignum = 0x123159);
    PROVIDE(wifi_rom_dump_ecc_point = 0x1231f1);
    PROVIDE(wifi_rom_ecc_point_read_binary = 0x123271);
    PROVIDE(wifi_rom_ecc_point_write_binary = 0x1232e1);
    PROVIDE(wifi_rom_ecc_point_add_point = 0x123315);
    PROVIDE(wifi_rom_bignum_add_mod = 0x123361);
    PROVIDE(wifi_rom_bignum_mul_mod = 0x123385);
    PROVIDE(wifi_rom_sae_memcmp = 0x123a2d);
    PROVIDE(wifi_rom_sae_buf_shift_right = 0x123a55);
    PROVIDE(wifi_rom_sae_crypto_ecc_init = 0x123a8d);
    PROVIDE(wifi_rom_sae_crypto_ecc_free = 0x123b45);
    PROVIDE(wifi_rom_sae_derive_pwe_ecc = 0x123b6d);
    PROVIDE(wifi_rom_sae_derive_pwe_ffc = 0x124355);
    PROVIDE(wifi_rom_sae_derive_pwe_from_pt = 0x124521);
    PROVIDE(wifi_rom_sae_derive_k_ecc = 0x12468d);
    PROVIDE(wifi_rom_sae_derive_k_ffc = 0x124755);
    PROVIDE(wifi_rom_sae_derive_pmk_and_kck = 0x124801);
    PROVIDE(wifi_rom_sae_derive_pt_ecc = 0x124ae1);
    PROVIDE(wifi_rom_dot11w_construct_action_frame = 0x124cad);
    PROVIDE(wifi_rom_dot11w_BIP_verify = 0x124d71);
    PROVIDE(wifi_rom_para_set_for_addba = 0x124f11);
    PROVIDE(wifi_rom_on_action_csa = 0x124f49);
    PROVIDE(wifi_rom_issue_assocreq_fill_cap = 0x124f99);
    PROVIDE(wifi_rom_issue_assocreq_fill_vendor_ie = 0x125171);
    PROVIDE(wifi_rom_issue_assocrsp_fill_cap = 0x1252b1);
    PROVIDE(wifi_rom_on_assocreq_check = 0x1254d5);
    PROVIDE(wifi_rom_on_assocrsp_wmm = 0x125869);
    PROVIDE(wifi_rom_set_beacon_timeout = 0x1258f1);
    PROVIDE(wifi_rom_beacon_rate_check = 0x125915);
    PROVIDE(wifi_rom_beacon_bw_check = 0x125981);
    PROVIDE(wifi_rom_beacon_ssid_check = 0x1259a5);
    PROVIDE(wifi_rom_beacon_encryption_check = 0x125a31);
    PROVIDE(wifi_rom_beacon_info_get = 0x125b39);
    PROVIDE(wifi_rom_beacon_csa_info_get = 0x125c59);
    PROVIDE(wifi_rom_ap_beacon_update_hidden_ssid = 0x125cb1);
    PROVIDE(wifi_rom_ap_beacon_update_bcn_tim = 0x125d09);
    PROVIDE(wifi_rom_ap_beacon_update_erpinfo_ie = 0x125e29);
    PROVIDE(wifi_rom_ap_beacon_add_ie = 0x125e89);
    PROVIDE(wifi_rom_ap_beacon_remove_ie = 0x125fa5);
    PROVIDE(wifi_rom_ap_beacon_gen_ie = 0x126035);
    PROVIDE(wifi_rom_get_center_ch = 0x126445);
    PROVIDE(wifi_rom_get_offset_by_chbw = 0x126505);
    PROVIDE(wifi_rom_get_primary_ch = 0x1265a9);
    PROVIDE(wifi_rom_he_get_highest_rate = 0x1265e9);
    PROVIDE(wifi_rom_he_nss_to_mcsmap = 0x126641);
    PROVIDE(wifi_rom_he_mcsmap_to_bitmap = 0x126691);
    PROVIDE(rtw_get_bit_value_from_ieee_value = 0x126735);
    PROVIDE(rtw_is_cckrates_included = 0x126759);
    PROVIDE(rtw_is_cckratesonly_included = 0x126791);
    PROVIDE(rtw_check_wireless_mode = 0x1267d1);
    PROVIDE(rtw_set_fixed_ie = 0x126865);
    PROVIDE(rtw_set_ie = 0x126885);
    PROVIDE(rtw_get_ie = 0x1268b1);
    PROVIDE(rtw_set_supported_rate = 0x1268ed);
    PROVIDE(rtw_get_rateset_len = 0x126941);
    PROVIDE(rtw_get_wpa_ie = 0x126959);
    PROVIDE(rtw_get_wpa2_ie = 0x1269fd);
    PROVIDE(rtw_get_wpa_cipher_suite = 0x126a39);
    PROVIDE(rtw_get_wpa2_cipher_suite = 0x126aa1);
    PROVIDE(rtw_parse_wpa_ie = 0x126b19);
    PROVIDE(rtw_parse_wpa2_ie = 0x126bd9);
    PROVIDE(rtw_get_sec_ie = 0x126cc5);
    PROVIDE(rtw_get_wps_ie = 0x126d75);
    PROVIDE(rtw_get_wps_attr = 0x126df9);
    PROVIDE(rtw_get_wps_attr_content = 0x126e89);
    PROVIDE(rtw_ieee802_11_parse_elems = 0x126ed1);
    PROVIDE(str_2char2num = 0x12707d);
    PROVIDE(key_2char2num = 0x1270fd);
    PROVIDE(convert_ip_addr = 0x12716d);
    PROVIDE(wifi_rom_channel_num2bitmap_index = 0x127245);
    PROVIDE(wifi_rom_channel_bitmap_index2num = 0x1272b1);
    PROVIDE(wifi_rom_channel_is_contained = 0x1272dd);
    PROVIDE(wifi_rom_chset_is_chbw_valid = 0x127361);
    PROVIDE(wifi_rom_get_band_type_by_channel_plan = 0x127419);
    PROVIDE(wifi_rom_init_channel_set = 0x12744d);
    PROVIDE(wifi_rom_update_wireless_mode = 0x1274e5);
    PROVIDE(wifi_rom_is_ap_in_tkip = 0x12755d);
    PROVIDE(wifi_rom_support_short_GI = 0x1275e5);
    PROVIDE(wifi_rom_fill_wlan_header = 0x1275f5);
    PROVIDE(wifi_rom_get_wpa2_akmp_suite = 0x127669);
    PROVIDE(wifi_rom_get_mgmt_group_cipher_suite = 0x12775d);
    PROVIDE(wifi_rom_parse_wpa_ie_akm = 0x1277b5);
    PROVIDE(wifi_rom_parse_sec_ie_akm_mgc = 0x127855);
    PROVIDE(wifi_rom_fill_frame_content = 0x127971);
    PROVIDE(wifi_rom_set_ie_ram = 0x12798d);
    PROVIDE(wifi_rom_get_ie_ex = 0x1279cd);
    PROVIDE(wifi_rom_parse_rsnxe = 0x127a4d);
    PROVIDE(wifi_rom_ht_mcsset_to_nss = 0x127aa9);
    PROVIDE(wifi_rom_append_rsnxe = 0x127ac5);
    PROVIDE(wifi_rom_restruct_wmm_ie = 0x127af1);
    PROVIDE(wifi_rom_restruct_sec_ie = 0x127b99);
    PROVIDE(wifi_rom_append_exented_cap = 0x127c29);
    PROVIDE(wifi_rom_HT_caps_handler = 0x127c9d);
    PROVIDE(wifi_rom_check_rsn_ie_pmkid = 0x127d01);
    PROVIDE(wifi_rom_pmksa_cache_encrypt_macaddr = 0x127d79);
    PROVIDE(wifi_rom_pmksa_append_pmkid = 0x127ec5);
    PROVIDE(wifi_rom_set_probersp_tim_ie = 0x127fb1);
    PROVIDE(wifi_rom_on_probereq = 0x128029);
    PROVIDE(wifi_rom_psk_strip_rsn_pairwise = 0x128079);
    PROVIDE(wifi_rom_psk_strip_wpa_pairwise = 0x128111);
    PROVIDE(wifi_rom_psk_strip_pmkid = 0x1281c9);
    PROVIDE(wifi_rom_psk_strip_rsn_akmp = 0x128235);
    PROVIDE(wifi_rom_cckrates_included = 0x128461);
    PROVIDE(wifi_rom_cckratesonly_included = 0x1284a5);
    PROVIDE(wifi_rom_updateBratetbl = 0x1284ed);
    PROVIDE(wifi_rom_updateBratetbl_softap = 0x128549);
    PROVIDE(wifi_rom_rate2tbl_inx = 0x128585);
    PROVIDE(wifi_rom_update_mcs_rate = 0x12861d);
    PROVIDE(wifi_rom_get_highest_rateidx = 0x128629);
    PROVIDE(wifi_rom_check_pn = 0x128651);
    PROVIDE(wifi_rom_check_indicate_seq = 0x128699);
    PROVIDE(wifi_rom_get_wmmps_ac = 0x1286e1);
    PROVIDE(wifi_rom_get_op_chs_by_cch_bw = 0x128725);
    PROVIDE(wifi_rom_ch2freq = 0x1287a1);
    PROVIDE(wifi_rom_freq2ch = 0x1287d5);
    PROVIDE(wifi_rom_chbw_to_freq_range = 0x128851);
    PROVIDE(wifi_rom_ch_to_bb_gain_sel = 0x1288cd);
    PROVIDE(wifi_rom_vht_get_highest_rate = 0x128915);
    PROVIDE(wifi_rom_vht_mcsmap_to_nss = 0x128985);
    PROVIDE(wifi_rom_vht_nss_to_mcsmap = 0x1289e1);
    PROVIDE(wifi_rom_vht_mcsmap_to_bitmap = 0x128a2d);
    PROVIDE(rtw_ht_caps_handler = 0x128ad1);
    PROVIDE(rtw_ht_info_handler = 0x128b19);
    PROVIDE(rtw_restructure_ht_ie = 0x128b41);
    PROVIDE(rtw_update_ht_cap = 0x128c1d);
    PROVIDE(rtw_ap_ht_operation_update = 0x128cf5);
    PROVIDE(rtw_uapsd_check_ap_support = 0x128dc1);
    PROVIDE(rtw_uapsd_restruct_wmm_ie = 0x128e2d);
    PROVIDE(rtw_network_update_for_scan = 0x128ee9);
    PROVIDE(rtw_network_check_same = 0x128f39);
    PROVIDE(rtw_pmksa_cache_entry_free = 0x128fa1);
    PROVIDE(rtw_pmksa_cache_entry_remove = 0x128fdd);
    PROVIDE(rtw_pmksa_cache_entry_add = 0x129049);
    PROVIDE(rtw_pmksa_cache_entry_search = 0x129169);
    PROVIDE(rtw_pmksa_cache_set_current_entry = 0x129195);
    PROVIDE(rtw_pmksa_cache_get_current_entry = 0x12919d);
    PROVIDE(rtw_pmksa_cache_flush = 0x1291a5);
    PROVIDE(rtw_pmksa_cache_init = 0x12920d);
    PROVIDE(rtw_pmksa_cache_deinit = 0x129235);
    PROVIDE(rtw_11d_get_chplan_from_country = 0x129259);
    PROVIDE(rtw_11d_restructure_country_ie = 0x129341);
    PROVIDE(rtw_11d_reset_info = 0x12937d);
    PROVIDE(rtw_11d_process_country_ie = 0x1293a5);
    PROVIDE(rtw_11d_country_decision_default = 0x129479);
    PROVIDE(HE_caps_handler = 0x12957d);
    PROVIDE(HE_operation_handler = 0x12966d);
    PROVIDE(HE_mu_edca_handler = 0x1296a5);
    PROVIDE(HE_spatial_reuse_handler = 0x1296ed);
    PROVIDE(HE_mac_caps_update_asoc_cap = 0x129701);
    PROVIDE(HE_phy_caps_update_asoc_cap = 0x1297fd);
    PROVIDE(HE_supp_mcs_update_asoc_cap = 0x1298ad);
    PROVIDE(rtw_restructure_he_ie = 0x1298b5);
    PROVIDE(rtw_he_fill_htc = 0x129b09);
    PROVIDE(HE_operation_update_asoc_cap = 0x129b25);
    PROVIDE(HE_caps_update_asoc_cap = 0x129c05);
    PROVIDE(rtw_recv_get_rx_sc_free_entry = 0x129c71);
    PROVIDE(rtw_get_tx_sc_free_entry = 0x129cdd);
    PROVIDE(rtw_set_tx_sc_entry = 0x129d3d);
    PROVIDE(rtw_setkey_to_cam = 0x129e29);
    PROVIDE(rtw_bss_set_key_for_wep = 0x129ff5);
    PROVIDE(rtw_setkey_clear_stakey = 0x12a0bd);
    PROVIDE(rtw_setkey_set_stakey = 0x12a111);
    PROVIDE(rtw_setkey_remove_ptk = 0x12a1ad);
    PROVIDE(rtw_on_beacon_rate_check = 0x12a1b9);
    PROVIDE(rtw_on_beacon_bw_check = 0x12a245);
    PROVIDE(rtw_on_beacon_info_update = 0x12a2f5);
    PROVIDE(rtw_check_mode_info_change = 0x12a4d1);
    PROVIDE(rtw_on_beacon_info_check = 0x12a569);
    PROVIDE(rtw_get_bcn_info = 0x12a6e9);
    PROVIDE(rtw_ap_set_parameters_from_beacon = 0x12a7b5);
    PROVIDE(rtw_ap_update_beacon = 0x12aa41);
    PROVIDE(_rtw_scan_start_scanpriv_init = 0x12aae1);
    PROVIDE(_rtw_scan_cmd_hdl_find_next_chan = 0x12ac29);
    PROVIDE(rtw_scan_report_free_remaining_node = 0x12ac89);
    PROVIDE(rtw_scan_report_node_process = 0x12acf9);
    PROVIDE(rtw_scan_report_node_sort_and_insert = 0x12ad99);
    PROVIDE(_rtw_scan_report_wps = 0x12ae31);
    PROVIDE(_rtw_scan_collect_bss_info_ieproc = 0x12af29);
    PROVIDE(_rtw_scan_check_security_and_rsnie = 0x12b105);
    PROVIDE(rtw_ap_enqueue_for_sleeping_sta = 0x12b3ad);
    PROVIDE(rtw_ap_sleeping_q_free = 0x12b3c5);
    PROVIDE(rtw_ap_sleeping_q_to_pending_q = 0x12b409);
    PROVIDE(rtw_ap_stop_sta_xmit = 0x12b54d);
    PROVIDE(rtw_ap_wakeup_sta_to_xmit = 0x12b5d1);
    PROVIDE(rtw_on_addba_req = 0x12b6ed);
    PROVIDE(rtw_issue_action_ba = 0x12b7a9);
    PROVIDE(rtw_add_ba_hdl = 0x12ba01);
    PROVIDE(rtw_send_delba = 0x12ba91);
    PROVIDE(rtw_addbareq_cmd = 0x12bb6d);
    PROVIDE(rtw_issue_addbareq_cmd = 0x12bbc1);
    PROVIDE(rtw_addba_timer_hdl = 0x12bc71);
    PROVIDE(rtw_dot11w_issue_action_sa_query = 0x12bcbd);
    PROVIDE(rtw_dot11w_on_action_sa_query = 0x12bd6d);
    PROVIDE(rtw_dot11w_BIP_verify = 0x12be11);
    PROVIDE(rtw_dot11w_validate_mgmt = 0x12be51);
    PROVIDE(rtw_issue_deauth = 0x12c1dd);
    PROVIDE(rtw_issue_auth = 0x12c285);
    PROVIDE(_rtw_ap_on_auth = 0x12c555);
    PROVIDE(_rtw_sta_on_auth = 0x12c989);
    PROVIDE(_rtw_on_deauth_frameproc = 0x12ca79);
    PROVIDE(rtw_addba_rsp_hdl = 0x12ccbd);
    PROVIDE(rtw_addba_delba_hdl = 0x12cd41);
    PROVIDE(_rtw_joinbss_set_uapsd_default_setting = 0x12ce11);
    PROVIDE(_rtw_joinbss_ie_restruct = 0x12ce71);
    PROVIDE(_rtw_joinbss_cmd = 0x12d035);
    PROVIDE(rtw_joinbss_target_network_update = 0x12d1c9);
    PROVIDE(rtw_bss_get_password_for_wep = 0x12d311);
    PROVIDE(rtw_joinbss_set_chanmap = 0x12d439);
    PROVIDE(_rtw_leavebss_event_indicate = 0x12d4c9);
    PROVIDE(_rtw_joinbss_cmd_hdl_ieproc = 0x12d551);
    PROVIDE(_rtw_joinbss_timeout_proc = 0x12d619);
    PROVIDE(rtw_joinbss_in_target_network = 0x12d761);
    PROVIDE(_rtw_joinbss_start_proc = 0x12d835);
    PROVIDE(rtw_joinbss_chk_done_concurrent_check_chan = 0x12db15);
    PROVIDE(rtw_joinbss_chk_done_concurrent_check_5g = 0x12dd4d);
    PROVIDE(_rtw_sta_issue_assocreq_validcheck = 0x12ded5);
    PROVIDE(_rtw_sta_issue_assocreq_htproc = 0x12df85);
    PROVIDE(_rtw_sta_issue_assocreq_frameproc = 0x12e071);
    PROVIDE(_rtw_ap_issue_assocrsp = 0x12e4ad);
    PROVIDE(_rtw_sta_on_assocrsp_frameproc = 0x12e5e9);
    PROVIDE(_rtw_ap_on_disassoc = 0x12e8fd);
    PROVIDE(_rtw_sta_on_disassoc = 0x12e9a1);
    PROVIDE(rtw_assoc_resources_free = 0x12eae1);
    PROVIDE(rtw_sta_on_assocrsp_wmm = 0x12ebbd);
    PROVIDE(_rtw_ap_on_assocreq_frameproc = 0x12ec19);
    PROVIDE(_rtw_ap_on_assocreq_getaid_and_setcam = 0x12eed1);
    PROVIDE(_rtw_ap_on_assocreq_asoc_list = 0x12ef5d);
    PROVIDE(rtw_security_method_get_rsn_ie_for_11w = 0x12eff5);
    PROVIDE(rtw_security_method_set_from_ie_for_joinbss = 0x12f191);
    PROVIDE(rtw_security_method_get_from_ie_for_joinbss = 0x12f405);
    PROVIDE(rtw_security_method_set_from_usr_for_joinbss = 0x12f7d5);
    PROVIDE(rtw_security_method_get_for_scan = 0x12fa09);
    PROVIDE(rtw_security_method_set_from_ie_for_ap = 0x12fbd9);
    PROVIDE(rtw_security_method_set_from_usr_for_ap = 0x12fcb1);
    PROVIDE(rtw_vht_use_default_setting = 0x12feb5);
    PROVIDE(rtw_process_vht_op_mode_notify = 0x12ff19);
    PROVIDE(rtw_build_vht_op_mode_notify_ie = 0x12ffa5);
    PROVIDE(rtw_build_vht_cap_ie = 0x12fffd);
    PROVIDE(rtw_restructure_vht_ie = 0x130125);
    PROVIDE(rtw_xmit_priv_free = 0x1301d9);
    PROVIDE(rtw_xmit_open_pktfile = 0x130239);
    PROVIDE(rtw_xmit_read_pktfile = 0x13024d);
    PROVIDE(rtw_xmit_update_attrib_qos_acm = 0x13027d);
    PROVIDE(rtw_xmit_update_attrib_set_qos = 0x1302dd);
    PROVIDE(rtw_xmit_pending_queue_check_all = 0x130391);
    PROVIDE(rtw_xmit_pending_queue_check = 0x1303ad);
    PROVIDE(rtw_xmit_put_snap = 0x1303b5);
    PROVIDE(rtw_xmitframe_alloc = 0x1303e9);
    PROVIDE(rtw_xmitframe_free = 0x13046d);
    PROVIDE(rtw_xmit_priv_init = 0x1304e5);
    PROVIDE(rtw_xmit_update_attrib_phy_info = 0x130579);
    PROVIDE(rtw_xmit_update_attrib_sec_info = 0x13064d);
    PROVIDE(rtw_xmit_enc_tkip_addmic = 0x130791);
    PROVIDE(rtw_xmit_update_attrib_vcs_info = 0x13088d);
    PROVIDE(rtw_xmit_count_stats = 0x1308e1);
    PROVIDE(rtw_xmit_mgntframe = 0x130961);
    PROVIDE(rtw_xmit_nulldata = 0x130a1d);
    PROVIDE(rtw_xmit_qos_nulldata = 0x130aed);
    PROVIDE(rtw_xmit_cts2self = 0x130c19);
    PROVIDE(rtw_xmit_enc_software = 0x130cbd);
    PROVIDE(rtw_xmit_make_wlanhdr = 0x130df1);
    PROVIDE(rtw_xmit_update_attrib_raw = 0x131095);
    PROVIDE(rtw_recv_init_priv = 0x131471);
    PROVIDE(rtw_recv_free_priv = 0x13151d);
    PROVIDE(rtw_recv_get_recvframe_data = 0x13154d);
    PROVIDE(rtw_recv_recvframe_pull_tail = 0x131555);
    PROVIDE(rtw_recv_recvframe_pull = 0x131575);
    PROVIDE(rtw_recv_recvframe_put = 0x131595);
    PROVIDE(rtw_recv_free_frame = 0x1315b5);
    PROVIDE(rtw_recv_alloc_frame = 0x131699);
    PROVIDE(rtw_recv_free_frame_queue = 0x1316f9);
    PROVIDE(rtw_recv_frame_chkmic = 0x13171d);
    PROVIDE(rtw_recv_frame_chkpn = 0x131819);
    PROVIDE(rtw_recv_count_rx_stats = 0x131899);
    PROVIDE(rtw_recv_sta2sta_data_frame = 0x1318f5);
    PROVIDE(rtw_recv_ap2sta_data_frame = 0x131a29);
    PROVIDE(rtw_recv_sta2ap_data_frame = 0x131b7d);
    PROVIDE(rtw_recv_validate_ctrl_frame = 0x131c3d);
    PROVIDE(rtw_recv_validate_data_frame = 0x131d0d);
    PROVIDE(rtw_recv_decryptor = 0x131fb5);
    PROVIDE(rtw_recv_validate_frame = 0x132195);
    PROVIDE(rtw_recv_func_prehandle = 0x132295);
    PROVIDE(rtw_recv_promisc = 0x1322c1);
    PROVIDE(rtw_recv_reorder_timer_hdl = 0x132385);
    PROVIDE(rtw_recv_reorder_dequeue = 0x1323dd);
    PROVIDE(rtw_recv_reorder = 0x132455);
    PROVIDE(rtw_recv_reorder_cmd_hdl = 0x13252d);
    PROVIDE(rtw_recv_reorder_alloc = 0x1325b5);
    PROVIDE(rtw_recv_reorder_free = 0x13261d);
    PROVIDE(rtw_mgntframe_alloc = 0x1326b9);
    PROVIDE(rtw_mgntframe_attrib_update = 0x132701);
    PROVIDE(rtw_state_set_ap_client_auth_ongoing = 0x132749);
    PROVIDE(rtw_state_set_ap_client_auth_success = 0x132751);
    PROVIDE(rtw_state_set_ap_client_assoc_success = 0x132759);
    PROVIDE(rtw_state_reset_ap_client_state = 0x132761);
    PROVIDE(rtw_state_check_ap_client_auth_ongoing = 0x132769);
    PROVIDE(rtw_state_check_ap_client_auth_success = 0x132779);
    PROVIDE(rtw_state_check_ap_client_assoc_success = 0x132789);
    PROVIDE(rtw_check_sta_associated_to_ap = 0x132799);
    PROVIDE(rtw_check_sta_linking = 0x1327b1);
    PROVIDE(rtw_state_set_join_prepare = 0x1327d1);
    PROVIDE(rtw_state_set_join_doing = 0x1327e9);
    PROVIDE(rtw_state_set_auth_doing = 0x132801);
    PROVIDE(rtw_state_set_assoc_doing = 0x132821);
    PROVIDE(rtw_state_set_assoc_success = 0x132841);
    PROVIDE(rtw_state_check_sta_no_link = 0x132861);
    PROVIDE(rtw_state_check_sta_prepare_join = 0x13287d);
    PROVIDE(rtw_state_check_sta_prepare_auth = 0x132895);
    PROVIDE(rtw_state_check_sta_auth_doing = 0x1328ad);
    PROVIDE(rtw_state_check_sta_assoc_doing = 0x1328c5);
    PROVIDE(rtw_state_set_sitesurvey_doing = 0x1328dd);
    PROVIDE(rtw_check_under_sitesurvey = 0x1328f1);
    PROVIDE(rtw_state_reset = 0x132901);
    PROVIDE(rtw_sta_reset_stats = 0x132915);
    PROVIDE(rtw_update_stainfo_ppe_g6 = 0x13292d);
    PROVIDE(rtw_ap_update_asoc_cap_g6 = 0x13299d);
    PROVIDE(rtw_sta_update_asoc_cap_g6 = 0x1329f1);
    PROVIDE(rtw_update_sta_info = 0x132a65);
    PROVIDE(rtw_sta_update_rateset = 0x132ba9);
    PROVIDE(rtw_ap_get_assoc_sta_num = 0x132bdd);
    PROVIDE(rtw_ap_init_mlme_info = 0x132bf5);
    PROVIDE(rtw_ap_update_bmc_sta = 0x132c61);
    PROVIDE(rtw_ap_associated_clients_update = 0x132d5d);
    PROVIDE(rtw_ap_sta_info_update = 0x132dc9);
    PROVIDE(rtw_ap_bss_cap_update_on_sta_join = 0x132f75);
    PROVIDE(rtw_ap_bss_cap_update_on_sta_leave = 0x1331b5);
    PROVIDE(rtw_ap_deauth_and_free_sta = 0x133369);
    PROVIDE(rtw_ap_expire_timeout_chk = 0x133451);
    PROVIDE(rtw_ap_deauth_asoc_list = 0x13365d);
    PROVIDE(get_buf_from_poll = 0x1336d9);
    PROVIDE(release_buf_to_poll = 0x133701);
    PROVIDE(init_skb_pool = 0x13371d);
    PROVIDE(deinit_skb_pool = 0x133805);
    PROVIDE(skb_reserve = 0x133831);
    PROVIDE(skb_assign_buf = 0x133841);
    PROVIDE(skb_tail_pointer = 0x13384d);
    PROVIDE(skb_end_pointer = 0x133851);
    PROVIDE(skb_set_tail_pointer = 0x133855);
    PROVIDE(skb_push = 0x13385d);
    PROVIDE(_rom_wifi_hal_debug_dump = 0x133879);
    PROVIDE(rom_wifi_hal_dbg_regu = 0x1339f5);
    PROVIDE(rom_wifi_hal_dbg_os_static = 0x133b11);
    PROVIDE(rom_wifi_hal_dbg_wpa_mode = 0x133be1);
    PROVIDE(rom_wifi_hal_dbg_read_mac_reg = 0x133c3d);
    PROVIDE(wifi_hal_rsvpage_construct_mgnt = 0x133c69);
    PROVIDE(wifi_hal_rsvpage_construct_pspoll = 0x133d15);
    PROVIDE(wifi_hal_rsvpage_construct_null = 0x133d61);
    PROVIDE(wifi_hal_rsvpage_construct_arp_response = 0x133e75);
    PROVIDE(wifi_hal_rsvpage_construct_null_bt = 0x1340ed);
    PROVIDE(wifi_hal_h2c_set_media_status = 0x1341d5);
    PROVIDE(wifi_hal_h2c_keep_alive_cmd = 0x134211);
    PROVIDE(wifi_hal_h2c_wowlan_ctrl_cmd = 0x134241);
    PROVIDE(wifi_hal_h2c_twt_param = 0x13428d);
    PROVIDE(wifi_hal_h2c_down = 0x134375);
    PROVIDE(wifi_hal_h2c_fill_cmd = 0x1343a9);
    PROVIDE(wifi_hal_rsvpage_fill_txff = 0x1344bd);
    PROVIDE(wifi_hal_h2c_set_rsvpage_cmd = 0x1344e9);
    PROVIDE(wifi_hal_clear_amsdu_priv = 0x134509);
    PROVIDE(wifi_hal_rx_amsdu = 0x13452d);
    PROVIDE(wifi_hal_rx_ring_init = 0x134a9d);
    PROVIDE(wifi_hal_rx_ring_free = 0x134c39);
    PROVIDE(wifi_hal_rx_ring_reset = 0x134c6d);
    PROVIDE(wifi_hal_rx_filter = 0x134c7d);
    PROVIDE(wifi_hal_pending_queue_free = 0x134cd1);
    PROVIDE(wifi_hal_polling_tx_done = 0x134d31);
    PROVIDE(rtw_txframes_pending = 0x134d91);
    PROVIDE(wifi_hal_xmitframe_dump = 0x134dad);
    PROVIDE(wifi_hal_raw_xmitframe_coalesce = 0x134ff5);
    PROVIDE(rtw_mgnt_80211w_sw_encrypt = 0x135089);
    PROVIDE(wifi_hal_xmitframe_coalesce = 0x13515d);
    PROVIDE(wifi_hal_tx_ring_init_queue = 0x1354e9);
    PROVIDE(wifi_hal_tx_ring_init = 0x13552d);
    PROVIDE(wifi_hal_tx_ring_free = 0x135569);
    PROVIDE(wifi_hal_tx_ring_reset = 0x1355d1);
    PROVIDE(wifi_hal_txbuf_check = 0x135661);
    PROVIDE(wifi_hal_txbd_enough_check = 0x1356c1);
    PROVIDE(wifi_hal_txbd_enough_check_all = 0x1356e5);
    PROVIDE(ROM_WIFI_AX_BSS_COLOR_SET = 0x135739);
    PROVIDE(ROM_WIFI_AX_SET_ULOFDMA_RESP_NOMINAL_PAD = 0x13576d);
    PROVIDE(ROM_WIFI_AX_MUEDCA_Para = 0x1357a9);
    PROVIDE(ROM_WIFI_AX_MBSSID_Cfg = 0x1357d9);
    PROVIDE(ROM_WIFI_AX_SpecialReuse_Cfg = 0x135841);
    PROVIDE(ROM_WIFI_AX_SET_MAC_AID = 0x135979);
    PROVIDE(ROM_WIFI_AX_GET_MUDECA_PARAM = 0x135999);
    PROVIDE(ROM_WIFI_BCN_FUNC = 0x135a25);
    PROVIDE(ROM_WIFI_BCN_Tx_Ctrl = 0x135a51);
    PROVIDE(ROM_WIFI_BCN_Interval = 0x135a5d);
    PROVIDE(ROM_WIFI_BCN_FUNC_Enable = 0x135a91);
    PROVIDE(ROM_WIFI_BCN_VALID = 0x135aa5);
    PROVIDE(rom_wifi_hal_dbg_mac_reg_dump = 0x135ac9);
    PROVIDE(rom_wifi_hal_dbg_bb_reg_dump = 0x135b59);
    PROVIDE(_rom_wifi_hal_get_rf_addr = 0x135cd9);
    PROVIDE(_rom_wifi_hal_write_rf_reg = 0x135d21);
    PROVIDE(rom_wifi_hal_dbg_rf_reg_dump = 0x135d35);
    PROVIDE(rom_wifi_hal_dbg_read_bb_reg = 0x135da9);
    PROVIDE(rom_wifi_hal_dbg_no_beacon = 0x135dd9);
    PROVIDE(rom_wifi_hal_dbg_rx_cnt = 0x135ed5);
    PROVIDE(rom_wifi_hal_dbg_tx_cnt = 0x136129);
    PROVIDE(rom_wifi_hal_dbg_rxff = 0x13628d);
    PROVIDE(rom_wifi_hal_dbg_txff = 0x1362c9);
    PROVIDE(ROM_WIFI_InitLLTTable = 0x136305);
    PROVIDE(ROM_WIFI_INIT_MACADDR_SET = 0x13635d);
    PROVIDE(ROM_WIFI_INIT_BeaconParameters = 0x136399);
    PROVIDE(ROM_WIFI_INIT_ReadChipVersion = 0x1363c9);
    PROVIDE(ROM_WIFI_Init_RESP_RATE = 0x1363dd);
    PROVIDE(ROM_WIFI_InitTxBufferBoundary = 0x136419);
    PROVIDE(ROM_WIFI_InitNormalChipRegPriority = 0x136449);
    PROVIDE(ROM_WIFI_InitPageBoundary = 0x1364a1);
    PROVIDE(ROM_WIFI_InitNetworkType = 0x1364ad);
    PROVIDE(ROM_WIFI_InitRCR = 0x1364e5);
    PROVIDE(ROM_WIFI_InitAdaptiveCtrl = 0x136525);
    PROVIDE(ROM_WIFI_InitSIFS = 0x13656d);
    PROVIDE(ROM_WIFI_InitEDCA = 0x1365bd);
    PROVIDE(ROM_WIFI_InitRateFallback = 0x1365d5);
    PROVIDE(ROM_WIFI_InitRetryFunction = 0x1365ed);
    PROVIDE(ROM_WIFI_InitBurstPktLen = 0x136615);
    PROVIDE(ROM_WIFI_InitMacClk = 0x136621);
    PROVIDE(ROM_WIFI_Init32kClk = 0x1366dd);
    PROVIDE(ROM_WIFI_InitLxDma = 0x1366f9);
    PROVIDE(ROM_WIFI_InitQueueReservedPage = 0x13682d);
    PROVIDE(ROM_WIFI_ENABLE_BB_RF = 0x136859);
    PROVIDE(ROM_WIFI_DISABLE_MAC_CG = 0x136871);
    PROVIDE(rtw_read8 = 0x1368c1);
    PROVIDE(rtw_read16 = 0x1368c9);
    PROVIDE(rtw_read32 = 0x1368d5);
    PROVIDE(rtw_write8 = 0x1368dd);
    PROVIDE(rtw_write16 = 0x1368e9);
    PROVIDE(rtw_write32 = 0x1368f5);
    PROVIDE(ROM_WIFI_DISCONNECT = 0x136901);
    PROVIDE(ROM_WIFI_BSSID_SET = 0x136931);
    PROVIDE(ROM_WIFI_SetMSR = 0x13696d);
    PROVIDE(ROM_WIFI_ACM_CTRL = 0x136991);
    PROVIDE(ROM_WIFI_PHY_STATUS_CFG = 0x1369c1);
    PROVIDE(ROM_WIFI_SET_APMODE = 0x1369dd);
    PROVIDE(ROM_WIFI_UPDATE_Slottime = 0x136b55);
    PROVIDE(ROM_WIFI_Set_TXOP_Thres = 0x136bd5);
    PROVIDE(ROM_WIFI_SET_CCA = 0x136c49);
    PROVIDE(ROM_WIFI_SET_EDCCA = 0x136c4d);
    PROVIDE(ROM_WIFI_Set_AC_Param = 0x136c51);
    PROVIDE(ROM_WIFI_Set_MLME_JOIN = 0x136c89);
    PROVIDE(ROM_WIFI_INIT_ADDR_CAM = 0x136dc1);
    PROVIDE(ROM_WIFI_ADDR_CAM_ENTRY_VALID = 0x136df9);
    PROVIDE(ROM_WIFI_Set_MLME_Sitesurvey = 0x136e29);
    PROVIDE(ROM_WIFI_PORT_EN_CTRL = 0x136f5d);
    PROVIDE(ROM_WIFI_32K_Cmd = 0x136f89);
    PROVIDE(ROM_WIFI_TX_PAUSE_SET = 0x137055);
    PROVIDE(ROM_WIFI_FIFO_CLEARN_UP = 0x1370d9);
    PROVIDE(ROM_WIFI_CHECK_TXBUF = 0x137171);
    PROVIDE(ROM_WIFI_CONFIG_CSI = 0x1371cd);
    PROVIDE(ROM_WIFI_IMR_UPDATE = 0x137289);
    PROVIDE(ROM_WIFI_ICV_CHECK_CFG = 0x1372f5);
    PROVIDE(ROM_WIFI_CTRL_PKT_RPT = 0x137315);
    PROVIDE(ROM_WIFI_SET_MAX_AGG_NUM = 0x137335);
    PROVIDE(ROM_WIFI_GET_MAX_AGG_NUM = 0x137351);
    PROVIDE(ROM_WIFI_High_Queue_Empty_Check = 0x137361);
    PROVIDE(ROM_WIFI_GET_TXBUF_PKT_NUM = 0x137371);
    PROVIDE(ROM_WIFI_MAC_CCK_CHECK = 0x137375);
    PROVIDE(ROM_WIFI_RX_COND_SET = 0x137399);
    PROVIDE(ROM_WIFI_RX_COND_GET = 0x1373b5);
    PROVIDE(ROM_WIFI_BRate_Get = 0x1373c5);
    PROVIDE(ROM_WIFI_BRate_Set = 0x13759d);
    PROVIDE(ROM_WIFI_MRateToHwRate = 0x1375dd);
    PROVIDE(ROM_WIFI_HwRateToMRate = 0x1376ad);
    PROVIDE(ROM_WIFI_CAM_WRITE = 0x1376f5);
    PROVIDE(ROM_WIFI_CAM_READ = 0x137769);
    PROVIDE(ROM_WIFI_SEC_CFG = 0x137835);
    PROVIDE(ROM_WIFI_CAM_WriteSppAmsduValid = 0x137851);
    PROVIDE(ROM_WIFI_CAM_Entry_Clear = 0x137861);
    PROVIDE(ROM_WIFI_CAM_FLUSH_ALL = 0x137899);
    PROVIDE(ROM_WIFI_TSF_SET = 0x1378e5);
    PROVIDE(ROM_WIFI_TSF_TIMER0_SOURCE_CFG = 0x137985);
    PROVIDE(ROM_WIFI_TSF_FREERUN_CFG = 0x137999);
    PROVIDE(ROM_WIFI_TSF_GetByPort = 0x1379b9);
    PROVIDE(ROM_WIFI_TSF_GetFreeRun = 0x137a05);
    PROVIDE(ROM_WIFI_TSF_Sync = 0x137a15);
    PROVIDE(ROM_WIFI_TSF_Sync_P1_From_P0 = 0x137a41);
    PROVIDE(ROM_WIFI_TXDESC_BWMapping = 0x137a55);
    PROVIDE(ROM_WIFI_TXDESC_SCMapping = 0x137a7d);
    PROVIDE(ROM_WIFI_TXDESC_FillSectype = 0x137ac1);
    PROVIDE(ROM_WIFI_TXDESC_FillFake = 0x137af1);
    PROVIDE(ROM_WIFI_TXDESC_Get_BufAddr = 0x137be1);
    PROVIDE(ROM_WIFI_TXDESC_FillVCS = 0x137bf5);
    PROVIDE(ROM_WIFI_HAL_RXDESC_Percentage2dbm = 0x137c69);
    PROVIDE(ROM_WIFI_HAL_RXDESC_Power2strength = 0x137c6d);
    PROVIDE(ROM_WIFI_HAL_RXDESC_ProcessSignal = 0x137c89);
    PROVIDE(wifi_hal_rxdesc_query = 0x137cb1);
    PROVIDE(rtw_init_listhead = 0x137df9);
    PROVIDE(rtw_is_list_empty = 0x137e01);
    PROVIDE(rtw_list_insert_head = 0x137e0d);
    PROVIDE(rtw_list_insert_tail = 0x137e19);
    PROVIDE(rtw_list_delete = 0x137e25);
    PROVIDE(rtw_init_queue = 0x137e35);
    PROVIDE(rtw_queue_empty = 0x137e49);
    PROVIDE(rtw_end_of_queue_search = 0x137e55);
    PROVIDE(rtw_recv_get_rx_sc_index = 0x137e61);
    PROVIDE(rtw_recv_indicatepkt_sc = 0x137e91);
    PROVIDE(rtw_recv_check_rx_shortcut_path = 0x138009);
    PROVIDE(rtw_get_tx_sc_index = 0x13819d);
    PROVIDE(rtw_xmit_entry_sc = 0x1381c5);
    PROVIDE(wifi_hal_check_rxtg = 0x1383c5);
    PROVIDE(wifi_hal_check_rxdesc_remain = 0x138429);
    PROVIDE(wifi_hal_hwqueue_get = 0x138455);
    PROVIDE(wifi_hal_tx_ring_skb_enqueue = 0x138469);
    PROVIDE(wifi_hal_tx_ring_skb_dequeue = 0x138481);
    PROVIDE(wifi_hal_pending_q_enqueue = 0x1384c5);
    PROVIDE(wifi_hal_pending_q_resume = 0x138505);
    PROVIDE(wifi_hal_xmitframe_dump_sc_filldesc = 0x1385b1);
    PROVIDE(wifi_hal_txbd_set = 0x138c35);
    PROVIDE(dh_groups = 0x13cb3c);
    PROVIDE(dh_group24_order = 0x13cc9c);
    PROVIDE(dh_group24_prime = 0x13ccbc);
    PROVIDE(dh_group24_generator = 0x13cdbc);
    PROVIDE(dh_group23_order = 0x13cebc);
    PROVIDE(dh_group23_prime = 0x13ced8);
    PROVIDE(dh_group23_generator = 0x13cfd8);
    PROVIDE(dh_group22_order = 0x13d0d8);
    PROVIDE(dh_group22_prime = 0x13d0ec);
    PROVIDE(dh_group22_generator = 0x13d16c);
    PROVIDE(dh_group18_prime = 0x13d5ec);
    PROVIDE(dh_group18_generator = 0x13d9ec);
    PROVIDE(dh_group17_order = 0x13d9f0);
    PROVIDE(dh_group17_prime = 0x13dcf0);
    PROVIDE(dh_group17_generator = 0x13dff0);
    PROVIDE(dh_group16_order = 0x13dff4);
    PROVIDE(dh_group16_prime = 0x13e1f4);
    PROVIDE(dh_group16_generator = 0x13e3f4);
    PROVIDE(dh_group15_prime = 0x13e578);
    PROVIDE(dh_group15_generator = 0x13e6f8);
    PROVIDE(dh_group14_order = 0x13e6fc);
    PROVIDE(dh_group14_prime = 0x13e7fc);
    PROVIDE(dh_group14_generator = 0x13e8fc);
    PROVIDE(dh_group5_order = 0x13e900);
    PROVIDE(dh_group5_prime = 0x13e9c0);
    PROVIDE(dh_group5_generator = 0x13ea80);
    PROVIDE(dh_group2_order = 0x13ea84);
    PROVIDE(dh_group2_prime = 0x13eb04);
    PROVIDE(dh_group2_generator = 0x13eb84);
    PROVIDE(dh_group1_order = 0x13eb88);
    PROVIDE(dh_group1_prime = 0x13ebe8);
    PROVIDE(dh_group1_generator = 0x13ec48);
    PROVIDE(rcons = 0x13f9bc);
    PROVIDE(igtk_expansion_const = 0x13fd90);
    PROVIDE(sae_groups = 0x1402ec);
    PROVIDE(WPS_OUI = 0x140730);
    PROVIDE(WMM_OUI = 0x140734);
    PROVIDE(WMM_PARA_OUI = 0x140738);
    PROVIDE(WMM_INFO_OUI = 0x140740);
    PROVIDE(RTW_WPA_OUI_TYPE = 0x14076c);
    PROVIDE(WPA_CIPHER_SUITE_NONE = 0x140770);
    PROVIDE(WPA_CIPHER_SUITE_WEP40 = 0x140774);
    PROVIDE(WPA_CIPHER_SUITE_TKIP = 0x140778);
    PROVIDE(WPA_CIPHER_SUITE_CCMP = 0x14077c);
    PROVIDE(WPA_CIPHER_SUITE_WEP104 = 0x140780);
    PROVIDE(RSN_CIPHER_SUITE_NONE = 0x140784);
    PROVIDE(RSN_CIPHER_SUITE_WEP40 = 0x140788);
    PROVIDE(RSN_CIPHER_SUITE_TKIP = 0x14078c);
    PROVIDE(RSN_CIPHER_SUITE_CCMP = 0x140790);
    PROVIDE(RSN_CIPHER_SUITE_WEP104 = 0x140794);
    PROVIDE(RSN_AUTH_KEY_MGMT_PSK_OVER_802_1X = 0x1407b4);
    PROVIDE(RSN_AUTH_KEY_MGMT_UNSPEC_802_1X = 0x1407b8);
    PROVIDE(RSN_VERSION_BSD = 0x1407bc);
    PROVIDE(RTW_ChannelPlanMap = 0x1407c0);
    PROVIDE(RTW_ChannelPlanBitmap2G = 0x1409c0);
    PROVIDE(RTW_ChannelPlanBitmap5G = 0x1409f8);
    PROVIDE(RTW_WPA_OUI = 0x140fe0);
    PROVIDE(RSN_TKIP_CIPHER = 0x140fe4);
    PROVIDE(RSN_AUTH_KEY_MGMT_FT_802_1X = 0x140fe8);
    PROVIDE(RSN_AUTH_KEY_MGMT_FT_PSK = 0x140fec);
    PROVIDE(RSN_AUTH_KEY_MGMT_802_1X_SHA256 = 0x140ff0);
    PROVIDE(RSN_AUTH_KEY_MGMT_PSK_SHA256 = 0x140ff4);
    PROVIDE(RSN_AUTH_KEY_MGMT_SAE = 0x140ff8);
    PROVIDE(RSN_AUTH_KEY_MGMT_FT_SAE = 0x140ffc);
    PROVIDE(RSN_AUTH_KEY_MGMT_802_1X_SUITE_B = 0x141000);
    PROVIDE(RSN_AUTH_KEY_MGMT_802_1X_SUITE_B_192 = 0x141004);
    PROVIDE(RSN_AUTH_KEY_MGMT_DPP = 0x141008);
    PROVIDE(RSN_AUTH_KEY_MGMT_OWE = 0x14100c);
    PROVIDE(RSN_CIPHER_SUITE_AES_128_CMAC = 0x141010);
    PROVIDE(RSN_CIPHER_SUITE_BIP_GMAC_128 = 0x141014);
    PROVIDE(RSN_CIPHER_SUITE_BIP_GMAC_256 = 0x141018);
    PROVIDE(RSN_CIPHER_SUITE_BIP_CMAC_256 = 0x14101c);
    PROVIDE(rtw_basic_rate_mix = 0x141044);
    PROVIDE(center_chs_2g_by_bw = 0x14104c);
    PROVIDE(op_chs_of_cch_2g_by_bw = 0x14105c);
    PROVIDE(center_chs_5g_by_bw = 0x14106c);
    PROVIDE(op_chs_of_cch_5g_by_bw = 0x14108c);
    PROVIDE(_ch_width_to_bw_cap = 0x1410ac);
    PROVIDE(op_chs_of_cch_5g_160m = 0x1410b4);
    PROVIDE(center_ch_5g_160m = 0x1410cc);
    PROVIDE(op_chs_of_cch_5g_80m = 0x1410d0);
    PROVIDE(center_ch_5g_80m = 0x1410ec);
    PROVIDE(op_chs_of_cch_5g_40m = 0x1410f4);
    PROVIDE(center_ch_5g_20m_40m = 0x141110);
    PROVIDE(center_ch_5g_40m = 0x14113c);
    PROVIDE(center_ch_5g_20m = 0x14114c);
    PROVIDE(center_ch_5g_all = 0x141168);
    PROVIDE(op_chs_of_cch_2g_40m = 0x14119c);
    PROVIDE(center_ch_2g_40m = 0x1411b0);
    PROVIDE(center_ch_2g = 0x1411bc);
    PROVIDE(default_rateset = 0x141800);
    PROVIDE(VHT_MCS_DATA_RATE_40M = 0x141888);
    PROVIDE(VHT_MCS_DATA_RATE_20M = 0x1418c4);
    PROVIDE(rtw_bridge_tunnel_header = 0x141a90);
    PROVIDE(rtw_rfc1042_header = 0x141a98);
    PROVIDE(oui_rfc1042 = 0x141aa0);
    PROVIDE(oui_8021h = 0x141aa4);
    PROVIDE(SNAP_HDR_APPLETALK_DDP = 0x141aa8);
    PROVIDE(SNAP_ETH_TYPE_TDLS = 0x141aac);
    PROVIDE(SNAP_ETH_TYPE_APPLETALK_DDP = 0x141ab0);
    PROVIDE(SNAP_ETH_TYPE_APPLETALK_AARP = 0x141ab4);
    PROVIDE(SNAP_ETH_TYPE_IPX = 0x141ab8);
    PROVIDE(MCS_rate_1R = 0x142778);
    PROVIDE(address_cam_entry = 0x142790);
    PROVIDE(AA__wifi_share_rom_end__ = 0x1427c0);
    PROVIDE(DelayUs = 0x14780d);
    PROVIDE(DelayMs = 0x14786d);
    PROVIDE(__km4tz_flash_text_start__ = 0x2000020);
    PROVIDE(__km4tz_flash_text_end__ = 0x4000000);
    PROVIDE(__km4ns_flash_text_start__ = 0x4000020);
    PROVIDE(__km4ns_flash_text_end__ = 0x6000000);
    PROVIDE(__km4tz_boot_text_start__ = 0x10400020);
    PROVIDE(__km4tz_boot_text_end__ = 0x10c00000);
    PROVIDE(__km4tz_img3_text_start__ = 0x10c00020);
    PROVIDE(__km4tz_img3_text_end__ = 0x11000000);
    PROVIDE(AA__wifi_share_bss_start__ = 0x200014c0);
    PROVIDE(p_wifi_rom_func_map = 0x200014c4);
    PROVIDE(wifi_rom_func_map = 0x200014c8);
    PROVIDE(sae_debug_level = 0x200014d4);
    PROVIDE(mf_list_head = 0x200014dc);
    PROVIDE(skbpriv = 0x200014e0);
    PROVIDE(wifi_setting = 0x20001508);
    PROVIDE(g_kvrpriv = 0x20001688);
    PROVIDE(g_wpspriv = 0x200016c4);
    PROVIDE(g_promiscpriv = 0x200016ec);
    PROVIDE(g_traffic_statis = 0x200016f8);
    PROVIDE(g_pspriv = 0x20001710);
    PROVIDE(g_apmlmepriv = 0x20001748);
    PROVIDE(g_hal_spec = 0x2000178c);
    PROVIDE(g_scanpriv = 0x200017b8);
    PROVIDE(g_mlmepriv = 0x20001810);
    PROVIDE(g_recvpriv = 0x200019e0);
    PROVIDE(g_xmitpriv = 0x20001a7c);
    PROVIDE(p_wifi_rom2flash = 0x20001b70);
    PROVIDE(wifi_rom2flash = 0x20001b74);
    PROVIDE(wifi_rom_bss_addr_cam_en = 0x20001ce8);
    PROVIDE(AA__wifi_share_bss_end__ = 0x20001cea);
    PROVIDE(__ipc_memory_start__ = 0x20004400);
    PROVIDE(__km4tz_bd_ram_start__ = 0x20006000);
    PROVIDE(__km4tz_bd_psram_start__ = 0x60000000);
    PROVIDE(__non_secure_psram_end__ = 0x60400000);
}
SECTIONS
{
    PROVIDE(AA__os_share_rom_start__ = 0x10f000);
    PROVIDE(xTaskCreateStatic = 0x10f389);
    PROVIDE(xTaskCreate = 0x10f3c5);
    PROVIDE(vTaskDelete = 0x10f425);
    PROVIDE(eTaskGetState = 0x10f4b9);
    PROVIDE(uxTaskPriorityGet = 0x10f521);
    PROVIDE(uxTaskPriorityGetFromISR = 0x10f549);
    PROVIDE(vTaskPrioritySet = 0x10f571);
    PROVIDE(vTaskResume = 0x10f619);
    PROVIDE(vTaskStartScheduler = 0x10f679);
    PROVIDE(vTaskEndScheduler = 0x10f705);
    PROVIDE(vTaskSuspendAll = 0x10f725);
    PROVIDE(xTaskGetTickCount = 0x10f739);
    PROVIDE(xTaskGetTickCountFromISR = 0x10f745);
    PROVIDE(uxTaskGetNumberOfTasks = 0x10f749);
    PROVIDE(pcTaskGetName = 0x10f755);
    PROVIDE(xTaskGetIdleTaskHandle = 0x10f769);
    PROVIDE(vTaskStepTick = 0x10f775);
    PROVIDE(xTaskIncrementTick = 0x10f789);
    PROVIDE(xTaskResumeAll = 0x10f88d);
    PROVIDE(xTaskDelayUntil = 0x10f93d);
    PROVIDE(vTaskDelay = 0x10f991);
    PROVIDE(xTaskCatchUpTicks = 0x10f9fd);
    PROVIDE(vTaskSetApplicationTaskTag = 0x10fa1d);
    PROVIDE(xTaskGetApplicationTaskTag = 0x10fa49);
    PROVIDE(xTaskGetApplicationTaskTagFromISR = 0x10fa75);
    PROVIDE(xTaskCallApplicationTaskHook = 0x10faa1);
    PROVIDE(vTaskSwitchContext = 0x10fac1);
    PROVIDE(vTaskSuspend = 0x10fbbd);
    PROVIDE(vTaskPlaceOnEventList = 0x10fc41);
    PROVIDE(vTaskPlaceOnUnorderedEventList = 0x10fc61);
    PROVIDE(vTaskPlaceOnEventListRestricted = 0x10fc89);
    PROVIDE(xTaskRemoveFromEventList = 0x10fcb5);
    PROVIDE(vTaskRemoveFromUnorderedEventList = 0x10fd1d);
    PROVIDE(vTaskSetTimeOutState = 0x10fd6d);
    PROVIDE(vTaskInternalSetTimeOutState = 0x10fd91);
    PROVIDE(xTaskCheckForTimeOut = 0x10fda5);
    PROVIDE(vTaskMissedYield = 0x10fdf9);
    PROVIDE(uxTaskGetTaskNumber = 0x10fe09);
    PROVIDE(vTaskSetTaskNumber = 0x10fe11);
    PROVIDE(eTaskConfirmSleepModeStatus = 0x10fe19);
    PROVIDE(vTaskSetThreadLocalStoragePointer = 0x10fe4d);
    PROVIDE(pvTaskGetThreadLocalStoragePointer = 0x10fe69);
    PROVIDE(vTaskGetInfo = 0x10fe8d);
    PROVIDE(uxTaskGetSystemState = 0x10ff61);
    PROVIDE(uxTaskGetStackHighWaterMark = 0x110015);
    PROVIDE(xTaskGetCurrentTaskHandle = 0x110031);
    PROVIDE(xTaskGetSchedulerState = 0x11003d);
    PROVIDE(xTaskPriorityInherit = 0x11005d);
    PROVIDE(xTaskPriorityDisinherit = 0x1100d9);
    PROVIDE(vTaskPriorityDisinheritAfterTimeout = 0x110131);
    PROVIDE(vTaskList = 0x110199);
    PROVIDE(vTaskGetRunTimeStats = 0x110235);
    PROVIDE(uxTaskResetEventItemValue = 0x1102dd);
    PROVIDE(pvTaskIncrementMutexHeldCount = 0x1102f5);
    PROVIDE(ulTaskGenericNotifyTake = 0x11030d);
    PROVIDE(xTaskGenericNotifyWait = 0x110381);
    PROVIDE(xTaskGenericNotify = 0x11041d);
    PROVIDE(xTaskGenericNotifyFromISR = 0x1104e1);
    PROVIDE(vTaskGenericNotifyGiveFromISR = 0x1105c5);
    PROVIDE(xTaskGenericNotifyStateClear = 0x110661);
    PROVIDE(ulTaskGenericNotifyValueClear = 0x110695);
    PROVIDE(ulTaskGetIdleRunTimeCounter = 0x1106c9);
    PROVIDE(TaskListDump = 0x1106d9);
    PROVIDE(TaskDumpSystemState = 0x110731);
    PROVIDE(GetTaskName = 0x1108b9);
    PROVIDE(vListInitialise = 0x1108bd);
    PROVIDE(vListInitialiseItem = 0x1108d5);
    PROVIDE(vListInsertEnd = 0x1108dd);
    PROVIDE(vListInsert = 0x1108f5);
    PROVIDE(uxListRemove = 0x110925);
    PROVIDE(xCoRoutineCreate = 0x110a85);
    PROVIDE(vCoRoutineAddToDelayedList = 0x110b09);
    PROVIDE(vCoRoutineSchedule = 0x110b49);
    PROVIDE(xCoRoutineRemoveFromEventList = 0x110bb1);
    PROVIDE(xQueueGenericReset = 0x110d9d);
    PROVIDE(xQueueGenericCreateStatic = 0x110e2d);
    PROVIDE(xQueueGenericCreate = 0x110e51);
    PROVIDE(xQueueGetMutexHolder = 0x110ea9);
    PROVIDE(xQueueGetMutexHolderFromISR = 0x110ec9);
    PROVIDE(xQueueCreateCountingSemaphoreStatic = 0x110ed5);
    PROVIDE(xQueueCreateCountingSemaphore = 0x110ef1);
    PROVIDE(xQueueGenericSend = 0x110f05);
    PROVIDE(xQueueCreateMutex = 0x111039);
    PROVIDE(xQueueCreateMutexStatic = 0x111051);
    PROVIDE(xQueueGiveMutexRecursive = 0x111071);
    PROVIDE(xQueueGenericSendFromISR = 0x11109d);
    PROVIDE(xQueueGiveFromISR = 0x11112d);
    PROVIDE(xQueueReceive = 0x1111a1);
    PROVIDE(xQueueSemaphoreTake = 0x11129d);
    PROVIDE(xQueueTakeMutexRecursive = 0x1113a9);
    PROVIDE(xQueuePeek = 0x1113d5);
    PROVIDE(xQueueReceiveFromISR = 0x1114cd);
    PROVIDE(xQueuePeekFromISR = 0x111531);
    PROVIDE(uxQueueMessagesWaiting = 0x111565);
    PROVIDE(uxQueueSpacesAvailable = 0x111579);
    PROVIDE(uxQueueMessagesWaitingFromISR = 0x111591);
    PROVIDE(vQueueDelete = 0x111595);
    PROVIDE(uxQueueGetQueueNumber = 0x1115b9);
    PROVIDE(vQueueSetQueueNumber = 0x1115bd);
    PROVIDE(ucQueueGetQueueType = 0x1115c1);
    PROVIDE(xQueueIsQueueEmptyFromISR = 0x1115c9);
    PROVIDE(xQueueIsQueueFullFromISR = 0x1115d5);
    PROVIDE(xQueueCRSend = 0x1115e5);
    PROVIDE(xQueueCRReceive = 0x111675);
    PROVIDE(xQueueCRSendFromISR = 0x111725);
    PROVIDE(xQueueCRReceiveFromISR = 0x111759);
    PROVIDE(vQueueWaitForMessageRestricted = 0x1117a9);
    PROVIDE(xQueueCreateSet = 0x1117f9);
    PROVIDE(xQueueAddToSet = 0x111801);
    PROVIDE(xQueueRemoveFromSet = 0x11182d);
    PROVIDE(xQueueSelectFromSet = 0x111859);
    PROVIDE(xQueueSelectFromSetFromISR = 0x111871);
    PROVIDE(QueueCreateTrace = 0x111889);
    PROVIDE(QueueDeleteTrace = 0x1118cd);
    PROVIDE(QueueTraceDump = 0x111901);
    PROVIDE(xTimerCreateTimerTask = 0x111b65);
    PROVIDE(xTimerCreate = 0x111bb9);
    PROVIDE(xTimerCreateStatic = 0x111bf1);
    PROVIDE(xTimerGenericCommand = 0x111c15);
    PROVIDE(xTimerGetTimerDaemonTaskHandle = 0x111f19);
    PROVIDE(xTimerGetPeriod = 0x111f25);
    PROVIDE(vTimerSetReloadMode = 0x111f29);
    PROVIDE(uxTimerGetReloadMode = 0x111f59);
    PROVIDE(xTimerGetExpiryTime = 0x111f71);
    PROVIDE(pcTimerGetName = 0x111f75);
    PROVIDE(xTimerIsTimerActive = 0x111f79);
    PROVIDE(pvTimerGetTimerID = 0x111f91);
    PROVIDE(vTimerSetTimerID = 0x111fa5);
    PROVIDE(xTimerPendFunctionCallFromISR = 0x111fb9);
    PROVIDE(xTimerPendFunctionCall = 0x111fe9);
    PROVIDE(uxTimerGetTimerNumber = 0x112019);
    PROVIDE(vTimerSetTimerNumber = 0x11201d);
    PROVIDE(xEventGroupCreateStatic = 0x112039);
    PROVIDE(xEventGroupCreate = 0x112051);
    PROVIDE(xEventGroupWaitBits = 0x11206d);
    PROVIDE(xEventGroupClearBits = 0x112115);
    PROVIDE(vEventGroupClearBitsCallback = 0x112131);
    PROVIDE(xEventGroupClearBitsFromISR = 0x112135);
    PROVIDE(xEventGroupGetBitsFromISR = 0x112149);
    PROVIDE(xEventGroupSetBits = 0x11215d);
    PROVIDE(xEventGroupSync = 0x1121c9);
    PROVIDE(vEventGroupSetBitsCallback = 0x112261);
    PROVIDE(vEventGroupDelete = 0x112265);
    PROVIDE(xEventGroupSetBitsFromISR = 0x11229d);
    PROVIDE(uxEventGroupGetNumber = 0x1122b9);
    PROVIDE(vEventGroupSetNumber = 0x1122c1);
    PROVIDE(xStreamBufferGenericCreate = 0x112435);
    PROVIDE(xStreamBufferGenericCreateStatic = 0x11247d);
    PROVIDE(vStreamBufferDelete = 0x1124bd);
    PROVIDE(xStreamBufferReset = 0x1124d1);
    PROVIDE(xStreamBufferSetTriggerLevel = 0x112509);
    PROVIDE(xStreamBufferSpacesAvailable = 0x112521);
    PROVIDE(xStreamBufferBytesAvailable = 0x112539);
    PROVIDE(xStreamBufferSend = 0x11253d);
    PROVIDE(xStreamBufferSendFromISR = 0x112619);
    PROVIDE(xStreamBufferReceive = 0x11268d);
    PROVIDE(xStreamBufferNextMessageLengthBytes = 0x112735);
    PROVIDE(xStreamBufferReceiveFromISR = 0x11276d);
    PROVIDE(xStreamBufferIsEmpty = 0x1127d9);
    PROVIDE(xStreamBufferIsFull = 0x1127e9);
    PROVIDE(xStreamBufferSendCompletedFromISR = 0x112801);
    PROVIDE(xStreamBufferReceiveCompletedFromISR = 0x112845);
    PROVIDE(uxStreamBufferGetStreamBufferNumber = 0x112889);
    PROVIDE(vStreamBufferSetStreamBufferNumber = 0x11288d);
    PROVIDE(ucStreamBufferGetStreamBufferType = 0x112891);
    PROVIDE(vPortYield = 0x1128dd);
    PROVIDE(vPortEnterCritical = 0x1128f5);
    PROVIDE(vPortExitCritical = 0x112919);
    PROVIDE(ulPortSetInterruptMask = 0x112935);
    PROVIDE(vPortClearInterruptMask = 0x112945);
    PROVIDE(SysTick_Handler = 0x11294d);
    PROVIDE(vPortSVCHandler_C = 0x112971);
    PROVIDE(pxPortInitialiseStack = 0x1129d1);
    PROVIDE(xPortStartScheduler = 0x112a51);
    PROVIDE(vPortEndScheduler = 0x112a8d);
    PROVIDE(vApplicationIdleHook = 0x112a91);
    PROVIDE(vApplicationStackOverflowHook = 0x112aa1);
    PROVIDE(vApplicationMallocFailedHook = 0x112ad1);
    PROVIDE(vAssertCalled = 0x112b35);
    PROVIDE(vApplicationGetIdleTaskMemory = 0x112b49);
    PROVIDE(vApplicationGetTimerTaskMemory = 0x112b6d);
    PROVIDE(vRestoreContextOfFirstTask = 0x112b91);
    PROVIDE(PendSV_Handler = 0x112bc9);
    PROVIDE(vPortAllocateSecureContext = 0x112c59);
    PROVIDE(vPortFreeSecureContext = 0x112c5d);
    PROVIDE(xIsPrivileged = 0x112c69);
    PROVIDE(vRaisePrivilege = 0x112c81);
    PROVIDE(vResetPrivilege = 0x112c91);
    PROVIDE(vStartFirstTask = 0x112ca1);
    PROVIDE(ulSetInterruptMaskFromISR = 0x112cc5);
    PROVIDE(vClearInterruptMaskFromISR = 0x112cd1);
    PROVIDE(SVC_Handler = 0x112cd9);
    PROVIDE(pvPortMalloc = 0x112d4d);
    PROVIDE(vPortFree = 0x112e11);
    PROVIDE(xPortGetFreeHeapSize = 0x112e69);
    PROVIDE(xPortGetMinimumEverFreeHeapSize = 0x112e75);
    PROVIDE(vPortDefineHeapRegions = 0x112e81);
    PROVIDE(vPortGetHeapStats = 0x112ef9);
    PROVIDE(pvPortReAlloc = 0x112f69);
    PROVIDE(pvPortCalloc = 0x112fd9);
    PROVIDE(SecureContext_Init_rom = 0x112ff9);
    PROVIDE(SecureInit_DePrioritizeNSExceptions_rom = 0x112ffd);
    PROVIDE(SecureContext_AllocateContext_rom = 0x113001);
    PROVIDE(SecureContext_FreeContext_rom = 0x113005);
    PROVIDE(SecureContext_LoadContext_rom = 0x113009);
    PROVIDE(SecureContext_SaveContext_rom = 0x11300d);
    PROVIDE(vPortAllocateSecureContext_rom = 0x113011);
    PROVIDE(vPortCleanUpTCB_rom = 0x113015);
    PROVIDE(traceTaskCreate_rom = 0x113019);
    PROVIDE(traceTaskDelete_rom = 0x11301d);
    PROVIDE(newlib_assign_1_rom = 0x113021);
    PROVIDE(newlib_assign_2_rom = 0x113025);
    PROVIDE(newlib_reent_init_ptr_rom = 0x113029);
    PROVIDE(newlib_reclaim_reent_rom = 0x11302d);
    PROVIDE(CSWTCH.245 = 0x11334c);
    PROVIDE(TaskStateString.0 = 0x113354);
    PROVIDE(uxTopUsedPriority = 0x11336c);
    PROVIDE(QueueTypeString.0 = 0x113568);
    PROVIDE(__FUNCTION__.0 = 0x113618);
    PROVIDE(AA__os_share_rom_end__ = 0x113636);
    PROVIDE(AA__os_share_bss_start__ = 0x20001cee);
    PROVIDE(xSuspendedTaskList = 0x20001cf0);
    PROVIDE(xPendingReadyList = 0x20001d04);
    PROVIDE(uxTopReadyPriority = 0x20001d18);
    PROVIDE(pxCurrentTCB = 0x20001d1c);
    PROVIDE(pxReadyTasksLists = 0x20001d20);
    PROVIDE(xNextTaskUnblockTime = 0x20001dfc);
    PROVIDE(xTickCount = 0x20001e00);
    PROVIDE(pxDelayedTaskList = 0x20001e04);
    PROVIDE(xDelayedTaskList1 = 0x20001e08);
    PROVIDE(xDelayedTaskList2 = 0x20001e1c);
    PROVIDE(xTasksWaitingTermination = 0x20001e30);
    PROVIDE(pxOverflowDelayedTaskList = 0x20001e44);
    PROVIDE(uxCurrentNumberOfTasks = 0x20001e48);
    PROVIDE(xSchedulerRunning = 0x20001e4c);
    PROVIDE(uxTaskNumber = 0x20001e50);
    PROVIDE(uxDeletedTasksWaitingCleanUp = 0x20001e54);
    PROVIDE(xIdleTaskHandle = 0x20001e58);
    PROVIDE(uxSchedulerSuspended = 0x20001e5c);
    PROVIDE(xNumOfOverflows = 0x20001e60);
    PROVIDE(xYieldPending = 0x20001e64);
    PROVIDE(xPendedTicks = 0x20001e68);
    PROVIDE(ulTotalRunTime = 0x20001e6c);
    PROVIDE(ulTaskSwitchedInTime = 0x20001e70);
    PROVIDE(FreeRTOS_errno = 0x20001e74);
    PROVIDE(pxReadyCoRoutineLists = 0x20001e78);
    PROVIDE(xDelayedCoRoutineList1 = 0x20001ea0);
    PROVIDE(xDelayedCoRoutineList2 = 0x20001eb4);
    PROVIDE(xPendingReadyCoRoutineList = 0x20001ec8);
    PROVIDE(pxDelayedCoRoutineList = 0x20001edc);
    PROVIDE(pxOverflowDelayedCoRoutineList = 0x20001ee0);
    PROVIDE(uxTopCoRoutineReadyPriority = 0x20001ee4);
    PROVIDE(xLastTickCount = 0x20001ee8);
    PROVIDE(xPassedTicks = 0x20001eec);
    PROVIDE(xCoRoutineTickCount = 0x20001ef0);
    PROVIDE(pxCurrentCoRoutine = 0x20001ef4);
    PROVIDE(Inited.1 = 0x20001ef8);
    PROVIDE(QueueTraceList = 0x20001efc);
    PROVIDE(pxCurrentTimerList = 0x20001f3c);
    PROVIDE(pxOverflowTimerList = 0x20001f40);
    PROVIDE(xTimerQueue = 0x20001f44);
    PROVIDE(xActiveTimerList1 = 0x20001f48);
    PROVIDE(xActiveTimerList2 = 0x20001f5c);
    PROVIDE(xStaticTimerQueue.0 = 0x20001f70);
    PROVIDE(ucStaticTimerQueueStorage = 0x20001fc4);
    PROVIDE(xTimerTaskHandle = 0x20001fc8);
    PROVIDE(xLastTime.1 = 0x20001fcc);
    PROVIDE(ulCriticalNesting = 0x20001fd0);
    PROVIDE(xSecureContext = 0x20001fd4);
    PROVIDE(xIdleTaskTCB = 0x20001fd8);
    PROVIDE(uxIdleTaskStack = 0x20001fdc);
    PROVIDE(xTimerTaskTCB = 0x20001fe0);
    PROVIDE(uxTimerTaskStack = 0x20001fe4);
    PROVIDE(msp_addr = 0x20001fe8);
    PROVIDE(xStart = 0x20001fec);
    PROVIDE(pxEnd = 0x20001ff4);
    PROVIDE(xBlockAllocatedBit = 0x20001ff8);
    PROVIDE(xFreeBytesRemaining = 0x20001ffc);
    PROVIDE(xMinimumEverFreeBytesRemaining = 0x20002000);
    PROVIDE(xNumberOfSuccessfulAllocations = 0x20002004);
    PROVIDE(xNumberOfSuccessfulFrees = 0x20002008);
    PROVIDE(patch_portEnable_Interrupts = 0x2000200c);
    PROVIDE(patch_portDisable_Interrupts = 0x20002010);
    PROVIDE(patch_newlib_reclaim_reent = 0x20002014);
    PROVIDE(patch_newlib_reent_init_ptr = 0x20002018);
    PROVIDE(patch_newlib_assign_2 = 0x2000201c);
    PROVIDE(patch_newlib_assign_1 = 0x20002020);
    PROVIDE(patch_traceTaskDelete = 0x20002024);
    PROVIDE(patch_traceTaskCreate = 0x20002028);
    PROVIDE(patch_vPortCleanUpTCB = 0x2000202c);
    PROVIDE(patch_vPortAllocateSecureContext = 0x20002030);
    PROVIDE(patch_SecureContext_Init = 0x20002034);
    PROVIDE(patch_SecureContext_SaveContext = 0x20002038);
    PROVIDE(patch_SecureContext_LoadContext = 0x2000203c);
    PROVIDE(patch_SecureInit_DePrioritizeNSExceptions = 0x20002040);
    PROVIDE(patch_SecureContext_FreeContext = 0x20002044);
    PROVIDE(patch_SecureContext_AllocateContext = 0x20002048);
    PROVIDE(os_cfg = 0x2000204c);
    PROVIDE(AA__os_share_bss_end__ = 0x200020a4);
}
