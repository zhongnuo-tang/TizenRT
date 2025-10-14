/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <tinyara/config.h>
#include <tinyara/mm/heap_regioninfo.h>
/* Scheduler includes. */
#ifndef CONFIG_PLATFORM_TIZENRT_OS
#include "FreeRTOS.h"
#include "task.h"     /* RTOS task related API prototypes. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#endif

#include "ameba_soc.h"

#ifndef CONFIG_PLATFORM_TIZENRT_OS
extern int main(void);
extern void arm_gic_set_CUTVersion(uint32_t CUTVersion);
#endif

static const char *TAG = "APP";
static u32 g_mpu_nregion_allocated;
extern int main(void);
extern void NS_ENTRY BOOT_IMG3(void);
extern void SOCPS_WakeFromPG_AP(void);

void SOCPS_WakeFromPG_AP_Stub(void) {
    // Minimal wake-up handling
    // Or just return if you're not using sleep modes
}
u32 app_mpu_nocache_check(u32 mem_addr)
{
	mpu_region_config mpu_cfg;


	mpu_cfg.region_base = (uint32_t)__ram_nocache_start__;
	mpu_cfg.region_size = __ram_nocache_end__ - __ram_nocache_start__;

	if ((mem_addr >= mpu_cfg.region_base) && (mem_addr < (mpu_cfg.region_base + mpu_cfg.region_size))) {
		return TRUE;
	} else {
		return FALSE;
	}
}

/*AP have 8 secure mpu entrys & 8 nonsecure mpu entrys*/
u32 app_mpu_nocache_init(void)
{
	mpu_region_config mpu_cfg;
	u32 mpu_entry = 0;

	/* ROM Code inside CPU does not enter Cache, Set to RO for NULL ptr access error */
	/* TCM Cache (0x000F0000, 0x00100000) is to be used as RAM in fullmac mode and it can't be read-only. */
	mpu_entry = mpu_entry_alloc();
	mpu_cfg.region_base = 0;
	mpu_cfg.region_size = 0x000F0000;
	mpu_cfg.xn = MPU_EXEC_ALLOW;
	mpu_cfg.ap = MPU_UN_PRIV_RO;
	mpu_cfg.sh = MPU_NON_SHAREABLE;
	mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
	mpu_region_cfg(mpu_entry, &mpu_cfg);

	/* Currently, open share rom cache for better throughput and fix issue https://jira.realtek.com/browse/RSWLANDIOT-11327 */
	/* close share rom cache. Delay rom section should be cacheable for accurate
	   delay so it is not included. */
	// mpu_entry = mpu_entry_alloc();
	// mpu_cfg.region_base = 0x00100000;
	// mpu_cfg.region_size = 0x00147800 - 0x00100000;
	// mpu_cfg.xn = MPU_EXEC_ALLOW;
	// mpu_cfg.ap = MPU_UN_PRIV_RO;
	// mpu_cfg.sh = MPU_NON_SHAREABLE;
	// mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
	// mpu_region_cfg(mpu_entry, &mpu_cfg);

	/* set nocache region */
	mpu_entry = mpu_entry_alloc();
	mpu_cfg.region_base = (uint32_t)__ram_nocache_start__;
	mpu_cfg.region_size = __ram_nocache_end__ - __ram_nocache_start__;
	mpu_cfg.xn = MPU_EXEC_ALLOW;
	mpu_cfg.ap = MPU_UN_PRIV_RW;
	mpu_cfg.sh = MPU_NON_SHAREABLE;
	mpu_cfg.attr_idx = MPU_MEM_ATTR_IDX_NC;
	if (mpu_cfg.region_size >= 32) {
		mpu_region_cfg(mpu_entry, &mpu_cfg);
	}
	g_mpu_nregion_allocated = mpu_entry + 1;
	return 0;
}

#ifndef CONFIG_WIFI_HOST_CONTROL
#if defined (__GNUC__)
/* Add This for C++ support to avoid compile error */
void _init(void) {}
#endif
#endif

SRAM_ONLY_DATA_SECTION
HAL_VECTOR_FUN RamVectorTable[95] ALIGNMTO(512) = {0};
extern unsigned int _sidle_stack;
extern unsigned int _sint_heap;
extern unsigned int _sext_heap;
extern unsigned int __StackLimit;
extern unsigned int __PsramStackLimit;

#define IDLE_STACK ((uintptr_t)&_sidle_stack + CONFIG_IDLETHREAD_STACKSIZE - 4)
#define HEAP_BASE  ((uintptr_t)&_sint_heap)
#define HEAP_LIMIT ((uintptr_t)&__StackLimit)
#define PSRAM_HEAP_BASE ((uintptr_t)&_sext_heap)
#define PSRAM_HEAP_LIMIT ((uintptr_t)&__PsramStackLimit)

const uintptr_t g_idle_topstack = IDLE_STACK;

void os_heap_init(void){
	kregionx_start[0] = (void *)HEAP_BASE;
	kregionx_size[0] = (size_t)(HEAP_LIMIT - HEAP_BASE);
#if CONFIG_KMM_REGIONS >= 2
#if CONFIG_KMM_REGIONS == 3
	kregionx_start[1] = (void *)PSRAM_HEAP_BASE;
	kregionx_size[1] = (size_t)kregionx_start[1] + kregionx_size[1] - PSRAM_HEAP_BASE;

	kregionx_start[2] = (void *)kregionx_start[2];
	kregionx_size[2] = (size_t)PSRAM_HEAP_LIMIT - (size_t)kregionx_start[2];
#elif CONFIG_KMM_REGIONS > 3
#error "Need to check here for heap."
#else
	kregionx_start[1] = (void *)PSRAM_HEAP_BASE;
	kregionx_size[1] = (size_t)(PSRAM_HEAP_LIMIT - PSRAM_HEAP_BASE);
#endif
#endif
}
// The Main App entry point
void app_start(void)
{
#if !defined(CONFIG_WIFI_HOST_CONTROL) || defined(CONFIG_FULLMAC_NEW_VERSION) // When fullmac support XIP, need enable Cache and cannot share cache to TCM
	/* enable non-secure cache */
	Cache_Enable(ENABLE);
#endif
	lldbg("%d %x\n",__LINE__,up_getsp());
	
	/* Rom Bss NS Initial */
	_memset((void *) __rom_bss_start_ns__, 0, (__rom_bss_end_ns__ - __rom_bss_start_ns__));
	/* Image2 Bss Initial */
	_memset((void *) __bss_start__, 0, (__bss_end__ - __bss_start__));

	RBSS_UDELAY_DIV = 5;

	/* When TZ not enabled, re-init pendsv/svcall/systick in the non-secure vector table for OS.*/
	// SCB->VTOR = (u32)RomVectorTable;
	_memset(RamVectorTable, 0, sizeof(RamVectorTable));
	_memcpy(RamVectorTable, RomVectorTable, sizeof(RamVectorTable));
	RamVectorTable[0] = (HAL_VECTOR_FUN)MSP_RAM_HP_NS;
	SCB->VTOR = (u32)RamVectorTable;
	// __set_MSP(MSP_RAM_HP_NS);
	lldbg("%d %x\n",__LINE__,SCB->VTOR);
	lldbg("%d %x\n",__LINE__,up_getsp());
	lldbg("NS MSP: %x\n", __get_MSP());
//#ifdef CONFIG_TRUSTZONE
	BOOT_IMG3();

	cmse_address_info_t cmse_address_info = cmse_TT((void *)app_start);
	RTK_LOGI(TAG, "IMG2 SECURE STATE: %d\n", cmse_address_info.flags.secure);
//#endif
	data_flash_highspeed_setup();
	SystemCoreClockUpdate();
	//RTK_LOGI(TAG, "AP CPU CLK: %lu Hz \n", SystemCoreClock);

	if ((SYSCFG_RLVersion()) >= SYSCFG_CUT_VERSION_B) {
		if (SYSCFG_CHIPType_Get() == CHIP_TYPE_ASIC_POSTSIM) {//Only Asic need OSC Calibration
			OSC4M_Init();
			OSC4M_Calibration(30000);
		}

	}
		// force SP align to 8 byte not 4 byte (initial SP is 4 byte align)
	__asm(
		"mov r0, sp\n"
		"bic r0, r0, #7\n"
		"mov sp, r0\n"
	);
	os_heap_init();
	XTAL_INIT();
	mpu_init();
	app_mpu_nocache_init();
#ifdef CONFIG_PLATFORM_TIZENRT_OS
#ifdef CONFIG_ARMV8M_MPU
	/* Initialize number of mpu regions for board specific purpose */
	mpu_set_nregion_board_specific(g_mpu_nregion_allocated);

	up_mpuinitialize();
#endif
#endif
#ifdef CONFIG_STACK_COLORATION
	/* Set the IDLE stack to the coloration value and jump into os_start() */

	go_os_start((FAR void *)g_idle_topstack - CONFIG_IDLETHREAD_STACKSIZE, CONFIG_IDLETHREAD_STACKSIZE);
#else
	/* Call os_start() */

	os_start();

	/* Shoulnd't get here */

	for (;;) ;
#endif
#ifndef CONFIG_WIFI_HOST_CONTROL
#if defined (__GNUC__)
	//extern void __libc_init_array(void);
	/* Add This for C++ support */
	//__libc_init_array();
#endif
#endif
#ifndef CONFIG_PLATFORM_TIZENRT_OS
	main();
#endif
}
IMAGE2_ENTRY_SECTION
RAM_START_FUNCTION Img2EntryFun0 = {
	app_start,
	SOCPS_WakeFromPG_AP_Stub,
	(u32) RomVectorTable
};

