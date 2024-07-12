/**
  ******************************************************************************
  * @file    usb_ch9.h
  * @author  Realsil WLAN5 Team
  * @brief   This file provides general defines for USB SPEC CH9
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "usb_hal.h"
#ifndef CONFIG_FLOADER_USBD_EN
#include "os_wrapper.h"
#endif

/* Private defines -----------------------------------------------------------*/

#ifndef CONFIG_FLOADER_USBD_EN
USB_DATA_SECTION
static const char *const TAG = "USB";
#endif

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  memory set
  * @param  buf: buffer
  * @param  val: value
  * @param  size: size in byte
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_memset(void *buf, u8 val, u32 size)
{
	memset(buf, val, size);
}

/**
  * @brief  memory copy
  * @param  dst: destination buffer
  * @param  src: source buffer
  * @param  size: size in byte
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_memcpy(void *dst, const void *src, u32 size)
{
	memcpy(dst, src, size);
}

/**
  * @brief  Delay us, will hold CPU
  * @param  us: time in us
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_delay_us(u32 us)
{
#ifdef CONFIG_FLOADER_USBD_EN
	DelayUs(us);
#else
	rtos_time_delay_us(us);
#endif
}

/**
  * @brief  sleep ms, will release CPU
  * @param  ms: time in ms
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_sleep_ms(u32 ms)
{
#ifdef CONFIG_FLOADER_USBD_EN
	DelayMs(ms);
#else
	rtos_time_delay_ms(ms);
#endif
}

#ifndef CONFIG_FLOADER_USBD_EN

/**
  * @brief  malloc for size
  * @param  size: the request memory size
  * @retval the malloc address
  */
USB_TEXT_SECTION
void *usb_os_malloc(u32 size)
{
	void *buf = NULL;
	if (size == 0) {
		return NULL;
	}
	buf = rtos_mem_zmalloc(size);
	if (NULL == buf) {
		RTK_LOGS(TAG, "[USB] Fail to alloc %d mem\n", size);
		return NULL;
	}

	return buf;
}
/**
  * @brief  free memory
  * @param  handle: the memory handle to be freed
  * @retval void
  */
USB_TEXT_SECTION
void usb_os_mfree(void *handle)
{
	if (handle) {
		rtos_mem_free(handle);
	}
}

/**
  * @brief  Allocate lock
  * @param  lock: pointer to the lock handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_lock_create(usb_os_lock_t *lock)
{
	int ret;

	ret = rtos_mutex_create(lock);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_MEM;

	return ret;
}

/**
  * @brief  Delete lock
  * @param  lock: lock handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_lock_delete(usb_os_lock_t lock)
{
	int ret;

	ret = rtos_mutex_delete(lock);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
  * @brief  Get lock
  * @param  lock: lock handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_lock(usb_os_lock_t lock)
{
	int ret;

	ret = rtos_mutex_take(lock, MUTEX_WAIT_TIMEOUT);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
  * @brief  Put lock
  * @param  lock: lock handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_unlock(usb_os_lock_t lock)
{
	int ret;

	ret = rtos_mutex_give(lock);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
  * @brief  Disable USB interrupt and get lock
  * @param  lock: lock handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_lock_safe(usb_os_lock_t lock)
{
	usb_hal_disable_interrupt();
	return usb_os_lock(lock);
}

/**
  * @brief  Put lock and enable USB interrupt
  * @param  lock: lock handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_unlock_safe(usb_os_lock_t lock)
{
	int ret;

	ret = usb_os_unlock(lock);
	usb_hal_enable_interrupt();

	return ret;
}

/**
  * @brief  Create sema
  * @param  sema: pointer to the sema handle
  * @retval status
  */
USB_TEXT_SECTION
int usb_os_sema_create(usb_os_sema_t *sema)
{
	int ret;

	ret = rtos_sema_create(sema, 0U, 1U);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_MEM;

	return ret;
}

/**
 * @brief  Delete sema
 * @param  sema: sema handle
 * @retval status
 */
int usb_os_sema_delete(usb_os_sema_t sema)
{
	int ret;

	ret = rtos_sema_delete(sema);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_MEM;

	return ret;
}

/**
 * @brief  Take sema
 * @param  sema: sema handle
 * @param  timeout_ms: timeout in ms
 * @retval status
 */
int usb_os_sema_take(usb_os_sema_t sema, u32 timeout_ms)
{
	int ret;

	ret = rtos_sema_take(sema, timeout_ms);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_TIMEOUT;

	return ret;
}

/**
 * @brief  Give sema
 * @param  sema: sema handle
 * @retval status
 */
int usb_os_sema_give(usb_os_sema_t sema)
{
	int ret;

	ret = rtos_sema_give(sema);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
 * @brief  Create queue
 * @param  queue: pointer to the queue handle
 * @param  msg_num: message number
 * @param  msg_size: message size
 * @retval status
 */
int usb_os_queue_create(usb_os_queue_t *queue, u32 msg_num, u32 msg_size)
{
	int ret;

	ret = rtos_queue_create(queue, msg_num, msg_size);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_MEM;

	return ret;
}

/**
 * @brief  Delete queue
 * @param  queue: queue handle
 * @retval status
 */
int usb_os_queue_delete(usb_os_queue_t queue)
{
	int ret;

	ret = rtos_queue_delete(queue);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
 * @brief  Send message to queue
 * @param  queue: queue handle
 * @param  msg: message buffer
 * @param  wait_ms: wait time in ms
 * @retval status
 */
int usb_os_queue_send(usb_os_queue_t queue, void *msg, u32 wait_ms)
{
	int ret;

	ret = rtos_queue_send(queue, msg, wait_ms);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
 * @brief  Receive message from queue
 * @param  queue: queue handle
 * @param  msg: message buffer
 * @param  wait_ms: wait time in ms
 * @retval status
 */
int usb_os_queue_receive(usb_os_queue_t queue, void *msg, u32 wait_ms)
{
	int ret;

	ret = rtos_queue_receive(queue, msg, wait_ms);
	ret = (ret == SUCCESS) ? HAL_OK : HAL_ERR_PARA;

	return ret;
}

/**
 * @brief  Get free heap size
 * @param  void
 * @retval Free heap size
 */
u32 usb_os_get_free_heap_size(void)
{
	return rtos_mem_get_free_heap_size();
}

#endif

