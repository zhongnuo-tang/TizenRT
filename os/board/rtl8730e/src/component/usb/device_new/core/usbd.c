/**
  ******************************************************************************
  * @file    usbd.c
  * @author  Realsil WLAN5 Team
  * @brief   This file provides the functionalities for USB device library
  ******************************************************************************
  * @attention
  *
  * This module is a confidential and proprietary property of RealTek and
  * possession or use of this module requires written permission of RealTek.
  *
  * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------ */

#include "usbd.h"
#include "usbd_hal.h"
#include "usbd_core.h"
#include "usbd_pcd.h"

/* Private defines -----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/


static const char *const TAG = "USBD";

/* USB device handler */
USB_BSS_SECTION
static usb_dev_t usbd_dev;

/* Private functions ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize the USB device stack
  * @param  cfg: USB device configuration
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_init(usbd_config_t *cfg)
{
	usb_dev_t *dev = &usbd_dev;

	RTK_LOGE(TAG, "[USBD] INIT\n");

	dev->dev_state  = USBD_STATE_DEFAULT;
	dev->ctrl_buf = (u8 *)usb_os_malloc(USB_OTG_HS_MAX_PACKET_SIZE);
	if (dev->ctrl_buf == NULL) {
		return HAL_ERR_MEM;
	}

	/* Init PCD Driver */
	return usbd_pcd_init(dev, cfg);
}

/**
  * @brief  De-Initialize the USB device stack
  * @param  None
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_deinit(void)
{
	int ret = HAL_OK;
	usb_dev_t *dev = &usbd_dev;

	RTK_LOGE(TAG, "[USBD] DEINIT\n");

	dev->dev_state  = USBD_STATE_DEFAULT;

	if (dev->driver != NULL) {
		dev->driver->clear_config(dev, dev->dev_config);
		dev->driver = NULL;
	}

	ret = usbd_pcd_deinit(dev);

	if (dev->ctrl_buf != NULL) {
		usb_os_mfree(dev->ctrl_buf);
		dev->ctrl_buf = NULL;
	}

	return ret;
}

/**
  * @brief  Get USB device attach status
  * @param  void
  * @retval Status: usbd_attach_status_t
  */
USB_TEXT_SECTION
int usbd_get_status(void)
{
	usb_dev_t *dev = &usbd_dev;
	return dev->dev_attach_status;
}

/**
  * @brief  Get the bus status
  * @param  bus_status: physical bus status
  * @retval status
  */
int usbd_get_bus_status(u32 *bus_status)
{
	usb_dev_t *dev = &usbd_dev;
	if (dev->pcd && bus_status) {
		return usbd_hal_get_bus_status(dev->pcd, bus_status);
	} else {
		RTK_LOGE(TAG, "[USBD] Not ready\n");
		return HAL_ERR_PARA;
	}
}
/**
  * @brief  Device send a remote wakeup signal to host
  * @param  None
  * @retval Status
  */
int usbd_wake_host(void)
{
	usb_dev_t *dev = &usbd_dev;
	if (dev->pcd) {
		return usbd_hal_wake_host(dev->pcd);
	} else {
		RTK_LOGE(TAG, "[USBD] Not ready\n");
		return HAL_ERR_PARA;
	}
}

/**
  * @brief  Register USB class
  * @param  driver: class driver
  * @retval None
  */
USB_TEXT_SECTION
int usbd_register_class(usbd_class_driver_t *driver)
{
	usb_dev_t *dev = &usbd_dev;

	dev->driver = driver;

	return usbd_pcd_start((usbd_pcd_t *)dev->pcd);
}

/**
  * @brief  Unregister USB class driver
  * @retval None
  */
USB_TEXT_SECTION
int usbd_unregister_class()
{
	usb_dev_t *dev = &usbd_dev;

	usb_hal_disable_global_interrupt();

	dev->driver = NULL;

	return HAL_OK;
}

/**
  * @brief  Open an endpoint
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  ep_type: Endpoint Type
  * @param  ep_mps: Endpoint Max Packet Size
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_init(usb_dev_t *dev, u8 ep_addr, u8 ep_type, u16 ep_mps)
{
	return usbd_pcd_ep_init(dev->pcd, ep_addr, ep_mps, ep_type);
}

/**
  * @brief  Close an endpoint
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_deinit(usb_dev_t *dev, u8 ep_addr)
{
	return usbd_pcd_ep_deinit(dev->pcd, ep_addr);
}

/**
  * @brief  Transmit data over an endpoint
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  buf: Data buffer
  * @param  len: Data length
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_transmit(usb_dev_t *dev, u8 ep_addr, u8 *buf, u16 len)
{
	return usbd_pcd_ep_transmit(dev->pcd, ep_addr, buf, len);
}

/**
  * @brief  Prepare an endpoint for reception
  * @param  dev: Device handle
  * @param  ep_addr: Endpoint Number
  * @param  buf: Data buffer
  * @param  len: Data length
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_receive(usb_dev_t *dev, u8 ep_addr, u8 *buf, u16 len)
{
	return usbd_pcd_ep_receive(dev->pcd, ep_addr, buf, len);
}

/**
  * @brief  Transmit CTRL IN data
  * @param  dev: USB device instance
  * @param  buf: Data buffer
  * @param  len: Data length
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep0_transmit(usb_dev_t *dev, u8 *buf, u16 len)
{
	/* Set EP0 State */
	dev->ep0_state = USBD_EP0_DATA_IN;
	dev->ep0_xfer_total_len = len;
	dev->ep0_xfer_rem_len = len;

	/* Start the transfer */
	return usbd_ep_transmit(dev, USB_EP0_IN, buf, len);
}

/**
  * @brief  Prepare to receive data from the CTRL pipe
  * @param  dev: USB device instance
  * @param  buf: pointer to data buffer
  * @param  len: length of data to be received
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep0_receive(usb_dev_t *dev, u8 *buf, u16 len)
{
	/* Set EP0 State */
	dev->ep0_state = USBD_EP0_DATA_OUT;
	dev->ep0_recv_rem_len = len;

	/* Start the transfer */
	return usbd_ep_receive(dev, USB_EP0_OUT, buf, len);
}

/**
  * @brief  Send ep0 status
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep0_transmit_status(usb_dev_t *dev)
{
	return usbd_core_ep0_transmit_status(dev);
}

/**
  * @brief  Prepare to receive ep0 status
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep0_receive_status(usb_dev_t *dev)
{
	return usbd_core_ep0_receive_status(dev);
}

/**
  * @brief  Set ep0 stall
  * @param  dev: USB device instance
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep0_set_stall(usb_dev_t *dev)
{
	return usbd_core_ep0_set_stall(dev);
}

/**
  * @brief  Set ep stall
  * @param  dev: USB device instance
  * @param  ep_addr: Endpoint address
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_set_stall(usb_dev_t *dev, u8 ep_addr)
{
	return usbd_core_ep_set_stall(dev, ep_addr);
}

/**
  * @brief  Clear ep stall status
  * @param  dev: USB device instance
  * @param  ep_addr: Endpoint address
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_clear_stall(usb_dev_t *dev, u8 ep_addr)
{
	return usbd_core_ep_clear_stall(dev, ep_addr);
}

/**
  * @brief  Check ep0 stall status
  * @param  dev: USB device instance
  * @param  ep_addr: Endpoint address
  * @retval Status
  */
USB_TEXT_SECTION
int usbd_ep_is_stall(usb_dev_t *dev, u8 ep_addr)
{
	return usbd_core_ep_is_stall(dev, ep_addr);
}

/**
  * @brief  Convert ascii string into unicode string descriptor
  * @param  str : ASCII string
  * @param  desc : Formatted unicode string descriptor
  * @param  len : descriptor length
  * @retval None
  */
USB_TEXT_SECTION
void usbd_get_str_desc(const char *str, u8 *desc, u16 *len)
{
	u8 idx = 0U;
	u8 *p = (u8 *)str;

	if (p != NULL) {
		*len = (u16)(strlen(str) * 2 + 2);
		desc[idx++] = *(u8 *)(void *)len;
		desc[idx++] = USB_DESC_TYPE_STRING;

		while (*p != '\0') {
			desc[idx++] = *p++;
			desc[idx++] =  0U;
		}
	}
}

