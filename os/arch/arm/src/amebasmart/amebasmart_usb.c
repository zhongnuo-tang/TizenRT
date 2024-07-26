/****************************************************************************
 *
 * Copyright 2024 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/arch.h>
#include <tinyara/kmalloc.h>
#include <tinyara/usb/usb.h>
#include <tinyara/usb/usbdev.h>
#include <tinyara/usb/usbdev_trace.h>

#include <arch/irq.h>



#if defined(CONFIG_USBDEV) && defined(CONFIG_AMEBASMART_USB)
#include "usb_os.h"
#include "usbd_pcd.h"
#include "usbd_core.h"
#include "usbd.h"
#include "usbd_hal.h"
#include "../../../../board/rtl8730e/src/component/usb/device_new/cdc_acm/usbd_cdc_acm.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_USBDEV_EP0_MAXSIZE
#define CONFIG_USBDEV_EP0_MAXSIZE 64
#endif

#ifndef CONFIG_USBDEV_SETUP_MAXDATASIZE
#define CONFIG_USBDEV_SETUP_MAXDATASIZE CONFIG_USBDEV_EP0_MAXSIZE
#endif

/* Initial interrupt mask: Reset + Suspend + Correct Transfer */

#define AMEBASMART_CNTR_SETUP     (USB_CNTR_RESETM|USB_CNTR_SUSPM|USB_CNTR_CTRM)

/* Endpoint identifiers. The AMEBASMART supports up to 16 mono-directional or 8
 * bidirectional endpoints.  However, when you take into account PMA buffer
 * usage (see below) and the fact that EP0 is bidirectional, then there is
 * a functional limitation of EP0 + 5 mono-directional endpoints = 6.  We'll
 * define AMEBASMART_NENDPOINTS to be 8, however, because that is how many
 * endpoint register sets there are.
 */

#define AMEBASMART_NENDPOINTS      USB_MAX_ENDPOINTS
#define EP0                   (0)
#define EP1                   (1)
#define EP2                   (2)
#define EP3                   (3)
#define EP4                   (4)
#define EP5                   (5)

#define AMEBASMART_ENDP_BIT(ep)    (1 << (ep))
#define AMEBASMART_ENDP_ALLSET     0xff

/* Packet sizes.  We us a fixed 64 max packet size for all endpoint types */

#define AMEBASMART_MAXPACKET_SHIFT (6)
#define AMEBASMART_MAXPACKET_SIZE  (1 << (AMEBASMART_MAXPACKET_SHIFT))
#define AMEBASMART_MAXPACKET_MASK  (AMEBASMART_MAXPACKET_SIZE-1)

#define AMEBASMART_EP0MAXPACKET    AMEBASMART_MAXPACKET_SIZE

/* Buffer descriptor table.  We assume that USB has exclusive use of CAN/USB
 * memory.  The buffer table is positioned at the beginning of the 512-byte
 * CAN/USB memory.  We will use the first AMEBASMART_NENDPOINTS*4 words for the buffer
 * table.  That is exactly 64 bytes, leaving 7*64 bytes for endpoint buffers.
 */

// #define AMEBASMART_BTABLE_ADDRESS  (0x00)	/* Start at the beginning of USB/CAN RAM */
// #define AMEBASMART_DESC_SIZE       (8)	/* Each descriptor is 4*2=8 bytes in size */
// #define AMEBASMART_BTABLE_SIZE     (AMEBASMART_NENDPOINTS*AMEBASMART_DESC_SIZE)

// /* Buffer layout.  Assume that all buffers are 64-bytes (maxpacketsize), then
//  * we have space for only 7 buffers; endpoint 0 will require two buffers, leaving
//  * 5 for other endpoints.
//  */

// #define AMEBASMART_BUFFER_START    AMEBASMART_BTABLE_SIZE
// #define AMEBASMART_EP0_RXADDR      AMEBASMART_BUFFER_START
// #define AMEBASMART_EP0_TXADDR      (AMEBASMART_EP0_RXADDR+AMEBASMART_EP0MAXPACKET)

// #define AMEBASMART_BUFFER_EP0      0x03
// #define AMEBASMART_NBUFFERS        7
// #define AMEBASMART_BUFFER_BIT(bn)  (1 << (bn))
// #define AMEBASMART_BUFFER_ALLSET   0x7f
// #define AMEBASMART_BUFNO2BUF(bn)   (AMEBASMART_BUFFER_START+((bn)<<AMEBASMART_MAXPACKET_SHIFT))

// /* USB-related masks */

// #define REQRECIPIENT_MASK     (USB_REQ_TYPE_MASK | USB_REQ_RECIPIENT_MASK)

// /* Endpoint register masks (handling toggle fields) */

// #define EPR_NOTOG_MASK        (USB_EPR_CTR_RX  | USB_EPR_SETUP  | USB_EPR_EPTYPE_MASK |\
// 							   USB_EPR_EP_KIND | USB_EPR_CTR_TX | USB_EPR_EA_MASK)
// #define EPR_TXDTOG_MASK       (USB_EPR_STATTX_MASK | EPR_NOTOG_MASK)
// #define EPR_RXDTOG_MASK       (USB_EPR_STATRX_MASK | EPR_NOTOG_MASK)

// /* Request queue operations *************************************************/

// #define amebasmart_rqempty(ep)     ((ep)->head == NULL)
// #define amebasmart_rqpeek(ep)      ((ep)->head)

// /* USB trace ****************************************************************/
// /* Trace error codes */

// #define AMEBASMART_TRACEERR_ALLOCFAIL            0x0001
// #define AMEBASMART_TRACEERR_BADCLEARFEATURE      0x0002
// #define AMEBASMART_TRACEERR_BADDEVGETSTATUS      0x0003
// #define AMEBASMART_TRACEERR_BADEPGETSTATUS       0x0004
// #define AMEBASMART_TRACEERR_BADEPNO              0x0005
// #define AMEBASMART_TRACEERR_BADEPTYPE            0x0006
// #define AMEBASMART_TRACEERR_BADGETCONFIG         0x0007
// #define AMEBASMART_TRACEERR_BADGETSETDESC        0x0008
// #define AMEBASMART_TRACEERR_BADGETSTATUS         0x0009
// #define AMEBASMART_TRACEERR_BADSETADDRESS        0x000a
// #define AMEBASMART_TRACEERR_BADSETCONFIG         0x000b
// #define AMEBASMART_TRACEERR_BADSETFEATURE        0x000c
// #define AMEBASMART_TRACEERR_BINDFAILED           0x000d
// #define AMEBASMART_TRACEERR_DISPATCHSTALL        0x000e
// #define AMEBASMART_TRACEERR_DRIVER               0x000f
// #define AMEBASMART_TRACEERR_DRIVERREGISTERED     0x0010
// #define AMEBASMART_TRACEERR_EP0BADCTR            0x0011
// #define AMEBASMART_TRACEERR_EP0SETUPSTALLED      0x0012
// #define AMEBASMART_TRACEERR_EPBUFFER             0x0013
// #define AMEBASMART_TRACEERR_EPDISABLED           0x0014
// #define AMEBASMART_TRACEERR_EPOUTNULLPACKET      0x0015
// #define AMEBASMART_TRACEERR_EPRESERVE            0x0016
// #define AMEBASMART_TRACEERR_INVALIDCTRLREQ       0x0017
// #define AMEBASMART_TRACEERR_INVALIDPARMS         0x0018
// #define AMEBASMART_TRACEERR_IRQREGISTRATION      0x0019
// #define AMEBASMART_TRACEERR_NOTCONFIGURED        0x001a
// #define AMEBASMART_TRACEERR_REQABORTED           0x001b

// /* Trace interrupt codes */

// #define AMEBASMART_TRACEINTID_CLEARFEATURE       0x0001
// #define AMEBASMART_TRACEINTID_DEVGETSTATUS       0x0002
// #define AMEBASMART_TRACEINTID_DISPATCH           0x0003
// #define AMEBASMART_TRACEINTID_EP0IN              0x0004
// #define AMEBASMART_TRACEINTID_EP0INDONE          0x0005
// #define AMEBASMART_TRACEINTID_EP0OUTDONE         0x0006
// #define AMEBASMART_TRACEINTID_EP0SETUPDONE       0x0007
// #define AMEBASMART_TRACEINTID_EP0SETUPSETADDRESS 0x0008
// #define AMEBASMART_TRACEINTID_EPGETSTATUS        0x0009
// #define AMEBASMART_TRACEINTID_EPINDONE           0x000a
// #define AMEBASMART_TRACEINTID_EPINQEMPTY         0x000b
// #define AMEBASMART_TRACEINTID_EPOUTDONE          0x000c
// #define AMEBASMART_TRACEINTID_EPOUTPENDING       0x000d
// #define AMEBASMART_TRACEINTID_EPOUTQEMPTY        0x000e
// #define AMEBASMART_TRACEINTID_ESOF               0x000f
// #define AMEBASMART_TRACEINTID_GETCONFIG          0x0010
// #define AMEBASMART_TRACEINTID_GETSETDESC         0x0011
// #define AMEBASMART_TRACEINTID_GETSETIF           0x0012
// #define AMEBASMART_TRACEINTID_GETSTATUS          0x0013
// #define AMEBASMART_TRACEINTID_HPINTERRUPT        0x0014
// #define AMEBASMART_TRACEINTID_IFGETSTATUS        0x0015
// #define AMEBASMART_TRACEINTID_LPCTR              0x0016
// #define AMEBASMART_TRACEINTID_LPINTERRUPT        0x0017
// #define AMEBASMART_TRACEINTID_NOSTDREQ           0x0018
// #define AMEBASMART_TRACEINTID_RESET              0x0019
// #define AMEBASMART_TRACEINTID_SETCONFIG          0x001a
// #define AMEBASMART_TRACEINTID_SETFEATURE         0x001b
// #define AMEBASMART_TRACEINTID_SUSP               0x001c
// #define AMEBASMART_TRACEINTID_SYNCHFRAME         0x001d
// #define AMEBASMART_TRACEINTID_WKUP               0x001e
// #define AMEBASMART_TRACEINTID_EP0SETUPOUT        0x001f
// #define AMEBASMART_TRACEINTID_EP0SETUPOUTDATA    0x0020

// /* Ever-present MIN and MAX macros */

// #ifndef MIN
// #define MIN(a, b) (a < b ? a : b)
// #endif

// #ifndef MAX
// #define MAX(a, b) (a > b ? a : b)
// #endif

// /* Byte ordering in host-based values */

// #ifdef CONFIG_ENDIAN_BIG
// #define LSB 1
// #define MSB 0
// #else
// #define LSB 0
// #define MSB 1
// #endif

// /****************************************************************************
//  * Private Type Definitions
//  ****************************************************************************/

// /* The various states of a control pipe */

// enum amebasmart_ep0state_e {
// 	EP0STATE_IDLE = 0,			/* No request in progress */
// 	EP0STATE_SETUP,			/* Set up recived with data for device in progress */
// 	EP0STATE_DATA_IN,
// 	EP0STATE_DATA_OUT,
// 	EP0STATE_STATUS_IN,
// 	EP0STATE_STATUS_OUT,
// 	EP0STATE_STALLED			/* We are stalled */
// };

// /* Resume states */

// enum amebasmart_rsmstate_e {
// 	RSMSTATE_IDLE = 0,			/* Device is either fully suspended or running */
// 	RSMSTATE_STARTED,			/* Resume sequence has been started */
// 	RSMSTATE_WAITING			/* Waiting (on ESOFs) for end of sequence */
// };

// union wb_u {
// 	uint16_t w;
// 	uint8_t b[2];
// };

// /* A container for a request so that the request make be retained in a list */

// struct amebasmart_req_s {
// 	struct usbdev_req_s req;	/* Standard USB request */
// 	struct amebasmart_req_s *flink;	/* Supports a singly linked list */
// };

// /* This is the internal representation of an endpoint */

// struct amebasmart_ep_s {
// 	/* Common endpoint fields.  This must be the first thing defined in the
// 	 * structure so that it is possible to simply cast from struct usbdev_ep_s
// 	 * to struct amebasmart_ep_s.
// 	 */

// 	struct usbdev_ep_s ep;		/* Standard endpoint structure */

// 	/* STR71X-specific fields */

// 	struct amebasmart_usbdev_s *dev;	/* Reference to private driver data */
// 	struct amebasmart_req_s *head;	/* Request list for this endpoint */
// 	struct amebasmart_req_s *tail;
// 	uint8_t bufno;				/* Allocated buffer number */
// 	uint8_t stalled:1;			/* true: Endpoint is stalled */
// 	uint8_t halted:1;			/* true: Endpoint feature halted */
// 	uint8_t txbusy:1;			/* true: TX endpoint FIFO full */
// 	uint8_t txnullpkt:1;		/* Null packet needed at end of transfer */
// };

struct amebasmart_usbdev_s {
	/* Common device fields.  This must be the first thing defined in the
	 * structure so that it is possible to simply cast from struct usbdev_s
	 * to struct amebasmart_usbdev_s.
	 */

	struct usbdev_s usbdev;

	/* The bound device class driver */

	struct usbdevclass_driver_s *driver;
#ifdef CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	static uint32_t cdc_acm_xfer_idx = 0;
	static uint8_t cdc_acm_async_xfer_buf[CONFIG_CDC_ACM_ASYNC_BUF_SIZE] __attribute__((aligned(CACHE_LINE_SIZE)));
	static uint16_t cdc_acm_async_xfer_buf_pos = 0;
	static volatile int cdc_acm_async_xfer_busy = 0;
	static sem_t cdc_acm_async_xfer_sema;
#endif
	/* AMEBASMART-specific fields */
	uint32_t ep0_xfer_total_len;					/* The total data length to transfer */
	uint32_t ep0_xfer_rem_len;					/* The remain data length to transfer */
	uint32_t ep0_recv_rem_len;					/* The remain data length to receive */
	uint8_t *ctrl_buf;							/* Buffer for control transfer */
	void *pcd;								/* PCD handle */
	uint16_t ep0_data_len;						/* EP0 data length */
	uint8_t ep0_state;							/* EP0 state */
	uint8_t dev_config;							/* Device config index */
	uint8_t dev_speed;							/* Device speed, usb_speed_type_t */
	uint8_t dev_state;							/* Device state, usbd_state_t */
	uint8_t dev_old_state;						/* Device old state, usbd_state_t */
	uint8_t dev_attach_status;					/* Device attach status, usbd_attach_status_t */
	uint8_t test_mode;							/* Test mode */
	uint8_t self_powered : 1;						/* Self powered or not, 0-bus powered, 1-self powered */
	uint8_t remote_wakeup_en : 1;					/* Remote wakeup enable or not, 0-disabled, 1-enabled */
	uint8_t remote_wakeup : 1;						/* Remote wakeup */
	uint16_t cdc_acm_bulk_out_xfer_size;
	uint16_t cdc_acm_bulk_in_xfer_size;
	uint8_t bitrate;
	uint8_t format;
	uint8_t parity_type;
	uint8_t data_type;
	// uint8_t ep0state;			/* State of EP0 (see enum amebasmart_ep0state_e) */
	// uint8_t rsmstate;			/* Resume state (see enum amebasmart_rsmstate_e) */
	// uint8_t nesofs;				/* ESOF counter (for resume support) */
	// uint8_t rxpending:1;		/* 1: OUT data in PMA, but no read requests */
	// uint8_t selfpowered:1;		/* 1: Device is self powered */
	// uint8_t epavail;			/* Bitset of available endpoints */
	// uint8_t bufavail;			/* Bitset of available buffers */
	// uint16_t rxstatus;			/* Saved during interrupt processing */
	// uint16_t txstatus;			/* "   " "    " "       " "        " */
	// uint16_t imask;				/* Current interrupt mask */

	/* E0 SETUP data buffering.
	 *
	 * ctrl
	 *   The 8-byte SETUP request is received on the EP0 OUT endpoint and is
	 *   saved.
	 *
	 * ep0data
	 *   For OUT SETUP requests, the SETUP data phase must also complete before
	 *   the SETUP command can be processed.  The ep0 packet receipt logic
	 *   amebasmart_ep0_rdrequest will save the accompanying EP0 OUT data in
	 *   ep0data[] before the SETUP command is re-processed.
	 *
	 * ep0datlen
	 *   Lenght of OUT DATA received in ep0data[]
	 */

	// struct usb_ctrlreq_s ctrl;	/* Last EP0 request */

	// uint8_t ep0data[CONFIG_USBDEV_SETUP_MAXDATASIZE];
	// uint16_t ep0datlen;

	// /* The endpoint list */

	// struct amebasmart_ep_s eplist[AMEBASMART_NENDPOINTS];
};




// struct amebasmart_usbd_cdc_acm_dev_t {
// 	usb_setup_req_t ctrl_req;
// 	usb_dev_t *dev;
// 	usbd_cdc_acm_cb_t *cb;
// #if CONFIG_CDC_ACM_NOTIFY
// 	usbd_cdc_acm_ntf_t *intr_in_buf;
// #endif
// 	u32 bulk_out_buf_size;
// 	u32 bulk_in_buf_size;
// 	u8 *bulk_out_buf;
// 	u8 *bulk_in_buf;
// 	u8 *ctrl_buf;
// #if CONFIG_CDC_ACM_NOTIFY
// 	u16 intr_notify_idx;
// #endif
// 	u8 bulk_out_zlp : 1;
// 	__IO u8 is_bulk_in_busy : 1;
// 	__IO u8 is_ready : 1;
// 	__IO u8 bulk_in_state : 1;
// #if CONFIG_CDC_ACM_NOTIFY
// 	__IO u8 is_intr_in_busy : 1;
// 	__IO u8 intr_in_state : 1;
// #endif
// };

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Register operations ******************************************************/

#if defined(CONFIG_AMEBASMART_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
static uint16_t amebasmart_getreg(uint32_t addr);
static void amebasmart_putreg(uint16_t val, uint32_t addr);
static void amebasmart_checksetup(void);
static void amebasmart_dumpep(int epno);
#else
#define amebasmart_getreg(addr)      getreg16(addr)
#define amebasmart_putreg(val, addr) putreg16(val, addr)
#define amebasmart_checksetup()
#define amebasmart_dumpep(epno)
#endif

/* Low-Level Helpers ********************************************************/

static inline void amebasmart_seteptxcount(uint8_t epno, uint16_t count);
static inline void amebasmart_seteptxaddr(uint8_t epno, uint16_t addr);
static inline uint16_t amebasmart_geteptxaddr(uint8_t epno);
static void amebasmart_seteprxcount(uint8_t epno, uint16_t count);
static inline uint16_t amebasmart_geteprxcount(uint8_t epno);
static inline void amebasmart_seteprxaddr(uint8_t epno, uint16_t addr);
static inline uint16_t amebasmart_geteprxaddr(uint8_t epno);
static inline void amebasmart_setepaddress(uint8_t epno, uint16_t addr);
static inline void amebasmart_seteptype(uint8_t epno, uint16_t type);
static inline void amebasmart_seteptxaddr(uint8_t epno, uint16_t addr);
static inline void amebasmart_setstatusout(uint8_t epno);
static inline void amebasmart_clrstatusout(uint8_t epno);
static void amebasmart_clrrxdtog(uint8_t epno);
static void amebasmart_clrtxdtog(uint8_t epno);
static void amebasmart_clrepctrrx(uint8_t epno);
static void amebasmart_clrepctrtx(uint8_t epno);
static void amebasmart_seteptxstatus(uint8_t epno, uint16_t state);
static void amebasmart_seteprxstatus(uint8_t epno, uint16_t state);
static inline uint16_t amebasmart_geteptxstatus(uint8_t epno);
static inline uint16_t amebasmart_geteprxstatus(uint8_t epno);
static bool amebasmart_eptxstalled(uint8_t epno);
static bool amebasmart_eprxstalled(uint8_t epno);
static void amebasmart_setimask(struct amebasmart_usbdev_s *priv, uint16_t setbits, uint16_t clrbits);

/* Suspend/Resume Helpers ***************************************************/

static void amebasmart_suspend(struct amebasmart_usbdev_s *priv);
static void amebasmart_initresume(struct amebasmart_usbdev_s *priv);
static void amebasmart_esofpoll(struct amebasmart_usbdev_s *priv);

/* Request Helpers **********************************************************/

static void amebasmart_copytopma(const uint8_t *buffer, uint16_t pma, uint16_t nbytes);
static inline void amebasmart_copyfrompma(uint8_t *buffer, uint16_t pma, uint16_t nbytes);
static struct amebasmart_req_s *amebasmart_rqdequeue(struct amebasmart_ep_s *privep);
static void amebasmart_rqenqueue(struct amebasmart_ep_s *privep, struct amebasmart_req_s *req);
static inline void amebasmart_abortrequest(struct amebasmart_ep_s *privep, struct amebasmart_req_s *privreq, int16_t result);
static void amebasmart_reqcomplete(struct amebasmart_ep_s *privep, int16_t result);
static void amebasmart_epwrite(struct amebasmart_usbdev_s *buf, struct amebasmart_ep_s *privep, const uint8_t *data, uint32_t nbytes);
static int amebasmart_wrrequest(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep);
inline static int amebasmart_wrrequest_ep0(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep);
static inline int amebasmart_ep0_rdrequest(struct amebasmart_usbdev_s *priv);
static int amebasmart_rdrequest(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep);
static void amebasmart_cancelrequests(struct amebasmart_ep_s *privep);

/* Interrupt level processing ***********************************************/

static void amebasmart_dispatchrequest(struct amebasmart_usbdev_s *priv);
static void amebasmart_epdone(struct amebasmart_usbdev_s *priv, uint8_t epno);
static void amebasmart_setdevaddr(struct amebasmart_usbdev_s *priv, uint8_t value);
static void amebasmart_ep0setup(struct amebasmart_usbdev_s *priv);
static void amebasmart_ep0out(struct amebasmart_usbdev_s *priv);
static void amebasmart_ep0in(struct amebasmart_usbdev_s *priv);
static inline void amebasmart_ep0done(struct amebasmart_usbdev_s *priv, uint16_t istr);
static void amebasmart_lptransfer(struct amebasmart_usbdev_s *priv);
static int amebasmart_hpinterrupt(int irq, void *context, FAR void *arg);
static int amebasmart_lpinterrupt(int irq, void *context, FAR void *arg);

/* Endpoint helpers *********************************************************/

static inline struct amebasmart_ep_s *amebasmart_epreserve(struct amebasmart_usbdev_s *priv, uint8_t epset);
static inline void amebasmart_epunreserve(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep);
static inline bool amebasmart_epreserved(struct amebasmart_usbdev_s *priv, int epno);
static int amebasmart_epallocpma(struct amebasmart_usbdev_s *priv);
static inline void amebasmart_epfreepma(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep);

/* Endpoint operations ******************************************************/

static int amebasmart_epconfigure(struct usbdev_ep_s *ep, const struct usb_epdesc_s *desc, bool last);
static int amebasmart_epdisable(struct usbdev_ep_s *ep);
static struct usbdev_req_s *amebasmart_epallocreq(struct usbdev_ep_s *ep);
static void amebasmart_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *);
static int amebasmart_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static int amebasmart_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req);
static int amebasmart_epstall(struct usbdev_ep_s *ep, bool resume);

/* USB device controller operations *****************************************/

static struct usbdev_ep_s *amebasmart_allocep(struct usbdev_s *dev, uint8_t epno, bool in, uint8_t eptype);
static void amebasmart_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep);
static int amebasmart_getframe(struct usbdev_s *dev);
static int amebasmart_wakeup(struct usbdev_s *dev);
static int amebasmart_selfpowered(struct usbdev_s *dev, bool selfpowered);

/* Initialization/Reset *****************************************************/

static void amebasmart_reset(struct amebasmart_usbdev_s *priv);
static void amebasmart_hwreset(struct amebasmart_usbdev_s *priv);
static void amebasmart_hwsetup(struct amebasmart_usbdev_s *priv);
static void amebasmart_hwshutdown(struct amebasmart_usbdev_s *priv);



/*usbd related APIs*/
static int amebasmart_usbd_init(struct amebasmart_usbdev_s *priv, usbd_config_t *cfg);
static int amebasmart_usbd_deinit(struct amebasmart_usbdev_s *priv);
static int amebasmart_usbd_attach_status(struct amebasmart_usbdev_s *priv);
static int amebasmart_usbd_bus_status(struct amebasmart_usbdev_s *priv, u32 *bus_status);
static int amebasmart_usbd_ep_init(struct amebasmart_usbdev_s *priv, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps);
static int amebasmart_usbd_ep_deinit(struct amebasmart_usbdev_s *priv, uint8_t ep_addr);
static int amebasmart_usbd_ep_transmit(struct amebasmart_usbdev_s *priv, uint8_t ep_addr, uint8_t *buf, uint16_t len);
static int amebasmart_usbd_ep_receive(struct amebasmart_usbdev_s *priv, uint8_t ep_addr, uint8_t *buf, uint16_t len);
static int amebasmart_usbd_ep0_transmit(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint16_t len);
static int amebasmart_usbd_ep0_receive(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint16_t len);
static int amebasmart_usbd_ep0_transmit_status(struct amebasmart_usbdev_s *priv);
static int amebasmart_usbd_ep0_receive_status(struct amebasmart_usbdev_s *priv);
static int amebasmart_usbd_ep0_set_stall(struct amebasmart_usbdev_s *priv);
static int amebasmart_usbd_ep_set_stall(struct amebasmart_usbdev_s *priv, uint8_t ep_addr);
static int amebasmart_usbd_ep_clear_stall(struct amebasmart_usbdev_s *priv, uint8_t ep_addr);
static int amebasmart_usbd_ep_is_stall(struct amebasmart_usbdev_s *priv, uint8_t ep_addr);
static int amebasmart_usbd_get_str_desc(const char *str, uint8_t *desc, uint16_t *len);


/*CDC ACM related*/
static int amebasmart_cdc_acm_cb_init(struct amebasmart_usbdev_s *priv);
static int amebasmart_cdc_acm_cb_deinit(struct amebasmart_usbdev_s *priv);
static int amebasmart_cdc_acm_cb_received(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint32_t len);
static int amebasmart_cdc_acm_cb_setup(usb_setup_req_t *req, u8 *buf);
static void amebasmart_cdc_acm_cb_status_changed(struct amebasmart_usbdev_s *priv, uint8_t status);
static int amebasmart_usbd_acm_init(struct amebasmart_usbdev_s *priv, usbd_cdc_acm_cb_t *cb);
static amebasmart_usbd_acm_deinit(struct amebasmart_usbdev_s *priv);



static usbd_cdc_acm_cb_t amebasmart_cdc_acm_cb = {
	.init = amebasmart_cdc_acm_cb_init,
	.deinit = amebasmart_cdc_acm_cb_deinit,
	.setup = amebasmart_cdc_acm_cb_setup,
	.received = amebasmart_cdc_acm_cb_received,
	.status_changed = amebasmart_cdc_acm_cb_status_changed
};
/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Since there is only a single USB interface, all status information can be
 * be simply retained in a single global instance.
 */


// static const struct usbdev_epops_s g_epops = {
// 	.configure = amebasmart_epconfigure,
// 	.disable = amebasmart_epdisable,
// 	.allocreq = amebasmart_epallocreq,
// 	.freereq = amebasmart_epfreereq,
// 	.submit = amebasmart_epsubmit,
// 	.cancel = amebasmart_epcancel,
// 	.stall = amebasmart_epstall,
// };

// static const struct usbdev_ops_s g_devops = {
// 	.allocep = amebasmart_allocep,
// 	.freeep = amebasmart_freeep,
// 	.getframe = amebasmart_getframe,
// 	.wakeup = amebasmart_wakeup,
// 	.selfpowered = amebasmart_selfpowered,
// 	.pullup = amebasmart_usbpullup,
// };



static int amebasmart_up_setup(struct amebasmart_usbdev_s *priv);
static void amebasmart_log_up_shutdown(struct amebasmart_usbdev_s *priv);
static int amebasmart_log_up_attach(struct amebasmart_usbdev_s *priv);
static void amebasmart_log_up_detach(struct amebasmart_usbdev_s *priv);
static int amebasmart_log_up_ioctl(FAR struct amebasmart_usbdev_s *priv, int cmd, unsigned long arg);
static int amebasmart_log_up_receive(struct amebasmart_usbdev_s *priv, unsigned int*status);
static void amebasmart_log_up_rxint(struct amebasmart_usbdev_s *priv, bool enable);
static bool amebasmart_log_up_rxavailable(struct amebasmart_usbdev_s *priv);
static void amebasmart_log_up_send(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint32_t len);
static void amebasmart_log_up_txint(struct amebasmart_usbdev_s *priv, bool enable);
static bool amebasmart_log_up_txready(struct amebasmart_usbdev_s *priv);
static bool amebasmart_log_up_txempty(struct amebasmart_usbdev_s *priv);

/* Serial driver USB operations */
// static const struct usb_ops_s log_g_usbd_ops = {
// 	.setup = amebasmart_up_setup,
// 	.shutdown = amebasmart_log_up_shutdown,
// 	// .attach = amebasmart_log_up_attach,
// 	// .detach = amebasmart_log_up_detach,
// 	// .ioctl = amebasmart_log_up_ioctl,
// 	// .receive = amebasmart_log_up_receive,
// 	// .rxint = amebasmart_log_up_rxint,
// 	// .rxavailable = amebasmart_log_up_rxavailable,

// 	.send = amebasmart_log_up_send,
// 	// .txint = amebasmart_log_up_txint,
// 	// .txready = amebasmart_log_up_txready,
// 	// .txempty = amebasmart_log_up_txempty,
// };
static struct amebasmart_usbdev_s g_usbdev = {
	// .usbdev       = {
	// 	&log_g_usbd_ops
	// },
	.cdc_acm_bulk_out_xfer_size = 2048,
	.cdc_acm_bulk_in_xfer_size = 2048,
	.bitrate = 150000,
	.format = 0x00,
	.parity_type = 0x00,
	.data_type = 0x08,
};

/*CDC ACM related*/


#define CONFIG_AMEBASMART_CDC_ACM_ISR_THREAD_PRIORITY 200
static usbd_config_t amebasmart_cdc_acm_cfg = {
	.speed = USB_SPEED_FULL,
	.dma_enable   = 1U,
	.isr_priority = CONFIG_AMEBASMART_CDC_ACM_ISR_THREAD_PRIORITY,
	.intr_use_ptx_fifo  = 0U,
	.nptx_max_epmis_cnt = 1U,
	.ext_intr_en        = USBD_EPMIS_INTR,
	.nptx_max_err_cnt   = {0U, 0U, 0U, 2000U, },
};


static usbd_cdc_acm_line_coding_t amebasmart_cdc_acm_line_coding;

static int amebasmart_cdc_acm_cb_init(struct amebasmart_usbdev_s *priv) {

	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	usbd_cdc_acm_line_coding_t *lc = &amebasmart_cdc_acm_line_coding;

	lc->bitrate = dev->bitrate;
	lc->format = dev->format;
	lc->parity_type = dev->parity_type;
	lc->data_type = dev->data_type;

#ifdef CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	dev->cdc_acm_async_xfer_buf_pos = 0;
	dev->cdc_acm_async_xfer_busy = 0;
#endif
	return 0;
}

/**
  * @brief  DeInitializes the CDC media layer
  * @param  None
  * @retval Status
  */
static int amebasmart_cdc_acm_cb_deinit(struct amebasmart_usbdev_s *priv) {

	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	int ret = 0;
	DEBUGASSERT(dev);
#ifdef CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	dev->cdc_acm_async_xfer_buf_pos = 0;
	dev->cdc_acm_async_xfer_busy = 0;
#endif
	return HAL_OK;
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface through this function.
  * @param  Buf: RX buffer
  * @param  Len: RX data length (in bytes)
  * @retval Status
  */
static int amebasmart_cdc_acm_cb_received(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint32_t len) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	int ret = 0;
	DEBUGASSERT(dev);

#ifdef CONFIG_AMEBASMART_USBD_CDC_ACM_ASYNC_XFER
	if (0 == dev->cdc_acm_async_xfer_busy) {
		if ((dev->cdc_acm_async_xfer_buf_pos + len) > dev->CONFIG_CDC_ACM_ASYNC_BUF_SIZE) {
			len = dev->CONFIG_CDC_ACM_ASYNC_BUF_SIZE - dev->cdc_acm_async_xfer_buf_pos;  // extra data discarded
		}

		memcpy((void *)((uint32_t)dev->cdc_acm_async_xfer_buf + dev->cdc_acm_async_xfer_buf_pos), buf, len);
		dev->cdc_acm_async_xfer_buf_pos += len;
		if (dev->cdc_acm_async_xfer_buf_pos >= dev->CONFIG_CDC_ACM_ASYNC_BUF_SIZE) {
			dev->cdc_acm_async_xfer_buf_pos = 0;
			(void)sem_post(&dev->cdc_acm_async_xfer_sema);
		}
	} else {
		dbg("[ACM] Busy, discard %dB\n", len);
		ret = HAL_BUSY;
	}

	return ret;
#else
	return usbd_cdc_acm_transmit(buf, len);
#endif
}

/**
  * @brief  Handle the CDC class control requests
  * @param  cmd: Command code
  * @param  buf: Buffer containing command data (request parameters)
  * @param  len: Number of data to be sent (in bytes)
  * @retval Status
  */
static uint16_t cdc_acm_ctrl_line_state;
static int amebasmart_cdc_acm_cb_setup(usb_setup_req_t *req, u8 *buf) {
	usbd_cdc_acm_line_coding_t *lc = &amebasmart_cdc_acm_line_coding;

	switch (req->bRequest) {
	case CDC_SEND_ENCAPSULATED_COMMAND:
		/* Do nothing */
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		/* Do nothing */
		break;

	case CDC_SET_COMM_FEATURE:
		/* Do nothing */
		break;

	case CDC_GET_COMM_FEATURE:
		/* Do nothing */
		break;

	case CDC_CLEAR_COMM_FEATURE:
		/* Do nothing */
		break;

	case CDC_SET_LINE_CODING:
		if (req->wLength == CDC_ACM_LINE_CODING_SIZE) {
			lc->bitrate = (u32)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
			lc->format = buf[4];
			lc->parity_type = buf[5];
			lc->data_type = buf[6];
		}
		break;

	case CDC_GET_LINE_CODING:
		buf[0] = (u8)(lc->bitrate & 0xFF);
		buf[1] = (u8)((lc->bitrate >> 8) & 0xFF);
		buf[2] = (u8)((lc->bitrate >> 16) & 0xFF);
		buf[3] = (u8)((lc->bitrate >> 24) & 0xFF);
		buf[4] = lc->format;
		buf[5] = lc->parity_type;
		buf[6] = lc->data_type;
		break;

	case CDC_SET_CONTROL_LINE_STATE:
		/*
		wValue:	wValue, Control Signal Bitmap
				D2-15:	Reserved, 0
				D1:	RTS, 0 - Deactivate, 1 - Activate
				D0:	DTR, 0 - Not Present, 1 - Present
		*/
		cdc_acm_ctrl_line_state = req->wValue;
		if (cdc_acm_ctrl_line_state & 0x01) {
			dbg("[ACM] VCOM port activate\n");
#if CONFIG_AMEBASMART_CDC_ACM_NOTIFY
			usbd_cdc_acm_notify_serial_state(CDC_ACM_CTRL_DSR | CDC_ACM_CTRL_DCD);
#endif
		}
		break;

	case CDC_SEND_BREAK:
		/* Do nothing */
		break;

	default:
		break;
	}

	return HAL_OK;
}

static void amebasmart_cdc_acm_cb_status_changed(struct amebasmart_usbdev_s *priv, uint8_t status) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
#if CONFIG_AMEBASMART_USBD_CDC_ACM_HOTPLUG
	dev->cdc_acm_attach_status = status;
	sem_post(&dev->cdc_acm_attach_status_changed_sema);
#endif
}


static int amebasmart_usbd_acm_init(struct amebasmart_usbdev_s *priv, usbd_cdc_acm_cb_t *cb) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	int ret = -1;
	ret = usbd_cdc_acm_init(dev->cdc_acm_bulk_out_xfer_size, dev->cdc_acm_bulk_in_xfer_size, cb);
	if (ret != 0) {
		dbg("USB ACM init failed\n");
		//amebasmart_usbd_deinit(dev);
	}
	return ret;
}

static amebasmart_usbd_acm_deinit(struct amebasmart_usbdev_s *priv)
{
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	usbd_cdc_acm_deinit();
}

static int amebasmart_usbd_init(struct amebasmart_usbdev_s *priv, usbd_config_t *cfg) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	dev->dev_state  = USBD_STATE_DEFAULT;
	dev->ctrl_buf = (u8 *)usb_os_malloc(USB_OTG_HS_MAX_PACKET_SIZE);
	if (dev->ctrl_buf == NULL) {
		return HAL_ERR_MEM;
	}

	/* Init PCD Driver */
	return usbd_pcd_init(dev, cfg);

};

// static int amebasmart_usbd_deinit(struct amebasmart_usbdev_s *priv) {
	
// 	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
// 	int ret = 0;
// 	DEBUGASSERT(dev);
// 	dev->dev_state  = USBD_STATE_DEFAULT;
// 	if (dev->driver != NULL) {
// 		dev->driver->clear_config(dev, dev->dev_config);
// 		dev->driver = NULL;
// 	}

// 	ret = usbd_pcd_deinit(dev);

// 	if (dev->ctrl_buf != NULL) {
// 		usb_os_mfree(dev->ctrl_buf);
// 		dev->ctrl_buf = NULL;
// 	}
// 	return ret;
// };

static int amebasmart_usbd_attach_status(struct amebasmart_usbdev_s *priv) {
	
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return dev->dev_attach_status;
};

static int amebasmart_usbd_bus_status(struct amebasmart_usbdev_s *priv, u32 *bus_status) {
	
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	if (dev->pcd && bus_status) {
		return usbd_hal_get_bus_status(dev->pcd, bus_status);
	} else {
		dbg("[USBD] Not ready\n");
		return HAL_ERR_PARA;
	}
};

static int amebasmart_usbd_ep_init(struct amebasmart_usbdev_s *priv, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_pcd_ep_init(dev->pcd, ep_addr, ep_mps, ep_type);
}

static int amebasmart_usbd_ep_deinit(struct amebasmart_usbdev_s *priv, uint8_t ep_addr) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_pcd_ep_deinit(dev->pcd, ep_addr);
}

static int amebasmart_usbd_ep_transmit(struct amebasmart_usbdev_s *priv, uint8_t ep_addr, uint8_t *buf, uint16_t len) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_pcd_ep_transmit(dev->pcd, ep_addr, buf, len);
}

static int amebasmart_usbd_ep_receive(struct amebasmart_usbdev_s *priv, uint8_t ep_addr, uint8_t *buf, uint16_t len) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_pcd_ep_receive(dev->pcd, ep_addr, buf, len);
}

static int amebasmart_usbd_ep0_transmit(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint16_t len) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	/* Set EP0 State */
	dev->ep0_state = USBD_EP0_DATA_IN;
	dev->ep0_xfer_total_len = len;
	dev->ep0_xfer_rem_len = len;

	/* Start the transfer */
	return usbd_ep_transmit(dev, USB_EP0_IN, buf, len);
}
static int amebasmart_usbd_ep0_receive(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint16_t len) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	/* Set EP0 State */
	dev->ep0_state = USBD_EP0_DATA_OUT;
	dev->ep0_recv_rem_len = len;

	/* Start the transfer */
	return usbd_ep_receive(dev, USB_EP0_OUT, buf, len);
}

static int amebasmart_usbd_ep0_transmit_status(struct amebasmart_usbdev_s *priv) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_core_ep0_transmit_status(dev);
}

static int amebasmart_usbd_ep0_receive_status(struct amebasmart_usbdev_s *priv) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_core_ep0_receive_status(dev);
}

static int amebasmart_usbd_ep0_set_stall(struct amebasmart_usbdev_s *priv) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_core_ep0_set_stall(dev);
}

static int amebasmart_usbd_ep_set_stall(struct amebasmart_usbdev_s *priv, uint8_t ep_addr) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_core_ep_set_stall(dev, ep_addr);
}

static int amebasmart_usbd_ep_clear_stall(struct amebasmart_usbdev_s *priv, uint8_t ep_addr) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_core_ep_clear_stall(dev, ep_addr);
}

static int amebasmart_usbd_ep_is_stall(struct amebasmart_usbdev_s *priv, uint8_t ep_addr) {
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	return usbd_core_ep_is_stall(dev, ep_addr);
}

static int amebasmart_usbd_get_str_desc(const char *str, uint8_t *desc, uint16_t *len) {
	uint8_t idx = 0U;
	uint8_t *p = (uint8_t *)str;

	if (p != NULL) {
		*len = (uint16_t)(strlen(str) * 2 + 2);
		desc[idx++] = *(uint8_t *)(void *)len;
		desc[idx++] = USB_DESC_TYPE_STRING;

		while (*p != '\0') {
			desc[idx++] = *p++;
			desc[idx++] =  0U;
		}
	}
}


static int amebasmart_up_setup(struct amebasmart_usbdev_s *priv)
{
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	int ret = -1;
	ret = amebasmart_usbd_init(dev, &amebasmart_cdc_acm_cfg);
	if (ret != 0) {
		dbg("usbd init fail\n");
		return ret;
	}
	ret = amebasmart_usbd_acm_init(dev, &amebasmart_cdc_acm_cb);
	if (ret != 0) {
		dbg("usbd acm init fail\n");
		return ret;
	}
	return ret;
}

// static void amebasmart_log_up_shutdown(struct amebasmart_usbdev_s *priv)
// {
// 	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
// 	DEBUGASSERT(dev);
// 	amebasmart_usbd_acm_deinit(dev);
// 	amebasmart_usbd_deinit(dev);

// }

static void amebasmart_log_up_send(struct amebasmart_usbdev_s *priv, uint8_t *buf, uint32_t len)
{
	struct amebasmart_usbdev_s *dev = (struct amebasmart_usbdev_s *)priv;
	DEBUGASSERT(dev);
	usbd_cdc_acm_transmit(buf, len);
}

void usb_printf(uint8_t *buf, int len) {
	FAR struct amebasmart_usbdev_s *priv = NULL;
	DiagPrintf("USB PRINT\n");
	priv = &g_usbdev;
	amebasmart_up_setup(priv);
	// amebasmart_log_up_send(priv, buf, len);
}
// void usb_printf(struct usbdev_s *dev, uint8_t *buf, int len) {
// 	lldbg("USB PRINT\n");
// 	FAR struct amebasmart_usbdev_s *priv = (FAR struct amebasmart_usbdev_s *)dev;
	
// }
/****************************************************************************
 * Public Data
 ****************************************************************************/

// #ifdef CONFIG_USBDEV_TRACE_STRINGS
// const struct trace_msg_t g_usb_trace_strings_intdecode[] = {
// 	TRACE_STR(AMEBASMART_TRACEINTID_CLEARFEATURE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_DEVGETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEINTID_DISPATCH),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0IN),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0INDONE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0OUTDONE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0SETUPDONE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0SETUPSETADDRESS),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EPGETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EPINDONE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EPINQEMPTY),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EPOUTDONE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EPOUTPENDING),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EPOUTQEMPTY),
// 	TRACE_STR(AMEBASMART_TRACEINTID_ESOF),
// 	TRACE_STR(AMEBASMART_TRACEINTID_GETCONFIG),
// 	TRACE_STR(AMEBASMART_TRACEINTID_GETSETDESC),
// 	TRACE_STR(AMEBASMART_TRACEINTID_GETSETIF),
// 	TRACE_STR(AMEBASMART_TRACEINTID_GETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEINTID_HPINTERRUPT),
// 	TRACE_STR(AMEBASMART_TRACEINTID_IFGETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEINTID_LPCTR),
// 	TRACE_STR(AMEBASMART_TRACEINTID_LPINTERRUPT),
// 	TRACE_STR(AMEBASMART_TRACEINTID_NOSTDREQ),
// 	TRACE_STR(AMEBASMART_TRACEINTID_RESET),
// 	TRACE_STR(AMEBASMART_TRACEINTID_SETCONFIG),
// 	TRACE_STR(AMEBASMART_TRACEINTID_SETFEATURE),
// 	TRACE_STR(AMEBASMART_TRACEINTID_SUSP),
// 	TRACE_STR(AMEBASMART_TRACEINTID_SYNCHFRAME),
// 	TRACE_STR(AMEBASMART_TRACEINTID_WKUP),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0SETUPOUT),
// 	TRACE_STR(AMEBASMART_TRACEINTID_EP0SETUPOUTDATA),
// 	TRACE_STR_END
// };
// #endif

// #ifdef CONFIG_USBDEV_TRACE_STRINGS
// const struct trace_msg_t g_usb_trace_strings_deverror[] = {
// 	TRACE_STR(AMEBASMART_TRACEERR_ALLOCFAIL),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADCLEARFEATURE),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADDEVGETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADEPGETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADEPNO),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADEPTYPE),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADGETCONFIG),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADGETSETDESC),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADGETSTATUS),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADSETADDRESS),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADSETCONFIG),
// 	TRACE_STR(AMEBASMART_TRACEERR_BADSETFEATURE),
// 	TRACE_STR(AMEBASMART_TRACEERR_BINDFAILED),
// 	TRACE_STR(AMEBASMART_TRACEERR_DISPATCHSTALL),
// 	TRACE_STR(AMEBASMART_TRACEERR_DRIVER),
// 	TRACE_STR(AMEBASMART_TRACEERR_DRIVERREGISTERED),
// 	TRACE_STR(AMEBASMART_TRACEERR_EP0BADCTR),
// 	TRACE_STR(AMEBASMART_TRACEERR_EP0SETUPSTALLED),
// 	TRACE_STR(AMEBASMART_TRACEERR_EPBUFFER),
// 	TRACE_STR(AMEBASMART_TRACEERR_EPDISABLED),
// 	TRACE_STR(AMEBASMART_TRACEERR_EPOUTNULLPACKET),
// 	TRACE_STR(AMEBASMART_TRACEERR_EPRESERVE),
// 	TRACE_STR(AMEBASMART_TRACEERR_INVALIDCTRLREQ),
// 	TRACE_STR(AMEBASMART_TRACEERR_INVALIDPARMS),
// 	TRACE_STR(AMEBASMART_TRACEERR_IRQREGISTRATION),
// 	TRACE_STR(AMEBASMART_TRACEERR_NOTCONFIGURED),
// 	TRACE_STR(AMEBASMART_TRACEERR_REQABORTED),
// 	TRACE_STR_END
// };
// #endif

// /****************************************************************************
//  * Private Private Functions
//  ****************************************************************************/

// /****************************************************************************
//  * Register Operations
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_getreg
//  ****************************************************************************/

// #if defined(CONFIG_AMEBASMART_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
// static uint16_t amebasmart_getreg(uint32_t addr)
// {
// 	static uint32_t prevaddr = 0;
// 	static uint16_t preval = 0;
// 	static uint32_t count = 0;

// 	/* Read the value from the register */

// 	uint16_t val = getreg16(addr);

// 	/* Is this the same value that we read from the same register last time?
// 	 * Are we polling the register?  If so, suppress some of the output.
// 	 */

// 	if (addr == prevaddr && val == preval) {
// 		if (count == 0xffffffff || ++count > 3) {
// 			if (count == 4) {
// 				lldbg("...\n");
// 			}
// 			return val;
// 		}
// 	}

// 	/* No this is a new address or value */

// 	else {
// 		/* Did we print "..." for the previous value? */

// 		if (count > 3) {
// 			/* Yes.. then show how many times the value repeated */

// 			lldbg("[repeats %d more times]\n", count - 3);
// 		}

// 		/* Save the new address, value, and count */

// 		prevaddr = addr;
// 		preval = val;
// 		count = 1;
// 	}

// 	/* Show the register value read */

// 	lldbg("%08x->%04x\n", addr, val);
// 	return val;
// }
// #endif

// /****************************************************************************
//  * Name: amebasmart_putreg
//  ****************************************************************************/

// #if defined(CONFIG_AMEBASMART_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
// static void amebasmart_putreg(uint16_t val, uint32_t addr)
// {
// 	/* Show the register value being written */

// 	lldbg("%08x<-%04x\n", addr, val);

// 	/* Write the value */

// 	putreg16(val, addr);
// }
// #endif

// /****************************************************************************
//  * Name: amebasmart_dumpep
//  ****************************************************************************/

// #if defined(CONFIG_AMEBASMART_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
// static void amebasmart_dumpep(int epno)
// {
// 	uint32_t addr;

// 	/* Common registers */

// 	lldbg("CNTR:   %04x\n", getreg16(AMEBASMART_USB_CNTR));
// 	lldbg("ISTR:   %04x\n", getreg16(AMEBASMART_USB_ISTR));
// 	lldbg("FNR:    %04x\n", getreg16(AMEBASMART_USB_FNR));
// 	lldbg("DADDR:  %04x\n", getreg16(AMEBASMART_USB_DADDR));
// 	lldbg("BTABLE: %04x\n", getreg16(AMEBASMART_USB_BTABLE));

// 	/* Endpoint register */

// 	addr = AMEBASMART_USB_EPR(epno);
// 	lldbg("EPR%d:   [%08x] %04x\n", epno, addr, getreg16(addr));

// 	/* Endpoint descriptor */

// 	addr = AMEBASMART_USB_BTABLE_ADDR(epno, 0);
// 	lldbg("DESC:   %08x\n", addr);

// 	/* Endpoint buffer descriptor */

// 	addr = AMEBASMART_USB_ADDR_TX(epno);
// 	lldbg("  TX ADDR:  [%08x] %04x\n", addr, getreg16(addr));

// 	addr = AMEBASMART_USB_COUNT_TX(epno);
// 	lldbg("     COUNT: [%08x] %04x\n", addr, getreg16(addr));

// 	addr = AMEBASMART_USB_ADDR_RX(epno);
// 	lldbg("  RX ADDR:  [%08x] %04x\n", addr, getreg16(addr));

// 	addr = AMEBASMART_USB_COUNT_RX(epno);
// 	lldbg("     COUNT: [%08x] %04x\n", addr, getreg16(addr));
// }
// #endif

// /****************************************************************************
//  * Name: amebasmart_checksetup
//  ****************************************************************************/

// #if defined(CONFIG_AMEBASMART_USBDEV_REGDEBUG) && defined(CONFIG_DEBUG)
// static void amebasmart_checksetup(void)
// {
// 	uint32_t cfgr = getreg32(AMEBASMART_RCC_CFGR);
// 	uint32_t apb1rstr = getreg32(AMEBASMART_RCC_APB1RSTR);
// 	uint32_t apb1enr = getreg32(AMEBASMART_RCC_APB1ENR);

// 	lldbg("CFGR: %08x APB1RSTR: %08x APB1ENR: %08x\n", cfgr, apb1rstr, apb1enr);

// 	if ((apb1rstr & RCC_APB1RSTR_USBRST) != 0 || (apb1enr & RCC_APB1ENR_USBEN) == 0) {
// 		lldbg("ERROR: USB is NOT setup correctly\n");
// 	}
// }
// #endif

// /****************************************************************************
//  * Low-Level Helpers
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_seteptxcount
//  ****************************************************************************/

// static inline void amebasmart_seteptxcount(uint8_t epno, uint16_t count)
// {
// 	volatile uint32_t *epaddr = (uint32_t *)AMEBASMART_USB_COUNT_TX(epno);
// 	*epaddr = count;
// }

// /****************************************************************************
//  * Name: amebasmart_seteptxaddr
//  ****************************************************************************/

// static inline void amebasmart_seteptxaddr(uint8_t epno, uint16_t addr)
// {
// 	volatile uint32_t *txaddr = (uint32_t *)AMEBASMART_USB_ADDR_TX(epno);
// 	*txaddr = addr;
// }

// /****************************************************************************
//  * Name: amebasmart_geteptxaddr
//  ****************************************************************************/

// static inline uint16_t amebasmart_geteptxaddr(uint8_t epno)
// {
// 	volatile uint32_t *txaddr = (uint32_t *)AMEBASMART_USB_ADDR_TX(epno);
// 	return (uint16_t)*txaddr;
// }

// /****************************************************************************
//  * Name: amebasmart_seteprxcount
//  ****************************************************************************/

// static void amebasmart_seteprxcount(uint8_t epno, uint16_t count)
// {
// 	volatile uint32_t *epaddr = (uint32_t *)AMEBASMART_USB_COUNT_RX(epno);
// 	uint32_t rxcount = 0;
// 	uint16_t nblocks;

// 	/* The upper bits of the RX COUNT value contain the size of allocated
// 	 * RX buffer.  This is based on a block size of 2 or 32:
// 	 *
// 	 * USB_COUNT_RX_BL_SIZE not set:
// 	 *   nblocks is in units of 2 bytes.
// 	 *     00000 - not allowed
// 	 *     00001 - 2 bytes
// 	 *     ....
// 	 *     11111 - 62 bytes
// 	 *
// 	 * USB_COUNT_RX_BL_SIZE set:
// 	 *     00000 - 32 bytes
// 	 *     00001 - 64 bytes
// 	 *     ...
// 	 *     01111 - 512 bytes
// 	 *     1xxxx - Not allowed
// 	 */

// 	if (count > 62) {
// 		/* Blocks of 32 (with 0 meaning one block of 32) */

// 		nblocks = (count >> 5) - 1;
// 		DEBUGASSERT(nblocks <= 0x0f);
// 		rxcount = (uint32_t)((nblocks << USB_COUNT_RX_NUM_BLOCK_SHIFT) | USB_COUNT_RX_BL_SIZE);
// 	} else if (count > 0) {
// 		/* Blocks of 2 (with 1 meaning one block of 2) */

// 		nblocks = (count + 1) >> 1;
// 		DEBUGASSERT(nblocks > 0 && nblocks < 0x1f);
// 		rxcount = (uint32_t)(nblocks << USB_COUNT_RX_NUM_BLOCK_SHIFT);
// 	}
// 	*epaddr = rxcount;
// }

// /****************************************************************************
//  * Name: amebasmart_geteprxcount
//  ****************************************************************************/

// static inline uint16_t amebasmart_geteprxcount(uint8_t epno)
// {
// 	volatile uint32_t *epaddr = (uint32_t *)AMEBASMART_USB_COUNT_RX(epno);
// 	return (*epaddr) & USB_COUNT_RX_MASK;
// }

// /****************************************************************************
//  * Name: amebasmart_seteprxaddr
//  ****************************************************************************/

// static inline void amebasmart_seteprxaddr(uint8_t epno, uint16_t addr)
// {
// 	volatile uint32_t *rxaddr = (uint32_t *)AMEBASMART_USB_ADDR_RX(epno);
// 	*rxaddr = addr;
// }

// /****************************************************************************
//  * Name: amebasmart_seteprxaddr
//  ****************************************************************************/

// static inline uint16_t amebasmart_geteprxaddr(uint8_t epno)
// {
// 	volatile uint32_t *rxaddr = (uint32_t *)AMEBASMART_USB_ADDR_RX(epno);
// 	return (uint16_t)*rxaddr;
// }

// /****************************************************************************
//  * Name: amebasmart_setepaddress
//  ****************************************************************************/

// static inline void amebasmart_setepaddress(uint8_t epno, uint16_t addr)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	regval = amebasmart_getreg(epaddr);
// 	regval &= EPR_NOTOG_MASK;
// 	regval &= ~USB_EPR_EA_MASK;
// 	regval |= (addr << USB_EPR_EA_SHIFT);
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_seteptype
//  ****************************************************************************/

// static inline void amebasmart_seteptype(uint8_t epno, uint16_t type)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	regval = amebasmart_getreg(epaddr);
// 	regval &= EPR_NOTOG_MASK;
// 	regval &= ~USB_EPR_EPTYPE_MASK;
// 	regval |= type;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_setstatusout
//  ****************************************************************************/

// static inline void amebasmart_setstatusout(uint8_t epno)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	/* For a BULK endpoint the EP_KIND bit is used to enabled double buffering;
// 	 * for a CONTROL endpoint, it is set to indicate that a status OUT
// 	 * transaction is expected.  The bit is not used with out endpoint types.
// 	 */

// 	regval = amebasmart_getreg(epaddr);
// 	regval &= EPR_NOTOG_MASK;
// 	regval |= USB_EPR_EP_KIND;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_clrstatusout
//  ****************************************************************************/

// static inline void amebasmart_clrstatusout(uint8_t epno)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	/* For a BULK endpoint the EP_KIND bit is used to enabled double buffering;
// 	 * for a CONTROL endpoint, it is set to indicate that a status OUT
// 	 * transaction is expected.  The bit is not used with out endpoint types.
// 	 */

// 	regval = amebasmart_getreg(epaddr);
// 	regval &= EPR_NOTOG_MASK;
// 	regval &= ~USB_EPR_EP_KIND;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_clrrxdtog
//  ****************************************************************************/

// static void amebasmart_clrrxdtog(uint8_t epno)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	regval = amebasmart_getreg(epaddr);
// 	if ((regval & USB_EPR_DTOG_RX) != 0) {
// 		regval &= EPR_NOTOG_MASK;
// 		regval |= USB_EPR_DTOG_RX;
// 		amebasmart_putreg(regval, epaddr);
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_clrtxdtog
//  ****************************************************************************/

// static void amebasmart_clrtxdtog(uint8_t epno)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	regval = amebasmart_getreg(epaddr);
// 	if ((regval & USB_EPR_DTOG_TX) != 0) {
// 		regval &= EPR_NOTOG_MASK;
// 		regval |= USB_EPR_DTOG_TX;
// 		amebasmart_putreg(regval, epaddr);
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_clrepctrrx
//  ****************************************************************************/

// static void amebasmart_clrepctrrx(uint8_t epno)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	regval = amebasmart_getreg(epaddr);
// 	regval &= EPR_NOTOG_MASK;
// 	regval &= ~USB_EPR_CTR_RX;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_clrepctrtx
//  ****************************************************************************/

// static void amebasmart_clrepctrtx(uint8_t epno)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	regval = amebasmart_getreg(epaddr);
// 	regval &= EPR_NOTOG_MASK;
// 	regval &= ~USB_EPR_CTR_TX;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_geteptxstatus
//  ****************************************************************************/

// static inline uint16_t amebasmart_geteptxstatus(uint8_t epno)
// {
// 	return (uint16_t)(amebasmart_getreg(AMEBASMART_USB_EPR(epno)) & USB_EPR_STATTX_MASK);
// }

// /****************************************************************************
//  * Name: amebasmart_geteprxstatus
//  ****************************************************************************/

// static inline uint16_t amebasmart_geteprxstatus(uint8_t epno)
// {
// 	return (amebasmart_getreg(AMEBASMART_USB_EPR(epno)) & USB_EPR_STATRX_MASK);
// }

// /****************************************************************************
//  * Name: amebasmart_seteptxstatus
//  ****************************************************************************/

// static void amebasmart_seteptxstatus(uint8_t epno, uint16_t state)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	/* The bits in the STAT_TX field can be toggled by software to set their
// 	 * value. When set to 0, the value remains unchanged; when set to one,
// 	 * value toggles.
// 	 */

// 	regval = amebasmart_getreg(epaddr);

// 	/* The exclusive OR will set STAT_TX bits to 1 if there value is different
// 	 * from the bits requested in 'state'
// 	 */

// 	regval ^= state;
// 	regval &= EPR_TXDTOG_MASK;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_seteprxstatus
//  ****************************************************************************/

// static void amebasmart_seteprxstatus(uint8_t epno, uint16_t state)
// {
// 	uint32_t epaddr = AMEBASMART_USB_EPR(epno);
// 	uint16_t regval;

// 	/* The bits in the STAT_RX field can be toggled by software to set their
// 	 * value. When set to 0, the value remains unchanged; when set to one,
// 	 * value toggles.
// 	 */

// 	regval = amebasmart_getreg(epaddr);

// 	/* The exclusive OR will set STAT_RX bits to 1 if there value is different
// 	 * from the bits requested in 'state'
// 	 */

// 	regval ^= state;
// 	regval &= EPR_RXDTOG_MASK;
// 	amebasmart_putreg(regval, epaddr);
// }

// /****************************************************************************
//  * Name: amebasmart_eptxstalled
//  ****************************************************************************/

// static inline bool amebasmart_eptxstalled(uint8_t epno)
// {
// 	return (amebasmart_geteptxstatus(epno) == USB_EPR_STATTX_STALL);
// }

// /****************************************************************************
//  * Name: amebasmart_eprxstalled
//  ****************************************************************************/

// static inline bool amebasmart_eprxstalled(uint8_t epno)
// {
// 	return (amebasmart_geteprxstatus(epno) == USB_EPR_STATRX_STALL);
// }

// /****************************************************************************
//  * Request Helpers
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_copytopma
//  ****************************************************************************/

// static void amebasmart_copytopma(const uint8_t *buffer, uint16_t pma, uint16_t nbytes)
// {
// 	uint16_t *dest;
// 	uint16_t ms;
// 	uint16_t ls;
// 	int nwords = (nbytes + 1) >> 1;
// 	int i;

// 	/* Copy loop.  Source=user buffer, Dest=packet memory */

// 	dest = (uint16_t *)(AMEBASMART_USBRAM_BASE + ((uint32_t)pma << 1));
// 	for (i = nwords; i != 0; i--) {
// 		/* Read two bytes and pack into on 16-bit word */

// 		ls = (uint16_t)(*buffer++);
// 		ms = (uint16_t)(*buffer++);
// 		*dest = ms << 8 | ls;

// 		/* Source address increments by 2*sizeof(uint8_t) = 2; Dest address
// 		 * increments by 2*sizeof(uint16_t) = 4.
// 		 */

// 		dest += 2;
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_copyfrompma
//  ****************************************************************************/

// static inline void amebasmart_copyfrompma(uint8_t *buffer, uint16_t pma, uint16_t nbytes)
// {
// 	uint32_t *src;
// 	int nwords = (nbytes + 1) >> 1;
// 	int i;

// 	/* Copy loop.  Source=packet memory, Dest=user buffer */

// 	src = (uint32_t *)(AMEBASMART_USBRAM_BASE + ((uint32_t)pma << 1));
// 	for (i = nwords; i != 0; i--) {
// 		/* Copy 16-bits from packet memory to user buffer. */

// 		*(uint16_t *)buffer = *src++;

// 		/* Source address increments by 1*sizeof(uint32_t) = 4; Dest address
// 		 * increments by 2*sizeof(uint8_t) = 2.
// 		 */

// 		buffer += 2;
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_rqdequeue
//  ****************************************************************************/

// static struct amebasmart_req_s *amebasmart_rqdequeue(struct amebasmart_ep_s *privep)
// {
// 	struct amebasmart_req_s *ret = privep->head;

// 	if (ret) {
// 		privep->head = ret->flink;
// 		if (!privep->head) {
// 			privep->tail = NULL;
// 		}

// 		ret->flink = NULL;
// 	}

// 	return ret;
// }

// /****************************************************************************
//  * Name: amebasmart_rqenqueue
//  ****************************************************************************/

// static void amebasmart_rqenqueue(struct amebasmart_ep_s *privep, struct amebasmart_req_s *req)
// {
// 	req->flink = NULL;
// 	if (!privep->head) {
// 		privep->head = req;
// 		privep->tail = req;
// 	} else {
// 		privep->tail->flink = req;
// 		privep->tail = req;
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_abortrequest
//  ****************************************************************************/

// static inline void amebasmart_abortrequest(struct amebasmart_ep_s *privep, struct amebasmart_req_s *privreq, int16_t result)
// {
// 	usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_REQABORTED), (uint16_t)USB_EPNO(privep->ep.eplog));

// 	/* Save the result in the request structure */

// 	privreq->req.result = result;

// 	/* Callback to the request completion handler */

// 	privreq->req.callback(&privep->ep, &privreq->req);
// }

// /****************************************************************************
//  * Name: amebasmart_reqcomplete
//  ****************************************************************************/

// static void amebasmart_reqcomplete(struct amebasmart_ep_s *privep, int16_t result)
// {
// 	struct amebasmart_req_s *privreq;
// 	irqstate_t flags;

// 	/* Remove the completed request at the head of the endpoint request list */

// 	flags = enter_critical_section();
// 	privreq = amebasmart_rqdequeue(privep);
// 	leave_critical_section(flags);

// 	if (privreq) {
// 		/* If endpoint 0, temporarily reflect the state of protocol stalled
// 		 * in the callback.
// 		 */

// 		bool stalled = privep->stalled;
// 		if (USB_EPNO(privep->ep.eplog) == EP0) {
// 			privep->stalled = (privep->dev->ep0state == EP0STATE_STALLED);
// 		}

// 		/* Save the result in the request structure */

// 		privreq->req.result = result;

// 		/* Callback to the request completion handler */

// 		privreq->flink = NULL;
// 		privreq->req.callback(&privep->ep, &privreq->req);

// 		/* Restore the stalled indication */

// 		privep->stalled = stalled;
// 	}
// }

// /****************************************************************************
//  * Name: tm32_epwrite
//  ****************************************************************************/

// static void amebasmart_epwrite(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep, const uint8_t *buf, uint32_t nbytes)
// {
// 	uint8_t epno = USB_EPNO(privep->ep.eplog);
// 	usbtrace(TRACE_WRITE(epno), nbytes);

// 	/* Check for a zero-length packet */

// 	if (nbytes > 0) {
// 		/* Copy the data from the user buffer into packet memory for this
// 		 * endpoint
// 		 */

// 		amebasmart_copytopma(buf, amebasmart_geteptxaddr(epno), nbytes);
// 	}

// 	/* Send the packet (might be a null packet nbytes == 0) */

// 	amebasmart_seteptxcount(epno, nbytes);
// 	priv->txstatus = USB_EPR_STATTX_VALID;

// 	/* Indicate that there is data in the TX packet memory.  This will be cleared
// 	 * when the next data out interrupt is received.
// 	 */

// 	privep->txbusy = true;
// }

// /****************************************************************************
//  * Name: amebasmart_wrrequest_ep0
//  *
//  * Description:
//  *   Handle the ep0 state on writes.
//  *
//  *****************************************************************************/

// inline static int amebasmart_wrrequest_ep0(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep)
// {
// 	int ret;
// 	ret = amebasmart_wrrequest(priv, privep);
// 	priv->ep0state = ((ret == OK) ? EP0STATE_WRREQUEST : EP0STATE_IDLE);
// 	return ret;
// }

// /****************************************************************************
//  * Name: amebasmart_wrrequest
//  ****************************************************************************/

// static int amebasmart_wrrequest(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep)
// {
// 	struct amebasmart_req_s *privreq;
// 	uint8_t *buf;
// 	uint8_t epno;
// 	int nbytes;
// 	int bytesleft;

// 	/* We get here when an IN endpoint interrupt occurs.  So now we know that
// 	 * there is no TX transfer in progress.
// 	 */

// 	privep->txbusy = false;

// 	/* Check the request from the head of the endpoint request queue */

// 	privreq = amebasmart_rqpeek(privep);
// 	if (!privreq) {
// 		/* There is no TX transfer in progress and no new pending TX
// 		 * requests to send.
// 		 */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EPINQEMPTY), 0);
// 		return -ENOENT;
// 	}

// 	epno = USB_EPNO(privep->ep.eplog);
// 	ullvdbg("epno=%d req=%p: len=%d xfrd=%d nullpkt=%d\n", epno, privreq, privreq->req.len, privreq->req.xfrd, privep->txnullpkt);
// 	UNUSED(epno);

// 	/* Get the number of bytes left to be sent in the packet */

// 	bytesleft = privreq->req.len - privreq->req.xfrd;
// 	nbytes = bytesleft;

// #warning "REVISIT: If the EP supports double buffering, then we can do better"

// 	/* Either (1) we are committed to sending the null packet (because txnullpkt == 1
// 	 * && nbytes == 0), or (2) we have not yet send the last packet (nbytes > 0).
// 	 * In either case, it is appropriate to clearn txnullpkt now.
// 	 */

// 	privep->txnullpkt = 0;

// 	/* If we are not sending a NULL packet, then clip the size to maxpacket
// 	 * and check if we need to send a following NULL packet.
// 	 */

// 	if (nbytes > 0) {
// 		/* Either send the maxpacketsize or all of the remaining data in
// 		 * the request.
// 		 */

// 		if (nbytes >= privep->ep.maxpacket) {
// 			nbytes = privep->ep.maxpacket;

// 			/* Handle the case where this packet is exactly the
// 			 * maxpacketsize.  Do we need to send a zero-length packet
// 			 * in this case?
// 			 */

// 			if (bytesleft == privep->ep.maxpacket && (privreq->req.flags & USBDEV_REQFLAGS_NULLPKT) != 0) {
// 				privep->txnullpkt = 1;
// 			}
// 		}
// 	}

// 	/* Send the packet (might be a null packet nbytes == 0) */

// 	buf = privreq->req.buf + privreq->req.xfrd;
// 	amebasmart_epwrite(priv, privep, buf, nbytes);

// 	/* Update for the next data IN interrupt */

// 	privreq->req.xfrd += nbytes;
// 	bytesleft = privreq->req.len - privreq->req.xfrd;

// 	/* If all of the bytes were sent (including any final null packet)
// 	 * then we are finished with the request buffer).
// 	 */

// 	if (bytesleft == 0 && !privep->txnullpkt) {
// 		/* Return the write request to the class driver */

// 		usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);
// 		privep->txnullpkt = 0;
// 		amebasmart_reqcomplete(privep, OK);
// 	}

// 	return OK;
// }

// /*******************************************************************************
//  * Name: amebasmart_ep0_rdrequest
//  *
//  * Description:
//  *   This function is called from the amebasmart_ep0out handler when the ep0state
//  *   is EP0STATE_SETUP_OUT and uppon new incoming data is available in the endpoint
//  *   0's buffer.  This function will simply copy the OUT data into ep0data.
//  *
//  *******************************************************************************/

// static inline int amebasmart_ep0_rdrequest(struct amebasmart_usbdev_s *priv)
// {
// 	uint32_t src;
// 	int pmalen;
// 	int readlen;

// 	/* Get the number of bytes to read from packet memory */

// 	pmalen = amebasmart_geteprxcount(EP0);

// 	ullvdbg("EP0: pmalen=%d\n", pmalen);
// 	usbtrace(TRACE_READ(EP0), pmalen);

// 	/* Read the data into our special buffer for SETUP data */

// 	readlen = MIN(CONFIG_USBDEV_SETUP_MAXDATASIZE, pmalen);
// 	src = amebasmart_geteprxaddr(EP0);

// 	/* Receive the next packet */

// 	amebasmart_copyfrompma(&priv->ep0data[0], src, readlen);

// 	/* Now we can process the setup command */

// 	priv->ep0state = EP0STATE_SETUP_READY;
// 	priv->ep0datlen = readlen;
// 	usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0SETUPOUTDATA), readlen);

// 	amebasmart_ep0setup(priv);
// 	priv->ep0datlen = 0;		/* mark the date consumed */

// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_rdrequest
//  ****************************************************************************/

// static int amebasmart_rdrequest(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep)
// {
// 	struct amebasmart_req_s *privreq;
// 	uint32_t src;
// 	uint8_t *dest;
// 	uint8_t epno;
// 	int pmalen;
// 	int readlen;

// 	/* Check the request from the head of the endpoint request queue */

// 	epno = USB_EPNO(privep->ep.eplog);
// 	privreq = amebasmart_rqpeek(privep);
// 	if (!privreq) {
// 		/* Incoming data available in PMA, but no packet to receive the data.
// 		 * Mark that the RX data is pending and hope that a packet is returned
// 		 * soon.
// 		 */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EPOUTQEMPTY), epno);
// 		return -ENOENT;
// 	}

// 	ullvdbg("EP%d: len=%d xfrd=%d\n", epno, privreq->req.len, privreq->req.xfrd);

// 	/* Ignore any attempt to receive a zero length packet */

// 	if (privreq->req.len == 0) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_EPOUTNULLPACKET), 0);
// 		amebasmart_reqcomplete(privep, OK);
// 		return OK;
// 	}

// 	usbtrace(TRACE_READ(USB_EPNO(privep->ep.eplog)), privreq->req.xfrd);

// 	/* Get the source and destination transfer addresses */

// 	dest = privreq->req.buf + privreq->req.xfrd;
// 	src = amebasmart_geteprxaddr(epno);

// 	/* Get the number of bytes to read from packet memory */

// 	pmalen = amebasmart_geteprxcount(epno);
// 	readlen = MIN(privreq->req.len, pmalen);

// 	/* Receive the next packet */

// 	amebasmart_copyfrompma(dest, src, readlen);

// 	/* If the receive buffer is full or this is a partial packet,
// 	 * then we are finished with the request buffer).
// 	 */

// 	privreq->req.xfrd += readlen;
// 	if (pmalen < privep->ep.maxpacket || privreq->req.xfrd >= privreq->req.len) {
// 		/* Return the read request to the class driver. */

// 		usbtrace(TRACE_COMPLETE(epno), privreq->req.xfrd);
// 		amebasmart_reqcomplete(privep, OK);
// 	}

// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_cancelrequests
//  ****************************************************************************/

// static void amebasmart_cancelrequests(struct amebasmart_ep_s *privep)
// {
// 	while (!amebasmart_rqempty(privep)) {
// 		usbtrace(TRACE_COMPLETE(USB_EPNO(privep->ep.eplog)), (amebasmart_rqpeek(privep))->req.xfrd);
// 		amebasmart_reqcomplete(privep, -ESHUTDOWN);
// 	}
// }

// /****************************************************************************
//  * Interrupt Level Processing
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_dispatchrequest
//  ****************************************************************************/

// static void amebasmart_dispatchrequest(struct amebasmart_usbdev_s *priv)
// {
// 	int ret;

// 	usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_DISPATCH), 0);
// 	if (priv && priv->driver) {
// 		/* Forward to the control request to the class driver implementation */

// 		ret = CLASS_SETUP(priv->driver, &priv->usbdev, &priv->ctrl, priv->ep0data, priv->ep0datlen);
// 		if (ret < 0) {
// 			/* Stall on failure */

// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_DISPATCHSTALL), 0);
// 			priv->ep0state = EP0STATE_STALLED;
// 		}
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_epdone
//  ****************************************************************************/

// static void amebasmart_epdone(struct amebasmart_usbdev_s *priv, uint8_t epno)
// {
// 	struct amebasmart_ep_s *privep;
// 	uint16_t epr;

// 	/* Decode and service non control endpoints interrupt */

// 	epr = amebasmart_getreg(AMEBASMART_USB_EPR(epno));
// 	privep = &priv->eplist[epno];

// 	/* OUT: host-to-device
// 	 * CTR_RX is set by the hardware when an OUT/SETUP transaction
// 	 * successfully completed on this endpoint.
// 	 */

// 	if ((epr & USB_EPR_CTR_RX) != 0) {
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EPOUTDONE), epr);

// 		/* Handle read requests.  First check if a read request is available to
// 		 * accept the host data.
// 		 */

// 		if (!amebasmart_rqempty(privep)) {
// 			/* Read host data into the current read request */

// 			(void)amebasmart_rdrequest(priv, privep);

// 			/* "After the received data is processed, the application software
// 			 *  should set the STAT_RX bits to '11' (Valid) in the USB_EPnR,
// 			 *  enabling further transactions. "
// 			 */

// 			priv->rxstatus = USB_EPR_STATRX_VALID;
// 		}

// 		/* NAK further OUT packets if there there no more read requests */

// 		if (amebasmart_rqempty(privep)) {
// 			usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EPOUTPENDING), (uint16_t)epno);

// 			/* Mark the RX processing as pending and NAK any OUT actions
// 			 * on this endpoint.  "While the STAT_RX bits are equal to '10'
// 			 * (NAK), any OUT request addressed to that endpoint is NAKed,
// 			 * indicating a flow control condition: the USB host will retry
// 			 * the transaction until it succeeds."
// 			 */

// 			priv->rxstatus = USB_EPR_STATRX_NAK;
// 			priv->rxpending = true;
// 		}

// 		/* Clear the interrupt status and set the new RX status */

// 		amebasmart_clrepctrrx(epno);
// 		amebasmart_seteprxstatus(epno, priv->rxstatus);
// 	}

// 	/* IN: device-to-host
// 	 * CTR_TX is set when an IN transaction successfully completes on
// 	 * an endpoint
// 	 */

// 	else if ((epr & USB_EPR_CTR_TX) != 0) {
// 		/* Clear interrupt status */

// 		amebasmart_clrepctrtx(epno);
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EPINDONE), epr);

// 		/* Handle write requests */

// 		priv->txstatus = USB_EPR_STATTX_NAK;
// 		if (epno == EP0) {
// 			(void)amebasmart_wrrequest_ep0(priv, privep);
// 		} else {
// 			(void)amebasmart_wrrequest(priv, privep);
// 		}

// 		/* Set the new TX status */

// 		amebasmart_seteptxstatus(epno, priv->txstatus);
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_setdevaddr
//  ****************************************************************************/

// static void amebasmart_setdevaddr(struct amebasmart_usbdev_s *priv, uint8_t value)
// {
// 	int epno;

// 	/* Set address in every allocated endpoint */

// 	for (epno = 0; epno < AMEBASMART_NENDPOINTS; epno++) {
// 		if (amebasmart_epreserved(priv, epno)) {
// 			amebasmart_setepaddress((uint8_t)epno, (uint8_t)epno);
// 		}
// 	}

// 	/* Set the device address and enable function */

// 	amebasmart_putreg(value | USB_DADDR_EF, AMEBASMART_USB_DADDR);
// }

// /****************************************************************************
//  * Name: amebasmart_ep0setup
//  ****************************************************************************/

// static void amebasmart_ep0setup(struct amebasmart_usbdev_s *priv)
// {
// 	struct amebasmart_ep_s *ep0 = &priv->eplist[EP0];
// 	struct amebasmart_req_s *privreq = amebasmart_rqpeek(ep0);
// 	struct amebasmart_ep_s *privep;
// 	union wb_u value;
// 	union wb_u index;
// 	union wb_u len;
// 	union wb_u response;
// 	bool handled = false;
// 	uint8_t epno;
// 	int nbytes = 0;				/* Assume zero-length packet */

// 	/* Terminate any pending requests (doesn't work if the pending request
// 	 * was a zero-length transfer!)
// 	 */

// 	while (!amebasmart_rqempty(ep0)) {
// 		int16_t result = OK;
// 		if (privreq->req.xfrd != privreq->req.len) {
// 			result = -EPROTO;
// 		}

// 		usbtrace(TRACE_COMPLETE(ep0->ep.eplog), privreq->req.xfrd);
// 		amebasmart_reqcomplete(ep0, result);
// 	}

// 	/* Assume NOT stalled; no TX in progress */

// 	ep0->stalled = 0;
// 	ep0->txbusy = 0;

// 	/* Check to see if called from the DATA phase of a SETUP Transfer */

// 	if (priv->ep0state != EP0STATE_SETUP_READY) {
// 		/* Not the data phase */
// 		/* Get a 32-bit PMA address and use that to get the 8-byte setup request */

// 		amebasmart_copyfrompma((uint8_t *)&priv->ctrl, amebasmart_geteprxaddr(EP0), USB_SIZEOF_CTRLREQ);

// 		/* And extract the little-endian 16-bit values to host order */

// 		value.w = GETUINT16(priv->ctrl.value);
// 		index.w = GETUINT16(priv->ctrl.index);
// 		len.w = GETUINT16(priv->ctrl.len);

// 		ullvdbg("SETUP: type=%02x req=%02x value=%04x index=%04x len=%04x\n", priv->ctrl.type, priv->ctrl.req, value.w, index.w, len.w);

// 		/* Is this an setup with OUT and data of length > 0 */

// 		if (USB_REQ_ISOUT(priv->ctrl.type) && len.w > 0) {

// 			usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0SETUPOUT), len.w);

// 			/* At this point priv->ctrl is the setup packet. */

// 			priv->ep0state = EP0STATE_SETUP_OUT;
// 			return;
// 		} else {
// 			priv->ep0state = EP0STATE_SETUP_READY;
// 		}
// 	}

// 	/* Dispatch any non-standard requests */

// 	if ((priv->ctrl.type & USB_REQ_TYPE_MASK) != USB_REQ_TYPE_STANDARD) {
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_NOSTDREQ), priv->ctrl.type);

// 		/* Let the class implementation handle all non-standar requests */

// 		amebasmart_dispatchrequest(priv);
// 		return;
// 	}

// 	/* Handle standard request.  Pick off the things of interest to the
// 	 * USB device controller driver; pass what is left to the class driver
// 	 */

// 	switch (priv->ctrl.req) {
// 	case USB_REQ_GETSTATUS: {
// 		/* type:  device-to-host; recipient = device, interface, endpoint
// 		 * value: 0
// 		 * index: zero interface endpoint
// 		 * len:   2; data = status
// 		 */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_GETSTATUS), priv->ctrl.type);
// 		if (len.w != 2 || (priv->ctrl.type & USB_REQ_DIR_IN) == 0 || index.b[MSB] != 0 || value.w != 0) {
// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADEPGETSTATUS), 0);
// 			priv->ep0state = EP0STATE_STALLED;
// 		} else {
// 			switch (priv->ctrl.type & USB_REQ_RECIPIENT_MASK) {
// 			case USB_REQ_RECIPIENT_ENDPOINT: {
// 				epno = USB_EPNO(index.b[LSB]);
// 				usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EPGETSTATUS), epno);
// 				if (epno >= AMEBASMART_NENDPOINTS) {
// 					usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADEPGETSTATUS), epno);
// 					priv->ep0state = EP0STATE_STALLED;
// 				} else {
// 					response.w = 0;	/* Not stalled */
// 					nbytes = 2;	/* Response size: 2 bytes */

// 					if (USB_ISEPIN(index.b[LSB])) {
// 						/* IN endpoint */

// 						if (amebasmart_eptxstalled(epno)) {
// 							/* IN Endpoint stalled */

// 							response.b[LSB] = 1;	/* Stalled */
// 						}
// 					} else {
// 						/* OUT endpoint */

// 						if (amebasmart_eprxstalled(epno)) {
// 							/* OUT Endpoint stalled */

// 							response.b[LSB] = 1;	/* Stalled */
// 						}
// 					}
// 				}
// 			}
// 			break;

// 			case USB_REQ_RECIPIENT_DEVICE: {
// 				if (index.w == 0) {
// 					usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_DEVGETSTATUS), 0);

// 					/* Features:  Remote Wakeup=YES; selfpowered=? */

// 					response.w = 0;
// 					response.b[LSB] = (priv->selfpowered << USB_FEATURE_SELFPOWERED) | (1 << USB_FEATURE_REMOTEWAKEUP);
// 					nbytes = 2;	/* Response size: 2 bytes */
// 				} else {
// 					usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADDEVGETSTATUS), 0);
// 					priv->ep0state = EP0STATE_STALLED;
// 				}
// 			}
// 			break;

// 			case USB_REQ_RECIPIENT_INTERFACE: {
// 				usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_IFGETSTATUS), 0);
// 				response.w = 0;
// 				nbytes = 2;	/* Response size: 2 bytes */
// 			}
// 			break;

// 			default: {
// 				usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADGETSTATUS), 0);
// 				priv->ep0state = EP0STATE_STALLED;
// 			}
// 			break;
// 			}
// 		}
// 	}
// 	break;

// 	case USB_REQ_CLEARFEATURE: {
// 		/* type:  host-to-device; recipient = device, interface or endpoint
// 		 * value: feature selector
// 		 * index: zero interface endpoint;
// 		 * len:   zero, data = none
// 		 */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_CLEARFEATURE), priv->ctrl.type);
// 		if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT) {
// 			/* Let the class implementation handle all recipients (except for the
// 			 * endpoint recipient)
// 			 */

// 			amebasmart_dispatchrequest(priv);
// 			handled = true;
// 		} else {
// 			/* Endpoint recipient */

// 			epno = USB_EPNO(index.b[LSB]);
// 			if (epno < AMEBASMART_NENDPOINTS && index.b[MSB] == 0 && value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0) {
// 				privep = &priv->eplist[epno];
// 				privep->halted = 0;
// 				(void)amebasmart_epstall(&privep->ep, true);
// 			} else {
// 				usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADCLEARFEATURE), 0);
// 				priv->ep0state = EP0STATE_STALLED;
// 			}
// 		}
// 	}
// 	break;

// 	case USB_REQ_SETFEATURE: {
// 		/* type:  host-to-device; recipient = device, interface, endpoint
// 		 * value: feature selector
// 		 * index: zero interface endpoint;
// 		 * len:   0; data = none
// 		 */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_SETFEATURE), priv->ctrl.type);
// 		if (((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE) && value.w == USB_FEATURE_TESTMODE) {
// 			/* Special case recipient=device test mode */

// 			ullvdbg("test mode: %d\n", index.w);
// 		} else if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_ENDPOINT) {
// 			/* The class driver handles all recipients except recipient=endpoint */

// 			amebasmart_dispatchrequest(priv);
// 			handled = true;
// 		} else {
// 			/* Handler recipient=endpoint */

// 			epno = USB_EPNO(index.b[LSB]);
// 			if (epno < AMEBASMART_NENDPOINTS && index.b[MSB] == 0 && value.w == USB_FEATURE_ENDPOINTHALT && len.w == 0) {
// 				privep = &priv->eplist[epno];
// 				privep->halted = 1;
// 				(void)amebasmart_epstall(&privep->ep, false);
// 			} else {
// 				usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADSETFEATURE), 0);
// 				priv->ep0state = EP0STATE_STALLED;
// 			}
// 		}
// 	}
// 	break;

// 	case USB_REQ_SETADDRESS: {
// 		/* type:  host-to-device; recipient = device
// 		 * value: device address
// 		 * index: 0
// 		 * len:   0; data = none
// 		 */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0SETUPSETADDRESS), value.w);
// 		if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) != USB_REQ_RECIPIENT_DEVICE || index.w != 0 || len.w != 0 || value.w > 127) {
// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADSETADDRESS), 0);
// 			priv->ep0state = EP0STATE_STALLED;
// 		}

// 		/* Note that setting of the device address will be deferred.  A zero-length
// 		 * packet will be sent and the device address will be set when the zero-
// 		 * length packet transfer completes.
// 		 */
// 	}
// 	break;

// 	case USB_REQ_GETDESCRIPTOR:
// 	/* type:  device-to-host; recipient = device
// 	 * value: descriptor type and index
// 	 * index: 0 or language ID;
// 	 * len:   descriptor len; data = descriptor
// 	 */
// 	case USB_REQ_SETDESCRIPTOR:
// 		/* type:  host-to-device; recipient = device
// 		 * value: descriptor type and index
// 		 * index: 0 or language ID;
// 		 * len:   descriptor len; data = descriptor
// 		 */

// 	{
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_GETSETDESC), priv->ctrl.type);
// 		/* The request seems valid... let the class implementation handle it */

// 		amebasmart_dispatchrequest(priv);
// 		handled = true;
// 	}
// 	break;

// 	case USB_REQ_GETCONFIGURATION:
// 		/* type:  device-to-host; recipient = device
// 		 * value: 0;
// 		 * index: 0;
// 		 * len:   1; data = configuration value
// 		 */

// 	{
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_GETCONFIG), priv->ctrl.type);
// 		if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE && value.w == 0 && index.w == 0 && len.w == 1) {
// 			/* The request seems valid... let the class implementation handle it */

// 			amebasmart_dispatchrequest(priv);
// 			handled = true;
// 		} else {
// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADGETCONFIG), 0);
// 			priv->ep0state = EP0STATE_STALLED;
// 		}
// 	}
// 	break;

// 	case USB_REQ_SETCONFIGURATION:
// 		/* type:  host-to-device; recipient = device
// 		 * value: configuration value
// 		 * index: 0;
// 		 * len:   0; data = none
// 		 */

// 	{
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_SETCONFIG), priv->ctrl.type);
// 		if ((priv->ctrl.type & USB_REQ_RECIPIENT_MASK) == USB_REQ_RECIPIENT_DEVICE && index.w == 0 && len.w == 0) {
// 			/* The request seems valid... let the class implementation handle it */

// 			amebasmart_dispatchrequest(priv);
// 			handled = true;
// 		} else {
// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADSETCONFIG), 0);
// 			priv->ep0state = EP0STATE_STALLED;
// 		}
// 	}
// 	break;

// 	case USB_REQ_GETINTERFACE:
// 	/* type:  device-to-host; recipient = interface
// 	 * value: 0
// 	 * index: interface;
// 	 * len:   1; data = alt interface
// 	 */
// 	case USB_REQ_SETINTERFACE:
// 		/* type:  host-to-device; recipient = interface
// 		 * value: alternate setting
// 		 * index: interface;
// 		 * len:   0; data = none
// 		 */

// 	{
// 		/* Let the class implementation handle the request */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_GETSETIF), priv->ctrl.type);
// 		amebasmart_dispatchrequest(priv);
// 		handled = true;
// 	}
// 	break;

// 	case USB_REQ_SYNCHFRAME:
// 		/* type:  device-to-host; recipient = endpoint
// 		 * value: 0
// 		 * index: endpoint;
// 		 * len:   2; data = frame number
// 		 */

// 	{
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_SYNCHFRAME), 0);
// 	}
// 	break;

// 	default: {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDCTRLREQ), priv->ctrl.req);
// 		priv->ep0state = EP0STATE_STALLED;
// 	}
// 	break;
// 	}

// 	/* At this point, the request has been handled and there are three possible
// 	 * outcomes:
// 	 *
// 	 * 1. The setup request was successfully handled above and a response packet
// 	 *    must be sent (may be a zero length packet).
// 	 * 2. The request was successfully handled by the class implementation.  In
// 	 *    case, the EP0 IN response has already been queued and the local variable
// 	 *    'handled' will be set to true and ep0state != EP0STATE_STALLED;
// 	 * 3. An error was detected in either the above logic or by the class implementation
// 	 *    logic.  In either case, priv->state will be set EP0STATE_STALLED
// 	 *    to indicate this case.
// 	 *
// 	 * NOTE: Non-standard requests are a special case.  They are handled by the
// 	 * class implementation and this function returned early above, skipping this
// 	 * logic altogether.
// 	 */

// 	if (priv->ep0state != EP0STATE_STALLED && !handled) {
// 		/* We will response.  First, restrict the data length to the length
// 		 * requested in the setup packet
// 		 */

// 		if (nbytes > len.w) {
// 			nbytes = len.w;
// 		}

// 		/* Send the response (might be a zero-length packet) */

// 		amebasmart_epwrite(priv, ep0, response.b, nbytes);
// 		priv->ep0state = EP0STATE_IDLE;
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_ep0in
//  ****************************************************************************/

// static void amebasmart_ep0in(struct amebasmart_usbdev_s *priv)
// {
// 	/* There is no longer anything in the EP0 TX packet memory */

// 	priv->eplist[EP0].txbusy = false;

// 	/* Are we processing the completion of one packet of an outgoing request
// 	 * from the class driver?
// 	 */

// 	if (priv->ep0state == EP0STATE_WRREQUEST) {
// 		amebasmart_wrrequest_ep0(priv, &priv->eplist[EP0]);
// 	}

// 	/* No.. Are we processing the completion of a status response? */

// 	else if (priv->ep0state == EP0STATE_IDLE) {
// 		/* Look at the saved SETUP command.  Was it a SET ADDRESS request?
// 		 * If so, then now is the time to set the address.
// 		 */

// 		if (priv->ctrl.req == USB_REQ_SETADDRESS && (priv->ctrl.type & REQRECIPIENT_MASK) == (USB_REQ_TYPE_STANDARD | USB_REQ_RECIPIENT_DEVICE)) {
// 			union wb_u value;
// 			value.w = GETUINT16(priv->ctrl.value);
// 			amebasmart_setdevaddr(priv, value.b[LSB]);
// 		}
// 	} else {
// 		priv->ep0state = EP0STATE_STALLED;
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_ep0out
//  ****************************************************************************/

// static void amebasmart_ep0out(struct amebasmart_usbdev_s *priv)
// {
// 	int ret;

// 	struct amebasmart_ep_s *privep = &priv->eplist[EP0];
// 	switch (priv->ep0state) {
// 	case EP0STATE_RDREQUEST:	/* Read request in progress */
// 	case EP0STATE_IDLE:		/* No transfer in progress */
// 		ret = amebasmart_rdrequest(priv, privep);
// 		priv->ep0state = ((ret == OK) ? EP0STATE_RDREQUEST : EP0STATE_IDLE);
// 		break;

// 	case EP0STATE_SETUP_OUT:	/* SETUP was waiting for data */
// 		ret = amebasmart_ep0_rdrequest(priv);	/* Off load the data and run the
// 											 * last set up command with the OUT
// 											 * data
// 											 */
// 		priv->ep0state = EP0STATE_IDLE;	/* There is no notion of reciving OUT
// 										 * data greater then the length of
// 										 * CONFIG_USBDEV_SETUP_MAXDATASIZE
// 										 * so we are done
// 										 */
// 		break;

// 	default:
// 		/* Unexpected state OR host aborted the OUT transfer before it
// 		 * completed, STALL the endpoint in either case  */

// 		priv->ep0state = EP0STATE_STALLED;
// 		break;
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_ep0done
//  ****************************************************************************/

// static inline void amebasmart_ep0done(struct amebasmart_usbdev_s *priv, uint16_t istr)
// {
// 	uint16_t epr;

// 	/* Initialize RX and TX status.  We shouldn't have to actually look at the
// 	 * status because the hardware is supposed to set the both RX and TX status
// 	 * to NAK when an EP0 SETUP occurs (of course, this might not be a setup)
// 	 */

// 	priv->rxstatus = USB_EPR_STATRX_NAK;
// 	priv->txstatus = USB_EPR_STATTX_NAK;

// 	/* Set both RX and TX status to NAK  */

// 	amebasmart_seteprxstatus(EP0, USB_EPR_STATRX_NAK);
// 	amebasmart_seteptxstatus(EP0, USB_EPR_STATTX_NAK);

// 	/* Check the direction bit to determine if this the completion of an EP0
// 	 * packet sent to or received from the host PC.
// 	 */

// 	if ((istr & USB_ISTR_DIR) == 0) {
// 		/* EP0 IN: device-to-host (DIR=0) */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0IN), istr);
// 		amebasmart_clrepctrtx(EP0);
// 		amebasmart_ep0in(priv);
// 	} else {
// 		/* EP0 OUT: host-to-device (DIR=1) */

// 		epr = amebasmart_getreg(AMEBASMART_USB_EPR(EP0));

// 		/* CTR_TX is set when an IN transaction successfully
// 		 * completes on an endpoint
// 		 */

// 		if ((epr & USB_EPR_CTR_TX) != 0) {
// 			usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0INDONE), epr);
// 			amebasmart_clrepctrtx(EP0);
// 			amebasmart_ep0in(priv);
// 		}

// 		/* SETUP is set by the hardware when the last completed
// 		 * transaction was a control endpoint SETUP
// 		 */

// 		else if ((epr & USB_EPR_SETUP) != 0) {
// 			usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0SETUPDONE), epr);
// 			amebasmart_clrepctrrx(EP0);
// 			amebasmart_ep0setup(priv);
// 		}

// 		/* Set by the hardware when an OUT/SETUP transaction successfully
// 		 * completed on this endpoint.
// 		 */

// 		else if ((epr & USB_EPR_CTR_RX) != 0) {
// 			usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_EP0OUTDONE), epr);
// 			amebasmart_clrepctrrx(EP0);
// 			amebasmart_ep0out(priv);
// 		}

// 		/* None of the above */

// 		else {
// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_EP0BADCTR), epr);
// 			return;				/* Does this ever happen? */
// 		}
// 	}

// 	/* Make sure that the EP0 packet size is still OK (superstitious?) */

// 	amebasmart_seteprxcount(EP0, AMEBASMART_EP0MAXPACKET);

// 	/* Now figure out the new RX/TX status.  Here are all possible
// 	 * consequences of the above EP0 operations:
// 	 *
// 	 * rxstatus txstatus ep0state  MEANING
// 	 * -------- -------- --------- ---------------------------------
// 	 * NAK      NAK      IDLE      Nothing happened
// 	 * NAK      VALID    IDLE      EP0 response sent from USBDEV driver
// 	 * NAK      VALID    WRREQUEST EP0 response sent from class driver
// 	 * NAK      ---      STALL     Some protocol error occurred
// 	 *
// 	 * First handle the STALL condition:
// 	 */

// 	if (priv->ep0state == EP0STATE_STALLED) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_EP0SETUPSTALLED), priv->ep0state);
// 		priv->rxstatus = USB_EPR_STATRX_STALL;
// 		priv->txstatus = USB_EPR_STATTX_STALL;
// 	}

// 	/* Was a transmission started?  If so, txstatus will be VALID.  The
// 	 * only special case to handle is when both are set to NAK.  In that
// 	 * case, we need to set RX status to VALID in order to accept the next
// 	 * SETUP request.
// 	 */

// 	else if (priv->rxstatus == USB_EPR_STATRX_NAK && priv->txstatus == USB_EPR_STATTX_NAK) {
// 		priv->rxstatus = USB_EPR_STATRX_VALID;
// 	}

// 	/* Now set the new TX and RX status */

// 	amebasmart_seteprxstatus(EP0, priv->rxstatus);
// 	amebasmart_seteptxstatus(EP0, priv->txstatus);
// }

// /****************************************************************************
//  * Name: amebasmart_lptransfer
//  ****************************************************************************/

// static void amebasmart_lptransfer(struct amebasmart_usbdev_s *priv)
// {
// 	uint8_t epno;
// 	uint16_t istr;

// 	/* Stay in loop while LP interrupts are pending */

// 	while (((istr = amebasmart_getreg(AMEBASMART_USB_ISTR)) & USB_ISTR_CTR) != 0) {
// 		amebasmart_putreg((uint16_t)~USB_ISTR_CTR, AMEBASMART_USB_ISTR);

// 		/* Extract highest priority endpoint number */

// 		epno = (uint8_t)(istr & USB_ISTR_EPID_MASK);

// 		/* Handle EP0 completion events */

// 		if (epno == 0) {
// 			amebasmart_ep0done(priv, istr);
// 		}

// 		/* Handle other endpoint completion events */

// 		else {
// 			amebasmart_epdone(priv, epno);
// 		}
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_hpinterrupt
//  ****************************************************************************/

// static int amebasmart_hpinterrupt(int irq, void *context, FAR void *arg)
// {
// 	/* For now there is only one USB controller, but we will always refer to
// 	 * it using a pointer to make any future ports to multiple USB controllers
// 	 * easier.
// 	 */

// 	struct amebasmart_usbdev_s *priv = &g_usbdev;
// 	uint16_t istr;
// 	uint8_t epno;

// 	/* High priority interrupts are only triggered by a correct transfer event
// 	 * for isochronous and double-buffer bulk transfers.
// 	 */

// 	istr = amebasmart_getreg(AMEBASMART_USB_ISTR);
// 	usbtrace(TRACE_INTENTRY(AMEBASMART_TRACEINTID_HPINTERRUPT), istr);
// 	while ((istr & USB_ISTR_CTR) != 0) {
// 		amebasmart_putreg((uint16_t)~USB_ISTR_CTR, AMEBASMART_USB_ISTR);

// 		/* Extract highest priority endpoint number */

// 		epno = (uint8_t)(istr & USB_ISTR_EPID_MASK);

// 		/* And handle the completion event */

// 		amebasmart_epdone(priv, epno);

// 		/* Fetch the status again for the next time through the loop */

// 		istr = amebasmart_getreg(AMEBASMART_USB_ISTR);
// 	}

// 	usbtrace(TRACE_INTEXIT(AMEBASMART_TRACEINTID_HPINTERRUPT), 0);
// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_lpinterrupt
//  ****************************************************************************/

// static int amebasmart_lpinterrupt(int irq, void *context, FAR void *arg)
// {
// 	/* For now there is only one USB controller, but we will always refer to
// 	 * it using a pointer to make any future ports to multiple USB controllers
// 	 * easier.
// 	 */

// 	struct amebasmart_usbdev_s *priv = &g_usbdev;
// 	uint16_t istr = amebasmart_getreg(AMEBASMART_USB_ISTR);

// 	usbtrace(TRACE_INTENTRY(AMEBASMART_TRACEINTID_LPINTERRUPT), istr);

// 	/* Handle Reset interrupts.  When this event occurs, the peripheral is left
// 	 * in the same conditions it is left by the system reset (but with the
// 	 * USB controller enabled).
// 	 */

// 	if ((istr & USB_ISTR_RESET) != 0) {
// 		/* Reset interrupt received. Clear the RESET interrupt status. */

// 		amebasmart_putreg(~USB_ISTR_RESET, AMEBASMART_USB_ISTR);
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_RESET), istr);

// 		/* Restore our power-up state and exit now because istr is no longer
// 		 * valid.
// 		 */

// 		amebasmart_reset(priv);
// 		goto exit_lpinterrupt;
// 	}

// 	/* Handle Wakeup interrupts.  This interrupt is only enable while the USB is
// 	 * suspended.
// 	 */

// 	if ((istr & USB_ISTR_WKUP & priv->imask) != 0) {
// 		/* Wakeup interrupt received. Clear the WKUP interrupt status.  The
// 		 * cause of the resume is indicated in the FNR register
// 		 */

// 		amebasmart_putreg(~USB_ISTR_WKUP, AMEBASMART_USB_ISTR);
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_WKUP), amebasmart_getreg(AMEBASMART_USB_FNR));

// 		/* Perform the wakeup action */

// 		amebasmart_initresume(priv);
// 		priv->rsmstate = RSMSTATE_IDLE;

// 		/* Disable ESOF polling, disable the wakeup interrupt, and
// 		 * re-enable the suspend interrupt.  Clear any pending SUSP
// 		 * interrupts.
// 		 */

// 		amebasmart_setimask(priv, USB_CNTR_SUSPM, USB_CNTR_ESOFM | USB_CNTR_WKUPM);
// 		amebasmart_putreg(~USB_CNTR_SUSPM, AMEBASMART_USB_ISTR);
// 	}

// 	if ((istr & USB_ISTR_SUSP & priv->imask) != 0) {
// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_SUSP), 0);
// 		amebasmart_suspend(priv);

// 		/* Clear of the ISTR bit must be done after setting of USB_CNTR_FSUSP */

// 		amebasmart_putreg(~USB_ISTR_SUSP, AMEBASMART_USB_ISTR);
// 	}

// 	if ((istr & USB_ISTR_ESOF & priv->imask) != 0) {
// 		amebasmart_putreg(~USB_ISTR_ESOF, AMEBASMART_USB_ISTR);

// 		/* Resume handling timing is made with ESOFs */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_ESOF), 0);
// 		amebasmart_esofpoll(priv);
// 	}

// 	if ((istr & USB_ISTR_CTR & priv->imask) != 0) {
// 		/* Low priority endpoint correct transfer interrupt */

// 		usbtrace(TRACE_INTDECODE(AMEBASMART_TRACEINTID_LPCTR), istr);
// 		amebasmart_lptransfer(priv);
// 	}

// exit_lpinterrupt:
// 	usbtrace(TRACE_INTEXIT(AMEBASMART_TRACEINTID_LPINTERRUPT), amebasmart_getreg(AMEBASMART_USB_EP0R));
// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_setimask
//  ****************************************************************************/

// static void amebasmart_setimask(struct amebasmart_usbdev_s *priv, uint16_t setbits, uint16_t clrbits)
// {
// 	uint16_t regval;

// 	/* Adjust the interrupt mask bits in the shadow copy first */

// 	priv->imask &= ~clrbits;
// 	priv->imask |= setbits;

// 	/* Then make the interrupt mask bits in the CNTR register match the shadow
// 	 * register (Hmmm... who is shadowing whom?)
// 	 */

// 	regval = amebasmart_getreg(AMEBASMART_USB_CNTR);
// 	regval &= ~USB_CNTR_ALLINTS;
// 	regval |= priv->imask;
// 	amebasmart_putreg(regval, AMEBASMART_USB_CNTR);
// }

// /****************************************************************************
//  * Suspend/Resume Helpers
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_suspend
//  ****************************************************************************/

// static void amebasmart_suspend(struct amebasmart_usbdev_s *priv)
// {
// 	uint16_t regval;

// 	/* Notify the class driver of the suspend event */

// 	if (priv->driver) {
// 		CLASS_SUSPEND(priv->driver, &priv->usbdev);
// 	}

// 	/* Disable ESOF polling, disable the SUSP interrupt, and enable the WKUP
// 	 * interrupt.  Clear any pending WKUP interrupt.
// 	 */

// 	amebasmart_setimask(priv, USB_CNTR_WKUPM, USB_CNTR_ESOFM | USB_CNTR_SUSPM);
// 	amebasmart_putreg(~USB_ISTR_WKUP, AMEBASMART_USB_ISTR);

// 	/* Set the FSUSP bit in the CNTR register.  This activates suspend mode
// 	 * within the USB peripheral and disables further SUSP interrupts.
// 	 */

// 	regval = amebasmart_getreg(AMEBASMART_USB_CNTR);
// 	regval |= USB_CNTR_FSUSP;
// 	amebasmart_putreg(regval, AMEBASMART_USB_CNTR);

// 	/* If we are not a self-powered device, the got to low-power mode */

// 	if (!priv->selfpowered) {
// 		/* Setting LPMODE in the CNTR register removes static power
// 		 * consumption in the USB analog transceivers but keeps them
// 		 * able to detect resume activity
// 		 */

// 		regval = amebasmart_getreg(AMEBASMART_USB_CNTR);
// 		regval |= USB_CNTR_LPMODE;
// 		amebasmart_putreg(regval, AMEBASMART_USB_CNTR);
// 	}

// 	/* Let the board-specific logic know that we have entered the suspend
// 	 * state
// 	 */

// 	amebasmart_usbsuspend((struct usbdev_s *)priv, false);
// }

// /****************************************************************************
//  * Name: amebasmart_initresume
//  ****************************************************************************/

// static void amebasmart_initresume(struct amebasmart_usbdev_s *priv)
// {
// 	uint16_t regval;

// 	/* This function is called when either (1) a WKUP interrupt is received from
// 	 * the host PC, or (2) the class device implementation calls the wakeup()
// 	 * method.
// 	 */

// 	/* Clear the USB low power mode (lower power mode was not set if this is
// 	 * a self-powered device.  Also, low power mode is automatically cleared by
// 	 * hardware when a WKUP interrupt event occurs).
// 	 */

// 	regval = amebasmart_getreg(AMEBASMART_USB_CNTR);
// 	regval &= (~USB_CNTR_LPMODE);
// 	amebasmart_putreg(regval, AMEBASMART_USB_CNTR);

// 	/* Restore full power -- whatever that means for this particular board */

// 	amebasmart_usbsuspend((struct usbdev_s *)priv, true);

// 	/* Reset FSUSP bit and enable normal interrupt handling */

// 	amebasmart_putreg(AMEBASMART_CNTR_SETUP, AMEBASMART_USB_CNTR);

// 	/* Notify the class driver of the resume event */

// 	if (priv->driver) {
// 		CLASS_RESUME(priv->driver, &priv->usbdev);
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_esofpoll
//  ****************************************************************************/

// static void amebasmart_esofpoll(struct amebasmart_usbdev_s *priv)
// {
// 	uint16_t regval;

// 	/* Called periodically from ESOF interrupt after RSMSTATE_STARTED */

// 	switch (priv->rsmstate) {
// 	/* One ESOF after internal resume requested */

// 	case RSMSTATE_STARTED:
// 		regval = amebasmart_getreg(AMEBASMART_USB_CNTR);
// 		regval |= USB_CNTR_RESUME;
// 		amebasmart_putreg(regval, AMEBASMART_USB_CNTR);
// 		priv->rsmstate = RSMSTATE_WAITING;
// 		priv->nesofs = 10;
// 		break;

// 	/* Countdown before completing the operation */

// 	case RSMSTATE_WAITING:
// 		priv->nesofs--;
// 		if (priv->nesofs == 0) {
// 			/* Okay.. we are ready to resume normal operation */

// 			regval = amebasmart_getreg(AMEBASMART_USB_CNTR);
// 			regval &= (~USB_CNTR_RESUME);
// 			amebasmart_putreg(regval, AMEBASMART_USB_CNTR);
// 			priv->rsmstate = RSMSTATE_IDLE;

// 			/* Disable ESOF polling, disable the SUSP interrupt, and enable
// 			 * the WKUP interrupt.  Clear any pending WKUP interrupt.
// 			 */

// 			amebasmart_setimask(priv, USB_CNTR_WKUPM, USB_CNTR_ESOFM | USB_CNTR_SUSPM);
// 			amebasmart_putreg(~USB_ISTR_WKUP, AMEBASMART_USB_ISTR);
// 		}
// 		break;

// 	case RSMSTATE_IDLE:
// 	default:
// 		priv->rsmstate = RSMSTATE_IDLE;
// 		break;
// 	}
// }

// /****************************************************************************
//  * Endpoint Helpers
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_epreserve
//  ****************************************************************************/

// static inline struct amebasmart_ep_s *amebasmart_epreserve(struct amebasmart_usbdev_s *priv, uint8_t epset)
// {
// 	struct amebasmart_ep_s *privep = NULL;
// 	irqstate_t flags;
// 	int epndx = 0;

// 	flags = enter_critical_section();
// 	epset &= priv->epavail;
// 	if (epset) {
// 		/* Select the lowest bit in the set of matching, available endpoints
// 		 * (skipping EP0)
// 		 */

// 		for (epndx = 1; epndx < AMEBASMART_NENDPOINTS; epndx++) {
// 			uint8_t bit = AMEBASMART_ENDP_BIT(epndx);
// 			if ((epset & bit) != 0) {
// 				/* Mark the endpoint no longer available */

// 				priv->epavail &= ~bit;

// 				/* And return the pointer to the standard endpoint structure */

// 				privep = &priv->eplist[epndx];
// 				break;
// 			}
// 		}
// 	}

// 	leave_critical_section(flags);
// 	return privep;
// }

// /****************************************************************************
//  * Name: amebasmart_epunreserve
//  ****************************************************************************/

// static inline void amebasmart_epunreserve(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep)
// {
// 	irqstate_t flags = enter_critical_section();
// 	priv->epavail |= AMEBASMART_ENDP_BIT(USB_EPNO(privep->ep.eplog));
// 	leave_critical_section(flags);
// }

// /****************************************************************************
//  * Name: amebasmart_epreserved
//  ****************************************************************************/

// static inline bool amebasmart_epreserved(struct amebasmart_usbdev_s *priv, int epno)
// {
// 	return ((priv->epavail & AMEBASMART_ENDP_BIT(epno)) == 0);
// }

// /****************************************************************************
//  * Name: amebasmart_epallocpma
//  ****************************************************************************/

// static int amebasmart_epallocpma(struct amebasmart_usbdev_s *priv)
// {
// 	irqstate_t flags;
// 	int bufno = ERROR;
// 	int bufndx;

// 	flags = enter_critical_section();
// 	for (bufndx = 2; bufndx < AMEBASMART_NBUFFERS; bufndx++) {
// 		/* Check if this buffer is available */

// 		uint8_t bit = AMEBASMART_BUFFER_BIT(bufndx);
// 		if ((priv->bufavail & bit) != 0) {
// 			/* Yes.. Mark the endpoint no longer available */

// 			priv->bufavail &= ~bit;

// 			/* And return the index of the allocated buffer */

// 			bufno = bufndx;
// 			break;
// 		}
// 	}

// 	leave_critical_section(flags);
// 	return bufno;
// }

// /****************************************************************************
//  * Name: amebasmart_epfreepma
//  ****************************************************************************/

// static inline void amebasmart_epfreepma(struct amebasmart_usbdev_s *priv, struct amebasmart_ep_s *privep)
// {
// 	irqstate_t flags = enter_critical_section();
// 	priv->epavail |= AMEBASMART_ENDP_BIT(privep->bufno);
// 	leave_critical_section(flags);
// }

// /****************************************************************************
//  * Endpoint operations
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_epconfigure
//  ****************************************************************************/

// static int amebasmart_epconfigure(struct usbdev_ep_s *ep, const struct usb_epdesc_s *desc, bool last)
// {
// 	struct amebasmart_ep_s *privep = (struct amebasmart_ep_s *)ep;
// 	uint16_t pma;
// 	uint16_t setting;
// 	uint16_t maxpacket;
// 	uint8_t epno;

// #ifdef CONFIG_DEBUG
// 	if (!ep || !desc) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		ulldbg("ERROR: ep=%p desc=%p\n");
// 		return -EINVAL;
// 	}
// #endif

// 	/* Get the unadorned endpoint address */

// 	epno = USB_EPNO(desc->addr);
// 	usbtrace(TRACE_EPCONFIGURE, (uint16_t)epno);
// 	DEBUGASSERT(epno == USB_EPNO(ep->eplog));

// 	/* Set the requested type */

// 	switch (desc->attr & USB_EP_ATTR_XFERTYPE_MASK) {
// 	case USB_EP_ATTR_XFER_INT:	/* Interrupt endpoint */
// 		setting = USB_EPR_EPTYPE_INTERRUPT;
// 		break;

// 	case USB_EP_ATTR_XFER_BULK:	/* Bulk endpoint */
// 		setting = USB_EPR_EPTYPE_BULK;
// 		break;

// 	case USB_EP_ATTR_XFER_ISOC:	/* Isochronous endpoint */
// #warning "REVISIT: Need to review isochronous EP setup"
// 		setting = USB_EPR_EPTYPE_ISOC;
// 		break;

// 	case USB_EP_ATTR_XFER_CONTROL:	/* Control endpoint */
// 		setting = USB_EPR_EPTYPE_CONTROL;
// 		break;

// 	default:
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADEPTYPE), (uint16_t)desc->type);
// 		return -EINVAL;
// 	}

// 	amebasmart_seteptype(epno, setting);

// 	/* Get the address of the PMA buffer allocated for this endpoint */

// #warning "REVISIT: Should configure BULK EPs using double buffer feature"
// 	pma = AMEBASMART_BUFNO2BUF(privep->bufno);

// 	/* Get the maxpacket size of the endpoint. */

// 	maxpacket = GETUINT16(desc->mxpacketsize);
// 	DEBUGASSERT(maxpacket <= AMEBASMART_MAXPACKET_SIZE);
// 	ep->maxpacket = maxpacket;

// 	/* Get the subset matching the requested direction */

// 	if (USB_ISEPIN(desc->addr)) {
// 		/* The full, logical EP number includes direction */

// 		ep->eplog = USB_EPIN(epno);

// 		/* Set up TX; disable RX */

// 		amebasmart_seteptxaddr(epno, pma);
// 		amebasmart_seteptxstatus(epno, USB_EPR_STATTX_NAK);
// 		amebasmart_seteprxstatus(epno, USB_EPR_STATRX_DIS);
// 	} else {
// 		/* The full, logical EP number includes direction */

// 		ep->eplog = USB_EPOUT(epno);

// 		/* Set up RX; disable TX */

// 		amebasmart_seteprxaddr(epno, pma);
// 		amebasmart_seteprxcount(epno, maxpacket);
// 		amebasmart_seteprxstatus(epno, USB_EPR_STATRX_VALID);
// 		amebasmart_seteptxstatus(epno, USB_EPR_STATTX_DIS);
// 	}

// 	amebasmart_dumpep(epno);
// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_epdisable
//  ****************************************************************************/

// static int amebasmart_epdisable(struct usbdev_ep_s *ep)
// {
// 	struct amebasmart_ep_s *privep = (struct amebasmart_ep_s *)ep;
// 	irqstate_t flags;
// 	uint8_t epno;

// #ifdef CONFIG_DEBUG
// 	if (!ep) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		ulldbg("ERROR: ep=%p\n", ep);
// 		return -EINVAL;
// 	}
// #endif

// 	epno = USB_EPNO(ep->eplog);
// 	usbtrace(TRACE_EPDISABLE, epno);

// 	/* Cancel any ongoing activity */

// 	flags = enter_critical_section();
// 	amebasmart_cancelrequests(privep);

// 	/* Disable TX; disable RX */

// 	amebasmart_seteprxcount(epno, 0);
// 	amebasmart_seteprxstatus(epno, USB_EPR_STATRX_DIS);
// 	amebasmart_seteptxstatus(epno, USB_EPR_STATTX_DIS);

// 	leave_critical_section(flags);
// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_epallocreq
//  ****************************************************************************/

// static struct usbdev_req_s *amebasmart_epallocreq(struct usbdev_ep_s *ep)
// {
// 	struct amebasmart_req_s *privreq;

// #ifdef CONFIG_DEBUG
// 	if (!ep) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return NULL;
// 	}
// #endif
// 	usbtrace(TRACE_EPALLOCREQ, USB_EPNO(ep->eplog));

// 	privreq = (struct amebasmart_req_s *)kmm_malloc(sizeof(struct amebasmart_req_s));
// 	if (!privreq) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_ALLOCFAIL), 0);
// 		return NULL;
// 	}

// 	memset(privreq, 0, sizeof(struct amebasmart_req_s));
// 	return &privreq->req;
// }

// /****************************************************************************
//  * Name: amebasmart_epfreereq
//  ****************************************************************************/

// static void amebasmart_epfreereq(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
// {
// 	struct amebasmart_req_s *privreq = (struct amebasmart_req_s *)req;

// #ifdef CONFIG_DEBUG
// 	if (!ep || !req) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return;
// 	}
// #endif
// 	usbtrace(TRACE_EPFREEREQ, USB_EPNO(ep->eplog));

// 	kmm_free(privreq);
// }

// /****************************************************************************
//  * Name: amebasmart_epsubmit
//  ****************************************************************************/

// static int amebasmart_epsubmit(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
// {
// 	struct amebasmart_req_s *privreq = (struct amebasmart_req_s *)req;
// 	struct amebasmart_ep_s *privep = (struct amebasmart_ep_s *)ep;
// 	struct amebasmart_usbdev_s *priv;
// 	irqstate_t flags;
// 	uint8_t epno;
// 	int ret = OK;

// #ifdef CONFIG_DEBUG
// 	if (!req || !req->callback || !req->buf || !ep) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		ulldbg("ERROR: req=%p callback=%p buf=%p ep=%p\n", req, req->callback, req->buf, ep);
// 		return -EINVAL;
// 	}
// #endif

// 	usbtrace(TRACE_EPSUBMIT, USB_EPNO(ep->eplog));
// 	priv = privep->dev;

// #ifdef CONFIG_DEBUG
// 	if (!priv->driver) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_NOTCONFIGURED), priv->usbdev.speed);
// 		ulldbg("ERROR: driver=%p\n", priv->driver);
// 		return -ESHUTDOWN;
// 	}
// #endif

// 	/* Handle the request from the class driver */

// 	epno = USB_EPNO(ep->eplog);
// 	req->result = -EINPROGRESS;
// 	req->xfrd = 0;
// 	flags = enter_critical_section();

// 	/* If we are stalled, then drop all requests on the floor */

// 	if (privep->stalled) {
// 		amebasmart_abortrequest(privep, privreq, -EBUSY);
// 		ulldbg("ERROR: stalled\n");
// 		ret = -EBUSY;
// 	}

// 	/* Handle IN (device-to-host) requests.  NOTE:  If the class device is
// 	 * using the bi-directional EP0, then we assume that they intend the EP0
// 	 * IN functionality.
// 	 */

// 	else if (USB_ISEPIN(ep->eplog) || epno == EP0) {
// 		/* Add the new request to the request queue for the IN endpoint */

// 		amebasmart_rqenqueue(privep, privreq);
// 		usbtrace(TRACE_INREQQUEUED(epno), req->len);

// 		/* If the IN endpoint FIFO is available, then transfer the data now */

// 		if (!privep->txbusy) {
// 			priv->txstatus = USB_EPR_STATTX_NAK;
// 			if (epno == EP0) {
// 				ret = amebasmart_wrrequest_ep0(priv, privep);
// 			} else {
// 				ret = amebasmart_wrrequest(priv, privep);
// 			}

// 			/* Set the new TX status */

// 			amebasmart_seteptxstatus(epno, priv->txstatus);
// 		}
// 	}

// 	/* Handle OUT (host-to-device) requests */

// 	else {
// 		/* Add the new request to the request queue for the OUT endpoint */

// 		privep->txnullpkt = 0;
// 		amebasmart_rqenqueue(privep, privreq);
// 		usbtrace(TRACE_OUTREQQUEUED(epno), req->len);

// 		/* This there a incoming data pending the availability of a request? */

// 		if (priv->rxpending) {
// 			/* Set STAT_RX bits to '11' in the USB_EPnR, enabling further
// 			 * transactions. "While the STAT_RX bits are equal to '10'
// 			 * (NAK), any OUT request addressed to that endpoint is NAKed,
// 			 * indicating a flow control condition: the USB host will retry
// 			 * the transaction until it succeeds."
// 			 */

// 			priv->rxstatus = USB_EPR_STATRX_VALID;
// 			amebasmart_seteprxstatus(epno, priv->rxstatus);

// 			/* Data is no longer pending */

// 			priv->rxpending = false;
// 		}
// 	}

// 	leave_critical_section(flags);
// 	return ret;
// }

// /****************************************************************************
//  * Name: amebasmart_epcancel
//  ****************************************************************************/

// static int amebasmart_epcancel(struct usbdev_ep_s *ep, struct usbdev_req_s *req)
// {
// 	struct amebasmart_ep_s *privep = (struct amebasmart_ep_s *)ep;
// 	irqstate_t flags;

// #ifdef CONFIG_DEBUG
// 	if (!ep || !req) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -EINVAL;
// 	}
// #endif
// 	usbtrace(TRACE_EPCANCEL, USB_EPNO(ep->eplog));

// 	flags = enter_critical_section();
// 	amebasmart_cancelrequests(privep);
// 	leave_critical_section(flags);
// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_epstall
//  ****************************************************************************/

// static int amebasmart_epstall(struct usbdev_ep_s *ep, bool resume)
// {
// 	struct amebasmart_ep_s *privep;
// 	struct amebasmart_usbdev_s *priv;
// 	uint8_t epno;
// 	uint16_t status;
// 	irqstate_t flags;

// #ifdef CONFIG_DEBUG
// 	if (!ep) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -EINVAL;
// 	}
// #endif
// 	privep = (struct amebasmart_ep_s *)ep;
// 	priv = (struct amebasmart_usbdev_s *)privep->dev;
// 	epno = USB_EPNO(ep->eplog);

// 	/* STALL or RESUME the endpoint */

// 	flags = enter_critical_section();
// 	usbtrace(resume ? TRACE_EPRESUME : TRACE_EPSTALL, USB_EPNO(ep->eplog));

// 	/* Get status of the endpoint; stall the request if the endpoint is
// 	 * disabled
// 	 */

// 	if (USB_ISEPIN(ep->eplog)) {
// 		status = amebasmart_geteptxstatus(epno);
// 	} else {
// 		status = amebasmart_geteprxstatus(epno);
// 	}

// 	if (status == 0) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_EPDISABLED), 0);

// 		if (epno == 0) {
// 			priv->ep0state = EP0STATE_STALLED;
// 		}

// 		return -ENODEV;
// 	}

// 	/* Handle the resume condition */

// 	if (resume) {
// 		/* Resuming a stalled endpoint */

// 		usbtrace(TRACE_EPRESUME, epno);
// 		privep->stalled = false;

// 		if (USB_ISEPIN(ep->eplog)) {
// 			/* IN endpoint */

// 			if (amebasmart_eptxstalled(epno)) {
// 				amebasmart_clrtxdtog(epno);

// 				/* Restart any queued write requests */

// 				priv->txstatus = USB_EPR_STATTX_NAK;
// 				if (epno == EP0) {
// 					(void)amebasmart_wrrequest_ep0(priv, privep);
// 				} else {
// 					(void)amebasmart_wrrequest(priv, privep);
// 				}

// 				/* Set the new TX status */

// 				amebasmart_seteptxstatus(epno, priv->txstatus);
// 			}
// 		} else {
// 			/* OUT endpoint */

// 			if (amebasmart_eprxstalled(epno)) {
// 				if (epno == EP0) {
// 					/* After clear the STALL, enable the default endpoint receiver */

// 					amebasmart_seteprxcount(epno, ep->maxpacket);
// 				} else {
// 					amebasmart_clrrxdtog(epno);
// 				}

// 				priv->rxstatus = USB_EPR_STATRX_VALID;
// 				amebasmart_seteprxstatus(epno, USB_EPR_STATRX_VALID);
// 			}
// 		}
// 	}

// 	/* Handle the stall condition */

// 	else {
// 		usbtrace(TRACE_EPSTALL, epno);
// 		privep->stalled = true;

// 		if (USB_ISEPIN(ep->eplog)) {
// 			/* IN endpoint */

// 			priv->txstatus = USB_EPR_STATTX_STALL;
// 			amebasmart_seteptxstatus(epno, USB_EPR_STATTX_STALL);
// 		} else {
// 			/* OUT endpoint */

// 			priv->rxstatus = USB_EPR_STATRX_STALL;
// 			amebasmart_seteprxstatus(epno, USB_EPR_STATRX_STALL);
// 		}
// 	}

// 	leave_critical_section(flags);
// 	return OK;
// }

// /****************************************************************************
//  * Device Controller Operations
//  ****************************************************************************/
// /****************************************************************************
//  * Name: amebasmart_allocep
//  ****************************************************************************/

// static struct usbdev_ep_s *amebasmart_allocep(struct usbdev_s *dev, uint8_t epno, bool in, uint8_t eptype)
// {
// 	struct amebasmart_usbdev_s *priv = (struct amebasmart_usbdev_s *)dev;
// 	struct amebasmart_ep_s *privep = NULL;
// 	uint8_t epset = AMEBASMART_ENDP_ALLSET;
// 	int bufno;

// 	usbtrace(TRACE_DEVALLOCEP, (uint16_t)epno);
// #ifdef CONFIG_DEBUG
// 	if (!dev) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return NULL;
// 	}
// #endif

// 	/* Ignore any direction bits in the logical address */

// 	epno = USB_EPNO(epno);

// 	/* A logical address of 0 means that any endpoint will do */

// 	if (epno > 0) {
// 		/* Otherwise, we will return the endpoint structure only for the requested
// 		 * 'logical' endpoint.  All of the other checks will still be performed.
// 		 *
// 		 * First, verify that the logical endpoint is in the range supported by
// 		 * by the hardware.
// 		 */

// 		if (epno >= AMEBASMART_NENDPOINTS) {
// 			usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BADEPNO), (uint16_t)epno);
// 			return NULL;
// 		}

// 		/* Convert the logical address to a physical OUT endpoint address and
// 		 * remove all of the candidate endpoints from the bitset except for the
// 		 * the IN/OUT pair for this logical address.
// 		 */

// 		epset = AMEBASMART_ENDP_BIT(epno);
// 	}

// 	/* Check if the selected endpoint number is available */

// 	privep = amebasmart_epreserve(priv, epset);
// 	if (!privep) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_EPRESERVE), (uint16_t)epset);
// 		goto errout;
// 	}

// 	/* Allocate a PMA buffer for this endpoint */

// #warning "REVISIT: Should configure BULK EPs using double buffer feature"
// 	bufno = amebasmart_epallocpma(priv);
// 	if (bufno < 0) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_EPBUFFER), 0);
// 		goto errout_with_ep;
// 	}
// 	privep->bufno = (uint8_t)bufno;
// 	return &privep->ep;

// errout_with_ep:
// 	amebasmart_epunreserve(priv, privep);
// errout:
// 	return NULL;
// }

// /****************************************************************************
//  * Name: amebasmart_freeep
//  ****************************************************************************/

// static void amebasmart_freeep(struct usbdev_s *dev, struct usbdev_ep_s *ep)
// {
// 	struct amebasmart_usbdev_s *priv;
// 	struct amebasmart_ep_s *privep;

// #ifdef CONFIG_DEBUG
// 	if (!dev || !ep) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return;
// 	}
// #endif
// 	priv = (struct amebasmart_usbdev_s *)dev;
// 	privep = (struct amebasmart_ep_s *)ep;
// 	usbtrace(TRACE_DEVFREEEP, (uint16_t)USB_EPNO(ep->eplog));

// 	if (priv && privep) {
// 		/* Free the PMA buffer assigned to this endpoint */

// 		amebasmart_epfreepma(priv, privep);

// 		/* Mark the endpoint as available */

// 		amebasmart_epunreserve(priv, privep);
// 	}
// }

// /****************************************************************************
//  * Name: amebasmart_getframe
//  ****************************************************************************/

// static int amebasmart_getframe(struct usbdev_s *dev)
// {
// 	uint16_t fnr;

// #ifdef CONFIG_DEBUG
// 	if (!dev) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -EINVAL;
// 	}
// #endif

// 	/* Return the last frame number detected by the hardware */

// 	fnr = amebasmart_getreg(AMEBASMART_USB_FNR);
// 	usbtrace(TRACE_DEVGETFRAME, fnr);
// 	return (fnr & USB_FNR_FN_MASK);
// }

// /****************************************************************************
//  * Name: amebasmart_wakeup
//  ****************************************************************************/

// static int amebasmart_wakeup(struct usbdev_s *dev)
// {
// 	struct amebasmart_usbdev_s *priv = (struct amebasmart_usbdev_s *)dev;
// 	irqstate_t flags;

// 	usbtrace(TRACE_DEVWAKEUP, 0);
// #ifdef CONFIG_DEBUG
// 	if (!dev) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -EINVAL;
// 	}
// #endif

// 	/* Start the resume sequence.  The actual resume steps will be driven
// 	 * by the ESOF interrupt.
// 	 */

// 	flags = enter_critical_section();
// 	amebasmart_initresume(priv);
// 	priv->rsmstate = RSMSTATE_STARTED;

// 	/* Disable the SUSP interrupt (until we are fully resumed), disable
// 	 * the WKUP interrupt (we are already waking up), and enable the
// 	 * ESOF interrupt that will drive the resume operations.  Clear any
// 	 * pending ESOF interrupt.
// 	 */

// 	amebasmart_setimask(priv, USB_CNTR_ESOFM, USB_CNTR_WKUPM | USB_CNTR_SUSPM);
// 	amebasmart_putreg(~USB_ISTR_ESOF, AMEBASMART_USB_ISTR);
// 	leave_critical_section(flags);
// 	return OK;
// }

// /****************************************************************************
//  * Name: amebasmart_selfpowered
//  ****************************************************************************/

// static int amebasmart_selfpowered(struct usbdev_s *dev, bool selfpowered)
// {
// 	struct amebasmart_usbdev_s *priv = (struct amebasmart_usbdev_s *)dev;

// 	usbtrace(TRACE_DEVSELFPOWERED, (uint16_t)selfpowered);

// #ifdef CONFIG_DEBUG
// 	if (!dev) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -ENODEV;
// 	}
// #endif

// 	priv->selfpowered = selfpowered;
// 	return OK;
// }

// /****************************************************************************
//  * Initialization/Reset
//  ****************************************************************************/

// /****************************************************************************
//  * Name: amebasmart_reset
//  ****************************************************************************/

// static void amebasmart_reset(struct amebasmart_usbdev_s *priv)
// {
// 	int epno;

// 	/* Put the USB controller in reset, disable all interrupts */

// 	amebasmart_putreg(USB_CNTR_FRES, AMEBASMART_USB_CNTR);

// 	/* Tell the class driver that we are disconnected.  The class driver
// 	 * should then accept any new configurations.
// 	 */

// 	CLASS_DISCONNECT(priv->driver, &priv->usbdev);

// 	/* Reset the device state structure */

// 	priv->ep0state = EP0STATE_IDLE;
// 	priv->rsmstate = RSMSTATE_IDLE;
// 	priv->rxpending = false;

// 	/* Reset endpoints */

// 	for (epno = 0; epno < AMEBASMART_NENDPOINTS; epno++) {
// 		struct amebasmart_ep_s *privep = &priv->eplist[epno];

// 		/* Cancel any queued requests.  Since they are canceled
// 		 * with status -ESHUTDOWN, then will not be requeued
// 		 * until the configuration is reset.  NOTE:  This should
// 		 * not be necessary... the CLASS_DISCONNECT above should
// 		 * result in the class implementation calling amebasmart_epdisable
// 		 * for each of its configured endpoints.
// 		 */

// 		amebasmart_cancelrequests(privep);

// 		/* Reset endpoint status */

// 		privep->stalled = false;
// 		privep->halted = false;
// 		privep->txbusy = false;
// 		privep->txnullpkt = false;
// 	}

// 	/* Re-configure the USB controller in its initial, unconnected state */

// 	amebasmart_hwreset(priv);
// 	priv->usbdev.speed = USB_SPEED_FULL;
// }

// /****************************************************************************
//  * Name: amebasmart_hwreset
//  ****************************************************************************/

// static void amebasmart_hwreset(struct amebasmart_usbdev_s *priv)
// {
// 	/* Put the USB controller into reset, clear all interrupt enables */

// 	amebasmart_putreg(USB_CNTR_FRES, AMEBASMART_USB_CNTR);

// 	/* Disable interrupts (and perhaps take the USB controller out of reset) */

// 	priv->imask = 0;
// 	amebasmart_putreg(priv->imask, AMEBASMART_USB_CNTR);

// 	/* Set the AMEBASMART BTABLE address */

// 	amebasmart_putreg(AMEBASMART_BTABLE_ADDRESS & 0xfff8, AMEBASMART_USB_BTABLE);

// 	/* Initialize EP0 */

// 	amebasmart_seteptype(EP0, USB_EPR_EPTYPE_CONTROL);
// 	amebasmart_seteptxstatus(EP0, USB_EPR_STATTX_NAK);
// 	amebasmart_seteprxaddr(EP0, AMEBASMART_EP0_RXADDR);
// 	amebasmart_seteprxcount(EP0, AMEBASMART_EP0MAXPACKET);
// 	amebasmart_seteptxaddr(EP0, AMEBASMART_EP0_TXADDR);
// 	amebasmart_clrstatusout(EP0);
// 	amebasmart_seteprxstatus(EP0, USB_EPR_STATRX_VALID);

// 	/* Set the device to respond on default address */

// 	amebasmart_setdevaddr(priv, 0);

// 	/* Clear any pending interrupts */

// 	amebasmart_putreg(0, AMEBASMART_USB_ISTR);

// 	/* Enable interrupts at the USB controller */

// 	amebasmart_setimask(priv, AMEBASMART_CNTR_SETUP, (USB_CNTR_ALLINTS & ~AMEBASMART_CNTR_SETUP));
// 	amebasmart_dumpep(EP0);
// }

// /****************************************************************************
//  * Name: amebasmart_hwsetup
//  ****************************************************************************/

// static void amebasmart_hwsetup(struct amebasmart_usbdev_s *priv)
// {
// 	int epno;

// 	/* Power the USB controller, put the USB controller into reset, disable
// 	 * all USB interrupts
// 	 */

// 	amebasmart_putreg(USB_CNTR_FRES | USB_CNTR_PDWN, AMEBASMART_USB_CNTR);

// 	/* Disconnect the device / disable the pull-up.  We don't want the
// 	 * host to enumerate us until the class driver is registered.
// 	 */

// 	amebasmart_usbpullup(&priv->usbdev, false);

// 	/* Initialize the device state structure.  NOTE: many fields
// 	 * have the initial value of zero and, hence, are not explicitly
// 	 * initialized here.
// 	 */

// 	memset(priv, 0, sizeof(struct amebasmart_usbdev_s));
// 	priv->usbdev.ops = &g_devops;
// 	priv->usbdev.ep0 = &priv->eplist[EP0].ep;
// 	priv->epavail = AMEBASMART_ENDP_ALLSET & ~AMEBASMART_ENDP_BIT(EP0);
// 	priv->bufavail = AMEBASMART_BUFFER_ALLSET & ~AMEBASMART_BUFFER_EP0;

// 	/* Initialize the endpoint list */

// 	for (epno = 0; epno < AMEBASMART_NENDPOINTS; epno++) {
// 		/* Set endpoint operations, reference to driver structure (not
// 		 * really necessary because there is only one controller), and
// 		 * the (physical) endpoint number which is just the index to the
// 		 * endpoint.
// 		 */

// 		priv->eplist[epno].ep.ops = &g_epops;
// 		priv->eplist[epno].dev = priv;
// 		priv->eplist[epno].ep.eplog = epno;

// 		/* We will use a fixed maxpacket size for all endpoints (perhaps
// 		 * ISOC endpoints could have larger maxpacket???).  A smaller
// 		 * packet size can be selected when the endpoint is configured.
// 		 */

// 		priv->eplist[epno].ep.maxpacket = AMEBASMART_MAXPACKET_SIZE;
// 	}

// 	/* Select a smaller endpoint size for EP0 */

// #if AMEBASMART_EP0MAXPACKET < AMEBASMART_MAXPACKET_SIZE
// 	priv->eplist[EP0].ep.maxpacket = AMEBASMART_EP0MAXPACKET;
// #endif

// 	/* Configure the USB controller.  USB uses the following GPIO pins:
// 	 *
// 	 *   PA9  - VBUS
// 	 *   PA10 - ID
// 	 *   PA11 - DM
// 	 *   PA12 - DP
// 	 *
// 	 * "As soon as the USB is enabled, these pins [DM and DP] are connected to
// 	 * the USB internal transceiver automatically."
// 	 */

// 	/* Power up the USB controller, holding it in reset.  There is a delay of
// 	 * about 1uS after applying power before the USB will behave predictably.
// 	 * A 5MS delay is more than enough.  NOTE that we leave the USB controller
// 	 * in the reset state; the hardware will not be initialized until the
// 	 * class driver has been bound.
// 	 */

// 	amebasmart_putreg(USB_CNTR_FRES, AMEBASMART_USB_CNTR);
// 	up_mdelay(5);
// }

// /****************************************************************************
//  * Name: amebasmart_hwshutdown
//  ****************************************************************************/

// static void amebasmart_hwshutdown(struct amebasmart_usbdev_s *priv)
// {
// 	priv->usbdev.speed = USB_SPEED_UNKNOWN;

// 	/* Disable all interrupts and force the USB controller into reset */

// 	amebasmart_putreg(USB_CNTR_FRES, AMEBASMART_USB_CNTR);

// 	/* Clear any pending interrupts */

// 	amebasmart_putreg(0, AMEBASMART_USB_ISTR);

// 	/* Disconnect the device / disable the pull-up */

// 	amebasmart_usbpullup(&priv->usbdev, false);

// 	/* Power down the USB controller */

// 	amebasmart_putreg(USB_CNTR_FRES | USB_CNTR_PDWN, AMEBASMART_USB_CNTR);
// }

// /****************************************************************************
//  * Public Functions
//  ****************************************************************************/
// /****************************************************************************
//  * Name: up_usbinitialize
//  * Description:
//  *   Initialize the USB driver
//  * Input Parameters:
//  *   None
//  *
//  * Returned Value:
//  *   None
//  *
//  ****************************************************************************/

// void up_usbinitialize(void)
// {
// 	/* For now there is only one USB controller, but we will always refer to
// 	 * it using a pointer to make any future ports to multiple USB controllers
// 	 * easier.
// 	 */

// 	struct amebasmart_usbdev_s *priv = &g_usbdev;

// 	usbtrace(TRACE_DEVINIT, 0);
// 	amebasmart_checksetup();

// 	/* Configure USB GPIO alternate function pins */

// #ifdef CONFIG_AMEBASMART_AMEBASMARTF30XX
// 	(void)amebasmart_configgpio(GPIO_USB_DM);
// 	(void)amebasmart_configgpio(GPIO_USB_DP);
// #endif

// 	/* Power up the USB controller, but leave it in the reset state */

// 	amebasmart_hwsetup(priv);

// 	/* Remap the USB interrupt as needed (Only supported by the AMEBASMART F3 family) */

// #ifdef CONFIG_AMEBASMART_AMEBASMARTF30XX
// #ifdef CONFIG_AMEBASMART_USB_ITRMP
// 	/* Clear the ITRMP bit to use the legacy, shared USB/CAN interrupts */

// 	modifyreg32(AMEBASMART_RCC_APB1ENR, SYSCFG_CFGR1_USB_ITRMP, 0);
// #else
// 	/* Set the ITRMP bit to use the AMEBASMART F3's dedicated USB interrupts */

// 	modifyreg32(AMEBASMART_RCC_APB1ENR, 0, SYSCFG_CFGR1_USB_ITRMP);
// #endif
// #endif

// 	/* Attach USB controller interrupt handlers.  The hardware will not be
// 	 * initialized and interrupts will not be enabled until the class device
// 	 * driver is bound.  Getting the IRQs here only makes sure that we have
// 	 * them when we need them later.
// 	 */

// 	if (irq_attach(AMEBASMART_IRQ_USBHP, amebasmart_hpinterrupt, NULL) != 0) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_IRQREGISTRATION), (uint16_t)AMEBASMART_IRQ_USBHP);
// 		goto errout;
// 	}

// 	if (irq_attach(AMEBASMART_IRQ_USBLP, amebasmart_lpinterrupt, NULL) != 0) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_IRQREGISTRATION), (uint16_t)AMEBASMART_IRQ_USBLP);
// 		goto errout;
// 	}
// 	return;

// errout:
// 	up_usbuninitialize();
// }

// /****************************************************************************
//  * Name: up_usbuninitialize
//  * Description:
//  *   Initialize the USB driver
//  * Input Parameters:
//  *   None
//  *
//  * Returned Value:
//  *   None
//  *
//  ****************************************************************************/

// void up_usbuninitialize(void)
// {
// 	/* For now there is only one USB controller, but we will always refer to
// 	 * it using a pointer to make any future ports to multiple USB controllers
// 	 * easier.
// 	 */

// 	struct amebasmart_usbdev_s *priv = &g_usbdev;
// 	irqstate_t flags;

// 	flags = enter_critical_section();
// 	usbtrace(TRACE_DEVUNINIT, 0);

// 	/* Disable and detach the USB IRQs */

// 	up_disable_irq(AMEBASMART_IRQ_USBHP);
// 	up_disable_irq(AMEBASMART_IRQ_USBLP);
// 	irq_detach(AMEBASMART_IRQ_USBHP);
// 	irq_detach(AMEBASMART_IRQ_USBLP);

// 	if (priv->driver) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_DRIVERREGISTERED), 0);
// 		usbdev_unregister(priv->driver);
// 	}

// 	/* Put the hardware in an inactive state */

// 	amebasmart_hwshutdown(priv);
// 	leave_critical_section(flags);
// }

// /****************************************************************************
//  * Name: usbdev_register
//  *
//  * Description:
//  *   Register a USB device class driver. The class driver's bind() method will be
//  *   called to bind it to a USB device driver.
//  *
//  ****************************************************************************/

// int usbdev_register(struct usbdevclass_driver_s *driver)
// {
// 	/* For now there is only one USB controller, but we will always refer to
// 	 * it using a pointer to make any future ports to multiple USB controllers
// 	 * easier.
// 	 */

// 	struct amebasmart_usbdev_s *priv = &g_usbdev;
// 	int ret;

// 	usbtrace(TRACE_DEVREGISTER, 0);

// #ifdef CONFIG_DEBUG
// 	if (!driver || !driver->ops->bind || !driver->ops->unbind || !driver->ops->disconnect || !driver->ops->setup) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -EINVAL;
// 	}

// 	if (priv->driver) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_DRIVER), 0);
// 		return -EBUSY;
// 	}
// #endif

// 	/* First hook up the driver */

// 	priv->driver = driver;

// 	/* Then bind the class driver */

// 	ret = CLASS_BIND(driver, &priv->usbdev);
// 	if (ret) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_BINDFAILED), (uint16_t)-ret);
// 	} else {
// 		/* Setup the USB controller -- enabling interrupts at the USB controller */

// 		amebasmart_hwreset(priv);

// 		/* Enable USB controller interrupts at the NVIC */

// 		up_enable_irq(AMEBASMART_IRQ_USBHP);
// 		up_enable_irq(AMEBASMART_IRQ_USBLP);

// 		/* Enable pull-up to connect the device.  The host should enumerate us
// 		 * some time after this
// 		 */

// 		amebasmart_usbpullup(&priv->usbdev, true);
// 		priv->usbdev.speed = USB_SPEED_FULL;
// 	}
// 	return ret;
// }

// /****************************************************************************
//  * Name: usbdev_unregister
//  *
//  * Description:
//  *   Un-register usbdev class driver. If the USB device is connected to a
//  *   USB host, it will first disconnect().  The driver is also requested to
//  *   unbind() and clean up any device state, before this procedure finally
//  *   returns.
//  *
//  ****************************************************************************/

// int usbdev_unregister(struct usbdevclass_driver_s *driver)
// {
// 	/* For now there is only one USB controller, but we will always refer to
// 	 * it using a pointer to make any future ports to multiple USB controllers
// 	 * easier.
// 	 */

// 	struct amebasmart_usbdev_s *priv = &g_usbdev;
// 	irqstate_t flags;

// 	usbtrace(TRACE_DEVUNREGISTER, 0);

// #ifdef CONFIG_DEBUG
// 	if (driver != priv->driver) {
// 		usbtrace(TRACE_DEVERROR(AMEBASMART_TRACEERR_INVALIDPARMS), 0);
// 		return -EINVAL;
// 	}
// #endif

// 	/* Reset the hardware and cancel all requests.  All requests must be
// 	 * canceled while the class driver is still bound.
// 	 */

// 	flags = enter_critical_section();
// 	amebasmart_reset(priv);

// 	/* Unbind the class driver */

// 	CLASS_UNBIND(driver, &priv->usbdev);

// 	/* Disable USB controller interrupts (but keep them attached) */

// 	up_disable_irq(AMEBASMART_IRQ_USBHP);
// 	up_disable_irq(AMEBASMART_IRQ_USBLP);

// 	/* Put the hardware in an inactive state.  Then bring the hardware back up
// 	 * in the reset state (this is probably not necessary, the amebasmart_reset()
// 	 * call above was probably sufficient).
// 	 */

// 	amebasmart_hwshutdown(priv);
// 	amebasmart_hwsetup(priv);

// 	/* Unhook the driver */

// 	priv->driver = NULL;
// 	leave_critical_section(flags);
// 	return OK;
// }

#endif							/* CONFIG_USBDEV && CONFIG_AMEBASMART_USB */
