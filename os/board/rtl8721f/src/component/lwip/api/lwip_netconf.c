/* Includes ------------------------------------------------------------------*/
#include "lwip_netconf.h"
#include "wifi_intf_drv_to_upper.h"
#include "ameba_pmu.h"
#include "lwip_intf_tizenrt.h"

#ifndef UNUSED
#define UNUSED(x) ((void)(x))
#endif


/*Static IP ADDRESS FOR ETHERNET*/
#ifndef ETH_IP_ADDR0
#define ETH_IP_ADDR0 192
#define ETH_IP_ADDR1 168
#define ETH_IP_ADDR2 0
#define ETH_IP_ADDR3 80
#endif

/*NETMASK FOR ETHERNET*/
#ifndef ETH_NETMASK_ADDR0
#define ETH_NETMASK_ADDR0 255
#define ETH_NETMASK_ADDR1 255
#define ETH_NETMASK_ADDR2 255
#define ETH_NETMASK_ADDR3 0
#endif

/*Gateway address for ethernet*/
#ifndef ETH_GW_ADDR0
#define ETH_GW_ADDR0 192
#define ETH_GW_ADDR1 168
#define ETH_GW_ADDR2 0
#define ETH_GW_ADDR3 1
#endif

/* Private define ------------------------------------------------------------*/
#define MAX_DHCP_TRIES 5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef CONFIG_WHC_HOST
#include "whc_ipc.h"
#endif

#if defined(CONFIG_WHC_DEV) && defined(CONFIG_WHC_DUAL_TCPIP)
struct static_ip_config user_static_ip;

unsigned char ap_ip[4] = {192, 168, 43, 1}, ap_netmask[4] = {255, 255, 255, 0}, ap_gw[4] = {192, 168, 43, 1};
#endif

#if (defined(CONFIG_LWIP_USB_ETHERNET) && CONFIG_LWIP_USB_ETHERNET) || (defined(CONFIG_ETHERNET) && CONFIG_ETHERNET)

struct netif eth_netif;
extern err_t ethernetif_mii_init(struct netif *netif);
#endif
extern void (*p_wifi_join_info_free)(u8 iface_type);
struct netif xnetif[NET_IF_NUM]; /* network interface structure */
/* Private functions ---------------------------------------------------------*/

int netif_get_idx(struct netif *pnetif)
{
#if defined(CONFIG_LWIP_LAYER) && (CONFIG_LWIP_LAYER == 1)
	int idx = pnetif - xnetif;

	switch (idx) {
	case 0:
		return 0;
	case 1:
		return 1;
	case 2:
		return 2;
	default:
		return -1;
	}
#else
	UNUSED(pnetif);
	return -1;
#endif
}

void LwIP_netif_set_up(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netifapi_netif_set_up(pnetif);
}

void LwIP_netif_set_down(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netifapi_netif_set_down(pnetif);
}

void LwIP_netif_set_link_up(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netifapi_netif_set_link_up(pnetif);
	if (idx == STA_WLAN_INDEX) {
		netifapi_netif_set_default(&xnetif[STA_WLAN_INDEX]);
	} else if ((idx == SOFTAP_WLAN_INDEX) && (!(xnetif[STA_WLAN_INDEX].flags & NETIF_FLAG_LINK_UP))) {
		/*default netif is on sta when sta and softap both up*/
		netifapi_netif_set_default(&xnetif[SOFTAP_WLAN_INDEX]);
	}
}

void LwIP_netif_set_link_down(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netifapi_netif_set_link_down(pnetif);
	if (idx == SOFTAP_WLAN_INDEX) {
		netifapi_netif_set_default(&xnetif[STA_WLAN_INDEX]);
	} else if (idx == STA_WLAN_INDEX) {
		netifapi_netif_set_default(&xnetif[SOFTAP_WLAN_INDEX]);
	}
}

uint8_t *LwIP_GetMAC(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	return (uint8_t *)(pnetif->hwaddr);
}

uint8_t *LwIP_GetIP(uint8_t idx)
{
	return rltk_wlan_get_ip(idx);
}

uint8_t *LwIP_GetGW(uint8_t idx)
{
	return rltk_wlan_get_gw(idx);
}

uint8_t *LwIP_GetMASK(uint8_t idx)
{
	return rltk_wlan_get_gwmask(idx);
}

void LwIP_wlan_set_netif_info(int idx_wlan, void *dev, unsigned char *dev_addr)
{
	memcpy(xnetif[idx_wlan].hwaddr, dev_addr, 6);
	xnetif[idx_wlan].state = dev;
}

void LwIP_ethernetif_recv(uint8_t idx, int total_len)
{
	ethernetif_recv(&xnetif[idx], total_len);
}

SRAM_WLAN_CRITICAL_CODE_SECTION
void LwIP_ethernetif_recv_inic(uint8_t idx, struct pbuf *p_buf)
{
	err_enum_t error = ERR_OK;
	error = xnetif[idx].input(p_buf, &xnetif[idx]);
	if (error != ERR_OK) {
		RTK_LOGS(TAG_WLAN_INIC, RTK_LOG_ERROR, "lwip input err (%d)\n", error);
		pbuf_free(p_buf);
	}
}

int LwIP_netif_is_valid_IP(int idx, unsigned char *ip_dest)
{
#if defined(CONFIG_LWIP_LAYER) && (CONFIG_LWIP_LAYER == 1)
	struct netif *pnetif = &xnetif[idx];

	ip_addr_t addr = { 0 };
	u32_t ip_dest_addr = { 0 };

	memcpy(&ip_dest_addr, ip_dest, 4);

	ip_addr_set_ip4_u32(&addr, ip_dest_addr);

	if ((ip_addr_get_ip4_u32(netif_ip_addr4(pnetif))) == 0) {
		return 1;
	}

	if (ip_addr_ismulticast(&addr) || ip_addr_isbroadcast(&addr, pnetif)) {
		return 1;
	}

	//if(ip_addr_netcmp(&(pnetif->ip_addr), &addr, &(pnetif->netmask))) //addr&netmask
	//	return 1;

	if (ip_addr_cmp(&(pnetif->ip_addr), &addr)) {
		return 1;
	}

	//RTK_LOGS(NOTAG, RTK_LOG_INFO, "invalid IP: %d.%d.%d.%d ",ip_dest[0],ip_dest[1],ip_dest[2],ip_dest[3]);
#endif
	UNUSED(idx);
	UNUSED(ip_dest);
	return 0;
}

#if LWIP_DNS
void LwIP_GetDNS(struct ip_addr *dns)
{
	struct ip_addr *tmp = (struct ip_addr *)dns_getserver(0);
	*dns = *tmp;
}

void LwIP_SetDNS(struct ip_addr *dns)
{
	dns_setserver(0, dns);
}
#endif

void LwIP_SetIP(uint8_t idx, u32_t addr, u32_t netmask_addr, u32_t gw_addr)
{
	struct netif *pnetif = &xnetif[idx];
	struct ip_addr ipaddr;
	struct ip_addr netmask;
	struct ip_addr gw;

#if CONFIG_WLAN
	ip_2_ip4(&ipaddr)->addr = PP_HTONL(addr);
	ip_2_ip4(&netmask)->addr = PP_HTONL(netmask_addr);
	ip_2_ip4(&gw)->addr = PP_HTONL(gw_addr);
	netifapi_netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
#endif
}

#if LWIP_AUTOIP
#include <lwip/autoip.h>
#include <lwip/prot/autoip.h>

void LwIP_AUTOIP(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	uint8_t *ip = LwIP_GetIP(idx);
	struct autoip *autoip = NULL;

	autoip = ((struct autoip *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_AUTOIP));
	if (autoip && (autoip->tried_llipaddr >= MAX_CONFLICTS)) { // before autoip_start(), autoip may be NULL
		autoip->tried_llipaddr = 0;
	}

	netifapi_autoip_start(pnetif);

	autoip = ((struct autoip *)netif_get_client_data(pnetif, LWIP_NETIF_CLIENT_DATA_INDEX_AUTOIP));

	while ((autoip->state == AUTOIP_STATE_PROBING) || (autoip->state == AUTOIP_STATE_ANNOUNCING)) {
		rtos_time_delay_ms(1000);
	}

	if (*((uint32_t *) ip) == 0) {
		struct ip_addr ipaddr;
		struct ip_addr netmask;
		struct ip_addr gw;

		RTK_LOGS(NOTAG, RTK_LOG_INFO, "AUTOIP timeout\n");

		/* Static address used */
		IP4_ADDR(ip_2_ip4(&ipaddr), STATIC_IP_ADDR0, STATIC_IP_ADDR1, STATIC_IP_ADDR2, STATIC_IP_ADDR3);
		IP4_ADDR(ip_2_ip4(&netmask), NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
		IP4_ADDR(ip_2_ip4(&gw), GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		netifapi_netif_set_addr(pnetif, ip_2_ip4(&ipaddr), ip_2_ip4(&netmask), ip_2_ip4(&gw));
		RTK_LOGS(NOTAG, RTK_LOG_INFO, "Static IP address : %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
	} else {
		RTK_LOGS(NOTAG, RTK_LOG_INFO, "\nLink-local address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
	}
}

void LwIP_AUTOIP_STOP(uint8_t idx)
{
	struct netif *pnetif = &xnetif[idx];
	netifapi_autoip_stop(pnetif);
}
#endif

#ifndef CONFIG_PLATFORM_TIZENRT_OS
#if LWIP_IPV6
/* Get IPv6 address with lwip 1.5.0 */
void LwIP_AUTOIP_IPv6(struct netif *pnetif)
{
	netif_create_ip6_linklocal_address(pnetif, 1);
	RTK_LOGS(NOTAG, RTK_LOG_INFO, "\nIPv6 link-local address: ");
	LwIP_DUMP_IPV6_ADDRESS(netif_ip6_addr(pnetif, 0)->addr);
}
#endif
#endif //#ifndef CONFIG_PLATFORM_TIZENRT_OS

//To check successful WiFi connection and obtain of an IP address
void LwIP_Check_Connectivity(void)
{
	u8 join_status = RTW_JOINSTATUS_UNKNOWN;
	rtos_time_delay_ms(2000);
	while (!((wifi_get_join_status(&join_status) == RTK_SUCCESS)
			 && (join_status == RTW_JOINSTATUS_SUCCESS) && (*(u32 *)LwIP_GetIP(0) != IP_ADDR_INVALID))) {
		RTK_LOGS(NOTAG, RTK_LOG_INFO, "Wait for WiFi and DHCP Connect Success...\n");
		RTK_LOGS(NOTAG, RTK_LOG_INFO, "Please use AT+WLCONN to connect AP first time\n");
		rtos_time_delay_ms(2000);
	}
}

/**
  * @brief  For sta get ipv4(dhcp) and ipv6 address
  * @param  None
  * @retval -1 for failed
  */

uint8_t LwIP_IP_Address_Request(void)
{
	uint8_t ret = -1;
#if LWIP_IPV6
	struct netif *pnetif = &xnetif[STA_WLAN_INDEX];
	LwIP_AUTOIP_IPv6(pnetif);
#endif
#if LWIP_IPV4
	ret = LwIP_DHCP(STA_WLAN_INDEX, DHCP_START);
#endif
	return ret;
}
