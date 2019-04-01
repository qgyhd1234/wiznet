/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-26     chenyong     first version
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <wiz.h>
#include <wiz_socket.h>

#include <W5500/w5500.h>
#ifdef WIZ_USING_DHCP
#include <DHCP/wizchip_dhcp.h>
#endif

#ifdef RT_USING_SAL
#include <sal_netif.h>
#endif

#if !defined(WIZ_SPI_DEVICE) || !defined(WIZ_RST_PIN) || !defined(WIZ_IRQ_PIN)
#error "please config SPI device name, reset pin and irq pin in menuconfig."
#endif

#define DBG_ENABLE
#define DBG_SECTION_NAME               "wiz"
#ifdef WIZ_DEBUG
#define DBG_LEVEL                      DBG_LOG
#else
#define DBG_LEVEL                      DBG_INFO
#endif /* WIZ_DEBUG */
#define DBG_COLOR
#include <rtdbg.h>

#define IMR_SENDOK                     0x10
#define IMR_TIMEOUT                    0x08
#define IMR_RECV                       0x04
#define IMR_DISCON                     0x02
#define IMR_CON                        0x01
#define WIZ_DEFAULT_MAC                "00-E0-81-DC-53-1A"

#define WIZ_ID_LEN                     6
char wiz_netif_name[WIZ_ID_LEN];

extern struct rt_spi_device *wiz_device;
extern int wiz_device_init(const char *spi_dev_name, rt_base_t rst_pin, rt_base_t isr_pin);
extern int wiz_inet_init(void);

rt_bool_t wiz_init_ok = RT_FALSE;
static wiz_NetInfo wiz_net_info;
static rt_timer_t  dns_tick_timer;

static void spi_write_byte(uint8_t data)
{
    struct rt_spi_message spi_msg;

    rt_memset(&spi_msg, 0x00, sizeof(spi_msg));

    spi_msg.send_buf = &data;
    spi_msg.length = 1;

    rt_spi_transfer_message(wiz_device, &spi_msg);
}

static uint8_t spi_read_byte(void)
{
    struct rt_spi_message spi_msg;
    uint8_t data;

    rt_memset(&spi_msg, 0x00, sizeof(spi_msg));

    spi_msg.recv_buf = &data;
    spi_msg.length = 1;

    rt_spi_transfer_message(wiz_device, &spi_msg);

    return data;
}

static void spi_write_burst(uint8_t *pbuf, uint16_t len)
{
    struct rt_spi_message spi_msg;

    rt_memset(&spi_msg, 0x00, sizeof(spi_msg));

    spi_msg.send_buf = pbuf;
    spi_msg.length = len;

    rt_spi_transfer_message(wiz_device, &spi_msg);
}

static void spi_read_burst(uint8_t *pbuf, uint16_t len)
{
    struct rt_spi_message spi_msg;

    rt_memset(&spi_msg, 0x00, sizeof(spi_msg));

    spi_msg.recv_buf = pbuf;
    spi_msg.length = len;

    rt_spi_transfer_message(wiz_device, &spi_msg);
}

static void spi_cris_enter(void)
{
    rt_enter_critical();
}

static void spi_cris_exit(void)
{
    rt_exit_critical();
}

static void spi_cs_select(void)
{
    rt_spi_take(wiz_device);
}

static void spi_cs_deselect(void)
{
    rt_spi_release(wiz_device);
}

/* register TCP communication related callback function */
static int wiz_callback_register(void)
{
    /* register critical section callback function */
    reg_wizchip_cris_cbfunc(spi_cris_enter, spi_cris_exit);

#if (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_) || (_WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_)
    /* register SPI device CS select callback function */
    reg_wizchip_cs_cbfunc(spi_cs_select, spi_cs_deselect);
#else
#if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
#error "Unknown _WIZCHIP_IO_MODE_"
#else
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
#endif
#endif
    /* register SPI device read/write data callback function */
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);

    return RT_EOK;
}

/* initialize WIZnet chip configures */
static int wiz_chip_cfg_init(void)
{
#define    CW_INIT_MODE         2
#define    CW_INIT_SOCKETS      8
#define    CW_INIT_TIMEOUT      (5 * RT_TICK_PER_SECOND)

    rt_tick_t start_tick, now_tick;
    uint8_t phy_status;
    uint8_t memsize[CW_INIT_MODE][CW_INIT_SOCKETS] = { 0 };

    /* reset WIZnet chip internal PHY, configures PHY mode. */
    if (ctlwizchip(CW_INIT_WIZCHIP, (void*) memsize) == -1)
    {
        LOG_E("WIZCHIP initialize failed.");
        return -RT_ERROR;
    }

    start_tick = rt_tick_get();
    do
    {
        now_tick = rt_tick_get();
        if (now_tick - start_tick > CW_INIT_TIMEOUT)
        {
            LOG_E("WIZnet chip configure initialize timeout.");
            return -RT_ETIMEOUT;
        }

        /* waiting for link status online */
        if (ctlwizchip(CW_GET_PHYLINK, (void*) &phy_status) == -1)
        {
            LOG_E("Unknown PHY Link stauts.");
        }

        rt_thread_mdelay(100);
    } while (phy_status == PHY_LINK_OFF);

    return RT_EOK;
}

/* WIZnet chip hardware reset */
static void wiz_reset(void)
{
    rt_pin_write(WIZ_RST_PIN, PIN_LOW);
    rt_thread_mdelay(1);

    rt_pin_write(WIZ_RST_PIN, PIN_HIGH);
    rt_thread_mdelay(1);
}

#ifdef WIZ_USING_DHCP
static void wiz_ip_assign(void)
{
    /* get the assigned IP address and reconfigure the IP address of the chip */
    getIPfromDHCP(wiz_net_info.ip);
    getGWfromDHCP(wiz_net_info.gw);
    getSNfromDHCP(wiz_net_info.sn);
    getDNSfromDHCP(wiz_net_info.dns);
    wiz_net_info.dhcp = NETINFO_DHCP;

    ctlnetwork(CN_SET_NETINFO, (void*) &wiz_net_info);
//    LOG_D("DHCP LEASED TIME : %d Sec.", getDHCPLeasetime());
}

static void wiz_ip_conflict(void)
{
    /* deal with conflict IP for WIZnet DHCP  */
    LOG_D("conflict IP from DHCP.");
    RT_ASSERT(0);
}

static void wiz_dhcp_timer_entry(void *parameter)
{
    DHCP_time_handler();
}

static int wiz_network_dhcp(void)
{
#define WIZ_DHCP_SOCKET      0
#define WIZ_DHCP_RETRY       5

    uint8_t dhcp_status;
    uint8_t data_buffer[1024];
    uint8_t dhcp_times = 0;
    rt_timer_t dhcp_timer;

    /* set default MAC address for DHCP */
    setSHAR(wiz_net_info.mac);
    /* DHCP configure initialize, clear information other than MAC address */
    setSn_RXBUF_SIZE(WIZ_DHCP_SOCKET, 0x02);
    setSn_TXBUF_SIZE(WIZ_DHCP_SOCKET, 0x02);
    DHCP_init(WIZ_DHCP_SOCKET, data_buffer);
    /* register to assign IP address and conflict callback */
    reg_dhcp_cbfunc(wiz_ip_assign, wiz_ip_assign, wiz_ip_conflict);

    dhcp_timer = rt_timer_create("w_dhcp", wiz_dhcp_timer_entry, RT_NULL, 1 * RT_TICK_PER_SECOND, RT_TIMER_FLAG_PERIODIC);
    if (dhcp_timer == RT_NULL)
    {
        return -RT_ERROR;
    }
    rt_timer_start(dhcp_timer);

    while (1)
    {
        /* DHCP start, return DHCP_IP_LEASED is success. */
        dhcp_status = DHCP_run();
        switch (dhcp_status)
        {
        case DHCP_IP_ASSIGN:
        case DHCP_IP_CHANGED:
        {
            // TODO: DHCP configure
            break;
        }
        case DHCP_IP_LEASED:
        {
            DHCP_stop();
            rt_timer_delete(dhcp_timer);
            return RT_EOK;
        }
        case DHCP_FAILED:
        {
            dhcp_times ++;
            if (dhcp_times > WIZ_DHCP_RETRY)
            {
                DHCP_stop();
                rt_timer_delete(dhcp_timer);
                return -RT_ETIMEOUT;
            }
            break;
        }
        default:
            break;
        }
    }
}
#endif /* WIZ_USING_DHCP */

static int wiz_netstr_to_array(const char *net_str, uint8_t *net_array)
{
    int ret;
    unsigned int idx;

    RT_ASSERT(net_str);
    RT_ASSERT(net_array);

    if (strstr(net_str, "."))
    {
        int ip_addr[4];

        /* resolve IP address, gateway address or subnet mask */
        ret = sscanf(net_str, "%d.%d.%d.%d", ip_addr + 0, ip_addr + 1, ip_addr + 2, ip_addr + 3);
        if (ret != 4)
        {
            LOG_E("input address(%s) resolve error.", net_str);
            return -RT_ERROR;
        }

        for (idx = 0; idx < sizeof(ip_addr)/sizeof(ip_addr[0]); idx++)
        {
            net_array[idx] = ip_addr[idx];
        }
    }
    else
    {
        int mac_addr[6];

        /* resolve MAC address */
        if (strstr(net_str, ":"))
        {
            ret = sscanf(net_str, "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr + 0, mac_addr + 1, mac_addr + 2,
                    mac_addr + 3,  mac_addr + 4,  mac_addr + 5);
        }
        else if (strstr(net_str, "-"))
        {
            ret = sscanf(net_str, "%02x-%02x-%02x-%02x-%02x-%02x", mac_addr + 0, mac_addr + 1, mac_addr + 2,
                    mac_addr + 3,  mac_addr + 4,  mac_addr + 5);
        }
        else
        {
            LOG_E("input MAC address(%s) format error.", net_str);
            return -RT_ERROR;
        }

        if (ret != 6)
        {
            LOG_E("input MAC address(%s) resolve error.", net_str);
            return -RT_ERROR;
        }

        for (idx = 0; idx < sizeof(mac_addr)/sizeof(mac_addr[0]); idx++)
        {
            net_array[idx] = mac_addr[idx];
        }
    }

    return RT_EOK;
}

/* initialize WIZnet network configures */
static int wiz_network_init(void)
{
    struct sal_netif * netif;
    wiz_NetInfo net_info;
#ifndef WIZ_USING_DHCP
    if(wiz_netstr_to_array(WIZ_IPADDR, wiz_net_info.ip) != RT_EOK ||
            wiz_netstr_to_array(WIZ_MSKADDR, wiz_net_info.sn) != RT_EOK ||
                wiz_netstr_to_array(WIZ_GWADDR, wiz_net_info.gw) != RT_EOK)
    {
        sal_netif_low_level_set_status(sal_netif_get_by_name(wiz_netif_name), RT_FALSE);
        sal_netif_low_level_set_link_status(sal_netif_get_by_name(wiz_netif_name), RT_FALSE);
        return -RT_ERROR;
    }
    wiz_net_info.dhcp = NETINFO_STATIC;
#endif

    /* set static WIZnet network information */
    ctlnetwork(CN_SET_NETINFO, (void*) &wiz_net_info);
    ctlnetwork(CN_GET_NETINFO, (void*) &wiz_net_info);

#ifdef WIZ_USING_DHCP
    /* alloc IP address through DHCP */
    {
        int result = RT_EOK;
        result = wiz_network_dhcp();
        if (result != RT_EOK)
        {
            LOG_E("WIZnet network initialize failed, DHCP timeout.");
            sal_netif_low_level_set_status(sal_netif_get_by_name(wiz_netif_name), RT_FALSE);
            sal_netif_low_level_set_link_status(sal_netif_get_by_name(wiz_netif_name), RT_FALSE);
            return result;
        }
    }
#endif

    netif = sal_netif_get_by_name(wiz_netif_name);
    sal_netif_low_level_set_status(netif, RT_TRUE);
    sal_netif_low_level_set_link_status(netif, RT_TRUE);
    ctlnetwork(CN_GET_NETINFO, (void *)&net_info);
    sal_netif_low_level_set_ipaddr(netif, (const ip_addr_t *)&net_info.ip);
    sal_netif_low_level_set_gw(netif, (const ip_addr_t *)&net_info.gw);
    sal_netif_low_level_set_netmask(netif, (const ip_addr_t *)&net_info.sn);
    sal_netif_low_level_set_dns_server(netif, 0, (const ip_addr_t *)&net_info.dns);
    memcpy(netif->hwaddr, (const void *)&net_info.mac, netif->hwaddr_len);
    /* 1 - Static, 2 - DHCP */
    sal_netif_low_level_set_dhcp_status(netif, net_info.dhcp - 1);

    return RT_EOK;
}

/* wizenet socket initialize */
static int wiz_socket_init(void)
{
    int idx = 0;

    /* socket(0-7) initialize */
    setSIMR(0xff);

    /* set socket receive/send buffer size */
    for (idx = 0; idx < WIZ_SOCKETS_NUM; idx++)
    {
        setSn_RXBUF_SIZE(idx, 0x02);
        setSn_TXBUF_SIZE(idx, 0x02);
    }

    /* set socket ISR state support */
    for (idx = 0; idx < WIZ_SOCKETS_NUM; idx++)
    {
        setSn_IMR(idx, (IMR_TIMEOUT | IMR_RECV | IMR_DISCON));
    }

    return RT_EOK;
}

/* set WIZnet device MAC address */
int wiz_set_mac(const char *mac)
{
    int result = RT_EOK;

    RT_ASSERT(mac);

    result = wiz_netstr_to_array(mac, wiz_net_info.mac);
    if (result != RT_EOK)
    {
        return result;
    }

    if (wiz_init_ok == RT_TRUE)
    {
        /* set default MAC address to chip */
        setSHAR(wiz_net_info.mac);
    }

    return RT_EOK;
}

static void wiz_dns_time_handler(void* arg)
{
    extern void DNS_time_handler(void);
    DNS_time_handler();
}

static int wiz_netif_set_up(struct sal_netif *netif)
{
    LOG_D("wiz network interface set up status.");
    return RT_EOK;
}

static int wiz_netif_set_down(struct sal_netif *netif)
{
    LOG_D("wiz network interface set down status.");
    return RT_EOK;
}

static int wiz_netif_set_addr_info(struct sal_netif *netif, ip_addr_t *ip_addr, ip_addr_t *netmask, ip_addr_t *gw)
{
    rt_err_t result = RT_EOK;

    RT_ASSERT(netif);
    RT_ASSERT(ip_addr || netmask || gw);

    ctlnetwork(CN_GET_NETINFO, (void *)&wiz_net_info);

    if (ip_addr)
        rt_memcpy(wiz_net_info.ip, &ip_addr->addr, sizeof(wiz_net_info.ip));

    if (netmask)
        rt_memcpy(wiz_net_info.sn, &netmask->addr, sizeof(wiz_net_info.sn));

    if (gw)
        rt_memcpy(wiz_net_info.gw, &gw->addr, sizeof(wiz_net_info.gw));

    if (ctlnetwork(CN_SET_NETINFO, (void *)&wiz_net_info) == RT_EOK)
    {
        if (ip_addr)
            sal_netif_low_level_set_ipaddr(netif, ip_addr);

        if (netmask)
            sal_netif_low_level_set_netmask(netif, netmask);

        if (gw)
            sal_netif_low_level_set_gw(netif, gw);

        result = RT_EOK;
    }
    else
    {
        LOG_E("%s set addr info failed!", wiz_netif_name);
        result = -RT_ERROR;
    }

    return result;
}

static int wiz_netif_set_dns_server(struct sal_netif *netif, ip_addr_t *dns_server)
{
    rt_err_t result = RT_EOK;

    RT_ASSERT(netif);
    RT_ASSERT(dns_server);

    ctlnetwork(CN_GET_NETINFO, (void *)&wiz_net_info);

    rt_memcpy(wiz_net_info.dns, &dns_server->addr, sizeof(wiz_net_info.dns));

    if (ctlnetwork(CN_SET_NETINFO, (void *)&wiz_net_info) == RT_EOK)
    {
        sal_netif_low_level_set_dns_server(netif, 0, (const ip_addr_t *)dns_server);
        result = RT_EOK;
    }
    else
    {
        LOG_E("%s set dns server failed!", wiz_netif_name);
        result = -RT_ERROR;
    }

    return result;
}

static int wiz_netif_set_dhcp(struct sal_netif *netif, rt_bool_t is_enabled)
{
    rt_err_t result = RT_EOK;

    RT_ASSERT(netif);

    ctlnetwork(CN_GET_NETINFO, (void *)&wiz_net_info);

    /* 1 - Static, 2 - DHCP */
    wiz_net_info.dhcp = is_enabled + 1;

    if (ctlnetwork(CN_SET_NETINFO, (void *)&wiz_net_info) == RT_EOK)
    {
        sal_netif_low_level_set_dhcp_status(netif, is_enabled);
        result = RT_EOK;
    }
    else
    {
        LOG_E("%s set dhcp info failed!", wiz_netif_name);
        result = -RT_ERROR;
    }

    return result;
}

static int wiz_netif_ping(struct sal_netif *netif, ip_addr_t *ip_addr, size_t data_len,
                              uint32_t timeout, struct sal_netif_ping_resp *ping_resp)
{
    RT_ASSERT(netif);
    RT_ASSERT(ip_addr);
    RT_ASSERT(ping_resp);

    extern int wiz_ping(ip_addr_t *ip_addr,  uint32_t times, struct sal_netif_ping_resp *ping_resp);

    return wiz_ping(ip_addr, timeout, ping_resp);
}

void wiz_netif_netstat(struct sal_netif *netif)
{
    // TODO
    return;
}

const struct sal_netif_ops sal_wiz_netif_ops =
{
    wiz_netif_set_up,
    wiz_netif_set_down,

    wiz_netif_set_addr_info,
    wiz_netif_set_dns_server,
    wiz_netif_set_dhcp,

    wiz_netif_ping,
    wiz_netif_netstat,
};

static int wiz_netif_add(const char *name)
{
#define ETHERNET_MTU        1472
#define HWADDR_LEN          6
    struct sal_netif *sal_netif = RT_NULL;

    sal_netif = (struct sal_netif *)rt_calloc(1, sizeof(struct sal_netif));
    if (sal_netif == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    sal_netif->flags = SAL_NETIF_FLAG_DHCP;
    sal_netif->mtu = ETHERNET_MTU;
    sal_netif->ops = &sal_wiz_netif_ops;
    sal_netif->hwaddr_len = HWADDR_LEN;

    extern int sal_netif_wiz_ops_register(struct sal_netif * netif);
    /* set the network interface socket/netdb operations */
    sal_netif_wiz_ops_register(sal_netif);

    return sal_netif_register(sal_netif, name, RT_NULL);
}

/* WIZnet initialize device and network */
int wiz_init(void)
{
    int result = RT_EOK;

    if (wiz_init_ok == RT_TRUE)
    {
        LOG_I("RT-Thread WIZnet package is already initialized.");
        return RT_EOK;
    }

    result = wiz_set_mac(WIZ_DEFAULT_MAC);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    /* WIZnet SPI device and pin initialize */
    result = wiz_device_init(WIZ_SPI_DEVICE, WIZ_RST_PIN, WIZ_IRQ_PIN);
    if (result != RT_EOK)
    {
        goto __exit;
    }

    /* Add wiz to the netif list */
    ctlwizchip(CW_GET_ID, (void*) wiz_netif_name);
    wiz_netif_add(wiz_netif_name);

    /* WIZnet SPI device reset */
    wiz_reset();
    /* set WIZnet device read/write data callback */
    wiz_callback_register();
    /* WIZnet chip configure initialize */
    result = wiz_chip_cfg_init();
    if (result != RT_EOK)
    {
        goto __exit;
    }
    /* WIZnet network initialize */
    result = wiz_network_init();
    if (result != RT_EOK)
    {
        goto __exit;
    }
    /* WIZnet socket initialize */
    wiz_socket_init();

    /* WIZnet socket register */
    wiz_inet_init();
    
    dns_tick_timer = rt_timer_create("dns_tick", wiz_dns_time_handler, RT_NULL, 1*RT_TICK_PER_SECOND, RT_TIMER_FLAG_SOFT_TIMER|RT_TIMER_FLAG_PERIODIC);
    rt_timer_start(dns_tick_timer);

__exit:
    if (result == RT_EOK)
    {
        wiz_init_ok = RT_TRUE;
        LOG_I("RT-Thread WIZnet package (V%s) initialize success.", WIZ_SW_VERSION);
    }
    else
    {
        LOG_E("RT-Thread WIZnet package (V%s) initialize failed(%d).", WIZ_SW_VERSION, result);
    }

    return result;
}
INIT_ENV_EXPORT(wiz_init);
