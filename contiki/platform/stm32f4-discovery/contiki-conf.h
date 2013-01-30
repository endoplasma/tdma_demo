#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

/* Platform name, type, and MCU clock rate */
#define PLATFORM_NAME  "STM32F4-Discovery"
#define PLATFORM_TYPE  STM32F4_Discovery

#define LEDS_GREEN  1
#define LEDS_YELLOW  2
#define LEDS_RED  4
#ifdef LEDS_BLUE
#undef LEDS_BLUE
#endif
#define LEDS_BLUE  8
#define LEDS_CONF_ALL 0xf

#include <stdint.h>

#define CCIF
#define CLIF

#define WITH_UIP 1
#define WITH_ASCII 1

#define CLOCK_CONF_SECOND 1000


//#define SLIP_PORT RS232_PORT_1

/* RADIOSTATS is used in rf230bb, clock.c and the webserver cgi to report radio usage */
/* It has less overhead than ENERGEST */
#define RADIOSTATS                1

/* More extensive stats, via main loop printfs or webserver status pages */
#define ENERGEST_CONF_ON          1

/* Packet statistics */
#define UIP_STATISTICS            0

/* Network setup */
/* TX routine passes the cca/ack result in the return parameter */
#define RDC_CONF_HARDWARE_ACK    1
/* TX routine does automatic cca and optional backoff */
#define RDC_CONF_HARDWARE_CSMA   1
/* Allow MCU sleeping between channel checks */
#define RDC_CONF_MCU_SLEEP         1

#if UIP_CONF_IPV6
#define RIMEADDR_CONF_SIZE        8
#define UIP_CONF_ICMP6            1
#define UIP_CONF_UDP              1
#define UIP_CONF_TCP              1
#define NETSTACK_CONF_NETWORK     sicslowpan_driver
#define SICSLOWPAN_CONF_COMPRESSION SICSLOWPAN_COMPRESSION_HC06
#else
/* ip4 should build but is largely untested */
#define RIMEADDR_CONF_SIZE        2
#define NETSTACK_CONF_NETWORK     rime_driver
#endif

#define UIP_CONF_LL_802154        1
#define UIP_CONF_LLH_LEN          0

/* 10 bytes per stateful address context - see sicslowpan.c */
/* Default is 1 context with prefix aaaa::/64 */
/* These must agree with all the other nodes or there will be a failure to communicate! */
#define SICSLOWPAN_CONF_MAX_ADDR_CONTEXTS 1
#define SICSLOWPAN_CONF_ADDR_CONTEXT_0 {addr_contexts[0].prefix[0]=0xaa;addr_contexts[0].prefix[1]=0xaa;}

/* Take the default TCP maximum segment size for efficiency and simpler wireshark captures */
/* Use this to prevent 6LowPAN fragmentation (whether or not fragmentation is enabled) */
//#define UIP_CONF_TCP_MSS       48

#define UIP_CONF_IP_FORWARD      0
#define UIP_CONF_FWCACHE_SIZE    0

#define UIP_CONF_IPV6_CHECKS     1
#define UIP_CONF_IPV6_QUEUE_PKT  1
#define UIP_CONF_IPV6_REASSEMBLY 0

#define UIP_CONF_UDP_CHECKSUMS   1
#define UIP_CONF_TCP_SPLIT       1
#define UIP_CONF_DHCP_LIGHT      1

#define NETSTACK_CONF_MAC         nullmac_driver
#define NETSTACK_CONF_RDC         nullrdc_driver
#define NETSTACK_CONF_FRAMER      framer_802154
#define NETSTACK_CONF_RADIO       rf230_driver

#define NULLRDC_CONF_802154_AUTOACK_HW 1
#define SICSLOWPAN_CONF_MAX_MAC_TRANSMISSIONS 4


#define CHANNEL_802_15_4          21
/* AUTOACK receive mode gives better rssi measurements, even if ACK is never requested */
#define RF230_CONF_AUTOACK        1
/* Request 802.15.4 ACK on all packets sent (else autoretry). This is primarily for testing. */
#define SICSLOWPAN_CONF_ACK_ALL   1
/* Number of auto retry attempts 0-15 (0 implies don't use extended TX_ARET_ON mode with CCA) */
#define RF230_CONF_AUTORETRIES    2
/* Default is one RAM buffer for received packets. More than one may benefit multiple TCP connections or ports */
#define RF230_CONF_RX_BUFFERS     3
#define SICSLOWPAN_CONF_FRAG      1
/* Most browsers reissue GETs after 3 seconds which stops fragment reassembly so a longer MAXAGE does no good */
#define SICSLOWPAN_CONF_MAXAGE    3
/* How long to wait before terminating an idle TCP connection. Smaller to allow faster sleep. Default is 120 seconds */
/* If wait is too short the connection can be reset as a result of multiple fragment reassembly timeouts */
#define UIP_CONF_WAIT_TIMEOUT    20
/* 211 bytes per queue buffer */
#define QUEUEBUF_CONF_NUM         8
/* 54 bytes per queue ref buffer */
#define QUEUEBUF_CONF_REF_NUM     2
/* Allocate remaining RAM as desired */
/* 30 bytes per TCP connection */
/* 6LoWPAN does not do well with concurrent TCP streams, as new browser GETs collide with packets coming */
/* from previous GETs, causing decreased throughput, retransmissions, and timeouts. Increase to study this. */
/* ACKs to other ports become interleaved with computation-intensive GETs, so ACKs are particularly missed. */
/* Increasing the number of packet receive buffers in RAM helps to keep ACKs from being lost */
#define UIP_CONF_MAX_CONNECTIONS  4
/* 2 bytes per TCP listening port */
#define UIP_CONF_MAX_LISTENPORTS  4
/* 25 bytes per UDP connection */
#define UIP_CONF_UDP_CONNS       10
/* See uip-ds6.h */
#define UIP_CONF_DS6_NBR_NBU      20
#define UIP_CONF_DS6_DEFRT_NBU    2
#define UIP_CONF_DS6_PREFIX_NBU   3
#define UIP_CONF_DS6_ROUTE_NBU    20
#define UIP_CONF_DS6_ADDR_NBU     3
#define UIP_CONF_DS6_MADDR_NBU    0
#define UIP_CONF_DS6_AADDR_NBU    0
	
/* ************************************************************************** */
//#pragma mark RPL Settings
/* ************************************************************************** */
#if UIP_CONF_IPV6_RPL

#define UIP_CONF_ROUTER                 1
#define UIP_CONF_ND6_SEND_RA		    0
#define UIP_CONF_ND6_REACHABLE_TIME     600000
#define UIP_CONF_ND6_RETRANS_TIMER      10000

/* For slow slip connections, to prevent buffer overruns */
//#define UIP_CONF_RECEIVE_WINDOW 300
#undef UIP_CONF_FWCACHE_SIZE
#define UIP_CONF_FWCACHE_SIZE    30
#define UIP_CONF_BROADCAST       1
#define UIP_ARCH_IPCHKSUM        1
#define UIP_CONF_PINGADDRCONF    0
#define UIP_CONF_LOGGING         0

#endif /* RPL */

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;


/* uIP configuration */
#define UIP_CONF_BUFFER_SIZE 116

#define UIP_CONF_TCP_FORWARD 1

/* Prefix for relocation sections in ELF files */
#define REL_SECT_PREFIX ".rel"

#define CC_BYTE_ALIGNED __attribute__ ((packed, aligned(1)))

/* Include Project Specific conf */
#ifdef PROJECT_CONF_H
#include PROJECT_CONF_H
#endif /* PROJECT_CONF_H */


#endif /* __CONTIKI_CONF_H__ */
