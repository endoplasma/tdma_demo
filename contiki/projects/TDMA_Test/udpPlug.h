#ifndef UDP_PLUG_H
#define	UDP_PLUG_H

#ifdef CONF_HAS_SWITCH
 #define	HAS_SWITCH		CONF_HAS_SWITCH
#else
 #define	HAS_SWITCH		1
#endif

#ifdef CONF_HAS_TEMP_SENSOR
 #define	HAS_TEMP_SENSOR		CONF_HAS_TEMP_SENSOR
#else
 #define	HAS_TEMP_SENSOR		0
#endif

#ifdef CONF_HAS_LIGHT_SENSOR
 #define	HAS_LIGHT_SENSOR	CONF_HAS_LIGHT_SENSOR
#else
 #define	HAS_LIGHT_SENSOR	0
#endif

typedef enum {
  SWITCH_OFF = 0x00,
  SWITCH_ON = 0x01,
  SWITCH_TOGGLE = 0x02
} switch_cmd_t;

#define PROFILE_ROUTING_TABLE           0x00
#define PROFILE_SERVICE_DISCOVER        0x02
#define	PROFILE_SENSOR_DATA		0x10
#define PROFILE_SWITCH			0x20

#define REQUEST_MSG			0x00
#define	REPLY_MSG			0x01

#define SWITCH_BIT			0
#define	TEMP_SENSOR_BIT			1
#define LIGHT_SENSOR_BIT		2

#define PLUG_PORT			13771


#endif
