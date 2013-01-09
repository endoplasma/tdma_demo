#ifndef UDP_PLUG_H
#define	UDP_PLUG_H

#define DEV_BOARD 1

#ifdef DEV_BOARD
	#define LIGHT_ON() 		leds_on(LEDS_YELLOW)
	#define LIGHT_OFF()		leds_off(LEDS_YELLOW)
	#define LIGHT_TOGGLE()	leds_toggle(LEDS_YELLOW)
	
	#define NODE_TYPE	0
#else
	#define LIGHT_ON() 		leds_on(LEDS_YELLOW | LEDS_RED); \
							leds_off(LEDS_GREEN)
							
	#define LIGHT_OFF()		leds_off(LEDS_YELLOW | LEDS_RED); \
							leds_on(LEDS_GREEN)
							
	#define LIGHT_TOGGLE()	leds_toggle(LEDS_YELLOW | LEDS_GREEN | LEDS_RED);
	#define NODE_TYPE	0
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

#ifdef CONF_HAS_PIR_SENSOR
	#define	HAS_PIR_SENSOR		CONF_HAS_PIR_SENSOR
#else
	#define	HAS_PIR_SENSOR		0
#endif

#ifdef CONF_HAS_SWITCH
	#define	HAS_SWITCH		CONF_HAS_SWITCH
#else
	#define	HAS_SWITCH		0
#endif

#ifdef CONF_HAS_PIR_SENSOR
	#define	HAS_PIR_SENSOR		CONF_HAS_PIR_SENSOR
#else
	#define	HAS_PIR_SENSOR		0
#endif


typedef enum {
	SWITCH_OFF = 0x00,
	SWITCH_ON = 0x01,
	SWITCH_TOGGLE = 0x02
} switch_cmd_t;

typedef enum {
	PIR_N_REGISTERED = 0x00,
	PIR_REGISTERED = 0x01
} pir_registered_t;


#define PROFILE_ROUTING_TABLE		0x00
#define PROFILE_SERVICE_DISCOVER	0x02
#define	PROFILE_SENSOR_DATA			0x10
#define PROFILE_SWITCH				0x20
#define PROFILE_PIR					0x30

#define REQUEST_MSG					0x00
#define	REPLY_MSG					0x01

#define PIR_REGISTER				1
#define PIR_UNREGISTER				0

#define SWITCH_BIT					0
#define	TEMP_SENSOR_BIT				1
#define LIGHT_SENSOR_BIT			2
#define PIR_SENSOR_BIT				3

#define PLUG_PORT					13771


void plug_send_message(uip_ipaddr_t *dest, void *data, uint8_t length);
void handle_service_discover_frame(uint8_t *frame);
void handle_sensor_data_frame(uint8_t *frame);
void handle_switch_frame(uint8_t *frame);
void handle_pir_frame(uint8_t *frame);
void send_pir_request(uint8_t *frame);

#endif
