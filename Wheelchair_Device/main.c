/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Bluetooth mesh light example
 *
 * This example implements a Bluetooth mesh light node.
 *
 ***************************************************************************************************
 * <b> (C) Copyright 2017 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* C Standard Library headers */
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"
#include <gpiointerrupt.h>

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
#include "mesh_generic_model_capi_types.h"
#include "mesh_lighting_model_capi_types.h"
#include "mesh_lib.h"
#include <mesh_sizes.h>

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include <em_gpio.h>

/* Device initialization header */
#include "hal-config.h"

/* LED driver with support for PWM dimming */
#include "led_driver.h"
#include "lcd_driver.h"

/* GPIO Header File */
#include "inc/gpio.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

struct mesh_generic_state current, target;

bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

// Maximum number of simultaneous Bluetooth connections
#define MAX_CONNECTIONS 2

// heap for Bluetooth stack
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// Bluetooth stack configuration
const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .max_timers = 16,
};

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

#define TIMER_ID_RESTART    78
#define TIMER_ID_FACTORY_RESET  77
#define TIMER_ID_PROVISIONING   66
#define TIMER_ID_SAVE_STATE   60
#define TIMER_ID_ONOFF_TRANSITION         53
#define TIMER_ID_LIGHTNESS_TRANSITION     52
#define TIMER_ID_DELAYED_ONOFF            51
#define TIMER_ID_DELAYED_LIGHTNESS        50
#define TIMER_ID_DELAYED_CTL              49
#define TIMER_ID_CTL_TRANSITION           48
#define TIMER_ID_DELAYED_CTL_TEMPERATURE  47
#define TIMER_ID_CTL_TEMP_TRANSITION      46
#define TIMER_ID_DELAYED_PRI_LEVEL        45
#define TIMER_ID_PRI_LEVEL_TRANSITION     44
#define TIMER_ID_DELAYED_SEC_LEVEL        43
#define TIMER_ID_SEC_LEVEL_TRANSITION     42

/*Macros for Direction $ Speed Bits*/
#define DIR_NONE			0x00
#define MOTOR_OFF			0x01
#define MOTOR_ON			0x02
#define DIR_FORWARD  		0x04
#define DIR_REVERSE     	0x08
#define DIR_LEFT     		0x10
#define DIR_RIGHT    		0x20
#define MOTOR_SPEED_DOWN	0x40
#define MOTOR_SPEED_UP		0x80
#define DEFAULT_MOTORSPEED  3
#define MOTORSPEED_MAX      10

#define GESTURE_THRESHOLD            	0x100
#define GESTURE_SENSETIVITY1   			0x200
#define GESTURE_SENSETIVITY2   			0x400
#define GESTURE_THRESHOLD_DECREASE  	0x101
#define GESTURE_SENSETIVITY1_DECREASE 	0x202
#define GESTURE_SENSETIVITY2_DECREASE  	0x404
#define OBSTACLE_ALERT         0X800

/*LCD Macros*/
#define DI_ROW_NAME         1    /* 1st row, device name */
#define DI_ROW_STATUS       2    /* 2nd row, node status */
#define DI_ROW_CONNECTION   3    /* 3rd row, connection status */
#define DI_ROW_FRIEND       4    /* 4th row, friendship FRIEND status */
#define DI_ROW_LPN          4    /* 4th row, friendship LPN status */
#define DI_ROW_DIRECTION    5    /* 5th row, direction of motion */
#define DI_ROW_MOTORONOFF   6    /* 6th row, motor status */
#define DI_ROW_IR           7    /* 7th row, IR Sensor Status */
#define DI_ROW_MAX          7    /* total number of rows used */

#define DI_ROW_LEN        32   /* up to 32 characters per each row */

/*LCD Display related*/
static char LCD_data[DI_ROW_MAX][DI_ROW_LEN];   /* 2D array for storing the LCD content */
static char *MODULE_NAME = "Wheelchair Module";


/** global variables */
static uint16 _primary_elem_index = 0xffff; /* For indexing elements of the node */
static uint16 _secondary_elem_index = 0xffff; /* For indexing elements of the node */
static uint16 _my_address = 0;    /* Address of the Primary Element of the Node */
static uint8 num_connections = 0;     /* number of active Bluetooth connections */
static uint8 conn_handle = 0xFF;      /* handle of the last opened LE connection */
static uint8 init_done = 0;

static PACKSTRUCT(struct wheelchair_state {
  // On/Off Server state
  uint8_t onoff_current;
  uint8_t onoff_target;

  // Transition Time Server state
  uint8_t transtime;

  // On Power Up Server state
  uint8_t onpowerup;

  // Lightness server
  uint16_t lightness_current;
  uint16_t lightness_target;
  uint16_t lightness_last;
  uint16_t lightness_default;

  // Primary Generic Level
  int16_t pri_level_current;
  int16_t pri_level_target;

  // Temperature server
  uint16_t temperature_current;
  uint16_t temperature_target;
  uint16_t temperature_default;
  uint16_t temperature_min;
  uint16_t temperature_max;

  // Delta UV
  int16_t deltauv_current;
  int16_t deltauv_target;
  int16_t deltauv_default;

  // Secondary Generic Level
  int16_t sec_level_current;
  int16_t sec_level_target;
}) wheelchair_state;

int16_t motor_speed;

/* copy of transition delay parameter, needed for delayed on/off, lightness, temperature, ctl and
 * primary and secondary generic level requests */
uint32_t delayed_onoff_trans = 0;
uint32_t delayed_lightness_trans = 0;
uint32_t delayed_ctl_trans = 0;
uint32_t delayed_ctl_temperature_trans = 0;
uint32_t delayed_pri_level_trans = 0;
uint32_t delayed_sec_level_trans = 0;

static int wheelchair_state_load(void);
static int wheelchair_state_store(void);
static void wheelchair_state_changed(void);

/**
 * Set device name in the GATT database. A unique name is generated using
 * the two last bytes from the Bluetooth address of this device. Name is also
 * displayed on the LCD.
 */
void set_device_name(bd_addr *pAddr)
{
  char name[20];
  uint16 res;

  // create unique device name using the last two bytes of the Bluetooth address

  sprintf(name, "Wheelchair Node %x:%x", pAddr->addr[1], pAddr->addr[0]);

  printf("Device name: '%s - %s'\r\n",MODULE_NAME, name);

  res = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(name), (uint8 *)name)->result;
  if (res)
  {
    printf("gecko_cmd_gatt_server_write_attribute_value() failed, code %x\r\n", res);
  }

  // show device name on the LCD
 // LCD_write(MODULE_NAME, DI_ROW_HEADER);
  LCD_write(name, DI_ROW_NAME);
}


static uint32_t default_transition_time(void)
{
  return mesh_lib_transition_time_to_ms(wheelchair_state.transtime);
}

static errorcode_t onoff_response(uint16_t element_index,
                                  uint16_t client_addr,
                                  uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = wheelchair_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = wheelchair_state.onoff_target;

  return mesh_lib_generic_server_response(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          &target,
                                          0,
                                          0x00);
}

static errorcode_t onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_on_off;
  current.on_off.on = wheelchair_state.onoff_current;

  target.kind = mesh_generic_state_on_off;
  target.on_off.on = wheelchair_state.onoff_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_off);
  }

  return e;
}

static void onoff_request(uint16_t model_id,
                          uint16_t element_index,
                          uint16_t client_addr,
                          uint16_t server_addr,
                          uint16_t appkey_index,
                          const struct mesh_generic_request *request,
                          uint32_t transition_ms,
                          uint16_t delay_ms,
                          uint8_t request_flags)
{
  printf("ON/OFF request: requested state=<%s>, transition=%lu, delay=%u\r\n",
         request->on_off ? "ON" : "OFF", transition_ms, delay_ms);

  if (wheelchair_state.onoff_current == request->on_off) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Turning lightbulb <%s>\r\n", request->on_off ? "ON" : "OFF");
    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      wheelchair_state.onoff_current = request->on_off;
      wheelchair_state.onoff_target = request->on_off;
      if (wheelchair_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
        LEDS_SetState(LED_STATE_OFF);
      } else {
        LEDS_SetState(LED_STATE_ON);
      }
    } else if (delay_ms > 0) {
      // a delay has been specified for the light change. Start a soft timer
      // that will trigger the change after the given delay
      // Current state remains as is for now
      wheelchair_state.onoff_target = request->on_off;
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_ONOFF, 1);
      // store transition parameter for later use
      delayed_onoff_trans = transition_ms;
    } else {
      // no delay but transition time has been set.
      wheelchair_state.onoff_target = request->on_off;

      if (request->on_off == MESH_GENERIC_ON_OFF_STATE_OFF) {
        LEDS_SetLevel(0, transition_ms);
      } else {
        // restore last brightness
        wheelchair_state.lightness_target = wheelchair_state.lightness_last;
        LEDS_SetLevel(wheelchair_state.lightness_target, transition_ms);
      }
      // lightbulb current state will be updated when transition is complete
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_ONOFF_TRANSITION, 1);
    }
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    onoff_response(element_index, client_addr, appkey_index);
  } else {
    onoff_update(element_index);
  }
}

static void onoff_change(uint16_t model_id,
                         uint16_t element_index,
                         const struct mesh_generic_state *current,
                         const struct mesh_generic_state *target,
                         uint32_t remaining_ms)
{
  if (current->on_off.on != wheelchair_state.onoff_current) {
    //printf("on-off state changed %u to %u\r\n", wheelchair_state.onoff_current, current->on_off.on);

    wheelchair_state.onoff_current = current->on_off.on;
    wheelchair_state_changed();
  } else {
    //printf("dummy onoff change - same state as before\r\n");
  }
}

static errorcode_t lightness_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_lighting_state_lightness_actual;
  current.lightness.level = wheelchair_state.lightness_current;

  target.kind = mesh_lighting_state_lightness_actual;
  target.lightness.level = wheelchair_state.lightness_target;

  return mesh_lib_generic_server_response(MESH_LIGHTING_LIGHTNESS_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          &target,
                                          0,
                                          0x00);
}

static errorcode_t lightness_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_lighting_state_lightness_actual;
  current.lightness.level = wheelchair_state.lightness_current;

  target.kind = mesh_lighting_state_lightness_actual;
  target.lightness.level = wheelchair_state.lightness_target;

  return mesh_lib_generic_server_update(MESH_LIGHTING_LIGHTNESS_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t lightness_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = lightness_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_LIGHTING_LIGHTNESS_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_lighting_state_lightness_actual);
  }

  return e;
}

static void lightness_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  // for simplicity, this demo assumes that all lightness requests use the actual scale.
  // other type of requests are ignored
  if (request->kind != mesh_lighting_request_lightness_actual)
  {
    return;
  }

  //printf("lightness_request: level=%u, transition=%lu, delay=%u\r\n",
         //request->lightness, transition_ms, delay_ms);

  if (wheelchair_state.lightness_current == request->lightness) {
    //printf("Request for current state received; no op\n");
  } else {
    printf("Setting lightness to <%u>\r\n", request->lightness);
    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      wheelchair_state.lightness_current = request->lightness;
      wheelchair_state.lightness_target = request->lightness;
      if (request->lightness != 0) {
        wheelchair_state.lightness_last = request->lightness;
      }

      // update LED PWM duty cycle
      LEDS_SetLevel(wheelchair_state.lightness_current, 0);
    } else if (delay_ms > 0) {
      // a delay has been specified for the light change. Start a soft timer
      // that will trigger the change after the given delay
      // Current state remains as is for now
      wheelchair_state.lightness_target = request->lightness;
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_LIGHTNESS, 1);
      // store transition parameter for later use
      delayed_lightness_trans = transition_ms;
    } else {
      // no delay but transition time has been set.
      wheelchair_state.lightness_target = request->lightness;
      LEDS_SetLevel(wheelchair_state.lightness_target, transition_ms);

      // lightbulb current state will be updated when transition is complete
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_LIGHTNESS_TRANSITION, 1);
    }
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    lightness_response(element_index, client_addr, appkey_index);
  } else {
    lightness_update(element_index);
  }
}

static void lightness_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  if (current->kind != mesh_lighting_state_lightness_actual) {
    // if kind is not 'actual' then just report the change here, no change to light state
    printf("lightness change, kind %u, value %u\r\n", current->kind, current->lightness.level);
    return;
  }

  if (wheelchair_state.lightness_current != current->lightness.level) {
    printf("lightness_change: from %u to %u\r\n", wheelchair_state.lightness_current, current->lightness.level);
    wheelchair_state.lightness_current = current->lightness.level;
    wheelchair_state_changed();
  } else {
    printf("lightness update -same value (%d)\r\n", wheelchair_state.lightness_current);
  }
}

static errorcode_t pri_level_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_level;
  current.level.level = wheelchair_state.pri_level_current;

  target.kind = mesh_generic_state_level;
  target.level.level = wheelchair_state.pri_level_target;

  return mesh_lib_generic_server_response(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          &target,
                                          0,
                                          0x00);
}

static errorcode_t pri_level_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_level;
  current.level.level = wheelchair_state.pri_level_current;

  target.kind = mesh_generic_state_level;
  target.level.level = wheelchair_state.pri_level_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t pri_level_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = pri_level_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_level);
  }

  return e;
}

static void pri_level_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  // for simplicity, this demo assumes that all level requests use set level.
  // other type of requests are ignored

  uint16_t lightness;
  char _to_display[20];
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_level;
  current.level.level = request->level;

  if (request->kind != mesh_generic_request_level)
  {
    return;
  }
	   LCD_write("",DI_ROW_DIRECTION);
	   LCD_write("",DI_ROW_MOTORONOFF);
	   LCD_write("",DI_ROW_IR);
	   printf("Received Level %d\r\n",request->level);
	   if(request->level == MOTOR_SPEED_DOWN)
	   {
	      if(motor_speed <= DEFAULT_MOTORSPEED) motor_speed = DEFAULT_MOTORSPEED;
	       	 else motor_speed--;

  	    printf("Motor Speed Down, Speed = %5d \r\n",motor_speed);
  	    sprintf(_to_display,"Motor Speed: %5u",motor_speed);
  	    LCD_write(_to_display,DI_ROW_MOTORONOFF);
	   }
	   else if(request->level == MOTOR_SPEED_UP)
	   {
	      if(motor_speed >= MOTORSPEED_MAX) motor_speed = MOTORSPEED_MAX;
	       	  else motor_speed++;

  	    printf("Motor Speed Up, Speed = %5d \r\n",motor_speed);
  	    sprintf(_to_display,"Motor Speed: %5u",motor_speed);
  	    LCD_write(_to_display,DI_ROW_MOTORONOFF);
	   }
	   /*Commands received from Central Device */
	   else if((request->level == GESTURE_THRESHOLD)||(request->level == GESTURE_SENSETIVITY1)||(request->level == GESTURE_THRESHOLD_DECREASE)||(request->level == GESTURE_SENSETIVITY1_DECREASE))
	    {
	     		   printf("\n Request for updating parameters \n");
	     		   int res;
	     		   res = mesh_lib_generic_server_response(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
	     		 				   	   	   	   	   	   	   	   	   0,
	     		 												   3,
	     		 												   0,
	     		 												   &current,
	     		 												   NULL,
	     		 												   0,
	     		 												   0x00);
	     		  if(res)
	     		 	printf("Server Response Failed %x\n",res);
	     		 else
	     		 	printf("Server response Success\n");
	     	   }


    switch (request->level)
    {

    	case DIR_FORWARD:
			printf("Forward \r\n");
			LCD_write("Forward",DI_ROW_DIRECTION);
    		break;
    	case DIR_REVERSE:
    		printf("Reverse \r\n");
    		LCD_write("Reverse",DI_ROW_DIRECTION);
    	    break;
    	case DIR_LEFT:
    		printf("Left \r\n");
    		LCD_write("Left",DI_ROW_DIRECTION);
    	    break;
    	case DIR_RIGHT:
    		printf("Right \r\n");
    		LCD_write("Right",DI_ROW_DIRECTION);
    	    break;
    	case MOTOR_ON:
    		printf("Motor On \r\n");
    		LCD_write("Motor On",DI_ROW_MOTORONOFF);
    		wheelchair_state_load();
    	    break;
    	case MOTOR_OFF:
			printf("Motor Off \r\n");
			LCD_write("Motor Off",DI_ROW_MOTORONOFF);
			wheelchair_state_store();
    	    break;
    	default:
    		printf("None \r\n");
    		 LCD_write("None",DI_ROW_DIRECTION);

    }


    if (transition_ms == 0 && delay_ms == 0)
    { // Immediate change
      wheelchair_state.pri_level_current = request->level;
      printf("LEVEL is %d", wheelchair_state.pri_level_current);
      wheelchair_state.pri_level_target = request->level;
      wheelchair_state.lightness_current = lightness;
      wheelchair_state.lightness_target = lightness;

      // update LED Temperature
      //EDS_SetLevel(lightness, 0);
    }
    /*else if (delay_ms > 0)
    {
      // a delay has been specified for the light change. Start a soft timer
      // that will trigger the change after the given delay
      // Current state remains as is for now
      wheelchair_state.pri_level_target = request->level;
      wheelchair_state.lightness_target = lightness;
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_PRI_LEVEL, 1);
      // store transition parameter for later use
      delayed_pri_level_trans = transition_ms;
    }
    else
    {
      // no delay but transition time has been set.
      wheelchair_state.pri_level_target = request->level;
      wheelchair_state.lightness_target = lightness;
      LEDS_SetLevel(lightness, transition_ms);

      // lightbulb current state will be updated when transition is complete
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_PRI_LEVEL_TRANSITION, 1);
    }
    */
  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED)
  {
    pri_level_response(element_index, client_addr, appkey_index);
  }
  else
  {
    pri_level_update(element_index);
  }
  wheelchair_state_store();
}

static void pri_level_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  if (wheelchair_state.pri_level_current != current->level.level) {
    printf("pri_level_change: from %d to %d\r\n", wheelchair_state.pri_level_current, current->level.level);
    wheelchair_state.pri_level_current = current->level.level;
    wheelchair_state_changed();
  } else {
    printf("pri_level update -same value (%d)\r\n", wheelchair_state.pri_level_current);
  }
}

static errorcode_t ctl_response(uint16_t element_index,
                                uint16_t client_addr,
                                uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_lighting_state_ctl;
  current.ctl.lightness = wheelchair_state.lightness_current;
  current.ctl.temperature = wheelchair_state.temperature_current;
  current.ctl.deltauv = wheelchair_state.deltauv_current;

  target.kind = mesh_lighting_state_ctl;
  target.ctl.lightness = wheelchair_state.lightness_target;
  target.ctl.temperature = wheelchair_state.temperature_target;
  target.ctl.deltauv = wheelchair_state.deltauv_target;

  return mesh_lib_generic_server_response(MESH_LIGHTING_CTL_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          &target,
                                          0,
                                          0x00);
}

static errorcode_t ctl_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_lighting_state_ctl;
  current.ctl.lightness = wheelchair_state.lightness_current;
  current.ctl.temperature = wheelchair_state.temperature_current;
  current.ctl.deltauv = wheelchair_state.deltauv_current;

  target.kind = mesh_lighting_state_ctl;
  target.ctl.lightness = wheelchair_state.lightness_target;
  target.ctl.temperature = wheelchair_state.temperature_target;
  target.ctl.deltauv = wheelchair_state.deltauv_target;

  return mesh_lib_generic_server_update(MESH_LIGHTING_CTL_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t ctl_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = ctl_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_LIGHTING_CTL_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_lighting_state_ctl);
  }

  return e;
}

static void ctl_request(uint16_t model_id,
                        uint16_t element_index,
                        uint16_t client_addr,
                        uint16_t server_addr,
                        uint16_t appkey_index,
                        const struct mesh_generic_request *request,
                        uint32_t transition_ms,
                        uint16_t delay_ms,
                        uint8_t request_flags)
{
  printf("ctl_request: lightness=%u, temperature=%u, delta_uv=%d, transition=%lu, delay=%u\r\n",
         request->ctl.lightness, request->ctl.temperature, request->ctl.deltauv, transition_ms, delay_ms);

  if ((wheelchair_state.lightness_current == request->ctl.lightness)
      && (wheelchair_state.temperature_current == request->ctl.temperature)
      && (wheelchair_state.deltauv_current == request->ctl.deltauv)) {
    printf("Request for current state received; no op\n");
  } else {
    if (wheelchair_state.lightness_current != request->ctl.lightness) {
      printf("Setting lightness to <%u>\r\n", request->lightness);
    }
    if (wheelchair_state.temperature_current != request->ctl.temperature) {
      printf("Setting temperature to <%u>\r\n", request->ctl.temperature);
    }
    if (wheelchair_state.deltauv_current != request->ctl.deltauv) {
      printf("Setting delta UV to <%d>\r\n", request->ctl.deltauv);
    }
    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      wheelchair_state.lightness_current = request->ctl.lightness;
      wheelchair_state.lightness_target = request->ctl.lightness;
      if (request->lightness != 0) {
        wheelchair_state.lightness_last = request->ctl.lightness;
      }

      // update LED PWM duty cycle
      LEDS_SetLevel(wheelchair_state.lightness_current, 0);

      wheelchair_state.temperature_current = request->ctl.temperature;
      wheelchair_state.temperature_target = request->ctl.temperature;
      wheelchair_state.deltauv_current = request->ctl.deltauv;
      wheelchair_state.deltauv_target = request->ctl.deltauv;

      // update LED color temperature
      LEDS_SetTemperature(wheelchair_state.temperature_current, wheelchair_state.deltauv_current, 0);
    } else if (delay_ms > 0) {
      // a delay has been specified for the light change. Start a soft timer
      // that will trigger the change after the given delay
      // Current state remains as is for now
      wheelchair_state.lightness_target = request->ctl.lightness;
      wheelchair_state.temperature_target = request->ctl.temperature;
      wheelchair_state.deltauv_target = request->ctl.deltauv;
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_CTL, 1);
      // store transition parameter for later use
      delayed_ctl_trans = transition_ms;
    } else {
      // no delay but transition time has been set.
      wheelchair_state.lightness_target = request->ctl.lightness;
      wheelchair_state.temperature_target = request->ctl.temperature;
      wheelchair_state.deltauv_target = request->ctl.deltauv;

      LEDS_SetLevel(wheelchair_state.lightness_target, transition_ms);
      LEDS_SetTemperature(wheelchair_state.temperature_target, wheelchair_state.deltauv_target, transition_ms);

      // lightbulb current state will be updated when transition is complete
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_CTL_TRANSITION, 1);
    }
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    ctl_response(element_index, client_addr, appkey_index);
  } else {
    ctl_update(element_index);
  }
}

static void ctl_change(uint16_t model_id,
                       uint16_t element_index,
                       const struct mesh_generic_state *current,
                       const struct mesh_generic_state *target,
                       uint32_t remaining_ms)
{
  if (current->kind != mesh_lighting_state_ctl) {
    // if kind is not 'ctl' then just report the change here
    printf("ctl change, kind %u\r\n", current->kind);
    return;
  }

  if (wheelchair_state.lightness_current != current->ctl.lightness) {
    printf("lightness_change: from %u to %u\r\n", wheelchair_state.lightness_current, current->ctl.lightness);
    wheelchair_state.lightness_current = current->ctl.lightness;
    wheelchair_state_changed();
  } else {
    printf("lightness update -same value (%u)\r\n", wheelchair_state.lightness_current);
  }

  if (wheelchair_state.temperature_current != current->ctl.temperature) {
    printf("temperature_change: from %u to %u\r\n", wheelchair_state.temperature_current, current->ctl.temperature);
    wheelchair_state.temperature_current = current->ctl.temperature;
    wheelchair_state_changed();
  } else {
    printf("temperature update -same value (%u)\r\n", wheelchair_state.temperature_current);
  }

  if (wheelchair_state.deltauv_current != current->ctl.deltauv) {
    printf("deltauv_change: from %d to %d\r\n", wheelchair_state.deltauv_current, current->ctl.deltauv);
    wheelchair_state.deltauv_current = current->ctl.deltauv;
    wheelchair_state_changed();
  } else {
    printf("deltauv update -same value (%d)\r\n", wheelchair_state.deltauv_current);
  }
}

static errorcode_t ctl_setup_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index,
                                      mesh_generic_state_t kind)
{
  struct mesh_generic_state current;

  current.kind = kind;

  switch (kind) {
    case mesh_lighting_state_ctl_default:
      current.ctl.lightness = wheelchair_state.lightness_default;
      current.ctl.temperature = wheelchair_state.temperature_default;
      current.ctl.deltauv = wheelchair_state.deltauv_default;
      break;
    case mesh_lighting_state_ctl_temperature_range:
      current.ctl_temperature_range.min = wheelchair_state.temperature_min;
      current.ctl_temperature_range.max = wheelchair_state.temperature_max;
      break;
    default:
      break;
  }

  return mesh_lib_generic_server_response(MESH_LIGHTING_CTL_SETUP_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t ctl_setup_update(uint16_t element_index, mesh_generic_state_t kind)
{
  struct mesh_generic_state current;

  current.kind = kind;

  switch (kind) {
    case mesh_lighting_state_ctl_default:
      current.ctl.lightness = wheelchair_state.lightness_default;
      current.ctl.temperature = wheelchair_state.temperature_default;
      current.ctl.deltauv = wheelchair_state.deltauv_default;
      break;
    case mesh_lighting_state_ctl_temperature_range:
      current.ctl_temperature_range.min = wheelchair_state.temperature_min;
      current.ctl_temperature_range.max = wheelchair_state.temperature_max;
      break;
    default:
      break;
  }

  return mesh_lib_generic_server_update(MESH_LIGHTING_CTL_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static void ctl_setup_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  switch (request->kind) {
    case mesh_lighting_request_ctl_default:
      printf("ctl_setup_request: state=ctl_default, default_lightness=%u, default_temperature=%u, default_delta_uv=%d",
             request->ctl.lightness, request->ctl.temperature, request->ctl.deltauv);

      if ((wheelchair_state.lightness_default == request->ctl.lightness)
          && (wheelchair_state.temperature_default == request->ctl.temperature)
          && (wheelchair_state.deltauv_default == request->ctl.deltauv)) {
        printf("Request for current state received; no op\n");
      } else {
        if (wheelchair_state.lightness_default != request->ctl.lightness) {
          printf("Setting default lightness to <%u>\r\n", request->ctl.lightness);
          wheelchair_state.lightness_default = request->ctl.lightness;
        }
        if (wheelchair_state.temperature_default != request->ctl.temperature) {
          printf("Setting default temperature to <%u>\r\n", request->ctl.temperature);
          wheelchair_state.temperature_default = request->ctl.temperature;
        }
        if (wheelchair_state.deltauv_default != request->ctl.deltauv) {
          printf("Setting default delta UV to <%d>\r\n", request->ctl.deltauv);
          wheelchair_state.deltauv_default = request->ctl.deltauv;
        }
        wheelchair_state_changed();
      }
      break;

    case mesh_lighting_request_ctl_temperature_range:
      printf("ctl_setup_request: state=ctl_temperature_range, min_temperature=%u, max_temperature=%u",
             request->ctl_temperature_range.min, request->ctl_temperature_range.max);

      if ((wheelchair_state.temperature_min == request->ctl_temperature_range.min)
          && (wheelchair_state.temperature_max == request->ctl_temperature_range.max)) {
        printf("Request for current state received; no op\n");
      } else {
        if (wheelchair_state.temperature_min != request->ctl_temperature_range.min) {
          printf("Setting min temperature to <%u>\r\n", request->ctl_temperature_range.min);
          wheelchair_state.temperature_min = request->ctl_temperature_range.min;
        }
        if (wheelchair_state.temperature_max != request->ctl_temperature_range.max) {
          printf("Setting max temperature to <%u>\r\n", request->ctl_temperature_range.max);
          wheelchair_state.temperature_max = request->ctl_temperature_range.max;
        }
        wheelchair_state_changed();
      }
      break;

    default:
      break;
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    ctl_setup_response(element_index, client_addr, appkey_index, request->kind);
  } else {
    ctl_setup_update(element_index, request->kind);
  }
}

static void ctl_setup_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  switch (current->kind) {
    case mesh_lighting_state_ctl_default:
      if (wheelchair_state.lightness_default != current->ctl.lightness) {
        printf("default_lightness_change: from %u to %u\r\n", wheelchair_state.lightness_default, current->ctl.lightness);
        wheelchair_state.lightness_default = current->ctl.lightness;
        wheelchair_state_changed();
      } else {
        printf("default lightness update -same value (%u)\r\n", wheelchair_state.lightness_default);
      }

      if (wheelchair_state.temperature_default != current->ctl.temperature) {
        printf("default_temperature_change: from %u to %u\r\n", wheelchair_state.temperature_default, current->ctl.temperature);
        wheelchair_state.temperature_default = current->ctl.temperature;
        wheelchair_state_changed();
      } else {
        printf("default temperature update -same value (%u)\r\n", wheelchair_state.temperature_default);
      }

      if (wheelchair_state.deltauv_current != current->ctl.deltauv) {
        printf("default_deltauv_change: from %d to %d\r\n", wheelchair_state.deltauv_default, current->ctl.deltauv);
        wheelchair_state.deltauv_default = current->ctl.deltauv;
        wheelchair_state_changed();
      } else {
        printf("default deltauv update -same value (%d)\r\n", wheelchair_state.deltauv_default);
      }

      break;

    case mesh_lighting_state_ctl_temperature_range:
      if (wheelchair_state.temperature_min != current->ctl_temperature_range.min) {
        printf("min_temperature_change: from %u to %u\r\n", wheelchair_state.temperature_min, current->ctl_temperature_range.min);
        wheelchair_state.temperature_min = current->ctl_temperature_range.min;
        wheelchair_state_changed();
      } else {
        printf("min temperature update -same value (%u)\r\n", wheelchair_state.temperature_min);
      }

      if (wheelchair_state.temperature_max != current->ctl_temperature_range.max) {
        printf("max_temperature_change: from %u to %u\r\n", wheelchair_state.temperature_max, current->ctl_temperature_range.max);
        wheelchair_state.temperature_max = current->ctl_temperature_range.max;
        wheelchair_state_changed();
      } else {
        printf("max temperature update -same value (%u)\r\n", wheelchair_state.temperature_max);
      }

      break;

    default:
      break;
  }
}

static errorcode_t ctl_temperature_response(uint16_t element_index,
                                            uint16_t client_addr,
                                            uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_lighting_state_ctl_temperature;
  current.ctl.temperature = wheelchair_state.temperature_current;
  current.ctl.deltauv = wheelchair_state.deltauv_current;

  target.kind = mesh_lighting_state_ctl_temperature;
  target.ctl.temperature = wheelchair_state.temperature_target;
  target.ctl.deltauv = wheelchair_state.deltauv_target;

  return mesh_lib_generic_server_response(MESH_LIGHTING_CTL_TEMPERATURE_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          &target,
                                          0,
                                          0x00);
}

static errorcode_t ctl_temperature_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_lighting_state_ctl_temperature;
  current.ctl.temperature = wheelchair_state.temperature_current;
  current.ctl.deltauv = wheelchair_state.deltauv_current;

  target.kind = mesh_lighting_state_ctl_temperature;
  target.ctl.temperature = wheelchair_state.temperature_target;
  target.ctl.deltauv = wheelchair_state.deltauv_target;

  return mesh_lib_generic_server_update(MESH_LIGHTING_CTL_TEMPERATURE_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t ctl_temperature_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = ctl_temperature_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_LIGHTING_CTL_TEMPERATURE_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_lighting_state_ctl_temperature);
  }

  return e;
}

static void ctl_temperature_request(uint16_t model_id,
                                    uint16_t element_index,
                                    uint16_t client_addr,
                                    uint16_t server_addr,
                                    uint16_t appkey_index,
                                    const struct mesh_generic_request *request,
                                    uint32_t transition_ms,
                                    uint16_t delay_ms,
                                    uint8_t request_flags)
{
  printf("ctl_temperature_request: temperature=%u, delta_uv=%d, transition=%lu, delay=%u\r\n",
         request->ctl_temperature.temperature, request->ctl_temperature.deltauv, transition_ms, delay_ms);

  if ((wheelchair_state.temperature_current == request->ctl_temperature.temperature)
      && (wheelchair_state.deltauv_current == request->ctl_temperature.deltauv)) {
    printf("Request for current state received; no op\n");
  } else {
    if (wheelchair_state.temperature_current != request->ctl_temperature.temperature) {
      printf("Setting temperature to <%u>\r\n", request->ctl_temperature.temperature);
    }
    if (wheelchair_state.deltauv_current != request->ctl_temperature.deltauv) {
      printf("Setting delta UV to <%d>\r\n", request->ctl_temperature.deltauv);
    }
    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      wheelchair_state.temperature_current = request->ctl_temperature.temperature;
      wheelchair_state.temperature_target = request->ctl_temperature.temperature;
      wheelchair_state.deltauv_current = request->ctl_temperature.deltauv;
      wheelchair_state.deltauv_target = request->ctl_temperature.deltauv;

      // update LED color temperature
      LEDS_SetTemperature(wheelchair_state.temperature_current, wheelchair_state.deltauv_current, 0);
    } else if (delay_ms > 0) {
      // a delay has been specified for the temperature change. Start a soft timer
      // that will trigger the change after the given delay
      // Current state remains as is for now
      wheelchair_state.temperature_target = request->ctl_temperature.temperature;
      wheelchair_state.deltauv_target = request->ctl_temperature.deltauv;
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_CTL_TEMPERATURE, 1);
      // store transition parameter for later use
      delayed_ctl_temperature_trans = transition_ms;
    } else {
      // no delay but transition time has been set.
      wheelchair_state.temperature_target = request->ctl_temperature.temperature;
      wheelchair_state.deltauv_target = request->ctl_temperature.deltauv;

      LEDS_SetTemperature(wheelchair_state.temperature_target, wheelchair_state.deltauv_target, transition_ms);

      // lightbulb current state will be updated when transition is complete
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_CTL_TEMP_TRANSITION, 1);
    }
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    ctl_temperature_response(element_index, client_addr, appkey_index);
  } else {
    ctl_temperature_update(element_index);
  }
}

static void ctl_temperature_change(uint16_t model_id,
                                   uint16_t element_index,
                                   const struct mesh_generic_state *current,
                                   const struct mesh_generic_state *target,
                                   uint32_t remaining_ms)
{
  if (wheelchair_state.temperature_current != current->ctl.temperature) {
    printf("temperature_change: from %u to %u\r\n", wheelchair_state.temperature_current, current->ctl.temperature);
    wheelchair_state.temperature_current = current->ctl.temperature;
    wheelchair_state_changed();
  } else {
    printf("temperature update -same value (%u)\r\n", wheelchair_state.temperature_current);
  }

  if (wheelchair_state.deltauv_current != current->ctl.deltauv) {
    printf("deltauv_change: from %d to %d\r\n", wheelchair_state.deltauv_current, current->ctl.deltauv);
    wheelchair_state.deltauv_current = current->ctl.deltauv;
    wheelchair_state_changed();
  } else {
    printf("deltauv update -same value (%d)\r\n", wheelchair_state.deltauv_current);
  }
}

static errorcode_t sec_level_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_level;
  current.level.level = wheelchair_state.sec_level_current;

  target.kind = mesh_generic_state_level;
  target.level.level = wheelchair_state.sec_level_target;

  return mesh_lib_generic_server_response(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          &target,
                                          0,
                                          0x00);
}

static errorcode_t sec_level_update(uint16_t element_index)
{
  struct mesh_generic_state current, target;

  current.kind = mesh_generic_state_level;
  current.level.level = wheelchair_state.sec_level_current;

  target.kind = mesh_generic_state_level;
  target.level.level = wheelchair_state.sec_level_target;

  return mesh_lib_generic_server_update(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        &target,
                                        0);
}

static errorcode_t sec_level_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = sec_level_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_level);
  }

  return e;
}

static void sec_level_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  // for simplicity, this demo assumes that all level requests use set level.
  // other type of requests are ignored

  uint16_t temperature;

  if (request->kind != mesh_generic_request_level) {
    return;
  }

  printf("sec_level_request: level=%d, transition=%lu, delay=%u\r\n",
         request->level, transition_ms, delay_ms);

  if (wheelchair_state.sec_level_current == request->level) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting sec_level to <%d>\r\n", request->level);

    temperature = wheelchair_state.temperature_min                                       \
                  + (uint32_t)(request->level + (int32_t)32768)                         \
                  * (wheelchair_state.temperature_max - wheelchair_state.temperature_min) \
                  / 65535;

    if (transition_ms == 0 && delay_ms == 0) { // Immediate change
      wheelchair_state.sec_level_current = request->level;
      wheelchair_state.sec_level_target = request->level;
      wheelchair_state.temperature_current = temperature;
      wheelchair_state.temperature_target = temperature;

      // update LED Temperature
      LEDS_SetTemperature(temperature, wheelchair_state.deltauv_current, 0);
    } else if (delay_ms > 0) {
      // a delay has been specified for the light change. Start a soft timer
      // that will trigger the change after the given delay
      // Current state remains as is for now
      wheelchair_state.sec_level_target = request->level;
      wheelchair_state.temperature_target = temperature;
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delay_ms), TIMER_ID_DELAYED_SEC_LEVEL, 1);
      // store transition parameter for later use
      delayed_sec_level_trans = transition_ms;
    } else {
      // no delay but transition time has been set.
      wheelchair_state.sec_level_target = request->level;
      wheelchair_state.temperature_target = temperature;
      LEDS_SetTemperature(temperature, wheelchair_state.deltauv_current, transition_ms);

      // lightbulb current state will be updated when transition is complete
      gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_SEC_LEVEL_TRANSITION, 1);
    }
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    sec_level_response(element_index, client_addr, appkey_index);
  } else {
    sec_level_update(element_index);
  }
}

static void sec_level_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  if (wheelchair_state.sec_level_current != current->level.level) {
    printf("sec_level_change: from %d to %d\r\n", wheelchair_state.sec_level_current, current->level.level);
    wheelchair_state.sec_level_current = current->level.level;
    wheelchair_state_changed();
  } else {
    printf("sec_level update -same value (%d)\r\n", wheelchair_state.sec_level_current);
  }
}

static errorcode_t power_onoff_response(uint16_t element_index,
                                        uint16_t client_addr,
                                        uint16_t appkey_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_on_power_up;
  current.on_power_up.on_power_up = wheelchair_state.onpowerup;

  return mesh_lib_generic_server_response(MESH_GENERIC_POWER_ON_OFF_SETUP_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t power_onoff_update(uint16_t element_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_on_power_up;
  current.on_power_up.on_power_up = wheelchair_state.onpowerup;

  return mesh_lib_generic_server_update(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static errorcode_t power_onoff_update_and_publish(uint16_t element_index)
{
  errorcode_t e;

  e = power_onoff_update(element_index);
  if (e == bg_err_success) {
    e = mesh_lib_generic_server_publish(MESH_GENERIC_POWER_ON_OFF_SERVER_MODEL_ID,
                                        element_index,
                                        mesh_generic_state_on_power_up);
  }

  return e;
}

static void power_onoff_request(uint16_t model_id,
                                uint16_t element_index,
                                uint16_t client_addr,
                                uint16_t server_addr,
                                uint16_t appkey_index,
                                const struct mesh_generic_request *request,
                                uint32_t transition_ms,
                                uint16_t delay_ms,
                                uint8_t request_flags)
{
  printf("ON POWER UP request received; state=<%s>\n",
         wheelchair_state.onpowerup == 0 ? "OFF"
         : wheelchair_state.onpowerup == 1 ? "ON"
         : "RESTORE");

  if (wheelchair_state.onpowerup == request->on_power_up) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting onpowerup to <%s>\n",
           request->on_power_up == 0 ? "OFF"
           : request->on_power_up == 1 ? "ON"
           : "RESTORE");
    wheelchair_state.onpowerup = request->on_power_up;
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    power_onoff_response(element_index, client_addr, appkey_index);
  } else {
    power_onoff_update(element_index);
  }
}

static void power_onoff_change(uint16_t model_id,
                               uint16_t element_index,
                               const struct mesh_generic_state *current,
                               const struct mesh_generic_state *target,
                               uint32_t remaining_ms)
{
  // TODO
}

static errorcode_t transtime_response(uint16_t element_index,
                                      uint16_t client_addr,
                                      uint16_t appkey_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_transition_time;
  current.transition_time.time = wheelchair_state.transtime;

  return mesh_lib_generic_server_response(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                          element_index,
                                          client_addr,
                                          appkey_index,
                                          &current,
                                          NULL,
                                          0,
                                          0x00);
}

static errorcode_t transtime_update(uint16_t element_index)
{
  struct mesh_generic_state current;
  current.kind = mesh_generic_state_transition_time;
  current.transition_time.time = wheelchair_state.transtime;

  return mesh_lib_generic_server_update(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                        element_index,
                                        &current,
                                        NULL,
                                        0);
}

static void transtime_request(uint16_t model_id,
                              uint16_t element_index,
                              uint16_t client_addr,
                              uint16_t server_addr,
                              uint16_t appkey_index,
                              const struct mesh_generic_request *request,
                              uint32_t transition_ms,
                              uint16_t delay_ms,
                              uint8_t request_flags)
{
  printf("TRANSTIME request received; state=<0x%x>\n",
         wheelchair_state.transtime);

  if (wheelchair_state.transtime == request->transition_time) {
    printf("Request for current state received; no op\n");
  } else {
    printf("Setting transtime to <0x%x>\n", request->transition_time);
    wheelchair_state.transtime = request->transition_time;
    wheelchair_state_changed();
  }

  if (request_flags & MESH_REQUEST_FLAG_RESPONSE_REQUIRED) {
    transtime_response(element_index, client_addr, appkey_index);
  } else {
    transtime_update(element_index);
  }
}

static void transtime_change(uint16_t model_id,
                             uint16_t element_index,
                             const struct mesh_generic_state *current,
                             const struct mesh_generic_state *target,
                             uint32_t remaining_ms)
{
  // TODO
}

/**
 * This function loads the saved light state from Persistent Storage and
 * copies the data in the global variable wheelchair_state
 */
static int wheelchair_state_load(void)
{
  char _to_print[20];
  printf("Wheelchair State Load \r\n");
  struct gecko_msg_flash_ps_load_rsp_t* pMotorLoad;

  pMotorLoad = gecko_cmd_flash_ps_load(0x2626);

  if(pMotorLoad->result)
  {
	 memset(&motor_speed,0, sizeof(motor_speed));
   	 motor_speed = DEFAULT_MOTORSPEED;
   	 return -1;
   }

  memcpy(&motor_speed, pMotorLoad->value.data, pMotorLoad->value.len);
  printf("Motor Speed from memory is %d",motor_speed);
  sprintf(_to_print,"Pwr On Speed: %d", motor_speed);
  LCD_write(_to_print,DI_ROW_MOTORONOFF);


  return 0;
}

/**
 * this function saves the current light state in Persistent Storage so that
 * the data is preserved over reboots and power cycles. The light state is hold
 * in a global variable wheelchair_state. a PS key with ID 0x4004 is used to store
 * the whole struct.
 */
static int wheelchair_state_store(void)
{
  printf("Wheelchair State Store \r\n");
  struct gecko_msg_flash_ps_save_rsp_t* pMotorSave;

  pMotorSave = gecko_cmd_flash_ps_save(0x2626, sizeof(uint16_t), (const uint8*)&motor_speed);
  if(pMotorSave->result)
  {
	  printf("Motor Speed State Loading Failed \r\n");
	  return -1;
  }
  return 0;
}

/**
 * this function is called each time the lightbulb state in RAM is changed. It sets up a soft timer
 * that will save the state in flash after small delay. The purpose is to reduce amount of unnecessary
 * flash writes.
 */
static void wheelchair_state_changed(void)
{
  gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(5000), TIMER_ID_SAVE_STATE, 1);
}

/**
 * Initialization of the models supported by this node. This function registers callbacks for
 * each of the three supported models.
 */
static void init_models(void)
{
  mesh_lib_generic_server_register_handler(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
                                           0,
                                           onoff_request,
                                           onoff_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_POWER_ON_OFF_SETUP_SERVER_MODEL_ID,
                                           0,
                                           power_onoff_request,
                                           power_onoff_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_TRANSITION_TIME_SERVER_MODEL_ID,
                                           0,
                                           transtime_request,
                                           transtime_change);

  mesh_lib_generic_server_register_handler(MESH_LIGHTING_LIGHTNESS_SERVER_MODEL_ID,
                                           0,
                                           lightness_request,
                                           lightness_change);
  mesh_lib_generic_server_register_handler(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
                                           0,
                                           pri_level_request,
                                           pri_level_change);
  mesh_lib_generic_server_register_handler(MESH_LIGHTING_CTL_SERVER_MODEL_ID,
                                           0,
                                           ctl_request,
                                           ctl_change);
  mesh_lib_generic_server_register_handler(MESH_LIGHTING_CTL_SETUP_SERVER_MODEL_ID,
                                           0,
                                           ctl_setup_request,
                                           ctl_setup_change);
  mesh_lib_generic_server_register_handler(MESH_LIGHTING_CTL_TEMPERATURE_SERVER_MODEL_ID,
                                           1,
                                           ctl_temperature_request,
                                           ctl_temperature_change);

  // uncomment following lines to enable generic level server on secondary element
  //mesh_lib_generic_server_register_handler(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
  //                                         1,
  //                                         sec_level_request,
  //                                         sec_level_change);
}

/**
 * Light node initialization. This is called at each boot if provisioning is already done.
 * Otherwise this function is called after provisioning is completed.
 */
void wheelchair_state_init(void)
{
  uint16 res;

  /* Initialize mesh lib */
  mesh_lib_init(malloc, free, 8);

  //Initialize Friend functionality
  printf("Friend mode initialization\r\n");
  res = gecko_cmd_mesh_friend_init()->result;
  if (res) {
    printf("Friend init failed 0x%x\r\n", res);
  }

  memset(&wheelchair_state, 0, sizeof(struct wheelchair_state));
  if (wheelchair_state_load() != 0) {
    printf("wheelchair_state_load() failed, using defaults\r\n");
    goto publish;
  }

  // Handle on power up behavior
  switch (wheelchair_state.onpowerup) {
    case MESH_GENERIC_ON_POWER_UP_STATE_OFF:
      printf("On power up state is OFF\r\n");
      wheelchair_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_OFF;
      wheelchair_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_OFF;
      LEDS_SetState(LED_STATE_OFF);
      LEDS_SetTemperature(wheelchair_state.temperature_default, wheelchair_state.deltauv_default, 0);
      break;
    case MESH_GENERIC_ON_POWER_UP_STATE_ON:
      printf("On power up state is ON\r\n");
      wheelchair_state.onoff_current = MESH_GENERIC_ON_OFF_STATE_ON;
      wheelchair_state.onoff_target = MESH_GENERIC_ON_OFF_STATE_ON;
      LEDS_SetState(LED_STATE_ON);
      LEDS_SetTemperature(wheelchair_state.temperature_default, wheelchair_state.deltauv_default, 0);
      break;
    case MESH_GENERIC_ON_POWER_UP_STATE_RESTORE:
      printf("On power up state is RESTORE\r\n");
      if (wheelchair_state.onoff_current != wheelchair_state.onoff_target) {
        uint32_t transition_ms = default_transition_time();

        if (transition_ms > 0) {
          printf("Starting on power up transition\r\n");
          if (wheelchair_state.onoff_target == MESH_GENERIC_ON_OFF_STATE_OFF) {
            LEDS_SetLevel(0, transition_ms);
          } else {
            LEDS_SetLevel(0xFFFF, transition_ms);
          }
          // state is updated when transition is complete
          gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(transition_ms), TIMER_ID_ONOFF_TRANSITION, 1);
        } else {
          // update current state without any transition time
          if (wheelchair_state.onoff_target == MESH_GENERIC_ON_OFF_STATE_OFF) {
            LEDS_SetState(LED_STATE_OFF);
          } else {
            LEDS_SetState(LED_STATE_ON);
          }
          wheelchair_state.onoff_current = wheelchair_state.onoff_target;
        }
      } else {
        printf("Keeping loaded state\r\n");
        if (wheelchair_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
          LEDS_SetState(LED_STATE_OFF);
        } else {
          LEDS_SetState(LED_STATE_ON);
        }
      }

      if (wheelchair_state.temperature_current != wheelchair_state.temperature_target) {
        LEDS_SetTemperature(wheelchair_state.temperature_target, wheelchair_state.deltauv_target, wheelchair_state.transtime);
      } else {
        LEDS_SetTemperature(wheelchair_state.temperature_current, wheelchair_state.deltauv_current, 0);
      }
      break;
  }

  publish:
  wheelchair_state_changed();
  init_models();
  onoff_update_and_publish(_primary_elem_index);
  power_onoff_update_and_publish(_primary_elem_index);

  init_done = 1;
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);

/**
 * button initialization. Configure pushbuttons PB0,PB1
 * as inputs.
 */
static void button_init()
{
  // configure pushbutton PB0 and PB1 as inputs, with pull-up enabled
  GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);
  GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);
}

void gpioint()
{
	int res;
	struct mesh_generic_state current;
	current.kind = mesh_generic_state_level;
	current.level.level = OBSTACLE_ALERT;
	printf("IR Interrupt \r\n");
	printf("\n Obstacle Detected \r\n");
	printf("\n Motor Off \r\n");
	LCD_write("Obstacle Alert!",DI_ROW_IR);
	LCD_write("Motor Off",DI_ROW_MOTORONOFF);

	/*Publish the alert to the Remote Node*/
	res = mesh_lib_generic_server_response(MESH_GENERIC_LEVEL_SERVER_MODEL_ID,
		     		 				   	   	   	   	   	   	   	   	   0,
		     		 												   4,
		     		 												   0,
		     		 												   &current,
		     		 												   NULL,
		     		 												   0,
		     		 												   0x00);

	if(res)
		printf("Obstacle Alert Server Response Failed %x\n",res);
	else
		printf("Obstacle Alert Server response Success\n");
}

/* This function is for enabling interrupts when obstacle is detected */
static void enable_ir_interrupt()
{
	printf("Enable IR Interrupts \r\n");
	GPIOINT_Init();
	GPIO_ExtIntConfig(IR_Port, IR_Pin, IR_Pin, true, true, true);
	GPIOINT_CallbackRegister(IR_Pin,gpioint);
}

/**
 * This function is called when a light on/off request with non-zero transition time has completed
 */
void onoff_transition_complete()
{
  // transition done -> set state, update and publish
  wheelchair_state.onoff_current = wheelchair_state.onoff_target;

  printf("transition complete. New state is %s\r\n", wheelchair_state.onoff_current ? "ON" : "OFF");

  wheelchair_state_changed();
  onoff_update_and_publish(_primary_elem_index);
}

/**
 * This function is called when a lightness request with non-zero transition time has completed
 */
void lightness_transition_complete()
{
  // transition done -> set state, update and publish
  wheelchair_state.lightness_current = wheelchair_state.lightness_target;
  if (wheelchair_state.lightness_target != 0) {
    wheelchair_state.lightness_last = wheelchair_state.lightness_target;
  }

  printf("transition complete. New level is %u\r\n", wheelchair_state.lightness_current);

  wheelchair_state_changed();
  lightness_update_and_publish(_primary_elem_index);
}

void pri_level_transition_complete()
{
  // transition done -> set state, update and publish
  wheelchair_state.pri_level_current = wheelchair_state.pri_level_target;
  wheelchair_state.lightness_current = wheelchair_state.lightness_target;

  printf("transition complete. New pri_level is %d\r\n", wheelchair_state.pri_level_current);

  wheelchair_state_changed();
  pri_level_update_and_publish(_primary_elem_index);
}

void ctl_transition_complete()
{
  // transition done -> set state, update and publish
  wheelchair_state.lightness_current = wheelchair_state.lightness_target;
  wheelchair_state.temperature_current = wheelchair_state.temperature_target;
  wheelchair_state.deltauv_current = wheelchair_state.deltauv_target;

  printf("transition complete. New lightness is %u, new temperature is %u and new deltauv is %d\r\n", wheelchair_state.lightness_current, wheelchair_state.temperature_current, wheelchair_state.deltauv_current);

  wheelchair_state_changed();
  ctl_update_and_publish(_primary_elem_index);
}

void ctl_temperature_transition_complete()
{
  // transition done -> set state, update and publish
  wheelchair_state.temperature_current = wheelchair_state.temperature_target;
  wheelchair_state.deltauv_current = wheelchair_state.deltauv_target;

  printf("transition complete. New temperature is %u and new deltauv is %d\r\n", wheelchair_state.temperature_current, wheelchair_state.deltauv_current);

  wheelchair_state_changed();
  ctl_temperature_update_and_publish(_secondary_elem_index);
}

void sec_level_transition_complete()
{
  // transition done -> set state, update and publish
  wheelchair_state.sec_level_current = wheelchair_state.sec_level_target;
  wheelchair_state.temperature_current = wheelchair_state.temperature_target;

  printf("transition complete. New sec_level is %d\r\n", wheelchair_state.sec_level_current);

  wheelchair_state_changed();
  sec_level_update_and_publish(_secondary_elem_index);
}

void delayed_onoff_request()
{
  printf("starting delayed on/off request: %u -> %u, %lu ms\r\n",
         wheelchair_state.onoff_current,
         wheelchair_state.onoff_target,
         delayed_onoff_trans
         );

  if (delayed_onoff_trans == 0) {
    // no transition delay, update state immediately

    wheelchair_state.onoff_current = wheelchair_state.onoff_target;
    if (wheelchair_state.onoff_current == MESH_GENERIC_ON_OFF_STATE_OFF) {
      LEDS_SetState(LED_STATE_OFF);
    } else {
      // restore last brightness level
      LEDS_SetLevel(wheelchair_state.lightness_last, 0);
      wheelchair_state.lightness_current = wheelchair_state.lightness_last;
    }

    wheelchair_state_changed();
    onoff_update_and_publish(_primary_elem_index);
  } else {
    if (wheelchair_state.onoff_target == MESH_GENERIC_ON_OFF_STATE_OFF) {
      LEDS_SetLevel(0, delayed_onoff_trans);
    } else {
      // restore last brightness level, with transition delay
      LEDS_SetLevel(wheelchair_state.lightness_last, delayed_onoff_trans);
      wheelchair_state.lightness_target = wheelchair_state.lightness_last;
    }

    // state is updated when transition is complete
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delayed_onoff_trans), TIMER_ID_ONOFF_TRANSITION, 1);
  }
}

void delayed_lightness_request()
{
  printf("starting delayed lightness request: level %u -> %u, %lu ms\r\n",
         wheelchair_state.lightness_current,
         wheelchair_state.lightness_target,
         delayed_lightness_trans
         );

  LEDS_SetLevel(wheelchair_state.lightness_target, delayed_lightness_trans);

  if (delayed_lightness_trans == 0) {
    // no transition delay, update state immediately
    wheelchair_state.lightness_current = wheelchair_state.lightness_target;
    if (wheelchair_state.lightness_target != 0) {
      wheelchair_state.lightness_last = wheelchair_state.lightness_target;
    }

    wheelchair_state_changed();
    lightness_update_and_publish(_primary_elem_index);
  } else {
    // state is updated when transition is complete
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delayed_lightness_trans), TIMER_ID_LIGHTNESS_TRANSITION, 1);
  }
}

void delayed_pri_level_request()
{
  printf("starting delayed primary level request: level %d -> %d, %lu ms\r\n",
         wheelchair_state.pri_level_current,
         wheelchair_state.pri_level_target,
         delayed_pri_level_trans
         );

  LEDS_SetLevel(wheelchair_state.lightness_target, delayed_pri_level_trans);

  if (delayed_pri_level_trans == 0) {
    // no transition delay, update state immediately
    wheelchair_state.pri_level_current = wheelchair_state.pri_level_target;
    wheelchair_state.lightness_current = wheelchair_state.lightness_target;

    wheelchair_state_changed();
    pri_level_update_and_publish(_primary_elem_index);
  } else {
    // state is updated when transition is complete
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delayed_pri_level_trans), TIMER_ID_PRI_LEVEL_TRANSITION, 1);
  }
}

void delayed_ctl_request()
{
  printf("starting delayed ctl request: lightness %u -> %u, temperature %u -> %u, deltauv %d -> %d, %lu ms\r\n",
         wheelchair_state.lightness_current,
         wheelchair_state.lightness_target,
         wheelchair_state.temperature_current,
         wheelchair_state.temperature_target,
         wheelchair_state.deltauv_current,
         wheelchair_state.deltauv_target,
         delayed_ctl_trans
         );

  LEDS_SetLevel(wheelchair_state.lightness_target, delayed_ctl_trans);
  LEDS_SetTemperature(wheelchair_state.temperature_target, wheelchair_state.deltauv_target, delayed_ctl_trans);

  if (delayed_ctl_trans == 0) {
    // no transition delay, update state immediately
    wheelchair_state.lightness_current = wheelchair_state.lightness_target;
    wheelchair_state.temperature_current = wheelchair_state.temperature_target;
    wheelchair_state.deltauv_current = wheelchair_state.deltauv_target;

    wheelchair_state_changed();
    ctl_update_and_publish(_primary_elem_index);
  } else {
    // state is updated when transition is complete
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delayed_ctl_trans), TIMER_ID_CTL_TRANSITION, 1);
  }
}

void delayed_ctl_temperature_request()
{
  printf("starting delayed ctl temperature request: temperature %u -> %u, deltauv %d -> %d, %lu ms\r\n",
         wheelchair_state.temperature_current,
         wheelchair_state.temperature_target,
         wheelchair_state.deltauv_current,
         wheelchair_state.deltauv_target,
         delayed_ctl_temperature_trans
         );

  LEDS_SetTemperature(wheelchair_state.temperature_target, wheelchair_state.deltauv_target, delayed_ctl_temperature_trans);

  if (delayed_ctl_temperature_trans == 0) {
    // no transition delay, update state immediately
    wheelchair_state.temperature_current = wheelchair_state.temperature_target;
    wheelchair_state.deltauv_current = wheelchair_state.deltauv_target;

    wheelchair_state_changed();
    ctl_temperature_update_and_publish(_secondary_elem_index);
  } else {
    // state is updated when transition is complete
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delayed_ctl_temperature_trans), TIMER_ID_CTL_TEMP_TRANSITION, 1);
  }
}

void delayed_sec_level_request()
{
  printf("starting delayed secondary level request: level %d -> %d, %lu ms\r\n",
         wheelchair_state.sec_level_current,
         wheelchair_state.sec_level_target,
         delayed_sec_level_trans
         );

  LEDS_SetTemperature(wheelchair_state.temperature_target, wheelchair_state.deltauv_current, delayed_sec_level_trans);

  if (delayed_sec_level_trans == 0) {
    // no transition delay, update state immediately
    wheelchair_state.sec_level_current = wheelchair_state.sec_level_target;
    wheelchair_state.temperature_current = wheelchair_state.temperature_target;

    wheelchair_state_changed();
    sec_level_update_and_publish(_secondary_elem_index);
  } else {
    // state is updated when transition is complete
    gecko_cmd_hardware_set_soft_timer(TIMER_MS_2_TIMERTICK(delayed_sec_level_trans), TIMER_ID_SEC_LEVEL_TRANSITION, 1);
  }
}

static void server_state_changed(struct gecko_msg_mesh_generic_server_state_changed_evt_t *pEvt)
{
  int i;

  printf("state changed: ");
  printf("model ID %4.4x, type %2.2x ", pEvt->model_id, pEvt->type);
  for (i = 0; i < pEvt->parameters.len; i++) {
    printf("%2.2x ", pEvt->parameters.data[i]);
  }
  printf("\r\n");
}

/**
 *  this function is called to initiate factory reset. Factory reset may be initiated
 *  by keeping one of the WSTK pushbuttons pressed during reboot. Factory reset is also
 *  performed if it is requested by the provisioner (event gecko_evt_mesh_node_reset_id)
 */
void initiate_factory_reset(void)
{
  printf("factory reset\r\n");
  LCD_write("\n***\nFACTORY RESET\n***", DI_ROW_STATUS);

  /* if connection is open then close it before rebooting */
  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  /* perform a factory reset by erasing PS storage. This removes all the keys and other settings
     that have been configured for this node */
  gecko_cmd_flash_ps_erase_all();
  // reboot after a small delay
  gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

int main()
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Minimize advertisement latency by allowing the advertiser to always
  // interrupt the scanner.
  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  //gecko_bgapi_class_gatt_init();
  gecko_bgapi_class_gatt_server_init();
  gecko_bgapi_class_endpoint_init();
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();
  //mesh_native_bgapi_init();
  gecko_bgapi_class_mesh_node_init();
  //gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();
  //gecko_bgapi_class_mesh_generic_client_init();
  gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  //gecko_bgapi_class_mesh_lpn_init();
  gecko_bgapi_class_mesh_friend_init();

  gecko_initCoexHAL();

  RETARGET_SerialInit();

  /* initialize LEDs and buttons. Note: some radio boards share the same GPIO for button & LED.
   * Initialization is done in this order so that default configuration will be "button" for those
   * radio boards with shared pins. LEDS_init() is called later as needed to (re)initialize the LEDs
   * */
  LEDS_init();
  button_init();
  LCD_init();

  while (1) {
    struct gecko_cmd_packet *evt = gecko_wait_event();
    bool pass = mesh_bgapi_listener(evt);
    if (pass) {
      handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
    }
  }
}

/**
 * Handling of stack events. Both BLuetooth LE and Bluetooth mesh events are handled here.
 */
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  uint16_t result;
  char buf[30];

  struct gecko_msg_mesh_node_provisioning_failed_evt_t  *prov_fail_evt;

  if (NULL == evt)
  {
    return;
  }

  switch (evt_id)
  {
    case gecko_evt_system_boot_id:
      // check pushbutton state at startup. If either PB0 or PB1 is held down then do factory reset
      if (GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN) == 0 || GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
        initiate_factory_reset();
      } else {
        struct gecko_msg_system_get_bt_address_rsp_t *pAddr = gecko_cmd_system_get_bt_address();

        set_device_name(&pAddr->address);

        // Initialize Mesh stack in Node operation mode, for OOB Input
        result = gecko_cmd_mesh_node_init_oob(0x00,0x03,0x0B,0x04,0x00,0x04,0x00)->result;
        if (result) {
          sprintf(buf, "init failed (0x%x)", result);
          LCD_write(buf, DI_ROW_STATUS);
        }

        // re-initialize LEDs (needed for those radio board that share same GPIO for button/LED)
        LEDS_init();
      }
      break;
    case gecko_evt_mesh_node_display_output_oob_id:
    {
        struct gecko_msg_mesh_node_display_output_oob_evt_t *pOOB = (struct gecko_msg_mesh_node_display_output_oob_evt_t *)&(evt->data);
        uint32_t oob_number = pOOB->data.data[pOOB->data.len -1] | (pOOB->data.data[pOOB->data.len -2]<<8) | (pOOB->data.data[pOOB->data.len -3]<<16);
        char oob_str[13] = {0};
        sprintf(oob_str,"Pass-Key: %u",oob_number);
        LCD_write(oob_str,DI_ROW_DIRECTION);
        printf("\n %s \r\n",oob_str);
        break;
    }

    case gecko_evt_hardware_soft_timer_id:
      switch (evt->data.evt_hardware_soft_timer.handle) {
        case TIMER_ID_FACTORY_RESET:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_RESTART:
          gecko_cmd_system_reset(0);
          break;

        case TIMER_ID_PROVISIONING:
          LEDS_SetState(LED_STATE_PROV);
          break;

        case TIMER_ID_SAVE_STATE:
          wheelchair_state_store();
          break;

        case TIMER_ID_DELAYED_ONOFF:
          /* delay for an on/off request has passed, now process the request */
          delayed_onoff_request();
          break;

        case TIMER_ID_DELAYED_LIGHTNESS:
          /* delay for a lightness request has passed, now process the request */
          delayed_lightness_request();
          break;

        case TIMER_ID_DELAYED_PRI_LEVEL:
          /* delay for a primary generic level request has passed, now process the request */
          delayed_pri_level_request();
          break;

        case TIMER_ID_DELAYED_CTL:
          /* delay for a ctl request has passed, now process the request */
          delayed_ctl_request();
          break;

        case TIMER_ID_DELAYED_CTL_TEMPERATURE:
          /* delay for a ctl temperature request has passed, now process the request */
          delayed_ctl_temperature_request();
          break;
        case TIMER_ID_DELAYED_SEC_LEVEL:
          /* delay for a secondary generic level request has passed, now process the request */
          delayed_sec_level_request();
          break;

        case TIMER_ID_ONOFF_TRANSITION:
          onoff_transition_complete();
          break;

        case TIMER_ID_LIGHTNESS_TRANSITION:
          lightness_transition_complete();
          break;

        case TIMER_ID_PRI_LEVEL_TRANSITION:
          pri_level_transition_complete();
          break;

        case TIMER_ID_CTL_TRANSITION:
          ctl_transition_complete();
          break;

        case TIMER_ID_CTL_TEMP_TRANSITION:
          ctl_temperature_transition_complete();
          break;

        case TIMER_ID_SEC_LEVEL_TRANSITION:
          sec_level_transition_complete();
          break;

        default:
          break;
      }

      break;

    case gecko_evt_mesh_node_initialized_id:
      printf("node initialized\r\n");

      gecko_cmd_mesh_generic_server_init();

      struct gecko_msg_mesh_node_initialized_evt_t *pData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);

      if (pData->provisioned) {
        printf("node is provisioned. address:%x, ivi:%ld\r\n", pData->address, pData->ivi);

        _my_address = pData->address;
        _primary_elem_index = 0;   // index of primary element is zero.
        _secondary_elem_index = 1; // index of secondary element is one.
        wheelchair_state_init();

        printf("Light initial state is <%s>\r\n", wheelchair_state.onoff_current ? "ON" : "OFF");
        LCD_write("provisioned", DI_ROW_STATUS);
        GPIO_Init();
        enable_ir_interrupt();
      }
      else
      {
        printf("node is unprovisioned\r\n");
        LCD_write("unprovisioned", DI_ROW_STATUS);

        printf("starting unprovisioned beaconing...\r\n");
        gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // enable ADV and GATT provisioning bearer
      }
      break;

    case gecko_evt_mesh_node_provisioning_started_id:
      printf("Started provisioning\r\n");
      LCD_write("provisioning...", DI_ROW_STATUS);
      // start timer for blinking LEDs to indicate which node is being provisioned
      gecko_cmd_hardware_set_soft_timer(32768 / 4, TIMER_ID_PROVISIONING, 0);
      break;

    case gecko_evt_mesh_node_provisioned_id:
      _primary_elem_index = 0;   // index of primary element is zero.
      _secondary_elem_index = 1; // index of secondary element is one.
      wheelchair_state_init();
      printf("node provisioned, got address=%x\r\n", evt->data.evt_mesh_node_provisioned.address);
      // stop LED blinking when provisioning complete
      gecko_cmd_hardware_set_soft_timer(0, TIMER_ID_PROVISIONING, 0);
      LEDS_SetState(LED_STATE_OFF);
      LEDS_SetTemperature(DEFAULT_TEMPERATURE, DEFAULT_DELTAUV, 0);
      LCD_write("provisioned", DI_ROW_STATUS);
      LCD_write("",DI_ROW_DIRECTION);
      GPIO_Init();
      enable_ir_interrupt();
      break;

    case gecko_evt_mesh_node_provisioning_failed_id:
      prov_fail_evt = (struct gecko_msg_mesh_node_provisioning_failed_evt_t  *)&(evt->data);
      printf("provisioning failed, code %x\r\n", prov_fail_evt->result);
      LCD_write("prov failed", DI_ROW_STATUS);
      /* start a one-shot timer that will trigger soft reset after small delay */
      gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_RESTART, 1);
      break;

    case gecko_evt_mesh_node_key_added_id:
      printf("got new %s key with index %x\r\n", evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application",
             evt->data.evt_mesh_node_key_added.index);
      break;

    case gecko_evt_mesh_node_model_config_changed_id:
      printf("model config changed\r\n");
      break;

    case gecko_evt_mesh_generic_server_client_request_id:
      printf("evt gecko_evt_mesh_generic_server_client_request_id\r\n");
      mesh_lib_generic_server_event_handler(evt);
      break;

    case gecko_evt_mesh_generic_server_state_changed_id:
      // pass the server state changed event to mesh lib handler that will invoke
      // the callback functions registered by application
      mesh_lib_generic_server_event_handler(evt);
      break;

    case gecko_evt_mesh_node_reset_id:
      printf("evt gecko_evt_mesh_node_reset_id\r\n");
      initiate_factory_reset();
      break;

    case gecko_evt_mesh_friend_friendship_established_id:
      printf("evt gecko_evt_mesh_friend_friendship_established, lpn_address=%x\r\n", evt->data.evt_mesh_friend_friendship_established.lpn_address);
      LCD_write("FRIEND", DI_ROW_FRIEND);
      break;

    case gecko_evt_mesh_friend_friendship_terminated_id:
      printf("evt gecko_evt_mesh_friend_friendship_terminated, reason=%x\r\n", evt->data.evt_mesh_friend_friendship_terminated.reason);
      LCD_write("NO LPN", DI_ROW_FRIEND);
      break;

    case gecko_evt_le_gap_adv_timeout_id:
      // adv timeout events silently discarded
      break;

    case gecko_evt_le_connection_opened_id:
      printf("evt:gecko_evt_le_connection_opened_id\r\n");
      num_connections++;
      conn_handle = evt->data.evt_le_connection_opened.connection;
      LCD_write("connected", DI_ROW_CONNECTION);
      break;

    case gecko_evt_le_connection_parameters_id:
      printf("evt:gecko_evt_le_connection_parameters_id\r\n");
      break;

    case gecko_evt_le_connection_closed_id:
      /* Check if need to boot to dfu mode */
      if (boot_to_dfu) {
        /* Enter to DFU OTA mode */
        gecko_cmd_system_reset(2);
      }

      printf("evt:conn closed, reason 0x%x\r\n", evt->data.evt_le_connection_closed.reason);
      conn_handle = 0xFF;
      if (num_connections > 0) {
        if (--num_connections == 0) {
          LCD_write("", DI_ROW_CONNECTION);
        }
      }
      break;
    case gecko_evt_gatt_server_user_write_request_id:
      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
        /* Set flag to enter to OTA mode */
        boot_to_dfu = 1;
        /* Send response to Write Request */
        gecko_cmd_gatt_server_send_user_write_response(
          evt->data.evt_gatt_server_user_write_request.connection,
          gattdb_ota_control,
          bg_err_success);

        /* Close connection to enter to DFU OTA mode */
        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
      }
      break;

    /*case gecko_evt_system_external_signal_id:
    {
      uint16_t current_level;
      char tmp[21];

      if (init_done && (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_LED_LEVEL_CHANGED)) {
         this signal from the LED PWM driver indicates that the level has changed,
         * we use it here to update the LCD status
        current_level = LEDS_GetLevel();
        sprintf(tmp, "Lightness: %5u%%", ++current_level * 100 / 65535);
        //LCD_write(tmp, DI_ROW_DIRECTION);
      }

      if (init_done && (evt->data.evt_system_external_signal.extsignals & EXT_SIGNAL_LED_TEMPERATURE_CHANGED)) {
         this signal from the LED driver indicates that the temperature level has changed,
         * we use it here to update the LCD status
        current_level = LEDS_GetTemperature();
        sprintf(tmp, "ColorTemp: %5uK", current_level);
        //LCD_write(tmp, DI_ROW_MOTORONOFF);

        current_level = LEDS_GetDeltaUV();
        sprintf(tmp, "Delta UV: %6d ", current_level);
        //LCD_write(tmp, DI_ROW_DELTAUV);
      }
    }
    break;*/

    default:
      //printf("unhandled evt: %8.8x class %2.2x method %2.2x\r\n", evt_id, (evt_id >> 16) & 0xFF, (evt_id >> 24) & 0xFF);
      break;
  }
}
