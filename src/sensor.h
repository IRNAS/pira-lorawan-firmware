#ifndef SENSOR_H_
#define SENSOR_H_

#include "Arduino.h"
#include "TimerMillis.h"
#include <STM32L0.h>
#include "lorawan.h"
#include "settings.h"
#include "status.h"
#include "GNSS.h"
#include "board.h"
#include "Wire.h"

#define LIS2DH12_ADDR 0x19
#define LIS2DW12_WHO_AM_I                    0x0FU
#define LIS2DW12_CTRL1                       0x20U

extern boolean sensor_send_flag;

/**
 * @brief LoraWAN sensor packet setup - port 1
 * 
 */
struct sensorData_t{
  uint8_t lat1;
  uint8_t lat2;
  uint8_t lat3;
  uint8_t lon1;
  uint8_t lon2;
  uint8_t lon3;
  uint16_t alt;
  uint8_t satellites_hdop;
  uint8_t time_to_fix;
  uint16_t epe;
  uint8_t lux;
  uint8_t motion;
}__attribute__((packed));

union sensorPacket_t{
  sensorData_t data;
  byte bytes[sizeof(sensorData_t)];
};

static const uint8_t sensor_packet_port = 1;
extern sensorPacket_t sensor_packet;
extern boolean sensor_gps_active;
extern boolean sensor_gps_done;

void sensor_timer_callback(void);
void sensor_scheduler(void);
void sensor_system_functions_load(void);
boolean sensor_gps_busy_timeout(uint16_t timeout);
boolean sensor_gps_init(void);
boolean sensor_gps_start(void);
void sensor_gps_acquiring_callback(void);
void sensor_gps_stop(boolean good_fix);
void sensor_gps_stop(void);
void sensor_init(void);
boolean sensor_read(void);
boolean sensor_send(void);

#endif