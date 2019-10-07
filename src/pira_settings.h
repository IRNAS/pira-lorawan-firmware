#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "Arduino.h"
#include "board.h"
#include <STM32L0.h>

#define TIME_INIT_VALUE             (1514764800UL)  // Initial Time is Mon, 1 Jan 2018 00:00:00
#define ON_PERIOD_INIT_VALUE_s      (7200)
#define OFF_PERIOD_INIT_VALUE_s     (7200)
#define RX_BUFFER_SIZE              (7)             // Size in B, do not change, comunication protocol between Pira and RPi depends on this
#define WATCHDOG_RESET_VALUE_s      (15000)
#define REBOOT_TIMEOUT_s            (60)

struct settings_pira_t{
    uint64_t status_time;
    uint16_t status_battery;
    uint32_t safety_power_period;
    uint32_t safety_sleep_period;
    uint32_t safety_reboot;
    uint32_t operational_wakeup;
    uint8_t turnOnRpi; //TODO actually implement this functionality, it is missing the command from outside
}__attribute__((packed));

union settings_packet_t{
    settings_pira_t data;
    uint8_t bytes[sizeof(settings_pira_t)];
};

extern settings_packet_t pira_settings_packet;

void settings_init(void);

#endif

/*** end of file ***/