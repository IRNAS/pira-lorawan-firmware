#include "pira_settings.h"

settings_packet_t pira_settings_packet = 
{ 
    pira_settings_packet.data.status_time = TIME_INIT_VALUE,
    pira_settings_packet.data.status_battery = 0,
    pira_settings_packet.data.safety_power_period = ON_PERIOD_INIT_VALUE_s,
    pira_settings_packet.data.safety_sleep_period = OFF_PERIOD_INIT_VALUE_s,
    pira_settings_packet.data.operational_wakeup = OFF_PERIOD_INIT_VALUE_s,
    pira_settings_packet.data.safety_reboot = REBOOT_TIMEOUT_s,
    pira_settings_packet.data.turnOnRpi = 0
};

/*** end of file ***/