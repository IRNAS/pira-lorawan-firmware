#ifndef BATTERY_VOLTAGE_H
#define BATTERY_VOLTAGE_H

#include "board.h" 
#include "stm32l0_adc.h"

#define RESISTOR_LOWER_KOHM 100     //100kOhm 
#define RESISTOR_UPPER_KOHM 226     //226kOhm 
#define REFERENCE_VOLTAGE_V 2.611f
#define ADC_MAX             4095   
#define RESOLUTION          12      // 12 bit resolution 

void init_battery_adc(void);
uint16_t get_raw_battery_voltage(void);
float get_battery_voltage(uint16_t adcValue);

#endif /* BATTERY_VOLTAGE_H */
/*** end of file ***/