#include "battery_voltage.h"

/**
 * @brief Prepares adc for reading battery voltage 
 *
 * @return none (void)
 */
void init_battery_adc(void)
{
    // Enable adc  
    stm32l0_adc_enable();
}
 
/**
 * @brief Returns voltage in adc counts  
 *
 * @return uint16_t 
 */
uint16_t get_raw_battery_voltage(void)
{
    // Configure channel and start conversion
    return (uint16_t)stm32l0_adc_read(BATTERY_VOLTAGE, 5); // 12 bit
}
    
/**
 * @brief Returns battery voltage in volts 
 *
 * @param[in] adcValue
 * @return float 
 */
float get_battery_voltage(uint16_t adcValue)
{
    return (adcValue * REFERENCE_VOLTAGE_V * 
            (RESISTOR_LOWER_KOHM + RESISTOR_UPPER_KOHM) / 
            (float)(ADC_MAX * RESISTOR_LOWER_KOHM));
}
/*** end of file ***/
