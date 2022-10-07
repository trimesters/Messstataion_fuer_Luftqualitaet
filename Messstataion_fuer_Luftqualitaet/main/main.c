/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */


#include "define.h"

#include "i2c_module.h"

#include "bme680.h"
#include "ccs811.h"


#define MAXLENGTH     16

#define CLEAR_DISPLAY_TIMER_SEC         3600
#define SET_TEMPERATUR_HUMIDITY_SEC     7200



int8_t ccs811_read_print(ssd1306_device_t* ssd1306Device_, ccs811_device_t* ccs811Device_, ccs811_result_t* ccs811Result_, char* buffer_name);
int8_t bme680_read_print(ssd1306_device_t* ssd1306Device_, bme680_device_t* bme680Device_, bme680_result_t* data_, char* buffer_name);

int8_t int16_t_to_string(int16_t vaule_, char* name_, char* buffer_name);
int8_t double_t_to_string(double vaule_, char* name_, char* buffer_name);

void app_main(void)
{
    uint64_t clear_start            = 0;
    uint64_t clear_end              = 0;

    uint64_t set_start              = 0;
    uint64_t set_end                = 0;

    int8_t flagCCS811                   = -1;
    int8_t flagBME680                   = -1;
    int8_t flagSSD1306                  = -1;
    

    char buffer_name[MAXLENGTH]     = {'\0'};

    bme680_device_t bme680Device;
    bme680_result_t bme680Data;

    ccs811_device_t ccs811Device;
    ccs811_result_t ccs811Result;

    ssd1306_device_t ssd1306Device;
    
    if( !i2c_master_driver_initialize() )
    {   
        while (1)
        {
            if (flagCCS811 >= 0 && flagBME680  >= 0 &&  flagSSD1306 >= 0)
            {
                flagCCS811 = ccs811_read_print(&ssd1306Device, &ccs811Device, &ccs811Result, buffer_name);

                flagBME680 = bme680_read_print(&ssd1306Device, &bme680Device, &bme680Data, buffer_name);
            
                vTaskDelay(500 / portTICK_PERIOD_MS);

                if ( (((clear_end - clear_start)/1000)/1000) > CLEAR_DISPLAY_TIMER_SEC ) 
                {
                    ssd1306_clear_screen(&ssd1306Device);
                    clear_start = esp_timer_get_time();
                }

                if ( (((set_end - set_start)/1000)/1000) > SET_TEMPERATUR_HUMIDITY_SEC ) 
                {
                    flagCCS811 = ccs811_set_environmental_data(&ccs811Device, bme680Data.temperature / 100.0f, bme680Data.humidity / 1000.0f);
                    set_start = esp_timer_get_time();
                }

                set_end = esp_timer_get_time();
                clear_end = set_end;
            }
            else if (flagSSD1306 < 0)
            {
                flagSSD1306 = ssd1306_make_ready_for_use(&ssd1306Device);
            }
            else if (flagCCS811 < 0)
            {
                flagCCS811 = ccs811_make_ready_for_use(&ccs811Device);
            }
            else if (flagBME680 < 0)
            {
                flagBME680 = bme680_make_ready_for_use(&bme680Device);
            }
        }
    }
}








