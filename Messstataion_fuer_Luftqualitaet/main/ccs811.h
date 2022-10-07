/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */

#ifndef CCS811_

    #define CCS811_

    #include "ccs811_d.h"

    #include "i2c_module.h"
    #include "ssd1306.h"
    

    /**************************************************************************/
    /*!
        @brief  -
        @param  -
        @returns -
    */
    /**************************************************************************/
    int8_t ccs811_make_ready_for_use(ccs811_device_t* ccs811Device_);

    /**************************************************************************/
    /*!
        @brief This internal API interleaves the register addresses and respective register data for a burst write
        @param  -
        @returns -
    */
    /**************************************************************************/
    int8_t ccs811_read_data(ccs811_device_t *dev_, ccs811_result_t* ccs811Result_);

    /**************************************************************************/
    /*!
        @brief -
        @param  -
        @returns -
    */
    /**************************************************************************/
    int8_t ccs811_set_environmental_data(ccs811_device_t* dev_, float humidity, float temperature);

    /**************************************************************************/
    /*!
        @brief -
        @param  -
        @returns -
    */
    /**************************************************************************/
    int8_t ccs811_read_print(ssd1306_device_t* ssd1306Device_, ccs811_device_t* ccs811Device_, ccs811_result_t* ccs811Result_, char* buffer_name);

#endif //CCS811_