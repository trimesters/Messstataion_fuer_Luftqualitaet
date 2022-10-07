/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */

// https://github.com/nopnop2002/esp-idf-ssd1306

#ifndef SSH1106_

    #define SSH1106_

    #include "define.h"
    #include "i2c_module.h"
    #include "ssd1306_d.h"

    

    /**************************************************************************/
    /*!
    @brief  -
    @param  -
    @returns -
    */
    /**************************************************************************/
    int8_t ssd1306_make_ready_for_use(ssd1306_device_t* ssd1306Device_);

    /**************************************************************************/
    /*!
    @brief  -
    @param  -
    @returns -
    */
    /**************************************************************************/
    int8_t ssd1306_display_text(ssd1306_device_t * dev, int page, char * text, int text_len);

    /**************************************************************************/
    /*!
    @brief  -
    @param  -
    @returns -
    */
    /**************************************************************************/
    void ssd1306_clear_screen(ssd1306_device_t * dev);

#endif //SSH1106_