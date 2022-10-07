/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */

#ifndef SSH1106_D_

    #define SSH1106_D_

    #include "define.h"

    #define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
    #define OLED_CONTROL_BYTE_CMD_STREAM    0x00
    #define OLED_CONTROL_BYTE_DATA_SINGLE   0xC0
    #define OLED_CONTROL_BYTE_DATA_STREAM   0x40

    // Fundamental commands (pg.28)
    #define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
    #define OLED_CMD_DISPLAY_RAM            0xA4
    #define OLED_CMD_DISPLAY_ALLON          0xA5
    #define OLED_CMD_DISPLAY_NORMAL         0xA6
    #define OLED_CMD_DISPLAY_INVERTED       0xA7
    #define OLED_CMD_DISPLAY_OFF            0xAE
    #define OLED_CMD_DISPLAY_ON             0xAF

    // Addressing Command Table (pg.30)
    #define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20
    #define OLED_CMD_SET_HORI_ADDR_MODE     0x00    // Horizontal Addressing Mode
    #define OLED_CMD_SET_VERT_ADDR_MODE     0x01    // Vertical Addressing Mode
    #define OLED_CMD_SET_PAGE_ADDR_MODE     0x02    // Page Addressing Mode
    #define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
    #define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

    // Hardware Config (pg.31)
    #define OLED_CMD_SET_DISPLAY_START_LINE 0x40
    #define OLED_CMD_SET_SEGMENT_REMAP_0    0xA0    
    #define OLED_CMD_SET_SEGMENT_REMAP_1    0xA1    
    #define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
    #define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
    #define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
    #define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
    #define OLED_CMD_NOP                    0xE3    // NOP

    // Timing and Driving Scheme (pg.32)
    #define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
    #define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
    #define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

    // Charge Pump (pg.62)
    #define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14

    // Scrolling Command
    #define OLED_CMD_HORIZONTAL_RIGHT       0x26
    #define OLED_CMD_HORIZONTAL_LEFT        0x27
    #define OLED_CMD_CONTINUOUS_SCROLL      0x29
    #define OLED_CMD_DEACTIVE_SCROLL        0x2E
    #define OLED_CMD_ACTIVE_SCROLL          0x2F
    #define OLED_CMD_VERTICAL		        0xA3

    #define OLED_I2C_ADDRESS                UINT8_C(0x3C)

    #define OLED_TRUE                       UINT8_C(0x01)
    #define OLED_FALSE                      UINT8_C(0x00)

    #define OLED_SIZE_WIDTH                 128
    #define OLED_SIZE_HIEGHT                64
    #define OLED_SIZE_PAGE                  8

    #define CONFIG_OFFSETX                  0
    
    #define I2C_NUM                         I2C_NUM_0


    typedef enum ssd1306_scroll_type ssd1306_scroll_type_t;
    enum ssd1306_scroll_type
    {
        SCROLL_RIGHT    = 1,
        SCROLL_LEFT     = 2,
        SCROLL_DOWN     = 3,
        SCROLL_UP       = 4,
        SCROLL_STOP     = 5
    };

    typedef struct page page_t;
    struct page  
    {
        int8_t valid;
        int8_t segLen; 
        uint8_t segs[128];
    };


    typedef struct ssd1306 ssd1306_device_t;
    struct ssd1306 
    {
        uint8_t address;
        int16_t width;
        int16_t height;
        int16_t pages;


        int16_t scStart;
        int16_t scEnd;
        int16_t	scDirection;

        int8_t flip;
        int8_t scEnable;

        page_t page[8];
    };

#endif //SSH1106_D_