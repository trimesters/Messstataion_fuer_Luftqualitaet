/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */

#ifndef I2C_MODULE_

    #define I2C_MODULE_
    
    #include "define.h"
    
    #include "driver/i2c.h"

    #define ALLOCATE_INTERRUPT          0 
    #define I2C_MASTER_TX_BUF_DISABLE   0
    #define I2C_MASTER_RX_BUF_DISABLE   0

    #define I2C_MASTER_NUM 0

    #define ACK_VAL         0x0 
    #define NACK_VAL        0x1
    #define ACK_CHECK_EN    0x1

    #define WRITE_BIT   I2C_MASTER_WRITE  
    #define READ_BIT    I2C_MASTER_READ   

    typedef struct i2c_args i2c_args_t;
    struct i2c_args
    {
        uint8_t i2c_addr;
        uint8_t reg_addr; 
        uint8_t *reg_data; 
        uint16_t length;
    };
    

    int8_t i2c_master_driver_initialize(void);
    int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);
    int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length);

#endif