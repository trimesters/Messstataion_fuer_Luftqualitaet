/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */


#include "i2c_module.h"


    gpio_num_t i2c_gpio_sda = 5;
    gpio_num_t i2c_gpio_scl = 6;

    uint32_t i2c_frequency = 200000;


/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_master_driver_initialize(void)
{
    uint32_t i2c_master_port = I2C_MASTER_NUM;

	i2c_config_t conf = 
	{
		.mode = 			I2C_MODE_MASTER,
		.sda_io_num = 		i2c_gpio_sda,         	// select GPIO specific to your project
		.sda_pullup_en = 	GPIO_PULLUP_ENABLE,
		.scl_io_num = 		i2c_gpio_scl,         	// select GPIO specific to your project
		.scl_pullup_en = 	GPIO_PULLUP_ENABLE,
		.master.clk_speed = i2c_frequency,  		// select frequency specific to your project
	};

    uint8_t err = i2c_param_config(i2c_master_port, &conf);
	if (err != ESP_OK) 
    {
        return err;
    }

	return i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, ALLOCATE_INTERRUPT);
}


/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint32_t i2c_master_port = I2C_MASTER_NUM;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    for (uint8_t i = 0; i < length; i++)
    {
        i2c_master_write_byte(cmd, reg_data[i], ACK_CHECK_EN);
    }
    
    i2c_master_stop(cmd);

    int8_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    return ret;
}


/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
    uint32_t i2c_master_port = I2C_MASTER_NUM;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | READ_BIT, ACK_CHECK_EN);

    if (length > 1) 
    {
        i2c_master_read(cmd, &(reg_data[0]), length - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, &(reg_data[0]) + length - 1, NACK_VAL);
    
    i2c_master_stop(cmd);

    int8_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, 1000 / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    return ret;
}

