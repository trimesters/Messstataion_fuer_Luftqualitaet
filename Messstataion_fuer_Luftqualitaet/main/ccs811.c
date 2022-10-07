/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */


#include "ccs811.h"

/**************************************************************************/
/*!
    @brief  -
    @param  -
    @returns -
*/
/**************************************************************************/
void ccs811_delay_ms(uint32_t period_ms);

/**************************************************************************/
/*!
    @brief  Setups the I2C interface and hardware and checks for communication.
    @param  addr Optional I2C address the sensor can be found on. Default is 0x5A
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
int8_t ccs811_init(ccs811_device_t* dev_);

/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_sw_reset(ccs811_device_t* dev_);


/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_hw_check(ccs811_device_t* dev_);


/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_status_available(ccs811_device_t* dev_);

/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
void ccs811_status_write(ccs811_device_t* dev_, int8_t status_data);


/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_start_app(ccs811_device_t* dev_);


/**************************************************************************/
/*!
    @brief  -
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_check_for_error(ccs811_device_t* dev_);

/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
void ccs811_error_write(ccs811_device_t* dev_, int8_t error_data);



/**************************************************************************/
/*!
    @brief  reset sequence from the datasheet
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_set_mode(ccs811_device_t* dev_);


/**************************************************************************/
/*!
    @brief This API read the data from the given register address of the sensor.
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, ccs811_device_t *dev_);

/**************************************************************************/
/*!
    @brief This API writes the given data to the register addresses of the sensor.
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_set_regs(uint8_t *reg_addr, uint8_t *reg_data, uint8_t len, ccs811_device_t *dev_);

/**************************************************************************/
/*!
    @brief This internal API is used to check for null-pointers in the device structure.
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t ccs811_null_ptr_check(ccs811_device_t *dev_);


/**************************************************************************/
/*!
    @brief This internal API interleaves the register addresses and respective register data for a burst write
    @param  -
    @returns -
*/
/**************************************************************************/
void ccs811_interleave_data(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);

/**************************************************************************/
/*!
    @brief -
    @param  -
    @returns -
*/
/**************************************************************************/
int8_t int16_t_to_string(int16_t vaule_, char* name_, char* buffer_name);


int8_t ccs811_make_ready_for_use(ccs811_device_t* ccs811Device_)
{
    int8_t rslt;

    ccs811Device_->dev_addr             = CCS811_I2C_ADDRESS;
    ccs811Device_->chip_id              = CCS811_I2C_DEVICE_ID;

    ccs811Device_->read_v               = i2c_reg_read;
    ccs811Device_->write_v              = i2c_reg_write;
    ccs811Device_->delay_ms             = ccs811_delay_ms;

    ccs811Device_->mode.intThresh       = 0;
    ccs811Device_->mode.intDatardy      = 0;
    ccs811Device_->mode.driveMode       = CCS811_DRIVE_MODE_1SEC;

    rslt = ccs811_init(ccs811Device_);

    return  rslt;
}

void ccs811_delay_ms(uint32_t period_ms)
{
    uint64_t start = esp_timer_get_time();
    uint64_t end = esp_timer_get_time();

    while (( (end - start)/1000 ) < period_ms) 
    {
        end = esp_timer_get_time();
    }
}

int8_t ccs811_init(ccs811_device_t* dev_) 
{
    int8_t rslt;

    rslt = ccs811_sw_reset(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }

    rslt = ccs811_hw_check(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }

    rslt = ccs811_status_available(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }
    
    rslt = ccs811_start_app(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }
    
    rslt = ccs811_check_for_error(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }
    
    rslt = ccs811_set_mode(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }
    rslt = ccs811_status_available(dev_);
    if (rslt != CCS811_OK)
    {
        return rslt;
    }
    
    return rslt;
}

int8_t ccs811_sw_reset(ccs811_device_t* dev_)
{
    int8_t rslt;

    uint8_t reg_addr = CCS811_SW_RESET;
    uint8_t soft_rst_cmd[4] = {0x11, 0xE5, 0x72, 0x8A};

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        rslt = ccs811_set_regs(&reg_addr, &(soft_rst_cmd[0]), 4, dev_);

        dev_->delay_ms(500);
    }

    return rslt;
}

int8_t ccs811_hw_check(ccs811_device_t* dev_)
{
    int8_t rslt;

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        rslt = ccs811_get_regs(CCS811_HW_ID, &(dev_->chip_id), 1, dev_);

        if ((rslt != CCS811_OK) && (dev_->chip_id != CCS811_I2C_DEVICE_ID))
        {
            return CCS811_E_DEV_NOT_FOUND;
        }
        
        rslt = ccs811_get_regs(CCS811_HW_VERSION, &(dev_->chip_ver), 1, dev_);

        if ((rslt != CCS811_OK) && (dev_->chip_ver != CCS811_I2C_DEVICE_VERSION))
        {
            return CCS811_E_DEV_NOT_FOUND;
        }
    }

    return rslt;
}

int8_t ccs811_status_available(ccs811_device_t* dev_)
{
    int8_t rslt;
    uint8_t status_data;

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        rslt = ccs811_get_regs(CCS811_STATUS, &status_data, 1, dev_);

        if (rslt == CCS811_OK) 
        {
            ccs811_status_write(dev_, status_data);
        }
    }

    return rslt;
}

void ccs811_status_write(ccs811_device_t* dev_, int8_t status_data)
{
    dev_->status.error      = status_data & 0x01;
    dev_->status.dataReady  = (status_data >> 3) & 0x01;
    dev_->status.appValid   = (status_data >> 4) & 0x01;
    dev_->status.fwMode     = (status_data >> 7) & 0x01;

    /*
    print...
    */
}

int8_t ccs811_start_app(ccs811_device_t* dev_)
{
    int8_t rslt;

    uint8_t reg_addr = CCS811_BOOTLOADER_APP_START;

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        rslt = dev_->write_v(dev_->dev_addr, reg_addr, NULL, 0);

        dev_->delay_ms(500);
    }

    return rslt;
}

int8_t ccs811_check_for_error(ccs811_device_t* dev_) 
{
    int8_t rslt;
    uint8_t error_data;

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        rslt = ccs811_get_regs(CCS811_ERROR_ID, &error_data, 1, dev_);

        if (rslt == CCS811_OK) 
        {
            ccs811_error_write(dev_, error_data);
        }
    }

    return rslt;
}

void ccs811_error_write(ccs811_device_t* dev_, int8_t error_data)
{
    dev_->error.writeRegInvalid     = error_data & 0x01;
    dev_->error.readRegInvalid      = (error_data & 0x02) >> 1;
    dev_->error.measmodeInvalid     = (error_data & 0x04) >> 2;
    dev_->error.maxResistance       = (error_data & 0x08) >> 3;
    dev_->error.heaterFault         = (error_data & 0x10) >> 4;
    dev_->error.heaterSupply        = (error_data & 0x20) >> 5;

    /*
    print...
    */
}

int8_t ccs811_set_mode(ccs811_device_t* dev_) 
{
    int8_t rslt;

    uint8_t reg_addr = CCS811_MEAS_MODE;
    uint8_t mode_data;

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        mode_data = (dev_->mode.intThresh << 2) | (dev_->mode.intDatardy << 3) | (dev_->mode.driveMode << 4);

        rslt = ccs811_set_regs(&reg_addr, &mode_data, 1, dev_);

        dev_->delay_ms(500);
    }

    return rslt;
}

int8_t ccs811_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint8_t len, ccs811_device_t *dev_)
{
    int8_t rslt;

    rslt = ccs811_null_ptr_check(dev_);

    if ((rslt == CCS811_OK) && (reg_data != NULL))
    {
        rslt = dev_->read_v(dev_->dev_addr, reg_addr, reg_data, len);

        /* Check for communication error and mask with an internal error code */
        if (rslt != CCS811_OK)
        {
            rslt = CCS811_E_COMM_FAIL;
        }
    }
    else
    {
        rslt = CCS811_E_NULL_PTR;
    }

    return rslt;
}

int8_t ccs811_set_regs(uint8_t *reg_addr, uint8_t *reg_data, uint8_t len, ccs811_device_t *dev_)
{
    int8_t rslt;
    uint8_t temp_buff[8]; /* Typically not to write more than 4 registers */
    uint16_t temp_len;

    if (len > 4)
    {
        len = 4;
    }
    rslt = ccs811_null_ptr_check(dev_);

    if ((rslt == CCS811_OK) && (reg_addr != NULL) && (reg_data != NULL))
    {
        if (len != 0)
        {
            temp_buff[0] = reg_data[0];

            /* Burst write mode */
            if (len > 1)
            {
                /* Interleave register address w.r.t data for burst write*/
                ccs811_interleave_data(reg_addr, temp_buff, reg_data, len);
                temp_len = ((len * 2) - 1);
            }
            else
            {
                temp_len = len;
            }
            
            rslt = dev_->write_v(dev_->dev_addr, reg_addr[0], temp_buff, temp_len);

            /* Check for communication error and mask with an internal error code */
            if (rslt != CCS811_OK)
            {
                rslt = CCS811_E_COMM_FAIL;
            }
        }
        else
        {
            rslt = CCS811_E_INVALID_LEN;
        }
    }
    else
    {
        rslt = CCS811_E_NULL_PTR;
    }

    return rslt;
}

int8_t ccs811_null_ptr_check(ccs811_device_t *dev_)
{
    int8_t rslt;

    if ((dev_ == NULL) || (dev_->read_v == NULL) || (dev_->write_v == NULL) || (dev_->delay_ms == NULL))
    {
        rslt = CCS811_E_NULL_PTR;
    }
    else
    {
        rslt = CCS811_OK;
    }

    return rslt;
}

void ccs811_interleave_data(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len)
{
    uint8_t index;

    for (index = 1; index < len; index++)
    {
        temp_buff[(index * 2) - 1]      = reg_addr[index];
        temp_buff[index * 2]            = reg_data[index];
    }
}

int8_t ccs811_read_data(ccs811_device_t *dev_, ccs811_result_t* ccs811Result_) 
{
    int8_t rslt;
    uint8_t tries = 10;
    uint8_t co2TvocBuffer[8];

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        do 
        {
            rslt = ccs811_get_regs(CCS811_ALG_RESULT_DATA, &(co2TvocBuffer[0]), 8, dev_);

            if (rslt == CCS811_OK) 
            {
                ccs811Result_->eco2 = ((uint16_t)co2TvocBuffer[0] << 8) | ((uint16_t)co2TvocBuffer[1]);
                ccs811Result_->tvoc = ((uint16_t)co2TvocBuffer[2] << 8) | ((uint16_t)co2TvocBuffer[3]);
                ccs811_status_write(dev_, co2TvocBuffer[4]);
                ccs811_error_write(dev_, co2TvocBuffer[5]);
                ccs811Result_->currentSelected = ((uint16_t)co2TvocBuffer[6] >> 2);
                ccs811Result_->rawADCreading = ((uint16_t)(co2TvocBuffer[6] & 3) << 8) | ((uint16_t)co2TvocBuffer[7]);

                return rslt;
            }

            dev_->delay_ms(CCS811_POLL_PERIOD_MS);

            tries--;

        }while (tries); 
    }
    return rslt;
}

int8_t ccs811_set_environmental_data(ccs811_device_t* dev_, float humidity, float temperature)
{
    int8_t rslt;

    rslt = ccs811_null_ptr_check(dev_);

    if (rslt == CCS811_OK)
    {
        uint8_t reg_addr = CCS811_ENV_DATA;

        uint16_t hum_conv = humidity * 512.0f + 0.5f;
        uint16_t temp_conv = (temperature + 25.0f) * 512.0f + 0.5f;

        uint8_t mode_data[] = {
            (uint8_t)((hum_conv >> 8) & 0xFF), (uint8_t)(hum_conv & 0xFF),
            (uint8_t)((temp_conv >> 8) & 0xFF), (uint8_t)(temp_conv & 0xFF)
            };

        rslt = ccs811_set_regs(&reg_addr, mode_data, 4, dev_);

        dev_->delay_ms(500);
    }

    return rslt;
}

int8_t ccs811_read_print(ssd1306_device_t* ssd1306Device_, ccs811_device_t* ccs811Device_, ccs811_result_t* ccs811Result_, char* buffer_name)
{
    int8_t rslt;
    int8_t count;

    rslt = ccs811_read_data(ccs811Device_, ccs811Result_);

    if (rslt >= 0)
    {
        count = int16_t_to_string(ccs811Result_->eco2, "eCO2: ", buffer_name);
        rslt = ssd1306_display_text(ssd1306Device_, 0, buffer_name, count);

        count = int16_t_to_string(ccs811Result_->tvoc, "TVOC: ", buffer_name);
        rslt = ssd1306_display_text(ssd1306Device_, 1, buffer_name, count);

        return rslt;
    }

    return rslt;
}

int8_t int16_t_to_string(int16_t vaule_, char* name_, char* buffer_name)
{
    char buffer_value[6]    = {0};
    buffer_name[0]          = '\0';

    snprintf(buffer_value, 10, "%d", vaule_);

    strcat(buffer_name, name_);
    strcat(buffer_name, buffer_value);

    for (int i = strlen(buffer_name); i < MAXLENGTH; i++)
    {
        buffer_name[i] = ' ';
    }

    return MAXLENGTH;
}

