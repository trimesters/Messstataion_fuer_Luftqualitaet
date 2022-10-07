/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */

#ifndef CCS811_D_

    #define CCS811_D_

    #include "define.h"

    //I2C ADDRESS/BITS
    #define CCS811_I2C_ADDRESS          UINT8_C(0x5A)
    #define CCS811_I2C_DEVICE_ID        UINT8_C(0x81)
    #define CCS811_I2C_DEVICE_VERSION   UINT8_C(0x10)

    #define CCS811_REF_RESISTOR         100000
    #define MAXLENGTH                   16    
    
    /*! @name Return codes */
        /*! @name Success code*/
        #define CCS811_OK                            INT8_C(0)

        /*! @name Error codes */
        #define CCS811_E_NULL_PTR                    INT8_C(-1)
        #define CCS811_E_DEV_NOT_FOUND               INT8_C(-2)
        #define CCS811_E_INVALID_LEN                 INT8_C(-3)
        #define CCS811_E_COMM_FAIL                   INT8_C(-4)
        #define CCS811_E_INVALID_MODE                INT8_C(-5)
        #define CCS811_E_BOND_WIRE                   INT8_C(-6)
        #define CCS811_E_IMPLAUS_TEMP                INT8_C(-7)
        #define CCS811_E_IMPLAUS_PRESS               INT8_C(-8)
        #define CCS811_E_CAL_PARAM_RANGE             INT8_C(-9)
        #define CCS811_E_UNCOMP_TEMP_RANGE           INT8_C(-10)
        #define CCS811_E_UNCOMP_PRES_RANGE           INT8_C(-11)
        #define CCS811_E_UNCOMP_TEMP_AND_PRESS_RANGE INT8_C(-12)
        #define CCS811_E_UNCOMP_DATA_CALC            INT8_C(-13)
        #define CCS811_E_32BIT_COMP_TEMP             INT8_C(-14)
        #define CCS811_E_32BIT_COMP_PRESS            INT8_C(-15)
        #define CCS811_E_64BIT_COMP_PRESS            INT8_C(-16)
        #define CCS811_E_DOUBLE_COMP_TEMP            INT8_C(-17)
        #define CCS811_E_DOUBLE_COMP_PRESS           INT8_C(-18)

    #define CCS811_E_NO_NEW_DATA        INT8_C(2)
    #define CCS811_POLL_PERIOD_MS       INT8_C(100)

    /*! @name Function pointer type definitions */
    typedef int8_t  (*ccs811_i2c_fptr_t)    (uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
    typedef void    (*ccs811_delay_fptr_t)  (uint32_t period);

    //REGISTERS
    enum {
        CCS811_STATUS           = 0x00,
        CCS811_MEAS_MODE        = 0x01,
        CCS811_ALG_RESULT_DATA  = 0x02,
        CCS811_RAW_DATA         = 0x03,
        CCS811_ENV_DATA         = 0x05,
        CCS811_NTC              = 0x06,
        CCS811_THRESHOLDS       = 0x10,
        CCS811_BASELINE         = 0x11,
        CCS811_HW_ID            = 0x20,
        CCS811_HW_VERSION       = 0x21,
        CCS811_FW_BOOT_VERSION  = 0x23,
        CCS811_FW_APP_VERSION   = 0x24,
        CCS811_ERROR_ID         = 0xE0,
        CCS811_SW_RESET         = 0xFF,
    };

    // bootloader registers
    enum {
        CCS811_BOOTLOADER_APP_ERASE     = 0xF1,
        CCS811_BOOTLOADER_APP_DATA      = 0xF2,
        CCS811_BOOTLOADER_APP_VERIFY    = 0xF3,
        CCS811_BOOTLOADER_APP_START     = 0xF4
    };

    enum {
        CCS811_DRIVE_MODE_IDLE      = 0x00,
        CCS811_DRIVE_MODE_1SEC      = 0x01,
        CCS811_DRIVE_MODE_10SEC     = 0x02,
        CCS811_DRIVE_MODE_60SEC     = 0x03,
        CCS811_DRIVE_MODE_250MS     = 0x04,
    };

    // The status register
    typedef struct status status_t;
    struct status 
    {
        uint8_t error;     // 0: no error,                                 1: error has occurred
        uint8_t dataReady; // 0: no samples are ready,                     1: samples are ready
        uint8_t appValid;  // 0: Valid application firmware loaded,        1: Valid application firmware loaded
        uint8_t fwMode;    // 0: boot mode, new firmware can be loaded,    1: application mode, can take measurements
    };

    // measurement and conditions register
    typedef struct meas_mode measMode_t;
    struct meas_mode 
    {
        uint8_t intThresh;     // 0: interrupt mode operates normally,     1: Interrupt mode (if enabled) only asserts the nINT signal (driven low) if the new ALG_RESULT_DATA crosses one of the thresholds set in the THRESHOLDS register by more than the hysteresis value (also in the THRESHOLDS register)
        uint8_t intDatardy;    // 0: int disabled,                         1: The nINT signal is asserted (driven low) when a new sample is ready in ALG_RESULT_DATA. The nINT signal will stop being driven low when ALG_RESULT_DATA is read on the I²C interface.
        uint8_t driveMode;     // 0:                                       1: 
    };

    // measurement and conditions register
    typedef struct error_id errorID_t;
    struct error_id 
    {
        uint8_t writeRegInvalid;  // The CCS811 received an I²C write request addressed to this station but with invalid register address ID
        uint8_t readRegInvalid;   // The CCS811 received an I²C read request to a mailbox ID that is invalid
        uint8_t measmodeInvalid;  // The CCS811 received an I²C request to write an unsupported mode to MEAS_MODE
        uint8_t maxResistance;    // The sensor resistance measurement has reached or exceeded the maximum range
        uint8_t heaterFault;      // The Heater current in the CCS811 is not in range
        uint8_t heaterSupply;     // The Heater voltage is not being applied correctly
    };

    /*! @name API device structure */
    typedef struct ccs811_device ccs811_device_t;
    struct ccs811_device
    {
        uint8_t                 dev_addr;
        uint8_t                 chip_id;
        uint8_t                 chip_ver;

        uint16_t                co2;
        uint16_t                tvoc;

        ccs811_i2c_fptr_t       read_v;
        ccs811_i2c_fptr_t       write_v;
        ccs811_delay_fptr_t     delay_ms;
        
        status_t                status;
        measMode_t              mode;
        errorID_t               error;
    };

    // measurement and conditions register
    typedef struct result ccs811_result_t;
    struct result 
    {
        uint16_t eco2;  
        uint16_t tvoc;   
        uint16_t currentSelected; 
        uint16_t rawADCreading;    
    };

#endif