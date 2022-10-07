/**
 * @file main.c
 * @author Florian Kalb (florian.kalb@outlook.at)
 * @version 0.4
 * @date 2022-02-02
 * @copyright Copyright (c) 2022
 */


#include "ssd1306.h"

#include "font8x8_basic.h"

int8_t ssd1306_i2c_init(ssd1306_device_t* dev);
int8_t i2c_contrast(ssd1306_device_t* dev, int contrast);



int8_t ssd1306_make_ready_for_use(ssd1306_device_t* ssd1306Device_)
{
	int8_t rslt;

	ssd1306Device_->address 		= OLED_I2C_ADDRESS;
	ssd1306Device_->flip 			= OLED_FALSE;
	ssd1306Device_->width 			= OLED_SIZE_WIDTH;
	ssd1306Device_->height 			= OLED_SIZE_HIEGHT;
	ssd1306Device_->pages 			= OLED_SIZE_PAGE;							//(dev->_height == 32) ? 4 : 8;

	rslt = ssd1306_i2c_init(ssd1306Device_);

	rslt = i2c_contrast(ssd1306Device_, 0xff);

	ssd1306_clear_screen(ssd1306Device_);

	return rslt;
}

int8_t ssd1306_i2c_init(ssd1306_device_t* dev) 
{
	int8_t rslt;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);

	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);				// AE
	i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);			// A8
	i2c_master_write_byte(cmd, 0x3F, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_OFFSET, true);		// D3
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);	// 40

	if (dev->flip) 
	{
		i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_0, true);		// A0
	} 
	else 
	{
		i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP_1, true);		// A1
	}

	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true);		// C8
	i2c_master_write_byte(cmd, OLED_CMD_SET_DISPLAY_CLK_DIV, true);		// D5
	i2c_master_write_byte(cmd, 0x80, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_PIN_MAP, true);			// DA
	i2c_master_write_byte(cmd, 0x12, true);
	
	i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);			// 81
	i2c_master_write_byte(cmd, 0xFF, true);
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_RAM, true);				// A4
	i2c_master_write_byte(cmd, OLED_CMD_SET_VCOMH_DESELCT, true);		// DB
	i2c_master_write_byte(cmd, 0x40, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);	// 20
	i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDR_MODE, true);		// 02
	
	// Set Lower Column Start Address for Page Addressing Mode
	i2c_master_write_byte(cmd, 0x00, true);

	// Set Higher Column Start Address for Page Addressing Mode
	i2c_master_write_byte(cmd, 0x10, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);			// 8D
	i2c_master_write_byte(cmd, 0x14, true);
	i2c_master_write_byte(cmd, OLED_CMD_DEACTIVE_SCROLL, true);			// 2E
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);			// A6
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);				// AF

	i2c_master_stop(cmd);

	rslt = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);

	i2c_cmd_link_delete(cmd);

	return rslt;
}


int8_t i2c_display_image(ssd1306_device_t* dev, int8_t page, int8_t seg, uint8_t* images, uint8_t width) 
{
	int8_t rslt;
	i2c_cmd_handle_t cmd;
	
	if (page >= dev->pages) return -1;
	if (seg >= dev->width) return -1;

	uint8_t _seg = seg + CONFIG_OFFSETX;
	uint8_t columLow = _seg & 0x0F;
	uint8_t columHigh = (_seg >> 4) & 0x0F;

	int _page = page;

	if (dev->flip) 
	{
		_page = (dev->pages - page) - 1;
	}

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	// Set Lower Column Start Address for Page Addressing Mode
	i2c_master_write_byte(cmd, (0x00 + columLow), true);
	// Set Higher Column Start Address for Page Addressing Mode
	i2c_master_write_byte(cmd, (0x10 + columHigh), true);
	// Set Page Start Address for Page Addressing Mode
	i2c_master_write_byte(cmd, 0xB0 | _page, true);

	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
	i2c_master_write(cmd, images, width, true);

	i2c_master_stop(cmd);
	rslt = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return rslt;
}

int8_t i2c_contrast(ssd1306_device_t* dev, int contrast) 
{
	int8_t rslt;
	i2c_cmd_handle_t cmd;

	int _contrast = contrast;
	if (contrast < 0x0) _contrast = 0;
	if (contrast > 0xFF) _contrast = 0xFF;

	cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev->address << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_CONTRAST, true);			// 81
	i2c_master_write_byte(cmd, _contrast, true);
	i2c_master_stop(cmd);
	rslt = i2c_master_cmd_begin(I2C_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	return rslt; 
}

int8_t ssd1306_display_text(ssd1306_device_t * dev, int page, char * text, int text_len)
{
	int8_t rslt = 0;

	if (page >= dev->pages) return -1 ;
	int _text_len = text_len;
	if (_text_len > 16) _text_len = 16;

	uint8_t seg = 0;
	uint8_t image[8];
	
	for (uint8_t i = 0; i < _text_len; i++) 
	{
		memcpy(image, font8x8_basic_tr[(uint8_t)text[i]], 8);

		rslt = i2c_display_image(dev, page, seg, image, 8);
		
		seg = seg + 8;
	}
	return rslt;
}

void ssd1306_clear_screen(ssd1306_device_t * dev)
{
    char space[16];

    memset(space, 0x20, sizeof(space));

    for (int page = 0; page < dev->pages; page++) 
	{
        ssd1306_display_text(dev, page, space, sizeof(space));
    }
}

