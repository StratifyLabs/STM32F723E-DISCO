
#include <device/sys.h>
#include <mcu/debug.h>
#include <sapi/sg.h>
#include <cortexm/mpu.h>
#include <mcu/arch/stm32/stm32f7xx/stm32f7xx_hal.h>
#include "display_device.h"
#include "stm32f723e_discovery_lcd.h"

static int m_is_initialized MCU_SYS_MEM;

#define DISPLAY_PALETTE_BITS_PER_PIXEL 4
#define DISPLAY_PALETTE_COLOR_COUNT (1<<DISPLAY_PALETTE_BITS_PER_PIXEL)
static display_palette_t m_display_palette MCU_SYS_MEM;
static u16 m_display_colors[DISPLAY_PALETTE_COLOR_COUNT];

static int display_device_getinfo(const devfs_handle_t * handle, void * ctl);
static int display_device_clear(const devfs_handle_t * handle, void * ctl);
static int display_device_refresh(const devfs_handle_t * handle, void * ctl);
static int display_device_enable(const devfs_handle_t * handle, void * ctl);
static int display_device_disable(const devfs_handle_t * handle, void * ctl);
static int display_device_isbusy(const devfs_handle_t * handle, void * ctl);

int display_device_open(const devfs_handle_t * handle){
	return SYSFS_RETURN_SUCCESS;
}

int display_device_close(const devfs_handle_t * handle){
	return SYSFS_RETURN_SUCCESS;
}

int display_device_ioctl(const devfs_handle_t * handle, int request, void * ctl){

	switch(request){
		case I_DISPLAY_GETVERSION: return DISPLAY_VERSION;
		case I_DISPLAY_GETINFO: return display_device_getinfo(handle, ctl);
		case I_DISPLAY_CLEAR: return display_device_clear(handle, ctl);
		case I_DISPLAY_REFRESH: return display_device_refresh(handle, ctl);
		case I_DISPLAY_INIT: return display_device_init(handle, ctl);
		case I_DISPLAY_ENABLE: return display_device_enable(handle, ctl);
		case I_DISPLAY_DISABLE: return display_device_disable(handle, ctl);
		case I_DISPLAY_ISBUSY: return display_device_isbusy(handle, ctl);
		case I_DISPLAY_GETPALETTE:
			memcpy(ctl, &m_display_palette, sizeof(m_display_palette));
			return SYSFS_RETURN_SUCCESS;
	}

	return SYSFS_SET_RETURN(EINVAL);
}

int display_device_read(const devfs_handle_t * handle, devfs_async_t * async){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int display_device_write(const devfs_handle_t * handle, devfs_async_t * async){

	//write to the display -- convert from sgfx format to native format
	//sgfx is currently linking to 16bpp

	/* Set Cursor */
	ST7789H2_SetCursor(0, 0);

	if( async->nbyte != DISPLAY_WIDTH*DISPLAY_HEIGHT*2 ){
		//return SYSFS_SET_RETURN(EINVAL);
	}

	sg_cursor_t x_cursor;
	sg_cursor_t y_cursor;

	sg_cursor_set(&y_cursor, async->buf, sg_point(0,0));


	/* Prepare to write to LCD RAM */
	ST7789H2_WriteReg(ST7789H2_WRITE_RAM, (uint8_t*)NULL, 0);   /* RAM write data command */


	for(sg_size_t h=0; h < DISPLAY_HEIGHT; h++){
		sg_cursor_copy(&x_cursor, &y_cursor);
		for(sg_size_t w=0; w < DISPLAY_WIDTH; w++){
			sg_color_t color = sg_cursor_get_pixel_increment(&x_cursor, 1, 0);
			//get the LCD color from the palette
			LCD_IO_WriteData(m_display_colors[color & 0x0f]);
		}
		sg_cursor_inc_y(&y_cursor);
	}

	return async->nbyte;
}

int display_device_getinfo(const devfs_handle_t * handle, void * ctl){
	MCU_UNUSED_ARGUMENT(handle);
	display_info_t * info = ctl;
	info->width = DISPLAY_WIDTH;
	info->height = DISPLAY_HEIGHT;
	info->mem = 0; //system does not provide display memory
	info->scratch_mem = 0; //system does not provide scratch pad
	info->size = DISPLAY_MEMORY_SIZE;
	info->cols = DISPLAY_WIDTH;
	info->rows = DISPLAY_HEIGHT;
	info->freq = 30;
	info->bits_per_pixel = DISPLAY_PALETTE_BITS_PER_PIXEL;
	info->margin_left = 8;
	info->margin_right = 8;
	info->margin_top = 0;
	info->margin_bottom = 0;
	return SYSFS_RETURN_SUCCESS;
}

int display_device_clear(const devfs_handle_t * handle, void * ctl){
	MCU_UNUSED_ARGUMENT(handle);
	MCU_UNUSED_ARGUMENT(ctl);

	for(u32 i=0; i < ST7789H2_LCD_PIXEL_HEIGHT; i++){
		ST7789H2_DrawHLine(0, 0, i, ST7789H2_LCD_PIXEL_WIDTH);
	}

	return SYSFS_RETURN_SUCCESS;
}

int display_device_refresh(const devfs_handle_t * handle, void * ctl){

	//this is supposed to take the memory and make it effective on the display

	return SYSFS_RETURN_SUCCESS;
}

int display_device_init(const devfs_handle_t * handle, void * ctl){
	//initialize and clear the display

	if( m_is_initialized == 0 ){
		//FMC Bank 2 needs to be non-cacheable because it is used to write the display driver chip
#if 1
		mpu_enable_region(TASK_APPLICATION_DATA_USER_REGION_LOW_PRIORITY,
							 (void*)(0x60000000 + 0x04000000), //bank 1 sub bank 2
							 0x04000000,
							 MPU_ACCESS_PRW,
							 MPU_MEMORY_PERIPHERALS,
							 0);
		//sychronize data since mpu has been updated
		mcu_core_disable_cache();
		mcu_core_enable_cache();
#endif

		mcu_debug_printf("initialize LCD\n");
		BSP_LCD_Init();
		m_is_initialized = 1;
	}

	m_display_palette.pixel_format = DISPLAY_PALETTE_PIXEL_FORMAT_RGB565;
	m_display_palette.count = DISPLAY_PALETTE_COLOR_COUNT;
	m_display_palette.colors = m_display_colors;

	//set the default palette
	m_display_colors[0] = 0x0004;
	m_display_colors[1] = 0x0008;
	m_display_colors[2] = 0x0010;
	m_display_colors[3] = LCD_COLOR_BLUE; //0x001F

	m_display_colors[4] = 0x0040;
	m_display_colors[5] = 0x00E0;
	m_display_colors[6] = 0x03E0;
	m_display_colors[7] = LCD_COLOR_GREEN;

	m_display_colors[8] = 0x0400;
	m_display_colors[9] = 0x0800;
	m_display_colors[10] = 0x7800;
	m_display_colors[11] = LCD_COLOR_RED;

	m_display_colors[12] = 0x0000;
	m_display_colors[13] = 0x6666;
	m_display_colors[14] = 0xCCCC;
	m_display_colors[15] = LCD_COLOR_WHITE;


	for(u32 i=0; i < ST7789H2_LCD_PIXEL_HEIGHT; i++){
		ST7789H2_DrawHLine(LCD_COLOR_WHITE, 0, i, ST7789H2_LCD_PIXEL_WIDTH);
	}
	return 0;
}

int display_device_enable(const devfs_handle_t * handle, void * ctl){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int display_device_disable(const devfs_handle_t * handle, void * ctl){
	return SYSFS_SET_RETURN(ENOTSUP);
}

int display_device_isbusy(const devfs_handle_t * handle, void * ctl){
	return 0;
}


