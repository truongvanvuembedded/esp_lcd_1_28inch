//==================================================================================================
//
//	File Name		: Drv_Display.c
//	CPU Type		: ESP32-C3
//	Project Name	: ESP32_C3_LCD_128Inch
//
//	Description		: Display application handle task such as init LCD and LVGL
//
//	History			: Ver.0.01		2025.09.09 V.Vu	 New
//
//==================================================================================================
//==================================================================================================
//	Compile Option
//==================================================================================================

//==================================================================================================
//	#pragma section
//==================================================================================================

//==================================================================================================
//	Local Compile Option
//==================================================================================================

//==================================================================================================
//	Header File
//==================================================================================================
// Standard lib

// Esp-Idf lib
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_gc9a01.h"

// User lib
#include "Drv_Display.h"

// External lib
#include "lvgl.h"
#include "demos/lv_demos.h"
//==================================================================================================
//	Local define
//==================================================================================================
/* LCD size */
#define LCD_H_RES   (240)
#define LCD_V_RES   (240)
//==================================================================================================
//	Local define I/O
//==================================================================================================
/* LCD settings */
#define LCD_SPI_NUM             (SPI2_HOST)
#define LCD_PIXEL_CLK_HZ        (40 * 1000 * 1000)
#define LCD_CMD_BITS            (8)
#define LCD_PARAM_BITS          (8)
#define LCD_COLOR_SPACE         (ESP_LCD_COLOR_SPACE_BGR)
#define LCD_BITS_PER_PIXEL      (16)
#define LCD_DRAW_BUFF_DOUBLE    (1)
#define LCD_DRAW_BUFF_HEIGHT    (50)
#define LCD_BL_ON_LEVEL         (1)
/* LCD pins */
#define LCD_GPIO_SCLK           (GPIO_NUM_6)
#define LCD_GPIO_MOSI           (GPIO_NUM_7)
#define LCD_GPIO_RST            (-1)
#define LCD_GPIO_DC             (GPIO_NUM_2)
#define LCD_GPIO_CS             (GPIO_NUM_10)
#define LCD_GPIO_BL             (GPIO_NUM_3)
//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;
/* LVGL display */
static lv_display_t *lvgl_disp = NULL;
//==================================================================================================
//	Local ROM
//==================================================================================================
static const char *TAG = "Display.c";
//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static esp_err_t drvLcd_Init(void);
static esp_err_t lvglPort_Init(void);
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	drvLcd_Init
//	Function:	Init Lcd hardware (SPI & IO handle) 
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t drvLcd_Init(void)
{
    esp_err_t ret = ESP_OK;

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_GPIO_BL
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    /* LCD initialization */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = GC9A01_PANEL_BUS_SPI_CONFIG(LCD_GPIO_SCLK, LCD_GPIO_MOSI, LCD_H_RES * LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t));
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = GC9A01_PANEL_IO_SPI_CONFIG(LCD_GPIO_CS, LCD_GPIO_DC, NULL, NULL);
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_GPIO_RST,
        .color_space = LCD_COLOR_SPACE,
        .bits_per_pixel = LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_gc9a01(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    // esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(LCD_GPIO_BL, LCD_BL_ON_LEVEL));

    return ret;

err:
    if (lcd_panel) {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io) {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(LCD_SPI_NUM);
    return ret;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	lvglPort_Init
//	Function:	Porting LVGL to Esp32
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t lvglPort_Init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 8192,         /* LVGL task stack size */
        .task_affinity = -1,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = LCD_DRAW_BUFF_DOUBLE,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	DrvDisplay_Init
//	Function:	Update system time data based on received information
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DrvDisplay_Init(void)
{
    /* LCD HW initialization */
    ESP_ERROR_CHECK(drvLcd_Init());
    /* LVGL initialization */
    ESP_ERROR_CHECK(lvglPort_Init());
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	DrvDisplay_GetDisplay
//	Function:	Get lvgl display
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	
//
////////////////////////////////////////////////////////////////////////////////////////////////////
lv_disp_t* DrvDisplay_GetDisplay( void )
{
    return lvgl_disp;
}
