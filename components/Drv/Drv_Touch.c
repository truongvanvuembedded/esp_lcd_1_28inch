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
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

// User lib
#include "Drv_Touch.h"
#include "Define.h"
// External lib

//==================================================================================================
//	Local define
//==================================================================================================

//==================================================================================================
//	Local define I/O
//==================================================================================================
// Touch pin
#define	U1_TP_RST_PIN				((U1)1)
#define	U1_TP_INT_PIN				((U1)0)
#define U1_I2C_MASTER_SDA_IO		((U1)4)
#define U1_I2C_MASTER_SCL_IO		((U1)5)
#define U4_I2C_MASTER_FREQ_HZ		((U4)400000)
#define U1_I2C_ADDR_CST816D			((U1)0x1)
// I2C Port
#define I2C_MASTER_PORT				I2C_NUM_0
// CST816D configuration
#define CST816D_ADDR 				((U1)0x15)
#define CST816D_REG_GESTURE			((U1)0x01)
#define CST816D_REG_FINGERNUM		((U1)0x02)
#define CST816D_REG_XPOS_H			((U1)0x03)
#define CST816D_REG_XPOS_L			((U1)0x04)
#define CST816D_REG_YPOS_H			((U1)0x05)
#define CST816D_REG_YPOS_L			((U1)0x06)
//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
static i2c_master_dev_handle_t cst816d_dev;		// cst816d touch device handle
static i2c_master_bus_handle_t i2cbus_handle;	// I2C master bus handle
static volatile U1 vu1_TouchFlag;	// Touch flag to indicate touch event
//==================================================================================================
//	Local ROM
//==================================================================================================
static const char *TAG = "CS816D";
//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static void cs816d_isr_handler(void* arg);
//==================================================================================================
//	Source Code
//==================================================================================================
static U1 u1_cst816d_i2c_read_continous(U1 u1_RegAdd, U1 *apu1_Data, U2 au2_Len);
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	DrvTouch_Init
//	Function:	Init I2C for CS816D
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void DrvTouch_Init(void)
{
	ESP_LOGI(TAG, "Initialize CS816D");
	vu1_TouchFlag = U1OFF;
	// Initialize I2C bus
	i2c_master_bus_config_t i2cbus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_MASTER_PORT,
		.scl_io_num = U1_I2C_MASTER_SCL_IO,
		.sda_io_num = U1_I2C_MASTER_SDA_IO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2cbus_config, &i2cbus_handle));

	// Add device to I2C bus
	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = CST816D_ADDR,
		.scl_speed_hz = U4_I2C_MASTER_FREQ_HZ,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cbus_handle, &dev_cfg, &cst816d_dev));

	// Configure INT pin as input
	gpio_config_t io_conf = {
		.pin_bit_mask = (1ULL << U1_TP_INT_PIN),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_NEGEDGE,
	};
	gpio_config(&io_conf);
	// C?i ??t ISR handler cho ch?n INT
	gpio_install_isr_service(0); // ??ng k? d?ch v? ISR (g?i 1 l?n duy nh?t, c? th? ??t ngo?i Init n?u d?ng nhi?u ch?n)
	gpio_isr_handler_add(U1_TP_INT_PIN, cs816d_isr_handler, NULL); // ??ng k? handler

	// Configure RST pin as output and perform reset
	gpio_config_t rst_conf = {
		.pin_bit_mask = (1ULL << U1_TP_RST_PIN),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config(&rst_conf);

	// Reset sequence
	gpio_set_level(U1_TP_RST_PIN, 0);
	vTaskDelay(10/portTICK_PERIOD_MS);
	gpio_set_level(U1_TP_RST_PIN, 1);
	vTaskDelay(50/portTICK_PERIOD_MS);
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	u1_DrvReadTouch
//	Function:	This function reads touch data from the CS816D touch controller via I2C.
//				It checks if a touch event is detected, then reads gesture, finger count, and X/Y coordinates.
//	
//	Argument:	apst_TouchData Pointer to ST_TOUCH_DATA structure to store the touch data.
//	Return	:	U1OK if data was successfully read, U1NG otherwise.
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
////////////////////////////////////////////////////////////////////////////////////////////////////
U1 u1_DrvReadTouch(ST_TOUCH_DATA *apst_TouchData)
{
    U1 au1_Data[6];      // Buffer to store raw data read from the touch controller
    U1 au1_Ret;          // Variable to store the return status
    au1_Ret = U1NG;      // Initialize return status as 'not good' (failure)

    // Check if the touch interrupt pin is active (touch detected)
    if (vu1_TouchFlag == 1) 
	{
		vu1_TouchFlag = 0; // Reset the touch flag
		// Read the gesture register to check for touch events
        au1_Ret = u1_cst816d_i2c_read_continous(CST816D_REG_GESTURE, au1_Data, sizeof(au1_Data));
        
        // If the read was successful
        if (au1_Ret == U1OK) 
        {
            // Parse gesture and finger number from the data buffer
            apst_TouchData->u1_Gesture = au1_Data[0];
            apst_TouchData->u1_FingerNum = au1_Data[1];
            // Parse X coordinate (12 bits: high nibble from au1_Data[2], low byte from au1_Data[3])
            apst_TouchData->u2_X = ((au1_Data[2] & 0x0F) << 8) | au1_Data[3];
            // Parse Y coordinate (12 bits: high nibble from au1_Data[4], low byte from au1_Data[5])
            apst_TouchData->u2_Y = ((au1_Data[4] & 0x0F) << 8) | au1_Data[5];
            // Log the detected touch coordinates for debugging
            //ESP_LOGI(TAG, "Touch detected - X: %d, Y: %d", apst_TouchData->u2_X, apst_TouchData->u2_Y);
        }
	}
    // Return the status of the operation (U1OK if successful, U1NG otherwise)
    return au1_Ret;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	u1_cst816d_i2c_read_continous
//	Function:	Read from I2C device using master API
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static U1 u1_cst816d_i2c_read_continous(U1 u1_RegAdd, U1 *apu1_Data, U2 au2_Len)
{
	S1 as1_Timeout;
	esp_err_t ret;
	as1_Timeout = 10; // Timeout in ms
	// Write register address first then read data
	ret = i2c_master_transmit_receive(cst816d_dev, &u1_RegAdd, 1, apu1_Data, au2_Len, as1_Timeout);
	if (ret != ESP_OK) {
		return U1NG;
	}
	return U1OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Name	:	cs816d_isr_handler
//	Function:	Callback function for touch interrupt
//	
//	Argument:	-
//	Return	:	-
//	Create	:	09/05/2025
//	Change	:	-
//	Remarks	:	-
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void cs816d_isr_handler(void* arg)
{
    vu1_TouchFlag = 1;  // ??nh d?u ?? c? ng?t x?y ra
}
