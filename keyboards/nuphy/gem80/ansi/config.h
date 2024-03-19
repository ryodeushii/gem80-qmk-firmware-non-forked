
#pragma once

#define USB_MODE                    0
#define THREE_MODE                  1
#define WORK_MODE                   THREE_MODE
#define TAP_CODE_DELAY              8
#define DYNAMIC_KEYMAP_MACRO_DELAY  8
#define EECONFIG_USER_DATA_SIZE  	12
#define DEV_MODE_PIN             	C0
#define SYS_MODE_PIN            	C1
#define DC_BOOST_PIN             	C2
#define NRF_RESET_PIN            	B4
#define NRF_BOOT_PIN             	B5
#define NRF_WAKEUP_PIN           	B8
#define WS2812_PWM_DRIVER       	PWMD3
#define WS2812_PWM_CHANNEL      	2
#define WS2812_PWM_PAL_MODE     	1
#define WS2812_DMA_STREAM       	STM32_DMA1_STREAM3
#define WS2812_DMA_CHANNEL      	3
#define WS2812_PWM_TARGET_PERIOD    800000
#define DRIVER_RGB_DI_PIN        	A7
#define DRIVER_LED_CS_PIN        	C6
#define DRIVER_SIDE_PIN          	C8
#define DRIVER_SIDE_CS_PIN       	C9
#define SERIAL_DRIVER            	SD1
#define SD1_TX_PIN               	B6
#define SD1_TX_PAL_MODE          	0
#define SD1_RX_PIN               	B7
#define SD1_RX_PAL_MODE          	0
#define RGB_MATRIX_DEFAULT_MODE     RGB_MATRIX_CUSTOM_position_mode
#define RGB_DEFAULT_COLOUR          168
#define RGB_DISABLE_WHEN_USB_SUSPENDED


