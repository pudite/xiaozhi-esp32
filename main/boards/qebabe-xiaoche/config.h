#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>

#define AUDIO_INPUT_SAMPLE_RATE  24000
#define AUDIO_OUTPUT_SAMPLE_RATE 24000

// 如果使用 Duplex I2S 模式，请注释下面一行
//#define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX

#define AUDIO_I2S_MIC_GPIO_WS   GPIO_NUM_42
#define AUDIO_I2S_MIC_GPIO_SCK  GPIO_NUM_41
#define AUDIO_I2S_MIC_GPIO_DIN  GPIO_NUM_1
#define AUDIO_I2S_SPK_GPIO_DOUT GPIO_NUM_2
#define AUDIO_I2S_SPK_GPIO_BCLK GPIO_NUM_43
#define AUDIO_I2S_SPK_GPIO_LRCK GPIO_NUM_44

#else

#define AUDIO_I2S_GPIO_WS GPIO_NUM_42
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_41
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_1
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_2

#endif


#define BUILTIN_LED_GPIO        GPIO_NUM_48
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
//#define TOUCH_BUTTON_GPIO       GPIO_NUM_47
//#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_40
//#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_39

#define DISPLAY_SDA_PIN GPIO_NUM_43
#define DISPLAY_SCL_PIN GPIO_NUM_44
#define DISPLAY_WIDTH   128

#if CONFIG_OLED_SSD1306_128X32
#define DISPLAY_HEIGHT  32
#elif CONFIG_OLED_SSD1306_128X64
#define DISPLAY_HEIGHT  64
#elif CONFIG_OLED_SH1106_128X64
#define DISPLAY_HEIGHT  64
#define SH1106
#else
#error "未选择 OLED 屏幕类型"
#endif

#define DISPLAY_MIRROR_X true
#define DISPLAY_MIRROR_Y true


// A MCP Test: Control a lamp
//#define LAMP_GPIO GPIO_NUM_18

// Motor control pins for TC1508 module (N20 motors as wheels)
// Temporarily changed MOTOR_RB_GPIO from GPIO_NUM_3 to GPIO_NUM_21 to avoid strapping pin issues
#define MOTOR_LF_GPIO GPIO_NUM_45   // Left Forward
#define MOTOR_LB_GPIO GPIO_NUM_47// Left Backward
#define MOTOR_RF_GPIO GPIO_NUM_21  // Right Forward
#define MOTOR_RB_GPIO GPIO_NUM_14  // Right Backward (was GPIO_NUM_3)

// OV2640 Camera DVP interface pins (ESP32-S3-N16R8 onboard camera)
// DVP 8-bit data + control signals: GPIO4-13, GPIO15-18
#define CAMERA_PIN_PWDN     GPIO_NUM_NC
#define CAMERA_PIN_RESET    GPIO_NUM_NC
#define CAMERA_PIN_XCLK     GPIO_NUM_15
#define CAMERA_PIN_SIOD     GPIO_NUM_4
#define CAMERA_PIN_SIOC     GPIO_NUM_5

#define CAMERA_PIN_D0       GPIO_NUM_11   // Y2
#define CAMERA_PIN_D1       GPIO_NUM_9    // Y3
#define CAMERA_PIN_D2       GPIO_NUM_8    // Y4
#define CAMERA_PIN_D3       GPIO_NUM_10   // Y5
#define CAMERA_PIN_D4       GPIO_NUM_12   // Y6
#define CAMERA_PIN_D5       GPIO_NUM_18   // Y7
#define CAMERA_PIN_D6       GPIO_NUM_17   // Y8
#define CAMERA_PIN_D7       GPIO_NUM_16   // Y9
#define CAMERA_PIN_VSYNC    GPIO_NUM_6
#define CAMERA_PIN_HREF     GPIO_NUM_7
#define CAMERA_PIN_PCLK     GPIO_NUM_13

#define XCLK_FREQ_HZ        20000000

// Note: Camera SCCB (GPIO4/5) uses separate I2C bus from display I2C (GPIO43/44).
// Camera DVP data/control pins are independent of display I2C.

#endif // _BOARD_CONFIG_H_
