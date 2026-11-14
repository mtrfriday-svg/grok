#ifndef MDEE_CONFIG_H
#define MDEE_CONFIG_H

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_lcd_panel_vendor.h>

// Single-board M'DEE: ESP32-S3 N16R8 (WROOM-1, 16 MB flash, 8 MB octal PSRAM).
// Every pin is 3.3 V logic. GPIO 21 and GPIO 47 are reserved and unused.

// ===================== AUDIO (shared I2S, duplex) =====================
// INMP441 mic + MAX98357A amp. INMP441 L/R -> GND. Amp VIN from the 5 V logic rail.
#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_4
#define AUDIO_I2S_GPIO_WS   GPIO_NUM_5
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_6    // INMP441 SD
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_7    // MAX98357A DIN

// ===================== SHARED SPI (SPI3) both ST7789 panels =====================
#define DISPLAY_MOSI_PIN GPIO_NUM_11
#define DISPLAY_CLK_PIN  GPIO_NUM_12
#define DISPLAY_DC_PIN   GPIO_NUM_9       // shared
#define DISPLAY_RST_PIN  GPIO_NUM_8       // shared, hardware reset by the face only
#define DISPLAY_SPI_MODE 0
#define DISPLAY_RGB_ORDER LCD_RGB_ELEMENT_ORDER_RGB

// Face: 2.0" ST7789V, landscape 320x240. Tweak mirrors here only.
#define DISPLAY_CS_PIN        GPIO_NUM_10
#define DISPLAY_WIDTH         320
#define DISPLAY_HEIGHT        240
#define DISPLAY_SWAP_XY       true
#define DISPLAY_MIRROR_X      false
#define DISPLAY_MIRROR_Y      false
#define DISPLAY_INVERT_COLOR  true
#define DISPLAY_OFFSET_X      0
#define DISPLAY_OFFSET_Y      0
#define DISPLAY_BL_PIN        GPIO_NUM_42  // both backlight wires joined, LEDC timer0 ch0
#define DISPLAY_BL_INVERT     false

// Status: 1.54" ST7789 240x240, no LVGL. Software reset (reset gpio = NC).
#define STATUS_CS_PIN         GPIO_NUM_13
#define STATUS_WIDTH          240
#define STATUS_HEIGHT         240
#define STATUS_SWAP_XY        false
#define STATUS_MIRROR_X       false
#define STATUS_MIRROR_Y       false
#define STATUS_INVERT_COLOR   true
#define STATUS_OFFSET_X       0
#define STATUS_OFFSET_Y       0
#define STATUS_SWAP_BYTES     1            // match LVGL flush (swap_bytes = 1)
#define STATUS_REFRESH_MS     500          // about 2 Hz

#define BOOT_LOGO_MS          3700

// ===================== MOTORS: TB6612FNG, PWM pins tied to 3V3 =====================
// Speed is MCPWM on the four IN pins (20 kHz). STBY is tied to 3V3.
// Set every IN low at boot before anything else (internal IN pulldowns also hold them).
#define MOTOR_AIN1_PIN GPIO_NUM_38
#define MOTOR_AIN2_PIN GPIO_NUM_39
#define MOTOR_BIN1_PIN GPIO_NUM_40
#define MOTOR_BIN2_PIN GPIO_NUM_41
#define MOTOR_LEFT_INVERT  0
#define MOTOR_RIGHT_INVERT 0
#define MOTOR_PWM_HZ       20000
// Pack is 6.0-8.4 V; motors are rated 2.5-6 V. Duty = target_volts / pack_volts.
#define MOTOR_MAX_VOLTS       5.5f
#define MOTOR_MIN_START_VOLTS 2.4f
#define MOTOR_ASSUME_PACK_V   7.4f         // used when no pack is detected
#define MOTOR_RAMP_PCT        8            // percent per 20 ms; stop is instant
#define MOTOR_CONTROL_MS      20
#define DRIVE_DEADMAN_MS      450
#define MOTOR_LOW_V_CUTOFF    1            // 1 = stop motors if pack stays under the floor
#define MOTOR_CUTOFF_VOLTS    6.3f
#define MOTOR_CUTOFF_MS       5000
#define OBSTACLE_STOP_CM      18
#define OBSTACLE_SLOW_CM      40
#define WANDER_FRONT_CM       24
#define WANDER_REAR_CLEAR_CM  28
#define FOLLOW_FAR_CM         90
#define FOLLOW_GO_CM          40
#define FOLLOW_CLOSE_CM       22

// Speed levels are fractions of the [MIN_START, MAX] voltage span: 25/50/75/100.
#define SPEED_SLOW_PCT   25
#define SPEED_MEDIUM_PCT 50
#define SPEED_FAST_PCT   75
#define SPEED_TURBO_PCT  100

// ===================== ULTRASONIC: one TRIG, two ECHO pins =====================
// Each ECHO goes through a 1k (sensor side) / 2k (GND) divider into the ESP32.
#define US_TRIG_PIN       GPIO_NUM_2
#define US_ECHO_FRONT_PIN GPIO_NUM_3       // strapping pin, input only
#define US_ECHO_REAR_PIN  GPIO_NUM_46      // input only
#define US_ECHO_WAIT_US   6000
#define US_ECHO_HIGH_US   15000
#define US_NOTHING_CM     400

// ===================== SERVOS: 4 positional, no mechanical end stops =====================
// LEDC 50 Hz, 14-bit, timer1 channels 1-4. Pulse 500-2400 us. Pose space is 0-100.
// Ears: 0 = down/droop, 100 = up/perk.
// Neck tilt: 0 = chin down, 100 = looking up, 50 = level.
// Neck turn: 0 = left, 50 = centre, 100 = right.
// angle_at_0 / angle_at_100 are the servo degrees for pose 0 and pose 100.
// Absolute command clamp is SERVO_ABS_MIN..SERVO_ABS_MAX (no end stops fitted).
// Set any *_ENABLED to 0 if that servo is missing; the pin is left as an input.
#define SERVO_ABS_MIN 15
#define SERVO_ABS_MAX 165
#define SERVO_EAR_MAX_DPS  320
#define SERVO_NECK_MAX_DPS 140

#define SERVO_TURN_ENABLED 1
#define SERVO_TURN_PIN     GPIO_NUM_14
#define SERVO_TURN_AT_0    50             // pose 0 = left
#define SERVO_TURN_AT_100  130            // pose 100 = right (pose 50 ~= 90 deg)

#define SERVO_TILT_ENABLED 1
#define SERVO_TILT_PIN     GPIO_NUM_15
#define SERVO_TILT_AT_0    125            // chin down
#define SERVO_TILT_AT_100  65             // looking up

#define SERVO_EAR_L_ENABLED 1
#define SERVO_EAR_L_PIN     GPIO_NUM_16
#define SERVO_EAR_L_AT_0    135           // droop
#define SERVO_EAR_L_AT_100  55            // perk

#define SERVO_EAR_R_ENABLED 1
#define SERVO_EAR_R_PIN     GPIO_NUM_17
#define SERVO_EAR_R_AT_0    45            // droop (mirrored)
#define SERVO_EAR_R_AT_100  125           // perk

#define SERVO_IDX_TURN 0
#define SERVO_IDX_TILT 1
#define SERVO_IDX_EAR_L 2
#define SERVO_IDX_EAR_R 3

// ===================== LIGHTS =====================
// 4 WS2812 chained, 330 ohm series. GPIO48 may also be the on-board RGB (mirrors pixel 0).
#define NEOPIXEL_PIN   GPIO_NUM_48
#define NEOPIXEL_COUNT 4
#define BLUE_LED_PIN   GPIO_NUM_18         // both blue LEDs joined, active high, LEDC timer2 ch5
#define LIGHT_DEFAULT_BRIGHTNESS 40

// ===================== PACK BATTERY (2S2P, one divider) =====================
// GPIO1 = ADC1_CH0. 100k top, 22k bottom, 100 nF from the pin to GND.
// Below PACK_NO_PACK_VOLTS the firmware assumes USB / no pack and never cuts off.
#define PACK_ADC_UNIT    ADC_UNIT_1
#define PACK_ADC_CHANNEL ADC_CHANNEL_0
#define PACK_R_UPPER     100000.0f
#define PACK_R_LOWER     22000.0f
#define BATTERY_CAL_FACTOR 1.00f
#define PACK_NO_PACK_VOLTS 4.0f
#define PACK_LOW_PCT       15
#define PACK_LOW_MS        5000
#define PACK_CRITICAL_VOLTS 6.2f
#define PACK_CRITICAL_MS    10000

// ===================== BUTTON / POWER / VOICE =====================
#define BOOT_BUTTON_GPIO GPIO_NUM_0
#define BOOT_LONG_PRESS_MS 2000
#define INSTANT_COMMANDS_MUTE_AI 1
#define INSTANT_MUTE_MS 4000

// Phone page. Empty string disables the PIN. Open http://<ip>/?k=<pin> when set.
#define WEB_CONTROL_PIN ""
#define WEB_CONTROL_PORT 80

// POSIX TZ string. Examples: "UTC0", "EST5EDT,M3.2.0,M11.1.0", "CST-8".
#define MDEE_TZ "UTC0"

// Reserved, unused (future link): GPIO 21 and GPIO 47. Do not assign them.

#endif
