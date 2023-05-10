/**
 * @file
 * @brief ESP LCD touch: XPT2046
 */

#pragma once

#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xpt2046_calib_point {
    int32_t x;
    int32_t y;
    int32_t z;
};

struct xpt2046_calibration_data {
	struct xpt2046_calib_point 	min;
	struct xpt2046_calib_point 	max;
	uint16_t										pixel_inset;
};


/**
 * @brief Create a new XPT2046 touch driver
 *
 * @note The SPI communication should be initialized before use this function.
 *
 * @param io LCD/Touch panel IO handle
 * @param config: Touch configuration
 * @param out_touch: Touch instance handle
 * @return
 *      - ESP_OK                    on success
 *      - ESP_ERR_NO_MEM            if there is no memory for allocating main structure
 */
esp_err_t esp_lcd_touch_new_spi_xpt2046(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch);


/**
 * @brief Set calibration data
 *
 *
 * @param calib_data: The calibration data to use
 */
void esp_lcd_touch_xpt2046_set_calibration(struct xpt2046_calibration_data *use_calibration_data);


/**
 * @brief Recommended clock for SPI read of the XPT2046
 *
 */
#define ESP_LCD_TOUCH_SPI_CLOCK_HZ   (2500000)

/**
 * @brief Communication SPI device IO structure
 *
 */
#define ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(touch_cs)      \
    {                                               \
        .cs_gpio_num = touch_cs,                    \
        .pclk_hz = ESP_LCD_TOUCH_SPI_CLOCK_HZ,      \
        .lcd_cmd_bits = 8,                          \
        .lcd_param_bits = 8,                        \
        .spi_mode = 0,                              \
        .trans_queue_depth = 1,                     \
    }

#ifdef __cplusplus
}
#endif
