
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_xpt2046.h"

static const char *TAG = "XPT2046";

/* XPT2046 registers */

#define ESP_LCD_TOUCH_XPT2046_SLEEP	 	(0x00)
#define ESP_LCD_TOUCH_XPT2046_READ_Y  (0x90)
#define ESP_LCD_TOUCH_XPT2046_READ_Z1 (0xB0)
#define ESP_LCD_TOUCH_XPT2046_READ_Z2 (0xC0)
#define ESP_LCD_TOUCH_XPT2046_READ_X  (0xD0)

#define CLAMP(value, low, high) value = (value < low) ? low : (value > high ? high : value);

static gpio_num_t int_gpio_num = GPIO_NUM_NC;


static struct xpt2046_calibration_data calibration_data = {
		.min.x = 200,
		.min.y = 360,
		.min.z = 5000,
		.max.x = 3750,
		.max.y = 3850,
		.max.z = 13000,
		.pixel_inset = 0
};

/*******************************************************************************
* Function definitions
*******************************************************************************/
static esp_err_t esp_lcd_touch_xpt2046_read_data(esp_lcd_touch_handle_t tp);
static bool esp_lcd_touch_xpt2046_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num);
static esp_err_t esp_lcd_touch_xpt2046_del(esp_lcd_touch_handle_t tp);

/*******************************************************************************
* Public API functions
*******************************************************************************/

esp_err_t esp_lcd_touch_new_spi_xpt2046(const esp_lcd_panel_io_handle_t io, const esp_lcd_touch_config_t *config, esp_lcd_touch_handle_t *out_touch)
{
    esp_err_t ret = ESP_OK;

    assert(io != NULL);
    assert(config != NULL);
    assert(out_touch != NULL);

    /* Prepare main structure */
    esp_lcd_touch_handle_t esp_lcd_touch_xpt2046 = heap_caps_calloc(1, sizeof(esp_lcd_touch_t), MALLOC_CAP_DEFAULT);
    ESP_GOTO_ON_FALSE(esp_lcd_touch_xpt2046, ESP_ERR_NO_MEM, err, TAG, "no mem for XPT2046 controller");

    /* Communication interface */
    esp_lcd_touch_xpt2046->io = io;

    /* Only supported callbacks are set */
    esp_lcd_touch_xpt2046->read_data = esp_lcd_touch_xpt2046_read_data;
    esp_lcd_touch_xpt2046->get_xy = esp_lcd_touch_xpt2046_get_xy;
    esp_lcd_touch_xpt2046->del = esp_lcd_touch_xpt2046_del;

    /* Mutex */
    esp_lcd_touch_xpt2046->data.lock.owner = portMUX_FREE_VAL;

    /* Save config */
    memcpy(&esp_lcd_touch_xpt2046->config, config, sizeof(esp_lcd_touch_config_t));

    /* Prepare pin for touch interrupt */
    if (esp_lcd_touch_xpt2046->config.int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t int_gpio_config = {
            .mode = GPIO_MODE_INPUT,
            .intr_type = GPIO_INTR_NEGEDGE,
            .pin_bit_mask = BIT64(esp_lcd_touch_xpt2046->config.int_gpio_num)
        };
        ret = gpio_config(&int_gpio_config);
        ESP_GOTO_ON_ERROR(ret, err, TAG, "GPIO config failed");
    }

err:
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error (0x%x)! Touch controller XPT2046 initialization failed!", ret);
        if (esp_lcd_touch_xpt2046) {
            esp_lcd_touch_xpt2046_del(esp_lcd_touch_xpt2046);
        }
    }

    *out_touch = esp_lcd_touch_xpt2046;

    return ret;
}

static inline esp_err_t xpt2046_read_register(esp_lcd_touch_handle_t tp, uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2] = {0, 0};
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(tp->io, reg, buffer, 2), TAG, "XPT2046 read error!");
    *value = ((((uint16_t)buffer[0]) << 8) | ((uint16_t)buffer[1])) >> 3;
    return ESP_OK;
}

static inline esp_err_t XPT2046_transfer_16_add(esp_lcd_touch_handle_t tp, uint8_t reg, uint32_t *value) {
		uint8_t buffer[2] = {0, 0};
    ESP_RETURN_ON_ERROR(esp_lcd_panel_io_rx_param(tp->io, reg, buffer, 2), TAG, "XPT2046 read error!");
		*value += ((((uint16_t)buffer[0]) << 8) | ((uint16_t)buffer[1])) >> 3;

		return ESP_OK;
}

static inline uint16_t xpt2046_clamp(uint16_t value, uint16_t low, uint16_t high) {
	return (value < low) ? low : (value > high ? high : value);
}

static esp_err_t esp_lcd_touch_xpt2046_read_data(esp_lcd_touch_handle_t tp)
{

	  bool touch_down = true;
	  if(int_gpio_num != GPIO_NUM_NC) {
	  		touch_down = gpio_get_level(int_gpio_num);
	  }

	  if(touch_down) {
				uint16_t average_samples = CONFIG_ESP_LCD_TOUCH_XPT2046_Z_OVERSAMPLE_COUNT;
				uint32_t raw_x = 0;
				uint32_t raw_y = 0;
				uint16_t raw_z1 = 0;
				uint16_t raw_z2 = 0;
				uint16_t discard_x = 0;

				// wake and throw away value
				ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, ESP_LCD_TOUCH_XPT2046_READ_X, &discard_x), TAG, "XPT2046 read error!");

				// basic rejection on z1
				ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, ESP_LCD_TOUCH_XPT2046_READ_Z1, &raw_z1), TAG, "XPT2046 read error!");

				if(raw_z1 > CONFIG_ESP_LCD_TOUCH_XPT2046_Z_NEEDED_FOR_TOUCH) {
					  ESP_RETURN_ON_ERROR(xpt2046_read_register(tp, ESP_LCD_TOUCH_XPT2046_READ_Z2, &raw_z2), TAG, "XPT2046 read error!");

						for(uint16_t u = 0 ; u < average_samples; u++) {
							ESP_RETURN_ON_ERROR(XPT2046_transfer_16_add(tp, ESP_LCD_TOUCH_XPT2046_READ_X, &raw_x), TAG, "XPT2046 read error!");
							ESP_RETURN_ON_ERROR(XPT2046_transfer_16_add(tp, ESP_LCD_TOUCH_XPT2046_READ_Y, &raw_y), TAG, "XPT2046 read error!");
						}


						// recheck int pin
						if(int_gpio_num!= GPIO_NUM_NC) {
								touch_down = gpio_get_level(int_gpio_num);
						}

						if(touch_down) {
								raw_x /= average_samples;
								raw_y /= average_samples;

								uint16_t raw_z = ((((raw_z2) / raw_z1) -1) * (raw_x+1));

								// clamp raw to calibration data
								raw_x = xpt2046_clamp(raw_x, calibration_data.min.x, calibration_data.max.x);
								raw_y = xpt2046_clamp(raw_y, calibration_data.min.y, calibration_data.max.y);

								// calc Z from calibration data
								raw_z = xpt2046_clamp(raw_z, calibration_data.min.z, calibration_data.max.z);
								uint16_t z = 100-(((raw_z - calibration_data.min.z) * 100) / (calibration_data.max.z - calibration_data.min.z));

								// convert to screen (from calibration data)
								uint32_t y, x;

								// calculate for no rotation
								x = (raw_x - calibration_data.min.x) * tp->config.x_max / (calibration_data.max.x - calibration_data.min.x);
								y = (raw_y - calibration_data.min.y) * tp->config.y_max / (calibration_data.max.y - calibration_data.min.y);

								// clamp x/y
								x = xpt2046_clamp(x, 0, tp->config.x_max);
								y = xpt2046_clamp(y, 0, tp->config.y_max);

								portENTER_CRITICAL(&tp->data.lock);
								tp->data.coords[0].x = x;
								tp->data.coords[0].y = y;
								tp->data.coords[0].strength = z;
								tp->data.points = 1;
								portEXIT_CRITICAL(&tp->data.lock);
						}
				}
	  }

	  if(!touch_down) {
      portENTER_CRITICAL(&tp->data.lock);
      tp->data.coords[0].strength = 0;
      tp->data.points = 0;
      portEXIT_CRITICAL(&tp->data.lock);
	  }

    return ESP_OK;
}

static bool esp_lcd_touch_xpt2046_get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength, uint8_t *point_num, uint8_t max_point_num)
{
    assert(tp != NULL);
    assert(x != NULL);
    assert(y != NULL);
    assert(point_num != NULL);
    assert(max_point_num > 0);

    portENTER_CRITICAL(&tp->data.lock);

    /* Count of points */
    *point_num = (tp->data.points > max_point_num ? max_point_num : tp->data.points);

    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;

        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }

    /* Invalidate */
    tp->data.points = 0;

    portEXIT_CRITICAL(&tp->data.lock);

    return (*point_num > 0);
}

static esp_err_t esp_lcd_touch_xpt2046_del(esp_lcd_touch_handle_t tp)
{
    assert(tp != NULL);

    /* Reset GPIO pin settings */
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }

    /* Reset GPIO pin settings */
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }

    free(tp);

    return ESP_OK;
}


void esp_lcd_touch_xpt2046_set_calibration(struct xpt2046_calibration_data *use_calibration_data) {
		memcpy(&calibration_data, use_calibration_data, sizeof(struct xpt2046_calibration_data));
}


