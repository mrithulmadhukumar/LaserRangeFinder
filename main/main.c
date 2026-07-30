#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"
#include "freertos/queue.h" 
#include "driver/gpio.h"
#include "driver/i2c_master.h" 
#include "esp_log.h"

// ST official VL53L1X ULD (needed in components/)
#include "vl53l1x.h"
//3rdparty i2c lcd driver (needed in components/)
#include "hd44780.h"

static const char *TAG = "Laser_Rangefinder";

//pin configuration
#define I2C_MASTER_SDA_IO	21
#define I2C_MASTER_SCL_IO	22
#define I2C_MASTER_FREQ_HZ	400000
#define I2C_PORT		I2C_NUM_0

#define TRIGGER_BUTTON_PIN	GPIO_NUM_4 // Button (Pulls GND when pressed)
#define LASER_CTRL_PIN		GPIO_NUM_18// S8050 Transistor Base Control
		
//i2c addresses
#define VL53L1X_ADDR		0x29 // Sensor Default I2C Address
#define LCD_I2C_ADDR		0x27 // LCD Backboard Address

//global handles
i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t sensor_device;
i2c_master_dev_handle_t lcd_device;

//Queue for passing measurement from sensing task to display task
QueueHandle_t distance_queue;

void i2c_bus_init(void){
	i2c_master_bus_config_t i2c_bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port   = I2C_PORT,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
/*
	//Register VL53L1X sensor
	i2c_device_config_t sensor_dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = VL53L1X_ADDR,
		.scl_speed_hz    = I2C_MASTER_FREQ_HZ,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &sensor_dev_cfg, &sensor_device)); 
	
*/
	//Register LCD Device
	i2c_device_config_t lcd_dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address  = LCD_I2C_ADDR, 
		.scl_speed_hz    = I2C_MASTER_FREQ_HZ,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &lcd_dev_cfg, &lcd_device));

	ESP_LOGI(TAG, "I2C Bus, Sensor, and LCD initialized.");
}

void init_gpios(void){
	//1. Button Input with Internal Pull-Up (Active LOW)
	gpio_config_t btn_cfg = {
		.pin_bit_mask = (1ULL << TRIGGER_BUTTON_PIN),
		.mode	      = GPIO_MODE_INPUT,
		.pull_up_en   = GPIO_PULLUP_ENABLE,
		.intr_type    = GPIO_INTR_DISABLE,
	};
	gpio_config(&btn_cfg);

	//laser switch output for transistor ( Active High)
	gpio_config_t laser_cfg = {
		.pin_bit_mask = (1ULL << LASER_CTRL_PIN),
		.mode	      = GPIO_MODE_OUTPUT,
	};
	gpio_config(&laser_cfg);
	gpio_set_level(LASER_CTRL_PIN, 0);//Ensure OFF by default
}

//-------------------
//Task 1 - Ranging and button test (Priority 10)
//-------------------
void rangefinder_task(void *pvParameters) {
    bool is_measuring = false;
    vl53l1x_t sensor = {0};
    vl53l1x_result_t result = {0};
    float current_dist_cm = 0.00f;

    // 1. Initialize device handle on the existing I2C bus handle
    // 0x29 is the default 7-bit address
    if (vl53l1x_init(&sensor, bus_handle, 0x29) != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X init failed!");
        vTaskDelete(NULL);
    }

    // 2. Run ST ULP mandatory sensor initialization
    if (vl53l1x_sensor_init(&sensor) != ESP_OK) {
        ESP_LOGE(TAG, "VL53L1X sensor init failed!");
        vTaskDelete(NULL);
    }

    // 3. (Optional) Quick preset for long distance (100ms timing)
    vl53l1x_config_long_100ms(&sensor);

    ESP_LOGI(TAG, "Sensor Ready.");

    while (1) {
        bool button_pressed = (gpio_get_level(TRIGGER_BUTTON_PIN) == 0);

        if (button_pressed) {
            // Action on button press
            if (!is_measuring) {
                ESP_LOGI(TAG, "Button Pressed: Start.");
                gpio_set_level(LASER_CTRL_PIN, 1); // Laser ON
                vl53l1x_start(&sensor);            // Start continuous ranging
                is_measuring = true;
            }

            // Poll for data ready and read measurement (100ms timeout)
            if (vl53l1x_read(&sensor, &result, 100) == ESP_OK) {
                if (result.status == 0) { // status 0 = OK
                    current_dist_cm = result.distance_mm / 10.0f;
                    ESP_LOGD(TAG, "D: %.2f cm", current_dist_cm);

                    xQueueOverwrite(distance_queue, &current_dist_cm);
                }
            }
        } else {
            // Action on button release
            if (is_measuring) {
                ESP_LOGI(TAG, "Button Released: Stop.");
                gpio_set_level(LASER_CTRL_PIN, 0); // Laser OFF
                vl53l1x_stop(&sensor);              // Stop ranging
                is_measuring = false;

                current_dist_cm = 0.00f;
                xQueueOverwrite(distance_queue, &current_dist_cm);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Poll every 50ms
    }
}
/*
void rangefinder_task(void *pvParameters){
	bool is_measuring = false;
	uint8_t data_ready = 0;
	uint16_t distance_mm = 0;
	float current_dist_cm = 0.00f;

	//Initialize sensor once
	if (VL53L1X_ULP_SensorInit(sensor_device) != 0){
		ESP_LOGE(TAG, "Sensor Init Failed");
		//
		vTaskDelete(NULL);
	}

	//Configure for longrange mode, 4m max
	VL53L1X_ULP_SetDistanceMode(sensor_device, 2); // 2 = long
	VL53L1X_ULP_SetTimingBudgetInMs(sensor_device, 100);
	VL53L1X_ULP_SetInterMeasurementInMs(sensor_device, 100);

	ESP_LOGI(TAG, "Sensor Ready.");

	while(1){
		// Read button state (LOW when pressed due to PULLUP)
		bool button_pressed = (gpio_get_level(TRIGGER_BUTTON_PIN) == 0);

		if(button_pressed){
			// action on pressing the button
			if (!is_measuring){
				ESP_LOGI(TAG, "Button Pressed: Start.");
				gpio_set_level(LASER_CTRL_PIN, 1); // laser on via s8050
				VL53L1X_ULP_StartRanging(sensor_device);//start continuous ranging
				is_measuring = true;
			}

			//Measurement Loop
			VL53L1X_CheckForDataReady(sensor_device, &data_ready);
			if(data_ready){
				VL53L1X_ULP_GetDistance(sensor_device, &distance_mm);
				VL53L1X_ULP_ClearInterrupt(sensor_device);

				current_dist_cm = distance_mm / 10.0f;
				ESP_LOGD(TAG, "D: %.2f cm", current_dist_cm);

				// send live value to distance task queue
				xQueueOverwrite(distance_queue, &current_dist_cm);
			}
		}
		else{
			// action on button release
			if (is_measuring){
				ESP_LOGI(TAG, "Button Released: Stop.");
				gpio_set_level(LASER_CTRL_PIN, 0);//laser off
				VL53L1X_ULP_StopRanging(sensor_device);//stop ranging
				is_measuring = false;

				//send reset value (0.00) to the display task queue
				current_dist_cm = 0.00f;
				xQueueOverwrite(distance_queue, &current_dist_cm);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(50));//Poll button every 50ms
		}
}
*/ 
//----------------
//Task2 : 1602 LCD Display Update (Priority 5)
//----------------
//----------------
// Task 2 : 1602 LCD Display Update (Priority 5)
//----------------
void display_task(void *pvParameters) {
        float rx_distance_cm = 0.00f;
        char buffer[16];

        // 1. Create LCD descriptor using the i2c_dev_t inside the library
        hd44780_t lcd = {
                .write_cb = NULL, // Handled internally via i2c_dev_t if using pcf8574
                .font = HD44780_FONT_5X8,
                .lines = 2,
                .pins = {
                        .rs = 0,
                        .e = 2,
                        .bl = 3,
                        .d4 = 4,
                        .d5 = 5,
                        .d6 = 6,
                        .d7 = 7
                }
        };

        // 2. Initialize LCD
        hd44780_init(&lcd);
        hd44780_clear(&lcd);
        hd44780_switch_backlight(&lcd, true);
        
        hd44780_gotoxy(&lcd, 0, 0);
        hd44780_puts(&lcd, "ESP32 Rangefinder");

        // Default / Reset state
        hd44780_gotoxy(&lcd, 0, 1);
        hd44780_puts(&lcd, "Hold Trigger...");

        while (1) {
                // Block indefinitely until we receive a new distance
                if (xQueueReceive(distance_queue, &rx_distance_cm, portMAX_DELAY) == pdTRUE) {
                        snprintf(buffer, sizeof(buffer), "%.2f cm   ", rx_distance_cm);

                        if (rx_distance_cm > 0.001f) {
                                hd44780_gotoxy(&lcd, 0, 1);
                                hd44780_puts(&lcd, buffer);
                        } else {
                                hd44780_gotoxy(&lcd, 0, 1);
                                hd44780_puts(&lcd, "Hold Trigger...");
                        }
                }
        }
}

void app_main(void){
	//Initialize Hardware
	i2c_bus_init();
	init_gpios();

	//Create inter task communication queue
	distance_queue = xQueueCreate(1, sizeof(float));
	if (distance_queue == NULL){
		ESP_LOGE(TAG, "Queue creation failed!");
		while(1);
	}

	//launch Tasks
	xTaskCreate(display_task, "display_task", 4096, NULL, 5, NULL);
	xTaskCreate(rangefinder_task, "rangefinder_task", 4096, NULL, 10, NULL);
}

