#include <stdio.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"



/*
	TODO
	1 - Set CPU frequency
	2 - Set PIN
	3 - Set GPIO
	4 - Set I2C
	5 - Set ADC
	6 - Set WiFi Connection
	7 - Set deep sleep mode
*/

/*
	PIN
	MTMS 9 IO14 I2C_SCL
	GPIO2 14 IO2 I2C_SDA
*/

#define GPIO_0  BIT(0)  // FLASH
#define GPIO_1  BIT(1)  // TXD0
#define GPIO_2  BIT(2)  // LED_ON_BOARD
#define GPIO_3  BIT(3)  // RXD0
#define GPIO_4  BIT(4)  // SDA
#define GPIO_5  BIT(5)  // SCL
#define GPIO_6  BIT(6)  //
#define GPIO_7  BIT(7)  //
#define GPIO_8  BIT(8)  //
#define GPIO_9  BIT(9)  //
#define GPIO_10 BIT(10) //
#define GPIO_11 BIT(11) //
#define GPIO_12 BIT(12) // MISO
#define GPIO_13 BIT(13) // MOSI
#define GPIO_14 BIT(14) // SCLK
#define GPIO_15 BIT(15) // SS
#define GPIO_16 BIT(16) // WAKE

uint8_t i2c_channel = 0;
esp_err_t err;
gpio_config_t gpio2;
uart_config_t uart0;
i2c_config_t i2c;

/* Main programm */
void app_main() {

	/* Configure uart0 */
	uart0.baud_rate = 9600;
	uart0.data_bits = 8;
	uart0.parity = UART_PARITY_DISABLE;
	uart0.stop_bits = UART_STOP_BITS_1;
	uart0.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	/* Configure GPIO2 */
	i2c.mode = I2C_MODE_MASTER;
	i2c.sda_io_num = GPIO_4;
	i2c.scl_io_num = GPIO_5;
	i2c.sda_pullup_en = GPIO_PULLUP_DISABLE;
	i2c.scl_pullup_en = GPIO_PULLUP_DISABLE;

	/* Configure i2c cahnnel 0*/
	gpio2.pin_bit_mask = BIT(2);  // select GPIO2
	gpio2.mode = GPIO_MODE_OUTPUT; // output mode
	gpio2.pull_up_en = GPIO_PULLUP_DISABLE; // disable pullup
	gpio2.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pulldown
	gpio2.intr_type= GPIO_INTR_DISABLE; // disable all interrupt in GPIO2

	err = uart_intr_config(UART_NUM_0, &uart0);
	if (err != ESP_OK) {

	}

	err = gpio_config(&gpio2); // apply configuration
	if (err != ESP_OK) {

	}

 	err =  i2c_param_config(i2c_channel, &i2c); // 
	if (err != ESP_OK) {
		
	}

	err = gpio_set_level(GPIO_2, 0); // power off led
	if (err != ESP_OK) {
		
	}

	return;
}
