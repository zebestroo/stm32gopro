/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>                                                                                                                                                     


/* The use of the X/Y standard coordinate grid is assumed
 *
 *
 *        (Y)
 *         
 *         ^
 *         |
 *         |
 *         |
 * --------|---------> (X)
 *        0|
 *         |
 *         |
 *	
 *
 * You need to think about that like watching from the camera perspective!
 *
 */

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)

// #define LEGS_BITMASK_RIGHT 	(1 << 0)	/* BITMASK 00000001 */
// #define LEGS_BITMASK_BOTTOM 	(1 << 1)	/* BITMASK 00000010 */
// #define LEGS_BITMASK_TOP 	(1 << 2)	/* BITMASK 00000100 */
// #define LEGS_BITMASK_LEFT 	(1 << 3)	/* BITMASK 00001000 */
// #define LEGS_BITMASK_HORIZONTAL (0x09)		/* BITMASK 00001001 */
// #define LEGS_BITMASK_VERTICAL 	(0x06)		/* BITMASK 00000110 */
// #define LEGS_LIM_OFFSET_RIGHT		(50)
// #define LEGS_LIM_OFFSET_BOTTOM		(-50)
// #define LEGS_LIM_OFFSET_TOP		(50)
// #define LEGS_LIM_OFFSET_LEFT		(-50)
// #define LEGS_CENTER_OFFSET_HORIZONTAL	(0)
// #define LEGS_CENTER_OFFSET_VERTICAL	(0)

#define HEAD_BITMASK_RIGHT 	(1 << 4)	/* BITMASK 00010000 */
#define HEAD_BITMASK_BOTTOM 	(1 << 5)	/* BITMASK 00100000 */
#define HEAD_BITMASK_TOP 	(1 << 6)	/* BITMASK 01000000 */
#define HEAD_BITMASK_LEFT 	(1 << 7)	/* BITMASK 10000000 */
#define HEAD_BITMASK_HORIZONTAL	(0x90)		/* BITMASK 10010000 */
#define HEAD_BITMASK_VERTICAL 	(0x60)		/* BITMASK 01100000 */
#define HEAD_LIM_OFFSET_RIGHT		(50)
#define HEAD_LIM_OFFSET_BOTTOM		(-50)
#define HEAD_LIM_OFFSET_TOP		(50)
#define HEAD_LIM_OFFSET_LEFT		(-50)
#define HEAD_CENTER_OFFSET_HORIZONTAL	(0)
#define HEAD_CENTER_OFFSET_VERTICAL	(0)

typedef struct {
	struct gpio_dt_spec dir_pin;
	struct gpio_dt_spec step_pin;
	struct gpio_dt_spec ms1_pin;
	struct gpio_dt_spec ms2_pin;
	struct gpio_dt_spec ms3_pin;
	int16_t axis_offset;
} motor_driver;

#define TIMER_PERIOD_MS 100

static struct k_timer my_timer;
static volatile bool timer_expired = true;

void timer_handler(struct k_timer *timer_id)
{
    timer_expired = true;
}


static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static const motor_driver motor_1 = {
	.dir_pin = GPIO_DT_SPEC_GET(DT_ALIAS(dirfirst), gpios),
	.step_pin = GPIO_DT_SPEC_GET(DT_ALIAS(stepfirst), gpios),
	.ms1_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ms1first), gpios),
	.ms2_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ms2first), gpios),
	.ms3_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ms3first), gpios),
};

static const motor_driver motor_2 = {
	.dir_pin = GPIO_DT_SPEC_GET(DT_ALIAS(dirsecond), gpios),
	.step_pin = GPIO_DT_SPEC_GET(DT_ALIAS(stepsecond), gpios),
	.ms1_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ms1second), gpios),
	.ms2_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ms2second), gpios),
	.ms3_pin = GPIO_DT_SPEC_GET(DT_ALIAS(ms3second), gpios),
};



void motor_drivers_init() {
	gpio_pin_configure_dt(&motor_1.ms1_pin, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&motor_1.ms2_pin, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&motor_1.ms3_pin, GPIO_OUTPUT_LOW);

	gpio_pin_configure_dt(&motor_2.ms1_pin, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&motor_2.ms2_pin, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&motor_2.ms3_pin, GPIO_OUTPUT_LOW);
}

void do_motor_steps(uint8_t *byte)
{
	if ((*byte & HEAD_BITMASK_HORIZONTAL) == HEAD_BITMASK_HORIZONTAL)
	{
		if (motor_1.axis_offset > HEAD_CENTER_OFFSET_HORIZONTAL)
		{
			gpio_pin_configure_dt(&motor_1.dir_pin, GPIO_OUTPUT_HIGH);	// Maybe wrong direction
			gpio_pin_configure_dt(&motor_1.step_pin, GPIO_OUTPUT_HIGH);
		}
											// Maybe they(^ _) need to be swapped
		if (motor_1.axis_offset < HEAD_CENTER_OFFSET_HORIZONTAL)
		{
			gpio_pin_configure_dt(&motor_1.dir_pin, GPIO_OUTPUT_LOW);	// Maybe wrong direction
			gpio_pin_configure_dt(&motor_1.step_pin, GPIO_OUTPUT_HIGH);
		}

	} else {

		if (*byte & HEAD_BITMASK_LEFT)
		{
			if (motor_1.axis_offset != HEAD_LIM_OFFSET_LEFT)
			{
				gpio_pin_configure_dt(&motor_1.dir_pin, GPIO_OUTPUT_HIGH);	// Maybe wrong direction
				gpio_pin_configure_dt(&motor_1.step_pin, GPIO_OUTPUT_HIGH);
			}

		}

		if (*byte & HEAD_BITMASK_RIGHT)
		{
			if (motor_1.axis_offset != HEAD_LIM_OFFSET_RIGHT)
			{
				gpio_pin_configure_dt(&motor_1.dir_pin, GPIO_OUTPUT_LOW);	// Maybe wrong direction
				gpio_pin_configure_dt(&motor_1.step_pin, GPIO_OUTPUT_HIGH);
			}

		}
	}


	if ((*byte & HEAD_BITMASK_VERTICAL) == HEAD_BITMASK_VERTICAL)
	{
		if (motor_1.axis_offset > HEAD_CENTER_OFFSET_VERTICAL)
		{
			gpio_pin_configure_dt(&motor_2.dir_pin, GPIO_OUTPUT_HIGH);	// Maybe wrong direction
			gpio_pin_configure_dt(&motor_2.step_pin, GPIO_OUTPUT_HIGH);
		}
											// Maybe they(^ _) need to be swapped
		if (motor_1.axis_offset < HEAD_CENTER_OFFSET_VERTICAL)
		{
			gpio_pin_configure_dt(&motor_2.dir_pin, GPIO_OUTPUT_LOW);	// Maybe wrong direction
			gpio_pin_configure_dt(&motor_2.step_pin, GPIO_OUTPUT_HIGH);
		}

	} else {

		if (*byte & HEAD_BITMASK_TOP)
		{
			if (motor_1.axis_offset != HEAD_LIM_OFFSET_TOP)
			{
				gpio_pin_configure_dt(&motor_2.dir_pin, GPIO_OUTPUT_HIGH);	// Maybe wrong direction
				gpio_pin_configure_dt(&motor_2.step_pin, GPIO_OUTPUT_HIGH);
			}

		}

		if (*byte & HEAD_BITMASK_BOTTOM)
		{
			if (motor_1.axis_offset != HEAD_LIM_OFFSET_BOTTOM)
			{
				gpio_pin_configure_dt(&motor_2.dir_pin, GPIO_OUTPUT_LOW);	// Maybe wrong direction
				gpio_pin_configure_dt(&motor_2.step_pin, GPIO_OUTPUT_HIGH);
			}

		}
	}

	k_sleep(K_USEC(1200));
	//k_sleep(K_MSEC(10));

	gpio_pin_configure_dt(&motor_1.step_pin, GPIO_OUTPUT_LOW);
	gpio_pin_configure_dt(&motor_2.step_pin, GPIO_OUTPUT_LOW);
	//printk("Processed\n");
}

void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (!uart_irq_rx_ready(uart_dev)) {
		return;
	}

	if (uart_fifo_read(uart_dev, &c, 1) == 1) {
		printk("Symbol is ---> %d\n", c);
		//do_motor_steps(c);
		timer_expired = false;
		k_timer_stop(&my_timer);
		k_timer_start(&my_timer, K_MSEC(TIMER_PERIOD_MS), K_NO_WAIT);
	}

}

int main(void)
{
	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);

	if (ret < 0) {
		if (ret == -ENOTSUP) {
			printk("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			printk("UART device does not support interrupt-driven API\n");
		} else {
			printk("Error setting UART callback: %d\n", ret);
		}
		return 0;
	}

	uart_irq_rx_enable(uart_dev);
	motor_drivers_init();


	bool dr = true; 
	char t = 49;
	int err;
	float j;

	printk("Timer status example\n");

	k_timer_init(&my_timer, timer_handler, NULL);
	// k_timer_start(&my_timer, K_MSEC(TIMER_PERIOD_MS), K_NO_WAIT);

	while(1) {
		if (!timer_expired) {
			do_motor_steps(&t);
		}

		//k_sleep(K_MSEC(20));

		//if (!timer_expired) {
			//do_motor_steps(&t);
		//}


	}

	return 0;
}
