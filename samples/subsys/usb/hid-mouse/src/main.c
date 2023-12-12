/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zephyr/logging/log.h>
#include "nrfx_clock.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);
static enum usb_dc_status_code usb_status;

#define MOUSE_BTN_LEFT		0
#define MOUSE_BTN_RIGHT		1

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

static uint8_t report[MOUSE_REPORT_COUNT];
static K_SEM_DEFINE(report_sem, 0, 1);

static bool configured;
static const struct device *hdev;
static void report_event_handler(struct k_timer *dummy);
static K_TIMER_DEFINE(event_timer, report_event_handler, NULL);
static struct k_work report_send;
static ATOMIC_DEFINE(hid_ep_in_busy, 1);

#define HID_EP_BUSY_FLAG	0
#define REPORT_PERIOD		K_MSEC(1)

static void int_in_ready_cb(const struct device *dev)
{
	ARG_UNUSED(dev);
	//LOG_INF("IN endpoint callback !");
	if (!atomic_test_and_clear_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
		LOG_WRN("IN endpoint callback without preceding buffer write");
	}
	else
	{
		k_sem_give(&report_sem);
	}
}

static const struct hid_ops ops = {
	.int_in_ready = int_in_ready_cb,
};


static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	usb_status = status;

	switch (status) {
	case USB_DC_RESET:
		configured = false;
		break;
	case USB_DC_CONFIGURED:
		if (!configured) {
			int_in_ready_cb(hdev);
			configured = true;
		}
		break;
	case USB_DC_SOF:
		break;
	default:
		LOG_DBG("status %u unhandled", status);
		break;
	}
}

static ALWAYS_INLINE void rwup_if_suspended(void)
{
	if (IS_ENABLED(CONFIG_USB_DEVICE_REMOTE_WAKEUP)) {
		if (usb_status == USB_DC_SUSPEND) {
			usb_wakeup_request();
			return;
		}
	}
}
#if 0
static void input_cb(struct input_event *evt)
{
	uint8_t tmp[MOUSE_REPORT_COUNT];

	(void)memcpy(tmp, report, sizeof(tmp));

	switch (evt->code) {
	case INPUT_KEY_0:
		rwup_if_suspended();
		WRITE_BIT(tmp[MOUSE_BTN_REPORT_IDX], MOUSE_BTN_LEFT, evt->value);
		break;
	case INPUT_KEY_1:
		rwup_if_suspended();
		WRITE_BIT(tmp[MOUSE_BTN_REPORT_IDX], MOUSE_BTN_RIGHT, evt->value);
		break;
	case INPUT_KEY_2:
		if (evt->value) {
			tmp[MOUSE_X_REPORT_IDX] += 10U;
		}

		break;
	case INPUT_KEY_3:
		if (evt->value) {
			tmp[MOUSE_Y_REPORT_IDX] += 10U;
		}

		break;
	default:
		LOG_INF("Unrecognized input code %u value %d",
			evt->code, evt->value);
		return;
	}

	if (memcmp(tmp, report, sizeof(tmp))) {
		memcpy(report, tmp, sizeof(report));
		k_sem_give(&report_sem);
	}
}

INPUT_CALLBACK_DEFINE(NULL, input_cb);
#endif

static void send_report(struct k_work *work)
{
	int ret, wrote;		

	if (!atomic_test_and_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG)) {
	
	  ret = hid_int_ep_write(hdev, report, sizeof(report),  &wrote);
		//report[MOUSE_X_REPORT_IDX] = 0U;
		//report[MOUSE_Y_REPORT_IDX] = 0U;
		if (ret != 0) {
			/*
			 * Do nothing and wait until host has reset the device
			 * and hid_ep_in_busy is cleared.
			 */
			LOG_ERR("Failed to submit report");
		} else {
			LOG_DBG("Report submitted");
			/* Toggle LED on sent report */
		//ret = gpio_pin_toggle(led0.port, led0.pin);
		if (ret < 0) {
			LOG_ERR("Failed to toggle the LED pin, error: %d", ret);
		}
		}
	} else {
		LOG_DBG("HID IN endpoint busy");
	}
}


static int point_index;
static void report_event_handler(struct k_timer *dummy)
{
	/* Increment reported data */
	//report_1.value++;
	uint8_t tmp[MOUSE_REPORT_COUNT];

	(void)memcpy(tmp, report, sizeof(tmp));

	
	switch (point_index) {
	case 0:
	    tmp[MOUSE_X_REPORT_IDX] = 15U;
        tmp[MOUSE_Y_REPORT_IDX] =  0U;

		break;
	case 1:
	    tmp[MOUSE_X_REPORT_IDX] =  0U;
        tmp[MOUSE_Y_REPORT_IDX] = 15U;	

		break;
	case 2:
	    tmp[MOUSE_X_REPORT_IDX] = 0xF1;
        tmp[MOUSE_Y_REPORT_IDX] =  0U;			

		break;
	case 3:
	    tmp[MOUSE_X_REPORT_IDX] =  0U;
        tmp[MOUSE_Y_REPORT_IDX] = 0xF1;			

		break;
	default:
		LOG_INF("Unrecognized point index");
		return;
	}


	if (memcmp(tmp, report, sizeof(tmp))) {
		memcpy(report, tmp, sizeof(report));
		  k_work_submit(&report_send);
	}

	point_index = (point_index+1)%4;

}

static void report_event_handler1(void)
{
	/* Increment reported data */
	//report_1.value++;
	uint8_t tmp[MOUSE_REPORT_COUNT];

	(void)memcpy(tmp, report, sizeof(tmp));

	
	switch (point_index) {
	case 0:
	    tmp[MOUSE_X_REPORT_IDX] = 15U;
        tmp[MOUSE_Y_REPORT_IDX] =  0U;

		break;
	case 1:
	    tmp[MOUSE_X_REPORT_IDX] =  0U;
        tmp[MOUSE_Y_REPORT_IDX] = 15U;	

		break;
	case 2:
	    tmp[MOUSE_X_REPORT_IDX] = 0xF1;
        tmp[MOUSE_Y_REPORT_IDX] =  0U;			

		break;
	case 3:
	    tmp[MOUSE_X_REPORT_IDX] =  0U;
        tmp[MOUSE_Y_REPORT_IDX] = 0xF1;			

		break;
	default:
		LOG_INF("Unrecognized point index");
		return;
	}


	if (memcmp(tmp, report, sizeof(tmp))) {
		memcpy(report, tmp, sizeof(report));
		  //k_work_submit(&report_send);
		  send_report(NULL);
	}

	point_index = (point_index+1)%4;

}

int main(void)
{
    int ret;

#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M
	/* For now hardcode to 128MHz */
	nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK,
			       NRF_CLOCK_HFCLK_DIV_1);
#endif

	if (!gpio_is_ready_dt(&led0)) {
		LOG_ERR("LED device %s is not ready", led0.port->name);
		return 0;
	}

	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return 0;
	}

	usb_hid_register_device(hdev,
				hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);


	usb_hid_init(hdev);

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	while (true) { 
		gpio_pin_set_raw(led0.port, led0.pin, 0);
        k_sem_take(&report_sem, K_FOREVER);
        gpio_pin_set_raw(led0.port, led0.pin, 1);

		report_event_handler1();
		//k_sleep(K_MSEC(1));
#if 0		
		k_sem_take(&report_sem, K_FOREVER);

		ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
		report[MOUSE_X_REPORT_IDX] = 0U;
		report[MOUSE_Y_REPORT_IDX] = 0U;
		if (ret) {
			LOG_ERR("HID write error, %d", ret);
		}

		/* Toggle LED on sent report */
		ret = gpio_pin_toggle(led0.port, led0.pin);
		if (ret < 0) {
			LOG_ERR("Failed to toggle the LED pin, error: %d", ret);
		}
#endif 
	}

	return 0;


}

int main1(void)
{
	//const struct device *hid_dev;
	int ret;

#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M
	/* For now hardcode to 128MHz */
	nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK,
			       NRF_CLOCK_HFCLK_DIV_1);
#endif

	if (!gpio_is_ready_dt(&led0)) {
		LOG_ERR("LED device %s is not ready", led0.port->name);
		return 0;
	}

	hdev = device_get_binding("HID_0");
	if (hdev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure the LED pin, error: %d", ret);
		return 0;
	}

	usb_hid_register_device(hdev,
				hid_report_desc, sizeof(hid_report_desc),
				&ops);

	atomic_set_bit(hid_ep_in_busy, HID_EP_BUSY_FLAG);
	k_timer_start(&event_timer, REPORT_PERIOD, REPORT_PERIOD);

	usb_hid_init(hdev);

	ret = usb_enable(status_cb);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	k_work_init(&report_send, send_report);
	return 0;
}
