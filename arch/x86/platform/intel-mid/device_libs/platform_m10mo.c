/*
 * platform_m10m0.c: m10m0 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_flis.h>
#include <linux/atomisp_platform.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_m10mo.h"

static int camera_reset;
static int camera_power_down;

/*
 * Ext-ISP m10mo platform data
 */
static int m10mo_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
		gpio_set_value(camera_reset, 1);
		usleep_range(1000, 1500);
	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int m10mo_gpio_intr_ctrl(struct v4l2_subdev *sd)
{
	int ret, pin;
	int gpio = -1;
	static const char gpio_name[] = "xenon_ready";
	bool flag;

	if (gpio == -1) {
		pin = get_gpio_by_name(gpio_name);
		if (pin == -1) {
			pr_err("Failed to get gpio\n");
			return -EINVAL;
		}
		pr_info("camera interrupt gpio: %d\n", pin);
	} else {
		pin = gpio;
	}

	ret = gpio_request(pin, gpio_name);
	if (ret) {
		pr_err("Failed to request interrupt gpio(pin %d)\n", pin);
		return -EINVAL;
	}

	config_pin_flis(ann_gp_camerasb_3, PULL, UP_50K);
	config_pin_flis(ann_gp_camerasb_3, MUX, MUX_EN_INPUT_EN | INPUT_EN);

	ret = gpio_direction_input(pin);

	flag = gpio_get_value(pin);

	if (ret) {
		pr_err("failed to set int gpio(pin %d) direction\n", pin);
		gpio_free(pin);
	}

	if (!ret)
		return pin;
	else
		return -EINVAL;
}

static int m10mo_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
#else
	pr_err("clock is not set.\n");
	return 0;
#endif
}

static int m10mo_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_YUV422_8, 0, flag);
}

static struct camera_sensor_platform_data m10mo_sensor_platform_data = {
	.gpio_ctrl      = m10mo_gpio_ctrl,
	.gpio_intr_ctrl	= m10mo_gpio_intr_ctrl,
	.flisclk_ctrl   = m10mo_flisclk_ctrl,
	.csi_cfg        = m10mo_csi_configure,
};

void *m10mo_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	return &m10mo_sensor_platform_data;
}

#ifdef CONFIG_VIDEO_M10MO_FAKE_SFI_TABLE
static struct sfi_device_table_entry m10mo_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x1F,
	.irq = 0x0,
	.max_freq = 0x0,
	.name = "m10mo",
};

static int __init platform_m10mo_module_init(void)
{
	struct devs_id *dev;
	dev = get_device_id(m10mo_entry.type, m10mo_entry.name);
	if (dev && dev->device_handler)
		dev->device_handler(&m10mo_entry, dev);
	return 0;
}

module_init(platform_m10mo_module_init);
#endif /* CONFIG_M10MO_FAKE_SFI_TABLE */
