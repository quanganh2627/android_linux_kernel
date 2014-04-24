/*
 * platform_s5k6b2yx.c: s5k6b2yx platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_s5k6b2yx.h"

enum {
	VT_CAM_RESET = 0,
	VT_CAM_VIS_STBY,
};

static struct gpio cam_gpios[] = {
	[VT_CAM_RESET] = {
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "VT_CAM_NRST"
	},
	[VT_CAM_VIS_STBY] = {
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "VT_CAM_STBY"
	},
};

static struct regulator *regulator;

static int cam_gpio_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(cam_gpios); i++) {
		cam_gpios[i].gpio = get_gpio_by_name(cam_gpios[i].label);
		if (cam_gpios[i].gpio  == -1) {
			pr_err("%s : failed to get gpio(name: %s)\n",
				__func__, cam_gpios[i].label);
			return -EINVAL;
		}
	}
	if (gpio_request_array(cam_gpios, ARRAY_SIZE(cam_gpios)) < 0) {
		pr_err("%s : gpio request error\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void cam_gpio_deinit(void)
{
	gpio_free_array(cam_gpios, ARRAY_SIZE(cam_gpios));
}

static int s5k6b2yx_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	if (flag) {
		gpio_set_value(cam_gpios[VT_CAM_RESET].gpio, 1);

		/* s5k6b2yx initializing time: t3 = 46uS + 16 EXTCLK
		 * 46us) + 0.8uS(19.2Mhz) before sccb communication
		 */
		usleep_range(100, 1000);
	} else {
		gpio_set_value(cam_gpios[VT_CAM_RESET].gpio, 0);
	}

	/* MAX - need to implement VISION mode */
	return 0;
}

static int s5k6b2yx_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret;

	/* need to check the first argument of intel_scu_ipc_osc_clk */
	ret = intel_scu_ipc_osc_clk(/*OSC_CLK_CAM1*/ 3, flag ? clock_khz : 0);
	if (ret < 0) {
		pr_err("%s, camera clock failed, result : %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int s5k6b2yx_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (flag)
		ret = regulator_enable(regulator);
	else
		ret = regulator_disable(regulator);
	if (unlikely(ret < 0)) {
		pr_err("%s: failed to control regulator [%d]!!\n", __func__, flag);
		return ret;
	}

	ret = intel_scu_ipc_msic_vprog2(flag);
	if (ret)
		pr_err("Failed to set regulator vprog2 %s - err:[%d]\n",
				flag ? "on" : "off", ret);

	if (flag)
		usleep_range(1000, 2000);

	if (flag) {
		ret = cam_gpio_init();
		if (ret < 0) {
			pr_err("%s, camera gpio init failed, result : %d\n",
								__func__, ret);
			return ret;
		}
	} else {
		cam_gpio_deinit();
	}

	return ret;
}

static int s5k6b2yx_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

static int s5k6b2yx_platform_init(struct i2c_client *client)
{
	regulator = regulator_get(NULL, "VT_CAM_1.8");

	if (IS_ERR(regulator)) {
		pr_err("%s: failed to get regulator VTCAM_EN\n", __func__);
		return -ENODEV;
	}
	return 0;
}

static int s5k6b2yx_platform_deinit(void)
{
	regulator_put(regulator);

	return 0;
}

static struct camera_sensor_platform_data s5k6b2yx_sensor_platform_data = {
	.gpio_ctrl      = s5k6b2yx_gpio_ctrl,
	.flisclk_ctrl   = s5k6b2yx_flisclk_ctrl,
	.power_ctrl     = s5k6b2yx_power_ctrl,
	.csi_cfg        = s5k6b2yx_csi_configure,
	.platform_init  = s5k6b2yx_platform_init,
	.platform_deinit = s5k6b2yx_platform_deinit,
};

void *s5k6b2yx_platform_data(void *info)
{
	return &s5k6b2yx_sensor_platform_data;
}

