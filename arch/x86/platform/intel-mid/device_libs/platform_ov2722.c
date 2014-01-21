/*
 * platform_ov2722.c: ov2722 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <linux/regulator/consumer.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/mfd/intel_mid_pmic.h>

#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_ov2722.h"

/* workround - pin defined for byt */
#define CAMERA_1_RESET 127
#define CAMERA_1_RESET_CHT 148
#define CAMERA_1_RESET_CRV2 120
#define CAMERA_1_PWDN 124
#define CAMERA_1_PWDN_CHT 152
#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM1_CLK 0x1
#define CLK_19P2MHz 0x1
#define CLK_ON	0x1
#define CLK_OFF	0x2
#endif

#ifdef CONFIG_CRYSTAL_COVE
static struct regulator *v1p8_reg;
static struct regulator *v2p8_reg;
#endif

static int camera_vprog1_on;
static int gp_camera1_power_down;
static int gp_camera1_reset;

/*
 * OV2722 platform data
 */

static int ov2722_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;

	if (!IS_BYT) {
		if (gp_camera1_power_down < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			gp_camera1_power_down = ret;
		}

		if (gp_camera1_reset < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			gp_camera1_reset = ret;
		}
	} else {
		/*
		 * FIXME: WA using hardcoded GPIO value here.
		 * The GPIO value would be provided by ACPI table, which is
		 * not implemented currently.
		 */
		if (spid.hardware_id == BYT_TABLET_BLK_CRV2)
			pin = CAMERA_1_RESET_CRV2;
		else if (IS_CHT)
			pin = CAMERA_1_RESET_CHT;
		else
			pin = CAMERA_1_RESET;

		if (gp_camera1_reset < 0) {
			ret = gpio_request(pin, "camera_1_reset");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
				return ret;
			}
		}
		gp_camera1_reset = pin;
		ret = gpio_direction_output(pin, 1);
		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}

		/*
		 * FIXME: WA using hardcoded GPIO value here.
		 * The GPIO value would be provided by ACPI table, which is
		 * not implemented currently.
		 */
		if (IS_CHT)
			pin = CAMERA_1_PWDN_CHT;
		else
			pin = CAMERA_1_PWDN;
		if (gp_camera1_power_down < 0) {
			ret = gpio_request(pin, "camera_1_power");
			if (ret) {
				pr_err("%s: failed to request gpio(pin %d)\n",
					__func__, pin);
				return ret;
			}
		}
		gp_camera1_power_down = pin;

		if (spid.hardware_id == BYT_TABLET_BLK_8PR0 ||
		    spid.hardware_id == BYT_TABLET_BLK_8PR1 ||
		    spid.hardware_id == BYT_TABLET_BLK_CRV2)
			ret = gpio_direction_output(pin, 0);
		else
			ret = gpio_direction_output(pin, 1);

		if (ret) {
			pr_err("%s: failed to set gpio(pin %d) direction\n",
				__func__, pin);
			gpio_free(pin);
			return ret;
		}
	}
	if (flag) {
		if (spid.hardware_id == BYT_TABLET_BLK_8PR0 ||
		    spid.hardware_id == BYT_TABLET_BLK_8PR1 ||
		    spid.hardware_id == BYT_TABLET_BLK_CRV2)
			gpio_set_value(gp_camera1_power_down, 0);
		else
			gpio_set_value(gp_camera1_power_down, 1);

		gpio_set_value(gp_camera1_reset, 0);
		msleep(20);
		gpio_set_value(gp_camera1_reset, 1);
	} else {
		gpio_set_value(gp_camera1_reset, 0);
		if (spid.hardware_id == BYT_TABLET_BLK_8PR0 ||
		    spid.hardware_id == BYT_TABLET_BLK_8PR1 ||
		    spid.hardware_id == BYT_TABLET_BLK_CRV2)
			gpio_set_value(gp_camera1_power_down, 1);
		else
			gpio_set_value(gp_camera1_power_down, 0);
	}

	return 0;
}

static int ov2722_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_VLV2_PLAT_CLK
	int ret = 0;
	if (flag) {
		ret = vlv2_plat_set_clock_freq(OSC_CAM1_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
		return vlv2_plat_configure_clock(OSC_CAM1_CLK, CLK_ON);
	}
	return vlv2_plat_configure_clock(OSC_CAM1_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
				     flag ? clock_khz : 0);
#else
	pr_err("ov2722 clock is not set.\n");
	return 0;
#endif
}

/*
 * The power_down gpio pin is to control OV2722's
 * internal power state.
 */
static int ov2722_power_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_CRYSTAL_COVE
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#endif
	int ret = 0;

#ifdef CONFIG_CRYSTAL_COVE
	if (!v1p8_reg || !v2p8_reg) {
		dev_err(&client->dev,
				"not avaiable regulator\n");
		return -EINVAL;
	}
#endif

	/*
	 * FIXME: VRF has no implementation for CHT now,
	 * remove pmic power control when VRF is ready.
	 */
#ifdef CONFIG_CRYSTAL_COVE
	if (IS_CHT) {
		if (flag) {
			if (!camera_vprog1_on) {
				ret = camera_set_pmic_power(CAMERA_2P8V, true);
				if (ret) {
					dev_err(&client->dev,
						"Failed to enable pmic power v2p8\n");
					return ret;
				}

				ret = camera_set_pmic_power(CAMERA_1P8V, true);
				if (ret) {
					camera_set_pmic_power(CAMERA_2P8V, false);
					dev_err(&client->dev,
						"Failed to enable pmic power v1p8\n");
				}
			}
		} else {
			if (camera_vprog1_on) {
				ret = camera_set_pmic_power(CAMERA_2P8V, false);
				if (ret)
					dev_warn(&client->dev,
						 "Failed to disable pmic power v2p8\n");
				ret = camera_set_pmic_power(CAMERA_1P8V, false);
				if (ret)
					dev_warn(&client->dev,
						 "Failed to disable pmic power v1p8\n");
			}
		}
		return ret;
	}
#endif
	if (flag) {
		if (!camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = regulator_enable(v2p8_reg);
			if (ret) {
				dev_err(&client->dev,
						"Failed to enable regulator v2p8\n");
				return ret;
			}
			ret = regulator_enable(v1p8_reg);
			if (ret) {
				regulator_disable(v2p8_reg);
				dev_err(&client->dev,
						"Failed to enable regulator v1p8\n");
			}
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(1);
#else
			pr_err("ov2722 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 1;
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = regulator_disable(v2p8_reg);
			if (ret)
				dev_warn(&client->dev,
						"Failed to disable regulator v2p8\n");
			ret = regulator_disable(v1p8_reg);
			if (ret)
				dev_warn(&client->dev,
						"Failed to disable regulator v1p8\n");
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(0);
#else
			pr_err("ov2722 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return 0;
}

static int ov2722_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

#ifdef CONFIG_CRYSTAL_COVE
static int ov2722_platform_init(struct i2c_client *client)
{
	v1p8_reg = regulator_get(&client->dev, "v1p8sx");
	if (IS_ERR(v1p8_reg)) {
		dev_err(&client->dev, "v1p8s regulator_get failed\n");
		return PTR_ERR(v1p8_reg);
	}

	v2p8_reg = regulator_get(&client->dev, "v2p85sx");
	if (IS_ERR(v2p8_reg)) {
		regulator_put(v1p8_reg);
		dev_err(&client->dev, "v2p85sx regulator_get failed\n");
		return PTR_ERR(v2p8_reg);
	}

	return 0;
}

static int ov2722_platform_deinit(void)
{
	regulator_put(v1p8_reg);
	regulator_put(v2p8_reg);

	return 0;
}
#endif

static struct camera_sensor_platform_data ov2722_sensor_platform_data = {
	.gpio_ctrl	= ov2722_gpio_ctrl,
	.flisclk_ctrl	= ov2722_flisclk_ctrl,
	.power_ctrl	= ov2722_power_ctrl,
	.csi_cfg	= ov2722_csi_configure,
#ifdef CONFIG_CRYSTAL_COVE
	.platform_init = ov2722_platform_init,
	.platform_deinit = ov2722_platform_deinit,
#endif
};

void *ov2722_platform_data(void *info)
{
	gp_camera1_power_down = -1;
	gp_camera1_reset = -1;
	return &ov2722_sensor_platform_data;
}
