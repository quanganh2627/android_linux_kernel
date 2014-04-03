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
#include <linux/lnw_gpio.h>
#include <linux/interrupt.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-subdev.h>
#include <media/m10mo_atomisp.h>
#include <linux/spi/spi.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include "platform_camera.h"
#include "platform_m10mo.h"

static int camera_reset = -1;
static int camera_power_down = -1;
static void setup_m10mo_spi(struct m10mo_atomisp_spi_platform_data *spi_pdata,
			    void *data);

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

	/* This should be done in pin cfg XML and not here */
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

static struct m10mo_fw_id fw_ids[] = {
	{ "TEST", M10MO_FW_TYPE_1 },
	{ NULL, 0},
};

/*
 * Configure these based on the board. Currently we don't get these
 * from BIOS / IAFW
 */
static int cs_chip_select = -1;

static void spi_hw_resources_setup(struct m10mo_atomisp_spi_platform_data *pdata)
{
	/* Setup SPI interface */
	if (gpio_request(pdata->spi_cs_gpio, "m10mo_spi_cs")) {
		pr_err("Can't allocate gpio for m10mo spi chip select.\n");
		pr_err("Disabling FW update over the SPI\n");
		pdata->spi_enabled = false;
		return;
	}

	cs_chip_select = pdata->spi_cs_gpio;
	lnw_gpio_set_alt(cs_chip_select, LNW_GPIO);
	gpio_direction_output(cs_chip_select, 0);

	/* Setup flis configuration if requested to do so */
	if (pdata->spi_clock_flis != -1)
		config_pin_flis(pdata->spi_clock_flis, MUX,
				MUX_EN_OUTPUT_EN | OUTPUT_EN);

	if (pdata->spi_dataout_flis != -1)
		config_pin_flis(pdata->spi_dataout_flis, MUX,
				MUX_EN_OUTPUT_EN | OUTPUT_EN);

	if (pdata->spi_datain_flis != -1)
		config_pin_flis(pdata->spi_datain_flis,
				MUX, MUX_EN_INPUT_EN | INPUT_EN);

	if (pdata->spi_cs_flis != -1)
		config_pin_flis(pdata->spi_cs_flis,
				MUX, MUX_EN_OUTPUT_EN | OUTPUT_EN);
}

static void spi_cs_control(u32 command)
{
	if (cs_chip_select == -1)
		return;

	/* CS must be set high during transmission */
	if (command == CS_ASSERT)
		gpio_set_value(cs_chip_select, 1);
	else
		gpio_set_value(cs_chip_select, 0);
};

static struct intel_mid_ssp_spi_chip spi_chip = {
	.burst_size	= DFLT_FIFO_BURST_SIZE,
	.timeout	= DFLT_TIMEOUT_VAL,
	.dma_enabled	= false,
	.cs_control	= spi_cs_control,
};

static struct m10mo_platform_data m10mo_sensor_platform_data = {
	/* Common part for all sensors used with atom isp */
	.common.gpio_ctrl	= m10mo_gpio_ctrl,
	.common.gpio_intr_ctrl	= m10mo_gpio_intr_ctrl,
	.common.flisclk_ctrl	= m10mo_flisclk_ctrl,
	.common.csi_cfg		= m10mo_csi_configure,

	/* platform data for spi flashing */
	.spi_pdata.spi_enabled	= false, /* By default SPI is not available */
	.spi_pdata.spi_bus_num	= 6, /* Board specific */
	.spi_pdata.spi_cs_gpio	= 117, /* Board specific */
	.spi_pdata.spi_speed_hz = 10000000, /* Board specific */
	/* Set flis values to -1 if the data is correct in pin cfg xml */
	.spi_pdata.spi_clock_flis	= ann_gp_ssp_6_clk, /* Board specific */
	.spi_pdata.spi_dataout_flis	= ann_gp_ssp_6_txd, /* Board specific */
	.spi_pdata.spi_datain_flis	= ann_gp_ssp_6_rxd, /* Board specific */
	.spi_pdata.spi_cs_flis		= ann_gp_ssp_6_fs, /* Board specific */

	.ref_clock_rate = 24000000, /* Board specific */
	.fw_ids		= fw_ids,
	.spi_setup	= setup_m10mo_spi,
};

static struct spi_board_info m10mo_spi_info[] = {
	{
		.modalias		= "m10mo_spi",
		.mode			= SPI_MODE_0,
		.chip_select		= 0,
		.controller_data	= &spi_chip,
		.platform_data		= &m10mo_sensor_platform_data.spi_pdata,
	}
};

static void setup_m10mo_spi(struct m10mo_atomisp_spi_platform_data *spi_pdata,
	void *data)
{
	m10mo_spi_info[0].bus_num	= spi_pdata->spi_bus_num;
	m10mo_spi_info[0].max_speed_hz	= spi_pdata->spi_speed_hz;
	spi_pdata->device_data		= data;

	spi_hw_resources_setup(spi_pdata);

	/* register SPI interface for the FW update */
	spi_register_board_info(m10mo_spi_info,
				ARRAY_SIZE(m10mo_spi_info));
}

void *m10mo_platform_data(void *info)
{
	/*
	 * Atom isp driver assumes to get pointer to struct
	 * "camera_sensor_platform_data". Nasty assumption but must honored
	 */
	return &m10mo_sensor_platform_data.common;
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
