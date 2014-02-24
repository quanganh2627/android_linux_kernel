/*
 * platform_pixter.c: Pixter2+ platform library file
 *
 * (C) Copyright 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include "platform_camera.h"
#include "platform_pixter.h"

static int pixter_0_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = PIXTER_0_LANES;

	if (PIXTER_0_TYPE == RAW_CAMERA)
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY,
			LANES, ATOMISP_INPUT_FORMAT_RAW_10,
			atomisp_bayer_order_rggb, flag);
	else
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY,
			LANES, ATOMISP_INPUT_FORMAT_YUV422_8, 0, flag);
}

static int pixter_1_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = PIXTER_1_LANES;

	if (PIXTER_1_TYPE == RAW_CAMERA)
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY,
			LANES, ATOMISP_INPUT_FORMAT_RAW_10,
			atomisp_bayer_order_rggb, flag);
	else
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY,
			LANES, ATOMISP_INPUT_FORMAT_YUV422_8, 0, flag);
}

static int pixter_2_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = PIXTER_2_LANES;

	if (PIXTER_2_TYPE == RAW_CAMERA)
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_TERTIARY,
			LANES, ATOMISP_INPUT_FORMAT_RAW_10,
			atomisp_bayer_order_rggb, flag);
	else
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_TERTIARY,
			LANES, ATOMISP_INPUT_FORMAT_YUV422_8, 0, flag);
}

static struct atomisp_camera_caps pixter_camera_caps[3];

static struct atomisp_camera_caps *pixter_0_get_camera_caps(void)
{
	pixter_camera_caps[0].sensor_num = 1;
	pixter_camera_caps[0].sensor[0].stream_num = PIXTER_0_STREAMS;
	return &pixter_camera_caps[0];
}

static struct atomisp_camera_caps *pixter_1_get_camera_caps(void)
{
	pixter_camera_caps[1].sensor_num = 1;
	pixter_camera_caps[1].sensor[0].stream_num = PIXTER_1_STREAMS;
	return &pixter_camera_caps[1];
}

static struct atomisp_camera_caps *pixter_2_get_camera_caps(void)
{
	pixter_camera_caps[2].sensor_num = 1;
	pixter_camera_caps[2].sensor[0].stream_num = PIXTER_2_STREAMS;
	return &pixter_camera_caps[2];
}

static struct camera_sensor_platform_data pixter_0_platform_data_ops = {
	.csi_cfg        = pixter_0_csi_configure,
	.get_camera_caps = pixter_0_get_camera_caps,
};

static struct camera_sensor_platform_data pixter_1_platform_data_ops = {
	.csi_cfg        = pixter_1_csi_configure,
	.get_camera_caps = pixter_1_get_camera_caps,
};

static struct camera_sensor_platform_data pixter_2_platform_data_ops = {
	.csi_cfg        = pixter_2_csi_configure,
	.get_camera_caps = pixter_2_get_camera_caps,
};

void *pixter_0_platform_data(void *info)
{
	return &pixter_0_platform_data_ops;
}

void *pixter_1_platform_data(void *info)
{
	return &pixter_1_platform_data_ops;
}

void *pixter_2_platform_data(void *info)
{
	return &pixter_2_platform_data_ops;
}

static struct sfi_device_table_entry pixter_dev0 = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x70,
	.irq = 0,
	.name = "pixter_0"
};

static struct sfi_device_table_entry pixter_dev1 = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x72,
	.irq = 0,
	.name = "pixter_1"
};

static struct sfi_device_table_entry pixter_dev2 = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x72,
	.irq = 0,
	.name = "pixter_2"
};

static int __init platform_pixter_module_init(void)
{
	struct devs_id *dev;
	dev = get_device_id(pixter_dev0.type, pixter_dev0.name);
	if (dev)
		dev->device_handler(&pixter_dev0, dev);
	dev = get_device_id(pixter_dev1.type, pixter_dev1.name);
	if (dev)
		dev->device_handler(&pixter_dev1, dev);
	/* Currently only two I2C salves are supported. */
#if 0
	dev = get_device_id(pixter_dev2.type, pixter_dev2.name);
	if (dev)
		dev->device_handler(&pixter_dev2, dev);
#endif
	return 0;
}

module_init(platform_pixter_module_init);
