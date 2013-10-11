/*
 *  controls_v2_dpcm.c - Intel MID Platform driver DPCM ALSA controls for Mrfld
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Omair Mohammed Abdullah <omair.m.abdullah@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <sound/soc.h>
#include "../sst_platform.h"
#include "../sst_platform_pvt.h"
#include "controls_v2.h"

int sst_dsp_init_v2_dpcm(struct snd_soc_platform *platform)
{
	struct sst_data *sst = snd_soc_platform_get_drvdata(platform);

	sst->byte_stream = devm_kzalloc(platform->dev,
					SST_MAX_BIN_BYTES, GFP_KERNEL);
	if (!sst->byte_stream) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}
	sst->widget = devm_kzalloc(platform->dev,
				   SST_NUM_WIDGETS * sizeof(*sst->widget),
				   GFP_KERNEL);
	if (!sst->widget) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}


	return 0;
}
