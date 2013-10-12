/*
 *  sst_widgets.h - Intel helpers to generate FW widgets
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

#ifndef __SST_WIDGETS_H__
#define __SST_WIDGETS_H__

#include <sound/soc.h>

struct sst_ids {
	u16 location_id;
	u16 module_id;
	u8  task_id;
	u8  ssp_id;
};

#define SST_SSP_AIF_IN(wname, wssp_id, wevent)						\
{	.id = snd_soc_dapm_aif_in, .name = wname, .sname = NULL,			\
	.reg = SND_SOC_NOPM, .shift = 0, .invert = 0,					\
	.event = wevent, .event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD,	\
	.priv = (void *)&(struct sst_ids) { .ssp_id = wssp_id, }			\
}

#define SST_SSP_AIF_OUT(wname, wssp_id, wevent)						\
{	.id = snd_soc_dapm_aif_out, .name = wname, .sname = NULL,			\
	.reg = SND_SOC_NOPM, .shift = 0, .invert = 0,					\
	.event = wevent, .event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD,	\
	.priv = (void *)&(struct sst_ids) { .ssp_id = wssp_id, }			\
}

#define SST_PATH(wname, wtask, wloc_id, wevent, wflags)					\
{	.id = snd_soc_dapm_pga, .name = wname, .reg = SND_SOC_NOPM, .shift = 0,		\
	.invert = 0, .kcontrol_news = NULL, .num_kcontrols = 0,				\
	.event = wevent, .event_flags = wflags,						\
	.priv = (void *)&(struct sst_ids) { .task_id = wtask, .location_id = wloc_id, }	\
}

/* output is triggered before input */
#define SST_PATH_INPUT(name, task_id, loc_id, event) \
	SST_PATH(name, task_id, loc_id, event, SND_SOC_DAPM_POST_PMU)

#define SST_PATH_OUTPUT(name, task_id, loc_id, event) \
	SST_PATH(name, task_id, loc_id, event, SND_SOC_DAPM_PRE_PMU)

#define SST_SWM_MIXER(wname, wreg, wtask, wloc_id, wcontrols, wevent)			\
{	.id = snd_soc_dapm_mixer, .name = wname, .reg = wreg, .shift = 0,		\
	.invert = 0, .kcontrol_news = wcontrols, .num_kcontrols = ARRAY_SIZE(wcontrols),\
	.event = wevent, .event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD,	\
	.priv = (void *)&(struct sst_ids) { .task_id = wtask, .location_id = wloc_id, }	\
}

#endif
