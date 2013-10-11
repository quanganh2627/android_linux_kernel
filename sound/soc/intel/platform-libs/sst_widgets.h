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
#include <sound/tlv.h>

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

enum sst_gain_kcontrol_type {
	SST_GAIN_TLV,
	SST_GAIN_MUTE,
	SST_GAIN_RAMP_DURATION,
};

struct sst_gain_mixer_control {
	bool stereo;
	enum sst_gain_kcontrol_type type;
	struct sst_gain_value *gain_val;
	int max;
	int min;
	u16 instance_id;
	u16 module_id;
	u16 pipe_id;
	u16 task_id;
};

struct sst_gain_value {
	u16 ramp_duration;
	s16 l_gain;
	s16 r_gain;
	bool mute;
};

#define SST_GAIN_VOLUME_DEFAULT		(-1440)
#define SST_GAIN_RAMP_DURATION_DEFAULT	5 /* timeconstant */
#define SST_GAIN_MUTE_DEFAULT		true

#define SST_GAIN_KCONTROL_TLV(xname, xhandler_get, xhandler_put, \
			      xmod, xpipe, xinstance, xtask, tlv_array, xgain_val, \
			      xmin, xmax) \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | \
		  SNDRV_CTL_ELEM_ACCESS_READWRITE, \
	.tlv.p = (tlv_array), \
	.info = sst_gain_ctl_info,\
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct sst_gain_mixer_control) \
	{ .stereo = true, .max = xmax, .min = xmin, .type = SST_GAIN_TLV, \
	  .module_id = xmod, .pipe_id = xpipe, .task_id = xtask,\
	  .instance_id = xinstance, .gain_val = xgain_val }

#define SST_GAIN_KCONTROL_INT(xname, xhandler_get, xhandler_put, \
			      xmod, xpipe, xinstance, xtask, xtype, xgain_val, \
			      xmin, xmax) \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = sst_gain_ctl_info, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct sst_gain_mixer_control) \
	{ .stereo = false, .max = xmax, .min = xmin, .type = xtype, \
	  .module_id = xmod, .pipe_id = xpipe, .task_id = xtask,\
	  .instance_id = xinstance, .gain_val = xgain_val }

#define SST_GAIN_KCONTROL_BOOL(xname, xhandler_get, xhandler_put,\
			       xmod, xpipe, xinstance, xtask, xgain_val) \
	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_bool_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&(struct sst_gain_mixer_control) \
	{ .stereo = false, .type = SST_GAIN_MUTE, \
	  .module_id = xmod, .pipe_id = xpipe, .task_id = xtask,\
	  .instance_id = xinstance, .gain_val = xgain_val }

#define SST_CONTROL_NAME(xpname, xinstance, xtype) \
	xpname " " "gain" " " #xinstance " " xtype

/*
 * 3 Controls for each Gain module
 * e.g.	- pcm0_in gain 0 volume
 *	- pcm0_in gain 0 rampduration
 *	- pcm0_in gain 0 mute
 */
#define SST_GAIN_KCONTROLS(xpname, xmin_gain, xmax_gain, xmin_tc, xmax_tc, \
			   xhandler_get, xhandler_put, \
			   xmod, xpipe, xinstance, xtask, tlv_array, xgain_val) \
	{ SST_GAIN_KCONTROL_INT(SST_CONTROL_NAME(xpname, xinstance, "rampduration"), \
		xhandler_get, xhandler_put, xmod, xpipe, xinstance, xtask, SST_GAIN_RAMP_DURATION, \
		xgain_val, xmin_tc, xmax_tc) }, \
	{ SST_GAIN_KCONTROL_BOOL(SST_CONTROL_NAME(xpname, xinstance, "mute"), \
		xhandler_get, xhandler_put, xmod, xpipe, xinstance, xtask, \
		xgain_val) } ,\
	{ SST_GAIN_KCONTROL_TLV(SST_CONTROL_NAME(xpname, xinstance, "volume"), \
		xhandler_get, xhandler_put, xmod, xpipe, xinstance, xtask, tlv_array, \
		xgain_val, xmin_gain, xmax_gain) }

#define SST_GAIN_TC_MIN		5
#define SST_GAIN_TC_MAX		5000
#define SST_GAIN_MIN_VALUE	-1440 /* in 0.1 DB units */
#define SST_GAIN_MAX_VALUE	360

#endif
