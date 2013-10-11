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
#include "../platform_ipc_v2.h"
#include "../sst_platform.h"
#include "../sst_platform_pvt.h"
#include "controls_v2.h"
#include "sst_widgets.h"

static inline void sst_fill_byte_control(char *param,
					 u8 ipc_msg, u8 block,
					 u8 task_id, u8 pipe_id,
					 u16 len, void *cmd_data)
{

	struct snd_sst_bytes_v2 *byte_data = (struct snd_sst_bytes_v2 *)param;
	byte_data->type = SST_CMD_BYTES_SET;
	byte_data->ipc_msg = ipc_msg;
	byte_data->block = block;
	byte_data->task_id = task_id;
	byte_data->pipe_id = pipe_id;

	if (len > SST_MAX_BIN_BYTES - sizeof(*byte_data)) {
		pr_err("%s: command length too big (%u)", __func__, len);
		len = SST_MAX_BIN_BYTES - sizeof(*byte_data);
		WARN_ON(1); /* this happens only if code is wrong */
	}
	byte_data->len = len;
	memcpy(byte_data->bytes, cmd_data, len);
}

static int sst_fill_and_send_cmd(struct sst_data *sst,
				 u8 ipc_msg, u8 block, u8 task_id, u8 pipe_id,
				 void *cmd_data, u16 len)
{
	int ret = 0;

	mutex_lock(&sst->lock);
	sst_fill_byte_control(sst->byte_stream, ipc_msg, block, task_id, pipe_id,
			      len, cmd_data);
	ret = sst_dsp->ops->set_generic_params(SST_SET_BYTE_STREAM,
					       sst->byte_stream);
	mutex_unlock(&sst->lock);

	print_hex_dump_bytes("", DUMP_PREFIX_OFFSET, cmd_data, len);
	return ret;
}

static int sst_vb_trigger_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct sst_cmd_generic cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);
	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.header.command_id = SBA_VB_START;
	else
		cmd.header.command_id = SBA_IDLE;

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	cmd.header.length = 0;

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
	return 0;
}

static int sst_ssp_event(struct snd_soc_dapm_widget *w,
			 struct snd_kcontrol *k, int event)
{
	struct sst_cmd_sba_hw_set_ssp cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);

	SST_FILL_DEFAULT_DESTINATION(cmd.header.dst);
	cmd.header.command_id = SBA_HW_SET_SSP;
	cmd.header.length = sizeof(struct sst_cmd_sba_hw_set_ssp)
				- sizeof(struct sst_dsp_header);

	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.switch_state = SST_SWITCH_ON;
	else
		cmd.switch_state = SST_SWITCH_OFF;

	/* TODO: allow to be modified */
	cmd.selection = ids->ssp_id;
	cmd.nb_bits_per_slots = 24;
	cmd.nb_slots = 4;
	cmd.mode = 2;
	cmd.duplex = 0;
	cmd.active_tx_slot_map = 0xF;
	cmd.active_rx_slot_map = 0xF;
	cmd.frame_sync_frequency = 3;
	cmd.frame_sync_polarity = 1;
	cmd.data_polarity = 1;
	cmd.frame_sync_width = 1;
	cmd.ssp_protocol = 0;
	cmd.start_delay = 0;

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
	return 0;
}

static int sst_set_media_path(struct snd_soc_dapm_widget *w,
			      struct snd_kcontrol *k, int event)
{
	struct sst_cmd_set_media_path cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;

	pr_debug("%s: widget=%s\n", __func__, w->name);
	pr_debug("%s: task=%u, location=%#x\n", __func__,
				ids->task_id, ids->location_id);

	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.switch_state = SST_PATH_ON;
	else
		cmd.switch_state = SST_PATH_OFF;

	SST_FILL_DESTINATION(2, cmd.header.dst,
			     ids->location_id, SST_DEFAULT_MODULE_ID);

	/* MMX_SET_MEDIA_PATH == SBA_SET_MEDIA_PATH */
	cmd.header.command_id = MMX_SET_MEDIA_PATH;
	cmd.header.length = sizeof(struct sst_cmd_set_media_path)
				- sizeof(struct sst_dsp_header);

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      ids->task_id, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
	return 0;
}

static int sst_set_media_loop(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct sst_cmd_sba_set_media_loop_map cmd;
	struct sst_data *sst = snd_soc_platform_get_drvdata(w->platform);
	struct sst_ids *ids = w->priv;

	pr_debug("Enter:%s, widget=%s\n", __func__, w->name);
	if (SND_SOC_DAPM_EVENT_ON(event))
		cmd.switch_state = SST_SWITCH_ON;
	else
		cmd.switch_state = SST_SWITCH_OFF;

	SST_FILL_DESTINATION(2, cmd.header.dst,
			     ids->location_id, SST_DEFAULT_MODULE_ID);

	cmd.header.command_id = SBA_SET_MEDIA_LOOP_MAP;
	cmd.header.length = sizeof(struct sst_cmd_sba_set_media_loop_map)
				 - sizeof(struct sst_dsp_header);
	cmd.param.part.rate = 2; /* 48khz */
	cmd.param.part.format = 3; /* stereo */
	cmd.param.part.sample_length = 1; /* 24bit left justified*/
	cmd.map = 0; /* Algo sequence: Gain - DRP - FIR - IIR  */

	sst_fill_and_send_cmd(sst, SST_IPC_IA_CMD, SST_FLAG_BLOCKED,
			      SST_TASK_SBA, 0, &cmd,
			      sizeof(cmd.header) + cmd.header.length);
	return 0;
}

static const struct snd_soc_dapm_widget sst_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("tone"),
	SND_SOC_DAPM_OUTPUT("aware"),
	SND_SOC_DAPM_OUTPUT("vad"),
	SST_SSP_AIF_IN("modem_in", SSP_MODEM, sst_ssp_event),
	SST_SSP_AIF_IN("codec_in0", SSP_CODEC, sst_ssp_event),
	SST_SSP_AIF_IN("codec_in1", SSP_CODEC, sst_ssp_event),
	SST_SSP_AIF_IN("bt_fm_in", SSP_FM, sst_ssp_event),
	SST_SSP_AIF_OUT("modem_out", SSP_MODEM, sst_ssp_event),
	SST_SSP_AIF_OUT("codec_out0", SSP_CODEC, sst_ssp_event),
	SST_SSP_AIF_OUT("codec_out1", SSP_CODEC, sst_ssp_event),
	SST_SSP_AIF_OUT("bt_fm_out", SSP_FM, sst_ssp_event),

	/* Media Paths */
	/* MediaX IN paths are set via ALLOC, so no SET_MEDIA_PATH command */
	SST_PATH_INPUT("media0_in", SST_TASK_MMX, SST_SWM_IN_MEDIA0, NULL),
	SST_PATH_INPUT("media1_in", SST_TASK_MMX, SST_SWM_IN_MEDIA1, NULL),
	SST_PATH_INPUT("media2_in", SST_TASK_MMX, SST_SWM_IN_MEDIA2, sst_set_media_path),
	SST_PATH_INPUT("media3_in", SST_TASK_MMX, SST_SWM_IN_MEDIA3, NULL),
	SST_PATH_OUTPUT("media0_out", SST_TASK_MMX, SST_SWM_OUT_MEDIA0, sst_set_media_path),
	SST_PATH_OUTPUT("media1_out", SST_TASK_MMX, SST_SWM_OUT_MEDIA1, sst_set_media_path),

	/* SBA PCM Paths */
	SST_PATH_INPUT("pcm0_in", SST_TASK_SBA, SST_SWM_IN_PCM0, sst_set_media_path),
	SST_PATH_INPUT("pcm1_in", SST_TASK_SBA, SST_SWM_IN_PCM1, sst_set_media_path),
	SST_PATH_OUTPUT("pcm0_out", SST_TASK_SBA, SST_SWM_OUT_PCM0, sst_set_media_path),
	SST_PATH_OUTPUT("pcm1_out", SST_TASK_SBA, SST_SWM_OUT_PCM1, sst_set_media_path),
	SST_PATH_OUTPUT("pcm2_out", SST_TASK_SBA, SST_SWM_OUT_PCM2, sst_set_media_path),
	/* TODO: check if this needs SET_MEDIA_PATH command*/
	SST_PATH_INPUT("low_pcm0_in", SST_TASK_SBA, SST_SWM_IN_LOW_PCM0, NULL),

	SST_PATH_INPUT("voip_in", SST_TASK_SBA, SST_SWM_IN_VOIP, sst_set_media_path),
	SST_PATH_OUTPUT("voip_out", SST_TASK_SBA, SST_SWM_OUT_VOIP, sst_set_media_path),
	SST_PATH_OUTPUT("aware_out", SST_TASK_SBA, SST_SWM_OUT_AWARE, sst_set_media_path),
	SST_PATH_OUTPUT("vad_out", SST_TASK_SBA, SST_SWM_OUT_VAD, sst_set_media_path),

	/* SBA Loops */
	SST_PATH_INPUT("sprot_loop_in", SST_TASK_SBA, SST_SWM_IN_SPROT_LOOP, NULL),
	SST_PATH_INPUT("media_loop1_in", SST_TASK_SBA, SST_SWM_IN_MEDIA_LOOP1, NULL),
	SST_PATH_INPUT("media_loop2_in", SST_TASK_SBA, SST_SWM_IN_MEDIA_LOOP2, NULL),
	SST_PATH_OUTPUT("sprot_loop_out", SST_TASK_SBA, SST_SWM_OUT_SPROT_LOOP, sst_set_media_loop),
	SST_PATH_OUTPUT("media_loop1_out", SST_TASK_SBA, SST_SWM_OUT_MEDIA_LOOP1, sst_set_media_loop),
	SST_PATH_OUTPUT("media_loop2_out", SST_TASK_SBA, SST_SWM_OUT_MEDIA_LOOP2, sst_set_media_loop),

	/* TODO: need to send command */
	SST_PATH_INPUT("sidetone_in", SST_TASK_SBA, SST_SWM_IN_SIDETONE, NULL),
	SST_PATH_INPUT("tone_in", SST_TASK_SBA, SST_SWM_IN_TONE, NULL),
	SST_PATH_INPUT("bt_in", SST_TASK_SBA, SST_SWM_IN_BT, NULL),
	SST_PATH_INPUT("fm_in", SST_TASK_SBA, SST_SWM_IN_FM, NULL),
	SST_PATH_OUTPUT("bt_out", SST_TASK_SBA, SST_SWM_OUT_BT, NULL),
	SST_PATH_OUTPUT("fm_out", SST_TASK_SBA, SST_SWM_OUT_FM, NULL),

	/* SBA Voice Paths */
	SST_PATH_INPUT("speech_in", SST_TASK_SBA, SST_SWM_IN_SPEECH, NULL),
	SST_PATH_INPUT("txspeech_in", SST_TASK_SBA, SST_SWM_IN_TXSPEECH, NULL),
	SST_PATH_OUTPUT("hf_sns_out", SST_TASK_SBA, SST_SWM_OUT_HF_SNS, NULL),
	SST_PATH_OUTPUT("hf_out", SST_TASK_SBA, SST_SWM_OUT_HF, NULL),
	SST_PATH_OUTPUT("speech_out", SST_TASK_SBA, SST_SWM_OUT_SPEECH, NULL),
	SST_PATH_OUTPUT("rxspeech_out", SST_TASK_SBA, SST_SWM_OUT_RXSPEECH, NULL),

	SND_SOC_DAPM_SUPPLY("VBTimer", SND_SOC_NOPM, 0, 0,
			    sst_vb_trigger_event,
			    SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

static const struct snd_soc_dapm_route intercon[] = {
};

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

	snd_soc_dapm_new_controls(&platform->dapm, sst_dapm_widgets,
			ARRAY_SIZE(sst_dapm_widgets));
	snd_soc_dapm_add_routes(&platform->dapm, intercon,
			ARRAY_SIZE(intercon));
	snd_soc_dapm_new_widgets(&platform->dapm);

	return 0;
}
