/*
 *
 * Copyright 2008 (c) Intel Corporation
 *   Jesse Barnes <jbarnes@virtuousgeek.org>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL TUNGSTEN GRAPHICS AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <drm/drmP.h>
#include <drm/i915_drm.h>
#include "intel_drv.h"
#include "i915_reg.h"
#include <linux/console.h>
#include <linux/intel_mid_pm.h>

static u8 i915_read_indexed(struct drm_device *dev, u16 index_port, u16 data_port, u8 reg)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE8(index_port, reg);
	return I915_READ8(data_port);
}

static u8 i915_read_ar(struct drm_device *dev, u16 st01, u8 reg, u16 palette_enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_READ8(st01);
	I915_WRITE8(VGA_AR_INDEX, palette_enable | reg);
	return I915_READ8(VGA_AR_DATA_READ);
}

static void i915_write_ar(struct drm_device *dev, u16 st01, u8 reg, u8 val, u16 palette_enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_READ8(st01);
	I915_WRITE8(VGA_AR_INDEX, palette_enable | reg);
	I915_WRITE8(VGA_AR_DATA_WRITE, val);
}

static void i915_write_indexed(struct drm_device *dev, u16 index_port, u16 data_port, u8 reg, u8 val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	I915_WRITE8(index_port, reg);
	I915_WRITE8(data_port, val);
}

static void i915_save_vga(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;
	u16 cr_index, cr_data, st01;

	/* VGA state */
	dev_priv->regfile.saveVGA0 = I915_READ(VGA0);
	dev_priv->regfile.saveVGA1 = I915_READ(VGA1);
	dev_priv->regfile.saveVGA_PD = I915_READ(VGA_PD);
	dev_priv->regfile.saveVGACNTRL = I915_READ(i915_vgacntrl_reg(dev));

	/* VGA color palette registers */
	dev_priv->regfile.saveDACMASK = I915_READ8(VGA_DACMASK);

	/* MSR bits */
	dev_priv->regfile.saveMSR = I915_READ8(VGA_MSR_READ);
	if (dev_priv->regfile.saveMSR & VGA_MSR_CGA_MODE) {
		cr_index = VGA_CR_INDEX_CGA;
		cr_data = VGA_CR_DATA_CGA;
		st01 = VGA_ST01_CGA;
	} else {
		cr_index = VGA_CR_INDEX_MDA;
		cr_data = VGA_CR_DATA_MDA;
		st01 = VGA_ST01_MDA;
	}

	/* CRT controller regs */
	i915_write_indexed(dev, cr_index, cr_data, 0x11,
			   i915_read_indexed(dev, cr_index, cr_data, 0x11) &
			   (~0x80));
	for (i = 0; i <= 0x24; i++)
		dev_priv->regfile.saveCR[i] =
			i915_read_indexed(dev, cr_index, cr_data, i);
	/* Make sure we don't turn off CR group 0 writes */
	dev_priv->regfile.saveCR[0x11] &= ~0x80;

	/* Attribute controller registers */
	I915_READ8(st01);
	dev_priv->regfile.saveAR_INDEX = I915_READ8(VGA_AR_INDEX);
	for (i = 0; i <= 0x14; i++)
		dev_priv->regfile.saveAR[i] = i915_read_ar(dev, st01, i, 0);
	I915_READ8(st01);
	I915_WRITE8(VGA_AR_INDEX, dev_priv->regfile.saveAR_INDEX);
	I915_READ8(st01);

	/* Graphics controller registers */
	for (i = 0; i < 9; i++)
		dev_priv->regfile.saveGR[i] =
			i915_read_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, i);

	dev_priv->regfile.saveGR[0x10] =
		i915_read_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, 0x10);
	dev_priv->regfile.saveGR[0x11] =
		i915_read_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, 0x11);
	dev_priv->regfile.saveGR[0x18] =
		i915_read_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, 0x18);

	/* Sequencer registers */
	for (i = 0; i < 8; i++)
		dev_priv->regfile.saveSR[i] =
			i915_read_indexed(dev, VGA_SR_INDEX, VGA_SR_DATA, i);
}

static void i915_restore_vga(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;
	u16 cr_index, cr_data, st01;

	/* VGA state */
	I915_WRITE(i915_vgacntrl_reg(dev), dev_priv->regfile.saveVGACNTRL);

	I915_WRITE(VGA0, dev_priv->regfile.saveVGA0);
	I915_WRITE(VGA1, dev_priv->regfile.saveVGA1);
	I915_WRITE(VGA_PD, dev_priv->regfile.saveVGA_PD);
	POSTING_READ(VGA_PD);
	udelay(150);

	/* MSR bits */
	I915_WRITE8(VGA_MSR_WRITE, dev_priv->regfile.saveMSR);
	if (dev_priv->regfile.saveMSR & VGA_MSR_CGA_MODE) {
		cr_index = VGA_CR_INDEX_CGA;
		cr_data = VGA_CR_DATA_CGA;
		st01 = VGA_ST01_CGA;
	} else {
		cr_index = VGA_CR_INDEX_MDA;
		cr_data = VGA_CR_DATA_MDA;
		st01 = VGA_ST01_MDA;
	}

	/* Sequencer registers, don't write SR07 */
	for (i = 0; i < 7; i++)
		i915_write_indexed(dev, VGA_SR_INDEX, VGA_SR_DATA, i,
				   dev_priv->regfile.saveSR[i]);

	/* CRT controller regs */
	/* Enable CR group 0 writes */
	i915_write_indexed(dev, cr_index, cr_data, 0x11, dev_priv->regfile.saveCR[0x11]);
	for (i = 0; i <= 0x24; i++)
		i915_write_indexed(dev, cr_index, cr_data, i, dev_priv->regfile.saveCR[i]);

	/* Graphics controller regs */
	for (i = 0; i < 9; i++)
		i915_write_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, i,
				   dev_priv->regfile.saveGR[i]);

	i915_write_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, 0x10,
			   dev_priv->regfile.saveGR[0x10]);
	i915_write_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, 0x11,
			   dev_priv->regfile.saveGR[0x11]);
	i915_write_indexed(dev, VGA_GR_INDEX, VGA_GR_DATA, 0x18,
			   dev_priv->regfile.saveGR[0x18]);

	/* Attribute controller registers */
	I915_READ8(st01); /* switch back to index mode */
	for (i = 0; i <= 0x14; i++)
		i915_write_ar(dev, st01, i, dev_priv->regfile.saveAR[i], 0);
	I915_READ8(st01); /* switch back to index mode */
	I915_WRITE8(VGA_AR_INDEX, dev_priv->regfile.saveAR_INDEX | 0x20);
	I915_READ8(st01);

	/* VGA color palette registers */
	I915_WRITE8(VGA_DACMASK, dev_priv->regfile.saveDACMASK);
}

static void i915_save_display(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	unsigned long flags;

	/* Display arbitration control */
	if (INTEL_INFO(dev)->gen <= 4)
		dev_priv->regfile.saveDSPARB = I915_READ(DSPARB);

	/* This is only meaningful in non-KMS mode */
	/* Don't regfile.save them in KMS mode */
	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		i915_save_display_reg(dev);

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	/* LVDS state */
	if (HAS_PCH_SPLIT(dev)) {
		dev_priv->regfile.savePP_CONTROL = I915_READ(PCH_PP_CONTROL);
		dev_priv->regfile.saveBLC_PWM_CTL = I915_READ(BLC_PWM_PCH_CTL1);
		dev_priv->regfile.saveBLC_PWM_CTL2 = I915_READ(BLC_PWM_PCH_CTL2);
		dev_priv->regfile.saveBLC_CPU_PWM_CTL = I915_READ(BLC_PWM_CPU_CTL);
		dev_priv->regfile.saveBLC_CPU_PWM_CTL2 = I915_READ(BLC_PWM_CPU_CTL2);
		if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev))
			dev_priv->regfile.saveLVDS = I915_READ(PCH_LVDS);
	} else {
		dev_priv->regfile.savePP_CONTROL = I915_READ(PP_CONTROL);
		dev_priv->regfile.savePFIT_PGM_RATIOS = I915_READ(PFIT_PGM_RATIOS);
		dev_priv->regfile.saveBLC_PWM_CTL = I915_READ(BLC_PWM_CTL);
		dev_priv->regfile.saveBLC_HIST_CTL = I915_READ(BLC_HIST_CTL);
		if (INTEL_INFO(dev)->gen >= 4)
			dev_priv->regfile.saveBLC_PWM_CTL2 = I915_READ(BLC_PWM_CTL2);
		if (IS_MOBILE(dev) && !IS_I830(dev))
			dev_priv->regfile.saveLVDS = I915_READ(LVDS);
	}

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	if (!IS_I830(dev) && !IS_845G(dev) && !HAS_PCH_SPLIT(dev))
		dev_priv->regfile.savePFIT_CONTROL = I915_READ(PFIT_CONTROL);

	if (HAS_PCH_SPLIT(dev)) {
		dev_priv->regfile.savePP_ON_DELAYS = I915_READ(PCH_PP_ON_DELAYS);
		dev_priv->regfile.savePP_OFF_DELAYS = I915_READ(PCH_PP_OFF_DELAYS);
		dev_priv->regfile.savePP_DIVISOR = I915_READ(PCH_PP_DIVISOR);
	} else {
		dev_priv->regfile.savePP_ON_DELAYS = I915_READ(PP_ON_DELAYS);
		dev_priv->regfile.savePP_OFF_DELAYS = I915_READ(PP_OFF_DELAYS);
		dev_priv->regfile.savePP_DIVISOR = I915_READ(PP_DIVISOR);
	}

	/* Only regfile.save FBC state on the platform that supports FBC */
	if (I915_HAS_FBC(dev)) {
		if (HAS_PCH_SPLIT(dev)) {
			dev_priv->regfile.saveDPFC_CB_BASE = I915_READ(ILK_DPFC_CB_BASE);
		} else if (IS_GM45(dev)) {
			dev_priv->regfile.saveDPFC_CB_BASE = I915_READ(DPFC_CB_BASE);
		} else {
			dev_priv->regfile.saveFBC_CFB_BASE = I915_READ(FBC_CFB_BASE);
			dev_priv->regfile.saveFBC_LL_BASE = I915_READ(FBC_LL_BASE);
			dev_priv->regfile.saveFBC_CONTROL2 = I915_READ(FBC_CONTROL2);
			dev_priv->regfile.saveFBC_CONTROL = I915_READ(FBC_CONTROL);
		}
	}

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		i915_save_vga(dev);
}

static void i915_restore_display(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 mask = 0xffffffff;
	unsigned long flags;

	/* Display arbitration */
	if (INTEL_INFO(dev)->gen <= 4)
		I915_WRITE(DSPARB, dev_priv->regfile.saveDSPARB);

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		i915_restore_display_reg(dev);

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	/* LVDS state */
	if (INTEL_INFO(dev)->gen >= 4 && !HAS_PCH_SPLIT(dev))
		I915_WRITE(BLC_PWM_CTL2, dev_priv->regfile.saveBLC_PWM_CTL2);

	if (drm_core_check_feature(dev, DRIVER_MODESET))
		mask = ~LVDS_PORT_EN;

	if (HAS_PCH_IBX(dev) || HAS_PCH_CPT(dev))
		I915_WRITE(PCH_LVDS, dev_priv->regfile.saveLVDS & mask);
	else if (INTEL_INFO(dev)->gen <= 4 && IS_MOBILE(dev) && !IS_I830(dev))
		I915_WRITE(LVDS, dev_priv->regfile.saveLVDS & mask);

	if (!IS_I830(dev) && !IS_845G(dev) && !HAS_PCH_SPLIT(dev))
		I915_WRITE(PFIT_CONTROL, dev_priv->regfile.savePFIT_CONTROL);

	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(BLC_PWM_PCH_CTL1, dev_priv->regfile.saveBLC_PWM_CTL);
		I915_WRITE(BLC_PWM_PCH_CTL2, dev_priv->regfile.saveBLC_PWM_CTL2);
		/* NOTE: BLC_PWM_CPU_CTL must be written after BLC_PWM_CPU_CTL2;
		 * otherwise we get blank eDP screen after S3 on some machines
		 */
		I915_WRITE(BLC_PWM_CPU_CTL2, dev_priv->regfile.saveBLC_CPU_PWM_CTL2);
		I915_WRITE(BLC_PWM_CPU_CTL, dev_priv->regfile.saveBLC_CPU_PWM_CTL);
		I915_WRITE(PCH_PP_ON_DELAYS, dev_priv->regfile.savePP_ON_DELAYS);
		I915_WRITE(PCH_PP_OFF_DELAYS, dev_priv->regfile.savePP_OFF_DELAYS);
		I915_WRITE(PCH_PP_DIVISOR, dev_priv->regfile.savePP_DIVISOR);
		I915_WRITE(PCH_PP_CONTROL, dev_priv->regfile.savePP_CONTROL);
		I915_WRITE(RSTDBYCTL,
			   dev_priv->regfile.saveMCHBAR_RENDER_STANDBY);
	} else {
		I915_WRITE(PFIT_PGM_RATIOS, dev_priv->regfile.savePFIT_PGM_RATIOS);
		I915_WRITE(BLC_PWM_CTL, dev_priv->regfile.saveBLC_PWM_CTL);
		I915_WRITE(BLC_HIST_CTL, dev_priv->regfile.saveBLC_HIST_CTL);
		I915_WRITE(PP_ON_DELAYS, dev_priv->regfile.savePP_ON_DELAYS);
		I915_WRITE(PP_OFF_DELAYS, dev_priv->regfile.savePP_OFF_DELAYS);
		I915_WRITE(PP_DIVISOR, dev_priv->regfile.savePP_DIVISOR);
		I915_WRITE(PP_CONTROL, dev_priv->regfile.savePP_CONTROL);
	}

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	/* only restore FBC info on the platform that supports FBC*/
	intel_disable_fbc(dev);
	if (I915_HAS_FBC(dev)) {
		if (HAS_PCH_SPLIT(dev)) {
			I915_WRITE(ILK_DPFC_CB_BASE, dev_priv->regfile.saveDPFC_CB_BASE);
		} else if (IS_GM45(dev)) {
			I915_WRITE(DPFC_CB_BASE, dev_priv->regfile.saveDPFC_CB_BASE);
		} else {
			I915_WRITE(FBC_CFB_BASE, dev_priv->regfile.saveFBC_CFB_BASE);
			I915_WRITE(FBC_LL_BASE, dev_priv->regfile.saveFBC_LL_BASE);
			I915_WRITE(FBC_CONTROL2, dev_priv->regfile.saveFBC_CONTROL2);
			I915_WRITE(FBC_CONTROL, dev_priv->regfile.saveFBC_CONTROL);
		}
	}

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		i915_restore_vga(dev);
	else
		i915_redisable_vga(dev);
}

int i915_save_state(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;

	pci_read_config_byte(dev->pdev, LBB, &dev_priv->regfile.saveLBB);

	mutex_lock(&dev->struct_mutex);

	i915_save_display(dev);

	if (!drm_core_check_feature(dev, DRIVER_MODESET)) {
		/* Interrupt state */
		if (HAS_PCH_SPLIT(dev)) {
			dev_priv->regfile.saveDEIER = I915_READ(DEIER);
			dev_priv->regfile.saveDEIMR = I915_READ(DEIMR);
			dev_priv->regfile.saveGTIER = I915_READ(GTIER);
			dev_priv->regfile.saveGTIMR = I915_READ(GTIMR);
			dev_priv->regfile.saveFDI_RXA_IMR = I915_READ(_FDI_RXA_IMR);
			dev_priv->regfile.saveFDI_RXB_IMR = I915_READ(_FDI_RXB_IMR);
			dev_priv->regfile.saveMCHBAR_RENDER_STANDBY =
				I915_READ(RSTDBYCTL);
			dev_priv->regfile.savePCH_PORT_HOTPLUG = I915_READ(PCH_PORT_HOTPLUG);
		} else {
			dev_priv->regfile.saveIER = I915_READ(IER);
			dev_priv->regfile.saveIMR = I915_READ(IMR);
		}
	}

	intel_disable_gt_powersave(dev);

	/* Cache mode state */
	dev_priv->regfile.saveCACHE_MODE_0 = I915_READ(CACHE_MODE_0);

	/* Memory Arbitration state */
	dev_priv->regfile.saveMI_ARB_STATE = I915_READ(MI_ARB_STATE);

	/* Scratch space */
	for (i = 0; i < 16; i++) {
		dev_priv->regfile.saveSWF0[i] = I915_READ(SWF00 + (i << 2));
		dev_priv->regfile.saveSWF1[i] = I915_READ(SWF10 + (i << 2));
	}
	for (i = 0; i < 3; i++)
		dev_priv->regfile.saveSWF2[i] = I915_READ(SWF30 + (i << 2));

	mutex_unlock(&dev->struct_mutex);

	return 0;
}

int i915_restore_state(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int i;

	pci_write_config_byte(dev->pdev, LBB, dev_priv->regfile.saveLBB);

	mutex_lock(&dev->struct_mutex);

	i915_gem_restore_fences(dev);
	i915_restore_display(dev);

	if (!drm_core_check_feature(dev, DRIVER_MODESET)) {
		/* Interrupt state */
		if (HAS_PCH_SPLIT(dev)) {
			I915_WRITE(DEIER, dev_priv->regfile.saveDEIER);
			I915_WRITE(DEIMR, dev_priv->regfile.saveDEIMR);
			I915_WRITE(GTIER, dev_priv->regfile.saveGTIER);
			I915_WRITE(GTIMR, dev_priv->regfile.saveGTIMR);
			I915_WRITE(_FDI_RXA_IMR, dev_priv->regfile.saveFDI_RXA_IMR);
			I915_WRITE(_FDI_RXB_IMR, dev_priv->regfile.saveFDI_RXB_IMR);
			I915_WRITE(PCH_PORT_HOTPLUG, dev_priv->regfile.savePCH_PORT_HOTPLUG);
		} else {
			I915_WRITE(IER, dev_priv->regfile.saveIER);
			I915_WRITE(IMR, dev_priv->regfile.saveIMR);
		}
	}

	/* Cache mode state */
	I915_WRITE(CACHE_MODE_0, dev_priv->regfile.saveCACHE_MODE_0 | 0xffff0000);

	/* Memory arbitration state */
	I915_WRITE(MI_ARB_STATE, dev_priv->regfile.saveMI_ARB_STATE | 0xffff0000);

	for (i = 0; i < 16; i++) {
		I915_WRITE(SWF00 + (i << 2), dev_priv->regfile.saveSWF0[i]);
		I915_WRITE(SWF10 + (i << 2), dev_priv->regfile.saveSWF1[i]);
	}
	for (i = 0; i < 3; i++)
		I915_WRITE(SWF30 + (i << 2), dev_priv->regfile.saveSWF2[i]);

	mutex_unlock(&dev->struct_mutex);

	intel_i2c_reset(dev);

	return 0;
}

static int __i915_drm_freeze(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_crtc *crtc;

	/* ignore lid events during suspend */
	mutex_lock(&dev_priv->modeset_restore_lock);
	dev_priv->modeset_restore = MODESET_SUSPENDED;
	mutex_unlock(&dev_priv->modeset_restore_lock);

	/* We do a lot of poking in a lot of registers, make sure they work
	 * properly. */
	hsw_disable_package_c8(dev_priv);
	intel_set_power_well(dev, true);

	drm_kms_helper_poll_disable(dev);

	pci_save_state(dev->pdev);

	/* If KMS is active, we do the leavevt stuff here */
	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		int error;

		mutex_lock(&dev->struct_mutex);
		error = i915_gem_idle(dev);
		mutex_unlock(&dev->struct_mutex);
		if (error) {
			dev_err(&dev->pdev->dev,
				"GEM idle failed, resume might fail\n");
			return error;
		}

		cancel_delayed_work_sync(&dev_priv->rps.delayed_resume_work);

		drm_irq_uninstall(dev);
		dev_priv->enable_hotplug_processing = false;
		/*
		 * Disable CRTCs directly since we want to preserve sw state
		 * for _thaw.
		 */
		list_for_each_entry(crtc, &dev->mode_config.crtc_list, head)
			dev_priv->display.crtc_disable(crtc);

		intel_modeset_suspend_hw(dev);
	}

	i915_save_state(dev);

	intel_opregion_fini(dev);

	console_lock();
	intel_fbdev_set_suspend(dev, FBINFO_STATE_SUSPENDED);
	console_unlock();

	return 0;
}

static void intel_resume_hotplug(struct drm_device *dev)
{
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct intel_encoder *encoder;

	mutex_lock(&mode_config->mutex);
	DRM_DEBUG_KMS("running encoder hotplug functions\n");

	list_for_each_entry(encoder, &mode_config->encoder_list, base.head)
		if (encoder->hot_plug)
			encoder->hot_plug(encoder);

	mutex_unlock(&mode_config->mutex);

	/* Just fire off a uevent and let userspace tell us what to do */
	drm_helper_hpd_irq_event(dev);
}

static int __i915_drm_thaw(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int error = 0;

	i915_restore_state(dev);
	intel_opregion_setup(dev);

	/* KMS EnterVT equivalent */
	if (drm_core_check_feature(dev, DRIVER_MODESET)) {
		intel_init_pch_refclk(dev);

		mutex_lock(&dev->struct_mutex);

		error = i915_gem_init_hw(dev);
		mutex_unlock(&dev->struct_mutex);

		/* We need working interrupts for modeset enabling ... */
		drm_irq_install(dev);

		intel_modeset_init_hw(dev);

		drm_modeset_lock_all(dev);
		intel_modeset_setup_hw_state(dev, true);
		drm_modeset_unlock_all(dev);

		/*
		 * ... but also need to make sure that hotplug processing
		 * doesn't cause havoc. Like in the driver load code we don't
		 * bother with the tiny race here where we might loose hotplug
		 * notifications.
		 * */
		intel_hpd_init(dev);
		dev_priv->enable_hotplug_processing = true;
		/* Config may have changed between suspend and resume */
		intel_resume_hotplug(dev);
	}

	intel_opregion_init(dev);

	/*
	 * The console lock can be pretty contented on resume due
	 * to all the printk activity.  Try to keep it out of the hot
	 * path of resume if possible.
	 */
	if (console_trylock()) {
		intel_fbdev_set_suspend(dev, FBINFO_STATE_RUNNING);
		console_unlock();
	} else {
		schedule_work(&dev_priv->console_resume_work);
	}

	/* Undo what we did at i915_drm_freeze so the refcount goes back to the
	 * expected level. */
	hsw_enable_package_c8(dev_priv);

	mutex_lock(&dev_priv->modeset_restore_lock);
	dev_priv->modeset_restore = MODESET_DONE;
	mutex_unlock(&dev_priv->modeset_restore_lock);
	return error;
}

void i915_pm_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	dev_priv->pm.drm_freeze = __i915_drm_freeze;
	dev_priv->pm.drm_thaw = __i915_drm_thaw;
}
