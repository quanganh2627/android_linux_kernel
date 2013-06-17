/*
 * Copyright  2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *	Adam Rutkowski <adam.j.rutkowski@intel.com>
 */
#include "drmP.h"
#include "drm.h"
#include "drm_crtc_helper.h"
#include "drm_fb_helper.h"
#include "intel_drv.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "i915_trace.h"
#include "linux/wait.h"
#include "i915_perfmon.h"

/**
 * i915_perfmon_set_rc6 - set RC6 state
 *
 * Enable and re-enable RC6 on demand in runtime.
 */
int i915_perfmon_set_rc6(struct drm_device *dev, __u32 enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!IS_VALLEYVIEW(dev))
		return -EINVAL;

	if (i915_enable_rc6 >= 0) {
		int rc6_module_setting =
			((i915_enable_rc6 & INTEL_RC6_ENABLE) != 0);
		int rc6_enable = (enable != 0);
		if (rc6_module_setting == rc6_enable)
			return 0;
		else
			return -EINVAL;
	}

	mutex_lock(&dev->struct_mutex);
	if (enable && dev_priv->perfmon.rc6_user_disable_count == 0) {
		mutex_unlock(&dev->struct_mutex);
		return -EINVAL;
	}
	if (!enable) {
		if (dev_priv->perfmon.rc6_user_disable_count++ == 0)
			vlv_rs_setstate(dev, false);
	} else {
		if (--dev_priv->perfmon.rc6_user_disable_count == 0)
			vlv_rs_setstate(dev, true);
	}
	mutex_unlock(&dev->struct_mutex);

	return 0;
}
/**
 * intel_set_max_freq - enable max GPU frequency override
 *
 * Overrides turbo algorithm to switch GPU to maximum
 * frequency.
 */
int intel_set_max_freq(struct drm_device *dev, int enable)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!IS_VALLEYVIEW(dev))
		return -EINVAL;

	mutex_lock(&dev->struct_mutex);
	mutex_lock(&dev_priv->rps.hw_lock);
	if (enable) {
		dev_priv->perfmon.max_frequency_mode = true;
		vlv_turbo_disable(dev);
		valleyview_set_rps(dev, dev_priv->rps.max_delay);
	} else {
		dev_priv->perfmon.max_frequency_mode = false;
		vlv_turbo_initialize(dev);
	}
	mutex_unlock(&dev_priv->rps.hw_lock);
	mutex_unlock(&dev->struct_mutex);

	return 0;
}

/**
 * valleyview_enable_perfmon_interrupt - enable perfmon interrupt
 *
 */
static int valleyview_enable_perfmon_interrupt(struct drm_device *dev,
						int enable)
{
	drm_i915_private_t *dev_priv = (drm_i915_private_t *) dev->dev_private;
	unsigned long irqflags;
	u32 imr;

	spin_lock_irqsave(&dev_priv->irq_lock, irqflags);
	imr = I915_READ(GTIMR);
	if (enable) {
		dev_priv->perfmon.interrupt_enabled = true;
		dev_priv->gt_irq_mask &= ~GT_RENDER_PERFMON_BUFFER_INTERRUPT;
		imr &= ~GT_RENDER_PERFMON_BUFFER_INTERRUPT;
	} else {
		dev_priv->perfmon.interrupt_enabled = false;
		dev_priv->gt_irq_mask |= GT_RENDER_PERFMON_BUFFER_INTERRUPT;
		imr |= GT_RENDER_PERFMON_BUFFER_INTERRUPT;
	}
	I915_WRITE(GTIMR, imr);
	spin_unlock_irqrestore(&dev_priv->irq_lock, irqflags);

	return 0;
}

/**
 * valleyview_wait_perfmon_interrupt - wait for perfmon buffer interrupt
 *
 * Blocks until perfmon buffer half full interrupt occurs or the wait
 * times out.
 */
static int valleyview_wait_perfmon_interrupt(struct drm_device *dev,
						int timeout_ms)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	int counter = atomic_read(&dev_priv->perfmon.buffer_interrupts);
	int retcode = I915_PERFMON_IRQ_WAIT_OK;
	int time_left = 0;

	time_left = wait_event_interruptible_timeout(
		dev_priv->perfmon.buffer_queue,
		atomic_read(&dev_priv->perfmon.buffer_interrupts) != counter,
		timeout_ms * HZ / 1000);

	if (time_left == 0)
		retcode = I915_PERFMON_IRQ_WAIT_TIMEOUT;
	else if (time_left == -ERESTARTSYS)
		retcode = I915_PERFMON_IRQ_WAIT_INTERRUPTED;
	else if (time_left < 0)
		retcode = I915_PERFMON_IRQ_WAIT_FAILED;

	return retcode;
}

/**
 * i915_perfmon_ioctl - performance monitoring support
 *
 * Main entry point to performance monitoring support
 * IOCTLs.
 */
int i915_perfmon_ioctl(struct drm_device *dev, void *data,
	struct drm_file *file)
{
	struct drm_i915_perfmon *perfmon = data;
	int retcode = 0;

	switch (perfmon->op) {
	case I915_PERFMON_SET_RC6:
		retcode = i915_perfmon_set_rc6(dev,
			perfmon->data.set_rc6.enable);
		break;
	case I915_PERFMON_SET_MAX_FREQ:
		retcode = intel_set_max_freq(dev,
			perfmon->data.set_max_freq.enable);
		break;
	case I915_PERFMON_SET_BUFFER_IRQS:
		retcode = valleyview_enable_perfmon_interrupt(
				dev,
				perfmon->data.set_irqs.enable);
		break;
	case I915_PERFMON_WAIT_BUFFER_IRQS:
		if (perfmon->data.wait_irqs.timeout >
				I915_PERFMON_WAIT_IRQ_MAX_TIMEOUT_MS)
			retcode =  -EINVAL;
		else
			perfmon->data.wait_irqs.ret_code =
				valleyview_wait_perfmon_interrupt(
					dev,
					perfmon->data.wait_irqs.timeout);
		break;
	case I915_PERFMON_GET_FREQ_INFO:
		/* obsolete - sysfs used instead */
		retcode = -EINVAL;
		break;
	default:
		/* unknown operation */
		retcode = -EINVAL;
		break;
	}

	return retcode;
}
