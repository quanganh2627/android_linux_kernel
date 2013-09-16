/* i915_debugfs.h -- Private header for the I915 driver debugfs interface -*-
 */
/*
 * Copyright 2013 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author:
 * Deepak S <deepak.s@intel.com>
 */

#ifndef _I915_DEBUGFS_H_
#define _I915_DEBUGFS_H_

#define MAX_BUFFER_STR_LEN	200

#define READ_TOKEN	        "READ"
#define WRITE_TOKEN	        "WRITE"

#define IOSF_FUSE_TOKEN	"FUSE"
#define IOSF_PUNIT_TOKEN	"PUNIT"

/* DebugFS Variable declaration */
struct debugfs_mmio_vars {
	char mmio_vars[MAX_BUFFER_STR_LEN];
	u32 mmio_input;
};

struct debugfs_iosf_vars {
	char iosf_vars[MAX_BUFFER_STR_LEN];
	u32 iosf_input;
};

union {
	struct debugfs_mmio_vars mmio;
	struct debugfs_iosf_vars iosf;
} i915_debugfs_vars;

enum {
	ACTIVE_LIST,
	INACTIVE_LIST,
	PINNED_LIST,
};

#endif /* _I915_DEBUGFS_H_ */
