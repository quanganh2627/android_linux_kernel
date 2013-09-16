/*
 * Intel mid pstore ram support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/platform_device.h>
#include <linux/memblock.h>
#include <linux/pstore_ram.h>
#include <linux/bootmem.h>

#define SZ_4K	0x00001000
#define SZ_1M	0x00100000
#define SZ_2M	0x00200000
#define SZ_16M	0x01000000

/* Board files use the following if they are ok with 16M start defaults */
#define PSTORE_RAM_START_DEFAULT	SZ_16M
#define PSTORE_RAM_SIZE_DEFAULT		SZ_2M

#ifdef CONFIG_X86_32
#define RAM_MAX_MEM (max_low_pfn << PAGE_SHIFT)
#else
#define RAM_MAX_MEM (1 << 28)
#endif

static struct ramoops_platform_data pstore_ram_data = {
	.mem_size	= PSTORE_RAM_SIZE_DEFAULT,
	.mem_address	= PSTORE_RAM_START_DEFAULT,
	.record_size	= SZ_4K,
	.console_size	= SZ_1M,
	.ftrace_size	= SZ_4K,
	.dump_oops	= 1,
};

static struct platform_device pstore_ram_dev = {
	.name = "ramoops",
	.dev = {
		.platform_data = &pstore_ram_data,
	},
};

static __initdata bool intel_mid_pstore_ram_inited;

/**
 * intel_mid_pstore_ram_register() - device_initcall to register ramoops device
 */
static int __init intel_mid_pstore_ram_register(void)
{
	int ret;

	if (!intel_mid_pstore_ram_inited)
		return -ENODEV;

	ret = platform_device_register(&pstore_ram_dev);
	if (ret) {
		pr_err("%s: unable to register pstore_ram device:", __func__);
		pr_err("start=0x%32x", (u32)pstore_ram_data.mem_address);
		pr_err("size=0x%32x", (u32)pstore_ram_data.mem_size);
		pr_err("ret=%d\n", ret);
	}

	return ret;
}
device_initcall(intel_mid_pstore_ram_register);

void __init pstore_ram_reserve_memory(void)
{
	phys_addr_t mem;
	size_t size;
	int ret;

	size = PSTORE_RAM_SIZE_DEFAULT;
	size = ALIGN(size, PAGE_SIZE);

	mem = memblock_find_in_range(0, RAM_MAX_MEM, size, PAGE_SIZE);
	if (!mem) {
		pr_err("Cannot find memblock range for pstore_ram\n");
		return;
	}

	ret = memblock_reserve(mem, size);
	if (ret) {
		pr_err("Failed to reserve memory from %08lx-%08lx\n",
			(long)mem, (long)(mem + size - 1));
		return;
	}

	pstore_ram_data.mem_address = mem;
	pstore_ram_data.mem_size = size;

	intel_mid_pstore_ram_inited = true;
}
