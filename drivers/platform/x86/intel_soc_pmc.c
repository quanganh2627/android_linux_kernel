/*
 * intel_soc_pmc.c - This driver provides interface to configure the Power
 * Management Controller (PMC).
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/suspend.h>
#include <linux/time.h>

#include <asm/intel_mid_pcihelpers.h>

#define	MAX_PLATFORM_STATES	5
#define BYT_S3_HINT		0x64
#define	S0I3			3

#define	S0IX_REGISTERS_OFFSET	0x80

#define	S0IR_TMR_OFFSET		0x80
#define	S0I1_TMR_OFFSET		0x84
#define	S0I2_TMR_OFFSET		0x88
#define	S0I3_TMR_OFFSET		0x8c
#define	S0_TMR_OFFSET		0x90

#define	S0IX_WAKE_EN		0x3c

#define	PMC_MMIO_BAR		1
#define	BASE_ADDRESS_MASK	0xFFFFFFFE00
#define	DISABLE_LPC_CLK_WAKE_EN 0xffffef

#define PM_SUPPORT		0x21

#define ISP_POS			7
#define ISP_SUB_CLASS		0x80

#define PUNIT_PORT		0x04
#define PWRGT_CNT		0x60
#define PWRGT_STATUS		0x61
#define VED_SS_PM0		0x32
#define ISP_SS_PM0		0x39
#define MIO_SS_PM		0x3B
#define SSS_SHIFT		24
#define RENDER_POS		0
#define MEDIA_POS		2
#define DISPLAY_POS		6

#define MAX_POWER_ISLANDS	16
#define ISLAND_UP		0x0
#define ISLAND_DOWN		0x1
/*Soft reset*/
#define ISLAND_SR		0x2

/* Soft reset mask */
#define SR_MASK			0x2

#define NC_PM_SSS		0x3F

#define GFX_LSS_INDEX		1

#define PMC_D0I0_MASK		0
#define PMC_D0I1_MASK		1
#define PMC_D0I2_MASK		2
#define PMC_D0I3_MASK		3

#define BITS_PER_LSS		2
#define PCI_ID_ANY		(~0)
#define SUB_CLASS_MASK		0xFF00

struct pmc_dev {
	u32 base_address;
	u32 __iomem *pmc_registers;
	u32 __iomem *s0ix_wake_en;
	struct pci_dev const *pdev;
	struct semaphore nc_ready_lock;
	u32 s3_residency;
	u32 residency_total;
	u32 s3_count;
};

char *states[] = {
	"S0IR",
	"S0I1",
	"S0I2",
	"S0I3",
	"S0",
	"S3",
};

/* REVISIT, FIXME: paranoia's sake */
struct pmc_dev *pmc;

static char *dstates[] = {"D0", "D0i1", "D0i2", "D0i3"};
struct nc_device {
	char *name;
	int reg;
	int sss_pos;
} nc_devices[] = {
	{ "GFX RENDER", PWRGT_STATUS,  RENDER_POS },
	{ "GFX MEDIA", PWRGT_STATUS, MEDIA_POS },
	{ "DISPLAY", PWRGT_STATUS,  DISPLAY_POS },
	{ "VED", VED_SS_PM0, SSS_SHIFT},
	{ "ISP", ISP_SS_PM0, SSS_SHIFT},
	{ "MIO", MIO_SS_PM, SSS_SHIFT},
};

static int pmc_wait_for_nc_pmcmd_complete(int verify_mask,
				int status_mask, int state_type , int reg)
{
	int pwr_sts;
	int count = 0;

	while (true) {
		if (reg == PWRGT_CNT)
			pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT,
							PWRGT_STATUS);
		else {
			pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
			pwr_sts = pwr_sts >> SSS_SHIFT;
		}
		if (state_type == ISLAND_DOWN ||
				state_type == ISLAND_SR) {
			if ((pwr_sts & status_mask) ==
					(verify_mask & status_mask))
				break;
			else
				usleep_range(10, 20);
		} else if (state_type == ISLAND_UP) {
			if ((~pwr_sts & status_mask)  ==
					(~verify_mask & status_mask))
				break;
			else
				usleep_range(10, 20);
		}

		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

/*
 * FIXME: The functions below are legacy and subject to change. They are kept
 * for backward compatibility.
 */
int pmc_nc_get_power_state(int islands, int reg)
{
	int pwr_sts, i, lss, ret = 0;

	if (unlikely(!pmc))
		return -EAGAIN;

	might_sleep();

	down(&pmc->nc_ready_lock);

	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	if (reg != PWRGT_STATUS)
		pwr_sts = pwr_sts >> SSS_SHIFT;

	for (i = 0; i < MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			ret = (pwr_sts >> (BITS_PER_LSS * i)) & PMC_D0I3_MASK;
			break;
		}
	}

	up(&pmc->nc_ready_lock);

	return ret;
}
EXPORT_SYMBOL(pmc_nc_get_power_state);

int pmc_nc_set_power_state(int islands, int state_type, int reg)
{
	u32 pwr_sts = 0;
	u32 pwr_mask = 0;
	int i, lss, mask;
	int ret = 0;
	int status_mask = 0;

	if (unlikely(!pmc))
		return -EAGAIN;

	might_sleep();

	down(&pmc->nc_ready_lock);

	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	pwr_mask = pwr_sts;

	for (i = 0; i < MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			mask = PMC_D0I3_MASK << (BITS_PER_LSS * i);
			status_mask = status_mask | mask;
			if (state_type == ISLAND_DOWN)
				pwr_mask |= mask;
			else if (state_type == ISLAND_UP)
				pwr_mask &= ~mask;
			/* Soft reset case */
			else if (state_type == ISLAND_SR) {
				pwr_mask &= ~mask;
				mask = SR_MASK << (BITS_PER_LSS * i);
				pwr_mask |= mask;
			}
		}
	}

	intel_mid_msgbus_write32(PUNIT_PORT, reg, pwr_mask);
	ret = pmc_wait_for_nc_pmcmd_complete(pwr_mask,
				status_mask, state_type, reg);

	up(&pmc->nc_ready_lock);

	return ret;
}
EXPORT_SYMBOL(pmc_nc_set_power_state);

static u32 pmc_register_read(int reg_offset)
{
	return readl(pmc->pmc_registers + reg_offset);
}

static void print_residency_per_state(struct seq_file *s, int state, u32 count)
{
	struct pmc_dev *pmc_cxt = (struct pmc_dev *)s->private;
	u32 rem_time, rem_res = 0;
	u64 rem_res_reduced = 0;

	/* Counter increments every 32 us. */
	u64 time = (u64)count << 5;
	u64 residency = (u64)count * 100;

	if (pmc_cxt->residency_total) {
		rem_res = do_div(residency, pmc_cxt->residency_total);
		rem_res_reduced = (u64)rem_res * 1000;
		do_div(rem_res_reduced,  pmc_cxt->residency_total);
	}
	rem_time = do_div(time, USEC_PER_SEC);
	seq_printf(s, "%s \t\t %.6llu.%.6u \t\t %.2llu.%.3llu", states[state],
			time, rem_time, residency, rem_res_reduced);
	if (state == MAX_PLATFORM_STATES)
		seq_printf(s, " \t\t %u\n", pmc_cxt->s3_count);
	else
		seq_printf(s, " \t\t %s\n", "--");
}

static int pmc_devices_state_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmc_cxt = (struct pmc_dev *)s->private;
	int i;
	u32 val, nc_pwr_sts, reg;
	unsigned int base_class, sub_class;
	struct pci_dev *dev = NULL;
	u16 pmcsr;
	u32 s0ix_residency[MAX_PLATFORM_STATES];

	pmc_cxt->residency_total = 0;

	/* Read s0ix residency counters */
	for (i = 0; i < MAX_PLATFORM_STATES; i++) {
		s0ix_residency[i] = pmc_register_read(i);
		pmc_cxt->residency_total += s0ix_residency[i];
	}
	s0ix_residency[S0I3] -= pmc_cxt->s3_residency;

	seq_puts(s, "State \t\t Time[sec] \t\t Residency[%%] \t\t Count\n");
	for (i = 0; i < MAX_PLATFORM_STATES; i++)
		print_residency_per_state(s, i, s0ix_residency[i]);
	print_residency_per_state(s, i, pmc_cxt->s3_residency);

	seq_puts(s, "\n\nNORTH COMPLEX DEVICES :\n");

	for (i = 0; i < ARRAY_SIZE(nc_devices); i++) {
		reg = nc_devices[i].reg;
		nc_pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
		nc_pwr_sts >>= nc_devices[i].sss_pos;
		val = nc_pwr_sts & PMC_D0I3_MASK;
		seq_printf(s, "%9s : %s\n", nc_devices[i].name, dstates[val]);
	}

	seq_puts(s, "\nSOUTH COMPLEX DEVICES :\n");

	while ((dev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, dev)) != NULL) {
		/* find the base class info */
		base_class = dev->class >> 16;
		sub_class  = (dev->class & SUB_CLASS_MASK) >> 8;

		if (base_class == PCI_BASE_CLASS_BRIDGE)
			continue;

		if ((base_class == PCI_BASE_CLASS_DISPLAY) && !sub_class)
			continue;

		if ((base_class == PCI_BASE_CLASS_MULTIMEDIA) &&
				(sub_class == ISP_SUB_CLASS))
			continue;

		pci_read_config_word(dev, dev->pm_cap + PCI_PM_CTRL, &pmcsr);
		val = pmcsr & PMC_D0I3_MASK;
		seq_printf(s, "%9s %15s : %s\n", dev_name(&dev->dev),
			dev_driver_string(&dev->dev), dstates[val]);
	}

	seq_puts(s, "\n");

	return 0;
}


static int devices_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_devices_state_show, inode->i_private);
}

static const struct file_operations devices_state_operations = {
	.open           = devices_state_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int nc_set_power_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t nc_set_power_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	int islands, state, reg, buf_size;
	struct pci_dev *dev = NULL;
	u16 pmcsr, val;

	buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%d %d %d", &islands, &state, &reg) != 3)
		return -EFAULT;

	if (!islands) {
		while ((dev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, dev))
								!= NULL) {
			pci_read_config_word(dev, dev->pm_cap +
							PCI_PM_CTRL, &pmcsr);
			val = pmcsr & PMC_D0I3_MASK;
			if (!val) {
				pmcsr |= PMC_D0I3_MASK;
				pci_write_config_word(dev, dev->pm_cap +
							PCI_PM_CTRL, pmcsr);
			}
		}
		return count;
	}

	pmc_nc_set_power_state(islands, state, reg);
	return count;
}

static int nc_set_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, nc_set_power_show, inode->i_private);
}

static const struct file_operations nc_set_power_operations = {
	.open           = nc_set_power_open,
	.read           = seq_read,
	.write          = nc_set_power_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int pmc_suspend_enter(suspend_state_t state)
{
	u32 temp = 0, count_before_entry, count_after_exit;

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;

	count_before_entry = pmc_register_read(S0I3);
	trace_printk("s3_entry\n");

	__monitor((void *)&temp, 0, 0);
	smp_mb();
	__mwait(BYT_S3_HINT, 1);

	trace_printk("s3_exit\n");
	pmc->s3_count += 1;
	count_after_exit = pmc_register_read(S0I3);

	pmc->s3_residency += (count_after_exit - count_before_entry);

	return 0;
}

static void put_driverless_pci_devices_in_d0i3(void)
{
	struct pci_dev *pdev = NULL;
	u16 pmcsr;

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev))
								!= NULL) {
		pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmcsr);

		/* In case, device doesn't have driver and it's in D0,
		 * put it in D0i3 */
		if (IS_ERR_OR_NULL(pdev->dev.driver) &&
				!(pmcsr & PMC_D0I3_MASK)) {
			dev_info(&pdev->dev, "put device in D0i3\n");
			pmcsr |= PMC_D0I3_MASK;
			pci_write_config_word(pdev, pdev->pm_cap +
						PCI_PM_CTRL, pmcsr);
		}
	}
}

static int mid_suspend_prepare(void)
{
	put_driverless_pci_devices_in_d0i3();
	return 0;
}

static const struct platform_suspend_ops pmc_suspend_ops = {
	.valid = suspend_valid_only_mem,
	.prepare = mid_suspend_prepare,
	.enter = pmc_suspend_enter,
};

static DEFINE_PCI_DEVICE_TABLE(pmc_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0F1C)},
	{0,}
};
MODULE_DEVICE_TABLE(pci, pmc_pci_tbl);

static int pmc_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	int error = 0;
	struct dentry *d, *d1;
	struct pmc_dev *pmc_cxt;

	pmc_cxt = devm_kzalloc(&pdev->dev,
			sizeof(struct pmc_dev), GFP_KERNEL);

	if (!pmc_cxt) {
		dev_err(&pdev->dev, "Failed to allocate memory for pmc_cxt.\n");
		return -ENOMEM;
	}

	pmc = pmc_cxt;

	if (pci_enable_device(pdev)) {
		dev_err(&pdev->dev, "Failed to initialize PMC as PCI device\n");
		error = -EFAULT;
		goto exit_err;
	}

	pci_read_config_dword(pdev, PCI_CB_LEGACY_MODE_BASE,
					&pmc_cxt->base_address);
	pmc_cxt->base_address &= BASE_ADDRESS_MASK;

	if (pci_request_region(pdev, PMC_MMIO_BAR, "pmc_driver")) {
		dev_err(&pdev->dev, "Failed to allocate requested PCI region\n");
		error = -EFAULT;
		goto exit_err;
	}

	pmc_cxt->pmc_registers = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + S0IX_REGISTERS_OFFSET, 20);

	pmc_cxt->s0ix_wake_en = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + S0IX_WAKE_EN, 4);

	if (!pmc_cxt->pmc_registers || !pmc_cxt->s0ix_wake_en) {
		dev_err(&pdev->dev, "Failed to map PMC registers.\n");
		error = -EFAULT;
		goto err_release_region;
	}

	suspend_set_ops(&pmc_suspend_ops);

	sema_init(&pmc_cxt->nc_ready_lock, 1);

	pci_set_drvdata(pdev, pmc_cxt);

	/* /sys/kernel/debug/mid_pmu_states */
	d = debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, pmc_cxt, &devices_state_operations);
	if (!d) {
		dev_err(&pdev->dev, "Can not create a debug file\n");
		error = -ENOMEM;
		goto err_release_region;
	}

	/* /sys/kernel/debug/pmc_states */
	d1 = debugfs_create_file("nc_set_power", S_IFREG | S_IRUGO,
				NULL, pmc_cxt, &nc_set_power_operations);

	if (!d) {
		dev_err(&pdev->dev, "Can not create a debug file\n");
		error = -ENOMEM;
		debugfs_remove(d);
		goto err_release_region;
	}

	writel(DISABLE_LPC_CLK_WAKE_EN, pmc_cxt->s0ix_wake_en);

	return 0;

err_release_region:
	pci_release_region(pdev, PMC_MMIO_BAR);
exit_err:
	dev_err(&pdev->dev, "Initialization failed\n");

	return error;
}

static struct pci_driver pmc_pci_driver = {
	.name = "pmc",
	.id_table = pmc_pci_tbl,
	.probe = pmc_pci_probe,
};

module_pci_driver(pmc_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel ATOM Platform Power Management Controller (PMC) Driver");
