/* gpio-langwell.c Moorestown platform Langwell chip GPIO driver
 * Copyright (c) 2008 - 2013,  Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Supports:
 * Moorestown platform Langwell chip.
 * Medfield platform Penwell chip.
 * Whitney point.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/stddef.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/lnw_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/irqdomain.h>

#define IRQ_TYPE_EDGE	(1 << 0)
#define IRQ_TYPE_LEVEL	(1 << 1)

/*
 * Langwell chip has 64 pins and thus there are 2 32bit registers to control
 * each feature, while Penwell chip has 96 pins for each block, and need 3 32bit
 * registers to control them, so we only define the order here instead of a
 * structure, to get a bit offset for a pin (use GPDR as an example):
 *
 * nreg = ngpio / 32;
 * reg = offset / 32;
 * bit = offset % 32;
 * reg_addr = reg_base + GPDR * nreg * 4 + reg * 4;
 *
 * so the bit of reg_addr is to control pin offset's GPDR feature
*/

enum GPIO_REG {
	GPLR = 0,	/* pin level read-only */
	GPDR,		/* pin direction */
	GPSR,		/* pin set */
	GPCR,		/* pin clear */
	GRER,		/* rising edge detect */
	GFER,		/* falling edge detect */
	GEDR,		/* edge detect result */
	GAFR,		/* alt function */
	GFBR = 9,	/* glitch filter bypas */
	GPIT,		/* interrupt type */
	GPIP = GFER,	/* level interrupt polarity */
	GPIM = GRER,	/* level interrupt mask */

	/* the following registers only exist on MRFLD */
	GFBR_TNG = 6,
	GIMR,		/* interrupt mask */
	GISR,		/* interrupt source */
	GITR = 32,	/* interrupt type */
	GLPR = 33,	/* level-input polarity */
};

enum GPIO_CONTROLLERS {
	LINCROFT_GPIO,
	PENWELL_GPIO_AON,
	PENWELL_GPIO_CORE,
	CLOVERVIEW_GPIO_AON,
	CLOVERVIEW_GPIO_CORE,
	TANGIER_GPIO,
};

/* langwell gpio driver data */
struct lnw_gpio_ddata_t {
	u16 ngpio;		/* number of gpio pins */
	u32 gplr_offset;	/* offset of first GPLR register from base */
	u32 flis_base;		/* base address of FLIS registers */
	u32 flis_len;		/* length of FLIS registers */
	u32 (*get_flis_offset)(int gpio);
	u32 chip_irq_type;	/* chip interrupt type */
};

struct gpio_flis_pair {
	int gpio;	/* gpio number */
	int offset;	/* register offset from FLIS base */
};

/*
 * The following mapping table lists the pin and flis offset pair
 * of some key gpio pins, the offset of other gpios can be calculated
 * from the table.
 */
static struct gpio_flis_pair gpio_flis_mapping_table[] = {
	{ 0,	0x2900 },
	{ 12,	0x2544 },
	{ 14,	0x0958 },
	{ 16,	0x2D18 },
	{ 17,	0x1D10 },
	{ 19,	0x1D00 },
	{ 23,	0x1D18 },
	{ 31,	-EINVAL }, /* No GPIO 31 in pin list */
	{ 32,	0x1508 },
	{ 44,	0x3500 },
	{ 64,	0x2534 },
	{ 68,	0x2D1C },
	{ 70,	0x1500 },
	{ 72,	0x3D00 },
	{ 77,	0x0D00 },
	{ 97,	0x0954 },
	{ 98,	-EINVAL }, /* No GPIO 98-101 in pin list */
	{ 102,	0x1910 },
	{ 120,	0x1900 },
	{ 124,	0x2100 },
	{ 136,	-EINVAL }, /* No GPIO 136 in pin list */
	{ 137,	0x2D00 },
	{ 143,	-EINVAL }, /* No GPIO 143-153 in pin list */
	{ 154,	0x092C },
	{ 164,	0x3900 },
	{ 177,	0x2500 },
	{ 190,	0x2D50 },
};

static u32 get_flis_offset_by_gpio(int gpio)
{
	int i;
	int start;
	u32 offset = -EINVAL;

	for (i = 0; i < ARRAY_SIZE(gpio_flis_mapping_table) - 1; i++) {
		if (gpio >= gpio_flis_mapping_table[i].gpio
			&& gpio < gpio_flis_mapping_table[i + 1].gpio)
			break;
	}

	start = gpio_flis_mapping_table[i].gpio;

	if (gpio_flis_mapping_table[i].offset != -EINVAL) {
		offset = gpio_flis_mapping_table[i].offset
				+ (gpio - start) * 4;
	}

	return offset;
}

static struct lnw_gpio_ddata_t lnw_gpio_ddata[] = {
	[LINCROFT_GPIO] = {
		.ngpio = 64,
	},
	[PENWELL_GPIO_AON] = {
		.ngpio = 96,
		.chip_irq_type = IRQ_TYPE_EDGE,
	},
	[PENWELL_GPIO_CORE] = {
		.ngpio = 96,
		.chip_irq_type = IRQ_TYPE_EDGE,
	},
	[CLOVERVIEW_GPIO_AON] = {
		.ngpio = 96,
		.chip_irq_type = IRQ_TYPE_EDGE | IRQ_TYPE_LEVEL,
	},
	[CLOVERVIEW_GPIO_CORE] = {
		.ngpio = 96,
		.chip_irq_type = IRQ_TYPE_EDGE,
	},
	[TANGIER_GPIO] = {
		.ngpio = 192,
		.gplr_offset = 4,
		.flis_base = 0xFF0C0000,
		.flis_len = 0x8000,
		.get_flis_offset = get_flis_offset_by_gpio,
		.chip_irq_type = IRQ_TYPE_EDGE | IRQ_TYPE_LEVEL,
	},
};

struct lnw_gpio {
	struct gpio_chip	chip;
	void			*reg_base;
	void			*reg_gplr;
	void			*flis_base;
	spinlock_t		lock;
	struct pci_dev		*pdev;
	struct irq_domain	*domain;
	u32			(*get_flis_offset)(int gpio);
	u32			chip_irq_type;
	int			type;
};

#define to_lnw_priv(chip)	container_of(chip, struct lnw_gpio, chip)

static void __iomem *gpio_reg(struct gpio_chip *chip, unsigned offset,
			enum GPIO_REG reg_type)
{
	struct lnw_gpio *lnw = to_lnw_priv(chip);
	unsigned nreg = chip->ngpio / 32;
	u8 reg = offset / 32;
	void __iomem *ptr;
	void *base;

	/**
	 * On TNG B0, GITR[0]' address is 0xFF008300, while GPLR[0]'s address
	 * is 0xFF008004. To count GITR[0]'s address, it's easier to count
	 * from 0xFF008000. So for GITR,GLPR... we switch the base to reg_base.
	 * This does not affect PNW/CLV, since the reg_gplr is the reg_base,
	 * while on TNG, the reg_gplr has an offset of 0x4.
	 */
	base = reg_type < GITR ? lnw->reg_gplr : lnw->reg_base;
	ptr = (void __iomem *)(base + reg_type * nreg * 4 + reg * 4);
	return ptr;
}

void lnw_gpio_set_alt(int gpio, int alt)
{
	struct lnw_gpio *lnw;
	u32 __iomem *mem;
	int reg;
	int bit;
	u32 offset;
	u32 value;
	unsigned long flags;

	/* use this trick to get memio */
	lnw = irq_get_chip_data(gpio_to_irq(gpio));
	if (!lnw) {
		pr_err("langwell_gpio: can not find pin %d\n", gpio);
		return;
	}
	if (gpio < lnw->chip.base || gpio >= lnw->chip.base + lnw->chip.ngpio) {
		dev_err(lnw->chip.dev, "langwell_gpio: wrong pin %d to config alt\n", gpio);
		return;
	}
#if 0
	if (lnw->irq_base + gpio - lnw->chip.base != gpio_to_irq(gpio)) {
		dev_err(lnw->chip.dev, "langwell_gpio: wrong chip data for pin %d\n", gpio);
		return;
	}
#endif
	gpio -= lnw->chip.base;

	if (lnw->type != TANGIER_GPIO) {
		reg = gpio / 16;
		bit = gpio % 16;

		mem = gpio_reg(&lnw->chip, 0, GAFR);
		spin_lock_irqsave(&lnw->lock, flags);
		value = readl(mem + reg);
		value &= ~(3 << (bit * 2));
		value |= (alt & 3) << (bit * 2);
		writel(value, mem + reg);
		spin_unlock_irqrestore(&lnw->lock, flags);
		dev_dbg(lnw->chip.dev, "ALT: writing 0x%x to %p\n",
			value, mem + reg);
	} else {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return;

		mem = (void __iomem *)(lnw->flis_base + offset);

		spin_lock_irqsave(&lnw->lock, flags);
		value = readl(mem);
		value &= ~7;
		value |= (alt & 7);
		writel(value, mem);
		spin_unlock_irqrestore(&lnw->lock, flags);
		dev_dbg(lnw->chip.dev, "ALT: writing 0x%x to %p\n", value, mem);
	}
}
EXPORT_SYMBOL_GPL(lnw_gpio_set_alt);

int gpio_get_alt(int gpio)
{
	struct lnw_gpio *lnw;
	u32 __iomem *mem;
	int reg;
	int bit;
	u32 value;
	u32 offset;

	 /* use this trick to get memio */
	lnw = irq_get_chip_data(gpio_to_irq(gpio));
	if (!lnw) {
		pr_err("langwell_gpio: can not find pin %d\n", gpio);
		return -1;
	}
	if (gpio < lnw->chip.base || gpio >= lnw->chip.base + lnw->chip.ngpio) {
		dev_err(lnw->chip.dev,
			"langwell_gpio: wrong pin %d to config alt\n", gpio);
		return -1;
	}
#if 0
	if (lnw->irq_base + gpio - lnw->chip.base != gpio_to_irq(gpio)) {
		dev_err(lnw->chip.dev,
			"langwell_gpio: wrong chip data for pin %d\n", gpio);
		return -1;
	}
#endif
	gpio -= lnw->chip.base;

	if (lnw->type != TANGIER_GPIO) {
		reg = gpio / 16;
		bit = gpio % 16;

		mem = gpio_reg(&lnw->chip, 0, GAFR);
		value = readl(mem + reg);
		value &= (3 << (bit * 2));
		value >>= (bit * 2);
	} else {
		offset = lnw->get_flis_offset(gpio);
		if (WARN(offset == -EINVAL, "invalid pin %d\n", gpio))
			return -EINVAL;

		mem = (void __iomem *)(lnw->flis_base + offset);

		value = readl(mem) & 7;
	}

	return value;
}
EXPORT_SYMBOL_GPL(gpio_get_alt);

static int lnw_gpio_set_debounce(struct gpio_chip *chip, unsigned offset,
				 unsigned debounce)
{
	struct lnw_gpio *lnw = to_lnw_priv(chip);
	void __iomem *gfbr;
	unsigned long flags;
	u32 value;
	enum GPIO_REG reg_type;

	reg_type = (lnw->type == TANGIER_GPIO) ? GFBR_TNG : GFBR;
	gfbr = gpio_reg(chip, offset, reg_type);

	if (lnw->pdev)
		pm_runtime_get(&lnw->pdev->dev);

	spin_lock_irqsave(&lnw->lock, flags);
	value = readl(gfbr);
	if (debounce) {
		/* debounce bypass disable */
		value &= ~BIT(offset % 32);
	} else {
		/* debounce bypass enable */
		value |= BIT(offset % 32);
	}
	writel(value, gfbr);
	spin_unlock_irqrestore(&lnw->lock, flags);

	if (lnw->pdev)
		pm_runtime_put(&lnw->pdev->dev);

	return 0;
}

static void __iomem *gpio_reg_2bit(struct gpio_chip *chip, unsigned offset,
				   enum GPIO_REG reg_type)
{
	struct lnw_gpio *lnw = to_lnw_priv(chip);
	unsigned nreg = chip->ngpio / 32;
	u8 reg = offset / 16;
	void __iomem *ptr;

	ptr = (void __iomem *)(lnw->reg_base + reg_type * nreg * 4 + reg * 4);
	return ptr;
}

static int lnw_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *gafr = gpio_reg_2bit(chip, offset, GAFR);
	u32 value = readl(gafr);
	int shift = (offset % 16) << 1, af = (value >> shift) & 3;

	if (af) {
		value &= ~(3 << shift);
		writel(value, gafr);
	}
	return 0;
}

static int lnw_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void __iomem *gplr = gpio_reg(chip, offset, GPLR);

	return readl(gplr) & BIT(offset % 32);
}

static void lnw_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	void __iomem *gpsr, *gpcr;

	if (value) {
		gpsr = gpio_reg(chip, offset, GPSR);
		writel(BIT(offset % 32), gpsr);
	} else {
		gpcr = gpio_reg(chip, offset, GPCR);
		writel(BIT(offset % 32), gpcr);
	}
}

static int lnw_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct lnw_gpio *lnw = to_lnw_priv(chip);
	void __iomem *gpdr = gpio_reg(chip, offset, GPDR);
	u32 value;
	unsigned long flags;

	if (lnw->pdev)
		pm_runtime_get(&lnw->pdev->dev);

	spin_lock_irqsave(&lnw->lock, flags);
	value = readl(gpdr);
	value &= ~BIT(offset % 32);
	writel(value, gpdr);
	spin_unlock_irqrestore(&lnw->lock, flags);

	if (lnw->pdev)
		pm_runtime_put(&lnw->pdev->dev);

	return 0;
}

static int lnw_gpio_direction_output(struct gpio_chip *chip,
			unsigned offset, int value)
{
	struct lnw_gpio *lnw = to_lnw_priv(chip);
	void __iomem *gpdr = gpio_reg(chip, offset, GPDR);
	unsigned long flags;

	lnw_gpio_set(chip, offset, value);

	if (lnw->pdev)
		pm_runtime_get(&lnw->pdev->dev);

	spin_lock_irqsave(&lnw->lock, flags);
	value = readl(gpdr);
	value |= BIT(offset % 32);
	writel(value, gpdr);
	spin_unlock_irqrestore(&lnw->lock, flags);

	if (lnw->pdev)
		pm_runtime_put(&lnw->pdev->dev);

	return 0;
}

static int lnw_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct lnw_gpio *lnw = to_lnw_priv(chip);
	return irq_create_mapping(lnw->domain, offset);
}

static int lnw_irq_type(struct irq_data *d, unsigned type)
{
	struct lnw_gpio *lnw = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	unsigned long flags;
	u32 value;
	int ret = 0;
	void __iomem *grer = gpio_reg(&lnw->chip, gpio, GRER);
	void __iomem *gfer = gpio_reg(&lnw->chip, gpio, GFER);
	void __iomem *gpit, *gpip;

	if (gpio >= lnw->chip.ngpio)
		return -EINVAL;

	if (lnw->pdev)
		pm_runtime_get(&lnw->pdev->dev);

	/* Chip that supports level interrupt has extra GPIT registers */
	if (lnw->chip_irq_type & IRQ_TYPE_LEVEL) {
		switch (lnw->type) {
		case CLOVERVIEW_GPIO_AON:
			gpit = gpio_reg(&lnw->chip, gpio, GPIT);
			gpip = gpio_reg(&lnw->chip, gpio, GPIP);
			break;
		case TANGIER_GPIO:
			gpit = gpio_reg(&lnw->chip, gpio, GITR);
			gpip = gpio_reg(&lnw->chip, gpio, GLPR);
			break;
		default:
			ret = -EINVAL;
			goto out;
		}

		spin_lock_irqsave(&lnw->lock, flags);
		if (type & IRQ_TYPE_LEVEL_MASK) {
			/* To prevent glitches from triggering an unintended
			 * level interrupt, configure GLPR register first
			 * and then configure GITR.
			 */
			if (type & IRQ_TYPE_LEVEL_LOW)
				value = readl(gpip) | BIT(gpio % 32);
			else
				value = readl(gpip) & (~BIT(gpio % 32));
			writel(value, gpip);

			value = readl(gpit) | BIT(gpio % 32);
			writel(value, gpit);

			__irq_set_handler_locked(d->irq, handle_level_irq);
		} else if (type & IRQ_TYPE_EDGE_BOTH) {
			value = readl(gpit) & (~BIT(gpio % 32));
			writel(value, gpit);

			if (type & IRQ_TYPE_EDGE_RISING)
				value = readl(grer) | BIT(gpio % 32);
			else
				value = readl(grer) & (~BIT(gpio % 32));
			writel(value, grer);

			if (type & IRQ_TYPE_EDGE_FALLING)
				value = readl(gfer) | BIT(gpio % 32);
			else
				value = readl(gfer) & (~BIT(gpio % 32));
			writel(value, gfer);

			__irq_set_handler_locked(d->irq, handle_edge_irq);
		}
		spin_unlock_irqrestore(&lnw->lock, flags);
	} else {
		if (type & IRQ_TYPE_LEVEL_MASK) {
			ret = -EINVAL;
		} else if (type & IRQ_TYPE_EDGE_BOTH) {
			spin_lock_irqsave(&lnw->lock, flags);

			if (type & IRQ_TYPE_EDGE_RISING)
				value = readl(grer) | BIT(gpio % 32);
			else
				value = readl(grer) & (~BIT(gpio % 32));
			writel(value, grer);

			if (type & IRQ_TYPE_EDGE_FALLING)
				value = readl(gfer) | BIT(gpio % 32);
			else
				value = readl(gfer) & (~BIT(gpio % 32));
			writel(value, gfer);

			spin_unlock_irqrestore(&lnw->lock, flags);
		}
	}

out:
	if (lnw->pdev)
		pm_runtime_put(&lnw->pdev->dev);

	return ret;
}

static int lnw_set_maskunmask(struct irq_data *d, enum GPIO_REG reg_type,
				unsigned unmask)
{
	struct lnw_gpio *lnw = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	unsigned long flags;
	u32 value;
	void __iomem *gp_reg;

	gp_reg = gpio_reg(&lnw->chip, gpio, reg_type);

	spin_lock_irqsave(&lnw->lock, flags);

	if (unmask) {
		/* enable interrupt from GPIO input pin */
		value = readl(gp_reg) | BIT(gpio % 32);
	} else {
		/* disable interrupt from GPIO input pin */
		value = readl(gp_reg) & (~BIT(gpio % 32));
	}

	writel(value, gp_reg);

	spin_unlock_irqrestore(&lnw->lock, flags);

	return 0;
}

static void lnw_irq_unmask(struct irq_data *d)
{
	struct lnw_gpio *lnw = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	void __iomem *gpit;

	if (gpio >= lnw->chip.ngpio)
		return;

	switch (lnw->type) {
	case CLOVERVIEW_GPIO_AON:
		gpit = gpio_reg(&lnw->chip, gpio, GPIT);

		/* if it's level trigger, unmask GPIM */
		if (readl(gpit) & BIT(gpio % 32))
			lnw_set_maskunmask(d, GPIM, 1);

		break;
	case TANGIER_GPIO:
		lnw_set_maskunmask(d, GIMR, 1);
		break;
	default:
		break;
	}
}

static void lnw_irq_mask(struct irq_data *d)
{
	struct lnw_gpio *lnw = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	void __iomem *gpit;

	if (gpio >= lnw->chip.ngpio)
		return;

	switch (lnw->type) {
	case CLOVERVIEW_GPIO_AON:
		gpit = gpio_reg(&lnw->chip, gpio, GPIT);

		/* if it's level trigger, mask GPIM */
		if (readl(gpit) & BIT(gpio % 32))
			lnw_set_maskunmask(d, GPIM, 0);

		break;
	case TANGIER_GPIO:
		lnw_set_maskunmask(d, GIMR, 0);
		break;
	default:
		break;
	}
}

static int lwn_irq_set_wake(struct irq_data *d, unsigned on)
{
	return 0;
}

static void lnw_irq_ack(struct irq_data *d)
{
}

static void lnw_irq_shutdown(struct irq_data *d)
{
	struct lnw_gpio *lnw = irq_data_get_irq_chip_data(d);
	u32 gpio = irqd_to_hwirq(d);
	unsigned long flags;
	u32 value;
	void __iomem *grer = gpio_reg(&lnw->chip, gpio, GRER);
	void __iomem *gfer = gpio_reg(&lnw->chip, gpio, GFER);

	spin_lock_irqsave(&lnw->lock, flags);
	value = readl(grer) & (~BIT(gpio % 32));
	writel(value, grer);
	value = readl(gfer) & (~BIT(gpio % 32));
	writel(value, gfer);
	spin_unlock_irqrestore(&lnw->lock, flags);
};


static struct irq_chip lnw_irqchip = {
	.name		= "LNW-GPIO",
	.flags		= IRQCHIP_SET_TYPE_MASKED,
	.irq_mask	= lnw_irq_mask,
	.irq_unmask	= lnw_irq_unmask,
	.irq_set_type	= lnw_irq_type,
	.irq_set_wake	= lwn_irq_set_wake,
	.irq_ack	= lnw_irq_ack,
	.irq_shutdown	= lnw_irq_shutdown,
};

static DEFINE_PCI_DEVICE_TABLE(lnw_gpio_ids) = {   /* pin number */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x080f),
		.driver_data = LINCROFT_GPIO },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081f),
		.driver_data = PENWELL_GPIO_AON },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x081a),
		.driver_data = PENWELL_GPIO_CORE },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08eb),
		.driver_data = CLOVERVIEW_GPIO_AON },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x08f7),
		.driver_data = CLOVERVIEW_GPIO_CORE },
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x1199),
		.driver_data = TANGIER_GPIO },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, lnw_gpio_ids);

static void lnw_irq_handler(unsigned irq, struct irq_desc *desc)
{
	struct irq_data *data = irq_desc_get_irq_data(desc);
	struct lnw_gpio *lnw = irq_data_get_irq_handler_data(data);
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	u32 base, gpio, mask;
	unsigned long pending;
	void __iomem *gp_reg;
	enum GPIO_REG reg_type;
	struct irq_desc *lnw_irq_desc;
	unsigned int lnw_irq;

	reg_type = (lnw->type == TANGIER_GPIO) ? GISR : GEDR;

	/* check GPIO controller to check which pin triggered the interrupt */
	for (base = 0; base < lnw->chip.ngpio; base += 32) {
		gp_reg = gpio_reg(&lnw->chip, base, reg_type);
		while ((pending = (lnw->type != TANGIER_GPIO) ?
			readl(gp_reg) :
			(readl(gp_reg) &
			readl(gpio_reg(&lnw->chip, base, GIMR))))) {
			gpio = __ffs(pending);
			/* Mask irq if not requested in kernel */
			lnw_irq = irq_find_mapping(lnw->domain, base + gpio);
			lnw_irq_desc = irq_to_desc(lnw_irq);
			if (lnw_irq_desc && unlikely(!lnw_irq_desc->action)) {
				lnw_irq_mask(&lnw_irq_desc->irq_data);
				continue;
			}

			mask = BIT(gpio);
			/* Clear before handling so we can't lose an edge */
			writel(mask, gp_reg);
			generic_handle_irq(lnw_irq);
		}
	}

	chip->irq_eoi(data);
}

static void lnw_irq_init_hw(struct lnw_gpio *lnw)
{
	void __iomem *reg;
	unsigned base;

	for (base = 0; base < lnw->chip.ngpio; base += 32) {
		/* Clear the rising-edge detect register */
		reg = gpio_reg(&lnw->chip, base, GRER);
		writel(0, reg);
		/* Clear the falling-edge detect register */
		reg = gpio_reg(&lnw->chip, base, GFER);
		writel(0, reg);
		/* Clear the edge detect status register */
		reg = gpio_reg(&lnw->chip, base, GEDR);
		writel(~0, reg);
	}
}

static int lnw_gpio_irq_map(struct irq_domain *d, unsigned int virq,
			    irq_hw_number_t hw)
{
	struct lnw_gpio *lnw = d->host_data;

	irq_set_chip_and_handler_name(virq, &lnw_irqchip, handle_simple_irq,
				      "demux");
	irq_set_chip_data(virq, lnw);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static const struct irq_domain_ops lnw_gpio_irq_ops = {
	.map = lnw_gpio_irq_map,
	.xlate = irq_domain_xlate_twocell,
};

static int lnw_gpio_runtime_resume(struct device *dev)
{
	return 0;
}

static int lnw_gpio_runtime_suspend(struct device *dev)
{
	return 0;
}

static int lnw_gpio_runtime_idle(struct device *dev)
{
	int err = pm_schedule_suspend(dev, 500);

	if (!err)
		return 0;

	return -EBUSY;
}

static const struct dev_pm_ops lnw_gpio_pm_ops = {
	SET_RUNTIME_PM_OPS(lnw_gpio_runtime_suspend,
			   lnw_gpio_runtime_resume,
			   lnw_gpio_runtime_idle)
};

static int lnw_gpio_probe(struct pci_dev *pdev,
			const struct pci_device_id *id)
{
	void *base;
	resource_size_t start, len;
	struct lnw_gpio *lnw;
	u32 gpio_base;
	u32 irq_base;
	int retval;
	struct lnw_gpio_ddata_t *ddata;
	int pid;

	pid = id->driver_data;
	ddata = &lnw_gpio_ddata[pid];

	retval = pci_enable_device(pdev);
	if (retval)
		return retval;

	retval = pci_request_regions(pdev, "langwell_gpio");
	if (retval) {
		dev_err(&pdev->dev, "error requesting resources\n");
		goto err_pci_req_region;
	}
	/* get the gpio_base from bar1 */
	start = pci_resource_start(pdev, 1);
	len = pci_resource_len(pdev, 1);
	base = ioremap_nocache(start, len);
	if (!base) {
		dev_err(&pdev->dev, "error mapping bar1\n");
		retval = -EFAULT;
		goto err_ioremap;
	}
	irq_base = *(u32 *)base;
	gpio_base = *((u32 *)base + 1);
	/* release the IO mapping, since we already get the info from bar1 */
	iounmap(base);
	/* get the register base from bar0 */
	start = pci_resource_start(pdev, 0);
	len = pci_resource_len(pdev, 0);
	base = devm_ioremap_nocache(&pdev->dev, start, len);
	if (!base) {
		dev_err(&pdev->dev, "error mapping bar0\n");
		retval = -EFAULT;
		goto err_ioremap;
	}

	lnw = devm_kzalloc(&pdev->dev, sizeof(*lnw), GFP_KERNEL);
	if (!lnw) {
		dev_err(&pdev->dev, "can't allocate langwell_gpio chip data\n");
		retval = -ENOMEM;
		goto err_ioremap;
	}

	lnw->type = pid;
	lnw->reg_base = base;
	lnw->reg_gplr = lnw->reg_base + ddata->gplr_offset;
	lnw->get_flis_offset = ddata->get_flis_offset;
	lnw->chip_irq_type = ddata->chip_irq_type;
	lnw->chip.label = dev_name(&pdev->dev);
	lnw->chip.request = lnw_gpio_request;
	lnw->chip.direction_input = lnw_gpio_direction_input;
	lnw->chip.direction_output = lnw_gpio_direction_output;
	lnw->chip.set_pinmux = lnw_gpio_set_alt;
	lnw->chip.get_pinmux = gpio_get_alt;
	lnw->chip.get = lnw_gpio_get;
	lnw->chip.set = lnw_gpio_set;
	lnw->chip.to_irq = lnw_gpio_to_irq;
	lnw->chip.base = gpio_base;
	lnw->chip.ngpio = ddata->ngpio;
	lnw->chip.can_sleep = 0;
	lnw->chip.set_debounce = lnw_gpio_set_debounce;
	lnw->pdev = pdev;
	spin_lock_init(&lnw->lock);
	lnw->domain = irq_domain_add_simple(pdev->dev.of_node,
					    lnw->chip.ngpio, irq_base,
					    &lnw_gpio_irq_ops, lnw);
	if (!lnw->domain) {
		retval = -ENOMEM;
		goto err_ioremap;
	}

	pci_set_drvdata(pdev, lnw);
	retval = gpiochip_add(&lnw->chip);
	if (retval) {
		dev_err(&pdev->dev, "langwell gpiochip_add error %d\n", retval);
		goto err_ioremap;
	}

	if (ddata->flis_base) {
		lnw->flis_base = ioremap_nocache(ddata->flis_base,
					ddata->flis_len);
		if (!lnw->flis_base) {
			dev_err(&pdev->dev, "error mapping flis base\n");
			retval = -EFAULT;
			goto err_ioremap;
		}
	}

	lnw_irq_init_hw(lnw);

	irq_set_handler_data(pdev->irq, lnw);
	irq_set_chained_handler(pdev->irq, lnw_irq_handler);

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;

err_ioremap:
	pci_release_regions(pdev);
err_pci_req_region:
	pci_disable_device(pdev);
	return retval;
}

static struct pci_driver lnw_gpio_driver = {
	.name		= "langwell_gpio",
	.id_table	= lnw_gpio_ids,
	.probe		= lnw_gpio_probe,
	.driver		= {
		.pm	= &lnw_gpio_pm_ops,
	},
};


static int wp_gpio_probe(struct platform_device *pdev)
{
	struct lnw_gpio *lnw;
	struct gpio_chip *gc;
	struct resource *rc;
	int retval = 0;

	rc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rc)
		return -EINVAL;

	lnw = kzalloc(sizeof(struct lnw_gpio), GFP_KERNEL);
	if (!lnw) {
		dev_err(&pdev->dev,
			"can't allocate whitneypoint_gpio chip data\n");
		return -ENOMEM;
	}
	lnw->reg_base = ioremap_nocache(rc->start, resource_size(rc));
	if (lnw->reg_base == NULL) {
		retval = -EINVAL;
		goto err_kmalloc;
	}
	spin_lock_init(&lnw->lock);
	gc = &lnw->chip;
	gc->label = dev_name(&pdev->dev);
	gc->owner = THIS_MODULE;
	gc->direction_input = lnw_gpio_direction_input;
	gc->direction_output = lnw_gpio_direction_output;
	gc->get = lnw_gpio_get;
	gc->set = lnw_gpio_set;
	gc->to_irq = NULL;
	gc->base = 0;
	gc->ngpio = 64;
	gc->can_sleep = 0;
	retval = gpiochip_add(gc);
	if (retval) {
		dev_err(&pdev->dev, "whitneypoint gpiochip_add error %d\n",
								retval);
		goto err_ioremap;
	}
	platform_set_drvdata(pdev, lnw);
	return 0;
err_ioremap:
	iounmap(lnw->reg_base);
err_kmalloc:
	kfree(lnw);
	return retval;
}

static int wp_gpio_remove(struct platform_device *pdev)
{
	struct lnw_gpio *lnw = platform_get_drvdata(pdev);
	int err;
	err = gpiochip_remove(&lnw->chip);
	if (err)
		dev_err(&pdev->dev, "failed to remove gpio_chip.\n");
	iounmap(lnw->reg_base);
	kfree(lnw);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver wp_gpio_driver = {
	.probe		= wp_gpio_probe,
	.remove		= wp_gpio_remove,
	.driver		= {
		.name	= "wp_gpio",
		.owner	= THIS_MODULE,
	},
};

static int __init lnw_gpio_init(void)
{
	int ret;
	ret =  pci_register_driver(&lnw_gpio_driver);
	if (ret < 0)
		return ret;
	ret = platform_driver_register(&wp_gpio_driver);
	if (ret < 0)
		pci_unregister_driver(&lnw_gpio_driver);
	return ret;
}

fs_initcall(lnw_gpio_init);
