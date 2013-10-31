
/* platform_ehci_sph_pci.c: USB EHCI/SPH platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>
#include <linux/usb/xhci-ush-hsic-pci.h>

static struct ush_hsic_pdata hsic_pdata = {
	.has_modem = 0,
	.enabled = 0
};

static unsigned int is_ush_hsic_supported(void)
{
	return  INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1);
}

static struct ush_hsic_pdata *get_hsic_platform_data(struct pci_dev *pdev)
{
	struct ush_hsic_pdata *pdata = &hsic_pdata;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_BYT_USH:
		if (is_ush_hsic_supported()) {
			/* request SPH CS_N gpio by name */
			pdata->has_modem = 1;
			pdata->enabled = 1;
		}
		break;

	default:
		return NULL;
		break;
	}

	return pdata;
}

static void hsic_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_hsic_platform_data(pci_dev);

	dev_dbg(&pci_dev->dev, "set run wake flag\n");
	device_set_run_wake(&pci_dev->dev, true);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			hsic_pci_early_quirks);

static void quirk_byt_ush_d3_delay(struct pci_dev *dev)
{
	dev->d3_delay = 10;
}
DECLARE_PCI_FIXUP_ENABLE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			quirk_byt_ush_d3_delay);

#define PCI_USH_SSCFG1		0xb0
#define PCI_USH_SSCFG1_D3	BIT(28)
#define PCI_USH_SSCFG1_SUS	BIT(30)

static void quirk_byt_ush_suspend(struct pci_dev *dev)
{
	u32	value;

	dev_dbg(&dev->dev, "USH suspend quirk\n");
	pci_read_config_dword(dev, PCI_USH_SSCFG1, &value);

	/* set SSCFG1 BIT 28 and 30 before enter D3hot */
	value |= (PCI_USH_SSCFG1_D3 | PCI_USH_SSCFG1_SUS);
	pci_write_config_dword(dev, PCI_USH_SSCFG1, value);

	pci_read_config_dword(dev, PCI_USH_SSCFG1, &value);
	dev_dbg(&dev->dev, "PCI B0 reg (SSCFG1) = 0x%x\n", value);
}
DECLARE_PCI_FIXUP_SUSPEND(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			quirk_byt_ush_suspend);

static void quirk_byt_ush_resume(struct pci_dev *dev)
{
	u32	value;

	dev_dbg(&dev->dev, "USH resume quirk\n");
	pci_read_config_dword(dev, PCI_USH_SSCFG1, &value);

	/* clear SSCFG1 BIT 28 and 30 after back to D0 */
	value &= (~(PCI_USH_SSCFG1_D3 | PCI_USH_SSCFG1_SUS));
	pci_write_config_dword(dev, PCI_USH_SSCFG1, value);

	pci_read_config_dword(dev, PCI_USH_SSCFG1, &value);
	dev_dbg(&dev->dev, "PCI B0 reg (SSCFG1) = 0x%x\n", value);
}
DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			quirk_byt_ush_resume);
