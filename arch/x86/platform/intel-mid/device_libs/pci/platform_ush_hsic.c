
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
#include <asm/intel-mid.h>
#include <linux/usb/xhci-ush-hsic-pci.h>

static struct ush_hsic_pdata hsic_pdata = {
	.has_modem = 0,
	.enabled = 0
};

static unsigned int is_ush_hsic_supported(void)
{
	return  INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0);
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
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			hsic_pci_early_quirks);
