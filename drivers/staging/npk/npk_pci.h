#ifndef _NPK_PCI_H_
#define _NPK_PCI_H_

#include <linux/io.h>
#include <linux/pci.h>

#define NPK_REG_CMD_R 0
#define NPK_REG_CMD_W 1

#define NPK_IO_LEN 0xfffff

#define MAX_NUM_OF_STH_CHANNELS 128

#define MAX_NUM_MSU 2

#define FIRST_SVEN_KERNEL_CHANNEL 7
#define LAST_SVEN_KERNEL_CHANNEL 10

struct npk_sth_master {
	void __iomem *base;
	DECLARE_BITMAP(channel_map, MAX_NUM_OF_STH_CHANNELS);
};

struct sth_channel_blk {
	int num_sth_channel;
	unsigned long phy;
	unsigned long pg_offset;
	unsigned long length;
	struct list_head element;
};

struct npk_sth_reg {
	u32 sthcap0;
	u32 sthcap1;
} __packed;

struct npk_msu_reg {
	u32 mscctrl;
	u32 mscsts;
	u32 mscbar;
	u32 mscdestsz;
	u32 mscmwp;
	u32 msctrp;
	u32 msctwp;
} __packed;

struct npk_device_info {
	struct npk_sth_reg *sth_reg;
	struct npk_msu_reg *msu_reg;
	bool (*check_reg_range)(u32 offset);
	bool (*check_pci_range)(u32 offset);
};

struct npk_pci_device {
	struct pci_dev *pdev;

	/* CRS/MTB bar MMIO region */
	void __iomem *csr_mtb_bar;
	phys_addr_t csr_mtb_paddr;

	/* SW bar MMIO region */
	void __iomem *sw_bar;
	phys_addr_t sw_paddr;
	u32 sw_iolen;
	u32 k_iolen;
	u32 u_iolen;

	/* Management of STH channels */
	struct sth_channel_blk *sth_head;
	struct list_head free_queue;
	struct list_head used_queue;

	/* NPK HW specific info */
	struct npk_device_info *dev_info;

	/* Number of channels per master */
	u8 chlcnt;
};

int npk_reg_read(u32 offset, u32 *data);
int npk_reg_write(u32 offset, u32 data);
int npk_reg_read_no_lock(u32 offset, u32 *data);
int npk_reg_write_no_lock(u32 offset, u32 data);

int npk_pci_read(u32 offset, u32 *data, u32 size);
int npk_pci_write(u32 offset, u32 data, u32 size);

struct npk_pci_device *get_npk_dev(void);
void put_npk_dev(void);

#endif /* _NPK_PCI_H */
