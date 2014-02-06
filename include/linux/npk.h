#ifndef _NPK_H_
#define _NPK_H_

#include <linux/ioctl.h>

#define NPK_IOCTL_MAGIC 'n'

/* ioctl commands to allocate/release an STH channel for SVEN instrumentation in user-space */
#define NPKIOC_ALLOC_PAGE_SVEN  _IOWR(NPK_IOCTL_MAGIC, 1, struct sth_page_request_info)
#define NPKIOC_FREE_PAGE_SVEN	_IOW(NPK_IOCTL_MAGIC, 1, struct sth_page_request_info)

/* ioctl commands to allocate/release a CSR-driven mode buffer for NPK trace */
#define NPKIOC_ALLOC_BUFF       _IOWR(NPK_IOCTL_MAGIC, 2, struct npk_csr_info)
#define NPKIOC_FREE_BUFF	_IOW(NPK_IOCTL_MAGIC, 2, u32)

/* ioctl commands to allocate/release a linked-list mode buffer for NPK trace */
#define NPKIOC_ALLOC_WINDOW	_IOWR(NPK_IOCTL_MAGIC, 3, struct npk_win_info)
#define NPKIOC_FREE_WINDOW	_IOW(NPK_IOCTL_MAGIC, 3, u32)

/* ioctl commands to read/write NPK registers */
#define NPKIOC_REG_READ         _IOWR(NPK_IOCTL_MAGIC, 4, struct npk_reg_cmd)
#define NPKIOC_REG_WRITE	_IOW(NPK_IOCTL_MAGIC, 4, struct npk_reg_cmd)
#define NPKIOC_PCI_REG_READ	_IOWR(NPK_IOCTL_MAGIC, 5, struct pci_cfg_cmd)
#define NPKIOC_PCI_REG_WRITE	_IOW(NPK_IOCTL_MAGIC, 5, struct pci_cfg_cmd)
#define NPKIOC_MEM_READ         _IOWR(NPK_IOCTL_MAGIC, 6, struct npk_mem_cmd)
#define NPKIOC_MEM_WRITE	_IOW(NPK_IOCTL_MAGIC, 6, struct npk_mem_cmd)

/* ioctl command to print debug info on NPK trace buffer */
#define NPKIOC_PRINT_MSU_INFO   _IOW(NPK_IOCTL_MAGIC, 7, u32)


union dn_sth_reg {
	u8 d8;
	u16 d16;
	u32 d32;
	u64 d64;
} __packed;

/* SW master/channel memory map (MIPI STPv2) */
struct sven_sth_channel {
	union dn_sth_reg Dn;
	union dn_sth_reg DnM;
	union dn_sth_reg DnTS;
	union dn_sth_reg DnMTS;
	u64 USER;
	u64 USER_TS;
	u32 FLAG;
	u32 FLAG_TS;
	u32 MERR;
	u32 reserved;
} __packed;

struct sth_page_request_info {
	u32 info_size;
	u32 channel_size;
	u32 offset;
	u32 length;
};

struct npk_csr_info {
	u32 idx;
	u32 size;
};

struct npk_win_info {
	u32 idx;
	u32 num_windows;
	u32 window_size;
	u32 num_blks;
	u32 blk_size;
	u32 pad;
};

struct npk_reg_cmd {
	u32 offset;
	u32 data;
};

struct pci_cfg_cmd {
	u32 offset;
	u32 data;
	u32 size;
};

struct npk_mem_cmd {
	u64 addr;
	u64 data;
	u32 size;
	u32 pad;
};

void *npk_alloc_sth_sven_ptr(int cpu);
void npk_free_sth_sven_ptr(int cpu, void __iomem *channel);

#endif /* _NPK_H_ */
