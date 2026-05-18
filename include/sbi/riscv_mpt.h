/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#ifndef __RISCV_MPT_H__
#define __RISCV_MPT_H__

#include <sbi/riscv_asm.h>
#include <sbi/riscv_locks.h>
#include <sbi/riscv_encoding.h>
#include <sbi/sbi_types.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_bitmap.h>

#define SBI_CSR_MMPT			0x382
#define SBI_CSR_MSDCFG			0x74E

/**
 * mmpt register layout
 * RV32: [31:30]MODE | [29:28]WARL | [27:22]SDID | [21:0]PPN
 * RV64: [63:60]MODE | [59:58]WARL | [57:52]SDID | [51:44]WARL | [43:0]PPN
 */

#if __riscv_xlen == 64
#define SBI_MMPT_MODE_SHIFT		60U
#define SBI_MMPT_MODE_MASK		(UL(0xF)   << SBI_MMPT_MODE_SHIFT)
#define SBI_MMPT_SDID_SHIFT		52U
#define SBI_MMPT_SDID_MASK		(UL(0x3F)  << SBI_MMPT_SDID_SHIFT)
#define SBI_MMPT_PPN_MASK		UL(0x00000FFFFFFFFFFF)   /* 44 bits [43:0] */
#else
#define SBI_MMPT_MODE_SHIFT		30
#define SBI_MMPT_MODE_MASK		(UL(0x3)   << SBI_MMPT_MODE_SHIFT)
#define SBI_MMPT_SDID_SHIFT		22
#define SBI_MMPT_SDID_MASK		(UL(0x3F)  << SBI_MMPT_SDID_SHIFT)
#define SBI_MMPT_PPN_MASK		UL(0x3FFFFF)             /* 22 bits [21:0] */
#endif

#define SBI_MMPT_MODE_BARE		0U

#if __riscv_xlen == 64
#define SBI_MMPT_MODE_SMMPT43		1U
#define SBI_MMPT_MODE_SMMPT52		2U
#define SBI_MMPT_MODE_SMMPT64		3U
#else
# define SBI_MMPT_MODE_SMMPT34		1
#endif

/**
 * MPTE Leaf and Non-Leaf bit definitions
 */
/* MPTE Valid bit */
#define SBI_MPTE_V			(UL(1) << 0)
/* Leaf MPTE */
#define SBI_MPTE_L			(UL(1) << 1)
/* Leaf NAPOT MPTE */
#define SBI_MPTE_N			(UL(1) << 2)

/* PPN shift in non-leaf mpte, same for both RV32 and RV64 */
#define SBI_MPTE_PPN_SHIFT		10U
/* XWR permission start bit in leaf mpte */
#define SBI_MPTE_XWR_BASE		8U
/* XWR permission width in leaf mpte */
#define SBI_MPTE_XWR_WIDTH		3U
/* XWR permission mask in leaf mpte */
#define SBI_MPTE_XWR_MASK		UL(0x7)	 /* Unshifted mask */

#define SBI_MPT_PAGE_SHIFT		PAGE_SHIFT
#define SBI_MPT_PAGE_SIZE		PAGE_SIZE
#define SBI_MPT_PAGE_MASK		(SBI_MPT_PAGE_SIZE - UL(1))

/**
 * XWR permission constants  (XWR=000 = "no access")
 */
#define SBI_MPT_PERM_NONE		0U
#define SBI_MPT_PERM_R			1U
#define SBI_MPT_PERM_W			2U
#define SBI_MPT_PERM_X			4U
#define SBI_MPT_PERM_RW			(SBI_MPT_PERM_R | SBI_MPT_PERM_W)
#define SBI_MPT_PERM_RX			(SBI_MPT_PERM_R | SBI_MPT_PERM_X)
#define SBI_MPT_PERM_RWX		(SBI_MPT_PERM_R | SBI_MPT_PERM_W | SBI_MPT_PERM_X)

/*
 * SDID constants
 */

/* Maximum implemented SDID bit-width */
#define SBI_MPT_SDIDMAX			6U
#define SBI_MPT_MAX_DOMAINS		(UL(1) << SBI_MPT_SDIDMAX)	/* 64 */
#define SBI_MPT_SDID_MASK		(SBI_MPT_MAX_DOMAINS - UL(1))
#define SBI_MPT_SDID_INVALID		SBI_MPT_MAX_DOMAINS		/* 64 */

/**
 * Software bound on the number of regions an supervisor domain
 * may support. One region for firmware and one region for mpt table itself and
 * rest for the general memory regions
 */
#define SBI_MPT_MAX_REGIONS_DOMAIN	32U

/**
 * MMPT and MPTE helper functions
 */
static inline unsigned long sbi_mmpt_encode(unsigned long mode,
					    u32 sdid, unsigned long ppn)
{
	return ((mode << SBI_MMPT_MODE_SHIFT) & SBI_MMPT_MODE_MASK) |
	       (((unsigned long)sdid << SBI_MMPT_SDID_SHIFT) & SBI_MMPT_SDID_MASK) |
	       (ppn & SBI_MMPT_PPN_MASK);
}

/* Physical Address to Non-leaf MPTE */
static inline unsigned long pa_to_mpte_nonleaf(unsigned long pa)
{
	return ((pa >> SBI_MPT_PAGE_SHIFT) << SBI_MPTE_PPN_SHIFT)
	       | SBI_MPTE_V;
}

/* Non-leaf MPTE to Physical Address*/
static inline unsigned long mpte_nonleaf_to_pa(unsigned long mpte_nonleaf)
{
	return (mpte_nonleaf >> SBI_MPTE_PPN_SHIFT) << SBI_MPT_PAGE_SHIFT;
}

/**
 * XWR Permission entry
 *
 * A leaf mpte provides access permissions to certain
 * number of pages in a certain range of the mpte.
 * Each page has an permission entry stored in mpte.
 * The number of entries is different between the modes
 * supported by MPT. Smmpt34 supports 8 entries and
 * Smmpt43/52/64 supports 16 entries.
 */
/* XWR entry(n) shift bits */
static inline u32 sbi_mpte_xwr_shift(u32 n)
{
	return SBI_MPTE_XWR_BASE + n * SBI_MPTE_XWR_WIDTH;
}

/* Get the XWR permissions for any entry(n) in mpte */
static inline u8 sbi_mpte_leaf_get_xwr(unsigned long mpte, u32 n)
{
	return (u8)((mpte >> sbi_mpte_xwr_shift(n)) & SBI_MPTE_XWR_MASK);
}

/* Set the XWR permissions in mpte for an entry(n) */
static inline unsigned long sbi_mpte_leaf_set_xwr(unsigned long mpte, u32 n, u8 xwr)
{
	u32 shift = sbi_mpte_xwr_shift(n);
	mpte &= ~(SBI_MPTE_XWR_MASK << shift);
	mpte |=  ((unsigned long)(xwr & SBI_MPTE_XWR_MASK)) << shift;
	return mpte;
}

/**
 * Fence helper functions
 */

/*
 * rs1=0 and rs2=0
 * Orders all reads and writes to any level of the MPTs for all
 * supervisor domain address spaces. Use after updating MPT structures
 * for any/all supervisor domains.
 */
static inline void sbi_mpt_mfence_all(void)
{
	asm volatile("mfence.pa x0, x0" ::: "memory");
}

/**
 * rs1=0 and rs2=sdid
 * Orders all reads and writes to any level of the MPT for the single
 * supervisor domain address space identified by sdid.
 * Use after updating MPT structures for one specific supervisor domain.
 *
 */
static inline void sbi_mpt_mfence_sdid(u32 sdid)
{
	unsigned long s = (unsigned long)(sdid & SBI_MPT_SDID_MASK);
	asm volatile("mfence.pa x0, %0" :: "r"(s) : "memory");
}

/**
 * rs1=paddr and rs2=x0
 * Orders all reads and writes to the leaf MPT entries that correspond
 * to the physical address paddr, for all supervisor domain address
 * spaces.
 * Use after updating a single page's MPT leaf entry across all domains.
 */
static inline void sbi_mpt_mfence_addr(unsigned long paddr)
{
	asm volatile("mfence.pa %0, x0" :: "r" (paddr) : "memory");
}

/**
 * rs1=paddr and rs2=sdid
 * Orders all reads and writes to the leaf MPT entries that correspond
 * to the physical address paddr, for the single supervisor domain
 * address space identified by sdid.
 * Use when updating one page in one domain.
 */
static inline void sbi_mpt_mfence_pa(unsigned long paddr, unsigned long sdid)
{
	unsigned long s = (unsigned long)(sdid & SBI_MPT_SDID_MASK);
	asm volatile("mfence.pa %0, %1" :: "r" (paddr), "r" (s) : "memory");
}

/**
 * rs1=0 and rs2=0
 * Invalidates all cached MPT entries for all supervisor domains for safe
 * global invalidation.
 */
static inline void sbi_mpt_minval_all(void)
{
	/* Prior stores globally visible to current hart before invalidation */
	asm volatile("sfence.w.inval" ::: "memory");

	asm volatile("minval.pa x0, x0" ::: "memory");

	/* Prior invalidation using minval.pa completes before
	 * subsequent implicit accesses by hart to mpt table */
	asm volatile("sfence.inval.ir" ::: "memory");
}

/**
 * rs1=paddr and rs2=sdid
 * Invalidates cached MPT entries for a given physical address paddr and
 * supervisor domain sdid.
 */
static inline void sbi_mpt_minval_sdid(unsigned long pa, u32 sdid)
{
	unsigned long s = (unsigned long)(sdid & SBI_MPT_SDID_MASK);

	asm volatile("sfence.w.inval" ::: "memory");
	asm volatile("minval.pa %0, %1" :: "r"(pa), "r"(s) : "memory");
	asm volatile("sfence.inval.ir" ::: "memory");
}

/**
 * Memory region mapped in a supervisor domain which is
 * programed in that domain mpt table
 */
struct sbi_mpt_region {
	unsigned long	pa;
	unsigned long	size;
	u8		xwr;
	bool		locked;
};

struct sbi_mpt_domain {
	u32			sdid;
	bool			valid;
	unsigned long		root_pa;
	struct sbi_mpt_mode	*mode;
	struct sbi_domain	*sbi_dom;
	unsigned int		nregions;
	struct sbi_mpt_region	regions[SBI_MPT_MAX_REGIONS_DOMAIN];
	spinlock_t		lock;
};

/**
 * MPT mode context
 * Describes the settings of different Smmtt modes —
 * Smmpt34, Smmpt43, Smmpt52, Smmpt64
 */
struct sbi_mpt_mode {
	/* mpt modE name */
	const char	*name;

	/* mmpt mode value */
	unsigned long	mode_val;

	/* assembles mmpt csr value as per the mpt format */
	unsigned long	(*encode_mmpt)(unsigned long ppn, u32 sdid);

	/* Walks the mpt table, allocating intermediate and leaf table pages
	 * as needed, and writes xwr permissions into every XWR tuple in mpte
	 * covering every page in [pa, pa+size] region
	 *
	 * It can also be used to revoke the permission by setting
	 * SBI_MPT_PERM_NONE to the XWR tuples */
	int		(*map_range)(struct sbi_mpt_domain *dom,
				     unsigned long pa, unsigned long size,
				     u8 xwr);
	unsigned long	(*root_table_size)(void);
	unsigned long	(*root_table_align)(void);
	bool		(*pa_in_range)(unsigned long pa, unsigned long size);
};

struct sbi_mpt_ctrl {
	bool			ready;
	struct sbi_mpt_mode	*mode;
	u32			sdidlen;
	u32			max_domains;
	/* bit set -> SDID is free and vice-versa */
	DECLARE_BITMAP(sdid_bitmap, SBI_MPT_MAX_DOMAINS);
	unsigned long		fw_pa;
	unsigned long		fw_size;
	struct sbi_mpt_domain	domains[SBI_MPT_MAX_DOMAINS];
	u32			ndomain;
	/*
	 * Per-hart scratch slot for active SDID tracking.
	 * Written by sbi_mpt_hart_activate/deactivate().
	 * Read by sbi_mpt_thishart_sdid() without CSR access.
	 */
	unsigned long		sdid_offset;
};

struct sbi_mpt_domain_config {
	struct sbi_domain	*sbi_dom;
	u8			xwr;
	bool			fw_protect;
	struct sbi_mpt_region	*regions;
	u32			nregions;
};

/**
 * Per-hart SDID query
 */
struct sbi_mpt_ctrl *sbi_mpt_ctrl_get(void);

/*
 * sbi_mpt_thishart_sdid() — SDID active on the calling hart.
 * Reads per-hart scratch slot; no CSR access, no domain array scan.
 * Returns SBI_MPT_SDID_INVALID if MPT not initialised or hart has
 * not activated any domain (MODE=BARE).
 */
static inline u32 sbi_mpt_thishart_sdid(void)
{
	struct sbi_mpt_ctrl *ctrl = sbi_mpt_ctrl_get();
	u32 *p;

	if (!ctrl->ready || !ctrl->sdid_offset)
		return (u32)SBI_MPT_SDID_INVALID;

	p = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(), ctrl->sdid_offset);

	return *p;
}

/**
 * sbi_mpt_init() — cold-boot initialisation.
 */
int sbi_mpt_init(void);

int sbi_mpt_domain_create(const struct sbi_mpt_domain_config *cfg, u32 *out_sdid);

/* Runtime Region Management */
int  sbi_mpt_domain_add_region(u32 sdid, unsigned long pa, unsigned long size, u8 xwr);
int  sbi_mpt_domain_remove_region(u32 sdid, unsigned long pa, unsigned long size);

int  sbi_mpt_hart_activate(u32 sdid);
void sbi_mpt_hart_deactivate(void);
int  sbi_mpt_hart_activate_for_domain(struct sbi_domain *sbi_dom);

struct sbi_mpt_domain *sbi_mpt_domain_get(u32 sdid);

/*
 * sbi_mpt_pool_alloc() — allocate a page-aligned table from the
 * dedicated MPT heap. Called from scheme walkers and
 * sbi_mpt_domain_create().
 */
unsigned long sbi_mpt_pool_alloc(unsigned long size, unsigned long align);

#endif /* !__RISCV_MPT_H__ */
