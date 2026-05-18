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

struct sbi_domain;
struct sbi_mpt_domain;
struct sbi_mpt_ctrl;
struct sbi_domain_memregion;

#define SBI_CSR_MMPT			0x382
#define SBI_CSR_MSDCFG			0x74E

/**
 * mmpt register layout
 * RV32: [31:30]MODE | [29:28]WARL | [27:22]SDID | [21:0]PPN
 * RV64: [63:60]MODE | [59:58]WARL | [57:52]SDID | [51:44]WARL | [43:0]PPN
 */

#if __riscv_xlen == 64
#define SBI_MMPT_MODE_SHIFT		60
#define SBI_MMPT_MODE_MASK		(UL(0xF)   << SBI_MMPT_MODE_SHIFT)
#define SBI_MMPT_SDID_SHIFT		52
#define SBI_MMPT_SDID_MASK		(UL(0x3F)  << SBI_MMPT_SDID_SHIFT)
#define SBI_MMPT_PPN_MASK		UL(0x00000FFFFFFFFFFF)   /* 44 bits [43:0] */
#else
#define SBI_MMPT_MODE_SHIFT		30
#define SBI_MMPT_MODE_MASK		(UL(0x3)   << SBI_MMPT_MODE_SHIFT)
#define SBI_MMPT_SDID_SHIFT		22
#define SBI_MMPT_SDID_MASK		(UL(0x3F)  << SBI_MMPT_SDID_SHIFT)
#define SBI_MMPT_PPN_MASK		UL(0x3FFFFF)             /* 22 bits [21:0] */
#endif

#define SBI_MMPT_MODE_BARE		0
#if __riscv_xlen == 64
# define SBI_MMPT_MODE_SMMPT43		1
# define SBI_MMPT_MODE_SMMPT52		2
# define SBI_MMPT_MODE_SMMPT64		3
#else
# define SBI_MMPT_MODE_SMMPT34		1
#endif

/* Maximum implemented SDID bit-width (SDIDMAX = 6 as per spec) */
#define MMPT_SDIDMAX			UL(6)
/* Maximum SDID value (supports up to 64 supervisor domains) */
#define MMPT_SDID_MAX_VAL		((UL(1) << MMPT_SDIDMAX) - UL(1))

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
#define SBI_MPTE_PPN_SHIFT		10
/* XWR permission start bit in leaf mpte */
#define SBI_MPTE_XWR_BASE		8
/* XWR permission width in leaf mpte */
#define SBI_MPTE_XWR_WIDTH		3
/* XWR permission mask in leaf mpte */
#define SBI_MPTE_XWR_MASK		UL(0x7)

#define SBI_MPT_PAGE_SHIFT		PAGE_SHIFT
#define SBI_MPT_PAGE_SIZE		PAGE_SIZE
#define SBI_MPT_PAGE_MASK		PAGE_MASK

static inline unsigned long sbi_mmpt_encode(unsigned long mode,
					    u32 sdid, unsigned long ppn)
{
	return ((mode << SBI_MMPT_MODE_SHIFT) & SBI_MMPT_MODE_MASK) |
	       ((_AT(unsigned long, sdid) << SBI_MMPT_SDID_SHIFT) & SBI_MMPT_SDID_MASK) |
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
static inline u32 mpte_xwr_shift(u32 n)
{
	return SBI_MPTE_XWR_BASE + n * SBI_MPTE_XWR_WIDTH;
}

/* Get the XWR permissions for any entry(n) in mpte */
static inline u8 mpte_leaf_get_xwr(unsigned long mpte, u32 n)
{
	return (u8)((mpte >> mpte_xwr_shift(n)) & SBI_MPTE_XWR_MASK);
}

/* Set the XWR permissions in mpte for an entry(n) */
static inline unsigned long mpte_leaf_set_xwr(unsigned long mpte, u32 n, u8 xwr)
{
	u32 shift = mpte_xwr_shift(n);
	mpte &= ~(SBI_MPTE_XWR_MASK << shift);
	mpte |=  ((unsigned long)(xwr & SBI_MPTE_XWR_MASK)) << shift;
	return mpte;
}

/**
 * XWR permission constants  (XWR=000 = "no access")
 */
#define SBI_MPT_PERM_NONE	0
#define SBI_MPT_PERM_R		1
#define SBI_MPT_PERM_W		2
#define SBI_MPT_PERM_X		4
#define SBI_MPT_PERM_RW		(SBI_MPT_PERM_R | SBI_MPT_PERM_W)
#define SBI_MPT_PERM_RX		(SBI_MPT_PERM_R | SBI_MPT_PERM_X)
#define SBI_MPT_PERM_RWX	(SBI_MPT_PERM_R | SBI_MPT_PERM_W | SBI_MPT_PERM_X)

/*
 * SDID constants
 */

#define SBI_MPT_SDIDMAX_BITS		6
#define SBI_MPT_MAX_SDID		((_UL(1) << SBI_MPT_SDIDMAX_BITS) - 1)
#define SBI_MPT_MAX_DOMAINS		(SBI_MPT_MAX_SDID + 1)
#define SBI_MPT_SDID_INVALID    	(SBI_MPT_MAX_SDID + UL(1))

/**
 * Software bound on the number of regions an supervisor domain
 * may support. One region for firmware and one region for mpt table itself and
 * rest for the general memory regions
 */
#define SBI_MPT_MAX_REGIONS_DOMAIN	32

/**
 * Fence helper functions
 */

/*
 * rs1=0 and rs2=0
 * Orders all reads and writes to any level of the MPTs for all
 * supervisor domain address spaces. Use after updating MPT structures
 * for any/all supervisor domains.
 */
static inline void mpt_mfence_pa_all(void)
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
static inline void mpt_mfence_pa_sdid(u32 sdid)
{
	unsigned long s = (unsigned long)(sdid & SBI_MPT_MAX_SDID);
	asm volatile("mfence.pa x0, %0" :: "r"(s) : "memory");
}

/**
 * rs1=paddr and rs2=x0
 * Orders all reads and writes to the leaf MPT entries that correspond
 * to the physical address paddr, for all supervisor domain address
 * spaces.
 * Use after updating a single page's MPT leaf entry across all domains.
 */
static inline void mpt_mfence_pa_addr(unsigned long paddr)
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
static inline void mpt_mfence_pa(unsigned long paddr, unsigned long sdid)
{
	unsigned long s = (unsigned long)(sdid & SBI_MPT_MAX_SDID);
	asm volatile("mfence.pa %0, %1" :: "r" (paddr), "r" (sdid) : "memory");
}

/**
 * rs1=0 and rs2=0
 * Invalidates all cached MPT entries for all supervisor domains for safe
 * global invalidation.
 */
static inline void mpt_minval_pa_all(void)
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
static inline void mpt_minval_pa_sdid(unsigned long pa, u32 sdid)
{
	unsigned long s = (unsigned long)(sdid & SBI_MPT_MAX_SDID);

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

/**
 * MPT scheme context
 * Describes the settings of different Smmtt schemes —
 * Smmpt34, Smmpt43, Smmpt52, Smmpt64
 */
struct sbi_mpt_scheme {
	/* mpt scheme name */
	const char	*name;

	/* mmpt mode value for the mpt scheme */
	unsigned long	mode_val;
	/* mpt scheme physical address width 34,43,52,64 */
	u32		pa_bits;

	/* number of levels in mpt */
	u32		levels;

	/*
	 * inner_idx_bits: bits indexing each intermediate table level.
	 * Always 9 for current schemes. Kept explicit for root_idx
	 * computation and registration sanity check.
	 */
	u32            inner_idx_bits;

	/* size in bytes of one MPTE.
	 * Smmpt34 -> 4
	 * Smmpt43/52/64 -> 8
	 */
	u32		mpte_size;

	/* number of 4 KiB pages covered by one leaf MPTE,
	 * Smmpt34 -> 8
	 * Smmpt43/52/64 -> 16
	 */
	u32		pages_per_leaf;

	/* assembles mmpt csr value as per the mpt format */
	unsigned long (*encode_mmpt)(unsigned long ppn, u32 sdid);

	/* Walks the mpt table, allocating intermediate and leaf table pages
	 * as needed, and writes xwr permissions into every XWR tuple in mpte
	 * covering every page in [pa, pa+size] region
	 *
	 * It can also be used to revoke the permission by setting
	 * SBI_MPT_PERM_NONE to the XWR tuples */
	int  (*map_range)(struct sbi_mpt_domain *dom,
			  unsigned long pa, unsigned long size, u8 xwr);
};

struct sbi_mpt_domain {
	u32                    sdid;
	bool                   valid;
	unsigned long          root_pa;
	struct sbi_mpt_scheme *scheme;
	struct sbi_domain     *sbi_dom;
	unsigned int           nregions;
	struct sbi_mpt_region  regions[SBI_MPT_MAX_REGIONS_DOMAIN];
	spinlock_t         lock;
};

struct sbi_mpt_pool {
	/*
	 * Heap approach: tables allocated individually via
	 * sbi_aligned_alloc_from(&mpt_hpctrl, align, size).
	 * No contiguous base/watermark. allocated tracks total
	 * bytes in use for sbi_mpt_dump() reporting.
	 */
	//unsigned long  base;
	//unsigned long  size;
	//unsigned long  watermark;
	unsigned long allocated;
	spinlock_t lock;
};

struct sbi_mpt_ctrl {
	bool                   ready;
	struct sbi_mpt_scheme *scheme;
	struct sbi_mpt_pool    pool;
	unsigned long          fw_pa;
	unsigned long          fw_size;
	struct sbi_mpt_domain  domains[SBI_MPT_MAX_DOMAINS];
	u32                    ndomain;
	unsigned long          sdid_offset;
};

static inline unsigned int smsdid_get_sdidlen(void)
{
	unsigned long mmpt_val, sdid_val;
	unsigned int  sdidlen = 0;

	mmpt_val = csr_read(CSR_MMPT);

	/* Write all-ones into SDID only */
	csr_write(CSR_MMPT, SBI_MMPT_SDID_MASK << SBI_MMPT_SDID_SHIFT);
	sdid_val = (csr_read(CSR_MMPT) & SBI_MMPT_SDID_MASK) >> SBI_MMPT_SDID_SHIFT;

	/* Count implemented bits from LSB */
	while ((sdid_val & 1UL) && (sdidlen < MMPT_SDIDMAX)) {
		sdidlen+=1;
		sdid_val >>= 1;
	}

	csr_write(CSR_MMPT, mmpt_val);
	return sdidlen;
}

/**
 * Scheme inline helpers
 *
 * sbi_mpt_log2(n): integer log2 of a power-of-2 value n.
 * count trailing zeros = log2 for powers of 2).
 */

static inline u32 sbi_mpt_log2(unsigned long n)
{
	return (u32)__builtin_ctzl(n);
}

/**
 * sbi_mpt_leaf_idx_bits() — bits indexing one leaf table.
 * A leaf table is one PAGE_SIZE page with PAGE_SIZE/mpte_size entries.
 *   index_bits = log2(PAGE_SIZE / mpte_size)
 *   RV32 (mpte=4B):   log2(1024) = 10
 *   RV64 (mpte=8B):   log2(512)  =  9
 */
static inline u32 sbi_mpt_leaf_idx_bits(const struct sbi_mpt_scheme *s)
{
	return sbi_mpt_log2(SBI_MPT_PAGE_SIZE / (unsigned long)s->mpte_size);
}

/**
 * sbi_mpt_root_idx_bits() — bits indexing the root table.
 * numpginrange = log2(pages_per_leaf) — XWR tuple selector width.
 *   RV32: log2(8)=3   RV64: log2(16)=4
 *
 * Derivation for each scheme:
 *   Smmpt34: 34-12-3-10-0×9 = 9  (512 root entries)
 *   Smmpt43: 43-12-4-9-1×9  = 9  (512 root entries)
 *   Smmpt52: 52-12-4-9-2×9  = 9  (512 root entries)
 *   Smmpt64: 64-12-4-9-3×9  = 12 (4096 root entries, 32KiB table)
 */
static inline u32 sbi_mpt_root_idx_bits(const struct sbi_mpt_scheme *s)
{
	u32 numpginrange = sbi_mpt_log2((unsigned long)s->pages_per_leaf);

	return s->pa_bits
	       - SBI_MPT_PAGE_SHIFT
	       - numpginrange
	       - sbi_mpt_leaf_idx_bits(s)
	       - (s->levels - 2U) * s->inner_idx_bits;
}

/**
 * sbi_mpt_root_size() — root table size in bytes.
 *   = 2^root_idx_bits × mpte_size
 *   = 1 << (root_idx_bits + log2(mpte_size))
 *
 * Smmpt43/52: 4KiB (one page)
 * Smmpt34:    2KiB (fits in one page; pool allocates one page minimum)
 * Smmpt64:   32KiB (spec mandates 32KiB alignment; mmpt.PPN[2:0]=0)
 */
static inline unsigned long sbi_mpt_root_size(const struct sbi_mpt_scheme *s)
{
	u32 idx = sbi_mpt_root_idx_bits(s);
	u32 msh = sbi_mpt_log2((unsigned long)s->mpte_size);

	return UL(1) << (idx + msh);
}

/**
 * sbi_mpt_root_align() — root table alignment requirement.
 * Equals root_size when root_size > PAGE_SIZE (Smmpt64 = 32KiB).
 * Otherwise PAGE_SIZE (heap allocator minimum granularity).
 */
static inline unsigned long sbi_mpt_root_align(const struct sbi_mpt_scheme *s)
{
	unsigned long rs = sbi_mpt_root_size(s);

	return (rs > SBI_MPT_PAGE_SIZE) ? rs : SBI_MPT_PAGE_SIZE;
}

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
 * Called from the platform_early_init.
 *
 * Ordering inside this function:
 *   Stage 1: sbi_hart_has_extension(SMSDID) — safe ISA string check
 *   Stage 2: sbi_hart_has_extension(SMMPT)  — safe ISA string check
 *   Stage 3: pool validation (NAPOT + fw overlap)
 *   Stage 4: scheme register + WARL probe (safe: Smsdid confirmed)
 *
 * Returns SBI_ENODEV if SMSDID or SMMPT absent from ISA string,
 *         SBI_EINVAL if pool misaligned or overlaps firmware.
 */
int sbi_mpt_init(unsigned long fw_pa,   unsigned long fw_size);

int sbi_mpt_domain_create(struct sbi_domain *sbi_dom, u8 xwr_default, u32 *out_sdid);

/* Phase 2+: runtime region management */
int  sbi_mpt_domain_add_region(u32 sdid, unsigned long pa, unsigned long size, u8 xwr);
int  sbi_mpt_domain_remove_region(u32 sdid, unsigned long pa, unsigned long size);

int  sbi_mpt_hart_activate(u32 sdid);
void sbi_mpt_hart_deactivate(void);
int  sbi_mpt_hart_activate_for_domain(struct sbi_domain *sbi_dom);

struct sbi_mpt_domain *sbi_mpt_domain_get(u32 sdid);
unsigned long sbi_mpt_pool_alloc(struct sbi_mpt_ctrl *ctrl, unsigned long size, unsigned long align);
int  sbi_mpt_scheme_register(struct sbi_mpt_scheme *scheme);
void sbi_mpt_dump(void);

#if __riscv_xlen == 64
void sbi_mpt_smmpt43_register(void);
void sbi_mpt_smmpt52_register(void);
void sbi_mpt_smmpt64_register(void);
#else
void sbi_mpt_smmpt34_register(void);
#endif

#endif /* !__RISCV_MPT_H__ */
