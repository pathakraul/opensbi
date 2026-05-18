/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#include <sbi/riscv_mpt.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_console.h>

#if __riscv_xlen == 64
/**
 * Constants and Defines
 */
#define RV64_INNER_ENTRIES	512U
#define RV64_MPTE_SIZE		8UL
#define RV64_TABLE_SIZE		(RV64_INNER_ENTRIES * RV64_MPTE_SIZE)	/* 4KiB */

#define RV64_SMMPT43_ROOT_SIZE	RV64_TABLE_SIZE
#define RV64_SMMPT52_ROOT_SIZE	RV64_TABLE_SIZE
/*
 * Smmpt64 root: 4096 entries × 8B = 32KiB. All other tables (inner
 * and leaf for all three modes) are RV64_TABLE_SIZE = 4KiB.
 */
#define RV64_SMMPT64_ROOT_ENTRIES	4096U
#define RV64_SMMPT64_ROOT_SIZE		(RV64_SMMPT64_ROOT_ENTRIES * RV64_MPTE_SIZE) /* 32KiB */

#define RV64_NUMPGINRANGE	4U
#define RV64_PAGES_LEAF		(1U << RV64_NUMPGINRANGE)
#define RV64_PG_MASK		(RV64_PAGES_LEAF - 1U)
#define RV64_INNER_IDX_BITS	9U

/**
 * RV64_LEAF_RANGE — bytes covered by one complete leaf table.
 * = pages_per_leaf × PAGE_SIZE = 16 × 4KiB = 64KiB = 2^16.
 * Used as the base for rv64_mpte_range() to derive per-level coverage.
 */
#define RV64_LEAF_RANGE		(RV64_PAGES_LEAF * SBI_MPT_PAGE_SIZE)  /* 64KiB */

/**
 * Per-scheme mmpt encoding
 *
 * RV64: mmpt[63:60]=MODE, mmpt[57:52]=SDID, mmpt[43:0]=PPN
 */
static unsigned long smmpt43_encode_mmpt(unsigned long ppn, u32 sdid)
{
    return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT43, sdid, ppn);
}

static unsigned long smmpt52_encode_mmpt(unsigned long ppn, u32 sdid)
{
	return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT52, sdid, ppn);
}

/**
 * Smmpt64 has ppn = root_pa >> 12. The root is 32KiB-aligned as per the spec
 * so (root_pa >> 12) has bits [2:0] = 0.
 */
static unsigned long smmpt64_encode_mmpt(unsigned long ppn, u32 sdid)
{
	return sbi_mmpt_encode(SBI_MMPT_MODE_SMMPT64, sdid, ppn);
}

/**
 * Per-mode layout functions
 */
static unsigned long smmpt43_root_table_size(void)
{
	return RV64_SMMPT52_ROOT_SIZE;
}

static unsigned long smmpt43_root_table_align(void)
{
	return SBI_MPT_PAGE_SIZE;
}

static bool smmpt43_pa_in_range(unsigned long pa, unsigned long size)
{
	unsigned long pa_max = UL(1) << 43;
	return (pa < pa_max && size <= pa_max - pa);
}

static unsigned long smmpt52_root_table_size(void)
{
	return RV64_SMMPT52_ROOT_SIZE;
}

static unsigned long smmpt52_root_table_align(void)
{
	return SBI_MPT_PAGE_SIZE;
}

static bool smmpt52_pa_in_range(unsigned long pa, unsigned long size)
{
	unsigned long pa_max = UL(1) << 52;
	return (pa < pa_max && size <= pa_max - pa);
}

static unsigned long smmpt64_root_table_size(void)
{
	return RV64_SMMPT64_ROOT_SIZE;
}

static unsigned long smmpt64_root_table_align(void)
{
	return RV64_SMMPT64_ROOT_SIZE;
}

static bool smmpt64_pa_in_range(unsigned long pa, unsigned long size)
{
	/**
	 * Smmpt64 covers the full 64-bit PA space. Every address is valid.
	 * No current RV64 hart can generate a PA above 2^56 which is currently
	 * the highest PA length defined for RV64
	 */
	return true;
}

/**
 * PA field extraction
 */

/*
 * rv64_pn() — extract pn[level] from a PA.
 *
 * shift = 16 + level × 9  (consistent for all levels 0..4)
 * mask  = 0x1FF (9 bits) for levels 0..3 and for non-Smmpt64 modes
 *	   0xFFF (12 bits) for Smmpt64 root (level 4 only)
 */
static inline u32 rv64_pn(unsigned long pa, u32 level,
			    const struct sbi_mpt_mode *sch)
{
	u32 shift = 16U + level * 9U;
	u32 mask  = (sch->mode_val == SBI_MMPT_MODE_SMMPT64 && level == 4U)
		    ? 0xFFFU : 0x1FFU;

	return (u32)((pa >> shift) & mask);
}

/*
 * rv64_pi_shift() — XWR tuple index (pi) shift at level i.
 */
static inline u32 rv64_pi_shift(u32 level)
{
	return SBI_MPT_PAGE_SHIFT + level * RV64_INNER_IDX_BITS;
}

static inline u32 rv64_pi(unsigned long pa, u32 level)
{
	return (u32)((pa >> rv64_pi_shift(level)) & RV64_PG_MASK);
}

/*
 * rv64_mpte_range() — bytes covered by one MPTE at the given level.
 */
static inline unsigned long rv64_mpte_range(u32 level)
{
	return RV64_LEAF_RANGE << (level * RV64_INNER_IDX_BITS);
}

/**
 * 8-byte MPTE I/O
 */
static inline u64 read_8byte_mpte(unsigned long pa)
{
	return *(u64 *)pa;
}
static inline void write_8byte_mpte(unsigned long pa, u64 v)
{
	*(u64 *)pa = v;
}

static inline unsigned long entry_pa64(unsigned long table_pa, u32 idx)
{
	return table_pa + (unsigned long)idx * RV64_MPTE_SIZE;
}

static inline unsigned long nl_next_pa64(u64 mpte)
{
	return (unsigned long)((u64)(mpte >> SBI_MPTE_PPN_SHIFT)
			       << SBI_MPT_PAGE_SHIFT);
}

/*
 * Best-level selection
 *
 * Returns the highest level at which the range [pa, pa+size] can be
 * covered by a single leaf MPTE (V=1, L=1).
 */

static u32 rv64_best_level(unsigned long pa, unsigned long size,
			    u32 top_level)
{
	u32 lvl;

	for (lvl = top_level; lvl >= 1U; lvl--) {
		unsigned long range = rv64_mpte_range(lvl);

		if (size >= range && (pa & (range - UL(1))) == 0)
			return lvl;
	}

	return 0;
}

/*
 * Generic N-level walk with MPT table allocation
 */
static unsigned long rv64_walk_alloc(struct sbi_mpt_domain *dom,
				      unsigned long pa,
				      unsigned long size,
				      u32 *out_level,
				      u32 top_level)
{
	const struct sbi_mpt_mode *sch = dom->mode;
	u32 best_lvl = rv64_best_level(pa, size, top_level);
	u32 lvl, idx;
	u64 mpte;
	unsigned long table_pa = dom->root_pa;
	unsigned long ep, new_pa;

	for (lvl = top_level; lvl >= 1U; lvl--) {
		idx = rv64_pn(pa, lvl, sch);
		ep = entry_pa64(table_pa, idx);
		mpte = read_8byte_mpte(ep);

		if (mpte & SBI_MPTE_L) {
			*out_level = lvl;
			return ep;
		}

		if (!(mpte & SBI_MPTE_V)) {
			if (lvl <= best_lvl) {
				/*
				 * Fresh entry at or below best level.
				 * map_range will install V=1, L=1 and XWR.
				 */
				*out_level = lvl;
				return ep;
			}
			/*
			 * Need next-level table. All inner tables including
			 * those under Smmpt64's level-4 root are 4KiB.
			 */
			new_pa = sbi_mpt_pool_alloc(
						RV64_TABLE_SIZE,
						SBI_MPT_PAGE_SIZE);
			if (!new_pa)
				return 0;
			write_8byte_mpte(ep, (u64)pa_to_mpte_nonleaf(new_pa));
			table_pa = new_pa;
		} else {
			/* Valid non-leaf — descend */
			table_pa = nl_next_pa64(mpte);
		}
	}

	/* Level 0: leaf table */
	*out_level = 0U;
	return entry_pa64(table_pa, rv64_pn(pa, 0U, sch));
}

/**
 * Generic map_range — shared by Smmpt43, Smmpt52, Smmpt64
 *
 * Iterates [pa, pa+size] in MPTE-sized batches. At each iteration:
 *
 * TODO: Handle the NAPOT Regions
 */

static int rv64_map_range(struct sbi_mpt_domain *dom,
			        unsigned long pa, unsigned long size,
			        u8 xwr, u32 top_level)
{
	unsigned long lmpte_pa, mpte_range, mpte_base, mpte_end, batch_end;
	u32 pg_first, pg_last, pg;
	u64 mpte;
	unsigned long cur = pa;
	unsigned long end = pa + size;

	while (cur < end) {
		u32 level;
		lmpte_pa = rv64_walk_alloc(dom, cur, end - cur, &level, top_level);
		if (!lmpte_pa)
			return SBI_ENOMEM;

		mpte_range = rv64_mpte_range(level);
		pg_first = rv64_pi(cur, level);
		mpte_base = cur & ~(mpte_range - UL(1));
		mpte_end = mpte_base + mpte_range;
		batch_end = (end < mpte_end) ? end : mpte_end;
		pg_last = rv64_pi(batch_end - SBI_MPT_PAGE_SIZE, level);

		mpte = read_8byte_mpte(lmpte_pa);
		if (!(mpte & (u64)(SBI_MPTE_V | SBI_MPTE_L)))
			mpte = (u64)(SBI_MPTE_V | SBI_MPTE_L);

		for (pg = pg_first; pg <= pg_last; pg++)
			mpte = (u64)sbi_mpte_leaf_set_xwr(
					(unsigned long)mpte, pg, xwr);

		write_8byte_mpte(lmpte_pa, mpte);
		cur = batch_end;
	}

	return 0;
}

/**
 * Per-mode map_range wrappers
 *
 * rv64_map_range() requires the walk depth (top_level).
 *
 * Smmpt43: root at level 2, 9-bit index, 512 entries, 4KiB root)
 * Smmpt52: root at level 3, 9-bit index, 512 entries, 4KiB root)
 * Smmpt64: root at level 4, 12-bit index, 4096 entries, 32KiB root)
 */

static int smmpt43_map_range(struct sbi_mpt_domain *dom, unsigned long pa,
				unsigned long size, u8 xwr)
{
	return rv64_map_range(dom, pa, size, xwr, 2U);
}

static int smmpt52_map_range(struct sbi_mpt_domain *dom, unsigned long pa,
				unsigned long size, u8 xwr)
{
	return rv64_map_range(dom, pa, size, xwr, 3U);
}

static int smmpt64_map_range(struct sbi_mpt_domain *dom, unsigned long pa,
				unsigned long size, u8 xwr)
{
	return rv64_map_range(dom, pa, size, xwr, 4U);
}
/**
 *
 * Mode descriptors
 *
 * detect_scheme() in sbi_mpt.c probes highest-first via WARL:
 * Smmpt64 → Smmpt52 → Smmpt43
 */
struct sbi_mpt_mode smmpt43_mode = {
	.name			= "Smmpt43",
	.mode_val		= SBI_MMPT_MODE_SMMPT43,
	.encode_mmpt		= smmpt43_encode_mmpt,
	.map_range		= smmpt43_map_range,
	.root_table_size	= smmpt43_root_table_size,
	.root_table_align	= smmpt43_root_table_align,
	.pa_in_range		= smmpt43_pa_in_range,
};

struct sbi_mpt_mode smmpt52_mode = {
	.name			= "Smmpt52",
	.mode_val		= SBI_MMPT_MODE_SMMPT52,
	.encode_mmpt		= smmpt52_encode_mmpt,
	.map_range		= smmpt52_map_range,
	.root_table_size	= smmpt52_root_table_size,
	.root_table_align	= smmpt52_root_table_align,
	.pa_in_range		= smmpt52_pa_in_range,
};

struct sbi_mpt_mode smmpt64_mode = {
	.name			= "Smmpt64",
	.mode_val		= SBI_MMPT_MODE_SMMPT64,
	.encode_mmpt		= smmpt64_encode_mmpt,
	.map_range		= smmpt64_map_range,
	.root_table_size	= smmpt64_root_table_size,
	.root_table_align	= smmpt64_root_table_align,
	.pa_in_range		= smmpt64_pa_in_range,
};
#else

/* Smmpt34 */

#endif
