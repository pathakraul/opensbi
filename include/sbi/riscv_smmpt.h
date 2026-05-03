/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#ifndef __RISCV_SMMPT_H__
#define __RISCV_SMMPT_H__

#include <sbi/riscv_asm.h>
#include <sbi/riscv_encoding.h>

/* M-Mode Memory Protection Tables register */
#define CSR_MMPT			0x382

/* M-Mode Supervisor Domain Configuration register */
#define CSR_MSDCFG			0x74E

#define MMPT32_MODE_SHIFT		30
#define MMPT32_MODE_WIDTH		2
#define MMPT32_MODE_MASK		_UL(0x3)
#define MMPT32_MODE			(MMPT32_MODE_MASK << MMPT32_MODE_SHIFT)

#define MMPT32_SDID_SHIFT		22
#define MMPT32_SDID_WIDTH		6
#define MMPT32_SDID_MASK		_UL(0x3F)
#define MMPT32_SDID			(MMPT32_SDID_MASK << MMPT32_SDID_SHIFT)

#define MMPT32_PPN_SHIFT		0
#define MMPT32_PPN_WIDTH		22          /* 22 + 12 = 34-bit PA   */
#define MMPT32_PPN_MASK			_UL(0x3FFFFF)
#define MMPT32_PPN			(MMPT32_PPN_MASK << MMPT32_PPN_SHIFT)

#define MMPT64_MODE_SHIFT		60
#define MMPT64_MODE_WIDTH		4
#define MMPT64_MODE_MASK		_ULL(0xF)
#define MMPT64_MODE			(MMPT64_MODE_MASK << MMPT64_MODE_SHIFT)

#define MMPT64_SDID_SHIFT		52
#define MMPT64_SDID_WIDTH		6
#define MMPT64_SDID_MASK		_ULL(0x3F)
#define MMPT64_SDID			(MMPT64_SDID_MASK << MMPT64_SDID_SHIFT)

#define MMPT64_PPN_SHIFT		0
#define MMPT64_PPN_WIDTH		44		/* 44 + 12 = 56-bit PA*/
#define MMPT64_PPN_MASK			_ULL(0x00000FFFFFFFFFFF)
#define MMPT64_PPN			(MMPT64_PPN_MASK << MMPT64_PPN_SHIFT)

/* MMPT Mode encodings */
#define MMPT_MODE32_BARE		_UL(0)  /* No page-based MPT */
#define MMPT_MODE32_SMMPT34		_UL(1)  /* 34-bit PA protection (RV32) */

#define MMPT_MODE64_BARE		_UL(0)  /* No page-based MPT */
#define MMPT_MODE64_SMMPT43		_UL(1)  /* 43-bit PA protection (RV64) */
#define MMPT_MODE64_SMMPT52		_UL(2)  /* 52-bit PA protection (RV64) */
#define MMPT_MODE64_SMMPT64		_UL(3)  /* 64-bit PA protection (RV64) */

/* Maximum implemented SDID bit-width (SDIDMAX = 6 as per spec) */
#define MMPT_SDIDMAX			_UL(6)
/* Maximum SDID value (supports up to 64 supervisor domains) */
#define MMPT_SDID_MAX_VAL		((_UL(1) << MMPT_SDIDMAX) - _UL(1))

#define MMPT32_GET_MODE(v)	(((unsigned long)(v) >> MMPT32_MODE_SHIFT) & MMPT32_MODE_MASK)
#define MMPT32_GET_SDID(v)	(((unsigned long)(v) >> MMPT32_SDID_SHIFT) & MMPT32_SDID_MASK)
#define MMPT32_GET_PPN(v)	(((unsigned long)(v) >> MMPT32_PPN_SHIFT)  & MMPT32_PPN_MASK)

#define MMPT32_SET_MODE(v)	(((unsigned long)(v) & MMPT32_MODE_MASK) << MMPT32_MODE_SHIFT)
#define MMPT32_SET_SDID(v)	(((unsigned long)(v) & MMPT32_SDID_MASK) << MMPT32_SDID_SHIFT)
#define MMPT32_SET_PPN(v)	(((unsigned long)(v) & MMPT32_PPN_MASK) << MMPT32_PPN_SHIFT)

/* Make RV32 MMPT register value */
#define MMPT32_MAKE(mode, sdid, ppn)		\
	(MMPT32_SET_MODE(mode) | MMPT32_SET_SDID(sdid) | MMPT32_SET_PPN(ppn))

#define MMPT64_GET_MODE(v)	(((unsigned long long)(v) >> MMPT64_MODE_SHIFT) & MMPT64_MODE_MASK)
#define MMPT64_GET_SDID(v)	(((unsigned long long)(v) >> MMPT64_SDID_SHIFT) & MMPT64_SDID_MASK)
#define MMPT64_GET_PPN(v)	(((unsigned long long)(v) >> MMPT64_PPN_SHIFT) & MMPT64_PPN_MASK)

#define MMPT64_SET_MODE(v)	(((unsigned long long)(v) & MMPT64_MODE_MASK) << MMPT64_MODE_SHIFT)
#define MMPT64_SET_SDID(v)	(((unsigned long long)(v) & MMPT64_SDID_MASK) << MMPT64_SDID_SHIFT)
#define MMPT64_SET_PPN(v)	(((unsigned long long)(v) & MMPT64_PPN_MASK) << MMPT64_PPN_SHIFT)

/* Make RV64 MMPT register value */
#define MMPT64_MAKE(mode, sdid, ppn)		\
	(MMPT64_SET_MODE(mode) | MMPT64_SET_SDID(sdid) | MMPT64_SET_PPN(ppn))

/*
 * Smmpt64 mode special alignment requirement for RV64:
 * When MODE = SMMPT64, the root MPT page must be aligned to a 32 KiB
 * boundary. Bits[2:0] of mmpt.PPN always read as zero in this mode.
 */
#define MMPT64_SMMPT64_PPN_ALIGN_BITS		_ULL(3)
#define MMPT64_SMMPT64_PPN_ALIGN_MASK		\
	((_ULL(1) << MMPT64_SMMPT64_PPN_ALIGN_BITS) - _ULL(1))

#if __riscv_xlen == 64

#define MMPT_MODE_SHIFT         MMPT64_MODE_SHIFT
#define MMPT_MODE_WIDTH         MMPT64_MODE_WIDTH
#define MMPT_MODE_MASK          MMPT64_MODE_MASK
#define MMPT_MODE               MMPT64_MODE

#define MMPT_SDID_SHIFT         MMPT64_SDID_SHIFT
#define MMPT_SDID_WIDTH         MMPT64_SDID_WIDTH
#define MMPT_SDID_MASK          MMPT64_SDID_MASK
#define MMPT_SDID               MMPT64_SDID

#define MMPT_PPN_SHIFT          MMPT64_PPN_SHIFT
#define MMPT_PPN_WIDTH          MMPT64_PPN_WIDTH
#define MMPT_PPN_MASK           MMPT64_PPN_MASK
#define MMPT_PPN                MMPT64_PPN

#define MMPT_GET_MODE(v)        MMPT64_GET_MODE(v)
#define MMPT_GET_SDID(v)        MMPT64_GET_SDID(v)
#define MMPT_GET_PPN(v)         MMPT64_GET_PPN(v)

#define MMPT_SET_MODE(v)        MMPT64_SET_MODE(v)
#define MMPT_SET_SDID(v)        MMPT64_SET_SDID(v)
#define MMPT_SET_PPN(v)         MMPT64_SET_PPN(v)

#define MMPT_MAKE(mode, sdid, ppn)  MMPT64_MAKE(mode, sdid, ppn)

/* BARE is 0 on both, but alias through the typed constant for consistency */
#define MMPT_MODE_BARE          MMPT_MODE64_BARE

#else /* __riscv_xlen == 32 */

#define MMPT_MODE_SHIFT         MMPT32_MODE_SHIFT
#define MMPT_MODE_WIDTH         MMPT32_MODE_WIDTH
#define MMPT_MODE_MASK          MMPT32_MODE_MASK
#define MMPT_MODE               MMPT32_MODE

#define MMPT_SDID_SHIFT         MMPT32_SDID_SHIFT
#define MMPT_SDID_WIDTH         MMPT32_SDID_WIDTH
#define MMPT_SDID_MASK          MMPT32_SDID_MASK
#define MMPT_SDID               MMPT32_SDID

#define MMPT_PPN_SHIFT          MMPT32_PPN_SHIFT
#define MMPT_PPN_WIDTH          MMPT32_PPN_WIDTH
#define MMPT_PPN_MASK           MMPT32_PPN_MASK
#define MMPT_PPN                MMPT32_PPN

#define MMPT_GET_MODE(v)        MMPT32_GET_MODE(v)
#define MMPT_GET_SDID(v)        MMPT32_GET_SDID(v)
#define MMPT_GET_PPN(v)         MMPT32_GET_PPN(v)

#define MMPT_SET_MODE(v)        MMPT32_SET_MODE(v)
#define MMPT_SET_SDID(v)        MMPT32_SET_SDID(v)
#define MMPT_SET_PPN(v)         MMPT32_SET_PPN(v)

#define MMPT_MAKE(mode, sdid, ppn)  MMPT32_MAKE(mode, sdid, ppn)

#define MMPT_MODE_BARE          MMPT_MODE32_BARE

#endif /* __riscv_xlen */

#define MSDCFG_SIDN_SHIFT		0
#define MSDCFG_SIDN_WIDTH		6
#define MSDCFG_SIDN_MASK		_UL(0x3F)
#define MSDCFG_SIDN			(MSDCFG_SIDN_MASK << MSDCFG_SIDN_SHIFT)

#define MSDCFG_SEDA_SHIFT		6
#define MSDCFG_SEDA_WIDTH		1
#define MSDCFG_SEDA_MASK		_UL(0x1)
#define MSDCFG_SEDA			(MSDCFG_SEDA_MASK << MSDCFG_SEDA_SHIFT)

#define MSDCFG_SETA_SHIFT		7
#define MSDCFG_SETA_WIDTH		1
#define MSDCFG_SETA_MASK		_UL(0x1)
#define MSDCFG_SETA			(MSDCFG_SETA_MASK << MSDCFG_SETA_SHIFT)

#define MSDCFG_SSRM_SHIFT		22
#define MSDCFG_SSRM_WIDTH		1
#define MSDCFG_SSRM_MASK		_UL(0x1)
#define MSDCFG_SSRM			(MSDCFG_SSRM_MASK << MSDCFG_SSRM_SHIFT)

#define MSDCFG_SSMM_SHIFT		23
#define MSDCFG_SSMM_WIDTH		1
#define MSDCFG_SSMM_MASK		_UL(0x1)
#define MSDCFG_SSMM			(MSDCFG_SSMM_MASK << MSDCFG_SSMM_SHIFT)

#define MSDCFG_SRL_SHIFT		24
#define MSDCFG_SRL_WIDTH		4
#define MSDCFG_SRL_MASK			_UL(0xF)
#define MSDCFG_SRL			(MSDCFG_SRL_MASK << MSDCFG_SRL_SHIFT)

#define MSDCFG_SML_SHIFT		28
#define MSDCFG_SML_WIDTH		4
#define MSDCFG_SML_MASK			_UL(0xF)
#define MSDCFG_SML			(MSDCFG_SML_MASK << MSDCFG_SML_SHIFT)

/**
 * MFENCE.PA helper functions
 *
 * Synchronize updates to the MPT structures with current execution.
 * Instruction is valid in M-Mode only.
 *
 * Variants as per the rs1/rs2 values combination
 * rs1    rs2   Scope
 * ---    ---   -----------------------------------------------
 * x0     x0    All MPT levels, all supervisor domain spaces
 * x0     sdid  All MPT levels, one supervisor domain space
 * paddr  x0    Leaf MPT entries for PA, all supervisor domains
 * paddr  sdid  Leaf MPT entries for PA, one supervisor domain
 *
 * Note: As per spec - bits XLEN-1:SDIDMAX in rs2 are reserved and software
 * must zero them.
 * If SDIDLEN < SDIDMAX the implementation ignores bits SDIDMAX-1:SDIDLEN.
 */

/*
 * rs1=0 and rs2=0
 * Orders all reads and writes to any level of the MPTs for all
 * supervisor domain address spaces. Use after updating MPT structures
 * for any/all supervisor domains.
 */
static inline void mfence_pa_all(void)
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
static inline void mfence_pa_sdid(unsigned long sdid)
{
	sdid &= MMPT_SDID_MAX_VAL;
	asm volatile(
		"mfence.pa x0, %0"
		:
		: "r" (sdid)
		: "memory");
}

/**
 * rs1=paddr and rs2=x0
 * Orders all reads and writes to the leaf MPT entries that correspond
 * to the physical address paddr, for all supervisor domain address
 * spaces.
 * Use after updating a single page's MPT leaf entry across all domains.
 */
static inline void mfence_pa_addr(unsigned long paddr)
{
	asm volatile(
		"mfence.pa %0, x0"
		:
		: "r" (paddr)
		: "memory");
}

/**
 * rs1=paddr and rs2=sdid
 * Orders all reads and writes to the leaf MPT entries that correspond
 * to the physical address paddr, for the single supervisor domain
 * address space identified by sdid.
 * Use when updating one page in one domain.
 */
static inline void mfence_pa(unsigned long paddr, unsigned long sdid)
{
	sdid &= MMPT_SDID_MAX_VAL;
	asm volatile(
		"mfence.pa %0, %1"
		:
		: "r" (paddr), "r" (sdid)
		: "memory");
}

/**
 * MINVAL.PA helper functions
 * Fine-grain MPT invalidation. Depends on Svinval and Smsdid.
 *
 * Sequence for invalidation:
 *	SFENCE.W.INVAL
 *	MINVAL.PA
 *	SFENCE.INVAL.IR
 */

/**
 * rs1=x0 and rs2=x0
 * Invalidates all cached MPT entries for all supervisor domains.
 * Safe global invalidation.
 */
static inline void minval_pa_all(void)
{
	/* Prior writes are globally visible before invalidation */
	asm volatile("sfence.w.inval" ::: "memory");

	asm volatile("minval.pa x0, x0" ::: "memory");

	/* Invalidation completes before subsequent accesses */
	asm volatile("sfence.inval.ir" ::: "memory");
}

/**
 * rs1=x0 and rs2=sdid
 * Invalidates cached MPT entries for a specific supervisor domain sdid.
 */
static inline void minval_pa_sdid(unsigned long sdid)
{
	sdid &= MMPT_SDID_MAX_VAL;

	asm volatile("sfence.w.inval" ::: "memory");

	asm volatile(
		"minval.pa x0, %0"
		:
		: "r" (sdid)
		: "memory");

	asm volatile("sfence.inval.ir" ::: "memory");
}

/**
 * rs1=paddr and rs2=x0
 * Invalidates cached MPT entries that correspond to a physical address paddr
 * across all supervisor domains.
 */
static inline void minval_pa_addr(unsigned long paddr)
{
	asm volatile("sfence.w.inval" ::: "memory");

	asm volatile(
		"minval.pa %0, x0"
		:
		: "r" (paddr)
		: "memory");

	asm volatile("sfence.inval.ir" ::: "memory");
}

/**
 * rs1=paddr and rs2=sdid
 * Invalidates cached MPT entries for a given physical address paddr and
 * supervisor domain sdid.
 */
static inline void minval_pa(unsigned long paddr, unsigned long sdid)
{
	sdid &= MMPT_SDID_MAX_VAL;

	asm volatile("sfence.w.inval" ::: "memory");

	asm volatile(
		"minval.pa %0, %1"
		:
		: "r" (paddr), "r" (sdid)
		: "memory");

	asm volatile("sfence.inval.ir" ::: "memory");
}

/**
 * Probe the number of SDID implemented bits.
 *
 * The least significant bits of SDID are implemented first:
 * that is, if SDIDLEN > 0, SDID[SDIDLEN-1:0].
 *
 * SDIDMAX = 6 as per spec.
 *
 */
static inline unsigned int smsdid_get_sdidlen(void)
{
    unsigned long mmpt_val, sdid_val;
    unsigned int  sdidlen = 0;

    mmpt_val = csr_read(CSR_MMPT);

    /* Write all-ones into SDID only */
    csr_write(CSR_MMPT, MMPT_SDID);
    sdid_val = (unsigned long)MMPT_GET_SDID(csr_read(CSR_MMPT));

    /* Count implemented bits from LSB */
    while ((sdid_val & 1UL) && (sdidlen < MMPT_SDIDMAX)) {
        sdidlen+=1;
        sdid_val >>= 1;
    }

    csr_write(CSR_MMPT, mmpt_val);
    return sdidlen;
}

#endif

