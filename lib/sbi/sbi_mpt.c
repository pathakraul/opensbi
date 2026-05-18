/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm Inc.
 *
 * Authors:
 *   Rahul Pathak <rahul.pathak@oss.qualcomm.com>
 */

#include <sbi/riscv_mpt.h>
#include <sbi/riscv_asm.h>
#include <sbi/riscv_locks.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_console.h>

static struct sbi_mpt_ctrl	mpt_ctrl;

#if __riscv_xlen == 64
extern struct sbi_mpt_mode smmpt43_mode;
extern struct sbi_mpt_mode smmpt52_mode;
extern struct sbi_mpt_mode smmpt64_mode;
#else
extern struct sbi_mpt_scheme smmpt34_scheme;
#endif

static inline unsigned int mpt_get_sdidlen(void)
{
	unsigned long mmpt_val, sdid_val;
	unsigned int  sdidlen = 0;

	mmpt_val = csr_read(SBI_CSR_MMPT);

	/* Write all-ones into SDID only */
	csr_write(SBI_CSR_MMPT, SBI_MMPT_SDID_MASK);
	sdid_val = (csr_read(SBI_CSR_MMPT) & SBI_MMPT_SDID_MASK) >> SBI_MMPT_SDID_SHIFT;

	/* Count implemented bits from LSB */
	while ((sdid_val & 1UL) && (sdidlen < SBI_MPT_SDIDMAX)) {
		sdidlen+=1;
		sdid_val >>= 1;
	}

	/* Restore the mmpt value */
	csr_write(SBI_CSR_MMPT, mmpt_val);
	return sdidlen;
}

struct sbi_mpt_domain *sbi_mpt_domain_get(u32 sdid)
{
	if (sdid >= mpt_ctrl.max_domains)
		return NULL;
	return mpt_ctrl.domains[sdid].valid ? &mpt_ctrl.domains[sdid] : NULL;
}

struct sbi_mpt_ctrl *sbi_mpt_ctrl_get(void)
{
	return &mpt_ctrl;
}

/**
 * MPT Pool allocator
 */
unsigned long sbi_mpt_pool_alloc(unsigned long size, unsigned long align)
{
	unsigned long ptr = (unsigned long)sbi_aligned_alloc(align, size);

	if (!ptr) {
		sbi_printf("sbi_mpt: alloc failed"
			   " (size=0x%lx align=0x%lx)\n", size, align);
		return 0;
	}
	sbi_memset((void *)ptr, 0, size);
	return ptr;
}

static bool mmpt_probe_mode(unsigned long mode_val)
{
	unsigned long m;
	csr_write(SBI_CSR_MMPT,
		  (mode_val << SBI_MMPT_MODE_SHIFT) & SBI_MMPT_MODE_MASK);

	m = (csr_read(SBI_CSR_MMPT) & SBI_MMPT_MODE_MASK) >> SBI_MMPT_MODE_SHIFT;
	return (m == mode_val);
}

/**
 * Probe the supported mode from mmpt csr
 * and set the associated mode handlers
 */
static struct sbi_mpt_mode *detect_mode(void)
{
	struct sbi_mpt_mode *m = NULL;

#if __riscv_xlen == 64
	/* Pick the widest SMMPT mode first */
	if (mmpt_probe_mode(SBI_MMPT_MODE_SMMPT64))
		m = &smmpt64_mode;
	else if (mmpt_probe_mode(SBI_MMPT_MODE_SMMPT52))
		m = &smmpt52_mode;
	else if (mmpt_probe_mode(SBI_MMPT_MODE_SMMPT43))
		m = &smmpt43_mode;
#else
	if      (mmpt_probe_mode(SBI_MMPT_MODE_SMMPT34)) s = &smmpt34_mode;
#endif

	if (!m)
		return NULL;

	/* Keep Bare mode for no MPT enforcement until domain is activated */
	csr_write(SBI_CSR_MMPT, UL(0));

	return m;
}

int sbi_mpt_init(void)
{
	struct sbi_scratch *scratch = sbi_scratch_thishart_ptr();
	unsigned long fw_pa = scratch->fw_start;
	unsigned long fw_size = scratch->fw_size;

	if (mpt_ctrl.ready)
		return 0;

	if (!sbi_hart_has_extension(scratch, SBI_HART_EXT_SMSDID)) {
		sbi_printf("sbi_mpt: Smsdid absent" " (\"smsdid\" not in ISA string)\n");
		return SBI_ENODEV;
	}

	if (!sbi_hart_has_extension(scratch, SBI_HART_EXT_SMMPT)) {
		sbi_printf("sbi_mpt: Smmpt absent" " (\"smmpt\" not in ISA string)\n");
		return SBI_ENODEV;
	}

	mpt_ctrl.sdidlen = mpt_get_sdidlen();
	if (!mpt_ctrl.sdidlen) {
		sbi_printf("sbi_mpt: SDIDLEN=0 —"" SDID field not implemented\n");
		return SBI_ENODEV;
	}

	mpt_ctrl.max_domains = UL(1) << mpt_ctrl.sdidlen;

	bitmap_fill(mpt_ctrl.sdid_bitmap, mpt_ctrl.max_domains);

	mpt_ctrl.mode = detect_mode();
	if (!mpt_ctrl.mode) {
		sbi_printf("sbi_mpt: WARL probe found no mode\n");
		return SBI_ENODEV;
	}

	mpt_ctrl.fw_pa = fw_pa;
	mpt_ctrl.fw_size = fw_size;

	/* Per-hart SDID scratch slot - initialize all harts to Invalid */
	mpt_ctrl.sdid_offset = sbi_scratch_alloc_offset(sizeof(u32));
	if (!mpt_ctrl.sdid_offset) {
		sbi_printf("scratch alloc for SDID failed\n");
		return SBI_ENOMEM;
	}

	sbi_for_each_hartindex(i) {
		struct sbi_scratch *s = sbi_hartindex_to_scratch(i);
		if (!s)
			continue;
		u32 *p = sbi_scratch_offset_ptr(s, mpt_ctrl.sdid_offset);
		*p = (u32)SBI_MPT_SDID_INVALID;
	}

	sbi_printf("sbi_mpt: scheme='%s' SDIDLEN=%u max_domains=%u"
		   " root_size=0x%lx\n",
		   mpt_ctrl.mode->name,
		   mpt_ctrl.sdidlen, mpt_ctrl.max_domains,
		   mpt_ctrl.mode->root_table_size());

	mpt_ctrl.ready = true;
	return 0;
}

static int map_range_perm(struct sbi_mpt_domain *dom,
			   struct sbi_mpt_mode *mode,
			   unsigned long pa, unsigned long size,
			   u8 xwr, bool locked)
{
	int rc;

	if (!pa || !size) {
		sbi_printf("sbi_mpt: invalid range 0x%lx+0x%lx\n", pa, size);
		return SBI_EINVAL;
	}

	if (pa & (SBI_MPT_PAGE_SIZE - 1) || size & (SBI_MPT_PAGE_SIZE - 1)) {
		sbi_printf("sbi_mpt: unaligned range 0x%lx+0x%lx\n", pa, size);
		return SBI_EINVAL;
	}
	/*
	 * Check PA range against the scheme's addressable space.
	 */
	if (!mode->pa_in_range(pa, size)) {
		/**
		 * Fail (locked = true) for S/U regions which are not in range.
		 */
		if (locked) {
			sbi_printf("sbi_mpt: locked region 0x%lx+0x%lx"
				   " out of scheme range\n", pa, size);
			return SBI_EINVAL;
		}
		/**
		 * Skip (locked=false) for S/U regions which are not in range.
		 */
		sbi_printf("sbi_mpt: skip out-of-range 0x%lx+0x%lx\n", pa, size);
		return 0;
	}

	rc = mode->map_range(dom, pa, size, xwr);
	if (rc)
		return rc;

	if (dom->nregions >= SBI_MPT_MAX_REGIONS_DOMAIN)
		return SBI_ENOMEM;

	dom->regions[dom->nregions].pa = pa;
	dom->regions[dom->nregions].size = size;
	dom->regions[dom->nregions].xwr = xwr;
	dom->regions[dom->nregions].locked = locked;
	dom->nregions += 1;

	return 0;
}

int sbi_mpt_domain_create(const struct sbi_mpt_domain_config *cfg, u32 *out_sdid)
{
	struct sbi_mpt_ctrl *ctrl = &mpt_ctrl;
	struct sbi_mpt_mode *mode;
	struct sbi_mpt_domain *dom;
	const struct sbi_domain_memregion *mr;
	unsigned long root_pa;
	u32 sdid, i = 0;
	unsigned long fw_pa_al, fw_size_al;
	int rc = 0;

	if (!ctrl->ready)
		return SBI_ENODEV;

	if (bitmap_empty(ctrl->sdid_bitmap, ctrl->max_domains))
		return SBI_ENOMEM;

	mode = ctrl->mode;

	for (sdid = 0; sdid < ctrl->max_domains; sdid++) {
		if (bitmap_test(ctrl->sdid_bitmap, sdid))
			break;
	}

	bitmap_clear(ctrl->sdid_bitmap, sdid, 1);

	dom = &ctrl->domains[sdid];

	SPIN_LOCK_INIT(dom->lock);

	root_pa = sbi_mpt_pool_alloc(mode->root_table_size(), mode->root_table_align());
	if (!root_pa) {
		sbi_printf("sbi_mpt: MPT memory allocation failed\n");
		return SBI_ENOMEM;
	}

	dom->sdid = sdid;
	dom->valid = true;
	dom->root_pa = root_pa;
	dom->mode = mode;
	dom->sbi_dom = cfg->sbi_dom;

	/* Map the S/U accessible regions */
	if (cfg->sbi_dom && cfg->xwr) {
		sbi_domain_for_each_memregion(cfg->sbi_dom, mr) {
			unsigned long mr_pa = mr->base;
			unsigned long mr_size = BIT(mr->order);

			if (!(mr->flags & SBI_DOMAIN_MEMREGION_SU_ACCESS_MASK))
				continue;

			rc = map_range_perm(dom, mode, mr_pa, mr_size, cfg->xwr, false);
			if (rc) {
				sbi_printf("map range with permission failed\n");
				goto fail;
			}
		}
	}

	for (i = 0; i < cfg->nregions; i++) {
		rc = map_range_perm(dom, mode, cfg->regions[i].pa,
						cfg->regions[i].size,
						cfg->regions[i].xwr, false);
		if (rc)
			goto fail;
	}

	if (cfg->fw_protect) {
		fw_pa_al = ctrl->fw_pa & ~SBI_MPT_PAGE_MASK;
		fw_size_al = (ctrl->fw_size + SBI_MPT_PAGE_SIZE - UL(1)) & ~SBI_MPT_PAGE_MASK;
		rc = map_range_perm(dom, mode, fw_pa_al, fw_size_al, SBI_MPT_PERM_NONE, true);
		if (rc) {
			sbi_printf("sbi_mpt: cannot protect firmware rc=%d\n", rc);
			goto fail;
		}
	}

	ctrl->ndomain += 1;
	if (out_sdid)
		*out_sdid = sdid;

	//sbi_mpt_mfence_sdid(sdid);
	sbi_printf("sbi_mpt: SDID %u root=0x%lx scheme=%s dom=%s\n",
		   sdid, root_pa, mode->name,
		   cfg->sbi_dom ? cfg->sbi_dom->name : "(none)");

	return 0;

fail:
	bitmap_set(ctrl->sdid_bitmap, sdid, 1);
	dom->valid = false;
	return rc;
}

/*
 *  Per-hart mmpt programming
 */
int sbi_mpt_hart_activate(u32 sdid)
{
	struct sbi_mpt_domain *dom;
	unsigned long ppn;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;

	dom = sbi_mpt_domain_get(sdid);
	if (!dom)
		return SBI_EINVAL;

	ppn = dom->root_pa >> SBI_MPT_PAGE_SHIFT;
	csr_write(SBI_CSR_MMPT, dom->mode->encode_mmpt(ppn, sdid));
//	sbi_mpt_mfence_sdid(sdid);

	if (mpt_ctrl.sdid_offset) {
		u32 *p = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
						mpt_ctrl.sdid_offset);
		*p = sdid;
	}

	return 0;
}

/**
 * [RAHUL]: Since mmpt csr is always active for access check unless in M-Mode
 * What else we can deactivate in this function apart from making SDID invalid
 * - REVISIT LATER
 */
void sbi_mpt_hart_deactivate(void)
{
	if (!mpt_ctrl.ready)
		return;

	csr_write(SBI_CSR_MMPT, UL(0));
//	sbi_mpt_mfence_all();

	if (mpt_ctrl.sdid_offset) {
		u32 *p = sbi_scratch_offset_ptr(sbi_scratch_thishart_ptr(),
						mpt_ctrl.sdid_offset);
		*p = (u32)SBI_MPT_SDID_INVALID;
	}
}

int sbi_mpt_hart_activate_for_domain(struct sbi_domain *sbi_dom)
{
	u32 s;

	if (!mpt_ctrl.ready)
		return SBI_ENODEV;
	if (!sbi_dom)
		return SBI_EINVAL;

	for (s = 0; s < mpt_ctrl.max_domains; s++) {
		if (mpt_ctrl.domains[s].valid &&
		    mpt_ctrl.domains[s].sbi_dom == sbi_dom)
			return sbi_mpt_hart_activate(s);
	}

	return 0;
}
