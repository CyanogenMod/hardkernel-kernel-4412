/* linux/drivers/char/exynos_mem.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/errno.h>	/* error codes */
#include <linux/fs.h>
#include <linux/ioctl.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>

#include <linux/exynos_mem.h>

#define L2_FLUSH_ALL	SZ_1M
#define L1_FLUSH_ALL	SZ_64K

struct exynos_mem {
	bool cacheable;
};

int exynos_mem_open(struct inode *inode, struct file *filp)
{
	struct exynos_mem *prv_data;

	prv_data = kzalloc(sizeof(struct exynos_mem), GFP_KERNEL);
	if (!prv_data) {
		pr_err("%s: not enough memory\n", __func__);
		return -ENOMEM;
	}

	prv_data->cacheable = true;	/* Default: cacheable */

	filp->private_data = prv_data;

	printk(KERN_DEBUG "[%s:%d] private_data(0x%08x)\n",
		__func__, __LINE__, (u32)prv_data);

	return 0;
}

int exynos_mem_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "[%s:%d] private_data(0x%08x)\n",
		__func__, __LINE__, (u32)filp->private_data);

	kfree(filp->private_data);

	return 0;
}

static void exynos_mem_paddr_cache_flush(dma_addr_t start, size_t length)
{
	if (length > (size_t) L2_FLUSH_ALL) {
		flush_cache_all();		/* L1 */
		smp_call_function((smp_call_func_t)__cpuc_flush_kern_all, NULL, 1);
		outer_flush_all();		/* L2 */
	} else if (length > (size_t) L1_FLUSH_ALL) {
		dma_addr_t end = start + length - 1;

		flush_cache_all();		/* L1 */
		smp_call_function((smp_call_func_t)__cpuc_flush_kern_all, NULL, 1);
		outer_flush_range(start, end);  /* L2 */
	} else {
		dma_addr_t end = start + length - 1;

		dmac_flush_range(phys_to_virt(start), phys_to_virt(end));
		outer_flush_range(start, end);	/* L2 */
	}
}

long exynos_mem_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case EXYNOS_MEM_SET_CACHEABLE:
	{
		struct exynos_mem *mem = filp->private_data;
		int cacheable;
		if (get_user(cacheable, (u32 __user *)arg)) {
			pr_err("[%s:%d] err: EXYNOS_MEM_SET_CACHEABLE\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		mem->cacheable = cacheable;
		break;
	}

	case EXYNOS_MEM_PADDR_CACHE_FLUSH:
	{
		struct exynos_mem_flush_range range;
		if (copy_from_user(&range,
				   (struct exynos_mem_flush_range __user *)arg,
				   sizeof(range))) {
			pr_err("[%s:%d] err: EXYNOS_MEM_PADDR_CACHE_FLUSH\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		exynos_mem_paddr_cache_flush(range.start, range.length);
		break;
	}

	default:
		pr_err("[%s:%d] error command\n", __func__, __LINE__);
		return -EINVAL;
	}

	return 0;
}

static void exynos_mem_mmap_open(struct vm_area_struct *vma)
{
	printk(KERN_DEBUG "[%s] addr(0x%08x)\n", __func__, (u32)vma->vm_start);
}

static void exynos_mem_mmap_close(struct vm_area_struct *vma)
{
	printk(KERN_DEBUG "[%s] addr(0x%08x)\n", __func__, (u32)vma->vm_start);
}

static struct vm_operations_struct exynos_mem_ops = {
	.open	= exynos_mem_mmap_open,
	.close	= exynos_mem_mmap_close,
};

int exynos_mem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct exynos_mem *mem = (struct exynos_mem *)filp->private_data;
	bool cacheable = mem->cacheable;
	dma_addr_t start = vma->vm_pgoff << PAGE_SHIFT;
	u32 pfn = vma->vm_pgoff;
	u32 size = vma->vm_end - vma->vm_start;

	/* TODO: currently lowmem is only avaiable */
	if ((phys_to_virt(start) < (void *)PAGE_OFFSET) ||
	    (phys_to_virt(start) >= high_memory)) {
		pr_err("[%s] invalid paddr(0x%08x)\n", __func__, start);
		return -EINVAL;
	}

	if (!cacheable)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_RESERVED;
	vma->vm_ops = &exynos_mem_ops;

	if ((vma->vm_flags & VM_WRITE) && !(vma->vm_flags & VM_SHARED)) {
		pr_err("writable mapping must be shared\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		pr_err("mmap fail\n");
		return -EINVAL;
	}

	vma->vm_ops->open(vma);

	return 0;
}
