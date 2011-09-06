/*
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 * (C) COPYRIGHT 2008-2011 ARM Limited
 * ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 */

#ifndef _UMP_KERNEL_LINUX_MEM_H_
#define _UMP_KERNEL_LINUX_MEM_H_


int umpp_linux_mmap(struct file * filp, struct vm_area_struct * vma);

#endif /* _UMP_KERNEL_LINUX_MEM_H_ */
