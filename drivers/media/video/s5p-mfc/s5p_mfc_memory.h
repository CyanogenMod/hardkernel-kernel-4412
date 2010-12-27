/*
 * drivers/media/video/samsung/mfc5/s5p_mfc_memory.h
 *
 * Header file for Samsung MFC (Multi Function Codec - FIMV) driver
 * Contains memory related defines.
 *
 * Kamil Debski, Copyright (c) 2010 Samsung Electronics
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef S5P_MFC_MEMORY_H_
#define S5P_MFC_MEMORY_H_

#include "s5p_mfc_common.h"

#define FIRMWARE_CODE_SIZE		0x60000	/* 384KB */
#define MFC_H264_INSTANCE_BUF_SIZE	0x96000	/* 600KB per H264 instance */
#define MFC_INSTANCE_BUF_SIZE		0x2800	/* 10KB per instance */
#define DESC_BUF_SIZE			0x20000	/* 128KB for DESC buffer */
#define SHARED_BUF_SIZE			0x01000	/* 4KB for shared buffer */
#define CPB_BUF_SIZE			0x400000/* 4MB fr decoder */

#if 0
/* Define names for CMA memory kinds used by MFC */
#define MFC_CMA_ALLOC_CTX_NUM	3

#define MFC_CMA_BANK1		"a"
#define MFC_CMA_BANK2		"b"
#define MFC_CMA_FW		"f"

#define MFC_CMA_BANK1_ALLOC_CTX 1
#define MFC_CMA_BANK2_ALLOC_CTX 0 
#define MFC_CMA_FW_ALLOC_CTX 	2

#define MFC_CMA_BANK1_ALIGN	0x2000	/* 8KB */
#define MFC_CMA_BANK2_ALIGN	0x2000	/* 8KB */
#define MFC_CMA_FW_ALIGN	0x20000	/* 128KB */
#else

#define MFC_ALLOC_CTX_NUM	2

#define MFC_BANK_A_ALLOC_CTX 	0
#define MFC_BANK_B_ALLOC_CTX 	1

#define MFC_BANK_A_ALIGN_ORDER	11
#define MFC_BANK_B_ALIGN_ORDER	11

#define MFC_CMA_BANK1_ALLOC_CTX MFC_BANK_A_ALLOC_CTX
#define MFC_CMA_BANK2_ALLOC_CTX MFC_BANK_B_ALLOC_CTX
#define MFC_CMA_FW_ALLOC_CTX 	MFC_BANK_A_ALLOC_CTX

#endif

#endif /* S5P_MFC_MEMORY_H_ */
