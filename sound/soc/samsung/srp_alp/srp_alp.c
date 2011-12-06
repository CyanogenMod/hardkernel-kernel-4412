/* sound/soc/samsung/alp/srp_alp.c
 *
 * SRP Audio driver for Samsung Exynos
 *
 * Copyright (c) 2011 Samsung Electronics
 * http://www.samsungsemi.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/serio.h>
#include <linux/time.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/cma.h>

#include <mach/hardware.h>
#include <mach/irqs.h>
#include <mach/map.h>

#include "../srp-types.h"
#include "srp_alp_reg.h"
#include "srp_alp_fw.h"
#include "srp_alp_ioctl.h"
#include "srp_alp_error.h"

#include "../idma.h"
#include "../audss.h"

#define SRP_DEV_MINOR	(250)

/* SRAM information */
#define IRAM_SIZE	(0x40000)
#define DMEM_SIZE	(0x20000)
#define ICACHE_SIZE	(0x10000)

/* Buffer information */
#define IBUF_SIZE	(0x4000)
#define OBUF_SIZE	(0x8000)
#define WBUF_SIZE	(IBUF_SIZE * 4)
#define IBUF_OFFSET	(0x30000)
#define OBUF_OFFSET	(0x4)
#define IBUF_NUM	(0x2)
#define OBUF_NUM	(0x2)

/* Commbox & Etc information */
#define COMMBOX_SIZE	(0x2000)

/* F/W code size */
#define VLIW_SIZE	(0x20000)	/* 128KBytes */
#define DATA_SIZE	(0x20000)	/* 128KBytes */
#define CGA_SIZE	(0x9000)	/* 36KBytes */

/* Reserved memory on DRAM */
#define BASE_MEM_SIZE	(CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP << 10)
#define VLIW_SIZE_MAX	(0x20000)
#define CGA_SIZE_MAX	(0x10000)
#define DATA_SIZE_MAX	(0x20000)
#define BITSTREAM_SIZE_MAX	(0x7FFFFFFF)

#ifdef USE_FW_ENDIAN_CONVERT
#define ENDIAN_CHK_CONV(VAL)		\
	(((VAL >> 24) & 0x000000FF) |	\
	((VAL >> 8) & 0x0000FF00) |	\
	((VAL << 8) & 0x00FF0000) |	\
	((VAL << 24) & 0xFF000000))
#else
#define ENDIAN_CHK_CONV(VAL)	(VAL)
#endif

#ifdef CONFIG_SND_SAMSUNG_RP_DEBUG
#define srpdbg(x...) printk(KERN_INFO "SRP: " x)
#else
#define srpdbg(x...)
#endif

struct srp_buf_info {
	void		*mmapped_addr;
	void		*addr;
	unsigned int	mmapped_size;
	unsigned int	size;
	int		num;
};

struct srp_info {
	struct srp_buf_info	*ibuf_info;
	struct srp_buf_info	*obuf_info;
	struct srp_buf_info	*pcm_info;

	spinlock_t	lock;
	int		state;

	void __iomem	*iram;
	void __iomem	*dmem;
	void __iomem	*icache;
	void __iomem	*commbox;

	unsigned char	*ibuf0;
	unsigned char	*ibuf1;
	unsigned char	*obuf0;
	unsigned char	*obuf1;

	unsigned int	ibuf0_pa;
	unsigned int	ibuf1_pa;
	unsigned int	obuf0_pa;
	unsigned int	obuf1_pa;
	unsigned long	obuf_size;		/* OBUF size byte */
	unsigned long	ibuf_size;		/* IBUF size byte */
	unsigned long	ibuf_fill_size[2];	/* IBUF Fill size */

	unsigned char	*wbuf;			/* Temporatry BUF VA */
	unsigned int	wbuf_pa;		/* Temporatry BUF PA */
	unsigned long	wbuf_pos;		/* Write pointer */
	unsigned long	wbuf_fill_size;		/* Total size by user write() */

	int ibuf_next;
	int ibuf_empty[2];

	int obuf_fill_done[2];
	int obuf_ready;
	int obuf_next;

	unsigned long frame_size;
	unsigned long frame_count;
	unsigned long frame_count_base;
	unsigned long channel;			/* Mono = 1, Stereo = 2 */
	unsigned long sample_rate;		/* Sampling Rate 8000 ~ 48000 */
	unsigned long bit_rate;

	int first_decoding;
	int decoding_started;
	int is_opened;				/* Running status of SRP */
	int is_running;				/* Open status of SRP */
	int is_pending;
	int block_mode;				/* Block Mode */
	int ibuf_req_skip;
	int stop_after_eos;
	int wait_for_eos;
	int prepare_for_eos;
	int save_ibuf_empty;

	unsigned char *fw_code_vliw;		/* VLIW */
	unsigned char *fw_code_cga;		/* CGA */
	unsigned char *fw_data;			/* DATA */

	dma_addr_t fw_mem_base;			/* Base memory for FW */
	unsigned long vliw_rp;
	unsigned long fw_mem_base_pa;		/* Physical address of base */
	unsigned long fw_code_vliw_pa;		/* Physical address of VLIW */
	unsigned long fw_code_cga_pa;		/* Physical address of CGA */
	unsigned long fw_data_pa;		/* Physical address of DATA */
	unsigned long fw_code_vliw_size;	/* Size of VLIW */
	unsigned long fw_code_cga_size;		/* Size of CGA */
	unsigned long fw_data_size;		/* Size of DATA */

	void	(*audss_clk_enable)(bool enable);
	wait_queue_head_t read_wait;
};

/* SRP Pending On/Off status */
enum {
	RUN = 0,
	STALL,
};

static struct srp_info srp;
static DEFINE_MUTEX(srp_mutex);

int srp_get_status(int cmd)
{
	return (cmd == IS_RUNNING) ? srp.is_running : srp.is_opened;
}

static void srp_pending_ctrl(int ctrl)
{
	int srp_ctrl = ctrl ? STALL : RUN;

	if (ctrl == srp.is_pending) {
		srpdbg("SRP: Current SRP Status[%s]\n",
			srp.is_pending ? "STALL" : "RUN");
		return;
	}

	writel(srp_ctrl, srp.commbox + SRP_PENDING);
	srp.is_pending = srp_ctrl;

	srpdbg("SRP: Current SRP Status[%s]\n",
			srp.is_pending ? "STALL" : "RUN");
}

static void srp_reset_frame_counter(void)
{
	srp.frame_count = 0;
	srp.frame_count_base = 0;
}

static unsigned long srp_get_frame_counter(void)
{
	unsigned long val;

	val = readl(srp.commbox + SRP_FRAME_INDEX);
	srp.frame_count = srp.frame_count_base + val;

	return srp.frame_count;
}

static void srp_check_stream_info(void)
{
	if (!srp.channel) {
		srp.channel = readl(srp.commbox
				+ SRP_ARM_INTERRUPT_CODE);
		srp.channel >>= SRP_ARM_INTR_CODE_CHINF_SHIFT;
		srp.channel &= SRP_ARM_INTR_CODE_CHINF_MASK;
		if (srp.channel)
			srpdbg("SRP: Channel = %lu\n", srp.channel);
	}

	if (!srp.sample_rate) {
		srp.sample_rate = readl(srp.commbox
				+ SRP_ARM_INTERRUPT_CODE);
		srp.sample_rate >>= SRP_ARM_INTR_CODE_SRINF_SHIFT;
		srp.sample_rate &= SRP_ARM_INTR_CODE_SRINF_MASK;
		if (srp.sample_rate)
			srpdbg("SRP: Sample Rate = %lu\n", srp.sample_rate);

	}

	if (!srp.frame_size) {
		switch (readl(srp.commbox
			+ SRP_ARM_INTERRUPT_CODE)
			& SRP_ARM_INTR_CODE_FRAME_MASK) {
		case SRP_ARM_INTR_CODE_FRAME_1152:
			srp.frame_size = 1152;
			break;
		case SRP_ARM_INTR_CODE_FRAME_1024:
			srp.frame_size = 1024;
			break;
		case SRP_ARM_INTR_CODE_FRAME_576:
			srp.frame_size = 576;
			break;
		case SRP_ARM_INTR_CODE_FRAME_384:
			srp.frame_size = 384;
			break;
		default:
			srp.frame_size = 0;
			break;
		}
		if (srp.frame_size)
			srpdbg("SRP: Frame size = %lu\n", srp.frame_size);
	}
}

static void srp_flush_ibuf(void)
{
	memset(srp.ibuf0, 0xFF, srp.ibuf_size);
	memset(srp.ibuf1, 0xFF, srp.ibuf_size);
}

static void srp_flush_obuf(void)
{
	memset(srp.obuf0, 0x0, srp.obuf_size);
	memset(srp.obuf1, 0x0, srp.obuf_size);
}

static void srp_reset(void)
{
	unsigned int reg = 0x0;

	srpdbg("SRP: Reset\n");

	if (waitqueue_active(&srp.read_wait))
		wake_up_interruptible(&srp.read_wait);

	srp_pending_ctrl(STALL);

	writel(reg, srp.commbox + SRP_FRAME_INDEX);
	writel(reg, srp.commbox + SRP_READ_BITSTREAM_SIZE);

	/* RESET */
	writel(reg, srp.commbox + SRP_CONT);
	writel(reg, srp.commbox + SRP_INTERRUPT);

	/* Store Total Count */
	srp.frame_count_base = srp.frame_count;
	srp.decoding_started = 0;
	srp.first_decoding = 1;

	/* Next IBUF is IBUF0 */
	srp.ibuf_next = 0;
	srp.ibuf_empty[0] = 1;
	srp.ibuf_empty[1] = 1;

	srp.ibuf_fill_size[0] = 0;
	srp.ibuf_fill_size[1] = 0;

	srp.obuf_next = 1;
	srp.obuf_ready = 0;
	srp.obuf_fill_done[0] = 0;
	srp.obuf_fill_done[1] = 0;

	srp.wbuf_pos = 0;
	srp.wbuf_fill_size = 0;

	srp.is_pending = STALL;
	srp.ibuf_req_skip = 1;

	srp.stop_after_eos = 0;
	srp.wait_for_eos = 0;
	srp.prepare_for_eos = 0;
	srp.save_ibuf_empty = 0;
}

static void srp_stop(void)
{
	srp_pending_ctrl(STALL);
}

static void srp_fill_ibuf(void)
{
	unsigned long fill_size;

	if (!srp.wbuf_pos)
		return;

	if (srp.wbuf_pos >= srp.ibuf_size) {
		fill_size = srp.ibuf_size;
		srp.wbuf_pos -= fill_size;
	} else {
		if (srp.wait_for_eos) {
			fill_size = srp.wbuf_pos;
			memset(&srp.wbuf[fill_size], 0xFF,
				srp.ibuf_size - fill_size);
			srp.wbuf_pos = 0;
		} else {
			srpdbg("SRP: Not filled temp buffer 16KB not yet!!\n");
			srp.save_ibuf_empty = 1;
			return;
		}
	}

	if (srp.ibuf_next == 0) {
		memcpy(srp.ibuf0, srp.wbuf, srp.ibuf_size);
		srpdbg("SRP: Fill IBUF0 (%lu)\n", fill_size);
		srp.ibuf_empty[0] = 0;
		srp.ibuf_next = 1;
		srp.ibuf_fill_size[0] = srp.ibuf_fill_size[1] + fill_size;
	} else {
		memcpy(srp.ibuf1, srp.wbuf, srp.ibuf_size);
		srpdbg("SRP: Fill IBUF1 (%lu)\n", fill_size);
		srp.ibuf_empty[1] = 0;
		srp.ibuf_next = 0;
		srp.ibuf_fill_size[1] = srp.ibuf_fill_size[0] + fill_size;
	}

	if (srp.wbuf_pos) {
		srpdbg("SRP: WBUF_POS = %ld\n", srp.wbuf_pos);
		memcpy(srp.wbuf, &srp.wbuf[srp.ibuf_size], srp.wbuf_pos);
	}
}

static ssize_t srp_write(struct file *file, const char *buffer,
					size_t size, loff_t *pos)
{
	unsigned long flags;

	srpdbg("SRP: write(%d bytes)\n", size);

	if (srp.obuf_fill_done[srp.obuf_ready]) {
		srp.obuf_fill_done[srp.obuf_ready] = 0;
		srpdbg("SRP: Elapsed Obuf[%d]\n", srp.obuf_ready);

		if (!srp.obuf_fill_done[srp.obuf_ready]) {
			srpdbg("SRP: Decoding start for filling OBUF[%d]\n",
							srp.obuf_ready);
			srp_pending_ctrl(RUN);
		}
		srp.obuf_ready = srp.obuf_ready ? 0 : 1;
		srp.obuf_next = srp.obuf_next ? 0 : 1;
	}

	spin_lock_irqsave(&srp.lock, flags);
	if (srp.wbuf_pos + size > WBUF_SIZE) {
		srpdbg("SRP: Occured Ibuf Overflow!!\n");
		spin_unlock_irqrestore(&srp.lock, flags);
		return SRP_ERROR_IBUF_OVERFLOW;
	}

	if (copy_from_user(&srp.wbuf[srp.wbuf_pos], buffer, size)) {
		spin_unlock_irqrestore(&srp.lock, flags);
		return -EFAULT;
	}

	srp.wbuf_pos += size;
	srp.wbuf_fill_size += size;
	if (srp.wbuf_pos < srp.ibuf_size) {
		srpdbg("SRP : Return WBUF POS %ld\n", srp.wbuf_pos);
		spin_unlock_irqrestore(&srp.lock, flags);
		return size;
	}
	spin_unlock_irqrestore(&srp.lock, flags);

	if (!srp.decoding_started) {
		srp_fill_ibuf();
		if (!srp.ibuf_empty[0] && !srp.ibuf_empty[1]) {
			srpdbg("SRP: First Start decoding!!\n");
			srp_pending_ctrl(RUN);
			srp.decoding_started = 1;
		}
	}

	if (srp.save_ibuf_empty) {
		srpdbg("SRP: Re-filled ibuffer missed data\n");
		srp_fill_ibuf();
		srp.save_ibuf_empty = 0;
	}

	return size;
}

static ssize_t srp_read(struct file *file, char *buffer,
				size_t size, loff_t *pos)
{
	struct srp_buf_info *argp = (struct srp_buf_info *)buffer;
	void *obuf0 = srp.obuf_info->addr;
	void *obuf1 = srp.obuf_info->addr + OBUF_SIZE;
	int ret = 0;

	srpdbg("SRP: Entered Get Obuf[%d] in PCM function\n", srp.obuf_ready);

	if (srp.prepare_for_eos) {
		srp.obuf_fill_done[srp.obuf_ready] = 0;
		srpdbg("SRP: Elapsed Obuf[%d] after Send EOS\n", srp.obuf_ready);

		if (!srp.ibuf_empty[0] || !srp.ibuf_empty[1]) {
			if (!srp.obuf_fill_done[srp.obuf_next]) {
				srpdbg("SRP: Decoding start for Send EOS\n");
				srp_pending_ctrl(RUN);
			}
		}
		srp.obuf_ready = srp.obuf_ready ? 0 : 1;
		srp.obuf_next = srp.obuf_next ? 0 : 1;
	}

	if (srp.wait_for_eos)
		srp.prepare_for_eos = 1;

	if (srp.decoding_started) {
		if (srp.first_decoding) {
			srpdbg("SRP: Enter to sleep until filling both Obuf\n");
			wait_event_interruptible(srp.read_wait, !srp.first_decoding);
			srpdbg("SRP: Wake up by Both Obuf\n");
		} else {
			if (srp.obuf_fill_done[srp.obuf_ready])
				srpdbg("SRP: Already filled obuf[%d]\n", srp.obuf_ready);
			else {
				srpdbg("SRP: Enter to sleep for Obuf INT\n");
				wait_event_interruptible(srp.read_wait, srp.is_pending == STALL);
				srpdbg("SRP: Wake up by Obuf INT\n");
			}
		}
	} else {
		srpdbg("SRP: not prepared not yet! OBUF[%d]\n", srp.obuf_ready);
		srp.pcm_info->size = 0;
		return copy_to_user(argp, srp.pcm_info, sizeof(struct srp_buf_info));
	}

	srp.pcm_info->addr = srp.obuf_ready ? obuf1 : obuf0;
	srp.pcm_info->size = readl(srp.commbox + SRP_PCM_DUMP_ADDR);
	srp.pcm_info->num = srp.obuf_info->num;

	ret = copy_to_user(argp, srp.pcm_info, sizeof(struct srp_buf_info));
	srpdbg("SRP: Return OBUF Num[%d] fill size %d\n",
			srp.obuf_ready, srp.pcm_info->size);

	/* For End-Of-Stream */
	if (srp.wait_for_eos && (srp.pcm_info->size < OBUF_SIZE)) {
		srpdbg("SRP: Stop EOS by PCM SIZE\n");
		srp.stop_after_eos = 1;
	}

	return ret;
}

static void srp_commbox_init(void)
{
	unsigned int reg = 0x0;

	srp_pending_ctrl(STALL);
	writel(reg, srp.commbox + SRP_FRAME_INDEX);
	writel(reg, srp.commbox + SRP_INTERRUPT);

	/* Support Mono Decoding */
	reg |= SRP_ARM_INTR_CODE_SUPPORT_MONO;
	writel(reg, srp.commbox + SRP_ARM_INTERRUPT_CODE);

	/* Init Ibuf information */
	writel(srp.ibuf0_pa, srp.commbox + SRP_BITSTREAM_BUFF_DRAM_ADDR0);
	writel(srp.ibuf1_pa, srp.commbox + SRP_BITSTREAM_BUFF_DRAM_ADDR1);
	writel(srp.ibuf_size, srp.commbox + SRP_BITSTREAM_BUFF_DRAM_SIZE);

	/* Output PCM control : 16bit */
	writel(SRP_CFGR_OUTPUT_PCM_16BIT, srp.commbox + SRP_CFGR);

	/* Bit stream size : Max */
	writel(BITSTREAM_SIZE_MAX, srp.commbox + SRP_BITSTREAM_SIZE);

	/* Configure fw address */
	writel(srp.vliw_rp, srp.commbox + SRP_CODE_START_ADDR);
	writel(srp.fw_data_pa, srp.commbox + SRP_DATA_START_ADDR);
	writel(srp.fw_code_cga_pa, srp.commbox + SRP_CONF_START_ADDR);
}

static void srp_commbox_deinit(void)
{
	unsigned int reg = 0x0;

	srpdbg("SRP: Commbox deinitialized\n");

	/* Reset value */
	srp_stop();
	writel(reg, srp.commbox + SRP_INTERRUPT);
}

static void srp_fw_download(void)
{
	unsigned long n;
	unsigned long *pval;
	unsigned int reg = 0x0;

#ifdef CONFIG_SND_SAMSUNG_RP_DEBUG
	struct timeval begin, end;

	do_gettimeofday(&begin);
#endif

	/* Fill ICACHE with first 64KB area : ARM access I$ */
	pval = (unsigned long *)srp.fw_code_vliw;
	for (n = 0; n < 0x10000; n += 4, pval++)
		writel(ENDIAN_CHK_CONV(*pval), srp.icache + n);

	reg = readl(srp.commbox + SRP_CFGR);
	reg |= (SRP_CFGR_BOOT_INST_INT_CC |	/* Fetchs instruction from I$ */
		SRP_CFGR_USE_ICACHE_MEM   |	/* SRP can access I$ */
		SRP_CFGR_FLOW_CTRL_OFF);

	writel(reg, srp.commbox + SRP_CFGR);

#ifdef CONFIG_SND_SAMSUNG_RP_DEBUG
	do_gettimeofday(&end);
	srpdbg("Firmware Download Time : %lu.%06lu seconds.\n",
		end.tv_sec - begin.tv_sec, end.tv_usec - begin.tv_usec);
#endif
}

static void srp_set_default_fw(void)
{
	/* Initialize Commbox & default parameters */
	srp_commbox_init();

	/* Download default Firmware */
	srp_fw_download();
}

static void srp_set_stream_size(void)
{
	/* Leave stream size max, if data is available */
	if (srp.wbuf_pos)
		return;

	srpdbg("Remained data size = %ld\n", srp.wbuf_fill_size);
	writel(srp.wbuf_fill_size, srp.commbox + SRP_BITSTREAM_SIZE);
}

static long srp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct srp_buf_info *argp = (struct srp_buf_info *)arg;
	unsigned long val = 0;
	long ret = 0;
	
	srpdbg("SRP: srp_ioctl(cmd:: %08X)\n", cmd);

	mutex_lock(&srp_mutex);

	switch (cmd) {
	case SRP_INIT:
		srpdbg("SRP: SRP_INIT\n");
		srp_flush_ibuf();
		srp_flush_obuf();
		srp_reset();
		break;

	case SRP_DEINIT:
		srpdbg("SRP: SRP DEINIT\n");
		srp.decoding_started = 0;

		/* Reset commbox */
		srp_commbox_deinit();
		break;

	case SRP_GET_MMAP_SIZE:
		srp.obuf_info->mmapped_size = OBUF_SIZE * OBUF_NUM + OBUF_OFFSET;
		val = srp.obuf_info->mmapped_size;
		srpdbg("SRP: SRP_GET_MMAP_SIZE = %ld\n", val);
		ret = copy_to_user((unsigned long *)arg,
					&val, sizeof(unsigned long));
		break;

	case SRP_IBUF_FLUSH:
		srp_stop();
		srp_flush_ibuf();
		srp_flush_obuf();
		srp_set_default_fw();
		srp_reset();
		break;

	case SRP_OBUF_FLUSH:
		break;

	case SRP_GET_IBUF_INFO:
		srp.ibuf_info->addr = (void *) srp.wbuf;
		srp.ibuf_info->size = IBUF_SIZE * 2;
		srp.ibuf_info->num  = IBUF_NUM;

		ret = copy_to_user(argp, srp.ibuf_info,
						sizeof(struct srp_buf_info));
		break;

	case SRP_GET_OBUF_INFO:
		ret = copy_from_user(srp.obuf_info, argp,
				sizeof(struct srp_buf_info));
		if (!ret) {
			srp.obuf_info->addr = srp.obuf_info->mmapped_addr
							+ OBUF_OFFSET;
			srp.obuf_info->size = OBUF_SIZE;
			srp.obuf_info->num = OBUF_NUM;
		}

		ret = copy_to_user(argp, srp.obuf_info,
					sizeof(struct srp_buf_info));

		break;

	case SRP_DECODED_FRAME_SIZE:
		if (srp.frame_size) {
			val = srp_get_frame_counter() * srp.frame_size;
			srpdbg("SRP: Decoded Frame Size [%lu]\n", val);
			ret = copy_to_user((unsigned long *)arg,
					&val, sizeof(unsigned long));
		}
		break;

	case SRP_GET_CHANNEL_COUNT:
		if (srp.channel) {
			srpdbg("SRP: Channel Count [%lu]\n", srp.channel);
			ret = copy_to_user((unsigned long *)arg,
				&srp.channel, sizeof(unsigned long));
		}
		break;

	case SRP_GET_SAMPLE_RATE:
		if (srp.sample_rate) {
			srpdbg("SRP: Sample Rate [%lu]\n", srp.sample_rate);
			ret = copy_to_user((unsigned long *)arg,
					&srp.sample_rate, sizeof(unsigned long));
		}
		break;

	case SRP_GET_BIT_RATE:
		if (srp.bit_rate) {
			srpdbg("SRP: Bit Rate [%lu]\n", srp.bit_rate);
			ret = copy_to_user((unsigned long *)arg,
					&srp.bit_rate, sizeof(unsigned long));
		}
		break;

	case SRP_SEND_EOS:
		srpdbg("SRP: Send End-Of-Stream!!\n");
		if (srp.wbuf_fill_size == 0) {
			srp.stop_after_eos = 1;
		} else if (srp.wbuf_fill_size < srp.ibuf_size * 2) {
			srpdbg("SRP: %ld, smaller than ibuf_size * 2\n", srp.wbuf_fill_size);
			srp.wait_for_eos = 1;
			srp_fill_ibuf();
			srp.ibuf_req_skip = 0;
			srp_set_stream_size();
			srp_pending_ctrl(RUN);
			srp.decoding_started = 1;
		} else if (srp.wbuf_fill_size >= srp.ibuf_size * 2) {
			srpdbg("SRP: %ld Bigger than ibuf * 2!!\n", srp.wbuf_fill_size);
			srp.wait_for_eos = 1;
		}
		break;

	case SRP_STOP_EOS_STATE:
		val = srp.stop_after_eos;

		srpdbg("SRP: Stop [%s]\n", val == 1 ? "ON" : "OFF");
		if (val) {
			srpdbg("SRP: Stop at EOS [0x%08lX:0x%08X]\n",
			srp.wbuf_pos,
			readl(srp.commbox + SRP_READ_BITSTREAM_SIZE));
		}
		val = copy_to_user((unsigned long *)arg,
			&val, sizeof(unsigned long));
		break;
	}

	mutex_unlock(&srp_mutex);

	return ret;
}

static int srp_open(struct inode *inode, struct file *file)
{
	srpdbg("SRP: srp is open!!\n");

	mutex_lock(&srp_mutex);
	if (srp.is_opened) {
		srpdbg("SRP: SRP is already opened.\n");
		mutex_unlock(&srp_mutex);
		return -1;
	}
	srp.is_opened = 1;
	mutex_unlock(&srp_mutex);

	srp.audss_clk_enable(true);

	if (!(file->f_flags & O_NONBLOCK)) {
		srpdbg("SRP: Block Mode\n");
		srp.block_mode = 1;
	} else {
		srpdbg("SRP: NonBlock Mode\n");
		srp.block_mode = 0;
	}

	srp.channel = 0;
	srp.frame_size = 0;
	srp.sample_rate = 0;
	srp_reset_frame_counter();
	srp_set_default_fw();

	return 0;
}

static int srp_release(struct inode *inode, struct file *file)
{
	srpdbg("SRP: srp is released!\n");

	mutex_lock(&srp_mutex);

	/* Reset commbox */
	srp_commbox_deinit();
	srp.is_opened = 0;

	mutex_unlock(&srp_mutex);

	return 0;
}

static int srp_mmap(struct file *filep, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned int pfn;

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	pfn = __phys_to_pfn(SRP_DMEM_BASE);

	if (remap_pfn_range(vma, vma->vm_start, pfn, size, vma->vm_page_prot)) {
		srpdbg("failed to mmap for Obuf\n");
		return -EAGAIN;
	}

	return 0;
}

static void srp_check_obuf_info(void)
{
	unsigned int buf0 = readl(srp.commbox + SRP_PCM_BUFF0);
	unsigned int buf1 = readl(srp.commbox + SRP_PCM_BUFF1);
	unsigned int size = readl(srp.commbox + SRP_PCM_BUFF_SIZE);

	if (srp.obuf0_pa != buf0)
		srpdbg("SRP: Wrong PCM BUF0[0x%x], OBUF0[0x%x]\n",
						buf0, srp.obuf0_pa);
	if (srp.obuf1_pa != buf1)
		srpdbg("SRP: Wrong PCM BUF1[0x%x], OBUF1[0x%x]\n",
						buf1, srp.obuf1_pa);
	if ((srp.obuf_size >> 2) != size)
		srpdbg("SRP: Wrong OBUF SIZE[%d]\n", size);
}

static irqreturn_t srp_irq(int irqno, void *dev_id)
{
	unsigned int pending = 0;
	unsigned int cfgr = 0;
	unsigned long irq_code = readl(srp.commbox + SRP_INTERRUPT_CODE);
	unsigned long irq_info = readl(srp.commbox + SRP_INFORMATION);
	unsigned long irq_code_req;

	pending = readl(srp.commbox + SRP_PENDING);
	cfgr = readl(srp.commbox + SRP_CFGR);

	srpdbg("IRQ: Code [%08lX], Pending [%s], CFGR [0x%x]",
			irq_code, pending ? "STALL" : "RUN", cfgr);

	irq_code &= SRP_INTR_CODE_MASK;
	irq_info &= SRP_INTR_INFO_MASK;

	if (irq_code & SRP_INTR_CODE_REQUEST) {
		irq_code_req = irq_code & SRP_INTR_CODE_REQUEST_MASK;
		switch (irq_code_req) {
		case SRP_INTR_CODE_NOTIFY_INFO:
			srp_check_stream_info();
			if (cfgr & SRP_CFGR_FLOW_CTRL_ON)
				cfgr &= ~SRP_CFGR_FLOW_CTRL_ON;

			if (!(cfgr & SRP_CFGR_USE_I2S_INTR))
				cfgr |= SRP_CFGR_USE_I2S_INTR;

			writel(cfgr, srp.commbox + SRP_CFGR);
			break;

		case SRP_INTR_CODE_IBUF_REQUEST:
			srp_check_stream_info();
			if (srp.ibuf_req_skip) {
				srpdbg("SRP-IRQ: first ibuf req skip!!!\n");
				srp.ibuf_req_skip = 0;
				break;
			}

			if ((irq_code & SRP_INTR_CODE_IBUF_MASK)
				== SRP_INTR_CODE_IBUF0_EMPTY) {
				srpdbg("SRP-IRQ: IBUF0 empty\n");
				srp.ibuf_empty[0] = 1;
			} else {
				srpdbg("SRP-IRQ: IBUF1 empty\n");
				srp.ibuf_empty[1] = 1;
			}

			srp_fill_ibuf();
			if (srp.decoding_started) {
				if (srp.ibuf_empty[0] && srp.ibuf_empty[1]) {
					if (srp.wait_for_eos) {
						srpdbg("SRP-IRQ: Stop at EOS (Both buffer empty)\n");
						srp.stop_after_eos = 1;
					}
				} else {
					if (srp.wait_for_eos & !srp.wbuf_pos) {
						srpdbg("SRP-IRQ: Set stream size for EOS\n");
						srp_set_stream_size();
					}
				}
			}

			break;

		case SRP_INTR_CODE_OBUF_FULL:
			srp.is_pending = STALL;

			if ((irq_code & SRP_INTR_CODE_OBUF_MASK)
				==  SRP_INTR_CODE_OBUF0_FULL) {
				srpdbg("SRP-IRQ: OBUF0 FULL\n");
				srp.obuf_fill_done[0] = 1;
			} else {
				srpdbg("SRP-IRQ: OBUF1 FULL\n");
				srp.obuf_fill_done[1] = 1;
			}

			if (srp.first_decoding) {
				if (!srp.obuf_fill_done[srp.obuf_next]) {
					srpdbg("SRP-IRQ: Decoding start for filling both OBUF\n");
					srp_pending_ctrl(RUN);
				}

				if (srp.obuf_fill_done[0] && srp.obuf_fill_done[1]) {
					srp.first_decoding = 0;
					if (waitqueue_active(&srp.read_wait))
						wake_up_interruptible(&srp.read_wait);
				}
			} else {
				if (waitqueue_active(&srp.read_wait))
					wake_up_interruptible(&srp.read_wait);
			}

			break;

		default:
			break;
		}
	}

	if (irq_code & SRP_INTR_CODE_NOTIFY_OBUF)
		srp_check_obuf_info();

	if (irq_code & SRP_INTR_CODE_PLAYDONE) {
		srpdbg("SRP-IRQ: Play Done interrupt!!\n");
		srp.stop_after_eos = 1;
		srp_pending_ctrl(STALL);
		if (waitqueue_active(&srp.read_wait))
			wake_up_interruptible(&srp.read_wait);
	}

	if (irq_code & SRP_INTR_CODE_UART_OUTPUT) {
		srpdbg("SRP-IRQ: UART Code received [0x%08X]\n",
		readl(srp.commbox + SRP_UART_INFORMATION));
	}

	writel(0x00000000, srp.commbox + SRP_INTERRUPT_CODE);
	writel(0x00000000, srp.commbox + SRP_INTERRUPT);

	return IRQ_HANDLED;
}

static int srp_prepare_fw_buff(struct device *dev)
{
	unsigned long mem_paddr;
	struct cma_info mem_info;
	int err;

	err = cma_info(&mem_info, dev, 0);
	if (err) {
		pr_err("SRP: Failed to get cma info\n");
		return -ENOMEM;
	}
	srp.fw_mem_base = cma_alloc(dev, "srp", BASE_MEM_SIZE, 0);
	srp.fw_mem_base_pa = (unsigned long)srp.fw_mem_base;
	if (IS_ERR_VALUE(srp.fw_mem_base_pa)) {
		pr_err("SRP: Failed to cma alloc for srp\n");
		return -ENOMEM;
	}

	mem_paddr = srp.fw_mem_base_pa;
	srp.fw_code_vliw_pa = mem_paddr;
	srp.fw_code_vliw = phys_to_virt(srp.fw_code_vliw_pa);
	mem_paddr += VLIW_SIZE_MAX;

	srp.fw_code_cga_pa = mem_paddr;
	srp.fw_code_cga = phys_to_virt(srp.fw_code_cga_pa);
	mem_paddr += CGA_SIZE_MAX;

	srp.fw_data_pa = mem_paddr;
	srp.fw_data = phys_to_virt(srp.fw_data_pa);
	mem_paddr += DATA_SIZE_MAX;

	srp.fw_code_vliw_size = rp_fw_vliw_len;
	srp.fw_code_cga_size = rp_fw_cga_len;
	srp.fw_data_size = rp_fw_data_len;

	/* Clear Firmware memory & IBUF */
	memset(srp.fw_code_vliw, 0, VLIW_SIZE);
	memset(srp.fw_code_cga, 0, CGA_SIZE);
	memset(srp.fw_data, 0, DATA_SIZE);

	srp.vliw_rp = srp.fw_code_vliw_pa;

	/* Copy Firmware */
	memcpy(srp.fw_code_vliw, rp_fw_vliw,
			srp.fw_code_vliw_size);
	memcpy(srp.fw_code_cga, rp_fw_cga,
			srp.fw_code_cga_size);
	memcpy(srp.fw_data, rp_fw_data,
			srp.fw_data_size);

	srp.wbuf = dma_alloc_writecombine(dev, WBUF_SIZE,
			   (dma_addr_t *)&srp.wbuf_pa, GFP_KERNEL);

	srp.ibuf_info = kzalloc(sizeof(struct srp_buf_info), GFP_KERNEL);
	srp.obuf_info = kzalloc(sizeof(struct srp_buf_info), GFP_KERNEL);
	srp.pcm_info = kzalloc(sizeof(struct srp_buf_info), GFP_KERNEL);
	if (!srp.ibuf_info || !srp.obuf_info || !srp.pcm_info) {
		srpdbg("SRP: Failed to alloc buf info\n");
		return -ENOMEM;
	}

	srp.ibuf_size = IBUF_SIZE;
	srp.obuf_size = OBUF_SIZE;

	memset(srp.ibuf0, 0xFF, srp.ibuf_size);
	memset(srp.ibuf1, 0xFF, srp.ibuf_size);

	srpdbg("SRP: [VA]IBUF0[0x%p], [PA]IBUF0[0x%x]\n",
						srp.ibuf0, srp.ibuf0_pa);
	srpdbg("SRP: [VA]IBUF1[0x%p], [PA]IBUF1[0x%x]\n",
						srp.ibuf1, srp.ibuf1_pa);
	srpdbg("SRP: [VA]OBUF0[0x%p], [PA]OBUF0[0x%x]\n",
						srp.obuf0, srp.obuf0_pa);
	srpdbg("SRP: [VA]OBUF1[0x%p], [PA]OBUF1[0x%x]\n",
						srp.obuf1, srp.obuf1_pa);
	srpdbg("SRP: [VA]WBUF [0x%p], [PA]WBUF[0x%x]\n",
						srp.wbuf, srp.wbuf_pa);
	srpdbg("SRP: IBUF SIZE [%ld]Bytes, OBUF SIZE [%ld]Bytes\n",
						srp.ibuf_size, srp.obuf_size);

	return 0;
}

static int srp_remove_fw_buff(void)
{
	dma_free_writecombine(0, WBUF_SIZE, srp.wbuf, srp.wbuf_pa);

	srp.fw_code_vliw_pa = 0;
	srp.fw_code_cga_pa = 0;
	srp.fw_data_pa = 0;
	srp.wbuf_pa = 0;
	srp.ibuf0_pa = 0;
	srp.ibuf1_pa = 0;
	srp.obuf0_pa = 0;
	srp.obuf1_pa = 0;

	return 0;
}

static const struct file_operations srp_fops = {
	.owner		= THIS_MODULE,
	.open		= srp_open,
	.release	= srp_release,
	.read		= srp_read,
	.write		= srp_write,
	.unlocked_ioctl	= srp_ioctl,
	.mmap		= srp_mmap,
};

static struct miscdevice srp_miscdev = {
	.minor		= SRP_DEV_MINOR,
	.name		= "srp",
	.fops		= &srp_fops,
};

static int srp_resume(struct platform_device *pdev)
{
	/* TODO: Restore ibuf and wbuf */
	return 0;
}

static int srp_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: Backup ibuf and wbuf */
	return 0;
}

static __devinit int srp_probe(struct platform_device *pdev)
{
	int ret = 0;

	srp.iram = ioremap(SRP_IRAM_BASE, IRAM_SIZE);
	if (srp.iram == NULL) {
		srpdbg("SRP: Failed to ioremap for sram area\n");
		ret = -ENOMEM;
		return ret;

	}

	srp.dmem = ioremap(SRP_DMEM_BASE, DMEM_SIZE);
	if (srp.dmem == NULL) {
		srpdbg("SRP: Failed to ioremap for sram area\n");
		ret = -ENOMEM;
		goto err1;

	}

	srp.icache = ioremap(SRP_ICACHE_ADDR, ICACHE_SIZE);
	if (srp.icache == NULL) {
		srpdbg("SRP: Failed to ioremap for audio subsystem\n");
		ret = -ENOMEM;
		goto err2;
	}

	srp.commbox = ioremap(SRP_COMMBOX_BASE, COMMBOX_SIZE);
	if (srp.commbox == NULL) {
		srpdbg("SRP: Failed to ioremap for audio subsystem\n");
		ret = -ENOMEM;
		goto err3;
	}

	srp.ibuf0 = srp.iram + IBUF_OFFSET;
	srp.ibuf1 = srp.ibuf0 + IBUF_SIZE;

	srp.obuf0 = srp.dmem + OBUF_OFFSET;
	srp.obuf1 = srp.obuf0 + OBUF_SIZE;

	srp.ibuf0_pa = SRP_IBUF_PHY_ADDR;
	srp.obuf0_pa = SRP_OBUF_PHY_ADDR;

	srp.ibuf1_pa = srp.ibuf0_pa + IBUF_SIZE;
	srp.obuf1_pa = srp.obuf0_pa + OBUF_SIZE;

	ret = srp_prepare_fw_buff(&pdev->dev);
	if (ret) {
		pr_err("SRP: Can't prepare memory for srp\n");
		goto err4;
	}

	ret = request_irq(IRQ_AUDIO_SS, srp_irq, 0, "samsung-rp", pdev);
	if (ret < 0) {
		pr_err("SRP: Fail to claim SRP(AUDIO_SS) irq\n");
		goto err5;
	}

	srp.audss_clk_enable = audss_clk_enable;
	init_waitqueue_head(&srp.read_wait);
	spin_lock_init(&srp.lock);

	ret = misc_register(&srp_miscdev);
	if (ret) {
		pr_err("SRP: Cannot register miscdev on minor=%d\n",
			SRP_DEV_MINOR);
		goto err6;
	}

	return 0;

err6:
	free_irq(IRQ_AUDIO_SS, pdev);
err5:
	srp_remove_fw_buff();
err4:
	iounmap(srp.commbox);
err3:
	iounmap(srp.icache);
err2:
	iounmap(srp.dmem);
err1:
	iounmap(srp.iram);

	return ret;
}

static __devexit int srp_remove(struct platform_device *pdev)
{
	free_irq(IRQ_AUDIO_SS, pdev);
	srp_remove_fw_buff();

	misc_deregister(&srp_miscdev);

	iounmap(srp.commbox);
	iounmap(srp.icache);
	iounmap(srp.dmem);
	iounmap(srp.iram);

	return 0;
}

static struct platform_driver srp_driver = {
	.probe		= srp_probe,
	.remove		= srp_remove,
	.suspend	= srp_suspend,
	.resume		= srp_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "samsung-rp",
	},
};

static char banner[] __initdata =
	KERN_INFO "Samsung SRP driver, (c) 2011 Samsung Electronics\n";

static int __init srp_init(void)
{
	printk(banner);

	return platform_driver_register(&srp_driver);
}

static void __exit srp_exit(void)
{
	platform_driver_unregister(&srp_driver);
}

module_init(srp_init);
module_exit(srp_exit);

MODULE_AUTHOR("Yeongman Seo <yman.seo@samsung.com>");
MODULE_DESCRIPTION("Samsung SRP driver");
MODULE_LICENSE("GPL");
