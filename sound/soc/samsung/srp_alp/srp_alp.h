#ifndef __SRP_ALP_H
#define __SRP_ALP_H

#define SRP_DEV_MINOR	(250)

/* SRAM information */
#if defined(CONFIG_CPU_EXYNOS4210)
#define IRAM_SIZE	(0x20000)
#else
#define IRAM_SIZE	(0x40000)
#endif
#define DMEM_SIZE	(0x20000)
#define ICACHE_SIZE	(0x10000)

/* Buffer information */
#define IBUF_SIZE	(0x4000)
#define OBUF_SIZE	(0x8000)
#define WBUF_SIZE	(IBUF_SIZE * 4)
#if defined(CONFIG_CPU_EXYNOS4210)
#define IBUF_OFFSET	(0x10000)
#else
#define IBUF_OFFSET	(0x30000)
#endif
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
#define srp_info(x... )	pr_info("SRP: " x)
#define srp_debug(x...)	pr_debug("SRP: " x)
#define srp_err(x...)	pr_err("SRP_ERR: " x)
#else
#define srp_info(x...)
#define srp_debug(x...)
#define srp_err(x...)
#endif

struct srp_buf_info {
	void		*mmapped_addr;
	void		*addr;
	unsigned int	mmapped_size;
	unsigned int	size;
	int		num;
};

struct srp_info {
	struct srp_buf_info	ibuf_info;
	struct srp_buf_info	obuf_info;
	struct srp_buf_info	pcm_info;

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
	unsigned long	wbuf_pos;		/* Write pointer */
	unsigned long	wbuf_fill_size;		/* Total size by user write() */
	unsigned int	wbuf_ready;

	unsigned int ibuf_next;
	unsigned int ibuf_empty[2];

	unsigned int obuf_fill_done[2];
	unsigned int obuf_ready;
	unsigned int obuf_next;

	unsigned long frame_size;
	unsigned long frame_count;
	unsigned long frame_count_base;
	unsigned long channel;			/* Mono = 1, Stereo = 2 */
	unsigned long sample_rate;		/* Sampling Rate 8000 ~ 48000 */
	unsigned long bit_rate;
	unsigned long set_bitstream_size;

	unsigned int first_decoding;
	unsigned int decoding_started;
	unsigned int wakeup_waitqueue;
	unsigned int is_opened;			/* Running status of SRP */
	unsigned int is_running;		/* Open status of SRP */
	unsigned int is_pending;		/* Pending status of SRP */
	unsigned int block_mode;		/* Block Mode */
	unsigned int stop_after_eos;
	unsigned int wait_for_eos;
	unsigned int prepare_for_eos;
	unsigned int play_done;
	unsigned int save_ibuf_empty;

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
};

/* SRP Pending On/Off status */
enum {
	RUN = 0,
	STALL,
};

#endif /* __SRP_ALP_H */
