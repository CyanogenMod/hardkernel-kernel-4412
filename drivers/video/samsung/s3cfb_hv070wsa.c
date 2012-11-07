/* linux/drivers/video/samsung/s3cfb_hv070wsa.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * HV070WSA 7" Landscape LCD module driver for the ORIGEN
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include "s3cfb.h"

static struct s3cfb_lcd hv070wsa = {
	.width  = 1024,
	.height = 600,
	.bpp    = 32,
	.freq   = 60,

	.timing = {
		.h_fp = 105,
		.h_bp = 213,
		.h_sw = 2,

		.v_fp = 10,
		.v_fpe = 0,
		.v_bp = 23,
		.v_bpe = 0,
		.v_sw = 2,
	},

	.polarity = {
		.rise_vclk = 1,
		.inv_hsync = 1,
		.inv_vsync = 1,
		.inv_vden = 0,
	},
};

/* name should be fixed as 's3cfb_set_lcd_info' */
void s3cfb_set_lcd_info(struct s3cfb_global *ctrl)
{
	hv070wsa.init_ldi = NULL;
	ctrl->lcd = &hv070wsa;
}
