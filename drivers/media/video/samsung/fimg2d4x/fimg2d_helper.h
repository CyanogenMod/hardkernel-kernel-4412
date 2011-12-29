/* linux/drivers/media/video/samsung/fimg2d4x/fimg2d_helper.h
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *	http://www.samsung.com/
 *
 * Samsung Graphics 2D driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#ifndef __FIMG2D_HELPER_H
#define __FIMG2D_HELPER_H

#include "fimg2d.h"

static inline char *imagename(enum image_object image)
{
	switch (image) {
	case IDST:
		return "DST";
	case ISRC:
		return "SRC";
	case IMSK:
		return "MSK";
	default:
		return NULL;
	}
}

static inline long elapsed_microsec(struct timeval *start,
				struct timeval *end, char *msg)
{
	long sec, usec, time;

	sec = end->tv_sec - start->tv_sec;
	if (end->tv_usec >= start->tv_usec) {
		usec = end->tv_usec - start->tv_usec;
	} else {
		usec = end->tv_usec + 1000000 - start->tv_usec;
		sec--;
	}
	time = sec * 1000000 + usec;
	printk(KERN_INFO "[%s] %ld usec elapsed\n", msg, time);

	return time; /* microseconds */
}

static inline unsigned long long elapsed_nanosec(unsigned long long start,
					unsigned long long end, char *msg)
{
	unsigned long long time;

	time = end - start;
	printk(KERN_INFO "[%s] %llu nsec elapsed\n", msg, time);
	return time; /* nanoseconds */
}

int point_to_offset(int point, enum color_format cf);
int width_to_bytes(int pixels, enum color_format cf);
void fimg2d_print_params(struct fimg2d_blit __user *u);
void fimg2d_dump_command(struct fimg2d_bltcmd *cmd);

#endif /* __FIMG2D_HELPER_H */
