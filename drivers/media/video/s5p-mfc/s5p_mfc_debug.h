/*
 * drivers/media/video/samsung/mfc5/s5p_mfc_debug.h
 *
 * Header file for Samsung MFC (Multi Function Codec - FIMV) driver
 * This file contains debug macros
 *
 * Kamil Debski, Copyright (c) 2010 Samsung Electronics
 * http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef S5P_MFC_DEBUG_H_
#define S5P_MFC_DEBUG_H_

#define DEBUG

#ifdef DEBUG
extern int debug;
/* Debug macro */

/*
dev_dbg(dev->v4l2_dev.dev, "%s:%s:%d:" fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__);	\
*/
#define mfc_debug(fmt, ...)						\
	do {								\
		if (debug)						\
			dev_dbg(dev->v4l2_dev.dev, "%s:%d:" fmt, __func__, __LINE__, ##__VA_ARGS__);	\
	}while (0)
#else
#define mfc_debug(fmt, ...)
#endif

#define mfc_debug_enter() mfc_debug("enter")
#define mfc_debug_leave() mfc_debug("leave")

#define mfc_err(fmt, ...)						\
	do {								\
			dev_err(dev->v4l2_dev.dev,  "%s:%s:%d:" fmt, __FILE__, __func__, __LINE__, ##__VA_ARGS__);	\
	}while (0)

#define mfc_info(fmt, ...)						\
	do {								\
			dev_info(dev->v4l2_dev.dev, "%s:%s:%d:" fmt,  __FILE__, __func__, __LINE__, ##__VA_ARGS__);\
	}while (0)

#endif /* S5P_MFC_DEBUG_H_ */

