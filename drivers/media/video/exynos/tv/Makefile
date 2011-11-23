# drivers/media/video/samsung/tvout/Makefile
#
# Copyright (c) 2010-2011 Samsung Electronics Co., Ltd.
#	http://www.samsung.com/
# Tomasz Stanislawski <t.stanislaws@samsung.com>
#
# Licensed under GPL

obj-$(CONFIG_VIDEO_EXYNOS_HDMIPHY) += s5p-hdmiphy.o
s5p-hdmiphy-y += hdmiphy_drv.o
obj-$(CONFIG_VIDEO_EXYNOS_HDMI) += s5p-hdmi.o
s5p-hdmi-y += hdcp_drv.o hdmi_drv.o
obj-$(CONFIG_VIDEO_EXYNOS_SDO) += s5p-sdo.o
s5p-sdo-y += sdo_drv.o
obj-$(CONFIG_VIDEO_EXYNOS_MIXER) += s5p-mixer.o
s5p-mixer-y += mixer_drv.o mixer_video.o mixer_reg.o mixer_grp_layer.o

ifeq ($(CONFIG_VIDEO_EXYNOS_HDMI),y)
obj-$(CONFIG_CPU_EXYNOS4210) += mixer_vp_layer.o
obj-$(CONFIG_CPU_EXYNOS5210) += mixer_video_layer.o
obj-$(CONFIG_CPU_EXYNOS4210) += hdmi_reg_4210.o
obj-$(CONFIG_CPU_EXYNOS5210) += hdmi_reg_5210.o
obj-$(CONFIG_CPU_EXYNOS4210) += hdmiphy_conf_4210.o
obj-$(CONFIG_CPU_EXYNOS5210) += hdmiphy_conf_5210.o
endif

