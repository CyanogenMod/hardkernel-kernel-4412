/* linux/drivers/usb/gadget/exynos_ss_udc.c
 *
 * Copyright 2011 Samsung Electronics Co., Ltd.
 *	http://www.samsung.com/
 *
 * EXYNOS SuperSpeed USB 3.0 Device Controlle driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/clk.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <mach/map.h>

#include <plat/regs-usb3-exynos-udc-phy.h>
#include <plat/regs-usb3-exynos-udc.h>
//#include <mach/regs-sys.h>
#include <plat/udc-ss.h>
#include <plat/cpu.h>



#define DMA_ADDR_INVALID (~((dma_addr_t)0))
#define EP0_MPS_LIMIT	512
#define EXYNOS_USB3_EPS	(9)

/* Has to be multiple of four */
#define EXYNOS_USB3_EVENT_BUFF_WSIZE	256
#define EXYNOS_USB3_EVENT_BUFF_BSIZE	(EXYNOS_USB3_EVENT_BUFF_WSIZE << 2)

#define call_gadget(_udc, _entry) \
	if ((_udc)->gadget.speed != USB_SPEED_UNKNOWN &&	\
	    (_udc)->driver && (_udc)->driver->_entry)	\
		(_udc)->driver->_entry(&(_udc)->gadget);

/* BOS Descriptor (Binary Object Store) */
struct bos_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wTotalLength;
	uint8_t bNumDeviceCaps;
} __attribute__((__packed__));

struct usb20_ext_cap_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDevCapabilityType;
	uint32_t bmAttributes;
} __attribute__((__packed__));

struct superspeed_cap_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDevCapabilityType;
	uint8_t bmAttributes;
	uint16_t wSpeedsSupported;
	uint8_t bFunctionalitySupport;
	uint8_t bU1DevExitLat;
	uint16_t wU2DevExitLat;
} __attribute__((__packed__));

struct container_id_cap_desc {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDevCapabilityType;
	uint8_t bReserved;
	uint8_t containerID[16];
} __attribute__((__packed__));

#define USB_CAP_20_EXT	0x2
#define USB_CAP_SS	0x3
#define USB_CAP_CID	0x4

static struct usb20_ext_cap_desc cap1 = {
	.bLength = sizeof(struct usb20_ext_cap_desc),
	.bDescriptorType = USB_DT_DEVICE_CAPABILITY,
	.bDevCapabilityType = USB_CAP_20_EXT,
	.bmAttributes = 0x2,
};

static struct superspeed_cap_desc cap2 = {
	.bLength = sizeof(struct superspeed_cap_desc),
	.bDescriptorType = USB_DT_DEVICE_CAPABILITY,
	.bDevCapabilityType = USB_CAP_SS,
	.bmAttributes = 0x0,
	.wSpeedsSupported = 0xc,
	.bFunctionalitySupport = 2,
	/* @todo set these to correct value */
	.bU1DevExitLat = 0x4,
	.wU2DevExitLat = 0x4,
};

static struct container_id_cap_desc cap3 = {
	.bLength = sizeof(struct container_id_cap_desc),
	.bDescriptorType = USB_DT_DEVICE_CAPABILITY,
	.bDevCapabilityType = USB_CAP_CID,
	.bReserved = 0x0,
	/* @todo Create UUID */
	.containerID = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

static struct bos_desc bos = {
	.bLength = sizeof(struct bos_desc),
	.bDescriptorType = USB_DT_BOS,
	.wTotalLength = sizeof(struct bos_desc) +
			sizeof(cap1) + sizeof(cap2) + sizeof(cap3),
	.bNumDeviceCaps = 3,
};

/**
 * struct exynos_ss_udc_trb - transfer request block (TRB)
 * @buff_ptr_low: Buffer pointer low.
 * @buff_ptr_high: Buffer pointer high.
 * @param1: TRB parameter 1.
 * @param2: TRB parameter 2.
 */
struct exynos_ss_udc_trb {
	u32 buff_ptr_low;
	u32 buff_ptr_high;
	u32 param1;
	u32 param2;
};

/**
 * struct exynos_ss_udc_ep - driver endpoint definition.
 * @ep: The gadget layer representation of the endpoint.
 * @queue: Queue of requests for this endpoint.
 * @parent: Reference back to the parent device structure.
 * @req: The current request that the endpoint is processing. This is
 *       used to indicate an request has been loaded onto the endpoint
 *       and has yet to be completed (maybe due to data move, or simply
 *	 awaiting an ack from the core all the data has been completed).
 * @lock: State lock to protect contents of endpoint.
 * @epnum: The USB endpoint number.
 * @dir_in: Set to true if this endpoint is of the IN direction, which
 *	    means that it is sending data to the Host.
 * @halted: Set if the endpoint has been halted.
 * @periodic: Set if this is a periodic ep, such as Interrupt
 * @sent_zlp: Set if we've sent a zero-length packet.
 * @name: The driver generated name for the endpoint.
 *
 * This is the driver's state for each registered enpoint, allowing it
 * to keep track of transactions that need doing. Each endpoint has a
 * lock to protect the state, to try and avoid using an overall lock
 * for the host controller as much as possible.
 */
struct exynos_ss_udc_ep {
	struct usb_ep		ep;
	struct list_head	queue;
	struct exynos_ss_udc	*parent;
	struct exynos_ss_udc_req	*req;
	spinlock_t		lock;

	unsigned char		epnum;
	unsigned int		dir_in:1;

	unsigned int		halted:1;
	unsigned int		periodic:1;
	unsigned int		sent_zlp:1;

	char			name[10];
};

/**
 * struct exynos_ss_udc_req - data transfer request
 * @req: The USB gadget request.
 * @queue: The list of requests for the endpoint this is queued for.
 * @trb: Transfer Request Block.
 * @mapped: DMA buffer for this request has been mapped via dma_map_single().
 */
struct exynos_ss_udc_req {
	struct usb_request	req;
	struct list_head	queue;
	struct exynos_ss_udc_trb	trb __attribute__ ((aligned(16))); 

	unsigned char		mapped;
};

/**
 * struct exynos_ss_udc_ep_command - endpoint command.
 * @ep: physical endpoint number.
 * @param0: Command parameter 0.
 * @param1: Command parameter 1.
 * @param2: Command parameter 2.
 * @cmdtype: Command to issue.
 * @cmdflags: Command flags.
 */
struct exynos_ss_udc_ep_command {
	int ep;
	u32 param0;
	u32 param1;
	u32 param2;
	u32 cmdtyp;
	u32 cmdflags;
};

/**
 * struct exynos_ss_udc - driver state.
 * @dev: The parent device supplied to the probe function
 * @driver: USB gadget driver
 * @plat: The platform specific configuration data.
 * @regs: The memory area mapped for accessing registers.
 * @regs_res: The resource that was allocated when claiming register space.
 * @irq: The IRQ number we are using.
 * @clk: The clock we are using.
 * @event_buff: Event buffer.
 * @event_buff_dma: DMA address of event buffer
 * @ep0_reply: Request used for ep0 reply.
 * @ctrl_req: Request for EP0 control packets.
 * @gadget: Represents USB slave device.
 * @eps: The endpoints being supplied to the gadget framework
 */
struct exynos_ss_udc {
	struct device		 *dev;
	struct usb_gadget_driver *driver;
	struct exynos_ss_udc_plat	 *plat;

	void __iomem		*regs;
	struct resource		*regs_res;
	int			irq;
	struct clk		*clk;

	/* According to documentation the address must be EVNTSIZ-aligned */
	u32			event_buff[EXYNOS_USB3_EVENT_BUFF_WSIZE]
			__attribute__ ((aligned(EXYNOS_USB3_EVENT_BUFF_BSIZE)));
	dma_addr_t		event_buff_dma;	

	bool			ep0_setup;
	u8			*ep0_buff;
	struct usb_request	*ep0_reply;
	struct usb_request	*ctrl_req;

	struct exynos_ss_udc_ep_command epcmd;

	struct usb_gadget	gadget;
	struct exynos_ss_udc_ep	eps[];

};

static void exynos_ss_udc_enqueue_setup(struct exynos_ss_udc *udc);
static int exynos_ss_udc_ep_queue(struct usb_ep *ep, struct usb_request *req,
			      gfp_t gfp_flags);
static void exynos_ss_udc_kill_all_requests(struct exynos_ss_udc *udc,
			      struct exynos_ss_udc_ep *ep,
			      int result, bool force);

static struct exynos_ss_udc *our_udc;

static struct usb_gadget_ops exynos_ss_udc_gadget_ops = {
};

/* conversion functions */
static inline struct exynos_ss_udc_req *our_req(struct usb_request *req)
{
	return container_of(req, struct exynos_ss_udc_req, req);
}

static inline struct exynos_ss_udc_ep *our_ep(struct usb_ep *ep)
{
	return container_of(ep, struct exynos_ss_udc_ep, ep);
}

static inline void __orr32(void __iomem *ptr, u32 val)
{
	writel(readl(ptr) | val, ptr);
}

static inline void __bic32(void __iomem *ptr, u32 val)
{
	writel(readl(ptr) & ~val, ptr);
}

static inline int get_phys_epnum(struct exynos_ss_udc_ep *udc_ep)
{
	return (udc_ep->epnum * 2 + udc_ep->dir_in);
}

static inline int get_usb_epnum(int index)
{
	return (index >> 1);
}

static bool exynos_ss_udc_poll_bit_clear(void __iomem *ptr, u32 val, int timeout)
{
	u32 reg;

	do {
		reg = readl(ptr);
	} while ((reg & val) && timeout-- > 0);

	if (reg & val)
		return false;

	return true;
}

static bool exynos_ss_udc_issue_cmd(struct exynos_ss_udc *udc,
				 struct exynos_ss_udc_ep_command *epcmd)
{
	bool res;
	u32 depcmd;

	/* If some of parameters are not in use, we will write it anyway
	   for simplification */
	writel(epcmd->param0, udc->regs + EXYNOS_USB3_DEPCMDPAR0(epcmd->ep));
	writel(epcmd->param1, udc->regs + EXYNOS_USB3_DEPCMDPAR1(epcmd->ep));
	writel(epcmd->param2, udc->regs + EXYNOS_USB3_DEPCMDPAR2(epcmd->ep));

	depcmd = epcmd->cmdtyp | epcmd->cmdflags;
	writel(depcmd, udc->regs + EXYNOS_USB3_DEPCMD(epcmd->ep));

	res = exynos_ss_udc_poll_bit_clear(udc->regs + EXYNOS_USB3_DEPCMD(0),
					EXYNOS_USB3_DEPCMDx_CmdAct,
					1000);
	return res;
}

/**
 * get_ep_head - return the first request on the endpoint
 * @udc_ep: The controller endpoint to get
 *
 * Get the first request on the endpoint.
*/
static struct exynos_ss_udc_req *get_ep_head(struct exynos_ss_udc_ep *udc_ep)
{
	if (list_empty(&udc_ep->queue))
		return NULL;

	return list_first_entry(&udc_ep->queue, struct exynos_ss_udc_req, queue);
}

/**
 * exynos_ss_udc_map_dma - map the DMA memory being used for the request
 * @udc: The device state.
 * @udc_ep: The endpoint the request is on.
 * @req: The request being processed.
 *
 * We've been asked to queue a request, so ensure that the memory buffer
 * is correctly setup for DMA. If we've been passed an extant DMA address
 * then ensure the buffer has been synced to memory. If our buffer has no
 * DMA memory, then we map the memory and mark our request to allow us to
 * cleanup on completion.
*/
static int exynos_ss_udc_map_dma(struct exynos_ss_udc *udc,
			     struct exynos_ss_udc_ep *udc_ep,
			     struct usb_request *req)
{
	enum dma_data_direction dir;
	struct exynos_ss_udc_req *udc_req = our_req(req);

	dir = udc_ep->dir_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* if the length is zero, ignore the DMA data */
	if (udc_req->req.length == 0)
		return 0;

	if (req->dma == DMA_ADDR_INVALID) {
		dma_addr_t dma;

		dma = dma_map_single(udc->dev,
					req->buf, req->length, dir);

		if (unlikely(dma_mapping_error(udc->dev, dma)))
			goto dma_error;

		udc_req->mapped = 1;
		req->dma = dma;
	} else
		dma_sync_single_for_device(udc->dev,
				req->dma, req->length, dir);

	return 0;

dma_error:
	dev_err(udc->dev, "%s: failed to map buffer %p, %d bytes\n",
		__func__, req->buf, req->length);

	return -EIO;
}

/**
 * exynos_ss_udc_unmap_dma - unmap the DMA memory being used for the request
 * @udc: The device state.
 * @udc_ep: The endpoint for the request
 * @udc_req: The request being processed.
 *
 * This is the reverse of exynos_ss_udc_map_dma(), called for the completion
 * of a request to ensure the buffer is ready for access by the caller.
*/
static void exynos_ss_udc_unmap_dma(struct exynos_ss_udc *udc,
				struct exynos_ss_udc_ep *udc_ep,
				struct exynos_ss_udc_req *udc_req)
{
	struct usb_request *req = &udc_req->req;
	enum dma_data_direction dir;

	dir = udc_ep->dir_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE;

	/* ignore this if we're not moving any data */
	if (udc_req->req.length == 0)
		return;

	if (udc_req->mapped) {
		/* we mapped this, so unmap and remove the dma */

		dma_unmap_single(udc->dev, req->dma, req->length, dir);

		req->dma = DMA_ADDR_INVALID;
		udc_req->mapped = 0;
	}
}

/**
 * ep_from_windex - convert control wIndex value to endpoint
 * @udc: The driver state.
 * @windex: The control request wIndex field (in host order).
 *
 * Convert the given wIndex into a pointer to an driver endpoint
 * structure, or return NULL if it is not a valid endpoint.
*/
static struct exynos_ss_udc_ep *ep_from_windex(struct exynos_ss_udc *udc,
					   u32 windex)
{
	struct exynos_ss_udc_ep *ep = &udc->eps[windex & 0x7F];
	int dir = (windex & USB_DIR_IN) ? 1 : 0;
	int idx = windex & 0x7F;

	if (windex >= 0x100)
		return NULL;

	if (idx > EXYNOS_USB3_EPS)
		return NULL;

	if (idx && ep->dir_in != dir)
		return NULL;

	return ep;
}

/**
 * exynos_ss_udc_ep_enable - enable the given endpoint
 * @ep: The USB endpint to configure
 * @desc: The USB endpoint descriptor to configure with.
 *
 * This is called from the USB gadget code's usb_ep_enable().
*/
static int exynos_ss_udc_ep_enable(struct usb_ep *ep,
			       const struct usb_endpoint_descriptor *desc)
{
	struct exynos_ss_udc_ep *udc_ep = our_ep(ep);
	struct exynos_ss_udc *udc = udc_ep->parent;
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	unsigned long flags;
	int epnum = udc_ep->epnum;
	u32 mps;
	int dir_in;
	int ret = 0;
	bool res;

	dev_dbg(udc->dev,
		"%s: ep %s: a 0x%02x, attr 0x%02x, mps 0x%04x, intr %d\n",
		__func__, ep->name, desc->bEndpointAddress, desc->bmAttributes,
		desc->wMaxPacketSize, desc->bInterval);

	/* not to be called for EP0 */
	WARN_ON(epnum == 0);

	dir_in = (desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) ? 1 : 0;
	if (dir_in != udc_ep->dir_in) {
		dev_err(udc->dev, "%s: direction mismatch!\n", __func__);
		return -EINVAL;
	}

	mps = le16_to_cpu(desc->wMaxPacketSize);

	spin_lock_irqsave(&udc_ep->lock, flags);

	epcmd->ep = get_phys_epnum(udc_ep);

	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_MPS(mps);

	/* update the endpoint state */
	udc_ep->ep.maxpacket = mps;

	/* default, set to non-periodic */
	udc_ep->periodic = 0;

	switch (desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) {
	case USB_ENDPOINT_XFER_ISOC:
		dev_err(udc->dev, "no current ISOC support\n");
		ret = -EINVAL;
		goto out;

	case USB_ENDPOINT_XFER_BULK:
		epcmd->param0 |= EXYNOS_USB3_DEPCMDPAR0x_EPType(2);
		break;

	case USB_ENDPOINT_XFER_INT:
		if (dir_in)
			udc_ep->periodic = 1;

		epcmd->param0 |= EXYNOS_USB3_DEPCMDPAR0x_EPType(3);
		break;

	case USB_ENDPOINT_XFER_CONTROL:
		break;
	}

	if (dir_in)
		/* Assigne FIFO Number by simply using the endpoint number */
		epcmd->param0 |= EXYNOS_USB3_DEPCMDPAR0x_FIFONum(epnum);

	epcmd->param1 = EXYNOS_USB3_DEPCMDPAR1x_EpNum(epnum) |
			(dir_in ? EXYNOS_USB3_DEPCMDPAR1x_EpDir : 0) |
			EXYNOS_USB3_DEPCMDPAR1x_XferNRdyEn |
			EXYNOS_USB3_DEPCMDPAR1x_XferCmplEn;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev, "Failed to configure physical EP\n");
		ret = -EINVAL;
		goto out;
	}
	
	/* Configure Pysical Endpoint Transfer Resource */
	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_NumXferRes(1);
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPXFERCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev,
			"Failed to configure physical EP transfer resource\n");
		ret = -EINVAL;
		goto out;
	}

	/* Enable Physical Endpoint */
	__orr32(udc->regs + EXYNOS_USB3_DALEPENA, 1 << epcmd->ep);
out:
	spin_unlock_irqrestore(&udc_ep->lock, flags);
	return ret;
}

static int exynos_ss_udc_ep_disable(struct usb_ep *ep)
{
	struct exynos_ss_udc_ep *udc_ep = our_ep(ep);
	struct exynos_ss_udc *udc = udc_ep->parent;
	int index = get_phys_epnum(udc_ep);
	unsigned long flags;

	dev_dbg(udc->dev, "%s(ep %p)\n", __func__, ep);

	if (ep == &udc->eps[0].ep) {
		dev_err(udc->dev, "%s: called for ep0\n", __func__);
		return -EINVAL;
	}

	/* terminate all requests with shutdown */
	exynos_ss_udc_kill_all_requests(udc, udc_ep, -ESHUTDOWN, false);

	spin_lock_irqsave(&udc_ep->lock, flags);
	__bic32(udc->regs + EXYNOS_USB3_DALEPENA, 1 << index);
	spin_unlock_irqrestore(&udc_ep->lock, flags);
	return 0;
}

/**
 * @ep: USB endpoint to allocate request for.
 * @flags: Allocation flags
 *
 * Allocate a new USB request structure appropriate for the specified endpoint
 */
static struct usb_request *exynos_ss_udc_ep_alloc_request(struct usb_ep *ep,
						      gfp_t flags)
{
	struct exynos_ss_udc_req *req;

	req = kzalloc(sizeof(struct exynos_ss_udc_req), flags);
	if (!req)
		return NULL;

	INIT_LIST_HEAD(&req->queue);

	req->req.dma = DMA_ADDR_INVALID;
	return &req->req;
}

static void exynos_ss_udc_ep_free_request(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct exynos_ss_udc_req *udc_req = our_req(req);

	kfree(udc_req);
}

static int exynos_ss_udc_ep_sethalt(struct usb_ep *ep, int value)
{
	struct exynos_ss_udc_ep *udc_ep = our_ep(ep);
	struct exynos_ss_udc *udc = udc_ep->parent;
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	int index = get_phys_epnum(udc_ep);
	unsigned long irqflags;
	bool res;

	dev_info(udc->dev, "%s(ep %p %s, %d)\n", __func__, ep, ep->name, value);

	spin_lock_irqsave(&udc_ep->lock, irqflags);

	epcmd->ep = index;
	if (value)
		epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPSSTALL;
	else
		epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPCSTALL;

	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev, "Failed to set/clear stall\n");
		return -EINVAL;
	}

	/* If EP0, do it for another direction */
	if (udc_ep->epnum == 0) {
		epcmd->ep = ~udc_ep->dir_in;

		res = exynos_ss_udc_issue_cmd(udc, epcmd);
		if (!res) {
			dev_err(udc->dev, "Failed to set/clear stall\n");
			return -EINVAL;
		}
	}

	/* If everything is Ok, we mark endpoint as halted */
	udc_ep->halted = value ? 1 : 0;

	spin_unlock_irqrestore(&udc_ep->lock, irqflags);

	return 0;
}

static struct usb_ep_ops exynos_ss_udc_ep_ops = {
	.enable		= exynos_ss_udc_ep_enable,
	.disable	= exynos_ss_udc_ep_disable,
	.alloc_request	= exynos_ss_udc_ep_alloc_request,
	.free_request	= exynos_ss_udc_ep_free_request,
	.queue		= exynos_ss_udc_ep_queue,
	.set_halt	= exynos_ss_udc_ep_sethalt,
}; 

/**
 * exynos_ss_udc_complete_oursetup - setup completion callback
 * @ep: The endpoint the request was on.
 * @req: The request completed.
 *
 * Called on completion of any requests the driver itself
 * submitted that need cleaning up.
 */
static void exynos_ss_udc_complete_oursetup(struct usb_ep *ep,
					struct usb_request *req)
{
	struct exynos_ss_udc_ep *udc_ep = our_ep(ep);
	struct exynos_ss_udc *udc = udc_ep->parent;

	dev_dbg(udc->dev, "%s: ep %p, req %p\n", __func__, ep, req);

	exynos_ss_udc_ep_free_request(ep, req);
}

/**
 * exynos_ss_udc_send_reply - send reply to control request
 * @udc: The device state
 * @ep: Endpoint 0
 * @buff: Buffer for request
 * @length: Length of reply.
 *
 * Create a request and queue it on the given endpoint. This is useful as
 * an internal method of sending replies to certain control requests, etc.
 */
static int exynos_ss_udc_send_reply(struct exynos_ss_udc *udc,
				struct exynos_ss_udc_ep *ep,
				void *buff,
				int length)
{
	struct usb_request *req;
	struct exynos_ss_udc_req *udc_req;
	int ret;

	dev_dbg(udc->dev, "%s: buff %p, len %d\n", __func__, buff, length);

	req = exynos_ss_udc_ep_alloc_request(&ep->ep, GFP_ATOMIC);
	udc->ep0_reply = req;
	if (!req) {
		dev_warn(udc->dev, "%s: cannot alloc req\n", __func__);
		return -ENOMEM;
	}

	udc_req = our_req(req);

	req->buf = udc->ep0_buff;
	req->length = length;
	req->zero = 1; /* always do zero-length final transfer */
	req->complete = exynos_ss_udc_complete_oursetup;

	if (length && (buff != req->buf))
		memcpy(req->buf, buff, length);
	else
		ep->sent_zlp = 1;

	ret = exynos_ss_udc_ep_queue(&ep->ep, req, GFP_ATOMIC);
	if (ret) {
		dev_warn(udc->dev, "%s: cannot queue req\n", __func__);
		return ret;
	}

	return 0;
}

/**
 * exynos_ss_udc_start_req - start a USB request from an endpoint's queue
 * @udc: The controller state.
 * @udc_ep: The endpoint to process a request for
 * @udc_req: The request to start.
 * @continuing: True if we are doing more for the current request.
 *
 * Start the given request running by setting the endpoint registers
 * appropriately, and writing any data to the FIFOs.
 */
static void exynos_ss_udc_start_req(struct exynos_ss_udc *udc,
				 struct exynos_ss_udc_ep *udc_ep,
				 struct exynos_ss_udc_req *udc_req,
				 bool continuing)
{
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	struct usb_request *ureq = &udc_req->req;
	int epnum = udc_ep->epnum;
	bool res;

	/* TODO: below is just very minimal code to start transfer,
	 * we need to complete it!!! */

	/* If endpoint is stalled, we will restart request later */
	if (udc_ep->halted) {
		dev_warn(udc->dev, "%s: ep%d is stalled\n", __func__, epnum);
		return;
	}

	udc_req->trb.buff_ptr_low = (u32) ureq->dma + ureq->actual;
	udc_req->trb.buff_ptr_high = 0;
	/* According to Buffer Size Rules for OUT endpoints the total size
	   of a buffer must be a multiple of MaxPacketSize. But if not what
	   can we do? */
	udc_req->trb.param1 =
			EXYNOS_USB3_TRB_BUFSIZ(ureq->length - ureq->actual);
	udc_req->trb.param2 = EXYNOS_USB3_TRB_IOC | EXYNOS_USB3_TRB_LST |
			EXYNOS_USB3_TRB_HWO;

	/* Start Transfer */
	epcmd->ep = get_phys_epnum(udc_ep);
	epcmd->param0 = (u32) virt_to_phys(&udc_req->trb);
	epcmd->param1 = 0;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPSTRTXFER;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res)
		dev_err(udc->dev, "Failed to start transfer\n");

}

/**
 * exynos_ss_udc_process_req_featire - process request {SET,CLEAR}_FEATURE
 * @udc: The device state
 * @ctrl: USB control request
 */
static int exynos_ss_udc_process_req_feature(struct exynos_ss_udc *udc,
					 struct usb_ctrlrequest *ctrl)
{
	struct exynos_ss_udc_ep *ep0 = &udc->eps[0];
	struct exynos_ss_udc_req *udc_req;
	bool restart;
	bool set = (ctrl->bRequest == USB_REQ_SET_FEATURE);
	struct exynos_ss_udc_ep *ep;
	int ret;

	dev_dbg(udc->dev, "%s: %s_FEATURE\n",
		__func__, set ? "SET" : "CLEAR");

	if (ctrl->bRequestType == USB_RECIP_ENDPOINT) {
		ep = ep_from_windex(udc, le16_to_cpu(ctrl->wIndex));
		if (!ep) {
			dev_dbg(udc->dev, "%s: no endpoint for 0x%04x\n",
				__func__, le16_to_cpu(ctrl->wIndex));
			return -ENOENT;
		}

		switch (le16_to_cpu(ctrl->wValue)) {
		case USB_ENDPOINT_HALT:
			exynos_ss_udc_ep_sethalt(&ep->ep, set);

			ret = exynos_ss_udc_send_reply(udc, ep0, NULL, 0);
			if (ret) {
				dev_err(udc->dev,
					"%s: failed to send reply\n", __func__);
				return ret;
			}

			if (!set) {
				/* If we have request in progress, then
				 * complete it */
				if (ep->req) {
					udc_req = ep->req;
					ep->req = NULL;
					list_del_init(&udc_req->queue);
					udc_req->req.complete(&ep->ep,
								 &udc_req->req);
				}

				/* If we have pending request, then start it */
				restart = !list_empty(&ep->queue);
				if (restart) {
					udc_req = get_ep_head(ep);
					exynos_ss_udc_start_req(udc, ep,
								udc_req, false);
				}
			}

			break;

		default:
			return -ENOENT;
		}
	} else
		return -ENOENT;  /* currently only deal with endpoint */

	return 1;
}

/**
 * exynos_ss_udc_process_req_status - process request GET_STATUS
 * @udc: The device state
 * @ctrl: USB control request
 */
static int exynos_ss_udc_process_req_status(struct exynos_ss_udc *udc,
					struct usb_ctrlrequest *ctrl)
{
	struct exynos_ss_udc_ep *ep0 = &udc->eps[0];
	struct exynos_ss_udc_ep *ep;
	__le16 reply;
	int ret;

	dev_dbg(udc->dev, "%s: USB_REQ_GET_STATUS\n", __func__);

	if (!ep0->dir_in) {
		dev_warn(udc->dev, "%s: direction out?\n", __func__);
		return -EINVAL;
	}

	switch (ctrl->bRequestType & USB_RECIP_MASK) {
	case USB_RECIP_DEVICE:
		reply = cpu_to_le16(0); /* bit 0 => self powered,
					 * bit 1 => remote wakeup */
		break;

	case USB_RECIP_INTERFACE:
		/* currently, the data result should be zero */
		reply = cpu_to_le16(0);
		break;

	case USB_RECIP_ENDPOINT:
		ep = ep_from_windex(udc, le16_to_cpu(ctrl->wIndex));
		if (!ep)
			return -ENOENT;

		reply = cpu_to_le16(ep->halted ? 1 : 0);
		break;

	default:
		return 0;
	}

	if (le16_to_cpu(ctrl->wLength) != 2)
		return -EINVAL;

	ret = exynos_ss_udc_send_reply(udc, ep0, &reply, 2);
	if (ret) {
		dev_err(udc->dev, "%s: failed to send reply\n", __func__);
		return ret;
	}

	return 1;
}

/**
 * exynos_ss_udc_process_control - process a control request
 * @udc: The device state
 * @ctrl: The control request received
 *
 * The controller has received the SETUP phase of a control request, and
 * needs to work out what to do next (and whether to pass it on to the
 * gadget driver).
 */
static void exynos_ss_udc_process_control(struct exynos_ss_udc *udc,
				      struct usb_ctrlrequest *ctrl)
{
	struct exynos_ss_udc_ep *ep0 = &udc->eps[0];
	int ret = 0;
	u8 *buf = udc->ep0_buff;
	int len;

	udc->ep0_setup = false;

	ep0->sent_zlp = 0;

	dev_dbg(udc->dev, "ctrl Req=%02x, Type=%02x, V=%04x, L=%04x\n",
		 ctrl->bRequest, ctrl->bRequestType,
		 ctrl->wValue, ctrl->wLength);

	/* record the direction of the request, for later use when enquing
	 * packets onto EP0. */

	ep0->dir_in = (ctrl->bRequestType & USB_DIR_IN) ? 1 : 0;
	dev_dbg(udc->dev, "ctrl: dir_in=%d\n", ep0->dir_in);

	/* if we've no data with this request, then the last part of the
	 * transaction is going to implicitly be IN. */
	if (ctrl->wLength == 0)
		ep0->dir_in = 1;

	if ((ctrl->bRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest) {
		case USB_REQ_SET_ADDRESS:
			__bic32(udc->regs + EXYNOS_USB3_DCFG,
				EXYNOS_USB3_DCFG_DevAddr_MASK);
			__orr32(udc->regs + EXYNOS_USB3_DCFG,
				EXYNOS_USB3_DCFG_DevAddr(ctrl->wValue));

			dev_info(udc->dev, "new address %d\n", ctrl->wValue);

			ret = exynos_ss_udc_send_reply(udc, ep0, NULL, 0);
			return;

		case USB_REQ_GET_STATUS:
			ret = exynos_ss_udc_process_req_status(udc, ctrl);
			break;

		case USB_REQ_CLEAR_FEATURE:
		case USB_REQ_SET_FEATURE:
			ret = exynos_ss_udc_process_req_feature(udc, ctrl);
			break;

		case USB_REQ_GET_DESCRIPTOR:
			if ((ctrl->wValue >> 8) == 0x0f) {
				/* Get BOS descriptor */
				memcpy(buf, &bos, sizeof(bos));
				buf += sizeof(bos);
				memcpy(buf, &cap1, sizeof(cap1));
				buf += sizeof(cap1);
				memcpy(buf, &cap2, sizeof(cap2));
				buf += sizeof(cap2);
				memcpy(buf, &cap3, sizeof(cap3));
				len = bos.wTotalLength < ctrl->wLength ?
				      bos.wTotalLength : ctrl->wLength;

				ret = exynos_ss_udc_send_reply(udc, ep0,
					udc->ep0_buff, len);
			}
			break;
		}
	}

	/* as a fallback, try delivering it to the driver to deal with */

	if (ret == 0 && udc->driver) {
		ret = udc->driver->setup(&udc->gadget, ctrl);
		if (ret < 0)
			dev_dbg(udc->dev, "driver->setup() ret %d\n", ret);
	}

	/* the request is either unhandlable, or is not formatted correctly
	 * so respond with a STALL for the status stage to indicate failure.
	 */

	if (ret < 0) {
		struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
		bool res;

		dev_dbg(udc->dev, "ep0 stall (dir=%d)\n", ep0->dir_in);

		epcmd->ep = (ep0->dir_in) ? 1 : 0;
		epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPSSTALL;
		epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

		res = exynos_ss_udc_issue_cmd(udc, epcmd);
		if (!res)
			dev_err(udc->dev, "Failed to set/clear stall\n");
		
		exynos_ss_udc_enqueue_setup(udc);
	}
}

/**
 * exynos_ss_udc_complete_setup - completion of a setup transfer
 * @ep: The endpoint the request was on.
 * @req: The request completed.
 *
 * Called on completion of any requests the driver itself submitted for
 * EP0 setup packets
 */
static void exynos_ss_udc_complete_setup(struct usb_ep *ep,
				      struct usb_request *req)
{
	struct exynos_ss_udc_ep *udc_ep = our_ep(ep);
	struct exynos_ss_udc *udc = udc_ep->parent;

	if (req->status < 0) {
		dev_dbg(udc->dev, "%s: failed %d\n", __func__, req->status);
		return;
	}

	if (req->actual == 0)
		exynos_ss_udc_enqueue_setup(udc);
	else
		exynos_ss_udc_process_control(udc, req->buf);
}

static int exynos_ss_udc_ep_queue(struct usb_ep *ep, struct usb_request *req,
			      gfp_t gfp_flags)
{
	struct exynos_ss_udc_req *udc_req = our_req(req);
	struct exynos_ss_udc_ep *udc_ep = our_ep(ep);
	struct exynos_ss_udc *udc = udc_ep->parent;
	unsigned long irqflags;
	bool first;
	int ret;

	dev_dbg(udc->dev, "%s: req %p: %d@%p, noi=%d, zero=%d, snok=%d\n",
		ep->name, req, req->length, req->buf, req->no_interrupt,
		req->zero, req->short_not_ok);

	/* initialise status of the request */
	INIT_LIST_HEAD(&udc_req->queue);

	req->actual = 0;
	req->status = -EINPROGRESS;

	/* Sync the buffers as necessary */
	ret = exynos_ss_udc_map_dma(udc, udc_ep, req);
	if (ret)
		return ret;

	spin_lock_irqsave(&udc_ep->lock, irqflags);

	first = list_empty(&udc_ep->queue);
	list_add_tail(&udc_req->queue, &udc_ep->queue);

	if (first)
		exynos_ss_udc_start_req(udc, udc_ep, udc_req, false);

	spin_unlock_irqrestore(&udc_ep->lock, irqflags);

	return 0;
}

/**:;
 * exynos_ss_udc_enqueue_setup - start a request for EP0 packets
 * @udc: The device state.
 *s
 * Enqueue a request on EP0 if necessary to received any SETUP packets
 * received from the host.
 */
static void exynos_ss_udc_enqueue_setup(struct exynos_ss_udc *udc)
{
	struct usb_request *req = udc->ctrl_req;
	struct exynos_ss_udc_req *udc_req = our_req(req);
	int ret;

	dev_dbg(udc->dev, "%s: queueing setup request\n", __func__);

	udc->ep0_setup = true;

	req->zero = 0;
	req->length = 8;
	req->buf = &udc_req->trb;
	req->complete = exynos_ss_udc_complete_setup;

	if (!list_empty(&udc_req->queue)) {
		dev_dbg(udc->dev, "%s already queued???\n", __func__);
		return;
	}

	udc->eps[0].dir_in = 0;

	ret = exynos_ss_udc_ep_queue(&udc->eps[0].ep, req, GFP_ATOMIC);
	if (ret < 0) {
		dev_err(udc->dev, "%s: failed queue (%d)\n", __func__, ret);
		/* Don't think there's much we can do other than watch the
		 * driver fail. */
	}
}

/**
 * exynos_ss_udc_complete_request - complete a request given to us
 * @udc: The device state.
 * @udc_ep: The endpoint the request was on.
 * @udc_req: The request to complete.
 * @result: The result code (0 => Ok, otherwise errno)
 *
 * The given request has finished, so call the necessary completion
 * if it has one and then look to see if we can start a new request
 * on the endpoint.
 *
 * Note, expects the ep to already be locked as appropriate.
*/
static void exynos_ss_udc_complete_request(struct exynos_ss_udc *udc,
				       struct exynos_ss_udc_ep *udc_ep,
				       struct exynos_ss_udc_req *udc_req,
				       int result)
{
	bool restart;

	if (!udc_req) {
		dev_dbg(udc->dev, "%s: nothing to complete\n", __func__);
		return;
	}

	dev_dbg(udc->dev, "complete: ep %p %s, req %p, %d => %p\n",
		udc_ep, udc_ep->ep.name, udc_req, result, udc_req->req.complete);

	/* only replace the status if we've not already set an error
	 * from a previous transaction */

	if (udc_req->req.status == -EINPROGRESS)
		udc_req->req.status = result;

	udc_ep->req = NULL;
	list_del_init(&udc_req->queue);

	exynos_ss_udc_unmap_dma(udc, udc_ep, udc_req);

	/* call the complete request with the locks off, just in case the
	 * request tries to queue more work for this endpoint. */

	if (udc_req->req.complete) {
		spin_unlock(&udc_ep->lock);
		udc_req->req.complete(&udc_ep->ep, &udc_req->req);
		spin_lock(&udc_ep->lock);
	}

	/* Look to see if there is anything else to do. Note, the completion
	 * of the previous request may have caused a new request to be started
	 * so be careful when doing this. */

	if (!udc_ep->req && result >= 0) {
		restart = !list_empty(&udc_ep->queue);
		if (restart) {
			udc_req = get_ep_head(udc_ep);
			exynos_ss_udc_start_req(udc, udc_ep, udc_req, false);
		}
	}
}

/**
 * exynos_ss_udc_kill_all_requests - remove all requests from the endpoint's queue
 * @udc: The device state.
 * @ep: The endpoint the requests may be on.
 * @result: The result code to use.
 * @force: Force removal of any current requests
 *
 * Go through the requests on the given endpoint and mark them
 * completed with the given result code.
 */
static void exynos_ss_udc_kill_all_requests(struct exynos_ss_udc *udc,
			      struct exynos_ss_udc_ep *ep,
			      int result, bool force)
{
	struct exynos_ss_udc_req *req, *treq;
	unsigned long flags;

	spin_lock_irqsave(&ep->lock, flags);

	list_for_each_entry_safe(req, treq, &ep->queue, queue) {

		exynos_ss_udc_complete_request(udc, ep, req,
					   result);
	}

	spin_unlock_irqrestore(&ep->lock, flags);
}

/**
 * exynos_ss_udc_complete_request_lock - complete a request given to us (locked)
 * @udc: The device state.
 * @udc_ep: The endpoint the request was on.
 * @udc_req: The request to complete.
 * @result: The result code (0 => Ok, otherwise errno)
 *
 * See exynos_ss_udc_complete_request(), but called with the endpoint's
 * lock held.
*/
static void exynos_ss_udc_complete_request_lock(struct exynos_ss_udc *udc,
					    struct exynos_ss_udc_ep *udc_ep,
					    struct exynos_ss_udc_req *udc_req,
					    int result)
{
	unsigned long flags;

	spin_lock_irqsave(&udc_ep->lock, flags);
	exynos_ss_udc_complete_request(udc, udc_ep, udc_req, result);
	spin_unlock_irqrestore(&udc_ep->lock, flags);
}

/**
 * exynos_ss_udc_complete_in - complete IN transfer
 * @udc: The device state.
 * @udc_ep: The endpoint that has just completed.
 *
 * An IN transfer has been completed, update the transfer's state and then
 * call the relevant completion routines.
 */
static void exynos_ss_udc_complete_in(struct exynos_ss_udc *udc,
				  struct exynos_ss_udc_ep *udc_ep)
{
	struct exynos_ss_udc_req *udc_req = udc_ep->req;
	int size_left;

	if (!udc_req) {
		dev_dbg(udc->dev, "XferCompl but no req\n");
		return;
	}

	if (udc_req->trb.param2 & EXYNOS_USB3_TRB_HWO) {
		dev_dbg(udc->dev, "%s: HWO bit set!\n", __func__);
		return;
	}

	size_left = udc_req->trb.param1 & EXYNOS_USB3_TRB_BUFSIZ_MASK;
	udc_req->req.actual = udc_req->req.length - size_left;

	if (udc_req->req.actual < udc_req->req.length) {
		dev_dbg(udc->dev, "%s trying more for req...\n", __func__);
		exynos_ss_udc_start_req(udc, udc_ep, udc_req, true);
	} else
		exynos_ss_udc_complete_request_lock(udc, udc_ep, udc_req, 0);
}

/**
 * exynos_ss_udc_send_zlp - send zero-length packet on control endpoint
 * @udc: The device instance
 * @req: The request currently on this endpoint
 */
static void exynos_ss_udc_send_zlp(struct exynos_ss_udc *udc,
			       struct exynos_ss_udc_req *req)
{
	static struct exynos_ss_udc_trb zlp_trb;
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	bool res;

	if (!req) {
		dev_warn(udc->dev, "%s: no request?\n", __func__);
		return;
	}

	if (req->req.length == 0) {
		udc->eps[0].sent_zlp = 1;
		exynos_ss_udc_enqueue_setup(udc);
		return;
	}

	udc->eps[0].sent_zlp = 1;

	dev_dbg(udc->dev, "sending zero-length packet\n");

	/* issue a zero-sized packet */

	/* We don't care about buff address since transfer size is 0 */
	zlp_trb.param1 = EXYNOS_USB3_TRB_BUFSIZ(0);
	zlp_trb.param2 = EXYNOS_USB3_TRB_IOC | EXYNOS_USB3_TRB_LST |
					EXYNOS_USB3_TRB_HWO;

	/* Start Transfer */
	epcmd->ep = 1;
	epcmd->param0 = (u32) virt_to_phys(&zlp_trb);
	epcmd->param1 = 0;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPSTRTXFER;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res)
		dev_err(udc->dev, "Failed to start transfer\n");
}

/**
 * exynos_ss_udc_handle_outdone - handle receiving OutDone/SetupDone from RXFIFO
 * @udc: The device instance
 * @epnum: The endpoint received from
 * @was_setup: Set if processing a SetupDone event.
*/
static void exynos_ss_udc_handle_outdone(struct exynos_ss_udc *udc,
				     int epnum, bool was_setup)
{
	struct exynos_ss_udc_ep *udc_ep = &udc->eps[epnum];
	struct exynos_ss_udc_req *udc_req = udc_ep->req;
	struct usb_request *req = &udc_req->req;
	int size_left;
	int result = 0;


	if (!udc_req) {
		dev_dbg(udc->dev, "%s: no request active\n", __func__);
		return;
	}

	if (udc_req->trb.param2 & EXYNOS_USB3_TRB_HWO) {
		dev_dbg(udc->dev, "%s: HWO bit set!\n", __func__);
		return;
	}

	size_left = udc_req->trb.param1 & EXYNOS_USB3_TRB_BUFSIZ_MASK;
	udc_req->req.actual = udc_req->req.length - size_left;

	if (req->actual < req->length) {
		exynos_ss_udc_start_req(udc, udc_ep, udc_req, true);
		return;
	}

	if (epnum == 0) {
		if (!was_setup && req->complete != exynos_ss_udc_complete_setup)
			exynos_ss_udc_send_zlp(udc, udc_req);
	}

	exynos_ss_udc_complete_request_lock(udc, udc_ep, udc_req, result);
}

static void exynos_ss_udc_irq_connectdone(struct exynos_ss_udc *udc)
{
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	u32 reg, speed;
	int mps;
	bool res;

	dev_dbg(udc->dev, "%s: connect done\n", __func__);

	reg = readl(udc->regs + EXYNOS_USB3_DSTS);

	/* TODO:
	   1. program GCTL:RAMClkSel
	 */
	speed = reg & EXYNOS_USB3_DSTS_ConnectSpd_MASK;

	/* Suspend the inactive Phy */
	if (speed == USB_SPEED_SUPER)
		__orr32(udc->regs + EXYNOS_USB3_GUSB2PHYCFG(0),
			EXYNOS_USB3_GUSB2PHYCFGx_SusPHY);
	else
		__orr32(udc->regs + EXYNOS_USB3_GUSB3PIPECTL(0),
			EXYNOS_USB3_GUSB3PIPECTLx_SuspSSPhy);

	switch (speed) {
	/* High-speed */
	case 0:
		udc->gadget.speed = USB_SPEED_HIGH;
		mps = 64;
		break;
	/* Full-speed */
	case 1:
	case 3:
		udc->gadget.speed = USB_SPEED_FULL;
		mps = 64;
		break;
	/* Low-speed */
	case 2:
		udc->gadget.speed = USB_SPEED_LOW;
		mps = 8;
		break;
	/* SuperSpeed */
	case 4:
		udc->gadget.speed = USB_SPEED_SUPER;
		mps = 512;
		break;
	}

	epcmd->ep = 0;
	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_MPS(mps);
	epcmd->param1 = EXYNOS_USB3_DEPCMDPAR1x_XferNRdyEn |
			EXYNOS_USB3_DEPCMDPAR1x_XferCmplEn;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res)
		dev_err(udc->dev, "Failed to configure physical EP0\n");

	epcmd->ep = 1;
	epcmd->param1 = EXYNOS_USB3_DEPCMDPAR1x_EpDir |
			EXYNOS_USB3_DEPCMDPAR1x_XferNRdyEn |
			EXYNOS_USB3_DEPCMDPAR1x_XferCmplEn;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res)
		dev_err(udc->dev, "Failed to configure physical EP1\n");
}

static void exynos_ss_udc_irq_usbrst(struct exynos_ss_udc *udc)
{
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	bool res;
	int epnum;

	dev_info(udc->dev, "%s: USB reset\n", __func__);

	/* End transfer for EP0-OUT */
	epcmd->ep = 0;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPENDXFER;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res)
		dev_err(udc->dev, "Failed to end transfer\n");

	/* End transfer for EP0-IN */
	epcmd->ep = 1;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res)
		dev_err(udc->dev, "Failed to end transfer\n");

	exynos_ss_udc_kill_all_requests(udc,
				     &udc->eps[0],
				     -ECONNRESET, true);

	exynos_ss_udc_enqueue_setup(udc);

	/* Clear STALL if EP0 is halted */
	if (udc->eps[0].halted)
		exynos_ss_udc_ep_sethalt(&udc->eps[0].ep, 0);

	/* End transfer, kill all requests and clear STALL on the
	   rest of endpoints */
	for (epnum = 1; epnum < EXYNOS_USB3_EPS; epnum++) {
		int index = get_phys_epnum(&udc->eps[epnum]);

		epcmd->ep = index;

		res = exynos_ss_udc_issue_cmd(udc, epcmd);
		if (!res)
			dev_err(udc->dev, "Failed to end transfer\n");


		exynos_ss_udc_kill_all_requests(udc,
					     &udc->eps[epnum],
					     -ECONNRESET, true);

		if (udc->eps[epnum].halted)
			exynos_ss_udc_ep_sethalt(&udc->eps[epnum].ep, 0);
	}

	/* Set device address to 0 */
	__bic32(udc->regs + EXYNOS_USB3_DCFG, EXYNOS_USB3_DCFG_DevAddr_MASK);

	exynos_ss_udc_enqueue_setup(udc);
}

/**
 * exynos_ss_udc_handle_depevt - handle endpoint-specific event
 * @udc: The driver state
 * @event: event to handle
 *
*/
static void exynos_ss_udc_handle_depevt(struct exynos_ss_udc *udc, u32 event)
{
	int index = (event & 0xfe) >> 1;
	int dir_in = index & 1;
	int epnum = get_usb_epnum(index);
	/* We will need it in future */
	struct exynos_ss_udc_ep *udc_ep = &udc->eps[epnum];
	
	switch (event & EXYNOS_USB3_DEPEVT_EVENT_MASK) {
	case EXYNOS_USB3_DEPEVT_EVENT_XferNotReady:
		/* TODO: If an XferNotReady is received before the
		 * XferComplete we should issue Set Stall */
		break;

	case EXYNOS_USB3_DEPEVT_EVENT_XferComplete:

		if (dir_in) {
			/* Handle "transfer complete" for IN EPs */
			exynos_ss_udc_complete_in(udc, udc_ep);

			if (epnum == 0 && !udc_ep->req)
				exynos_ss_udc_enqueue_setup(udc);
		} else {
			/* Handle "transfer complete" for OUT EPs */

			/* TODO: We need to distinguish Setup packets from other
			 * OUT packets. We will use EP0 states for it. */

			if (epnum == 0 && udc->ep0_setup)
				exynos_ss_udc_handle_outdone(udc, epnum, true);
			else
				exynos_ss_udc_handle_outdone(udc, epnum, false);
		}

		break;
	}

}

/**
 * exynos_ss_udc_handle_devt - handle device-specific event
 * @udc: The driver state
 * @event: event to handle
 *
*/
static void exynos_ss_udc_handle_devt(struct exynos_ss_udc *udc, u32 event)
{

	switch (event & EXYNOS_USB3_DEVT_EVENT_MASK) {
	case EXYNOS_USB3_DEVT_EVENT_ULStChng:
		break;	

	case EXYNOS_USB3_DEVT_EVENT_ConnectDone:
		exynos_ss_udc_irq_connectdone(udc);
		break;

	case EXYNOS_USB3_DEVT_EVENT_USBRst:
		exynos_ss_udc_irq_usbrst(udc);
		break;

	default:
		break;
	}
}

static void exynos_ss_udc_handle_otgevt(struct exynos_ss_udc *udc, u32 event)
{}

static void exynos_ss_udc_handle_gevt(struct exynos_ss_udc *udc, u32 event)
{}

/**
 * exynos_ss_udc_irq - handle device interrupt
 * @irq: The IRQ number triggered
 * @pw: The pw value when registered the handler.
 */
static irqreturn_t exynos_ss_udc_irq(int irq, void *pw)
{
	struct exynos_ss_udc *udc = pw;
	static int indx;
	int gevntcount;
	u32 event;
	u32 ecode1, ecode2;

	gevntcount = readl(udc->regs + EXYNOS_USB3_GEVNTCOUNT(0)) &
			EXYNOS_USB3_GEVNTCOUNTx_EVNTCount_MASK;
	/* TODO: what if number of events more then buffer size? */

	while (gevntcount--) {
		event = udc->event_buff[indx++];

		ecode1 = event & 0x01;

		if (ecode1 == 0)
			/* Device Endpoint-Specific Event */
			exynos_ss_udc_handle_depevt(udc, event);
		else {
			ecode2 = (event & 0xfe) >> 1;

			switch (ecode2) {
			/* Device-Specific Event */
			case 0x00:
				exynos_ss_udc_handle_devt(udc, event);
			break;

			/* OTG Event */
			case 0x01:
				exynos_ss_udc_handle_otgevt(udc, event);
			break;

			/* Other Core Event */
			case 0x03:
			case 0x04:
				exynos_ss_udc_handle_gevt(udc, event);
			break;

			/* Unknown Event Type */
			default:
				dev_info(udc->dev, "Unknown event type\n");
			break;
			}
		}
		/* We processed 1 event (4 bytes) */
		writel(4, udc->regs + EXYNOS_USB3_GEVNTCOUNT(0));

		if (indx > (EXYNOS_USB3_EVENT_BUFF_WSIZE - 1))
			indx = 0;
	}

	/* Do we need to read GEVENTCOUNT here and retry? */

	return IRQ_HANDLED;
}

/**
 * exynos_ss_udc_initep - initialise a single endpoint
 * @udc: The device state.
 * @udc_ep: The endpoint to be initialised.
 * @epnum: The endpoint number
 *
 * Initialise the given endpoint (as part of the probe and device state
 * creation) to give to the gadget driver. Setup the endpoint name, any
 * direction information and other state that may be required.
 */
static void __devinit exynos_ss_udc_initep(struct exynos_ss_udc *udc,
				       struct exynos_ss_udc_ep *udc_ep,
				       int epnum)
{
	char *dir;

	if (epnum == 0)
		dir = "";
	else if ((epnum % 2) == 0) {
		dir = "out";
	} else {
		dir = "in";
		udc_ep->dir_in = 1;
	}

	udc_ep->epnum = epnum;

	snprintf(udc_ep->name, sizeof(udc_ep->name), "ep%d%s", epnum, dir);

	INIT_LIST_HEAD(&udc_ep->queue);
	INIT_LIST_HEAD(&udc_ep->ep.ep_list);

	spin_lock_init(&udc_ep->lock);

	/* add to the list of endpoints known by the gadget driver */
	if (epnum)
		list_add_tail(&udc_ep->ep.ep_list, &udc->gadget.ep_list);

	udc_ep->parent = udc;
	udc_ep->ep.name = udc_ep->name;
	udc_ep->ep.maxpacket = epnum ? 1024 : EP0_MPS_LIMIT;
	udc_ep->ep.ops = &exynos_ss_udc_ep_ops;

}

/**
 * exynos_ss_udc_gate - set the hardware gate for the block
 * @pdev: The device we bound to
 * @on: On or off.
 *
 * Set the hardware gate setting into the block.
 */
static void exynos_ss_udc_gate(struct platform_device *pdev, bool on)
{
}

/**
 * exynos_ss_udc_phyreset - reset the EXYNOS phy block
 * @udc: The host state.
 *
 * Power up the phy, set the basic configuration and start the PHY.
 */
static void exynos_ss_udc_phyreset(struct exynos_ss_udc *udc)
{
}


/**
 * exynos_ss_udc_corereset - issue softreset to the core
 * @udc: The device state
 *
 * Issue a soft reset to the core, and await the core finishing it.
*/
static int exynos_ss_udc_corereset(struct exynos_ss_udc *udc)
{
	bool res;

	/* issue soft reset */
	__orr32(udc->regs + EXYNOS_USB3_DCTL, EXYNOS_USB3_DCTL_CSftRst);

	res = exynos_ss_udc_poll_bit_clear(udc->regs + EXYNOS_USB3_DCTL,
					EXYNOS_USB3_DCTL_CSftRst,
					1000);
	if (!res) {
		dev_err(udc->dev, "Failed to get CSftRst asserted\n");
		return -EINVAL;
	}

	return 0;
}

static int exynos_ss_udc_init(struct exynos_ss_udc *udc)
{
	struct exynos_ss_udc_ep_command *epcmd = &udc->epcmd;
	u32 reg;
	bool res;

	/* TODO: GSBUSCFG0/1 - are power-on values correct
	   in coreConsultant? */

	/* TODO: GTXTHRCFG/GRXTHRCFG - are power-on values correct
	   in coreConsultant? */

	reg = readl(udc->regs + EXYNOS_USB3_GSNPSID);
	dev_info(udc->dev, "Core ID Number: 0x%04x\n", reg >> 16);
	dev_info(udc->dev, "Release Number: %d\n", reg & 0xffff);
	/* TODO: configure the driver for any version-specific features */

	/* TODO: GUID - is this register selected for implementation
	   in coreConsultant? */

	/* TODO: GUSB2PHYCHG - are power-on values correct
	   in coreConsultant? */

	/* TODO: GUSB3PIPECTL - are power-on values correct
	   in coreConsultatnt? */

	/* TODO: GTXFIFOSIZn/GRXFIFOSIZ0 - will use default values? */

	/* Event buffer */
	writel(0, udc->regs + EXYNOS_USB3_GEVNTADR_63_32(0));
	writel(udc->event_buff_dma, udc->regs + EXYNOS_USB3_GEVNTADR_31_0(0));
	/* FIXME: do we need to set Event Interrupt Mask now? */
	writel(EXYNOS_USB3_EVENT_BUFF_BSIZE, udc->regs + EXYNOS_USB3_GEVNTSIZ(0));
	writel(0, udc->regs + EXYNOS_USB3_GEVNTCOUNT(0));

	/* TODO: GCTL - will use default values? */

	/* DCFG - will use default value (3'b100) for Device Speed */

	/* Enable events */
	writel(EXYNOS_USB3_DEVTEN_ULStCngEn | EXYNOS_USB3_DEVTEN_ConnectDoneEn |
		EXYNOS_USB3_DEVTEN_USBRstEn, udc->regs + EXYNOS_USB3_DEVTEN);

	/* (!!!) Now we will issue the first command. How about write
	 * separate functions for each command? */

	/* Start New Configuration */
	epcmd->ep = 0;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPSTARTCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev, "Failed to start new configuration\n");
		return -EINVAL;
	}

	/* Configure Physical Endpoint 0 */
	epcmd->ep = 0;
	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_MPS(0x200);
	epcmd->param1 = EXYNOS_USB3_DEPCMDPAR1x_XferNRdyEn |
			EXYNOS_USB3_DEPCMDPAR1x_XferCmplEn;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev, "Failed to configure physical EP0\n");
		return -EINVAL;
	}
	
	/* Configure Physical Endpoint 1 */
	epcmd->ep = 1;
	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_MPS(0x200);
	epcmd->param1 = EXYNOS_USB3_DEPCMDPAR1x_EpDir |
			EXYNOS_USB3_DEPCMDPAR1x_XferNRdyEn |
			EXYNOS_USB3_DEPCMDPAR1x_XferCmplEn;
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev, "Failed to configure physical EP1\n");
		return -EINVAL;
	}

	/* Configure Pysical Endpoint 0 Transfer Resource */
	epcmd->ep = 0;
	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_NumXferRes(1);
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPXFERCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev,
			"Failed to configure physical EP0 transfer resource\n");
		return -EINVAL;
	}

	/* Configure Pysical Endpoint 1 Transfer Resource */
	epcmd->ep = 1;
	epcmd->param0 = EXYNOS_USB3_DEPCMDPAR0x_NumXferRes(1);
	epcmd->cmdtyp = EXYNOS_USB3_DEPCMDx_CmdTyp_DEPXFERCFG;
	epcmd->cmdflags = EXYNOS_USB3_DEPCMDx_CmdAct;

	res = exynos_ss_udc_issue_cmd(udc, epcmd);
	if (!res) {
		dev_err(udc->dev,
			"Failed to configure physical EP1 transfer resource\n");
		return -EINVAL;
	}

	/* According to documentation we need here to start transfer on
	   Phys EP 0 and 1, but we will do it after USB Reset */

	/* Enable Physical Endpoints 0 and 1 */
	writel(3, udc->regs + EXYNOS_USB3_DALEPENA);

	/* Start the device controller operation */
	__orr32(udc->regs + EXYNOS_USB3_DCTL, EXYNOS_USB3_DCTL_Run_Stop);

	return 0;
}

int usb_gadget_probe_driver(struct usb_gadget_driver *driver,
		int (*bind)(struct usb_gadget *))
{
	struct exynos_ss_udc *udc = our_udc;
	int ret;

	if (!udc) {
		printk(KERN_ERR "%s: called with no device\n", __func__);
		return -ENODEV;
	}

	if (!driver) {
		dev_err(udc->dev, "%s: no driver\n", __func__);
		return -EINVAL;
	}

	if (driver->speed != USB_SPEED_SUPER &&
	    driver->speed != USB_SPEED_HIGH &&
	    driver->speed != USB_SPEED_FULL) {
		dev_err(udc->dev, "%s: bad speed\n", __func__);
	}

	if (!bind || !driver->setup) {
		dev_err(udc->dev, "%s: missing entry points\n", __func__);
		return -EINVAL;
	}

	WARN_ON(udc->driver);

	driver->driver.bus = NULL;
	udc->driver = driver;
	udc->gadget.dev.driver = &driver->driver;
	udc->gadget.dev.dma_mask = udc->dev->dma_mask;
	udc->gadget.speed = USB_SPEED_UNKNOWN;

	ret = device_add(&udc->gadget.dev);
	if (ret) {
		dev_err(udc->dev, "failed to register gadget device\n");
		goto err;
	}

	ret = bind(&udc->gadget);
	if (ret) {
		dev_err(udc->dev, "failed bind %s\n", driver->driver.name);

		udc->gadget.dev.driver = NULL;
		udc->driver = NULL;
		goto err;
	}

	/* we must now enable ep0 ready for host detection and then
	 * set configuration. */

	exynos_ss_udc_corereset(udc);

	exynos_ss_udc_init(udc);

	/* report to the user, and return */

	dev_info(udc->dev, "bound driver %s\n", driver->driver.name);
	return 0;

err:
	udc->driver = NULL;
	udc->gadget.dev.driver = NULL;
	return ret;
}
EXPORT_SYMBOL(usb_gadget_probe_driver);

int usb_gadget_unregister_driver(struct usb_gadget_driver *driver)
{
	struct exynos_ss_udc *udc = our_udc;
	int ep;

	if (!udc)
		return -ENODEV;

	if (!driver || driver != udc->driver || !driver->unbind)
		return -EINVAL;

	/* all endpoints should be shutdown */
	for (ep = 0; ep < EXYNOS_USB3_EPS; ep++)
		exynos_ss_udc_ep_disable(&udc->eps[ep].ep);

	call_gadget(udc, disconnect);

	driver->unbind(&udc->gadget);
	udc->driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;

	device_del(&udc->gadget.dev);

	dev_info(udc->dev, "unregistered gadget driver '%s'\n",
		 driver->driver.name);

	return 0;
}
EXPORT_SYMBOL(usb_gadget_unregister_driver);

/* TODO: define platform data structure */
static struct exynos_ss_udc_plat exynos_ss_udc_default_pdata;

static int __devinit exynos_ss_udc_probe(struct platform_device *pdev)
{
	struct exynos_ss_udc_plat *plat = pdev->dev.platform_data;
	struct device *dev = &pdev->dev;
	struct exynos_ss_udc *udc;
	struct resource *res;
	int epnum;
	int ret;

	if (!plat)
		plat = &exynos_ss_udc_default_pdata;

	udc = kzalloc(sizeof(struct exynos_ss_udc) +
			sizeof(struct exynos_ss_udc_ep) * EXYNOS_USB3_EPS,
			GFP_KERNEL);
	if (!udc) {
		dev_err(dev, "cannot get memory\n");
		return -ENOMEM;
	}

	udc->ep0_buff = kzalloc(512, GFP_KERNEL);
	if (!udc->ep0_buff) {
		dev_err(dev, "cannot get memory for EP0 buffer\n");
		ret = -ENOMEM;
		goto err_mem;
	}

	udc->dev = dev;
	udc->plat = plat;

	/* FIXME: clock name and error message */	
	udc->clk = clk_get(&pdev->dev, "ssexynos");
	if (IS_ERR(udc->clk)) {
		dev_err(dev, "cannot get UDC clock\n");
		ret = -EINVAL;
		goto err_mem;
	}

	platform_set_drvdata(pdev, udc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "cannot find register resource 0\n");
		ret = -EINVAL;
		goto err_clk;
	}
	
	udc->regs_res = request_mem_region(res->start, resource_size(res),
					     dev_name(dev));
	if (udc->regs_res) {
		dev_err(dev, "cannot reserve registers\n");
		ret = -ENOENT;
		goto err_clk;
	}

	udc->regs = ioremap(res->start, resource_size(res));
	if (!udc->regs) {
		dev_err(dev, "cannot map registers\n");
		ret = -ENXIO;
		goto err_regs_res;
	}

	/* According to documentation GEVNTADR register holds the
	   "Event Buffer DMA Address" */
	udc->event_buff_dma = dma_map_single(udc->dev,
					     udc->event_buff,
					     sizeof udc->event_buff,
					     DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(udc->dev, udc->event_buff_dma))) {
		dev_err(dev, "cannot map event buffer\n");
		ret = -EIO;
		goto err_regs;
	}

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(dev, "cannot find irq\n");
		goto err_regs;
	}
	
	udc->irq = ret;

	ret = request_irq(ret, exynos_ss_udc_irq, 0, dev_name(dev), udc);
	if (ret < 0) {
		dev_err(dev, "cannot claim IRQ\n");
		goto err_regs;
	}

	dev_info(dev, "regs %p, irq %d\n", udc->regs, udc->irq);
	/* FIXME */
	device_initialize(&udc->gadget.dev);

	dev_set_name(&udc->gadget.dev, "gadget");

	udc->gadget.is_dualspeed = 1; /* FIXME */
	udc->gadget.ops = &exynos_ss_udc_gadget_ops;
	udc->gadget.name = dev_name(dev);

	udc->gadget.dev.parent = dev;
	udc->gadget.dev.dma_mask = dev->dma_mask;
	
	/* setup endpoint information */

	INIT_LIST_HEAD(&udc->gadget.ep_list);
	udc->gadget.ep0 = &udc->eps[0].ep;

	/* allocate EP0 request */

	udc->ctrl_req = exynos_ss_udc_ep_alloc_request(&udc->eps[0].ep,
						     GFP_KERNEL);
	if (!udc->ctrl_req) {
		dev_err(dev, "failed to allocate ctrl req\n");
		goto err_regs;
	}

	/* reset the system */

	clk_enable(udc->clk);
	/* Need SoC datasheet */
	exynos_ss_udc_gate(pdev, true);
	/* Need SoC datasheet */
	exynos_ss_udc_phyreset(udc);
	exynos_ss_udc_corereset(udc);
	

	/* initialise the endpoints now the core has been initialised */
	for (epnum = 0; epnum < EXYNOS_USB3_EPS; epnum++)
		exynos_ss_udc_initep(udc, &udc->eps[epnum], epnum);

	our_udc = udc;
	return 0;

err_regs:
	iounmap(udc->regs);

err_regs_res:
	release_resource(udc->regs_res);
	kfree(udc->regs_res);
err_clk:
	clk_put(udc->clk);
err_mem:
	kfree(udc->ep0_buff);
	kfree(udc);
	return ret;

}

static int __devexit exynos_ss_udc_remove(struct platform_device *pdev)
{
	struct exynos_ss_udc *udc = platform_get_drvdata(pdev);

	usb_gadget_unregister_driver(udc->driver);

	free_irq(udc->irq, udc);
	iounmap(udc->regs);

	release_resource(udc->regs_res);
	kfree(udc->regs_res);

	exynos_ss_udc_gate(pdev, false);

	clk_disable(udc->clk);
	clk_put(udc->clk);

	kfree(udc);

	return 0;
}

#define exynos_ss_udc_suspend NULL
#define exynos_ss_udc_resume NULL

static struct platform_driver exynos_ss_udc_driver = {
	.driver		= {
		.name	= "exynos-ss-udc",
		.owner	= THIS_MODULE,
	},
	.probe		= exynos_ss_udc_probe,
	.remove		= __devexit_p(exynos_ss_udc_remove),
	.suspend	= exynos_ss_udc_suspend,
	.resume		= exynos_ss_udc_resume,
};

static int __init exynos_ss_udc_modinit(void)
{
	return platform_driver_register(&exynos_ss_udc_driver);
}

static void __exit exynos_ss_udc_modexit(void)
{
	platform_driver_unregister(&exynos_ss_udc_driver);
}

module_init(exynos_ss_udc_modinit);
module_exit(exynos_ss_udc_modexit);

MODULE_DESCRIPTION("EXYNOS SuperSpeed USB 3.0 Device Controller");
MODULE_AUTHOR("Anton Tikhomirov <av.tikhomirov@samsung.com>");
MODULE_LICENSE("GPL");
