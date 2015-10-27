/*
 * Copyright 2015 www.starterkit.ru <info@starterkit.ru>
 *
 * Based on:
 * Driver for Intersil|Techwell TW6869 based DVR cards
 * (c) 2011-12 liran <jli11@intersil.com> [Intersil|Techwell China]
 *
 * V4L2 PCI Skeleton Driver
 * Copyright 2014 Cisco Systems, Inc. and/or its affiliates.
 * All rights reserved.
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/videodev2.h>

#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/control.h>

#include "tw6869.h"

MODULE_DESCRIPTION("tw6869/65 media bridge driver");
MODULE_AUTHOR("starterkit <info@starterkit.ru>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.3.2");

static const struct pci_device_id tw6869_pci_tbl[] = {
	{PCI_DEVICE(PCI_VENDOR_ID_TECHWELL, PCI_DEVICE_ID_6869)},
	{ 0, }
};

struct tw6869_buf {
	struct vb2_buffer vb;
	struct list_head list;
	dma_addr_t dma;
};

/**
 * struct tw6869_vch - instance of one video channel
 * @dev: parent device
 * @vdev: video channel (v4l2 video device)
 * @id: DMA id (0...7)
 * @p_buf: DMA P-buffer
 * @b_buf: DMA B-buffer
 * @pb: P-buffer | B-buffer ping-pong state
 * @mlock: the main serialization lock
 * @queue: queue maintained by videobuf2 layer
 * @buf_list: list of buffer in use
 * @lock: spinlock controlling access to channel
 * @hdl: handler for control framework
 * @format: pixel format
 * @std: video standard (e.g. PAL/NTSC)
 * @input: input line for video signal
 * @sequence: sequence number of acquired buffer
 * @dcount: number of dropped frames
 * @fps: current frame rate
 */
struct tw6869_vch {
	struct tw6869_dev *dev;
	struct video_device vdev;

	unsigned int id;
	struct tw6869_buf *p_buf;
	struct tw6869_buf *b_buf;
	unsigned int pb;

	struct mutex mlock;
	struct vb2_queue queue;
	struct list_head buf_list;
	spinlock_t lock;

	struct v4l2_ctrl_handler hdl;
	struct v4l2_pix_format format;
	v4l2_std_id std;
	unsigned int input;

	unsigned int sequence;
	unsigned int dcount;
	unsigned int fps;
};

/**
 * struct tw6869_ach - instance of one audio channel
 * @dev: parent device
 * @ss: audio channel (pcm substream)
 * @id: DMA id (8...15)
 * @p_buf: DMA P-buffer
 * @b_buf: DMA B-buffer
 * @pb: P-buffer | B-buffer ping-pong state
 * @buf: split an contiguous buffer into chunks
 * @buf_list: chunk list
 * @lock: spinlock controlling access to channel
 * @ptr: PCM buffer pointer
 */
struct tw6869_ach {
	struct tw6869_dev *dev;
	struct snd_pcm_substream *ss;

	unsigned int id;
	struct tw6869_buf *p_buf;
	struct tw6869_buf *b_buf;
	unsigned int pb;

	struct tw6869_buf buf[TW_APAGE_MAX];
	struct list_head buf_list;
	spinlock_t lock;

	dma_addr_t ptr;
};

/**
 * struct tw6869_dev - instance of device
 * @pdev: PCI device
 * @mmio: hardware base address
 * @rlock: spinlock controlling access to registers
 * @ch_max: channels used
 * @id_err: DMA error counters
 * @v4l2_dev: device registered in v4l2 layer
 * @alloc_ctx: context for videobuf2
 * @vch: array of video channel instance
 * @snd_card: device registered in ALSA layer
 * @ach: array of audio channel instance
 */
struct tw6869_dev {
	struct pci_dev *pdev;
	unsigned char __iomem *mmio;
	spinlock_t rlock;

	unsigned int ch_max;
	unsigned int id_err[2 * TW_CH_MAX];

	struct v4l2_device v4l2_dev;
	struct vb2_alloc_ctx *alloc_ctx;
	struct tw6869_vch vch[TW_CH_MAX];

	struct snd_card *snd_card;
	struct tw6869_ach ach[TW_CH_MAX];
};

/**********************************************************************/

static inline void tw_write(struct tw6869_dev *dev, unsigned int reg,
				unsigned int val)
{
	iowrite32(val, dev->mmio + reg);
}

static inline unsigned int tw_read(struct tw6869_dev *dev,
				unsigned int reg)
{
	return ioread32(dev->mmio + reg);
}

static inline void tw_write_mask(struct tw6869_dev *dev,
				unsigned int reg, unsigned int val, unsigned int mask)
{
	unsigned int v = tw_read(dev, reg);

	v = (v & ~mask) | (val & mask);
	tw_write(dev, reg, v);
}

static inline void tw_clear(struct tw6869_dev *dev,
				unsigned int reg, unsigned int val)
{
	tw_write_mask(dev, reg, 0, val);
}

static inline void tw_set(struct tw6869_dev *dev,
				unsigned int reg, unsigned int val)
{
	tw_write_mask(dev, reg, val, val);
}

static inline unsigned int tw_id_is_on(struct tw6869_dev *dev,
				unsigned int id)
{
	unsigned int e = tw_read(dev, R32_DMA_CHANNEL_ENABLE);
	unsigned int c = tw_read(dev, R32_DMA_CMD);

	return (c & BIT(31)) && (c & e & BIT_ID(id));
}

static inline unsigned int tw_id_is_off(struct tw6869_dev *dev,
				unsigned int id)
{
	unsigned int e = tw_read(dev, R32_DMA_CHANNEL_ENABLE);
	unsigned int c = tw_read(dev, R32_DMA_CMD);

	return !((c | e) & BIT_ID(id));
}

static inline void tw_id_on(struct tw6869_dev *dev, unsigned int id)
{
	int c = 3;

	while (!tw_id_is_on(dev, id) && c--) {
		tw_set(dev, R32_DMA_CMD, BIT(31) | BIT_ID(id));
		tw_set(dev, R32_DMA_CHANNEL_ENABLE, BIT_ID(id));
	}
}

static inline void tw_id_off(struct tw6869_dev *dev, unsigned int id)
{
	int c = 3;

	while (!tw_id_is_off(dev, id) && c--) {
		tw_clear(dev, R32_DMA_CMD, BIT_ID(id));
		tw_clear(dev, R32_DMA_CHANNEL_ENABLE, BIT_ID(id));
	}

	if (!tw_read(dev, R32_DMA_CHANNEL_ENABLE))
		tw_write(dev, R32_DMA_CMD, 0);
}

static void tw6869_id_dma_cmd(struct tw6869_dev *dev,
				unsigned int id,
				unsigned int cmd)
{
	switch (cmd) {
	case TW_DMA_ON:
		dev->id_err[ID2ID(id)] = 0;
		tw_id_on(dev, id);
		dev_info(&dev->pdev->dev, "DMA %u ON\n", id);
		break;
	case TW_DMA_OFF:
		tw_id_off(dev, id);
		dev_info(&dev->pdev->dev, "DMA %u OFF\n", id);
		break;
	case TW_DMA_RST:
		if (tw_id_is_on(dev, id)) {
			tw_id_off(dev, id);
			if (++dev->id_err[ID2ID(id)] > TW_DMA_ERR_MAX) {
				dev_err(&dev->pdev->dev, "DMA %u forced OFF\n", id);
				break;
			}
			tw_id_on(dev, id);
			dev_info(&dev->pdev->dev, "DMA %u RST\n", id);
		} else {
			dev_info(&dev->pdev->dev, "DMA %u spurious RST\n", id);
		}
		break;
	default:
		dev_err(&dev->pdev->dev, "DMA %u unknown cmd %u\n", id, cmd);
	}
}

static unsigned int tw6869_virq(struct tw6869_dev *dev,
				unsigned int id,
				unsigned int pb,
				unsigned int err)
{
	struct tw6869_vch *vch = &dev->vch[ID2CH(id)];
	struct tw6869_buf *done = NULL;
	struct tw6869_buf *next = NULL;

	spin_lock(&vch->lock);
	if (!vb2_is_streaming(&vch->queue) || !vch->p_buf || !vch->b_buf) {
		spin_unlock(&vch->lock);
		return TW_DMA_OFF;
	}

	if (err || (vch->pb != pb)) {
		vch->pb = 0;
		spin_unlock(&vch->lock);
		return TW_DMA_RST;
	}

	if (!list_empty(&vch->buf_list)) {
		next = list_first_entry(&vch->buf_list, struct tw6869_buf, list);
		list_del(&next->list);
		if (pb) {
			done = vch->b_buf;
			vch->b_buf = next;
		} else {
			done = vch->p_buf;
			vch->p_buf = next;
		}
	}
	vch->pb = !pb;
	spin_unlock(&vch->lock);

	if (done && next) {
		tw_write(dev, pb ? R32_DMA_B_ADDR(id) : R32_DMA_P_ADDR(id), next->dma);
		v4l2_get_timestamp(&done->vb.v4l2_buf.timestamp);
		done->vb.v4l2_buf.sequence = vch->sequence++;
		vb2_buffer_done(&done->vb, VB2_BUF_STATE_DONE);
	} else {
		dev_info(&dev->pdev->dev, "vch%u NOBUF seq=%u dcount=%u\n",
			ID2CH(id), vch->sequence, ++vch->dcount);
	}
	return 0;
}

static unsigned int tw6869_airq(struct tw6869_dev *dev,
				unsigned int id,
				unsigned int pb)
{
	struct tw6869_ach *ach = &dev->ach[ID2CH(id)];
	struct tw6869_buf *done = NULL;
	struct tw6869_buf *next = NULL;

	spin_lock(&ach->lock);
	if (!ach->ss || !ach->p_buf || !ach->b_buf) {
		spin_unlock(&ach->lock);
		return TW_DMA_OFF;
	}

	if (ach->pb != pb) {
		ach->pb = 0;
		spin_unlock(&ach->lock);
		return TW_DMA_RST;
	}

	if (!list_empty(&ach->buf_list)) {
		next = list_first_entry(&ach->buf_list, struct tw6869_buf, list);
		list_move_tail(&next->list, &ach->buf_list);
		if (pb) {
			done = ach->p_buf;
			ach->b_buf = next;
		} else {
			done = ach->b_buf;
			ach->p_buf = next;
		}
	}
	ach->pb = !pb;
	spin_unlock(&ach->lock);

	if (done && next) {
		tw_write(dev, pb ? R32_DMA_B_ADDR(id) : R32_DMA_P_ADDR(id), next->dma);
		ach->ptr = done->dma - ach->buf[0].dma;
		snd_pcm_period_elapsed(ach->ss);
	} else {
		return TW_DMA_OFF;
	}
	return 0;
}

static irqreturn_t tw6869_irq(int irq, void *dev_id)
{
	struct tw6869_dev *dev = dev_id;
	unsigned int int_sts, fifo_sts, pb_sts, dma_en, id;

	int_sts = tw_read(dev, R32_INT_STATUS);
	fifo_sts = tw_read(dev, R32_FIFO_STATUS);
	pb_sts = tw_read(dev, R32_PB_STATUS);
	dma_en = tw_read(dev, R32_DMA_CHANNEL_ENABLE);
	dma_en &= tw_read(dev, R32_DMA_CMD);

	for (id = 0; id < (2 * TW_CH_MAX); id++) {
		unsigned int verr = fifo_sts & TW_VERR(id);

		if ((dma_en & BIT(id)) && ((int_sts & BIT(id)) || verr)) {
			unsigned int pb = !!(pb_sts & BIT(id));
			unsigned int cmd = (BIT(id) & TW_VID) ?
				tw6869_virq(dev, id, pb, verr) :
				tw6869_airq(dev, id, pb);

			if (cmd) {
				spin_lock(&dev->rlock);
				tw6869_id_dma_cmd(dev, id, cmd);
				spin_unlock(&dev->rlock);
			} else {
				dev->id_err[id] = 0;
			}
		}
	}
	return IRQ_HANDLED;
}

/**********************************************************************/

static inline struct tw6869_buf *to_tw6869_buf(struct vb2_buffer *vb2)
{
	return container_of(vb2, struct tw6869_buf, vb);
}

static int to_tw6869_pixformat(unsigned int pixelformat)
{
	int ret;

	switch (pixelformat) {
	case V4L2_PIX_FMT_YUYV:
		ret = TW_FMT_YUYV;
		break;
	case V4L2_PIX_FMT_UYVY:
		ret = TW_FMT_UYVY;
		break;
	case V4L2_PIX_FMT_RGB565:
		ret = TW_FMT_RGB565;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static unsigned int tw6869_fields_map(v4l2_std_id std, unsigned int rate)
{
	unsigned int map[15] = {
		0x00000000, 0x00000001, 0x00004001, 0x00104001, 0x00404041,
		0x01041041, 0x01104411, 0x01111111, 0x04444445, 0x04511445,
		0x05145145, 0x05151515, 0x05515455, 0x05551555, 0x05555555
	};
	unsigned int std_625_50[26] = {
		0, 1, 1, 2,  3,  3,  4,  4,  5,  5,  6,  7,  7,
		   8, 8, 9, 10, 10, 11, 11, 12, 13, 13, 14, 14, 0
	};
	unsigned int std_525_60[31] = {
		0, 1, 1, 1, 2,  2,  3,  3,  4,  4,  5,  5,  6,  6, 7, 7,
		   8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 0, 0
	};
	unsigned int i = (std & V4L2_STD_625_50) ?
			std_625_50[(rate > 25) ? 25 : rate] :
			std_525_60[(rate > 30) ? 30 : rate];

	return map[i];
}

static void tw6869_fill_pix_format(struct tw6869_vch *vch,
				struct v4l2_pix_format *pix)
{
	pix->width = 720;
	pix->height = (vch->std & V4L2_STD_625_50) ? 576 : 480;
	pix->field = V4L2_FIELD_INTERLACED_BT;
	pix->colorspace = V4L2_COLORSPACE_SMPTE170M;
	pix->bytesperline = pix->width * 2;
	pix->sizeimage = pix->bytesperline * pix->height;
	pix->priv = 0;
}

static void tw6869_vch_hw_cfg(struct tw6869_vch *vch)
{
	struct tw6869_dev *dev = vch->dev;
	struct v4l2_pix_format *pix = &vch->format;
	unsigned int cfg;

	if (vch->std & V4L2_STD_625_50)
		tw_set(dev, R32_VIDEO_CONTROL1, BIT_CH(vch->id) << 13);
	else
		tw_clear(dev, R32_VIDEO_CONTROL1, BIT_CH(vch->id) << 13);

	cfg = tw6869_fields_map(vch->std, vch->fps) << 1;
	cfg |= cfg << 1;
	if (cfg > 0)
		cfg |= BIT(31);
	tw_write(dev, R32_VIDEO_FIELD_CTRL(vch->id), cfg);

	/* Analog mux input0, no drop, enable master */
	cfg = ((vch->input & 0x0) << 30) |
		BIT(27) | (ID2SC(vch->id) << 25) |
		((to_tw6869_pixformat(pix->pixelformat) & 0x7) << 20);
	tw_write(dev, R32_DMA_CHANNEL_CONFIG(vch->id), cfg);

	cfg = (((pix->height >> 1) & 0x3FF) << 22) |
		((pix->bytesperline & 0x7FF) << 11) |
		(pix->bytesperline & 0x7FF);
	tw_write(dev, R32_DMA_WHP(vch->id), cfg);

	tw_write(dev, R32_DMA_P_ADDR(vch->id), vch->p_buf->dma);
	tw_write(dev, R32_DMA_B_ADDR(vch->id), vch->b_buf->dma);
}

/*
 * Videobuf2 Operations
 */
static int queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
				unsigned int *nbuffers, unsigned int *nplanes,
				unsigned int sizes[], void *alloc_ctxs[])
{
	struct tw6869_vch *vch = vb2_get_drv_priv(vq);
	struct tw6869_dev *dev = vch->dev;

	if (vq->num_buffers + *nbuffers < TW_FRAME_MAX)
		*nbuffers = TW_FRAME_MAX - vq->num_buffers;

	if (fmt && fmt->fmt.pix.sizeimage < vch->format.sizeimage)
		return -EINVAL;

	*nplanes = 1;
	sizes[0] = fmt ? fmt->fmt.pix.sizeimage : vch->format.sizeimage;
	alloc_ctxs[0] = dev->alloc_ctx;
	return 0;
}

static int buffer_init(struct vb2_buffer *vb)
{
	struct tw6869_buf *buf = to_tw6869_buf(vb);

	buf->dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	INIT_LIST_HEAD(&buf->list);
	{	/* pass the buffer physical address */
		unsigned int *cpu = vb2_plane_vaddr(vb, 0);
		*cpu = buf->dma;
	}
	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct tw6869_vch *vch = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = vch->format.sizeimage;

	if (vb2_plane_size(vb, 0) < size) {
		v4l2_err(&vch->dev->v4l2_dev, "buffer too small (%lu < %lu)\n",
			vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct tw6869_vch *vch = vb2_get_drv_priv(vb->vb2_queue);
	struct tw6869_buf *buf = to_tw6869_buf(vb);
	unsigned long flags;

	spin_lock_irqsave(&vch->lock, flags);
	list_add_tail(&buf->list, &vch->buf_list);
	spin_unlock_irqrestore(&vch->lock, flags);
}

static int start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct tw6869_vch *vch = vb2_get_drv_priv(vq);
	struct tw6869_dev *dev = vch->dev;
	unsigned long flags;

	if (count < 2)
		return -ENOBUFS;

	spin_lock_irqsave(&vch->lock, flags);
	vch->p_buf = list_first_entry(&vch->buf_list, struct tw6869_buf, list);
	list_del(&vch->p_buf->list);

	vch->b_buf = list_first_entry(&vch->buf_list, struct tw6869_buf, list);
	list_del(&vch->b_buf->list);

	vch->sequence = 0;
	vch->dcount = 0;
	vch->pb = 0;
	spin_unlock_irqrestore(&vch->lock, flags);

	spin_lock_irqsave(&dev->rlock, flags);
	tw6869_vch_hw_cfg(vch);
	tw6869_id_dma_cmd(dev, vch->id, TW_DMA_ON);
	spin_unlock_irqrestore(&dev->rlock, flags);

	return 0;
}

static int stop_streaming(struct vb2_queue *vq)
{
	struct tw6869_vch *vch = vb2_get_drv_priv(vq);
	struct tw6869_dev *dev = vch->dev;
	struct tw6869_buf *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&dev->rlock, flags);
	tw6869_id_dma_cmd(dev, vch->id, TW_DMA_OFF);
	spin_unlock_irqrestore(&dev->rlock, flags);

	spin_lock_irqsave(&vch->lock, flags);
	if (vch->p_buf) {
		buf = vch->p_buf;
		vch->p_buf = NULL;
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	if (vch->b_buf) {
		buf = vch->b_buf;
		vch->b_buf = NULL;
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(buf, node, &vch->buf_list, list) {
		vb2_buffer_done(&buf->vb, VB2_BUF_STATE_ERROR);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&vch->lock, flags);

	return 0;
}

static struct vb2_ops tw6869_qops = {
	.queue_setup		= queue_setup,
	.buf_init		= buffer_init,
	.buf_prepare		= buffer_prepare,
	.buf_queue		= buffer_queue,
	.start_streaming	= start_streaming,
	.stop_streaming		= stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int tw6869_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct tw6869_vch *vch = video_drvdata(file);
	struct tw6869_dev *dev = vch->dev;

	strlcpy(cap->driver, KBUILD_MODNAME, sizeof(cap->driver));
	snprintf(cap->card, sizeof(cap->card), "tw6869 vch%u", ID2CH(vch->id));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "PCI:%s",
		pci_name(dev->pdev));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
		V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int tw6869_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct tw6869_vch *vch = video_drvdata(file);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	ret = to_tw6869_pixformat(pix->pixelformat);
	if (ret < 0)
		return ret;

	tw6869_fill_pix_format(vch, pix);
	return 0;
}

static int tw6869_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct tw6869_vch *vch = video_drvdata(file);
	int ret;

	ret = tw6869_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	if (vb2_is_busy(&vch->queue))
		return -EBUSY;

	vch->format = f->fmt.pix;
	return 0;
}

static int tw6869_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct tw6869_vch *vch = video_drvdata(file);

	f->fmt.pix = vch->format;
	return 0;
}

static int tw6869_enum_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_fmtdesc *f)
{
	if (f->index > 2)
		return -EINVAL;

	switch (f->index) {
	case 1:
		strcpy(f->description, "4:2:2, packed, YUYV");
		f->pixelformat = V4L2_PIX_FMT_YUYV;
		break;
	case 2:
		strcpy(f->description, "16 bpp RGB, le");
		f->pixelformat = V4L2_PIX_FMT_RGB565;
		break;
	default:
		strcpy(f->description, "4:2:2, packed, UYVY");
		f->pixelformat = V4L2_PIX_FMT_UYVY;
	}
	f->flags = 0;
	return 0;
}

static int tw6869_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct tw6869_vch *vch = video_drvdata(file);

	if (fsize->index != 0)
		return -EINVAL;

	fsize->discrete.width = vch->format.width;
	fsize->discrete.height = vch->format.height;

	return 0;
}

static int tw6869_querystd(struct file *file, void *priv, v4l2_std_id *std)
{
	struct tw6869_vch *vch = video_drvdata(file);
	struct tw6869_dev *dev = vch->dev;
	unsigned int std_now;
	char *std_str;

	std_now = (tw_read(dev, R8_STANDARD_SEL(vch->id)) & 0x70) >> 4;

	switch (std_now) {
	case TW_STD_PAL_M:
		std_str = "PAL (M)";
		*std = V4L2_STD_525_60;
		break;
	case TW_STD_PAL_60:
		std_str = "PAL 60";
		*std = V4L2_STD_525_60;
		break;
	case TW_STD_NTSC_M:
		std_str = "NTSC (M)";
		*std = V4L2_STD_525_60;
		break;
	case TW_STD_NTSC_443:
		std_str = "NTSC 4.43";
		*std = V4L2_STD_525_60;
		break;
	case TW_STD_PAL:
		std_str = "PAL (B,D,G,H,I)";
		*std = V4L2_STD_625_50;
		break;
	case TW_STD_PAL_CN:
		std_str = "PAL (CN)";
		*std = V4L2_STD_625_50;
		break;
	case TW_STD_SECAM:
		std_str = "SECAM";
		*std = V4L2_STD_625_50;
		break;
	default:
		std_str = "Not valid";
		*std = 0;
	}
	v4l2_info(&dev->v4l2_dev, "vch%u std %s\n", ID2CH(vch->id), std_str);
	return 0;
}

static int tw6869_s_std(struct file *file, void *priv, v4l2_std_id std)
{
	struct tw6869_vch *vch = video_drvdata(file);
	v4l2_std_id new_std = (std & V4L2_STD_625_50) ?
				V4L2_STD_625_50 : V4L2_STD_525_60;

	if (new_std == vch->std)
		return 0;

	if (vb2_is_busy(&vch->queue))
		return -EBUSY;

	vch->std = new_std;
	vch->fps = (new_std & V4L2_STD_625_50) ? 25 : 30;
	tw6869_fill_pix_format(vch, &vch->format);
	return 0;
}

static int tw6869_g_std(struct file *file, void *priv, v4l2_std_id *std)
{
	struct tw6869_vch *vch = video_drvdata(file);
	v4l2_std_id new_std = 0;

	tw6869_querystd(file, priv, &new_std);
	if (new_std)
		tw6869_s_std(file, priv, new_std);

	vch->fps = (vch->std & V4L2_STD_625_50) ? 25 : 30;
	*std = vch->std;
	return 0;
}

/* SK-TW6869: only input0 is available */
static int tw6869_enum_input(struct file *file, void *priv,
				struct v4l2_input *i)
{
	if (i->index >= TW_VIN_MAX)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	i->std = V4L2_STD_ALL;
	sprintf(i->name, "Camera %d", i->index);
	return 0;
}

static int tw6869_s_input(struct file *file, void *priv, unsigned int i)
{
	struct tw6869_vch *vch = video_drvdata(file);

	vch->input = i;
	return 0;
}

static int tw6869_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct tw6869_vch *vch = video_drvdata(file);

	*i = vch->input;
	return 0;
}

static int tw6869_g_parm(struct file *file, void *priv,
				struct v4l2_streamparm *sp)
{
	struct tw6869_vch *vch = video_drvdata(file);
	struct v4l2_captureparm *cp = &sp->parm.capture;

	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = vch->fps;
	return 0;
}

static int tw6869_s_parm(struct file *file, void *priv,
				struct v4l2_streamparm *sp)
{
	struct tw6869_vch *vch = video_drvdata(file);
	unsigned int denominator = sp->parm.capture.timeperframe.denominator;
	unsigned int numerator = sp->parm.capture.timeperframe.numerator;
	unsigned int fps;

	fps = (!numerator || !denominator) ? 0 : denominator / numerator;
	if (vch->std & V4L2_STD_625_50)
		fps = (!fps || fps > 25) ? 25 : fps;
	else
		fps = (!fps || fps > 30) ? 30 : fps;

	vch->fps = fps;
	v4l2_info(&vch->dev->v4l2_dev,
		"vch%u fps %u\n", ID2CH(vch->id), vch->fps);

	return tw6869_g_parm(file, priv, sp);
}

/* The control handler */
static int tw6869_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tw6869_vch *vch =
		container_of(ctrl->handler, struct tw6869_vch, hdl);
	struct tw6869_dev *dev = vch->dev;
	unsigned int id = vch->id;
	int ret = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		tw_write(dev, R8_BRIGHT_CTRL(id), ctrl->val);
		break;
	case V4L2_CID_CONTRAST:
		tw_write(dev, R8_CONTRAST_CTRL(id), ctrl->val);
		break;
	case V4L2_CID_SATURATION:
		tw_write(dev, R8_SAT_U_CTRL(id), ctrl->val);
		tw_write(dev, R8_SAT_V_CTRL(id), ctrl->val);
		break;
	case V4L2_CID_HUE:
		tw_write(dev, R8_HUE_CTRL(id), ctrl->val);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

/*
 * File operations for the device
 */
static const struct v4l2_ctrl_ops tw6869_ctrl_ops = {
	.s_ctrl = tw6869_s_ctrl,
};

static const struct v4l2_ioctl_ops tw6869_ioctl_ops = {
	.vidioc_querycap = tw6869_querycap,
	.vidioc_try_fmt_vid_cap = tw6869_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap = tw6869_s_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap = tw6869_g_fmt_vid_cap,
	.vidioc_enum_fmt_vid_cap = tw6869_enum_fmt_vid_cap,
	.vidioc_enum_framesizes = tw6869_enum_framesizes,

	.vidioc_querystd = tw6869_querystd,
	.vidioc_s_std = tw6869_s_std,
	.vidioc_g_std = tw6869_g_std,

	.vidioc_enum_input = tw6869_enum_input,
	.vidioc_s_input = tw6869_s_input,
	.vidioc_g_input = tw6869_g_input,

	.vidioc_g_parm = tw6869_g_parm,
	.vidioc_s_parm = tw6869_s_parm,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_log_status = v4l2_ctrl_log_status,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations tw6869_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

static int tw6869_vch_register(struct tw6869_vch *vch)
{
	struct tw6869_dev *dev = vch->dev;
	struct v4l2_ctrl_handler *hdl = &vch->hdl;
	struct vb2_queue *q = &vch->queue;
	struct video_device *vdev = &vch->vdev;
	int ret = 0;

	/* Add the controls */
	v4l2_ctrl_handler_init(hdl, 4);
	v4l2_ctrl_new_std(hdl, &tw6869_ctrl_ops,
		  V4L2_CID_BRIGHTNESS, -128, 127, 1, 0);
	v4l2_ctrl_new_std(hdl, &tw6869_ctrl_ops,
		  V4L2_CID_CONTRAST, 0, 255, 1, 100);
	v4l2_ctrl_new_std(hdl, &tw6869_ctrl_ops,
		  V4L2_CID_SATURATION, 0, 255, 1, 128);
	v4l2_ctrl_new_std(hdl, &tw6869_ctrl_ops,
		  V4L2_CID_HUE, -128, 127, 1, 0);
	if (hdl->error) {
		ret = hdl->error;
		return ret;
	}

	/* Fill in the initial format-related settings */
	vch->std = V4L2_STD_525_60;
	vch->fps = 30;
	vch->format.pixelformat = V4L2_PIX_FMT_UYVY;
	tw6869_fill_pix_format(vch, &vch->format);

	mutex_init(&vch->mlock);

	/* Initialize the vb2 queue */
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_READ;
	q->drv_priv = vch;
	q->buf_struct_size = sizeof(struct tw6869_buf);
	q->ops = &tw6869_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &vch->mlock;
	q->gfp_flags = __GFP_DMA32;
	ret = vb2_queue_init(q);
	if (ret)
		goto free_hdl;

	spin_lock_init(&vch->lock);
	INIT_LIST_HEAD(&vch->buf_list);

	/* Initialize the video_device structure */
	strlcpy(vdev->name, KBUILD_MODNAME, sizeof(vdev->name));
	vdev->release = video_device_release_empty;
	vdev->fops = &tw6869_fops,
	vdev->ioctl_ops = &tw6869_ioctl_ops,
	vdev->lock = &vch->mlock;
	vdev->queue = q;
	vdev->v4l2_dev = &dev->v4l2_dev;
	vdev->ctrl_handler = hdl;
	vdev->tvnorms = V4L2_STD_ALL;
	video_set_drvdata(vdev, vch);
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (!ret)
		return 0;

free_hdl:
	v4l2_ctrl_handler_free(hdl);
	return ret;
}

static void tw6869_video_unregister(struct tw6869_dev *dev)
{
	unsigned int i;

	/* Reset and disable all DMA channels */
	tw_write(dev, R32_DMA_CMD, 0);
	tw_write(dev, R32_DMA_CHANNEL_ENABLE, 0);

	if (!dev->alloc_ctx)
		return;

	if (dev->ch_max > TW_CH_MAX)
		dev->ch_max = TW_CH_MAX;

	for (i = 0; i < dev->ch_max; i++) {
		struct tw6869_vch *vch = &dev->vch[ID2CH(i)];
		video_unregister_device(&vch->vdev);
		v4l2_ctrl_handler_free(&vch->hdl);
	}

	v4l2_device_unregister(&dev->v4l2_dev);
	vb2_dma_contig_cleanup_ctx(dev->alloc_ctx);
	dev->alloc_ctx = NULL;
}

static int tw6869_video_register(struct tw6869_dev *dev)
{
	struct pci_dev *pdev = dev->pdev;
	unsigned int i;
	int ret;

	/* Initialize the top-level structure */
	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret)
		return ret;

	dev->alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
	if (IS_ERR(dev->alloc_ctx)) {
		ret = PTR_ERR(dev->alloc_ctx);
		v4l2_err(&dev->v4l2_dev, "can't allocate buffer context\n");
		v4l2_device_unregister(&dev->v4l2_dev);
		dev->alloc_ctx = NULL;
		return ret;
	}

	for (i = 0; i < TW_CH_MAX; i++) {
		struct tw6869_vch *vch = &dev->vch[ID2CH(i)];
		vch->dev = dev;
		vch->id = i;
		ret = tw6869_vch_register(vch);
		if (ret) {
			dev->ch_max = i;
			tw6869_video_unregister(dev);
			return ret;
		}
		dev_info(&pdev->dev, "vch%i registered as %s\n",
			 vch->id,
			 video_device_node_name(&vch->vdev));
	}
	return 0;
}

/**********************************************************************/

static int tw6869_pcm_hw_params(struct snd_pcm_substream *ss,
				struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(hw_params));
}

static int tw6869_pcm_hw_free(struct snd_pcm_substream *ss)
{
	return snd_pcm_lib_free_pages(ss);
}

static const struct snd_pcm_hardware tw6869_capture_hw = {
	.info			= (SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP_VALID),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= SNDRV_PCM_RATE_48000,
	.rate_min		= 48000,
	.rate_max		= 48000,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= TW_PAGE_SIZE * TW_APAGE_MAX,
	.period_bytes_min	= TW_PAGE_SIZE,
	.period_bytes_max	= TW_PAGE_SIZE,
	.periods_min		= 2,
	.periods_max		= TW_APAGE_MAX,
};

static int tw6869_pcm_open(struct snd_pcm_substream *ss)
{
	struct tw6869_dev *dev = snd_pcm_substream_chip(ss);
	struct tw6869_ach *ach = &dev->ach[ID2CH(ss->number)];
	struct snd_pcm_runtime *rt = ss->runtime;
	int ret;

	rt->hw = tw6869_capture_hw;
	ret = snd_pcm_hw_constraint_integer(rt, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		return ret;

	ach->ss = ss;
	return 0;
}

static int tw6869_pcm_close(struct snd_pcm_substream *ss)
{
	struct tw6869_dev *dev = snd_pcm_substream_chip(ss);
	struct tw6869_ach *ach = &dev->ach[ID2CH(ss->number)];

	ach->ss = NULL;
	return 0;
}

static int tw6869_pcm_prepare(struct snd_pcm_substream *ss)
{
	struct tw6869_dev *dev = snd_pcm_substream_chip(ss);
	struct tw6869_ach *ach = &dev->ach[ID2CH(ss->number)];
	struct snd_pcm_runtime *rt = ss->runtime;
	unsigned int period = snd_pcm_lib_period_bytes(ss);
	unsigned long flags;
	unsigned int i;

	spin_lock_irqsave(&ach->lock, flags);
	ach->p_buf = NULL;
	ach->b_buf = NULL;

	if ((period != TW_PAGE_SIZE) ||
		(rt->periods < 2) ||
		(rt->periods > TW_APAGE_MAX)) {
		spin_unlock_irqrestore(&ach->lock, flags);
		return -EINVAL;
	}

	INIT_LIST_HEAD(&ach->buf_list);
	for (i = 0; i < rt->periods; i++) {
		ach->buf[i].dma = rt->dma_addr + period * i;
		INIT_LIST_HEAD(&ach->buf[i].list);
		list_add_tail(&ach->buf[i].list, &ach->buf_list);
	}

	ach->p_buf = list_first_entry(&ach->buf_list, struct tw6869_buf, list);
	list_move_tail(&ach->p_buf->list, &ach->buf_list);

	ach->b_buf = list_first_entry(&ach->buf_list, struct tw6869_buf, list);
	list_move_tail(&ach->b_buf->list, &ach->buf_list);

	ach->ptr = 0;
	ach->pb = 0;
	spin_unlock_irqrestore(&ach->lock, flags);

	return 0;
}

static void tw6869_ach_hw_cfg(struct tw6869_ach *ach)
{
	struct tw6869_dev *dev = ach->dev;

	tw_write(dev, R32_DMA_P_ADDR(ach->id), ach->p_buf->dma);
	tw_write(dev, R32_DMA_B_ADDR(ach->id), ach->b_buf->dma);
}

static int tw6869_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct tw6869_dev *dev = snd_pcm_substream_chip(ss);
	struct tw6869_ach *ach = &dev->ach[ID2CH(ss->number)];
	unsigned long flags;
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (ach->p_buf && ach->b_buf) {
			spin_lock_irqsave(&dev->rlock, flags);
			tw6869_ach_hw_cfg(ach);
			tw6869_id_dma_cmd(dev, ach->id, TW_DMA_ON);
			spin_unlock_irqrestore(&dev->rlock, flags);
		} else {
			ret = -EIO;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&dev->rlock, flags);
		tw6869_id_dma_cmd(dev, ach->id, TW_DMA_OFF);
		spin_unlock_irqrestore(&dev->rlock, flags);

		spin_lock_irqsave(&ach->lock, flags);
		ach->p_buf = NULL;
		ach->b_buf = NULL;
		spin_unlock_irqrestore(&ach->lock, flags);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static snd_pcm_uframes_t tw6869_pcm_pointer(struct snd_pcm_substream *ss)
{
	struct tw6869_dev *dev = snd_pcm_substream_chip(ss);
	struct tw6869_ach *ach = &dev->ach[ID2CH(ss->number)];

	return bytes_to_frames(ss->runtime, ach->ptr);
}

static struct snd_pcm_ops tw6869_pcm_ops = {
	.open = tw6869_pcm_open,
	.close = tw6869_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = tw6869_pcm_hw_params,
	.hw_free = tw6869_pcm_hw_free,
	.prepare = tw6869_pcm_prepare,
	.trigger = tw6869_pcm_trigger,
	.pointer = tw6869_pcm_pointer,
};

static int tw6869_snd_pcm_init(struct tw6869_dev *dev)
{
	struct snd_card *card = dev->snd_card;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *ss;
	int ret;

	ret = snd_pcm_new(card, card->driver, 0, 0, TW_CH_MAX, &pcm);
	if (ret < 0)
		return ret;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &tw6869_pcm_ops);
	snd_pcm_chip(pcm) = dev;
	pcm->info_flags = 0;
	strcpy(pcm->name, card->shortname);

	ss = pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream;
	while (ss && (ss->next != ss)) {
		struct tw6869_ach *ach = &dev->ach[ID2CH(ss->number)];
		ach->dev = dev;
		ach->id = ID2CH(ss->number) + TW_CH_MAX;
		spin_lock_init(&ach->lock);
		sprintf(ss->name, "vch%u audio", ID2CH(ss->number));
		ss = ss->next;
	}

	return snd_pcm_lib_preallocate_pages_for_all(pcm,
				SNDRV_DMA_TYPE_DEV,
				snd_dma_pci_data(dev->pdev),
				TW_APAGE_MAX * TW_PAGE_SIZE,
				TW_APAGE_MAX * TW_PAGE_SIZE);
}

static void tw6869_audio_unregister(struct tw6869_dev *dev)
{
	/* Reset and disable audio DMA */
	tw_clear(dev, R32_DMA_CMD, TW_AID);
	tw_clear(dev, R32_DMA_CHANNEL_ENABLE, TW_AID);

	if (!dev->snd_card)
		return;

	snd_card_free(dev->snd_card);
	dev->snd_card = NULL;
}

/* TODO: mixer controls */
static int tw6869_audio_register(struct tw6869_dev *dev)
{
	struct pci_dev *pdev = dev->pdev;
	static struct snd_device_ops ops = { NULL };
	struct snd_card *card;
	int ret;

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, KBUILD_MODNAME,
				THIS_MODULE, 0, &card);
	if (ret < 0)
		return ret;

	dev->snd_card = card;

	strcpy(card->driver, KBUILD_MODNAME);
	strcpy(card->shortname, KBUILD_MODNAME);
	sprintf(card->longname, "%s on %s IRQ %d", card->shortname,
		pci_name(pdev), pdev->irq);

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, dev, &ops);
	if (ret < 0)
		goto snd_error;

	snd_card_set_dev(card, &pdev->dev);

	ret = tw6869_snd_pcm_init(dev);
	if (ret < 0)
		goto snd_error;

	ret = snd_card_register(card);
	if (!ret)
		return 0;

snd_error:
	snd_card_free(card);
	dev->snd_card = NULL;
	return ret;
}

/**********************************************************************/

static void tw6869_reset(struct tw6869_dev *dev)
{
	/* Software Reset */
	tw_write(dev, R32_SYS_SOFT_RST, 0x01);
	tw_write(dev, R32_SYS_SOFT_RST, 0x0F);

	/* Reset Internal audio and video decoders */
	tw_write(dev, R8_AVSRST(0), 0x1F);
	tw_write(dev, R8_AVSRST(4), 0x1F);

	/* Reset all DMA channels */
	tw_write(dev, R32_DMA_CMD, 0);

	/* Disable all DMA channels */
	tw_write(dev, R32_DMA_CHANNEL_ENABLE, 0);

	/* Enable DMA FIFO overflow and pointer check */
	tw_write(dev, R32_DMA_CONFIG, 0x00FFFF04);

	/* Minimum time span for DMA interrupting host (default: 0x00098968) */
	tw_write(dev, R32_DMA_TIMER_INTERVAL, 0x00098968);

	/* DMA timeout (default: 0x140C8584) */
	tw_write(dev, R32_DMA_CHANNEL_TIMEOUT, 0x140C8584);

	/* Frame mode DMA */
	tw_write(dev, R32_PHASE_REF, 0xAAAA144D);

	/* Show blue background if no signal */
	tw_write(dev, R8_MISC_CONTROL1(0), 0xE7);
	tw_write(dev, R8_MISC_CONTROL1(4), 0xE7);

	/* Audio DMA 4096 bytes, sampling frequency reference 48 kHz */
	tw_write(dev, R32_AUDIO_CONTROL1, 0x80000001 | (0x0A2C << 5));
	tw_write(dev, R32_AUDIO_CONTROL2, 0x0A2C2AAA);
}

static int tw6869_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct tw6869_dev *dev;
	int ret;

	/* Allocate a new instance */
	dev = devm_kzalloc(&pdev->dev, sizeof(struct tw6869_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;

	pci_set_master(pdev);
	ret = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "no suitable DMA available\n");
		goto disable_pci;
	}

	ret = pci_request_regions(pdev, KBUILD_MODNAME);
	if (ret)
		goto disable_pci;

	dev->mmio = pci_iomap(pdev, 0, 0);
	if (!dev->mmio) {
		ret = -EIO;
		goto release_regs;
	}

	tw6869_reset(dev);

	ret = devm_request_irq(&pdev->dev, pdev->irq,
				tw6869_irq, IRQF_SHARED, KBUILD_MODNAME, dev);
	if (ret) {
		dev_err(&pdev->dev, "request_irq failed\n");
		goto unmap_regs;
	}

	dev->pdev = pdev;
	dev->ch_max = TW_CH_MAX;
	spin_lock_init(&dev->rlock);

	ret = tw6869_video_register(dev);
	if (ret)
		goto unmap_regs;

	ret = tw6869_audio_register(dev);
	if (ret)
		goto video_unreg;

	dev_info(&pdev->dev, "driver loaded\n");
	return 0;

video_unreg:
	tw6869_video_unregister(dev);
unmap_regs:
	pci_iounmap(pdev, dev->mmio);
release_regs:
	pci_release_regions(pdev);
disable_pci:
	pci_disable_device(pdev);
	return ret;
}

static void tw6869_remove(struct pci_dev *pdev)
{
	struct v4l2_device *v4l2_dev = pci_get_drvdata(pdev);
	struct tw6869_dev *dev =
		container_of(v4l2_dev, struct tw6869_dev, v4l2_dev);

	tw6869_audio_unregister(dev);
	tw6869_video_unregister(dev);
	pci_iounmap(pdev, dev->mmio);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

/* TODO: PM */
static struct pci_driver tw6869_driver = {
	.name = KBUILD_MODNAME,
	.probe = tw6869_probe,
	.remove = tw6869_remove,
	.id_table = tw6869_pci_tbl,
};

module_pci_driver(tw6869_driver);
