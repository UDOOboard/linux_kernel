/*
 * tda1997x.c -- MXC video capture driver for i.MX boards with
 *               tda1997x HDMI receiver
 *
 * Copyright (C) 2013 Gateworks Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/* The tda1997x-core driver manages the i2c interface with the device.
 * This driver merely calls into that driver for current video parameters
 * when necessary.  The platform data for the core driver contains the
 * video output bus configuration that configures the video port
 * mapping between the tda1997x and a i.MX6 CSI parallel bus for 8bit
 * bt656 with embedded syncs as this is the only video format compatible
 * between the tda1997x and the i.MX6 CSI.
 *
 * see <linux/include/mfd/tda1997x-core.h> for details
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fsl_devices.h>
#include <linux/mfd/core.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/tda1997x-core.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>

#include <media/v4l2-chip-ident.h>

#include "v4l2-int-device.h"
#include "mxc_v4l2_capture.h"

/*!
 * Maintains the information on the current state of the sensor.
 */
struct sensor {
	struct sensor_data sen;
	tda1997x_videofmt_t vidfmt;
} tda1997x_data;

/***********************************************************************
 * mxc_v4l2_capture interface.
 ***********************************************************************/

/***********************************************************************
 * IOCTL Functions from v4l2_int_ioctl_desc.
 ***********************************************************************/

/*!
 * ioctl_g_ifparm - V4L2 sensor interface handler for vidioc_int_g_ifparm_num
 * s: pointer to standard V4L2 device structure
 * p: pointer to standard V4L2 vidioc_int_g_ifparm_num ioctl structure
 *
 * Gets slave interface parameters.
 * Calculates the required xclk value to support the requested
 * clock parameters in p.  This value is returned in the p
 * parameter.
 *
 * vidioc_int_g_ifparm returns platform-specific information about the
 * interface settings used by the sensor.
 *
 * Called on open.
 */
static int ioctl_g_ifparm(struct v4l2_int_device *s, struct v4l2_ifparm *p)
{
	struct sensor *sensor = s->priv;
	tda1997x_vidout_fmt_t fmt;

	if (s == NULL) {
		pr_err("   ERROR!! no slave device set!\n");
		return -ENODEV;
	}

	if (tda1997x_get_vidout_fmt(&fmt))
		return -ENODEV;
	pr_debug("%s: %dx%d%c@%dfps\n", __func__, fmt.width, fmt.height,
		fmt.interlaced?'i':'p', fmt.fps);

	/* Initialize structure to 0s then set any non-0 values. */
	memset(p, 0, sizeof(*p));
	p->if_type = V4L2_IF_TYPE_BT656; /* This is the only possibility. */

	if (sensor->vidfmt == VIDEOFMT_422_SMP) { /* YCbCr 4:2:2 semi-planar */
		p->u.bt656.nobt_vs_inv = 0;
		p->u.bt656.nobt_hs_inv = 1;
		p->u.bt656.bt_sync_correct = 1;
		p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_16BIT;
		p->u.bt656.clock_curr = -1; /* gated clock mode */
	} else if (sensor->vidfmt == VIDEOFMT_422_CCIR) { /* YCbCr 4:2:2 CCIR656 */
		p->u.bt656.mode = V4L2_IF_TYPE_BT656_MODE_NOBT_8BIT;
		p->u.bt656.clock_curr = (fmt.interlaced)?0:1;
	}

	return 0;
}

/*!
 * Sets the device power.
 *
 * s  pointer to the device device
 * on if 1, power is to be turned on.  0 means power is to be turned off
 *
 * ioctl_s_power - V4L2 sensor interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @on: power state to which device is to be set
 *
 * Sets devices power state to requrested state, if possible.
 * This is called on open, close, suspend and resume.
static int ioctl_s_power(struct v4l2_int_device *s, int on)
{
	struct sensor *sensor = s->priv;

	if (on && !sensor->sen.on) {
	} else if (!on && sensor->sen.on) {
	}

	sensor->sen.on = on;

	return 0;
}
 */

/*!
 * ioctl_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ioctl_g_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	struct sensor *sensor = s->priv;
	struct v4l2_captureparm *cparm = &a->parm.capture;

	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   type is V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->sen.streamcap.capability;
		cparm->timeperframe = sensor->sen.streamcap.timeperframe;
		cparm->capturemode = sensor->sen.streamcap.capturemode;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("ioctl_g_parm:type is unknown %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 *
 * This driver cannot change these settings.
 */
static int ioctl_s_parm(struct v4l2_int_device *s, struct v4l2_streamparm *a)
{
	switch (a->type) {
	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		break;
	}

	return 0;
}

/*!
 * ioctl_g_fmt_cap - V4L2 sensor interface handler for ioctl_g_fmt_cap
 * @s: pointer to standard V4L2 device structure
 * @f: pointer to standard V4L2 v4l2_format structure
 *
 * Returns the sensor's current pixel format in the v4l2_format
 * parameter.
 */
static int ioctl_g_fmt_cap(struct v4l2_int_device *s, struct v4l2_format *f)
{
	struct sensor *sensor = s->priv;
	tda1997x_vidout_fmt_t fmt;

	if (tda1997x_get_vidout_fmt(&fmt))
		return -ENODEV;

	pr_debug("%s: %dx%d%c@%dfps\n", __func__, fmt.width, fmt.height,
		fmt.interlaced?'i':'p', fmt.fps);
	sensor->sen.pix.height = fmt.height;
	sensor->sen.pix.width = fmt.width;
	sensor->sen.streamcap.timeperframe.denominator = fmt.fps;
	sensor->sen.streamcap.timeperframe.numerator = 1;
	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		pr_debug("   Returning size of %dx%d\n",
			 sensor->sen.pix.width, sensor->sen.pix.height);
		f->fmt.pix = sensor->sen.pix;
		pr_debug("   Returning format of %s\n", (char*)&f->fmt.pix.pixelformat);
		break;

	case V4L2_BUF_TYPE_PRIVATE: {
		}
		break;

	default:
		f->fmt.pix = sensor->sen.pix;
		break;
	}

	return 0;
}

/*!
 * ioctl_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ioctl_enum_framesizes(struct v4l2_int_device *s,
				 struct v4l2_frmsizeenum *fsize)
{
	struct sensor *sensor = s->priv;
	tda1997x_vidout_fmt_t fmt;

	if (fsize->index >= 1)
		return -EINVAL;

	if (tda1997x_get_vidout_fmt(&fmt))
		return -ENODEV;

	pr_debug("%s: %dx%d%c@%dfps\n", __func__, fmt.height, fmt.width,
		fmt.interlaced?'i':'p', fmt.fps);
	sensor->sen.pix.height = fmt.height;
	sensor->sen.pix.width = fmt.width;
	fsize->discrete.width = sensor->sen.pix.width;
	fsize->discrete.height  = sensor->sen.pix.height;

	return 0;
}

/*!
 * ioctl_g_chip_ident - V4L2 sensor interface handler for
 *			VIDIOC_DBG_G_CHIP_IDENT ioctl
 * @s: pointer to standard V4L2 device structure
 * @id: pointer to int
 *
 * Return 0.
 */
static int ioctl_g_chip_ident(struct v4l2_int_device *s, int *id)
{
	struct sensor *sensor = s->priv;
	((struct v4l2_dbg_chip_ident *)id)->match.type =
					V4L2_CHIP_MATCH_I2C_DRIVER;
	if (sensor->vidfmt == VIDEOFMT_422_SMP) { /* YCbCr 4:2:2 semi-planar */
		strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"tda1997x_decoder_yuv422");
	} else
		strcpy(((struct v4l2_dbg_chip_ident *)id)->match.name,
						"tda1997x_decoder_bt656");
	((struct v4l2_dbg_chip_ident *)id)->ident = V4L2_IDENT_TDA19971;

	return 0;
}

/*!
 * This structure defines all the ioctls for this module.
 */
static struct v4l2_int_ioctl_desc tda1997x_ioctl_desc[] = {

/* {vidioc_int_s_power_num, (v4l2_int_ioctl_func*)ioctl_s_power}, */
	{vidioc_int_g_ifparm_num, (v4l2_int_ioctl_func*)ioctl_g_ifparm},
	{vidioc_int_g_fmt_cap_num, (v4l2_int_ioctl_func*)ioctl_g_fmt_cap},
	/*!
	 * If the requested format is supported, configures the HW to use that
	 * format, returns error code if format not supported or HW can't be
	 * correctly configured.
	 */
	{vidioc_int_g_parm_num, (v4l2_int_ioctl_func*)ioctl_g_parm},
	{vidioc_int_s_parm_num, (v4l2_int_ioctl_func*)ioctl_s_parm},
	{vidioc_int_enum_framesizes_num,
				(v4l2_int_ioctl_func *) ioctl_enum_framesizes},
	{vidioc_int_g_chip_ident_num,
				(v4l2_int_ioctl_func *)ioctl_g_chip_ident},
};

static struct v4l2_int_slave tda1997x_slave = {
	.ioctls = tda1997x_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tda1997x_ioctl_desc),
};

static struct v4l2_int_device tda1997x_int_device = {
	.module = THIS_MODULE,
	.name = "tda1997x-video",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tda1997x_slave,
	},
};

static int tda1997x_video_probe(struct platform_device *pdev)
{
	struct sensor *sens = &tda1997x_data;
	struct pinctrl *pinctrl;
	struct device *dev = &pdev->dev;
	struct regmap *gpr;
	tda1997x_vidout_fmt_t fmt;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	/* pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}

	/* Set initial values for the sensor struct. */
	memset(sens, 0, sizeof(tda1997x_data));
	if (tda1997x_get_vidout_fmt(&fmt))
		return -ENODEV;
	sens->vidfmt = fmt.sensor_vidfmt;
	sens->sen.streamcap.timeperframe.denominator = 0;
	sens->sen.streamcap.timeperframe.numerator = 0;
	sens->sen.pix.width = 0;
	sens->sen.pix.height = 0;
	if (sens->vidfmt == VIDEOFMT_422_SMP)
		sens->sen.pix.pixelformat = IPU_PIX_FMT_GENERIC_16;
	else
		sens->sen.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	sens->sen.on = true;

	ret = of_property_read_u32(dev->of_node, "csi_id",
				   &(sens->sen.csi));
	if (ret) {
		dev_err(dev, "csi_id invalid\n");
		return ret;
	}

	ret = of_property_read_u32(dev->of_node, "ipu_id",
				   &(sens->sen.ipu_id));
	if (ret) {
		dev_err(dev, "ipu_id invalid\n");
		return ret;
	}

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q")) {
			int mask = sens->sen.csi ? (1 << 20) : (1 << 19);

			regmap_update_bits(gpr, IOMUXC_GPR1, mask, mask);
		} else if (of_machine_is_compatible("fsl,imx6dl")) {
			int mask = sens->sen.csi ? (7 << 3) : (7 << 0);
			int val =  sens->sen.csi ? (4 << 3) : (4 << 0);

			regmap_update_bits(gpr, IOMUXC_GPR13, mask, val);
		}
	} else {
		pr_err("%s: failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}

	dev_dbg(dev, "IPU%d_CSI%d\n", sens->sen.ipu_id + 1, sens->sen.csi);
	dev_dbg(dev, "type is %d (expect %d)\n",
		 tda1997x_int_device.type, v4l2_int_type_slave);
	dev_dbg(dev, "num ioctls is %d\n",
		 tda1997x_int_device.u.slave->num_ioctls);

	/* This function attaches this structure to the /dev/video<n> device */
	tda1997x_int_device.priv = sens;
	ret = v4l2_int_device_register(&tda1997x_int_device);

	return ret;
}

static int tda1997x_video_remove(struct platform_device *pdev)
{
	v4l2_int_device_unregister(&tda1997x_int_device);

	return 0;
}

static const struct of_device_id tda1997x_video_driver_ids[] = {
	{ .compatible = "fsl,imx-tda1997x-video", },
	{ /* sentinel */ }
};

static struct platform_driver tda1997x_video_driver = {
	.driver   = {
		.owner  = THIS_MODULE,
		.name = "imx-tda1997x-video",
		.of_match_table = tda1997x_video_driver_ids,
	},
	.probe    = tda1997x_video_probe,
	.remove   = tda1997x_video_remove,
};
module_platform_driver(tda1997x_video_driver);

MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_DESCRIPTION("TDA1997X hdmi receiver MXC video capture driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imx-tda1997x-video");
