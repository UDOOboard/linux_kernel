/*
 * imx_udooneo_mcc_tty.c - tty driver used to comunicate with
 * M4 core on posix tty interface.
 *
 * Copyright (C) 2015-2016 Seco S.r.L. All Rights Reserved.
 * Based on IMX MCC TEST driver.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/mcc_config_linux.h>
#include <linux/mcc_common.h>
#include <linux/mcc_api.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#define MCC_TTY_NON_LOCKING_READ    1000
#define MCC_TTY_LOCKING_READ        0xffffffff
#define MCC_TTY_BUFFER_SEND_SIZE    512

/**
 * struct mcctty_port - Wrapper struct for imx mcc tty port.
 * @port:		TTY port data
 */
struct mcctty_port {
	struct delayed_work	read;
	struct tty_port		port;
	spinlock_t		rx_lock;
};

static struct mcctty_port mcc_tty_port;

enum {
	MCC_NODE_A9 = 0,
	MCC_NODE_M4 = 0,

	MCC_A9_PORT = 1,
	MCC_M4_PORT = 2,
};

/* mcc endpoints */
static MCC_ENDPOINT mcc_endpoint_a9 = { 0, MCC_NODE_A9, MCC_A9_PORT };
static MCC_ENDPOINT mcc_endpoint_m4 = { 1, MCC_NODE_M4, MCC_M4_PORT };

// used for receive messages
struct mcc_tty_msg {
	char data[MCC_ATTR_BUFFER_SIZE_IN_BYTES - 24];
	uint16_t dummy;		// for zero terminator
};

static struct tty_port_operations mcctty_port_ops = { };

struct task_struct *task;
int mcc_read_thread_running = 1;

static int mcc_read_thread(void *arg)
{
	int ret = 0, space;
	unsigned char *cbuf;
	MCC_MEM_SIZE num_of_received_bytes;
	struct mcc_tty_msg tty_msg;
	struct mcctty_port *cport = &mcc_tty_port;

	while (mcc_read_thread_running) {
		ret = mcc_recv(&mcc_endpoint_m4, &mcc_endpoint_a9, &tty_msg,
			       sizeof(struct mcc_tty_msg),
			       &num_of_received_bytes,
			       MCC_TTY_NON_LOCKING_READ);

		if (ret == MCC_SUCCESS) {
			if (num_of_received_bytes > 0) {
				/* flush the recv-ed data to tty node */
				tty_msg.data[num_of_received_bytes++] = 0;
				spin_lock_bh(&cport->rx_lock);
				space = tty_prepare_flip_string(&cport->port, &cbuf,
								strlen(tty_msg.data));
				if (space <= 0)
					return -ENOMEM;

				memcpy(cbuf, &tty_msg.data, num_of_received_bytes);
				tty_flip_buffer_push(&cport->port);
				spin_unlock_bh(&cport->rx_lock);
			}
		} else if (ret != MCC_ERR_TIMEOUT) {
			pr_err("ttyMCC: mcc_recv failed (%d)!\n", ret);
		}
	}

	do_exit(0);
	return 0;
}

static void ttymcc_thread_init(void)
{
	mcc_read_thread_running = 1;
	printk(KERN_INFO "ttyMCC read thread started\n");
	task = kthread_run(&mcc_read_thread, NULL, "ttyMCC.read");
}

static void ttymcc_thread_exit(void)
{
	mcc_read_thread_running = 0;
}

static int mcctty_install(struct tty_driver *driver, struct tty_struct *tty)
{
	return tty_port_install(&mcc_tty_port.port, driver, tty);
}

static int mcctty_open(struct tty_struct *tty, struct file *filp)
{
	ttymcc_thread_init();
	return tty_port_open(tty->port, tty, filp);
}

static void mcctty_close(struct tty_struct *tty, struct file *filp)
{
	ttymcc_thread_exit();
	return tty_port_close(tty->port, tty, filp);
}

static int mcctty_write(struct tty_struct *tty, const unsigned char *buf,
			int total)
{
	int ret = 0;
	struct mcc_tty_msg tty_msg;

	if (NULL == buf) {
		pr_err("ttyMCC: outbound message should not be null!\n");
		return -ENOMEM;
	}

	if (total > MCC_ATTR_BUFFER_SIZE_IN_BYTES) {
		pr_err("ttyMCC: outbound message must be shorter than %d!\n",
		       MCC_TTY_BUFFER_SEND_SIZE);
		return -ENOMEM;
	}

	strlcpy(tty_msg.data, buf, total + 1);
	ret = mcc_send(&mcc_endpoint_a9,
		       &mcc_endpoint_m4, &tty_msg, total, 1000);

	if (ret != MCC_SUCCESS) {
		pr_err("ttyMCC: write A9->M4 error: %d\n", ret);
	}

	return total;
}

static int mcctty_write_room(struct tty_struct *tty)
{
	/* report the space in the mcc buffer */
	return MCC_ATTR_BUFFER_SIZE_IN_BYTES;
}

static const struct tty_operations imxmcctty_ops = {
	.install		= mcctty_install,
	.open			= mcctty_open,
	.close			= mcctty_close,
	.write			= mcctty_write,
	.write_room		= mcctty_write_room,
};

static struct tty_driver *mcctty_driver;

static int imx_mcc_tty_probe(struct platform_device *pdev)
{
	int ret;
	struct mcctty_port *cport = &mcc_tty_port;
	MCC_INFO_STRUCT mcc_info;

	mcctty_driver = tty_alloc_driver(1,
					 TTY_DRIVER_RESET_TERMIOS |
					 TTY_DRIVER_UNNUMBERED_NODE);
	if (IS_ERR(mcctty_driver))
		return PTR_ERR(mcctty_driver);

	mcctty_driver->driver_name = "mcc_tty";
	mcctty_driver->name = "ttyMCC";
	mcctty_driver->major = TTYAUX_MAJOR;
	mcctty_driver->minor_start = 3;
	mcctty_driver->type = TTY_DRIVER_TYPE_CONSOLE;
	mcctty_driver->init_termios = tty_std_termios;
	mcctty_driver->init_termios.c_cflag |= CLOCAL;

	tty_set_operations(mcctty_driver, &imxmcctty_ops);

	tty_port_init(&cport->port);
	cport->port.ops = &mcctty_port_ops;
	spin_lock_init(&cport->rx_lock);
	cport->port.low_latency = cport->port.flags | ASYNC_LOW_LATENCY;

	ret = tty_register_driver(mcctty_driver);
	if (ret < 0) {
		pr_err("Couldn't install mcc tty driver: err %d\n", ret);
		goto error;
	} else
		pr_info("Installing ttyMCC driver!\n");

	ret = mcc_initialize(MCC_NODE_A9);
	if (ret) {
		pr_err("Failed to initialize ttyMCC.\n");
		ret = -ENODEV;
		goto error;
	}

	ret = mcc_get_info(MCC_NODE_A9, &mcc_info);
	if (ret) {
		pr_err("Failed to get ttyMCC info.\n");
		ret = -ENODEV;
		goto error;
	} else {
		pr_info("\nA9 ttyMCC prepares to run, MCC version is %s\n",
			mcc_info.version_string);
	}

	ret = mcc_create_endpoint(&mcc_endpoint_a9, MCC_A9_PORT);
	if (ret) {
		pr_err("Failed to create A9 ttyMCC endpoint.\n");
		ret = -ENODEV;
		goto error;
	}

	return 0;

error:
	tty_unregister_driver(mcctty_driver);
	put_tty_driver(mcctty_driver);
	tty_port_destroy(&cport->port);
	mcctty_driver = NULL;

	return ret;
}

static int imx_mcc_tty_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct mcctty_port *cport = &mcc_tty_port;

	/* destroy the mcc tty endpoint here */
	ret = mcc_destroy_endpoint(&mcc_endpoint_a9);
	if (ret)
		pr_err("Failed to destroy A9 ttyMCC endpoint.\n");
	else
		pr_info("Destroyed A9 ttyMCC endpoint.\n");

	tty_unregister_driver(mcctty_driver);
	tty_port_destroy(&cport->port);
	put_tty_driver(mcctty_driver);

	return ret;
}

static const struct of_device_id imx6sx_mcc_tty_ids[] = {
	{.compatible = "fsl,imx6sx-mcc-tty",},
	{ /* sentinel */ }
};

static struct platform_driver imxmcctty_driver = {
	.driver = {
		.name = "imx6sx-mcc-tty",
		.owner  = THIS_MODULE,
		.of_match_table = imx6sx_mcc_tty_ids,
	},
	.probe = imx_mcc_tty_probe,
	.remove = imx_mcc_tty_remove,
};

/*!
 * Initialise the imxmcctty_driver.
 *
 * @return  The function always returns 0.
 */

static int __init imxmcctty_init(void)
{
	if (platform_driver_register(&imxmcctty_driver) != 0)
		return -ENODEV;

	printk(KERN_INFO "IMX MCC TTY driver module loaded\n");
	return 0;
}

static void __exit imxmcctty_exit(void)
{
	/* Unregister the device structure */
	platform_driver_unregister(&imxmcctty_driver);
}

module_init(imxmcctty_init);
module_exit(imxmcctty_exit);

MODULE_AUTHOR("Seco S.r.L.");
MODULE_DESCRIPTION("IMX M4 UDOONEO, BASED ON MCC TTY");
MODULE_LICENSE("GPL");

