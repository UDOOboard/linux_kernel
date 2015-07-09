#ifndef __LINUX_I2C_ECONTROLLER_H
#define __LINUX_I2C_ECONTROLLER_H

#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>

/* linux/i2c/econtroller.h */

#define ECTRL_IOCTL_REG_READ	 	_IOR('o', 1, struct ectrl_reg)
#define ECTRL_IOCTL_REG_WRITE		_IOWR('o', 2, struct ectrl_reg)

enum ECTRL_EVENTS {
	EVNT_PWR_BTN, EVNT_RST_BTN, EVNT_SLEEP, 
	EVNT_BATTERY, EVNT_LID, EVNT_WAKE
};

enum ECTRL_BOOTDEV_ID {
	BOOT_USDHC4  = (u8)0,
	BOOT_USBHC1  = (u8)1,
	BOOT_EMMC    = (u8)2,
	BOOT_SPI     = (u8)3,
};

struct ectrl_bootdev {
	enum ECTRL_BOOTDEV_ID   id;
	char                    *label;
};

struct ectrl_platform_data {
	void            	(*task_pre_halt_signal) (void);
	void            	(*task_post_halt_signal) (void);
	enum ECTRL_EVENTS	*event_list;
	int                     nr_event;
	struct ectrl_bootdev    *available_bootdev;
	int                     nr_bootdev;
	int			irq;
	unsigned long		irq_flags;
};

struct data_list {
	u16 data;
	struct list_head list;
};

enum {
	STOP_OP = (unsigned short int)0, 	// stop all operations
	R_OP    = (unsigned short int)1,	// read register operation
	W_OP    = (unsigned short int)2,	// write register operation
	RVE_OP  = (unsigned short int)3,	// read vector's element operation
	WVE_OP  = (unsigned short int)4,	// write vector's element operation 
};

struct ectrl_reg {
	u16 addr;
	u16 data;
};	

struct ectrl_reg_rw {
	unsigned short int op;
	struct ectrl_reg reg;
};

#endif
