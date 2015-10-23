#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>



#include <linux/i2c/ectrl.h>
#include <dt-bindings/seco/ectrl.h>  


#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>


#define WDT_CTRL_REG                   0x00
#define GINO_TASK_REG                  0x01
#define DVI2LVDS_FLAGS_REG             0x02
#define DATA_REG                       0x03
#define INDEX_REG                      0x04
#define WDT_DELAY_REG                  0x05
#define WDT_TIMEOUT_REG                0x06
#define WDT_TIMER1_REG                 0x07
#define WDT_TIMER2_REG                 0x08
#define WDT_CONFIG_REG                 0x09
#define ADC_READING_0_REG              0x0A
#define ADC_READING_1_REG              0x0B
#define BUILDREV_REG                   0x0C
#define FW_ID_REV_REG                  0x0E
#define FLAG_REG                       0x0F 
#define ENABLE_REG                     0x10
#define STATUS_REG                     0x11

/* Flash registers */
#define ALWAYS_STATE_REG               0x01
#define WDT_F_DELAY_REG_LSB            0x0A
#define WDT_F_DELAY_REG_MSB            0x0B
#define WDT_F_TIMEOUT_REG_LSB          0x0C
#define WDT_F_TIMEOUT_REG_MSB          0x0D
#define WDT_F_CONFIG_REG_LSB           0x0E
#define WDT_F_CONFIG_REG_MSB           0x0F
#define BOOT0_REG                      0x10
#define BOOT1_REG                      0x11
#define BOOT2_REG                      0x12
#define BOOT3_REG                      0x13
#define EN_FLASH_REG_LSB               0x14
#define EN_FLASH_REG_MSB               0x15

#define SBLOCK_CMD                     0x55AA
#define HALT_CMD                       0x6101
#define REBOOT_CMD                     0x6505

#define ECTRL_POLL_PERIOD                   2  // (ms)
#define ECTRL_POLL_PERIOD_SNOOPER_PWR_BTN   50 // (ms)

#define MAX_LEN_NAME                   64
#define MAX_LEN_LABEL                  32


#define PWR_BUTTON                     0
#define RST_BUTTON                     1
#define SLEEP_SIGNAL                   2
#define BATLOW_HL_SIGNAL               3	// when BATLOW becomes HIGH - battery charge low
#define BATLOW_LH_SIGNAL               4 	// when BATLOW becomes LOW - battery full
#define LID_HL_SIGNAL                  5	// when LID becomes HIGH
#define LID_LH_SIGNAL                  6	// when LID becomes LOW
#define WAKE_SIGNAL                    7
#define PWR_BTN_4SEC                   8


#define PWR_BUTTON_MASK_REG            0x0001
#define RST_BUTTON_MASK_REG            0x0002
#define SLEEP_SIGNAL_MASK_REG          0x0004
#define BATLOW_HL_SIGNAL_MASK_REG      0x0008
#define BATLOW_LH_SIGNAL_MASK_REG      0x0010
#define LID_HL_SIGNAL_MASK_REG         0x0020
#define LID_LH_SIGNAL_MASK_REG         0x0040
#define WAKE_SIGNAL_MASK_REG           0x0080
#define PWR_BTN_4SEC_MASK_REG          0x0100


#define PWR_BUTTON_SHIFT_REG           0
#define RST_BUTTON_SHIFT_REG           1
#define SLEEP_SIGNAL_SHIFT_REG         2
#define BATLOW_HL_SIGNAL_SHIFT_REG     3
#define BATLOW_LH_SIGNAL_SHIFT_REG     4
#define LID_HL_SIGNAL_SHIFT_REG        5
#define LID_LH_SIGNAL_SHIFT_REG        6
#define WAKE_SIGNAL_SHIFT_REG          7
#define PWR_BTN_4SEC_SHIFT_REG         8


#define PWR_BUTTON_NAME                "PowerButton"
#define RST_BUTTON_NAME                "ResetButton"
#define SLEEP_SIGNAL_NAME              "Sleep"
#define BATLOW_HL_SIGNAL_NAME          "BatteryLow_H2L"
#define BATLOW_LH_SIGNAL_NAME          "BatteryLow_L2H"
#define LID_HL_SIGNAL_NAME             "Lid_H2L"
#define LID_LH_SIGNAL_NAME             "Lid_L2H"
#define WAKE_SIGNAL_NAME               "Wake"
#define PWR_BTN_4SEC_NAME              "PoworButton4Sec"


#define PWR_BUTTON_LABEL               "power_button"
#define RST_BUTTON_LABEL               "reset_button"
#define SLEEP_SIGNAL_LABEL             "sleep"
#define BATLOW_HL_SIGNAL_LABEL         "battery_low_h2l"
#define BATLOW_LH_SIGNAL_LABEL         "battery_low_l2h"
#define LID_HL_SIGNAL_LABEL            "lid_h2l"
#define LID_LH_SIGNAL_LABEL            "lid_l2h"
#define WAKE_SIGNAL_LABEL              "wake"
#define PWR_BTN_4SEC_LABEL             "pwr_btn_4sec"


#define EVNT_PWR_BTN_STATE_LABEL       "power_button"
#define EVNT_RST_BTN_STATE_LABEL       "reset_button"
#define EVNT_SLEEP_STATE_LABEL         "sleep"
#define EVNT_BATTERY_STATE_LABEL       "battery_low"
#define EVNT_LID_STATE_LABEL           "lid"
#define EVNT_WAKE_STATE_LABEL          "wake"


#define PM_STATE_ALWAYS_ON             "always_on"
#define PM_STATE_ALWAYS_OFF            "always_off"

#define ALWAYS_ON                      0x0055
#define ALWAYS_OFF                     0x0000 


#define WDT_SHIFT_ENABLE               0
#define WDT_SHIFT_EVENT                1

#define WDT_MASK_ENABLE                (0x0001 << WDT_SHIFT_ENABLE)
#define WDT_MASK_EVENT                 (0x0007 << WDT_SHIFT_EVENT)

#define WDT_CTRL_WDTOUT_RESET          (0x0001 << 8) // Auto Resettable Byte
#define WDT_CTRL_WDTOUT_TRIGGER        (0x0002 << 8) // Auto Resettable Byte

#define INPUT_DEV_NAME                 "Embedded Controller"

#define ECTRL_USE_OWN_STATE            0

/*  ID used to code the board  (read from Embedded Controller)  */
#define BOARD_928_ID                   0x28
#define BOARD_962_ID                   0x62
#define BOARD_984_ID                   0x84


#define DRV_VERSION                    "1.0"

#define ECTRL_INFO(fmt, arg...)        printk(KERN_INFO "Embedded Controller: " fmt "\n" , ## arg)
#define ECTRL_ERR(fmt, arg...)         printk(KERN_ERR "%s: " fmt "\n" , __func__ , ## arg)
#define ECTRL_DBG(fmt, arg...)         pr_debug("%s: " fmt "\n" , __func__ , ## arg)



/*  element index of the structures relative to the boards  */
#define BOARD_928                      0
#define BOARD_962                      1
#define BOARD_984                      2

static int board_nr_states [] = {
	[BOARD_928] = 6,
	[BOARD_962] = 5,
	[BOARD_984] = 5,
};


static char board_name [][30] = {
	[BOARD_928] = "Quadmo747 (Q7) - 928",
	[BOARD_962] = "uQ7 - 962",
	[BOARD_984] = "uSBC - 984",
};


/*  Maps the event to the relative index into the register 
 *  (-1 if the event is not available for the board)  
 */
static int board_reg_state_idx [][8] = {
	[BOARD_928] = {
		[EVNT_PWR_BTN] =  1,
		[EVNT_RST_BTN] =  2,
		[EVNT_SLEEP]   =  4,
		[EVNT_BATTERY] =  5,
		[EVNT_LID]     =  6,
		[EVNT_WAKE]    =  3,
	},
	[BOARD_962] = {
		[EVNT_PWR_BTN] =  1,
		[EVNT_RST_BTN] =  2,
		[EVNT_SLEEP]   =  4,
		[EVNT_BATTERY] =  5,
		[EVNT_LID]     = -1,
		[EVNT_WAKE]    =  3,
	},
	[BOARD_984] = {
		[EVNT_PWR_BTN] =  1,
		[EVNT_RST_BTN] =  2,
		[EVNT_SLEEP]   =  4,
		[EVNT_BATTERY] =  5,
		[EVNT_LID]     = -1,
		[EVNT_WAKE]    =  3,
	},
};



struct event_dev {
	unsigned short int      id;
	char                    name[MAX_LEN_NAME];
	char                    label[MAX_LEN_LABEL];
	unsigned int            index_reg;
	unsigned int            shift;
	struct proc_dir_entry   *proc_dir; 
	struct input_dev        *input;
	char                    input_name[MAX_LEN_NAME];
};


#define EVENT_DEFINE(_event)						\
	[_event] = {							\
		.id          = _event,					\
		.name        = _event ## _NAME,				\
		.label       = _event ## _LABEL,			\
		.index_reg   = _event ## _MASK_REG,			\
		.shift       = _event ## _SHIFT_REG,			\
		.proc_dir    = NULL,					\
		.input       = NULL,					\
		.input_name  = INPUT_DEV_NAME " - " _event ## _LABEL	\
	}	


#define EVENT_NO_SIGNAL_DEFINE(_event)					\
	[_event] = {							\
		.id          = _event,					\
		.name        = _event ## _NAME,				\
		.label       = "",	   				\
		.index_reg   = _event ## _MASK_REG,			\
		.shift       = _event ## _SHIFT_REG,			\
		.proc_dir    = NULL,					\
		.input       = NULL,					\
		.input_name  = ""					\
	}	


/*  Main sructure list of all fasible events  */
static struct event_dev global_event_list [] = {
	EVENT_DEFINE(PWR_BUTTON),
	EVENT_DEFINE(RST_BUTTON),
	EVENT_DEFINE(SLEEP_SIGNAL),
	EVENT_DEFINE(BATLOW_HL_SIGNAL),
	EVENT_DEFINE(BATLOW_LH_SIGNAL),
	EVENT_DEFINE(LID_HL_SIGNAL),
	EVENT_DEFINE(LID_LH_SIGNAL),
	EVENT_DEFINE(WAKE_SIGNAL),
	EVENT_NO_SIGNAL_DEFINE(PWR_BTN_4SEC),
};


struct event_state {
	enum ECTRL_EVENTS    evn;
	int                  reg_idx;
	char                 *label;
};


#define EVENT_STATE_DEFINE(_event)			\
	[_event] = {					\
		.evn      = _event,			\
		.reg_idx  = -1,				\
		.label    = _event ## _STATE_LABEL	\
	}


static struct event_state event_state_list [] = {
	EVENT_STATE_DEFINE(EVNT_PWR_BTN),  
	EVENT_STATE_DEFINE(EVNT_RST_BTN),  
	EVENT_STATE_DEFINE(EVNT_SLEEP), 
	EVENT_STATE_DEFINE(EVNT_BATTERY),
	EVENT_STATE_DEFINE(EVNT_LID),   
	EVENT_STATE_DEFINE(EVNT_WAKE),
};  



static char *PM_PWR_STATE[] = {
	PM_STATE_ALWAYS_OFF,
	PM_STATE_ALWAYS_ON,
};


enum BOOTDEV_ID {
	BOOTDEV_ID0  =  (u8)0,
	BOOTDEV_ID1  =  (u8)1,
	BOOTDEV_ID2  =  (u8)2,
	BOOTDEV_ID3  =  (u8)3,
};


struct bootdev {
        enum BOOTDEV_ID   id;
        const char        *label;
};


#define BOOT_IDX0   0
#define BOOT_IDX1   1
#define BOOT_IDX2   2
#define BOOT_IDX3   3


enum wdt_event {
	WDT_EVNT_WDOUT          = 0,
	WDT_EVNT_RESET          = 1,
	WDT_EVNT_PWRBTN1SEC     = 2,
	WDT_EVNT_PWRBTN4SEC     = 3,
	WDT_EVNT_BOOT_RECOVERY  = 4,
};


struct wdt_event_element {
	enum wdt_event id;
	char           *label;
};


#define WDT_EVENT_DEFINE(wdt_evnt, name)	\
	[wdt_evnt] = {				\
		.id = wdt_evnt,			\
		.label = name,			\
	}


static struct wdt_event_element wdt_evnt_list [] = {
	WDT_EVENT_DEFINE(WDT_EVNT_WDOUT, "wdout"),
	WDT_EVENT_DEFINE(WDT_EVNT_RESET, "reset"),
	WDT_EVENT_DEFINE(WDT_EVNT_PWRBTN1SEC, "power_button_1sec"),
	WDT_EVENT_DEFINE(WDT_EVNT_PWRBTN4SEC, "power_button_4sec"),
	WDT_EVENT_DEFINE(WDT_EVNT_BOOT_RECOVERY, "boot_recovery"),
};


/*  Main structure, used by whole driver  */
struct econtroller {
	/*  I2C client  */
	struct i2c_client             *client;
	/*  board identificator  */
	int                           board_id;
	/*  list of all feasible events for the current board  */
	struct event_dev              **events;
	int                           nr_evnt;

	struct event_state            **evn_state;
	int                           nr_evn;
	struct bootdev                *bootdev_list;
	int                           nr_bootdev;
	struct wdt_event_element      *wdt_event_list;
	int                           nr_wdt_event;
	void 		              (*task_pre_halt_signal) (void);
	void                          (*task_post_halt_signal) (void);
	int		              irq;
	unsigned int                  irq_flags;
	char                          *phys;
	struct delayed_work           work;
	struct delayed_work           work_snooper_pwr_btn;
	unsigned long                 poll_period;
	unsigned long                 poll_period_snooper_pwr_btn;
	unsigned long                 orig_jiffies; //used only for the power button snooping
	struct mutex                  fs_lock;
};


struct ectrl_proc_event {
	struct econtroller  *ectrl;
	int                 event;
	int                 is_true_event : 1;
};


struct ectrl_proc_event_state {
	struct econtroller  *ectrl;
	int                 state_id;
};


struct ectrl_proc_boot {
	struct econtroller   *ectrl;
	int                  idx;
};


struct snooper_work {
	struct econtroller   *ectrl;
	unsigned long        orig_jiffies;
};


static struct econtroller *ectrl;

static int PowerState;
static int StateValidate;


static struct ectrl_reg_rw reg_halt[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,

			},
	},
	{
		.op   = WVE_OP,
		.reg  = { 
				.addr = HALT_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_reboot[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = REBOOT_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_en_flash[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = EN_FLASH_REG_LSB,
				.data = 0x0,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = EN_FLASH_REG_MSB,
				.data = 0x0,
			},
	},

	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_boot_idx0[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = BOOT0_REG,
				.data = 0x0,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_boot_idx1[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = BOOT1_REG,
				.data = 0x0,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_boot_idx2[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = BOOT2_REG,
				.data = 0x0,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_boot_idx3[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = BOOT3_REG,
				.data = 0x0,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_wdt_delay[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = WDT_F_DELAY_REG_LSB,
				.data = 0x0,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = WDT_F_DELAY_REG_MSB,
				.data = 0x0,
			},
	},

	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_wdt_timeout[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = WDT_F_TIMEOUT_REG_LSB,
				.data = 0x0,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = WDT_F_TIMEOUT_REG_MSB,
				.data = 0x0,
			},
	},

	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_wdt_config[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = WDT_F_CONFIG_REG_LSB,
				.data = 0x0,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = WDT_F_CONFIG_REG_MSB,
				.data = 0x0,
			},
	},

	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};

static struct ectrl_reg_rw reg_pm_always_state[] = {
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = SBLOCK_CMD,
				.data = 0x0001,
			},
	},
	{
		.op   = WVE_OP,
		.reg  = {
				.addr = ALWAYS_STATE_REG,
				.data = 0x0,
			},
	},
	{
		.op   = STOP_OP,
		.reg  = {
				.addr = 0,
				.data = 0,
			},
	},
};



/* --------------------------------------------------------------------------
                                 BASIC FUNCTIONS
   -------------------------------------------------------------------------- */


static inline u16 ectrl_read_data (struct i2c_client *client, u16 addr) {
	s32 data;
	u16 val;
	data = i2c_smbus_read_word_data(client, (u8)addr);
	if (data < 0) {
		dev_err(&client->dev, "i2c io (read) error: %d\n", data);
		return data;
	}
	val = (u16)data;
	dev_dbg(&client->dev, "data: 0x%x, val: 0x%x\n", data, val);
	return val;
}


static inline u16 ectrl_write_data (struct i2c_client *client, u16 addr, u16 data) {
	return i2c_smbus_write_word_data(client, (u8)addr, data);
}


static inline u16 ectrl_read_vector_element (struct i2c_client *client, u16 addr) {
	int retval = ectrl_write_data (client, INDEX_REG, addr);
	if (!(retval < 0)) {
		retval = ectrl_read_data (client, DATA_REG);
	}
	return (u16)retval;
}


static inline u16 ectrl_write_vector_element (struct i2c_client *client, u16 addr, u16 data) {
	int retval = ectrl_write_data (client, DATA_REG, data);
	if (!(retval < 0)) {
		retval = ectrl_write_data (client, INDEX_REG, addr);
	}
	return (u16)retval;
}


static int ectrl_mem_single_op (struct i2c_client *client, struct ectrl_reg_rw *reg_rw) {
	int retval = 0;
	
	u16 val;
	switch (reg_rw->op) {
		case R_OP:
			val = ectrl_read_data (client, reg_rw->reg.addr);
			if (val >= 0) {
				dev_dbg(&client->dev, "read data done: 0x%x\n", val);
				reg_rw->reg.data = val;	
				retval = 1;
			} else {
				dev_err(&client->dev, "read data failed: 0x%x\n", val);
				retval = -1;
			}
			break;
		case W_OP:
			val = ectrl_write_data (client, reg_rw->reg.addr, reg_rw->reg.data);
			if (val >= 0) {
				dev_dbg(&client->dev, "write data done: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = 0;
			} else {
				dev_err(&client->dev, "write data failed: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = -1;
			}
			break;
		case RVE_OP:
			val = ectrl_read_vector_element (client, reg_rw->reg.addr);
			if (val >= 0) {
				dev_dbg(&client->dev, "read vector element done: 0x%x", val);
				reg_rw->reg.data = val;
				retval = 1;
			} else {
				dev_err(&client->dev, "read vector's element failed: 0x%x\n", val);
				retval = -1;
			}
			break;
		case WVE_OP:
			val = ectrl_write_vector_element (client, reg_rw->reg.addr, reg_rw->reg.data);
			if (val >= 0) {
				dev_dbg(&client->dev, "write vector's element done: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = 1;		
			} else {
				dev_err(&client->dev, "write vector's element failed: addr 0x%x  data 0x%x\n", reg_rw->reg.addr, reg_rw->reg.data);
				retval = -1;
			}
			mdelay (75);  // needed to access to the flash of the embedded controller
			break;
		default:
			dev_dbg(&client->dev, "invalid operation!\n");
	 		retval = -1;
	}
	return retval;
}


static int ectrl_mem_op (struct i2c_client *client, struct ectrl_reg_rw regs[], struct data_list *data_l) {
	struct ectrl_reg_rw *next = regs;
	struct data_list *tmp;
	int retval = 0;
	int val;
	for (; next->op != STOP_OP ; next++) {
		val = ectrl_mem_single_op (client, next);
		if (val < 0)
			return val;
		if (val > 0) {
			if (data_l != NULL) {
				tmp = kzalloc (sizeof (struct data_list), GFP_KERNEL);
				tmp->data = next->reg.data;
				list_add (&(tmp->list), &(data_l->list));
			}
		}
		retval += val;
	}
	return retval;
}


static int get_reg_rw (struct ectrl_reg_rw *reg_rw, struct ectrl_reg reg, int op) {
	reg_rw->op = op;
	reg_rw->reg.data = reg.data;
	reg_rw->reg.addr = reg.addr;
	return 0;
}



/* --------------------------------------------------------------------------
                              ADVANCED FUNCTIONS
   -------------------------------------------------------------------------- */

			/* DATA FUNCTIONS */

#define getBoardID(client)    (ectrl_read_data ((client), FW_ID_REV_REG) & 0xFF00) >> 8



			/* EVENT FUNCTIONS */

#define getStatusReg(client)    ectrl_read_data ((client), STATUS_REG)
#define getFlagReg(client)      ectrl_read_data ((client), FLAG_REG)
#define getEnableReg(client)    ectrl_read_data ((client), ENABLE_REG)
#define getEnFlashReg(client)   (ectrl_read_vector_element ((client), EN_FLASH_REG_MSB) << 8) | \
				ectrl_read_vector_element ((client), EN_FLASH_REG_LSB)  

#define setFlagReg(client, regv)      ectrl_write_data ((client), FLAG_REG, (regv))
#define setEnableReg(client, regv)    ectrl_write_data ((client), ENABLE_REG, (regv))
#define setEnFlashReg(client, regv)   reg_en_flash[1].reg.data = (regv) & 0x00FF;  \
		   		      reg_en_flash[3].reg.data = ((regv) >> 8) & 0x00FF;  \
                                      ectrl_mem_op ((client), reg_en_flash, NULL) 


static inline int getStatus (struct i2c_client *client, int reg_idx) {
	int reg = (int)getStatusReg(client);
	return ((reg >> (reg_idx)) & 0x1);
}

	
static inline int getFlag (struct i2c_client *client, int event) {
	int reg = (int)getFlagReg(client);
	return (reg & global_event_list[event].index_reg) >> global_event_list[event].shift;
}


static inline int getEnable (struct i2c_client *client, int event) {
	int reg = (int)getEnableReg(client);
	return (reg & global_event_list[event].index_reg) >> global_event_list[event].shift;
}


static inline int getEnFlash (struct i2c_client *client, int event) {
	int reg = (int)getEnFlashReg(client);
	return (reg & global_event_list[event].index_reg) >> global_event_list[event].shift;
}


static inline void setFlag (struct i2c_client *client, int event, int set) {
	int reg = (int)getFlagReg(client);
	reg = (set) ? reg | (1u << global_event_list[event].shift) : reg & ~(1u << global_event_list[event].shift);
	setFlagReg(client, reg);	
}


static inline void setEnable (struct i2c_client *client, int event, int set) {
	int reg = (int)getEnableReg(client);
	reg = (set) ? reg | (1u << global_event_list[event].shift) : reg & ~(1u << global_event_list[event].shift);
	setEnableReg(client, reg);	
}


static inline void setEnFlash (struct i2c_client *client, int event, int set) {
	int reg = (int)getEnFlashReg(client);
	reg = (set) ? reg | (1u << global_event_list[event].shift) : reg & ~(1u << global_event_list[event].shift);
	setEnFlashReg(client, reg);	
}


			/* POWER MANAGEMENT FUNCTIONS */

#define PMgetAlwaysState(client)             ectrl_read_vector_element ((client), ALWAYS_STATE_REG)
#define PMsetAlwaysState(client, state)      reg_pm_always_state[1].reg.data = (state); \
                                             ectrl_mem_op ((client), reg_pm_always_state, NULL)


			/* BOOT FUNCTIONS */

#define BOOT_SEQUENCE_LENGTH         4
#define IS_VALID_BOOT_SEQ_IDX(idx)   ((idx) >= 0 && (idx) < BOOT_SEQUENCE_LENGTH) ? 1 : 0

#define getBoot0Reg(client)        ectrl_read_vector_element ((client), BOOT0_REG)
#define getBoot1Reg(client)        ectrl_read_vector_element ((client), BOOT1_REG)
#define getBoot2Reg(client)        ectrl_read_vector_element ((client), BOOT2_REG)
#define getBoot3Reg(client)        ectrl_read_vector_element ((client), BOOT3_REG)

#define setBoot0Reg(client, regv)   reg_boot_idx0[1].reg.data = (regv); \
 				    ectrl_mem_op ((client), reg_boot_idx0, NULL)
#define setBoot1Reg(client, regv)   reg_boot_idx1[1].reg.data = (regv); \
 				    ectrl_mem_op ((client), reg_boot_idx1, NULL)
#define setBoot2Reg(client, regv)   reg_boot_idx2[1].reg.data = (regv); \
 				    ectrl_mem_op ((client), reg_boot_idx2, NULL)
#define setBoot3Reg(client, regv)   reg_boot_idx3[1].reg.data = (regv); \
 				    ectrl_mem_op ((client), reg_boot_idx3, NULL)



static int isAvaildableBootdev (struct econtroller *ectrl, enum BOOTDEV_ID id) {
	int i, isValid = -1;
	for (i = 0 ; i < ectrl->nr_bootdev ; i++) {
		if (ectrl->bootdev_list[i].id == id) {
			isValid = i;
			break;
		}
	}
	return isValid;
}


static int getBootDev (struct econtroller *ectrl, int boot_idx) {
	int reg_val;
	switch (boot_idx) {
		case BOOT_IDX0:
			reg_val = getBoot0Reg(ectrl->client);
			break;
		case BOOT_IDX1:
			reg_val = getBoot1Reg(ectrl->client);
			break;
		case BOOT_IDX2:
			reg_val = getBoot2Reg(ectrl->client);
			break;
		case BOOT_IDX3:
			reg_val = getBoot3Reg(ectrl->client);
			break;
		default:
			reg_val = -1;
	}
	return reg_val;
}


static int setBootDev (struct econtroller *ectrl, int boot_idx, enum BOOTDEV_ID bootdev_id) {
	if (isAvaildableBootdev (ectrl, bootdev_id) < 0)
		return -1;
	else {
		switch (boot_idx) {
			case BOOT_IDX0:
				setBoot0Reg(ectrl->client, (int)bootdev_id);
				break;
			case BOOT_IDX1:
				setBoot1Reg(ectrl->client, (int)bootdev_id);
				break;
			case BOOT_IDX2:
				setBoot2Reg(ectrl->client, (int)bootdev_id);
				break;
			case BOOT_IDX3:
				setBoot3Reg(ectrl->client, (int)bootdev_id);
				break;
			default:
				return -1;
		}
	}
	return boot_idx;	
}



			/* WATCHDOG FUNCTIONS */


#define WDTgetEnableReg(client)    (ectrl_read_data ((client), WDT_CONFIG_REG) & \
					WDT_MASK_ENABLE) >> WDT_SHIFT_ENABLE
#define WDTgetEventReg(client)     (ectrl_read_data ((client), WDT_CONFIG_REG) & \
					WDT_MASK_EVENT) >> WDT_SHIFT_EVENT
#define WDTgetDelayReg(client)     (ectrl_read_data ((client), WDT_DELAY_REG))
#define WDTgetTimeoutReg(client)   (ectrl_read_data ((client), WDT_TIMEOUT_REG))
#define WDTgetTimer1Reg(client)    (ectrl_read_data ((client), WDT_TIMER1_REG))
#define WDTgetTimer2Reg(client)    (ectrl_read_data ((client), WDT_TIMER2_REG))


#define WDT_F_getDelayReg(client)    (ectrl_read_vector_element ((client), WDT_F_DELAY_REG_MSB) << 8) |  \
				     ectrl_read_vector_element ((client), WDT_F_DELAY_REG_LSB)
#define WDT_F_getTimeoutReg(client)  (ectrl_read_vector_element ((client), WDT_F_TIMEOUT_REG_MSB) << 8) |  \
				     ectrl_read_vector_element ((client), WDT_F_TIMEOUT_REG_LSB)
#define WDT_F_getEnableReg(client)   (ectrl_read_vector_element ((client), WDT_F_CONFIG_REG_LSB) & \
					WDT_MASK_ENABLE) >> WDT_SHIFT_ENABLE
#define WDT_F_getEventReg(client)    (ectrl_read_vector_element ((client), WDT_F_CONFIG_REG_LSB) & \
					WDT_MASK_EVENT) >> WDT_SHIFT_EVENT


#define WDTsetEnableReg(client, en)         ectrl_write_data ((client), WDT_CONFIG_REG, !(en) ? \
				ectrl_read_data ((client), WDT_CONFIG_REG) & ~WDT_MASK_ENABLE : \
				ectrl_read_data ((client), WDT_CONFIG_REG) | WDT_MASK_ENABLE)
#define WDTsetEventReg(client, evn)         ectrl_write_data ((client), WDT_CONFIG_REG, \
				(ectrl_read_data ((client), WDT_CONFIG_REG) \
				& ~WDT_MASK_EVENT) | (((evn) << WDT_SHIFT_EVENT) & WDT_MASK_EVENT))
#define WDTsetDelayReg(client, regval)      ectrl_write_data ((client), WDT_DELAY_REG, (u16)(regval))
#define WDTsetTimeoutReg(client, regval)    ectrl_write_data ((client), WDT_TIMEOUT_REG, (u16)(regval))

#define WDTrefresh(client)         ectrl_write_data ((client), WDT_CTRL_REG, WDT_CTRL_WDTOUT_TRIGGER)
#define WDTwdtout_reset(client)    ectrl_write_data ((client), WDT_CTRL_REG, WDT_CTRL_WDTOUT_RESET)

#define WDT_F_setDelayReg(client, regval)     reg_wdt_delay[1].reg.data = (regval) & 0x00FF; \
					      reg_wdt_delay[3].reg.data = ((regval) >> 8) & 0x00FF; \
                                              ectrl_mem_op ((client), reg_wdt_delay, NULL)              
#define WDT_F_setTimeoutReg(client, regval)   reg_wdt_timeout[1].reg.data = (regval) & 0x00FF; \
					      reg_wdt_timeout[3].reg.data = ((regval) >> 8) & 0x00FF; \
                                              ectrl_mem_op ((client), reg_wdt_timeout, NULL)
#define WDT_F_setEnableReg(client, regval)    reg_wdt_config[1].reg.data = (regval);  \
                                              ectrl_mem_op ((client), reg_wdt_config, NULL)
#define WDT_F_setEventReg(client, regval)     reg_wdt_config[1].reg.data = (regval) & 0x00FF;  \
					      reg_wdt_config[3].reg.data = ((regval) >> 8) & 0x00FF; \
                                              ectrl_mem_op ((client), reg_wdt_config, NULL)


static inline enum wdt_event WDTgetEvent (struct i2c_client *client) {
	return (enum wdt_event)WDTgetEventReg(client);
}


static inline int WDTsetEvent (struct i2c_client *client, enum wdt_event event) {
	u16 retval;
	switch (event) {
		case WDT_EVNT_WDOUT:
		case WDT_EVNT_RESET:
		case WDT_EVNT_PWRBTN1SEC:
		case WDT_EVNT_PWRBTN4SEC:
		case WDT_EVNT_BOOT_RECOVERY:
			WDTsetEventReg(client, (u16)event);
			retval = (u16)event;
			break;
		default:
			retval = -1;
	}
	return (int)retval;
}


static inline void WDT_F_setEnable (struct i2c_client *client, int en) {
	int reg = (int)(ectrl_read_vector_element ((client), WDT_F_CONFIG_REG_LSB) |
			ectrl_read_vector_element ((client), WDT_F_CONFIG_REG_MSB) << 8);
	reg = (en) ? reg | WDT_MASK_ENABLE : reg & ~WDT_MASK_ENABLE;
	if (en == 0 || en == 1)
		WDT_F_setEnableReg (client, reg);
}


static inline int WDT_F_setEvent (struct i2c_client *client,  enum wdt_event event) {
	int reg = (int)(ectrl_read_vector_element ((client), WDT_F_CONFIG_REG_LSB) |
			ectrl_read_vector_element ((client), WDT_F_CONFIG_REG_MSB) << 8);
	u16 retval;
	switch (event) {
		case WDT_EVNT_WDOUT:
		case WDT_EVNT_RESET:
		case WDT_EVNT_PWRBTN1SEC:
		case WDT_EVNT_PWRBTN4SEC:
		case WDT_EVNT_BOOT_RECOVERY:
			retval = (u16)event;
			reg = (reg & ~WDT_MASK_EVENT) | ((retval << WDT_SHIFT_EVENT) & WDT_MASK_EVENT);
			WDT_F_setEnableReg(client, reg);
			break;
		default:
			retval = -1;
	}
	return (int)retval;

}



			/* POWER FUNCTIONS */

static int ectrl_SystemHalt (struct i2c_client *client) {
	int retval;
	retval = ectrl_mem_op (client, reg_halt, NULL);
	if (!(retval < 0)) {
		printk (KERN_INFO "Seco halt performed!\n");
	} else {
		printk (KERN_ERR "Seco halt not performed!\n");
	}
	return retval;
}


static int ectrl_SystemReboot (struct i2c_client *client) {
	int retval;
	retval = ectrl_mem_op (client, reg_reboot, NULL);
	if (!(retval < 0)) {
		printk (KERN_INFO "Seco reboot performed!\n");
	} else {
		printk (KERN_ERR "Seco reboot not performed!\n");
	}
	return retval;
}


/* --------------------------------------------------------------------------
                              FS Interface (/proc)
   -------------------------------------------------------------------------- */

static struct proc_dir_entry *ectrl_root_dir;
static struct proc_dir_entry *ectrl_events_dir;
static struct proc_dir_entry *ectrl_events_state_dir;
static struct proc_dir_entry *ectrl_pm_dir;
static struct proc_dir_entry *ectrl_pm_pwr_btn_4sec_dir;
static struct proc_dir_entry *ectrl_boot_dir;
static struct proc_dir_entry *ectrl_watchdog_dir;
static struct proc_dir_entry *ectrl_watchdog_permanent_dir;


#define ECTRL_PROC_ROOT                          "ectrl"

#define ECTRL_PROC_EVENTS                        "events"
#define ECTRL_PROC_EVENTS_STATE                  "event_state"
#define ECTRL_PROC_PM                            "power_management"
#define ECTRL_PROC_BOOT                          "boot"
#define ECTRL_PROC_WATCHDOG                      "watchdog"
#define ECTRL_PROC_WDT_PERMANENT                 "permanent"

#define ECTRL_PROC_LID                           "lid"
#define ECTRL_PROC_BATTERY                       "battery"
#define ECTRL_PROC_PWR_BTN                       "power_button"
#define ECTRL_PROC_RST_BTN                       "reset_button"
#define ECTRL_PROC_SLEEP                         "sleep"

#define ECTRL_ENTRY_STATUS                       "status"
#define ECTRL_ENTRY_ENABLE                       "enable"
#define ECTRL_ENTRY_ENFLASH                      "en_flash"

#define ECTRL_ENTRY_PM_PWR_STATE                 "power_state"
#define ECTRL_ENTRY_PM_STATE                     "available_states"
#define ECTRL_ENTRY_PM_PWR_BTN_4SEC              "pwr_btn_4sec"
#define ECTRL_ENTRY_PM_PWR_BTN_4SEC_EN           "enable"
#define ECTRL_ENTRY_PM_PWR_BTN_4SEC_ENFLASH      "en_flash"

#define ECTRL_ENTRY_DEVBOOT0                     "devboot_1th"
#define ECTRL_ENTRY_DEVBOOT1                     "devboot_2th"
#define ECTRL_ENTRY_DEVBOOT2                     "devboot_3th"
#define ECTRL_ENTRY_RECOVERY_BOOT                "recovery_boot"
#define ECTRL_ENTRY_BOOTDEV_LIST                 "available_bootdevices"
#define ECTRL_ENTRY_BOOT_SEQUENCE                "boot_sequence"

#define ECTRL_ENTRY_WDT_ENABLE                   "enable"
#define ECTRL_ENTRY_WDT_DELAY                    "delay_sec"
#define ECTRL_ENTRY_WDT_TIMEOUT                  "timeout_sec"
#define ECTRL_ENTRY_WDT_TIMER1                   "timer_delay"
#define ECTRL_ENTRY_WDT_TIMER2                   "timer_wd"
#define ECTRL_ENTRY_WDT_EVENT                    "event"
#define ECTRL_ENTRY_WDT_AVAL_EVN                 "available_events"
#define ECTRL_ENTRY_WDT_STATE                    "state"
#define ECTRL_ENTRY_WDT_REFRESH                  "refresh"
#define ECTRL_ENTRY_WDT_RESTORE                  "restore"
#define ECTRL_ENTRY_WDT_WDTOUT_RESET             "wdtout_reset"

#define INPUT_BUF_SIZE   256


/* -------------------------- PROC STATE FILEs OPT (r only) -------------------------- */

static int ectrl_proc_state_show (struct seq_file *seq, void *offset) {
	struct ectrl_proc_event *proc = (struct ectrl_proc_event *)seq->private;
	int status;
	if (!proc) 
		return -EINVAL;
	mutex_lock (&proc->ectrl->fs_lock);
	status = getStatus (proc->ectrl->client, proc->event);
	mutex_unlock (&proc->ectrl->fs_lock);
        seq_printf(seq, "%s\n", !status ? "active" : "inactive");
        return 0;
}


static int ectrl_proc_state_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_state_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_state_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_state_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC STATUS FILEs OPT (r only) -------------------------- */

static int ectrl_proc_status_show (struct seq_file *seq, void *offset) {
	struct ectrl_proc_event_state *proc = (struct ectrl_proc_event_state *)seq->private;
	int status;
	if (!proc) 
		return -EINVAL;
	mutex_lock (&proc->ectrl->fs_lock);
	status = getStatus (proc->ectrl->client, 
		proc->ectrl->evn_state[proc->state_id]->reg_idx);
	mutex_unlock (&proc->ectrl->fs_lock);
        seq_printf(seq, "%s\n", !status ? "active" : "inactive");
        return 0;
}


static int ectrl_proc_status_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_status_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_status_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_status_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC ENABLE FILEs OPT (r/w) -------------------------- */

static int ectrl_proc_enable_show (struct seq_file *seq, void *offset) {
	struct ectrl_proc_event *proc = (struct ectrl_proc_event *)seq->private;
	int enable;
	if (!proc) 
		return -EINVAL;

	mutex_lock (&proc->ectrl->fs_lock);
	enable = getEnable (proc->ectrl->client, proc->event);
	mutex_unlock (&proc->ectrl->fs_lock);
        seq_printf(seq, "%s\n", enable ? "enable" : "disable");
        return 0;
}


static int ectrl_proc_enable_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	int err_conv;
	long en;
	struct ectrl_proc_event *proc = (struct ectrl_proc_event *)
		((struct seq_file *)file->private_data)->private;
	if (!proc)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol (input, 0, &en);
	
	if (err_conv == 0) {
		if (en == 0 || en == 1) {
			mutex_lock (&proc->ectrl->fs_lock);
			setEnable (proc->ectrl->client, proc->event, en);
			mutex_unlock (&proc->ectrl->fs_lock);
		} else 
			return -EINVAL;
	}
	return count;
}


static int ectrl_proc_enable_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_enable_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_enable_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_enable_open_fs,
        .read = seq_read,
	.write = ectrl_proc_enable_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC ENFLASH FILEs OPT (r/w) -------------------------- */

static int ectrl_proc_enflash_show (struct seq_file *seq, void *offset) {
	struct ectrl_proc_event *proc = (struct ectrl_proc_event *)seq->private;
	int enflash;
	if (!proc) 
		return -EINVAL;

	mutex_lock (&proc->ectrl->fs_lock);
	enflash = getEnFlash (proc->ectrl->client, proc->event);
	mutex_unlock (&proc->ectrl->fs_lock);
        seq_printf(seq, "%s\n", enflash ? "enable" : "disable");
        return 0;
}


static int ectrl_proc_enflash_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	char input[INPUT_BUF_SIZE];
	int err_conv;
	long en;
	struct ectrl_proc_event *proc = (struct ectrl_proc_event *)
		((struct seq_file *)file->private_data)->private;
		
	if (!proc)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol (input, 0, &en);
	
	if (err_conv == 0) {
		if (en == 0 || en == 1) {
			mutex_lock (&proc->ectrl->fs_lock);
			setEnFlash (proc->ectrl->client, proc->event, en);
			mutex_unlock (&proc->ectrl->fs_lock);
		} else 
			return -EINVAL;
	}
	return count;

}


static int ectrl_proc_enflash_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_enflash_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_enflash_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_enflash_open_fs,
        .read = seq_read,
	.write = ectrl_proc_enflash_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC PM PWM_STATE FILEs OPT (r/w) -------------------------- */

static int ectrl_proc_pm_pwr_state_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	u16 always;
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	always = PMgetAlwaysState (ectrl->client);
	mutex_unlock (&ectrl->fs_lock);
        seq_printf(seq, "%s\n", always == (u16)ALWAYS_ON ? "always_on" : "always_off");
        return 0;
}


static int ectrl_proc_pm_pwr_state_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	char input[INPUT_BUF_SIZE];
	int err_conv;
	long en;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;
	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol (input, 0, &en);
	
	if (err_conv == 0) {
		if (en == 1) {
			mutex_lock (&ectrl->fs_lock);
			PMsetAlwaysState (ectrl->client, ALWAYS_ON);
			mutex_unlock (&ectrl->fs_lock);
		} else if (en == 0) {
			mutex_lock (&ectrl->fs_lock);
			PMsetAlwaysState (ectrl->client, ALWAYS_OFF);
			mutex_unlock (&ectrl->fs_lock);
		} else 
			return -EINVAL;
	}
	return count;

}


static int ectrl_proc_pm_pwr_state_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_pm_pwr_state_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_pm_pwr_state_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_pm_pwr_state_open_fs,
        .read = seq_read,
	.write = ectrl_proc_pm_pwr_state_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC AVAILABLE PM STATE FILE OPT (r only) -------------------------- */

static int ectrl_proc_available_pm_state_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	int i;

	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	for (i = 0 ; i < ARRAY_SIZE (PM_PWR_STATE) ; i++) {
		seq_printf (seq, "%d: %s\n", i, PM_PWR_STATE[i]);
	}
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_available_pm_state_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_available_pm_state_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_available_pm_state_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_available_pm_state_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC BOOT FILEs OPT (r/w) -------------------------- */

static int ectrl_proc_bootdev_show (struct seq_file *seq, void *offset) {
	struct ectrl_proc_boot *proc = (struct ectrl_proc_boot *)seq->private;
	enum BOOTDEV_ID bootdev;
	int index;
	if (!proc) 
		return -EINVAL;

	if (!IS_VALID_BOOT_SEQ_IDX(proc->idx))
		return -EINVAL;

	mutex_lock (&proc->ectrl->fs_lock);
	bootdev = (enum BOOTDEV_ID)getBootDev (proc->ectrl, proc->idx);
	index = isAvaildableBootdev (proc->ectrl, bootdev);
	mutex_unlock (&proc->ectrl->fs_lock);
        seq_printf (seq, "%d %s\n", (int)bootdev, index < 0 ? "<no label>" : ectrl->bootdev_list[index].label);
        return 0;
}


static int ectrl_proc_bootdev_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	int err_conv;
	long  id;
	
	struct ectrl_proc_boot *proc = (struct ectrl_proc_boot *)
		((struct seq_file *)file->private_data)->private;
	
	if (!proc)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &id);

	if (err_conv == 0) {
		mutex_lock (&proc->ectrl->fs_lock);
		if (setBootDev (proc->ectrl, proc->idx, (enum BOOTDEV_ID)id) < 0) {
			mutex_unlock (&proc->ectrl->fs_lock);
			return -EINVAL;
		}
		mutex_unlock (&proc->ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_bootdev_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_bootdev_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_bootdev_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_bootdev_open_fs,
        .read = seq_read,
	.write = ectrl_proc_bootdev_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC BOOT_DEV_LIST FILEs OPT (r only) -------------------------- */

static int ectrl_proc_bootdev_list_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	int i;
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	for (i = 0 ; i < ectrl->nr_bootdev ; i++) {
		seq_printf (seq, "%d %s\n",  
				ectrl->bootdev_list[i].id, ectrl->bootdev_list[i].label);
	}
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}

static int ectrl_proc_bootdev_list_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_bootdev_list_show, PDE_DATA(inode));
	// PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_bootdev_list_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_bootdev_list_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC BOOT_SEQUENCE FILE OPT (r only) -------------------------- */

static int ectrl_proc_bootseq_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	int i;
	enum BOOTDEV_ID bootdev;
	int index;
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	for (i = 0 ; i < BOOT_SEQUENCE_LENGTH ; i++) {
		bootdev = (enum BOOTDEV_ID)getBootDev (ectrl, i);
		index = isAvaildableBootdev (ectrl, bootdev);
		if (index >= 0) {
			if ( i != (BOOT_SEQUENCE_LENGTH -1)) {
				seq_printf (seq, "BootDev%d: %d %s\n", i,  
			   	   ectrl->bootdev_list[index].id, ectrl->bootdev_list[index].label);
			} else {
				seq_printf (seq, "Recovery: %d %s\n", 
				   ectrl->bootdev_list[index].id, ectrl->bootdev_list[index].label);
			}
		}
	}
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}

static int ectrl_proc_bootseq_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_bootseq_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_bootseq_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_bootseq_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT ENABLE FILEs OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_enable_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	int enable;
	
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	enable = WDTgetEnableReg (ectrl->client);
	mutex_unlock (&ectrl->fs_lock);
        seq_printf(seq, "%s\n", enable ? "enable" : "disable");
        return 0;
}


static int ectrl_proc_wdt_enable_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	int err_conv;
	long en;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol (input, 0, &en);
	
	if (err_conv == 0) {
		if (en == 0 || en == 1) {
			mutex_lock (&ectrl->fs_lock);
			WDTsetEnableReg (ectrl->client, en);
			mutex_unlock (&ectrl->fs_lock);
		} else 
			return -EINVAL;
	}
	return count;
}


static int ectrl_proc_wdt_enable_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_enable_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_enable_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_enable_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_enable_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT EVENT FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_event_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	enum wdt_event event;
	int index;
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	event = WDTgetEvent (ectrl->client);
	mutex_unlock (&ectrl->fs_lock);
	index = (int)event;
        seq_printf(seq, "%d %s\n", ectrl->wdt_event_list[index].id, ectrl->wdt_event_list[index].label);
        return 0;
}


static int ectrl_proc_wdt_event_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long id;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &id);

	if (err_conv == 0) {
		mutex_lock (&ectrl->fs_lock);
		if (WDTsetEvent (ectrl->client, (enum wdt_event)id) < 0) {
			mutex_unlock (&ectrl->fs_lock);
			return -EINVAL;
		}
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_event_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_event_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_event_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_event_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_event_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC AVAILABLE WDT EVENTS FILE OPT (r only) -------------------------- */

static int ectrl_proc_avalaible_wdt_event_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	int i;

	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	for (i = 0 ; i < ectrl->nr_wdt_event ; i++) {
		seq_printf (seq, "%d: %s\n",  ectrl->wdt_event_list[i].id, 
					ectrl->wdt_event_list[i].label);
	}
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_available_wdt_event_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_avalaible_wdt_event_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_avalaible_wdt_event_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_available_wdt_event_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT DELAY FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_delay_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
        seq_printf(seq, "%d\n", WDTgetDelayReg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_delay_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long delay;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &delay);

	if (err_conv == 0) {
		if ((u16)delay < 1)
			return -EINVAL;
		mutex_lock (&ectrl->fs_lock);
		WDTsetDelayReg (ectrl->client, (u16)delay);
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_delay_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_delay_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_delay_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_delay_open_fs,
        .read = seq_read,
		.write = ectrl_proc_wdt_delay_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT TIMEOUT FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_timeout_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

	mutex_unlock (&ectrl->fs_lock);
        seq_printf(seq, "%d\n", WDTgetTimeoutReg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_timeout_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long timeout;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &timeout);

	if (err_conv == 0) {
		if ((u16)timeout < 1)
			return -EINVAL; 
		mutex_lock (&ectrl->fs_lock);
		WDTsetTimeoutReg (ectrl->client, (u16)timeout);
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_timeout_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_timeout_show,  PDE_DATA(inode));}


static const struct file_operations ectrl_proc_wdt_timeout_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_timeout_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_timeout_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT REFRESH FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_refresh_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

        return 0;
}


static int ectrl_proc_wdt_refresh_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long in;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &in);

	if (err_conv == 0) {
		if ((u16)in != 1)
			return -EINVAL; 
		mutex_lock (&ectrl->fs_lock);
		WDTrefresh (ectrl->client);
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_refresh_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_refresh_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_refresh_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_refresh_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_refresh_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT RESTORE FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_restore_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

        return 0;
}


static int ectrl_proc_wdt_restore_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long in;
	int  err_conv;
	u16 f_delay, f_timeout;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &in);

	if (err_conv == 0) {
		if ((u16)in != 1)
			return -EINVAL; 
		mutex_lock (&ectrl->fs_lock);
		f_delay   = (u16)WDT_F_getDelayReg (ectrl->client);
		f_timeout = (u16)WDT_F_getTimeoutReg (ectrl->client);

		WDTsetDelayReg (ectrl->client, f_delay);
		WDTsetTimeoutReg (ectrl->client, f_timeout);	
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_restore_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_restore_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_restore_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_restore_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_restore_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDTOUT RESET FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_wdtout_reset_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

        return 0;
}


static int ectrl_proc_wdt_wdtout_reset_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long in;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &in);

	if (err_conv == 0) {
		if ((u16)in != 1)
			return -EINVAL;
		mutex_lock (&ectrl->fs_lock);
		WDTwdtout_reset (ectrl->client); 
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_wdtout_reset_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_wdtout_reset_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_wdtout_reset_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_wdtout_reset_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_wdtout_reset_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT TIMER1 FILE OPT (r only) -------------------------- */

static int ectrl_proc_wdt_timer1_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;

	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
        seq_printf(seq, "%d\n", WDTgetTimer1Reg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_timer1_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_timer1_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_timer1_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_timer1_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT TIMER2 FILE OPT (r only) -------------------------- */

static int ectrl_proc_wdt_timer2_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;

	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
        seq_printf(seq, "%d\n", WDTgetTimer2Reg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_timer2_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_timer2_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_timer2_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_timer2_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT STATE FILE OPT (r only) -------------------------- */

static int ectrl_proc_wdt_state_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	enum wdt_event event;
	int enable, index;

	if (!ectrl) 
		return -EINVAL;
	
	mutex_lock (&ectrl->fs_lock);
	enable = WDTgetEnableReg (ectrl->client);

	event = WDTgetEvent (ectrl->client);
	index = (int)event;

        seq_printf(seq, "  %20s : %s\n  %20s : %d - %s\n  %20s : %d\n  %20s : %d\n  %20s : %d\n  %20s : %d\n",
				ECTRL_ENTRY_WDT_ENABLE,   enable ? "enable" : "disable",
				ECTRL_ENTRY_WDT_EVENT,    ectrl->wdt_event_list[index].id, 
							  ectrl->wdt_event_list[index].label,
				ECTRL_ENTRY_WDT_DELAY,    WDTgetDelayReg (ectrl->client),
				ECTRL_ENTRY_WDT_TIMEOUT,  WDTgetTimeoutReg (ectrl->client),
				ECTRL_ENTRY_WDT_TIMER1,   WDTgetTimer1Reg (ectrl->client),
				ECTRL_ENTRY_WDT_TIMER2,   WDTgetTimer2Reg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_state_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_state_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_state_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_state_open_fs,
        .read = seq_read,
	.write = NULL,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT FLASH ENABLE FILEs OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_f_enable_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	int enable;
	
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	enable = WDT_F_getEnableReg (ectrl->client);
	mutex_unlock (&ectrl->fs_lock);
        seq_printf(seq, "%s\n", enable ? "enable" : "disable");
        return 0;
}


static int ectrl_proc_wdt_f_enable_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	int err_conv;
	long en;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol (input, 0, &en);
	
	if (err_conv == 0) {
		if (en == 0 || en == 1) {
			mutex_lock (&ectrl->fs_lock);
			WDT_F_setEnable (ectrl->client, en);
			mutex_unlock (&ectrl->fs_lock);
		} else 
			return -EINVAL;
	}
	return count;
}


static int ectrl_proc_wdt_f_enable_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_f_enable_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_f_enable_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_f_enable_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_f_enable_write,
        .llseek = seq_lseek,
        .release = single_release,
};




/* -------------------------- PROC WDT FLASH EVENT FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_f_event_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	enum wdt_event event;
	int index;
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
	event = WDT_F_getEventReg (ectrl->client);
	mutex_unlock (&ectrl->fs_lock);
	index = (int)event;
        seq_printf(seq, "%d %s\n", ectrl->wdt_event_list[index].id, ectrl->wdt_event_list[index].label);
        return 0;
}


static int ectrl_proc_wdt_f_event_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long id;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &id);

	if (err_conv == 0) {
		mutex_lock (&ectrl->fs_lock);
		if (WDT_F_setEvent (ectrl->client, (enum wdt_event)id) < 0) {
			mutex_unlock (&ectrl->fs_lock);
			return -EINVAL;
		}
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_f_event_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_f_event_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_f_event_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_f_event_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_f_event_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT FLASH DELAY FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_f_delay_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

	mutex_unlock (&ectrl->fs_lock);
        seq_printf(seq, "%d\n", WDT_F_getDelayReg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_f_delay_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long delay;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &delay);

	if (err_conv == 0) {
		if ((u16)delay < 1)
			return -EINVAL;
		mutex_lock (&ectrl->fs_lock);
		WDT_F_setDelayReg (ectrl->client, (u16)delay);
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_f_delay_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_f_delay_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_f_delay_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_f_delay_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_f_delay_write,
        .llseek = seq_lseek,
        .release = single_release,
};


/* -------------------------- PROC WDT FLASH TIMEOUT FILE OPT (r/w) -------------------------- */

static int ectrl_proc_wdt_f_timeout_show (struct seq_file *seq, void *offset) {
	struct econtroller *ectrl = (struct econtroller *)seq->private;
	
	if (!ectrl) 
		return -EINVAL;

	mutex_lock (&ectrl->fs_lock);
        seq_printf(seq, "%d\n", WDT_F_getTimeoutReg (ectrl->client));
	mutex_unlock (&ectrl->fs_lock);
        return 0;
}


static int ectrl_proc_wdt_f_timeout_write (struct file *file, const char __user *buf,
			size_t count, loff_t *pos) {
	
	char input[INPUT_BUF_SIZE];
	long timeout;
	int  err_conv;
	struct econtroller *ectrl = (struct econtroller *)
		((struct seq_file *)file->private_data)->private;

	if (!ectrl)
		return -EINVAL;

	if (!capable(CAP_SYS_ADMIN))
                return -EACCES;

        if (count >= INPUT_BUF_SIZE)
                count = INPUT_BUF_SIZE - 1;

	memset(input, 0, INPUT_BUF_SIZE);
	if (copy_from_user (input, buf, count))
		return -EFAULT;	

	err_conv = strict_strtol(input, 0, &timeout);

	if (err_conv == 0) {
		if ((u16)timeout < 1)
			return -EINVAL; 
		mutex_lock (&ectrl->fs_lock);
		WDT_F_setTimeoutReg (ectrl->client, (u16)timeout);
		mutex_unlock (&ectrl->fs_lock);
	} else
		return err_conv;

	return count;
}


static int ectrl_proc_wdt_f_timeout_open_fs (struct inode *inode, struct file *file) {
        return single_open(file, ectrl_proc_wdt_f_timeout_show,  PDE_DATA(inode));
}


static const struct file_operations ectrl_proc_wdt_f_timeout_fops = {
        .owner = THIS_MODULE,
        .open = ectrl_proc_wdt_f_timeout_open_fs,
        .read = seq_read,
	.write = ectrl_proc_wdt_f_timeout_write,
        .llseek = seq_lseek,
        .release = single_release,
};



			/* --------------------------- */

struct proc_entry {
	char                   label[30];
	struct proc_dir_entry  *entry;
	struct list_head       list; 
};

static struct proc_entry *entry_list;

#define add_to_list(item, head, name, entry_dir)       item = kzalloc (sizeof (struct proc_entry), GFP_KERNEL); \
							strcpy (item->label, name); \
							item->entry = entry_dir; \
							list_add (&item->list, &head->list)



static int ectrl_add_proc_fs (struct econtroller *ectrl) {
	struct proc_dir_entry *entry = NULL;
        int i;

	struct ectrl_proc_event *pwr_btn_eproc;
	struct ectrl_proc_event *rst_btn_eproc;
	struct ectrl_proc_event *lid_hl_eproc;
	struct ectrl_proc_event *lid_lh_eproc;
	struct ectrl_proc_event *batlow_hl_eproc;
	struct ectrl_proc_event *batlow_lh_eproc;
	struct ectrl_proc_event *sleep_eproc;
	struct ectrl_proc_event *wake_eproc;
	
	struct ectrl_proc_event *pwr_btn_4sec_eproc; // no event and status for it

	struct ectrl_proc_event_state *pwr_btn_state_eproc;
	struct ectrl_proc_event_state *rst_btn_state_eproc;
	struct ectrl_proc_event_state *lid_state_eproc;
	struct ectrl_proc_event_state *batlow_state_eproc;
	struct ectrl_proc_event_state *sleep_state_eproc;
	struct ectrl_proc_event_state *wake_state_eproc;

	struct ectrl_proc_boot  *boot0_bproc;
	struct ectrl_proc_boot  *boot1_bproc;
	struct ectrl_proc_boot  *boot2_bproc;
	struct ectrl_proc_boot  *boot3_bproc;

	struct proc_entry *tmp;
	struct list_head *pos, *q;

	entry_list = kzalloc (sizeof (struct proc_entry), GFP_KERNEL);
	INIT_LIST_HEAD (&entry_list->list);

	/* create /proc/ectrl */
        ectrl_root_dir = proc_mkdir (ECTRL_PROC_ROOT, NULL);
        if (!ectrl_root_dir)
                return -ENODEV;
	else {
		add_to_list (tmp, entry_list, ECTRL_PROC_ROOT, NULL);
	}		

	/* EVENTS TREE */

	/* create /proc/ectrl/events */
	ectrl_events_dir = proc_mkdir (ECTRL_PROC_EVENTS, ectrl_root_dir);
	if (!ectrl_events_dir) {
		//TODO: error
	} else {
		add_to_list (tmp, entry_list, ECTRL_PROC_EVENTS, ectrl_root_dir);
	}

	for (i = 0 ; i < ectrl->nr_evnt ; i++) {
		ectrl->events[i]->proc_dir = proc_mkdir (ectrl->events[i]->label, ectrl_events_dir);
		if (!ectrl->events[i]->proc_dir) {
	                goto remove_dir;
		} else {
			add_to_list (tmp, entry_list, ectrl->events[i]->label, ectrl_events_dir);
		}
		switch (ectrl->events[i]->id) {
			case PWR_BUTTON:
				// --- power button --- //
				pwr_btn_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!pwr_btn_eproc) {
			                goto remove_dir;
				}
				pwr_btn_eproc->ectrl = ectrl;
				pwr_btn_eproc->event = PWR_BUTTON;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						pwr_btn_eproc);
				if (!entry) {
                                        goto remove_dir;		
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}
			
				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						pwr_btn_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					 add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						pwr_btn_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif
				break;

			case RST_BUTTON:
				// --- reset button --- //
				rst_btn_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!rst_btn_eproc) {
                                        goto remove_dir;
				}
				rst_btn_eproc->ectrl = ectrl;
				rst_btn_eproc->event = RST_BUTTON;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						rst_btn_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else  {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}
			
				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						rst_btn_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						rst_btn_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif		
		
				break;

			case LID_HL_SIGNAL:
				// --- Lid high to low --- //
				lid_hl_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!lid_hl_eproc) {
                                        goto remove_dir;
				}
				lid_hl_eproc->ectrl = ectrl;
				lid_hl_eproc->event = LID_HL_SIGNAL;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						lid_hl_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}
			
				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						lid_hl_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						lid_hl_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif
			
				break;
			
			case LID_LH_SIGNAL:
				// --- Lid low to high --- //
				lid_lh_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!lid_lh_eproc) {
                                        goto remove_dir;
				}
				lid_lh_eproc->ectrl = ectrl;
				lid_lh_eproc->event = LID_LH_SIGNAL;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						lid_lh_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					 add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}
			
				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						lid_lh_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						lid_lh_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif

				break;

			case BATLOW_HL_SIGNAL:
				// --- battery high to low --- //
				batlow_hl_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!batlow_hl_eproc) {
                                        goto remove_dir;
				}
				batlow_hl_eproc->ectrl = ectrl;
				batlow_hl_eproc->event = BATLOW_HL_SIGNAL;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						batlow_hl_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}
			
				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						batlow_hl_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						batlow_hl_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif

				break;

			case BATLOW_LH_SIGNAL:
				// --- battery low to high --- //
				batlow_lh_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!batlow_lh_eproc) {
                                        goto remove_dir;
				}
				batlow_lh_eproc->ectrl = ectrl;
				batlow_lh_eproc->event = BATLOW_LH_SIGNAL;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						batlow_lh_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}
			
				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						batlow_lh_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						batlow_lh_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else  {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif

				break;


			case SLEEP_SIGNAL:
				// --- sleep --- //
				sleep_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!sleep_eproc) {
                                        goto remove_dir;
				}
				sleep_eproc->ectrl = ectrl;
				sleep_eproc->event = SLEEP_SIGNAL;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						sleep_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}

				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						sleep_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						sleep_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif
				
				break;

			case WAKE_SIGNAL:
				// --- wake signal --- //
				wake_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
				if (!wake_eproc) {
			                goto remove_dir;
				}
				wake_eproc->ectrl = ectrl;
				wake_eproc->event = WAKE_SIGNAL;
			
				entry = proc_create_data(ECTRL_ENTRY_ENABLE,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enable_fops, 
						wake_eproc);
				if (!entry) {
                                        goto remove_dir;		
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENABLE, global_event_list[ectrl->events[i]->id].proc_dir);
				}

				entry = proc_create_data(ECTRL_ENTRY_ENFLASH,  S_IRUGO | S_IWUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_enflash_fops, 
						wake_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_ENFLASH, global_event_list[ectrl->events[i]->id].proc_dir);
				}

#if ECTRL_USE_OWN_STATE
				entry = proc_create_data(ECTRL_ENTRY_STATUS,  S_IRUGO,
				 		global_event_list[ectrl->events[i]->id].proc_dir, &ectrl_proc_state_fops, 
						wake_eproc);
				if (!entry) {
                                        goto remove_dir;
			        } else {
					add_to_list (tmp, entry_list, ECTRL_ENTRY_STATUS, global_event_list[ectrl->events[i]->id].proc_dir);
				}
#endif
				break;

			default:
				continue;

		}
	}

	/* create /proc/ectrl/events/event_state */
	ectrl_events_state_dir = proc_mkdir (ECTRL_PROC_EVENTS_STATE, ectrl_events_dir);
	if (!ectrl_events_state_dir) {
		goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_PROC_EVENTS_STATE, ectrl_events_dir);
	}

	for (i = 0 ; i < ectrl->nr_evn ; i++) {
		struct ectrl_proc_event_state *state_proc;
		switch (ectrl->evn_state[i]->evn) {
			case EVNT_PWR_BTN:
				pwr_btn_state_eproc = kzalloc (sizeof (struct ectrl_proc_event_state),
						 GFP_KERNEL);
				pwr_btn_state_eproc->ectrl = ectrl;
				state_proc = pwr_btn_state_eproc;
				break;
			case EVNT_RST_BTN:
				rst_btn_state_eproc = kzalloc (sizeof (struct ectrl_proc_event_state),
						 GFP_KERNEL);
				rst_btn_state_eproc->ectrl = ectrl;
				state_proc = rst_btn_state_eproc;
				break;
			case EVNT_SLEEP:
				sleep_state_eproc = kzalloc (sizeof (struct ectrl_proc_event_state),
						 GFP_KERNEL);
				sleep_state_eproc->ectrl = ectrl;
				state_proc = sleep_state_eproc;
				break;
			case EVNT_BATTERY:
				batlow_state_eproc = kzalloc (sizeof (struct ectrl_proc_event_state),
						 GFP_KERNEL);
				batlow_state_eproc->ectrl = ectrl;
				state_proc = batlow_state_eproc;
				break;
			case EVNT_LID:
				lid_state_eproc = kzalloc (sizeof (struct ectrl_proc_event_state),
						 GFP_KERNEL);
				lid_state_eproc->ectrl = ectrl;
				state_proc = lid_state_eproc;
				break;
			case EVNT_WAKE:
				wake_state_eproc = kzalloc (sizeof (struct ectrl_proc_event_state),
						 GFP_KERNEL);
				wake_state_eproc->ectrl = ectrl;
				state_proc = wake_state_eproc;
				break;
			default:
				state_proc = NULL;
				break;
		}	

		if (state_proc != NULL) {
			state_proc->state_id = i;
			entry = proc_create_data(ectrl->evn_state[i]->label, 
				S_IRUGO, ectrl_events_state_dir, &ectrl_proc_status_fops, state_proc);
			if (!entry) {
                        	goto remove_dir;
			} else {
				add_to_list (tmp, entry_list, ectrl->evn_state[i]->label, ectrl_events_state_dir);
			}
		}
		
	}


	/* POWER MANAGEMENT TREE */

	/* create /proc/ectrl/power_management */
	ectrl_pm_dir = proc_mkdir (ECTRL_PROC_PM, ectrl_root_dir);
	if (!ectrl_pm_dir) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ECTRL_PROC_PM, ectrl_root_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_PM_STATE, S_IRUGO,
				ectrl_pm_dir, &ectrl_proc_available_pm_state_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_PM_STATE, ectrl_pm_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_PM_PWR_STATE, S_IRUGO | S_IWUGO,
				ectrl_pm_dir, &ectrl_proc_pm_pwr_state_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		 add_to_list (tmp, entry_list, ECTRL_ENTRY_PM_PWR_STATE, ectrl_pm_dir);
	}
	
	ectrl_pm_pwr_btn_4sec_dir = proc_mkdir (ECTRL_ENTRY_PM_PWR_BTN_4SEC, ectrl_pm_dir);
	if (!ectrl_pm_dir) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_PM_PWR_BTN_4SEC, ectrl_pm_dir);
	}
	
	pwr_btn_4sec_eproc = kzalloc (sizeof (struct ectrl_proc_event), GFP_KERNEL);
	if (!pwr_btn_4sec_eproc) {
		goto remove_dir;
	}
	pwr_btn_4sec_eproc->ectrl = ectrl;
	pwr_btn_4sec_eproc->event = PWR_BTN_4SEC;
	pwr_btn_4sec_eproc->is_true_event = 0;

	entry = proc_create_data(ECTRL_ENTRY_PM_PWR_BTN_4SEC_EN,  S_IRUGO | S_IWUGO,
			ectrl_pm_pwr_btn_4sec_dir, &ectrl_proc_enable_fops, 
			pwr_btn_4sec_eproc);
	if (!entry) {
		goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_PM_PWR_BTN_4SEC_EN, ectrl_pm_pwr_btn_4sec_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_PM_PWR_BTN_4SEC_ENFLASH,  S_IRUGO | S_IWUGO,
			ectrl_pm_pwr_btn_4sec_dir, &ectrl_proc_enflash_fops, 
			pwr_btn_4sec_eproc);
	if (!entry) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_PM_PWR_BTN_4SEC_ENFLASH, ectrl_pm_pwr_btn_4sec_dir);
	}

	/* BOOT TREE */

	/* create /proc/ectrl/boot */
	ectrl_boot_dir = proc_mkdir (ECTRL_PROC_BOOT, ectrl_root_dir);
	if (!ectrl_boot_dir) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ECTRL_PROC_BOOT, ectrl_root_dir);
	}

	boot0_bproc = kzalloc (sizeof (struct ectrl_proc_boot), GFP_KERNEL);
	boot1_bproc = kzalloc (sizeof (struct ectrl_proc_boot), GFP_KERNEL);
	boot2_bproc = kzalloc (sizeof (struct ectrl_proc_boot), GFP_KERNEL);
	boot3_bproc = kzalloc (sizeof (struct ectrl_proc_boot), GFP_KERNEL);

	if (!boot0_bproc || !boot1_bproc || !boot2_bproc || !boot3_bproc) {
		goto remove_dir;
	}

	i = 0;
	boot0_bproc->ectrl = ectrl;
	boot0_bproc->idx = BOOT_IDX0;
	entry = proc_create_data(ECTRL_ENTRY_DEVBOOT0,  S_IRUGO | S_IWUGO,
				ectrl_boot_dir, &ectrl_proc_bootdev_fops, boot0_bproc);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_DEVBOOT0, ectrl_boot_dir);
	}

	i++;
	boot1_bproc->ectrl = ectrl;
	boot1_bproc->idx = BOOT_IDX1;
	entry = proc_create_data(ECTRL_ENTRY_DEVBOOT1,  S_IRUGO | S_IWUGO,
				ectrl_boot_dir, &ectrl_proc_bootdev_fops, boot1_bproc);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_DEVBOOT1, ectrl_boot_dir);
	}

	i++;
	boot2_bproc->ectrl = ectrl;
	boot2_bproc->idx = BOOT_IDX2;
	entry = proc_create_data(ECTRL_ENTRY_DEVBOOT2,  S_IRUGO | S_IWUGO,
				ectrl_boot_dir, &ectrl_proc_bootdev_fops, boot2_bproc);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_DEVBOOT2, ectrl_boot_dir);
	}

	i++;
	boot3_bproc->ectrl = ectrl;
	boot3_bproc->idx = BOOT_IDX3;
	entry = proc_create_data(ECTRL_ENTRY_RECOVERY_BOOT, S_IRUGO | S_IWUGO,
				ectrl_boot_dir, &ectrl_proc_bootdev_fops, boot3_bproc);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_RECOVERY_BOOT, ectrl_boot_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_BOOTDEV_LIST, S_IRUGO,
				ectrl_boot_dir, &ectrl_proc_bootdev_list_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_BOOTDEV_LIST, ectrl_boot_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_BOOT_SEQUENCE, S_IRUGO,
				ectrl_boot_dir, &ectrl_proc_bootseq_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_BOOT_SEQUENCE, ectrl_boot_dir);
	}

	/* WATCHDOG TREE */

	/* create /proc/ectrl/watchdog */
	ectrl_watchdog_dir = proc_mkdir (ECTRL_PROC_WATCHDOG, ectrl_root_dir);
	if (!ectrl_watchdog_dir) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ECTRL_PROC_WATCHDOG, ectrl_root_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_AVAL_EVN, S_IRUGO,
				ectrl_watchdog_dir, &ectrl_proc_avalaible_wdt_event_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_AVAL_EVN, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_ENABLE, S_IRUGO | S_IWUGO,   
				ectrl_watchdog_dir, &ectrl_proc_wdt_enable_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_ENABLE, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_EVENT, S_IRUGO | S_IWUGO,   
				ectrl_watchdog_dir, &ectrl_proc_wdt_event_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_EVENT, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_DELAY, S_IRUGO | S_IWUGO,
				ectrl_watchdog_dir, &ectrl_proc_wdt_delay_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_DELAY, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_TIMEOUT, S_IRUGO | S_IWUGO,
				ectrl_watchdog_dir, &ectrl_proc_wdt_timeout_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_TIMEOUT, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_TIMER1, S_IRUGO,
				ectrl_watchdog_dir, &ectrl_proc_wdt_timer1_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_TIMER1, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_TIMER2, S_IRUGO,
			ectrl_watchdog_dir, &ectrl_proc_wdt_timer2_fops, ectrl);
	if (!entry) {
		goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_TIMER2, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_STATE, S_IRUGO,   
				ectrl_watchdog_dir, &ectrl_proc_wdt_state_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		 add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_STATE, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_REFRESH, S_IWUGO,   
				ectrl_watchdog_dir, &ectrl_proc_wdt_refresh_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_REFRESH, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_RESTORE, S_IWUGO,   
				ectrl_watchdog_dir, &ectrl_proc_wdt_restore_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_RESTORE, ectrl_watchdog_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_WDTOUT_RESET, S_IWUGO,   
				ectrl_watchdog_dir, &ectrl_proc_wdt_wdtout_reset_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_WDTOUT_RESET, ectrl_watchdog_dir);
	}

	ectrl_watchdog_permanent_dir = proc_mkdir (ECTRL_PROC_WDT_PERMANENT, ectrl_watchdog_dir);
	if (!ectrl_watchdog_permanent_dir) {
		goto remove_dir;
	} else {
		add_to_list (tmp, entry_list, ECTRL_PROC_WDT_PERMANENT, ectrl_watchdog_dir);
	}


	entry = proc_create_data(ECTRL_ENTRY_WDT_EVENT, S_IRUGO | S_IWUGO,   
				ectrl_watchdog_permanent_dir, &ectrl_proc_wdt_f_event_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_EVENT, ectrl_watchdog_permanent_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_DELAY, S_IRUGO | S_IWUGO,
				ectrl_watchdog_permanent_dir, &ectrl_proc_wdt_f_delay_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_DELAY, ectrl_watchdog_permanent_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_TIMEOUT, S_IRUGO | S_IWUGO,
				ectrl_watchdog_permanent_dir, &ectrl_proc_wdt_f_timeout_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_TIMEOUT, ectrl_watchdog_permanent_dir);
	}

	entry = proc_create_data(ECTRL_ENTRY_WDT_ENABLE, S_IRUGO | S_IWUGO,   
				ectrl_watchdog_permanent_dir, &ectrl_proc_wdt_f_enable_fops, ectrl);
	if (!entry) {
                goto remove_dir;		
	} else {
		add_to_list (tmp, entry_list, ECTRL_ENTRY_WDT_ENABLE, ectrl_watchdog_permanent_dir);
	}

	return 0;
remove_dir:
	list_for_each_safe (pos, q, &entry_list->list) {
		tmp = list_entry (pos, struct proc_entry, list);
		remove_proc_entry (tmp->label, tmp->entry);
		list_del (pos);
		kfree (tmp);	
	}
	return -EINVAL;
}


static void ectrl_remove_proc_fs (struct econtroller *ectrl) {
	struct proc_entry *tmp;
	struct list_head *pos, *q;
	list_for_each_safe (pos, q, &entry_list->list) {
		tmp = list_entry (pos, struct proc_entry, list);
		remove_proc_entry (tmp->label, tmp->entry);
		list_del (pos);
		kfree (tmp);	
	}
}


/* --------------------------------------------------------------------------
                                     IOCTL
   -------------------------------------------------------------------------- */

static long ectrl_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	struct i2c_client *client = file->private_data;
	int err = 0;
	int retval = 0;
	dev_dbg(&client->dev, "ioctl, cmd=0x%02x, arg=0x%02lx\n", cmd, arg);
	switch (cmd) {
		case ECTRL_IOCTL_REG_READ: {
			struct ectrl_reg reg;
			struct ectrl_reg_rw reg_rw;
			if (copy_from_user (&reg, (const void __user *)arg, sizeof (reg))) {
				return -EFAULT;
			}
			get_reg_rw (&reg_rw, reg, R_OP);
			err = ectrl_mem_single_op (client, &reg_rw);
			reg.data = reg_rw.reg.data;
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &reg, sizeof (reg))) {
				retval = -EFAULT;
			}
			break;
		}
		case ECTRL_IOCTL_REG_WRITE: {
			struct ectrl_reg reg;
			struct ectrl_reg_rw reg_rw;
			if (copy_from_user (&reg, (const void __user *)arg, sizeof (reg))) {
				return -EFAULT;
			}
			get_reg_rw (&reg_rw, reg, W_OP);
			err = ectrl_mem_single_op (client, &reg_rw);
			reg.data = reg_rw.reg.data;
			if (err < 0)
				retval = err;
			if (copy_to_user ((void __user *)arg, &reg, sizeof (reg))) {
				retval = -EFAULT;
			}
			break;
		}
		default:
			break;
	}
	return retval;
}


static int ectrl_open(struct inode *inode, struct file *file) {
	file->private_data = ectrl->client;
	return 0;
}


int ectrl_release(struct inode *inode, struct file *file) {
	file->private_data = NULL;
	return 0;
}


static const struct file_operations ectrl_fileops = {
	.owner = THIS_MODULE,
	.open = ectrl_open,
	.unlocked_ioctl = ectrl_ioctl,
	.release = ectrl_release,
};


static struct miscdevice ectrl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "econtroller",
	.fops = &ectrl_fileops,
};

/************************************/

static void manage_event (struct econtroller *ectrl, struct event_dev *event) {
	int code = 0;
	switch (event->id) {
		case PWR_BUTTON:
			code = BTN_ECTRL_PWR;
			break;
		case SLEEP_SIGNAL:
			code = BTN_ECTRL_SLEEP;
			break;
		case WAKE_SIGNAL:
			code = BTN_ECTRL_WAKE;
			break;
		case BATLOW_HL_SIGNAL:
			code = BTN_ECTRL_BATLOW_HL;
			break;
		case BATLOW_LH_SIGNAL:
			code = BTN_ECTRL_BATLOW_LH;
			break;
		case LID_HL_SIGNAL:
			code = BTN_ECTRL_LID_HL;
			break;
		case LID_LH_SIGNAL:
			code = BTN_ECTRL_LID_LH;
			break;
		default:
			code = -1;
			break; 
	}
	
	if ( code > 0 ) {
		input_event (event->input, EV_PWR, code, 1);
		input_sync (event->input);
		ECTRL_INFO ("event name: %s\n", event->name);
	} else {
		ECTRL_ERR ("invalid event occures !!!");
	}
}


static int getIdxStatus (struct econtroller *ectrl, int event) {
	int idx;
	for (idx = 0 ; idx < ectrl->nr_evn ; idx++)
		if (ectrl->evn_state[idx]->evn == event)
			return idx;
	return -1;
}

#define TIME_POWER_PRESSED  40 /* ms */
static void ectr_work_pwr_btn_snooper (struct work_struct *work) {
	struct econtroller *ectrl = container_of(to_delayed_work(work), struct econtroller, work_snooper_pwr_btn);
	int status = 0;
	if (ectrl->client) { 
		status = getStatus (ectrl->client, 
				ectrl->evn_state[getIdxStatus(ectrl, EVNT_PWR_BTN)]->reg_idx);
		if (time_after (jiffies, ectrl->orig_jiffies + TIME_POWER_PRESSED)) {
			return;
		} else {
			if (!status) { // button still active, re-schedule the work
				schedule_delayed_work (&ectrl->work_snooper_pwr_btn, 
					msecs_to_jiffies(ectrl->poll_period_snooper_pwr_btn));
			} else { // button is no longer active, we have a valid short pressed
				 // power button event, so we will notify this to the system
				manage_event (ectrl, ectrl->events[PWR_BUTTON]);
				return;
			}
		}
	} else {
		return;
	}
}


#define GET_EVENT_FLAG(reg, event)  ((reg) & event->index_reg) >> event->shift

static void ectrl_work (struct work_struct *work) {	
	int i;
	static int  flag_reg = 0;
	static int  en_reg = 0; 
	struct econtroller *ectrl = container_of(to_delayed_work(work), struct econtroller, work);
	if (ectrl->client) {
		// since an event was detected, we temporary disable all event, writing 
		// 0x0000 into the enable register. But, first we must save the current
		// state of this register
		if (en_reg == 0) {
			en_reg = getEnableReg (ectrl->client);
			setEnableReg (ectrl->client, 0x0000);
		}
		if (flag_reg == 0) {
			flag_reg = getFlagReg (ectrl->client);	
		}
	} else {
		return;
	}
	if (flag_reg) {
		// detect event
		for (i = 0 ; i < ectrl->nr_evnt ; i++) {
			if (GET_EVENT_FLAG(flag_reg, ectrl->events[i]) == 1) {
				if (ectrl->events[i]->id == PWR_BUTTON) {
					ectrl->orig_jiffies = jiffies;
					schedule_delayed_work (&ectrl->work_snooper_pwr_btn,
						msecs_to_jiffies(ectrl->poll_period_snooper_pwr_btn));
				} else {
					manage_event (ectrl, ectrl->events[i]);
				}
				flag_reg &= ~(1 << (*ectrl->events[i]).shift);
			}
		}
		// we have to clean the served event's flag 
		setFlagReg (ectrl->client, 0);
		udelay (10);
		schedule_delayed_work (&ectrl->work, msecs_to_jiffies(ectrl->poll_period));
	} else {
		// there is no event to handle
		// this is a safe write, to ensure that the IRQ signal becomes high
		ectrl_write_data (ectrl->client, FLAG_REG, 0);
		enable_irq (ectrl->irq);
		setEnableReg (ectrl->client, en_reg);
		en_reg = 0;
	}
}


static void ectrl_free_irq (struct econtroller *ectrl) {
	free_irq (ectrl->irq, ectrl);
        if (cancel_delayed_work_sync(&ectrl->work)) {
                /*
                 * Work was pending, therefore we need to enable
                 * IRQ here to balance the disable_irq() done in the
                 * interrupt handler.
                 */
                enable_irq(ectrl->irq);
        }
}


static irqreturn_t irq_interrupt_manager (int irq, void *dev_id)  {
	struct econtroller *ectrl = *((struct econtroller **)dev_id);

	disable_irq_nosync (ectrl->irq);
	schedule_delayed_work (&ectrl->work, msecs_to_jiffies(ectrl->poll_period));

	return IRQ_HANDLED;
}


static void ectrl_detect_state (struct econtroller *ectrl, int *enable, int *flags, int *states) {
	*flags = getFlagReg (ectrl->client);
	*states = getStatusReg (ectrl->client);
	// to ensure that no other events rise during the resolving of the state of the
	// machine, the enable of all events will be put to zero.
	// The normal state of enable register will be restore at the end of this procedure.
	*enable = getEnableReg (ectrl->client);
	setEnableReg (ectrl->client, 0x0000);
}


static void ectrl_resolve_state (struct econtroller *ectrl, int enable, int flags, int status) {
	int i;
	int nr_events = ectrl->nr_evnt;
	for (i = 0 ; i < nr_events ; i++) {
		if (GET_EVENT_FLAG(flags, ectrl->events[i]) == 1) {
			manage_event (ectrl, ectrl->events[i]);
			flags &= ~(1 << global_event_list[ectrl->events[i]->id].shift);
		}
	}
	// we have to clean the served event's flag 
	setFlagReg (ectrl->client, 0x0000);
	udelay (10);
	
	// At this point, the original state of the enable register can be restored
	setEnableReg (ectrl->client, enable);
}


static int ectrl_parse_dt (struct econtroller *ectrl, int board_id) {

	struct device dev = ectrl->client->dev;
	struct device_node *np;

	struct property *prop;
	int length, num_event;
	u32 *of_event_list, *event_list;
	struct event_dev **events;

	int idx, nre, i, idxs;
	int ret, err;

	int num_bootdev;
	struct device_node *parent, *child;


	np = of_node_get (dev.of_node);
	if ( ! np )
		return -EINVAL;

	/* -----------------------------------
	 * retrive the list of feasible events
	 * ----------------------------------- */
	prop = of_find_property (np, "events", &length);
	if ( ! prop ) {
		pr_err("%s: could not find property %s\n",
				of_node_full_name(np), "events");
		return -EINVAL;
	}

	num_event = length / sizeof(u32);

	ECTRL_DBG ("Num. of event found: %d", num_event);

	/*  list of event, as u32, retrived directly from device tree  */
	of_event_list = kzalloc (sizeof (u32) * num_event , GFP_KERNEL);
	if ( ! of_event_list ) {
		err = -EINVAL;
		goto err_alloc_of_event_list;
	}

	/*  list of event used by the driver  */
	event_list = kzalloc (sizeof (u32) * num_event, GFP_KERNEL);
	if ( ! event_list ) {
		err = -EINVAL;
		goto err_alloc_event_list;
	}

	ret = of_property_read_u32_array (np, "events", of_event_list, num_event);

	nre = 0;
	for ( i = 0 ; i < num_event ; i++ )  {

		/*  switch from DTB data to event index  */
		switch ( of_event_list[i] ) {
			case ECTRL_EVNT_PWR_BTN:
				event_list[i] = EVNT_PWR_BTN;
				break;
			case ECTRL_EVNT_RST_BTN:
				event_list[i] = EVNT_RST_BTN;
				break;
			case ECTRL_EVNT_BATTERY:
				event_list[i] = EVNT_BATTERY;
				break;
			case ECTRL_EVNT_LID:
				event_list[i] = EVNT_LID;
				break;
			case ECTRL_EVNT_SLEEP:
				event_list[i] = EVNT_SLEEP;
				break;
			case ECTRL_EVNT_WAKE:
				event_list[i] = EVNT_WAKE;
				break;
			default:
				/*  unrecognized event code  */
				return -EINVAL;
		}
		
		/*  count the effective event states used by the driver.
		 *  Some event source requires two event (one for each edge
		 *  of the signal.
		 *  */
		if ( of_event_list[i] & (ECTRL_EVNT_BATTERY | ECTRL_EVNT_LID) ) 
			nre += 2;
		if ( of_event_list[i] & (ECTRL_EVNT_PWR_BTN | ECTRL_EVNT_RST_BTN |
					ECTRL_EVNT_SLEEP | ECTRL_EVNT_WAKE) )
			nre++;

	}

	ectrl->nr_evn = board_nr_states[board_id]; 
	ectrl->evn_state = (struct event_state **)kzalloc (sizeof (struct event_state *) * ectrl->nr_evn, GFP_KERNEL);
	if (!ectrl->evn_state) {
		err = -ENOMEM;
		goto err_alloc_evn_state;
	}
      
	events = (struct event_dev **)kzalloc (sizeof (struct event_dev *) * nre, GFP_KERNEL);
	if (!events) {
		err = -ENOMEM;
		goto err_alloc_events;
	}

	for ( idx = 0, idxs = 0, i = 0 ; i < num_event ; i++ ) {

		//  check if the event is available for the board
		if ( board_reg_state_idx[board_id][event_list[i]] != -1 ) {
			ectrl->evn_state[idxs] = &event_state_list[event_list[i]];
			ectrl->evn_state[idxs]->reg_idx = board_reg_state_idx[board_id][event_list[i]];
			idxs++;
		}

		switch ( event_list[i] ) {
			case EVNT_PWR_BTN:
				events[idx] = &global_event_list[PWR_BUTTON];
				idx++;
				break;
			case EVNT_RST_BTN:
				events[idx] = &global_event_list[RST_BUTTON];
				idx++;
				break;
			case EVNT_SLEEP:
				events[idx] = &global_event_list[SLEEP_SIGNAL];
				idx++;
				break;
			case EVNT_BATTERY:
				events[idx] = &global_event_list[BATLOW_HL_SIGNAL];
				events[idx + 1] = &global_event_list[BATLOW_LH_SIGNAL];
				idx += 2;
				break;
			case EVNT_LID:
				events[idx] = &global_event_list[LID_HL_SIGNAL];
				events[idx + 1] = &global_event_list[LID_LH_SIGNAL];
				idx += 2;
				break;
			case EVNT_WAKE:
				events[idx] = &global_event_list[WAKE_SIGNAL];
				idx ++;
				break;
			default:
				break;
		}	
	}

	ectrl->events = events; 
	ectrl->nr_evnt = nre;


	parent = of_get_child_by_name(np, "boot_device");
	if ( ! parent ) {
		ECTRL_ERR ("boot_device node not found");
		err = -EINVAL;
		goto err_no_boot_device_node;
	}
		
	num_bootdev = 0;
	for_each_child_of_node (parent, child) 
		if ( strcmp (child->name, "bootdev") == 0 )
				num_bootdev++;
	
	if ( !num_bootdev ) {
		/*  No boot device found  */
		ECTRL_ERR ("No boot device found!");	
		err = -EINVAL;
		goto err_no_boot_device;
	}

	ECTRL_DBG ("Num of bootdev: %d", num_bootdev);
	ectrl->nr_bootdev = num_bootdev;
	ectrl->bootdev_list = kzalloc (sizeof (struct bootdev *) * ectrl->nr_bootdev, GFP_KERNEL);
	if ( !ectrl->bootdev_list ) {
		err = -ENOMEM;
		goto err_alloc_bootdev_list;
	}

	i = 0;
	for_each_child_of_node (parent, child) {
		/*  check if the node is a boot device node info  */
		if ( strcmp (child->name, "bootdev") == 0 ) {
			of_property_read_u32 (child, "id", (u32 *)&ectrl->bootdev_list[i].id);
			of_property_read_string (child, "label", &ectrl->bootdev_list[i].label);
			i++;
		}
	
	}
	
	ectrl->irq = irq_of_parse_and_map (np, 0);

	return 0;

err_alloc_bootdev_list:
err_no_boot_device:
err_no_boot_device_node:
	for ( i = 0 ; i < nre ; i++ )
		kfree (events[i]);
	kfree (events);
err_alloc_events:
	for ( i = 0 ; i < ectrl->nr_evn ; i++ )
		kfree (ectrl->evn_state[i]);
	kfree (ectrl->evn_state);
err_alloc_evn_state:
	kfree (event_list);
err_alloc_event_list:
	kfree (of_event_list);
err_alloc_of_event_list:
	kfree (prop);

	return err;
}


static const struct i2c_device_id ectrl_idtable[] = {
	{ "ectrl", 0 },
	{ /*  sentinel  */ }
};
MODULE_DEVICE_TABLE(i2c, ectrl_idtable);


static const struct of_device_id seco_ectrl_i2c_match[] = {
	{ .compatible = "seco,ectrl" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, seco_ectrl_of_match);


void SetPowerState (int state, int validate);

static int ectrl_probe(struct i2c_client *client, const struct i2c_device_id *id) {

	int err, ret, reg = 0, i;
	int enable_reg, flag_reg, status_reg;
	int bidx;
	struct input_dev  *input;

	const struct of_device_id *match = NULL;

	if ( !i2c_check_functionality (client->adapter, 
				I2C_FUNC_SMBUS_READ_WORD_DATA | I2C_FUNC_I2C) ) {
		err = -EIO;
		goto err_i2c_check;
	}

	ectrl = kzalloc (sizeof (struct econtroller), GFP_KERNEL);

	if ( !ectrl ) {
		err = -ENOMEM;
		goto err_alloc_ectrl;
	}

	if ( client->dev.of_node ) {
		match = of_match_device (of_match_ptr (seco_ectrl_i2c_match),
				&client->dev);
		if ( !match ) {
			ECTRL_ERR ("No device match found");
			err = -ENODEV;
			goto err_no_match;
		}
	}

	ectrl->client = client;

	ectrl->board_id = getBoardID(ectrl->client);

	/*  retrive the current board id  */
	switch (ectrl->board_id) {
		case BOARD_928_ID:
			bidx = BOARD_928;
			break; 
		case BOARD_962_ID:
			bidx = BOARD_962;
			break;
		case BOARD_984_ID:
			bidx = BOARD_984;
			break;
		default:
			bidx = -1;
			break;
	}

	if (bidx == -1) {
		//  unsupported board
		ECTRL_ERR ("Unsupported board: %d", ectrl->board_id);	
		err = -EINVAL;
		goto err_inv_id;
	}

	ECTRL_INFO ("Detected board: %s", board_name[bidx]);	

	ret = ectrl_parse_dt (ectrl, bidx);

	if ( ret ) {
		ECTRL_ERR ("error in parsing dt: %d", ret);
		err = ret;
		goto err_parsing_dt;
	}

	ectrl->wdt_event_list = wdt_evnt_list;
	ectrl->nr_wdt_event = ARRAY_SIZE (wdt_evnt_list);

	INIT_DELAYED_WORK (&ectrl->work, ectrl_work);
	ectrl->poll_period  = ECTRL_POLL_PERIOD;
	INIT_DELAYED_WORK (&ectrl->work_snooper_pwr_btn, ectr_work_pwr_btn_snooper);
	ectrl->poll_period_snooper_pwr_btn = ECTRL_POLL_PERIOD_SNOOPER_PWR_BTN;

	
	/* input interface */

	input = input_allocate_device();
	if ( !input) {
		err = -ENOMEM;
		goto err_input;
	}

	ectrl->phys = kzalloc (sizeof (char) * 32, GFP_KERNEL);
	snprintf(ectrl->phys, sizeof(char) * 32,
			"%s/input%d", dev_name(&client->dev), 0);

	input->name        = "seco_ectrl";
	input->phys        = ectrl->phys;
	input->id.bustype  = BUS_I2C;
	input->dev.parent  = &client->dev;

	for ( i = 0 ; i < ectrl->nr_evnt ; i++ ) {

		ectrl->events[i]->input = input;

		switch ( ectrl->events[i]->id ) {
			case PWR_BUTTON:
				input_set_capability (input, EV_PWR, BTN_ECTRL_PWR);
				break;
			case SLEEP_SIGNAL:
				input_set_capability (input, EV_PWR, BTN_ECTRL_SLEEP);
				break;
			case WAKE_SIGNAL:
				input_set_capability (input, EV_PWR, BTN_ECTRL_WAKE);
			case BATLOW_HL_SIGNAL:
				input_set_capability (input, EV_PWR, BTN_ECTRL_BATLOW_HL);
				break;
			case BATLOW_LH_SIGNAL:
				input_set_capability (input, EV_PWR, BTN_ECTRL_BATLOW_LH);
				break;
			case LID_HL_SIGNAL:
				input_set_capability (input, EV_PWR, BTN_ECTRL_LID_HL);
				break;
			case LID_LH_SIGNAL:
				input_set_capability (input, EV_PWR, BTN_ECTRL_LID_LH);
				break;
			case RST_BUTTON:
			case PWR_BTN_4SEC:
				/*  no input for these signals since the embedded 
				 *  controller will perform a brutal shotdown 
				 *  when these event rise.
				 */
			default:
				break;
		}

	}

	err = input_register_device (input);
	if ( err ) {
		ECTRL_ERR ("No input assigned");
		err = -EINVAL;
		reg = i;
		goto err_reg_input;
	}

	ectrl_add_proc_fs (ectrl);

	/* detect and resolve the initial state */
	ectrl_detect_state (ectrl, &enable_reg, &flag_reg, &status_reg);
	ectrl_resolve_state (ectrl, enable_reg, flag_reg, status_reg);

	/* mutex initialization */
	mutex_init (&ectrl->fs_lock);

	/* irq acquisition */
	if ( ectrl->irq == 0 ) {
		ECTRL_ERR ("No IRQ assigned");
		err = -EINVAL;
		goto err_irq;
	}
	ret = request_irq (ectrl->irq, irq_interrupt_manager,
				IRQF_SHARED | IRQF_TRIGGER_FALLING, "ectrl", &ectrl);
	
	if ( ret ) {
		ECTRL_ERR ("IRQ not acquired: error %d", ret);
		err = -EIO;
		goto err_free_irq;
	}

	ret = misc_register (&ectrl_device);
	if ( ret ) {
		ECTRL_ERR ("misc registration failed: %d", ret);
		err = -EIO;
		goto err_misc_register;
	}

	SetPowerState (0x0, 0);

	ECTRL_INFO (" probe done");
	printk (KERN_INFO "ec: 0x%02x\n", ectrl_read_data (ectrl->client, BUILDREV_REG) >> 8);

	dev_set_drvdata (&client->dev, ectrl);

	return 0;


err_misc_register:
	ectrl_free_irq (ectrl);	
err_free_irq:
err_irq:
	input_unregister_device (input);
err_reg_input:
	kfree (input);
err_input:
	for ( i = 0 ; i < ectrl->nr_evnt ; i++ )
		kfree (ectrl->events[i]);
	kfree (ectrl->events);
	for ( i = 0 ; i < ectrl->nr_evn ; i++ )
		kfree (ectrl->evn_state[i]);
	kfree (ectrl->evn_state);
err_parsing_dt:
	kfree (match);
err_inv_id:
err_no_match:
	kfree (ectrl);
err_alloc_ectrl:
err_i2c_check:

	return err;
}


static int ectrl_remove(struct i2c_client *client) {
	int i;
	struct ectrl_platform_data *data;
	data = i2c_get_clientdata(client);
	misc_deregister (&ectrl_device);
	ectrl_free_irq (ectrl);
	ectrl_remove_proc_fs (ectrl);
	kfree (entry_list);

	for ( i = 0 ; i < ectrl->nr_evnt ; i++ )
		kfree (ectrl->events[i]);
	kfree (ectrl->events);

	for ( i = 0 ; i < ectrl->nr_evn ; i++ )
		kfree (ectrl->evn_state[i]);
	kfree (ectrl->evn_state);

	kfree (ectrl);
	kfree (data);
	return 0;
}


void SetPowerState (int state, int validate) {
	PowerState = state;
	StateValidate = validate;
}





void device_shutdown_ectrl(struct i2c_client *client) {
	int retval;
	if (StateValidate) {
		switch (PowerState) {
			case LINUX_REBOOT_CMD_RESTART:
				retval = ectrl_SystemReboot (client);
				break;
			case LINUX_REBOOT_CMD_HALT:
				retval = ectrl_SystemHalt (client);
				break;
			case LINUX_REBOOT_CMD_CAD_ON:
				break;
			case LINUX_REBOOT_CMD_CAD_OFF:
				break;
			case LINUX_REBOOT_CMD_POWER_OFF:
				break;
			case LINUX_REBOOT_CMD_RESTART2:
				break;
			case LINUX_REBOOT_CMD_SW_SUSPEND:
				break;
			case LINUX_REBOOT_CMD_KEXEC:
				break;
			default:
				break;
		}
	}
}




static struct i2c_driver ectrl_driver = {
	.driver = {
		.owner	         = THIS_MODULE,
		.name	         = "ectrl",
		.of_match_table  = of_match_ptr(seco_ectrl_i2c_match),
	},
	.id_table	= ectrl_idtable,
	.probe		= ectrl_probe,
	.remove		= ectrl_remove,
	.shutdown   = device_shutdown_ectrl,
	
};


//module_i2c_driver(ectrl_driver);

static int __init ectrl_init(void) {
	return i2c_add_driver(&ectrl_driver);
}


static void __exit ectrl_exit(void) {
	i2c_del_driver(&ectrl_driver);
}


module_init(ectrl_init);
module_exit(ectrl_exit);


MODULE_AUTHOR("Davide Cardillo, SECO srl");
MODULE_DESCRIPTION("SECO system halt with Embedded Controller");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);
