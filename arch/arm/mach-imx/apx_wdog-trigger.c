/*
 * 
 *
 * WDOG Trigger driver
 *
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
//#include <linux/init.h>
//#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
//#include <linux/platform_device.h>


#include <linux/init.h>
#include <linux/smp.h>
#include <asm/cacheflush.h>
#include <asm/page.h>
#include <asm/smp_scu.h>
#include <asm/mach/map.h>

#include "common.h"


#include "apx_wdog-trigger.h"
#include "hardware.h"

#define WDOG_TRIGGER_REFRESH			msecs_to_jiffies(50)		// milliseconds
#define WDOG_TIME				msecs_to_jiffies(10*20)	// 10s


#define WDT_PAD_MUX_GPIO(x)     writel_relaxed(readl_relaxed((x)) | 0x5, (x))

#define WDT_PAD_CTRL_GPIO(x)    writel_relaxed(0x06028, (x))

#define WDT_DIR_OUT_GPIO(x,n)   writel_relaxed(readl_relaxed((x) + 0x4) | ( 1 << (n)), (x))

#define WDT_SET_H_GPIO(x,n)     writel_relaxed(readl_relaxed((x)) | (1 << (n)) , (x))

#define WDT_SET_L_GPIO(x,n)     writel_relaxed(readl_relaxed((x)) & ~(1 << (n)) , (x))



typedef struct wdog_data {
	void __iomem *iomux;
        void __iomem *padctrl;
        void __iomem *base;
        int           num;
} wdog_data_t;


static struct wdog_trigger_data {

        void           (*refresh_wdt)(void);
	unsigned long  wdt_time;
	unsigned long  wdt_count;
	int            wdt_refresh;
	int            wdt_always_on;
	int            wdt_enable;
	wdog_data_t    pin_trg;
	wdog_data_t    pin_en;
	int            is_initialized;
} wdt;




static void wdog_trigger_work_handler(struct work_struct *w);
 
static struct workqueue_struct *wq = 0;
static DECLARE_DELAYED_WORK(wdog_trigger_work, wdog_trigger_work_handler);



static void imx6_a62_wdog_refresh (void) {

	WDT_SET_H_GPIO(wdt.pin_trg.base, wdt.pin_trg.num);

	udelay(1);

	WDT_SET_L_GPIO(wdt.pin_trg.base, wdt.pin_trg.num);

}


static void apx_wdog_trigger_configure_pins(char *dir) {

	switch (dir[0]) {
		case 'e':

			WDT_PAD_MUX_GPIO(wdt.pin_trg.iomux);

			WDT_PAD_CTRL_GPIO(wdt.pin_trg.padctrl);

			WDT_DIR_OUT_GPIO(wdt.pin_trg.base, wdt.pin_trg.num);

		
			WDT_PAD_MUX_GPIO(wdt.pin_en.iomux);

			WDT_PAD_CTRL_GPIO(wdt.pin_en.padctrl);

			WDT_DIR_OUT_GPIO(wdt.pin_en.base, wdt.pin_en.num);

			WDT_SET_H_GPIO(wdt.pin_en.base, wdt.pin_en.num);

			break;

		case 'd':
		
			WDT_PAD_MUX_GPIO(wdt.pin_trg.iomux);

			WDT_PAD_CTRL_GPIO(wdt.pin_trg.padctrl);

			WDT_DIR_OUT_GPIO(wdt.pin_trg.base, wdt.pin_trg.num);

			
			WDT_PAD_MUX_GPIO(wdt.pin_en.iomux);

			WDT_PAD_CTRL_GPIO(wdt.pin_en.padctrl);

			WDT_DIR_OUT_GPIO(wdt.pin_en.base, wdt.pin_en.num);
			
			WDT_SET_L_GPIO(wdt.pin_en.base, wdt.pin_en.num);

			break;

		default:
			printk(KERN_ERR "apx_wdog: unknown pins direction\n");
		break;
	}
}


static void enable_triggering (void) {
        apx_wdog_trigger_configure_pins ("enable");
        wdt.wdt_count = wdt.wdt_time;
        wdt.wdt_always_on = 0;
        wdt.refresh_wdt ();
        pr_info ("apx_wdog: enabled wdog - %u s between refresh\n", jiffies_to_msecs(WDOG_TIME) / 1000 );
        wdt.refresh_wdt();
        if ( !wq )
                wq = create_singlethread_workqueue ("wdog_trigger");
        if ( wq )
                queue_delayed_work (wq, &wdog_trigger_work, WDOG_TRIGGER_REFRESH);

}


static void disable_triggering (void) {
        if( wq )
                flush_workqueue (wq);
        apx_wdog_trigger_configure_pins ("disable");
}



void apx_wdog_trigger_early_init (const struct apx_wdog_trigger_data *apx_wdt_data, int state) {

	wdt.pin_trg.iomux      = ioremap (apx_wdt_data->gpio_trg__iomux_ctrl, 0x20);
        wdt.pin_trg.padctrl    = ioremap (apx_wdt_data->gpio_trg__pad_ctrl, 0x20);
        wdt.pin_trg.base       = ioremap (apx_wdt_data->gpio_trg__base, 0x20);
	wdt.pin_trg.num        = apx_wdt_data->gpio_trg__num;
        wdt.pin_en.iomux       = ioremap (apx_wdt_data->gpio_en__iomux_ctrl, 0x20);
        wdt.pin_en.padctrl     = ioremap (apx_wdt_data->gpio_en__pad_ctrl, 0x20);
        wdt.pin_en.base        = ioremap (apx_wdt_data->gpio_en__base, 0x20);
	wdt.pin_en.num	       = apx_wdt_data->gpio_en__num;

	wdt.is_initialized = 0;

	apx_wdog_trigger_configure_pins("disable");

}


static void wdog_trigger_work_handler (struct work_struct *w) {
	wdt.refresh_wdt();
//	if( wdt.wdt_count > 0 ) {
		queue_delayed_work(wq, &wdog_trigger_work, WDOG_TRIGGER_REFRESH);
//		if (wdt.wdt_refresh)
//			wdt.wdt_refresh = 0;
//		else
//			wdt.wdt_count = wdt.wdt_count - 1;
//		if ( wdt.wdt_always_on )
//			 wdt.wdt_count = wdt.wdt_time;
//	}
}






int __init apx_wdog_trigger_work_init (int state) {
	
	pr_info("apx_wdog trigger loaded\n");

	/* Variables init */
	wdt.refresh_wdt = imx6_a62_wdog_refresh;
	wdt.wdt_time = WDOG_TIME;
	wdt.wdt_refresh = 0;
	wdt.wdt_count = wdt.wdt_time;	
	wdt.wdt_enable = 0;
	wdt.wdt_always_on = 0;

	wdt.is_initialized = 1;


	if ( state ) {
		apx_wdog_trigger_configure_pins("enable");
		enable_triggering ();
	} else {
 		disable_triggering ();
	      	apx_wdog_trigger_configure_pins("disable");
	}
	
	return 0;
}






