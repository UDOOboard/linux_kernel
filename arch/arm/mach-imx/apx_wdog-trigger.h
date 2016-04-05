#ifndef __WDOG_TRIGGER_PLATFORM_H__
#define __WDOG_TRIGGER_PLATFORM_H__



struct apx_wdog_trigger_data {
        	
	void (*refresh_wdt)(void);
        unsigned long wdt_jiffies;
	u32 gpio_trg__iomux_ctrl;
	u32 gpio_trg__pad_ctrl;
	u32 gpio_trg__base;
	int gpio_trg__num;
	u32 gpio_en__iomux_ctrl;
        u32 gpio_en__pad_ctrl;
	u32 gpio_en__base;
	int gpio_en__num;
};

int apx_wdog_trigger_work_init (int state);
void apx_wdog_trigger_early_init (const struct apx_wdog_trigger_data *wdtd, int state);

#endif /* __WDOG_TRIGGER_PLATFORM_H__ */

