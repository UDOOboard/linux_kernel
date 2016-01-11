/*
 * UDOO board power off
 *
 * Copyright (C) 2014 Jasbir Matharu
 * Copyright (C) 2015 Peter Vicman
 * Copyright (C) 2015 Francesco Montefoschi
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <../arch/arm/mach-imx/hardware.h>

static void (*pm_power_off_orig)(void) = NULL;
static int pwr_5v_gpio = -EINVAL;
static int lcd_panel_on_gpio = -EINVAL;
static int lcd_backlight_gpio = -EINVAL;

static void udoo_set_gpio(unsigned gpio, int value) {
	int ret;

	if (! gpio_is_valid(gpio)) return;

	ret = gpio_direction_output(gpio, value);
	if (ret) {
		pr_err("%s: gpio %u/%d failed\n", __func__, gpio, value);
	}
}

static void udoo_request_gpio(struct device *dev, unsigned gpio, unsigned long flags, const char *label) {
	int ret;

	if (! gpio_is_valid(gpio)) return;
		
	ret = devm_gpio_request_one(dev, gpio, flags, label);
	if (ret) {
		dev_err(dev, "request of gpio %s %u failed with %d\n", label, gpio, ret);
	}
}

static void udoo_power_off(void) {
	pr_emerg("%s: powering off\n", __func__);

	if (pm_power_off_orig != NULL) {
		pm_power_off_orig();
	}

	if (lcd_panel_on_gpio != -EINVAL) {
		udoo_set_gpio(lcd_panel_on_gpio, 0);
	}
	if (lcd_backlight_gpio != -EINVAL) {
		udoo_set_gpio(lcd_backlight_gpio, 0);
	}

	if (!cpu_is_imx6sx()) {
		if (gpio_is_valid(pwr_5v_gpio)) {
			pr_emerg("%s: 5V power down\n", __func__);
			udoo_set_gpio(pwr_5v_gpio, 1);
		}
	}
}

static int udoo_power_off_probe(struct platform_device *pdev)
{
	struct device_node *pwr_off_np;

	pwr_off_np = of_find_compatible_node(NULL, NULL, "udoo,lvds-power");
	if (pwr_off_np) {
		printk("[UDOO power-off] Probed LVDS power.\n");
		lcd_panel_on_gpio = of_get_named_gpio(pwr_off_np, "gpio", 0);
		of_node_put(pwr_off_np);
		udoo_request_gpio(&pdev->dev, lcd_panel_on_gpio, GPIOF_OUT_INIT_HIGH, "lcd_panel_on_gpio");
	}
	
	pwr_off_np = of_find_compatible_node(NULL, NULL, "udoo,lvds-backlight");
	if (pwr_off_np) {
		printk("[UDOO power-off] Probed LVDS backlight.\n");
		lcd_backlight_gpio = of_get_named_gpio(pwr_off_np, "gpio", 0);
		of_node_put(pwr_off_np);
		udoo_request_gpio(&pdev->dev, lcd_backlight_gpio, GPIOF_OUT_INIT_HIGH, "lcd_backlight_gpio");
	}
	
	pwr_off_np = of_find_compatible_node(NULL, NULL, "udoo,poweroff");
	if (pwr_off_np) {
		if (!cpu_is_imx6sx()) {
			printk("[UDOO power-off] Probed UDOO Quad/Dual.\n");
			pwr_5v_gpio = of_get_named_gpio(pwr_off_np, "pwr_5v_gpio", 0);
			of_node_put(pwr_off_np);
			udoo_request_gpio(&pdev->dev, pwr_5v_gpio, GPIOF_OUT_INIT_LOW, "pwr_5v_gpio");
		}

		pm_power_off_orig = pm_power_off;
		pm_power_off = udoo_power_off;
		return 0;
	}

	/* If a pm_power_off function has already been added, leave it alone */
	if (pm_power_off != NULL) {
		printk("[UDOO power-off] pm_power_off already registered.\n");
		return -EBUSY;
	}

	return -ENODEV;
}

static int udoo_power_off_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id power_off_dt_ids[] = {
	{ .compatible = "udoo,poweroff", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, power_off_dt_ids);

static struct platform_driver udoo_power_off_driver = {
	.driver = {
		.name = "udoo_power_off",
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(power_off_dt_ids),
	},
	.probe    = udoo_power_off_probe,
	.remove   = udoo_power_off_remove,
};
module_platform_driver(udoo_power_off_driver);

MODULE_AUTHOR("Jasbir Matharu, Peter Vicman, Francesco Montefoschi");
MODULE_DESCRIPTION("UDOO Power off driver v4");
MODULE_LICENSE("GPL v2");
