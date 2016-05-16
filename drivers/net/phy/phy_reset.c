
/*	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
	GNU General Public License for more details.

	Seco A62 board's ETHERNET PHY LAYER reset support, based on Markus Pargmann approach.
*/

#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/fec.h>

struct phy_reset_cfg {
	struct pinctrl *pctl;
	struct pinctrl_state *pstate_reset;
	struct pinctrl_state *pstate_run;
	int gpio_rgmii;
	int gpio_reset;
};

static int phy_parse_pinctl(struct device *dev,
		struct phy_reset_cfg *cfg)
{
	struct pinctrl *p;
	struct pinctrl_state *state;
	int gpio;
	int ret;

	p = devm_pinctrl_get(dev);
	if (IS_ERR(p)) {
		dev_err(dev, "Failed to get pinctrl\n");
		return PTR_ERR(p);
	}
	cfg->pctl = p;

	state = pinctrl_lookup_state(p, "phy-reset");
	if (IS_ERR(state)) {
		dev_err(dev, "Can't find pinctrl state phy-reset\n");
		return PTR_ERR(state);
	}
	cfg->pstate_reset = state;
	
	state = pinctrl_lookup_state(p, "phy-running");
	if (IS_ERR(state)) {
		dev_err(dev, "Can't find pinctrl state phy-running\n");
		return PTR_ERR(state);
	}
	cfg->pstate_run = state;

	gpio = of_get_named_gpio(dev->of_node, "phy-gpios", 0);
	if (gpio < 0) {
		dev_err(dev, "Can't find phy-rgmii gpio\n");
		return gpio;
	}
	ret = devm_gpio_request(dev, gpio, "phy rgmii");
	if (ret) {
		dev_err(dev, "Failed requesting phy-rgmii gpio\n");
		return ret;
	}
	cfg->gpio_rgmii = gpio;

	gpio = of_get_named_gpio(dev->of_node, "phy-gpios", 1);
	if (gpio < 0) {
		dev_err(dev, "Can't find phy-reset gpio %d\n", gpio);
		return gpio;
	}
	ret = devm_gpio_request(dev, gpio, "phy reset");
	if (ret) {
		dev_err(dev, "Failed requesting phy-reset gpio\n");
		return ret;
	}
	
	cfg->gpio_reset = gpio;
	printk("PHY reset pinctl parsing ok!!!\n");
	return 0;
}

int fec_phy_reset(struct platform_device *pdev)
{	
	struct phy_reset_cfg cfg;
	struct pinctrl *p; 
	int ret;

	ret = phy_parse_pinctl(&pdev->dev, &cfg);
	if (ret)
		return ret;

	p = cfg.pctl;
	pinctrl_select_state(p, cfg.pstate_reset);

	gpio_direction_output(cfg.gpio_rgmii, 1);
	gpio_set_value(cfg.gpio_rgmii, 1);
	gpio_direction_output(cfg.gpio_reset, 1);
	gpio_set_value(cfg.gpio_reset, 0);

	msleep(2);
	gpio_set_value(cfg.gpio_reset, 1);
	msleep(1);	

	pinctrl_select_state(p, cfg.pstate_run);
	//msleep(1);
	
	return 0;
}
EXPORT_SYMBOL_GPL(fec_phy_reset);

