/*
 * Copyright 2011-2015 Freescale Semiconductor, Inc.
 * Copyright 2011 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/cpu.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/pci.h>
#include <linux/phy.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/micrel_phy.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/of_net.h>
#include <linux/fec.h>
#include <linux/netdevice.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"
#include "apx_wdog-trigger.h"

static struct fec_platform_data fec_pdata;
static struct flexcan_platform_data flexcan_pdata[2];
static int flexcan_en_gpio;
static int flexcan_stby_gpio;
static int flexcan0_en;
static int flexcan1_en;



static char *audio_options __read_mostly;
static int __init early_audio_codec (char *options) {
	audio_options = options;
	return 0;
}
early_param("audio_codec", early_audio_codec);


/*  For 928, selection between UART4 and CAN1 interface */
/*  For A75, selection between UART1 and CAN1 interface */
static char *serial_options __read_mostly;
static int __init early_serial_device (char *options) {
	serial_options = options;
	return 0;
}
early_param("serial_dev", early_serial_device);




static void imx6q_fec_sleep_enable(int enabled)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (enabled)
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ,
					   IMX6Q_GPR13_ENET_STOP_REQ);
		else
			regmap_update_bits(gpr, IOMUXC_GPR13,
					   IMX6Q_GPR13_ENET_STOP_REQ, 0);
	} else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");
}

static void __init imx6q_enet_plt_init(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02100000/ethernet@02188000");
	if (np && of_get_property(np, "fsl,magic-packet", NULL))
		fec_pdata.sleep_mode_enable = imx6q_fec_sleep_enable;
}

static void mx6q_flexcan_switch(void)
{
	if (flexcan0_en || flexcan1_en) {
		/*
		 * The transceiver TJA1041A on sabreauto RevE baseboard will
		 * fail to transit to Normal state if EN/STBY is high by default
		 * after board power up. So we set the EN/STBY initial state to low
		 * first then to high to guarantee the state transition successfully.
		 */
		gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);

		gpio_set_value_cansleep(flexcan_en_gpio, 1);
		gpio_set_value_cansleep(flexcan_stby_gpio, 1);
	} else {
		/*
		 * avoid to disable CAN xcvr if any of the CAN interfaces
		 * are down. XCRV will be disabled only if both CAN2
		 * interfaces are DOWN.
		*/
		gpio_set_value_cansleep(flexcan_en_gpio, 0);
		gpio_set_value_cansleep(flexcan_stby_gpio, 0);
	}
}

static void imx6q_flexcan0_switch_auto(int enable)
{
	flexcan0_en = enable;
	mx6q_flexcan_switch();
}

static void imx6q_flexcan1_switch_auto(int enable)
{
	flexcan1_en = enable;
	mx6q_flexcan_switch();
}

static int __init imx6q_flexcan_fixup_auto(void)
{
	struct device_node *np;

	np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
	if (!np)
		return -ENODEV;

	flexcan_en_gpio = of_get_named_gpio(np, "trx-en-gpio", 0);
	flexcan_stby_gpio = of_get_named_gpio(np, "trx-stby-gpio", 0);
	if (gpio_is_valid(flexcan_en_gpio) && gpio_is_valid(flexcan_stby_gpio) &&
		!gpio_request_one(flexcan_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en") &&
		!gpio_request_one(flexcan_stby_gpio, GPIOF_DIR_OUT, "flexcan-trx-stby")) {
		/* flexcan 0 & 1 are using the same GPIOs for transceiver */
		flexcan_pdata[0].transceiver_switch = imx6q_flexcan0_switch_auto;
		flexcan_pdata[1].transceiver_switch = imx6q_flexcan1_switch_auto;
	}

	return 0;
}

static inline int __init set_device_mode (const char *node_name, const char *sel_mode) {
	struct device_node *np, *entry;
	struct device_node *def_mode;
	struct device_node *phandle_enable = NULL;
	struct device_node *phandle_set = NULL;
	struct device_node *phandle_source = NULL;
	int num_code, err, use_default, num_element, i;
	const char *code_name, *def_code_name;
	const char *en_code = "okay";

	np = of_find_node_by_path(node_name);
	if ( !np ) {
		pr_warn ("%s: failed to get %s structure\n", __func__, node_name);
		return -ENODEV;
	}

	entry = of_parse_phandle (np, "default_mode", 0);
	if ( !entry ) {
		pr_err ("%s: no default mode given!!!\n", of_node_full_name(np));
		err = -1;
		goto fail_device_def;
	}
	def_mode = entry;

	if ( sel_mode )
		use_default = 0;
	else {
		err = of_property_read_string (def_mode, "code_name", &def_code_name);
		if ( err ) {
			pr_err ("%s: error reading code_name string\n", of_node_full_name(def_mode));
			goto fail_device_def;
		}
		pr_info ("%s, use default mode: %s\n", of_node_full_name(np),
				def_code_name);
		use_default = 1;
	}

	num_code = of_get_child_count (np);
	if ( num_code == 0 ) {
		pr_err ("%s: no device modes specified!!!\n", of_node_full_name(np));
		goto fail_device_def;
	}

	for_each_child_of_node (np, entry) {

		err = of_property_read_string (entry, "code_name", &code_name);
		if ( err ) {
			pr_err ("%s error reading codec element!!!\n",
						of_node_full_name(entry));
			goto fail_entry_codec;
		}

		pr_info ("%s, code name: %s\n",  of_node_full_name(np), code_name);

		if ( !strcasecmp (code_name, use_default == 0 ?
					sel_mode : def_code_name) ) {
			pr_info ("%s, found selected mode\n", of_node_full_name(np));

			err = of_property_read_u32 (entry, "phandle-num-enable",
				   	(u32 *)&num_element);
			if ( err ) {
				pr_err ("%s error reading phanlde enable num element!!!\n",
						of_node_full_name(entry));
				goto fail_entry_codec;
			}

			for ( i = 0 ; i < num_element ; i++ ) {
				phandle_enable = of_parse_phandle (entry, "phandle-list-enable", i);
				if ( !phandle_enable ) {
					pr_err ("%s, failed to obtain phandle\n", of_node_full_name(entry));
					return -ENODEV;
				}
				err = of_property_write_string (phandle_enable, "status", en_code);
				if ( err ) {
					pr_err ("%s, phandle status enable error!!!\n",
							of_node_full_name(phandle_enable));
					goto fail_phandle_codec;
				}
				of_node_put (phandle_enable);
			}


			err = of_property_read_u32 (entry, "phandle-num-set",
				  	(u32 *)&num_element);
			if ( err ) {
				pr_err ("%s error reading phanlde set num element!!!\n",
						of_node_full_name(entry));
					goto fail_phandle_set;
			}

			for ( i = 0 ; i < num_element ; i++ ) {
				phandle_set = of_parse_phandle (entry, "phandle-list-set", i);
				phandle_source = of_parse_phandle (entry, "phandle-list-source", i);
				if ( !phandle_set || !phandle_source ) {
					pr_err ("%s, failed to obtain phandle source/set\n", of_node_full_name(entry));
					return -ENODEV;
				}

				of_concat_property (phandle_set, phandle_source);

				of_node_put (phandle_source);
				of_node_put (phandle_set);
			}


			break;
		}
	}

	of_node_put (def_mode);
	return 0;
fail_phandle_set:
	err = -1;
fail_phandle_codec:
	of_node_put (phandle_enable);
fail_entry_codec:
	of_node_put (entry);
fail_device_def:
	of_node_put (np);
	return err;
}


/* For imx6q sabrelite board: set KSZ9021RN RGMII pad skew */
static int ksz9021rn_phy_fixup(struct phy_device *phydev)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/* min rx data delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_RX_DATA_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0x0000);

		/* max rx/tx clock delay, min rx/tx control delay */
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			0x8000 | MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_DATA_WRITE, 0xf0f0);
		phy_write(phydev, MICREL_KSZ9021_EXTREG_CTRL,
			MICREL_KSZ9021_RGMII_CLK_CTRL_PAD_SCEW);
	}

	return 0;
}

static void mmd_write_reg(struct phy_device *dev, int device, int reg, int val)
{
	phy_write(dev, 0x0d, device);
	phy_write(dev, 0x0e, reg);
	phy_write(dev, 0x0d, (1 << 14) | device);
	phy_write(dev, 0x0e, val);
}

static int ksz9031rn_phy_fixup(struct phy_device *dev)
{
	/*
	 * min rx data delay, max rx/tx clock delay,
	 * min rx/tx control delay
	 */
	mmd_write_reg(dev, 2, 4, 0);
	mmd_write_reg(dev, 2, 5, 0);
	mmd_write_reg(dev, 2, 8, 0x003ff);

	return 0;
}


static int ksz8091rn_phy_fixup(struct phy_device *dev)
{
	return 0;
}


/*
 * fixup for PLX PEX8909 bridge to configure GPIO1-7 as output High
 * as they are used for slots1-7 PERST#
 */
static void ventana_pciesw_early_fixup(struct pci_dev *dev)
{
	u32 dw;

	if (!of_machine_is_compatible("gw,ventana"))
		return;

	if (dev->devfn != 0)
		return;

	pci_read_config_dword(dev, 0x62c, &dw);
	dw |= 0xaaa8; // GPIO1-7 outputs
	pci_write_config_dword(dev, 0x62c, dw);

	pci_read_config_dword(dev, 0x644, &dw);
	dw |= 0xfe;   // GPIO1-7 output high
	pci_write_config_dword(dev, 0x644, dw);

	msleep(100);
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8609, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8606, ventana_pciesw_early_fixup);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_PLX, 0x8604, ventana_pciesw_early_fixup);

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	/* To enable AR8031 output a 125MHz clk from CLK_25M */
	phy_write(dev, 0xd, 0x7);
	phy_write(dev, 0xe, 0x8016);
	phy_write(dev, 0xd, 0x4007);

	val = phy_read(dev, 0xe);
	val &= 0xffe3;
	val |= 0x18;
	phy_write(dev, 0xe, val);

	/* introduce tx clock delay */
	phy_write(dev, 0x1d, 0x5);
	val = phy_read(dev, 0x1e);
	val |= 0x0100;
	phy_write(dev, 0x1e, val);

	return 0;
}

#define PHY_ID_AR8031	0x004dd074

static int ar8035_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	 * which cause ethernet link down/up issue, so disable SmartEEE
	 */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);

	val = phy_read(dev, 0xe);
	phy_write(dev, 0xe, val & ~(1 << 8));

	/*
	 * Enable 125MHz clock from CLK_25M on the AR8031.  This
	 * is fed in to the IMX6 on the ENET_REF_CLK (V22) pad.
	 * Also, introduce a tx clock delay.
	 *
	 * This is the same as is the AR8031 fixup.
	 */
	ar8031_phy_fixup(dev);

	/*check phy power*/
	val = phy_read(dev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(dev, 0x0, val & ~BMCR_PDOWN);

	return 0;
}

#define PHY_ID_AR8035 0x004dd072

static void __init imx6q_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_KSZ9021, MICREL_PHY_ID_MASK,
				ksz9021rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ9031, MICREL_PHY_ID_MASK,
				ksz9031rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_KSZ8091, MICREL_PHY_ID_MASK,
				ksz8091rn_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
				ar8031_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8035, 0xffffffef,
				ar8035_phy_fixup);
	}
}

static void __init imx6q_1588_init(void)
{
	struct device_node *np;
	struct clk *ptp_clk;
	struct regmap *gpr;
	const char *pm;
	int err, clk_out = 1;


	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-fec");
	if (!np) {
		pr_warn("%s: failed to find fec node\n", __func__);
		return;
	}

	ptp_clk = of_clk_get(np, 2);
	if (IS_ERR(ptp_clk)) {
		pr_warn("%s: failed to get ptp clock\n", __func__);
		goto put_node;
	}

	err = of_property_read_string (np, "phy-mode", &pm);
	if (err < 0)
		return err;

	if ( !strcasecmp (pm, "rmii") )
		clk_out = 0;

	/*
	 * If enet_ref from ANATOP/CCM is the PTP clock source, we need to
	 * set bit IOMUXC_GPR1[21].  Or the PTP clock must be from pad
	 * (external OSC), and we need to clear the bit.
	 * If we have a RMII phy we want to get enet tx reference clk 
	 * from pad (external OSC for both external PHY and Internal Controller)
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {

		if ( clk_out )
			regmap_update_bits(gpr, IOMUXC_GPR1,
					IMX6Q_GPR1_ENET_CLK_SEL_MASK, 
					IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);
		else
			regmap_update_bits(gpr, IOMUXC_GPR1,
					IMX6Q_GPR1_ENET_CLK_SEL_MASK, 
					~IMX6Q_GPR1_ENET_CLK_SEL_ANATOP);

	} else
		pr_err("failed to find fsl,imx6q-iomux-gpr regmap\n");

	clk_put(ptp_clk);
put_node:
	of_node_put(np);
}

static void __init imx6q_csi_mux_init(void)
{
	/*
	 * MX6Q SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR1 bit 19 to 0x1.
	 *
	 * MX6DL SabreSD board:
	 * IPU1 CSI0 connects to parallel interface.
	 * Set GPR13 bit 0-2 to 0x4.
	 * IPU1 CSI1 connects to MIPI CSI2 virtual channel 1.
	 * Set GPR13 bit 3-5 to 0x1.
	 */
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		if (of_machine_is_compatible("fsl,imx6q-sabresd") ||
			of_machine_is_compatible("fsl,imx6q-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR1, 1 << 19, 1 << 19);
		else if (of_machine_is_compatible("fsl,imx6dl-sabresd") ||
			 of_machine_is_compatible("fsl,imx6dl-sabreauto"))
			regmap_update_bits(gpr, IOMUXC_GPR13, 0x3F, 0x0C);
	} else {
		pr_err("%s(): failed to find fsl,imx6q-iomux-gpr regmap\n",
		       __func__);
	}
}

#define OCOTP_MACn(n)	(0x00000620 + (n) * 0x10)
void __init imx6_enet_mac_init(const char *compatible)
{
	struct device_node *ocotp_np, *enet_np, *from = NULL;
	void __iomem *base;
	struct property *newmac;
	u32 macaddr_low;
	u32 macaddr_high = 0;
	u32 macaddr1_high = 0;
	u8 *macaddr;
	int i;

	for (i = 0; i < 2; i++) {
		enet_np = of_find_compatible_node(from, NULL, compatible);
		if (!enet_np)
			return;

		from = enet_np;

		if (of_get_mac_address(enet_np))
			goto put_enet_node;

		ocotp_np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
		if (!ocotp_np) {
			pr_warn("failed to find ocotp node\n");
			goto put_enet_node;
		}

		base = of_iomap(ocotp_np, 0);
		if (!base) {
			pr_warn("failed to map ocotp\n");
			goto put_ocotp_node;
		}

		macaddr_low = readl_relaxed(base + OCOTP_MACn(1));
		if (i)
			macaddr1_high = readl_relaxed(base + OCOTP_MACn(2));
		else
			macaddr_high = readl_relaxed(base + OCOTP_MACn(0));

		newmac = kzalloc(sizeof(*newmac) + 6, GFP_KERNEL);
		if (!newmac)
			goto put_ocotp_node;

		newmac->value = newmac + 1;
		newmac->length = 6;
		newmac->name = kstrdup("local-mac-address", GFP_KERNEL);
		if (!newmac->name) {
			kfree(newmac);
			goto put_ocotp_node;
		}

		macaddr = newmac->value;
		if (i) {
			macaddr[5] = (macaddr_low >> 16) & 0xff;
			macaddr[4] = (macaddr_low >> 24) & 0xff;
			macaddr[3] = macaddr1_high & 0xff;
			macaddr[2] = (macaddr1_high >> 8) & 0xff;
			macaddr[1] = (macaddr1_high >> 16) & 0xff;
			macaddr[0] = (macaddr1_high >> 24) & 0xff;
		} else {
			macaddr[5] = macaddr_high & 0xff;
			macaddr[4] = (macaddr_high >> 8) & 0xff;
			macaddr[3] = (macaddr_high >> 16) & 0xff;
			macaddr[2] = (macaddr_high >> 24) & 0xff;
			macaddr[1] = macaddr_low & 0xff;
			macaddr[0] = (macaddr_low >> 8) & 0xff;
		}

		of_update_property(enet_np, newmac);

put_ocotp_node:
	of_node_put(ocotp_np);
put_enet_node:
	of_node_put(enet_np);
	}
}

static inline void imx6q_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx6q-fec");
	imx6q_enet_phy_init();
	imx6q_1588_init();
	imx6q_enet_plt_init();
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6q_auxdata_lookup[] __initconst = {
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02090000, NULL, &flexcan_pdata[0]),
	OF_DEV_AUXDATA("fsl,imx6q-flexcan", 0x02094000, NULL, &flexcan_pdata[1]),
	OF_DEV_AUXDATA("fsl,imx6q-fec", 0x02188000, NULL, &fec_pdata),
	{ /* sentinel */ }
};

static const struct apx_wdog_trigger_data apx_wdt __initconst = {
	.gpio_trg__iomux_ctrl   = 0x020e00b8,
	.gpio_trg__pad_ctrl     = 0x20e03cc,
	.gpio_trg__base         = 0x020a4000,
	.gpio_trg__num           = 25,
	.gpio_en__iomux_ctrl    = 0x20e020c,
	.gpio_en__pad_ctrl      = 0x20e05dc,
	.gpio_en__base          = 0x020a8000,
	.gpio_en__num           = 11,
};

static void __init imx6q_init_machine(void)
{
	struct device *parent;
	struct device_node *np = NULL;
	struct clk *p_clk = NULL;
	int ret;

	if ( of_machine_is_compatible("fsl,imx6q-SBC_A62") ||
			of_machine_is_compatible("fsl,imx6dl-SBC_A62") ) {

		apx_wdog_trigger_early_init (&apx_wdt, 0);

	}

	imx_print_silicon_rev(cpu_is_imx6dl() ? "i.MX6DL" : "i.MX6Q",
			      imx_get_soc_revision());

	mxc_arch_reset_init_dt();

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_populate(NULL, of_default_bus_match_table,
					imx6q_auxdata_lookup, parent);

	imx6q_enet_init();
	imx_anatop_init();
	imx6q_csi_mux_init();
	cpu_is_imx6q() ?  imx6q_pm_init() : imx6dl_pm_init();

	if ( of_machine_is_compatible("fsl,imx6q-SBC_A62") ||
			of_machine_is_compatible("fsl,imx6dl-SBC_A62") ) {

		apx_wdog_trigger_work_init(0);

		/*  set clock CKO2 to use the USBH1 with external clock  */
		np = of_find_node_by_path("/external_clocks");
		if ( !np ) {
			pr_warn ("%s: failed to get %s structure\n", __func__, "/external_clocks");
			goto put_node;
		}
		   p_clk = of_clk_get (np, 0);
		if ( IS_ERR(p_clk) ) {
			pr_warn("%s: failed to get clock\n", __func__);
			goto put_node;
		}
		ret = clk_prepare_enable (p_clk);
		if (ret) {
			pr_err ("can't enable clock\n");
		} else {
			pr_info ("clock enabled\n");
		}
	}

	clk_put (p_clk);
put_node:
	of_node_put(np);

}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_1P2GHZ		0x3
#define OCOTP_CFG3_SPEED_996MHZ		0x2
#define OCOTP_CFG3_SPEED_852MHZ		0x1

static void __init imx6q_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-ocotp");
	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * SPEED_GRADING[1:0] defines the max speed of ARM:
	 * 2b'11: 1200000000Hz;
	 * 2b'10: 996000000Hz;
	 * 2b'01: 852000000Hz; -- i.MX6Q Only, exclusive with 996MHz.
	 * 2b'00: 792000000Hz;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;

	if (val != OCOTP_CFG3_SPEED_1P2GHZ)
		if (dev_pm_opp_disable(cpu_dev, 1200000000))
			pr_warn("failed to disable 1.2 GHz OPP\n");
	if (val < OCOTP_CFG3_SPEED_996MHZ)
		if (dev_pm_opp_disable(cpu_dev, 996000000))
			pr_warn("failed to disable 996 MHz OPP\n");
	if (cpu_is_imx6q()) {
		if (val != OCOTP_CFG3_SPEED_852MHZ)
			if (dev_pm_opp_disable(cpu_dev, 852000000))
				pr_warn("failed to disable 852 MHz OPP\n");
	}

	if (IS_ENABLED(CONFIG_MX6_VPU_352M)) {
		if (dev_pm_opp_disable(cpu_dev, 396000000))
			pr_warn("failed to disable 396MHz OPP\n");
		pr_info("remove 396MHz OPP for VPU running at 352MHz!\n");
	}

put_node:
	of_node_put(np);
}

static void __init imx6q_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (of_init_opp_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6q_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static struct platform_device imx6q_cpufreq_pdev = {
	.name = "imx6q-cpufreq",
};

static void __init imx6q_init_late(void)
{
	struct device_node *np;
	int can_en_gpio;


	/*
	 * WAIT mode is broken on TO 1.0 and 1.1, so there is no point
	 * to run cpuidle on them.
	 */
	if ((cpu_is_imx6q() && imx_get_soc_revision() > IMX_CHIP_REVISION_1_1)
		|| (cpu_is_imx6dl() && imx_get_soc_revision() >
		IMX_CHIP_REVISION_1_0))
		imx6q_cpuidle_init();

	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6q_opp_init();
		platform_device_register(&imx6q_cpufreq_pdev);
	}

	if (of_machine_is_compatible("fsl,imx6q-sabreauto")
		|| of_machine_is_compatible("fsl,imx6dl-sabreauto"))
		imx6q_flexcan_fixup_auto();


	if ( of_machine_is_compatible("fsl,imx6q-quadmo747_928")
		|| of_machine_is_compatible("fsl,imx6dl-quadmo747_928")
		|| of_machine_is_compatible("fsl,imx6q-uq7_962")
		|| of_machine_is_compatible("fsl,imx6dl-uq7_962"))
	{
		np = of_find_node_by_path ("/serial_switch");
		if ( np ) {

			can_en_gpio = of_get_named_gpio (np, "selector", 0);

			if ( gpio_is_valid(can_en_gpio) &&
				!gpio_request_one(can_en_gpio, GPIOF_DIR_OUT, "can-en") ) {

				if ( strcmp (serial_options, "flexcan") == 0 )
					gpio_set_value_cansleep(can_en_gpio, 0);
				else
					gpio_set_value_cansleep(can_en_gpio, 1);
			}

		}
	}
}

static void __init imx6q_map_io(void)
{
	debug_ll_io_init();
	imx_scu_map_io();
	imx6_pm_map_io();
	imx6_busfreq_map_io();
}

static void __init imx6q_init_irq(void)
{
	imx_init_revision_from_anatop();
	imx_init_l2cache();
	imx_src_init();
	imx_gpc_init();
	irqchip_init();
}



static void __init imx6q_init_early (void) {

	if (of_machine_is_compatible("fsl,imx6q-quadmo747_928")
		|| of_machine_is_compatible("fsl,imx6dl-quadmo747_928")
		|| of_machine_is_compatible("fsl,imx6q-uq7_962")
		|| of_machine_is_compatible("fsl,imx6dl-uq7_962")
		|| of_machine_is_compatible("fsl,imx6q-uq7-j_A75")
		|| of_machine_is_compatible("fsl,imx6dl-uq7-j_A75"))

	{
		set_device_mode ("/dynamic_choice/chip_audio", audio_options);
	}

	if ( of_machine_is_compatible("fsl,imx6q-uq7-j_A75")
		|| of_machine_is_compatible("fsl,imx6dl-uq7-j_A75"))
	{
		set_device_mode ("/dynamic_choice/serial_device", serial_options);
	}

	if ( of_machine_is_compatible("fsl,imx6q-quadmo747_928")
		|| of_machine_is_compatible("fsl,imx6dl-quadmo747_928")
		|| of_machine_is_compatible("fsl,imx6q-uq7_962")
		|| of_machine_is_compatible("fsl,imx6dl-uq7_962"))
	{
		set_device_mode ("/dynamic_choice/serial_device", serial_options);
	}
}


static const char *imx6q_dt_compat[] __initdata = {
	"fsl,imx6dl",
	"fsl,imx6q",
	NULL,
};

DT_MACHINE_START(IMX6Q, "Freescale i.MX6 Quad/DualLite (Device Tree)")
	/*
	 * i.MX6Q/DL maps system memory at 0x10000000 (offset 256MiB), and
	 * GPU has a limit on physical address that it accesses, which must
	 * be below 2GiB.
	 */
	.dma_zone_size	= (SZ_2G - SZ_256M),
	.smp		= smp_ops(imx_smp_ops),
	.map_io		= imx6q_map_io,
	.init_early     = imx6q_init_early,
	.init_irq	= imx6q_init_irq,
	.init_machine	= imx6q_init_machine,
	.init_late      = imx6q_init_late,
	.dt_compat	= imx6q_dt_compat,
	.restart	= mxc_restart,
MACHINE_END
