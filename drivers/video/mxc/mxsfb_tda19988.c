/*
 * Copyright (C) 2015 Jasbir Matharu.
 * Copyright (C) Texas Instruments
 * 
 * This is a short term workaround for initialising the tda19988 
 * on the UDO Neo. We rely on the MX28 LCD controller for creating the 
 * framebuffer and setting the resolution. This driver piggy backs 
 * from those settings. Longer term this needs to be replaced with a
 * KMS driver for MX28 LCD and use the existing tda998x kernel driver.
 * A large portion of this code is taken from the tda998x_drv.c kernel 
 * driver originally developed by Rob Clark <robdclark@gmail.com>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
 
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/fsl_devices.h>
#include <linux/interrupt.h>
#include <linux/reset.h>
#include <linux/hdmi.h>
#include <asm/mach-types.h>
#include <sound/asoundef.h>

#define REG(page, addr) (((page) << 8) | (addr))
#define REG2ADDR(reg)   ((reg) & 0xff)
#define REG2PAGE(reg)   (((reg) >> 8) & 0xff)

#define REG_CURPAGE               0xff                /* write */

/* Page 00h: General Control */
#define REG_VERSION_LSB           REG(0x00, 0x00)     /* read */
#define REG_MAIN_CNTRL0           REG(0x00, 0x01)     /* read/write */
# define MAIN_CNTRL0_SR           (1 << 0)
# define MAIN_CNTRL0_DECS         (1 << 1)
# define MAIN_CNTRL0_DEHS         (1 << 2)
# define MAIN_CNTRL0_CECS         (1 << 3)
# define MAIN_CNTRL0_CEHS         (1 << 4)
# define MAIN_CNTRL0_SCALER       (1 << 7)
#define REG_VERSION_MSB           REG(0x00, 0x02)     /* read */
#define REG_SOFTRESET             REG(0x00, 0x0a)     /* write */
# define SOFTRESET_AUDIO          (1 << 0)
# define SOFTRESET_I2C_MASTER     (1 << 1)
#define REG_DDC_DISABLE           REG(0x00, 0x0b)     /* read/write */
#define REG_CCLK_ON               REG(0x00, 0x0c)     /* read/write */
#define REG_I2C_MASTER            REG(0x00, 0x0d)     /* read/write */
# define I2C_MASTER_DIS_MM        (1 << 0)
# define I2C_MASTER_DIS_FILT      (1 << 1)
# define I2C_MASTER_APP_STRT_LAT  (1 << 2)
#define REG_FEAT_POWERDOWN        REG(0x00, 0x0e)     /* read/write */
# define FEAT_POWERDOWN_SPDIF     (1 << 3)
#define REG_INT_FLAGS_0           REG(0x00, 0x0f)     /* read/write */
#define REG_INT_FLAGS_1           REG(0x00, 0x10)     /* read/write */
#define REG_INT_FLAGS_2           REG(0x00, 0x11)     /* read/write */
# define INT_FLAGS_2_EDID_BLK_RD  (1 << 1)
#define REG_ENA_ACLK              REG(0x00, 0x16)     /* read/write */
#define REG_ENA_VP_0              REG(0x00, 0x18)     /* read/write */
#define REG_ENA_VP_1              REG(0x00, 0x19)     /* read/write */
#define REG_ENA_VP_2              REG(0x00, 0x1a)     /* read/write */
#define REG_ENA_AP                REG(0x00, 0x1e)     /* read/write */
#define REG_VIP_CNTRL_0           REG(0x00, 0x20)     /* write */
# define VIP_CNTRL_0_MIRR_A       (1 << 7)
# define VIP_CNTRL_0_SWAP_A(x)    (((x) & 7) << 4)
# define VIP_CNTRL_0_MIRR_B       (1 << 3)
# define VIP_CNTRL_0_SWAP_B(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_1           REG(0x00, 0x21)     /* write */
# define VIP_CNTRL_1_MIRR_C       (1 << 7)
# define VIP_CNTRL_1_SWAP_C(x)    (((x) & 7) << 4)
# define VIP_CNTRL_1_MIRR_D       (1 << 3)
# define VIP_CNTRL_1_SWAP_D(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_2           REG(0x00, 0x22)     /* write */
# define VIP_CNTRL_2_MIRR_E       (1 << 7)
# define VIP_CNTRL_2_SWAP_E(x)    (((x) & 7) << 4)
# define VIP_CNTRL_2_MIRR_F       (1 << 3)
# define VIP_CNTRL_2_SWAP_F(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_3           REG(0x00, 0x23)     /* write */
# define VIP_CNTRL_3_X_TGL        (1 << 0)
# define VIP_CNTRL_3_H_TGL        (1 << 1)
# define VIP_CNTRL_3_V_TGL        (1 << 2)
# define VIP_CNTRL_3_EMB          (1 << 3)
# define VIP_CNTRL_3_SYNC_DE      (1 << 4)
# define VIP_CNTRL_3_SYNC_HS      (1 << 5)
# define VIP_CNTRL_3_DE_INT       (1 << 6)
# define VIP_CNTRL_3_EDGE         (1 << 7)
#define REG_VIP_CNTRL_4           REG(0x00, 0x24)     /* write */
# define VIP_CNTRL_4_BLC(x)       (((x) & 3) << 0)
# define VIP_CNTRL_4_BLANKIT(x)   (((x) & 3) << 2)
# define VIP_CNTRL_4_CCIR656      (1 << 4)
# define VIP_CNTRL_4_656_ALT      (1 << 5)
# define VIP_CNTRL_4_TST_656      (1 << 6)
# define VIP_CNTRL_4_TST_PAT      (1 << 7)
#define REG_VIP_CNTRL_5           REG(0x00, 0x25)     /* write */
# define VIP_CNTRL_5_CKCASE       (1 << 0)
# define VIP_CNTRL_5_SP_CNT(x)    (((x) & 3) << 1)
#define REG_MUX_AP                REG(0x00, 0x26)     /* read/write */
# define MUX_AP_SELECT_I2S	  0x64
# define MUX_AP_SELECT_SPDIF	  0x40
#define REG_MUX_VP_VIP_OUT        REG(0x00, 0x27)     /* read/write */
#define REG_MAT_CONTRL            REG(0x00, 0x80)     /* write */
# define MAT_CONTRL_MAT_SC(x)     (((x) & 3) << 0)
# define MAT_CONTRL_MAT_BP        (1 << 2)
#define REG_VIDFORMAT             REG(0x00, 0xa0)     /* write */
#define REG_REFPIX_MSB            REG(0x00, 0xa1)     /* write */
#define REG_REFPIX_LSB            REG(0x00, 0xa2)     /* write */
#define REG_REFLINE_MSB           REG(0x00, 0xa3)     /* write */
#define REG_REFLINE_LSB           REG(0x00, 0xa4)     /* write */
#define REG_NPIX_MSB              REG(0x00, 0xa5)     /* write */
#define REG_NPIX_LSB              REG(0x00, 0xa6)     /* write */
#define REG_NLINE_MSB             REG(0x00, 0xa7)     /* write */
#define REG_NLINE_LSB             REG(0x00, 0xa8)     /* write */
#define REG_VS_LINE_STRT_1_MSB    REG(0x00, 0xa9)     /* write */
#define REG_VS_LINE_STRT_1_LSB    REG(0x00, 0xaa)     /* write */
#define REG_VS_PIX_STRT_1_MSB     REG(0x00, 0xab)     /* write */
#define REG_VS_PIX_STRT_1_LSB     REG(0x00, 0xac)     /* write */
#define REG_VS_LINE_END_1_MSB     REG(0x00, 0xad)     /* write */
#define REG_VS_LINE_END_1_LSB     REG(0x00, 0xae)     /* write */
#define REG_VS_PIX_END_1_MSB      REG(0x00, 0xaf)     /* write */
#define REG_VS_PIX_END_1_LSB      REG(0x00, 0xb0)     /* write */
#define REG_VS_LINE_STRT_2_MSB    REG(0x00, 0xb1)     /* write */
#define REG_VS_LINE_STRT_2_LSB    REG(0x00, 0xb2)     /* write */
#define REG_VS_PIX_STRT_2_MSB     REG(0x00, 0xb3)     /* write */
#define REG_VS_PIX_STRT_2_LSB     REG(0x00, 0xb4)     /* write */
#define REG_VS_LINE_END_2_MSB     REG(0x00, 0xb5)     /* write */
#define REG_VS_LINE_END_2_LSB     REG(0x00, 0xb6)     /* write */
#define REG_VS_PIX_END_2_MSB      REG(0x00, 0xb7)     /* write */
#define REG_VS_PIX_END_2_LSB      REG(0x00, 0xb8)     /* write */
#define REG_HS_PIX_START_MSB      REG(0x00, 0xb9)     /* write */
#define REG_HS_PIX_START_LSB      REG(0x00, 0xba)     /* write */
#define REG_HS_PIX_STOP_MSB       REG(0x00, 0xbb)     /* write */
#define REG_HS_PIX_STOP_LSB       REG(0x00, 0xbc)     /* write */
#define REG_VWIN_START_1_MSB      REG(0x00, 0xbd)     /* write */
#define REG_VWIN_START_1_LSB      REG(0x00, 0xbe)     /* write */
#define REG_VWIN_END_1_MSB        REG(0x00, 0xbf)     /* write */
#define REG_VWIN_END_1_LSB        REG(0x00, 0xc0)     /* write */
#define REG_VWIN_START_2_MSB      REG(0x00, 0xc1)     /* write */
#define REG_VWIN_START_2_LSB      REG(0x00, 0xc2)     /* write */
#define REG_VWIN_END_2_MSB        REG(0x00, 0xc3)     /* write */
#define REG_VWIN_END_2_LSB        REG(0x00, 0xc4)     /* write */
#define REG_DE_START_MSB          REG(0x00, 0xc5)     /* write */
#define REG_DE_START_LSB          REG(0x00, 0xc6)     /* write */
#define REG_DE_STOP_MSB           REG(0x00, 0xc7)     /* write */
#define REG_DE_STOP_LSB           REG(0x00, 0xc8)     /* write */
#define REG_TBG_CNTRL_0           REG(0x00, 0xca)     /* write */
# define TBG_CNTRL_0_TOP_TGL      (1 << 0)
# define TBG_CNTRL_0_TOP_SEL      (1 << 1)
# define TBG_CNTRL_0_DE_EXT       (1 << 2)
# define TBG_CNTRL_0_TOP_EXT      (1 << 3)
# define TBG_CNTRL_0_FRAME_DIS    (1 << 5)
# define TBG_CNTRL_0_SYNC_MTHD    (1 << 6)
# define TBG_CNTRL_0_SYNC_ONCE    (1 << 7)
#define REG_TBG_CNTRL_1           REG(0x00, 0xcb)     /* write */
# define TBG_CNTRL_1_H_TGL        (1 << 0)
# define TBG_CNTRL_1_V_TGL        (1 << 1)
# define TBG_CNTRL_1_TGL_EN       (1 << 2)
# define TBG_CNTRL_1_X_EXT        (1 << 3)
# define TBG_CNTRL_1_H_EXT        (1 << 4)
# define TBG_CNTRL_1_V_EXT        (1 << 5)
# define TBG_CNTRL_1_DWIN_DIS     (1 << 6)
#define REG_ENABLE_SPACE          REG(0x00, 0xd6)     /* write */
#define REG_HVF_CNTRL_0           REG(0x00, 0xe4)     /* write */
# define HVF_CNTRL_0_SM           (1 << 7)
# define HVF_CNTRL_0_RWB          (1 << 6)
# define HVF_CNTRL_0_PREFIL(x)    (((x) & 3) << 2)
# define HVF_CNTRL_0_INTPOL(x)    (((x) & 3) << 0)
#define REG_HVF_CNTRL_1           REG(0x00, 0xe5)     /* write */
# define HVF_CNTRL_1_FOR          (1 << 0)
# define HVF_CNTRL_1_YUVBLK       (1 << 1)
# define HVF_CNTRL_1_VQR(x)       (((x) & 3) << 2)
# define HVF_CNTRL_1_PAD(x)       (((x) & 3) << 4)
# define HVF_CNTRL_1_SEMI_PLANAR  (1 << 6)
#define REG_RPT_CNTRL             REG(0x00, 0xf0)     /* write */
#define REG_I2S_FORMAT            REG(0x00, 0xfc)     /* read/write */
# define I2S_FORMAT(x)            (((x) & 3) << 0)
#define REG_AIP_CLKSEL            REG(0x00, 0xfd)     /* write */
# define AIP_CLKSEL_AIP_SPDIF	  (0 << 3)
# define AIP_CLKSEL_AIP_I2S	  (1 << 3)
# define AIP_CLKSEL_FS_ACLK	  (0 << 0)
# define AIP_CLKSEL_FS_MCLK	  (1 << 0)
# define AIP_CLKSEL_FS_FS64SPDIF  (2 << 0)

/* Page 02h: PLL settings */
#define REG_PLL_SERIAL_1          REG(0x02, 0x00)     /* read/write */
# define PLL_SERIAL_1_SRL_FDN     (1 << 0)
# define PLL_SERIAL_1_SRL_IZ(x)   (((x) & 3) << 1)
# define PLL_SERIAL_1_SRL_MAN_IZ  (1 << 6)
#define REG_PLL_SERIAL_2          REG(0x02, 0x01)     /* read/write */
# define PLL_SERIAL_2_SRL_NOSC(x) ((x) << 0)
# define PLL_SERIAL_2_SRL_PR(x)   (((x) & 0xf) << 4)
#define REG_PLL_SERIAL_3          REG(0x02, 0x02)     /* read/write */
# define PLL_SERIAL_3_SRL_CCIR    (1 << 0)
# define PLL_SERIAL_3_SRL_DE      (1 << 2)
# define PLL_SERIAL_3_SRL_PXIN_SEL (1 << 4)
#define REG_SERIALIZER            REG(0x02, 0x03)     /* read/write */
#define REG_BUFFER_OUT            REG(0x02, 0x04)     /* read/write */
#define REG_PLL_SCG1              REG(0x02, 0x05)     /* read/write */
#define REG_PLL_SCG2              REG(0x02, 0x06)     /* read/write */
#define REG_PLL_SCGN1             REG(0x02, 0x07)     /* read/write */
#define REG_PLL_SCGN2             REG(0x02, 0x08)     /* read/write */
#define REG_PLL_SCGR1             REG(0x02, 0x09)     /* read/write */
#define REG_PLL_SCGR2             REG(0x02, 0x0a)     /* read/write */
#define REG_AUDIO_DIV             REG(0x02, 0x0e)     /* read/write */
# define AUDIO_DIV_SERCLK_1       0
# define AUDIO_DIV_SERCLK_2       1
# define AUDIO_DIV_SERCLK_4       2
# define AUDIO_DIV_SERCLK_8       3
# define AUDIO_DIV_SERCLK_16      4
# define AUDIO_DIV_SERCLK_32      5
#define REG_SEL_CLK               REG(0x02, 0x11)     /* read/write */
# define SEL_CLK_SEL_CLK1         (1 << 0)
# define SEL_CLK_SEL_VRF_CLK(x)   (((x) & 3) << 1)
# define SEL_CLK_ENA_SC_CLK       (1 << 3)
#define REG_ANA_GENERAL           REG(0x02, 0x12)     /* read/write */


/* Page 09h: EDID Control */
#define REG_EDID_DATA_0           REG(0x09, 0x00)     /* read */
/* next 127 successive registers are the EDID block */
#define REG_EDID_CTRL             REG(0x09, 0xfa)     /* read/write */
#define REG_DDC_ADDR              REG(0x09, 0xfb)     /* read/write */
#define REG_DDC_OFFS              REG(0x09, 0xfc)     /* read/write */
#define REG_DDC_SEGM_ADDR         REG(0x09, 0xfd)     /* read/write */
#define REG_DDC_SEGM              REG(0x09, 0xfe)     /* read/write */


/* Page 10h: information frames and packets */
#define REG_IF1_HB0               REG(0x10, 0x20)     /* read/write */
#define REG_IF2_HB0               REG(0x10, 0x40)     /* read/write */
#define REG_IF3_HB0               REG(0x10, 0x60)     /* read/write */
#define REG_IF4_HB0               REG(0x10, 0x80)     /* read/write */
#define REG_IF5_HB0               REG(0x10, 0xa0)     /* read/write */


/* Page 11h: audio settings and content info packets */
#define REG_AIP_CNTRL_0           REG(0x11, 0x00)     /* read/write */
# define AIP_CNTRL_0_RST_FIFO     (1 << 0)
# define AIP_CNTRL_0_SWAP         (1 << 1)
# define AIP_CNTRL_0_LAYOUT       (1 << 2)
# define AIP_CNTRL_0_ACR_MAN      (1 << 5)
# define AIP_CNTRL_0_RST_CTS      (1 << 6)
#define REG_CA_I2S                REG(0x11, 0x01)     /* read/write */
# define CA_I2S_CA_I2S(x)         (((x) & 31) << 0)
# define CA_I2S_HBR_CHSTAT        (1 << 6)
#define REG_LATENCY_RD            REG(0x11, 0x04)     /* read/write */
#define REG_ACR_CTS_0             REG(0x11, 0x05)     /* read/write */
#define REG_ACR_CTS_1             REG(0x11, 0x06)     /* read/write */
#define REG_ACR_CTS_2             REG(0x11, 0x07)     /* read/write */
#define REG_ACR_N_0               REG(0x11, 0x08)     /* read/write */
#define REG_ACR_N_1               REG(0x11, 0x09)     /* read/write */
#define REG_ACR_N_2               REG(0x11, 0x0a)     /* read/write */
#define REG_CTS_N                 REG(0x11, 0x0c)     /* read/write */
# define CTS_N_K(x)               (((x) & 7) << 0)
# define CTS_N_M(x)               (((x) & 3) << 4)
#define REG_ENC_CNTRL             REG(0x11, 0x0d)     /* read/write */
# define ENC_CNTRL_RST_ENC        (1 << 0)
# define ENC_CNTRL_RST_SEL        (1 << 1)
# define ENC_CNTRL_CTL_CODE(x)    (((x) & 3) << 2)
#define REG_DIP_FLAGS             REG(0x11, 0x0e)     /* read/write */
# define DIP_FLAGS_ACR            (1 << 0)
# define DIP_FLAGS_GC             (1 << 1)
#define REG_DIP_IF_FLAGS          REG(0x11, 0x0f)     /* read/write */
# define DIP_IF_FLAGS_IF1         (1 << 1)
# define DIP_IF_FLAGS_IF2         (1 << 2)
# define DIP_IF_FLAGS_IF3         (1 << 3)
# define DIP_IF_FLAGS_IF4         (1 << 4)
# define DIP_IF_FLAGS_IF5         (1 << 5)
#define REG_CH_STAT_B(x)          REG(0x11, 0x14 + (x)) /* read/write */


/* Page 12h: HDCP and OTP */
#define REG_TX3                   REG(0x12, 0x9a)     /* read/write */
#define REG_TX4                   REG(0x12, 0x9b)     /* read/write */
# define TX4_PD_RAM               (1 << 1)
#define REG_TX33                  REG(0x12, 0xb8)     /* read/write */
# define TX33_HDMI                (1 << 1)


/* Page 13h: Gamut related metadata packets */

/* CEC registers: (not paged)
 */
#define REG_CEC_INTSTATUS	  0xee		      /* read */
# define CEC_INTSTATUS_CEC	  (1 << 0)
# define CEC_INTSTATUS_HDMI	  (1 << 1)
#define REG_CEC_FRO_IM_CLK_CTRL   0xfb                /* read/write */
# define CEC_FRO_IM_CLK_CTRL_GHOST_DIS (1 << 7)
# define CEC_FRO_IM_CLK_CTRL_ENA_OTP   (1 << 6)
# define CEC_FRO_IM_CLK_CTRL_IMCLK_SEL (1 << 1)
# define CEC_FRO_IM_CLK_CTRL_FRO_DIV   (1 << 0)
#define REG_CEC_RXSHPDINTENA	  0xfc		      /* read/write */
#define REG_CEC_RXSHPDINT	  0xfd		      /* read */
#define REG_CEC_RXSHPDLEV         0xfe                /* read */
# define CEC_RXSHPDLEV_RXSENS     (1 << 0)
# define CEC_RXSHPDLEV_HPD        (1 << 1)

#define REG_CEC_ENAMODS           0xff                /* read/write */
# define CEC_ENAMODS_DIS_FRO      (1 << 6)
# define CEC_ENAMODS_DIS_CCLK     (1 << 5)
# define CEC_ENAMODS_EN_RXSENS    (1 << 2)
# define CEC_ENAMODS_EN_HDMI      (1 << 1)
# define CEC_ENAMODS_EN_CEC       (1 << 0)

#define REG_AVI_IF                REG(0x10, 0x40)   /* AVI Infoframe packet */
#define REG_AUDIO_IF              REG(0x10, 0x80)   /* AVI Infoframe packet */
#define SEL_AIP_I2S              (1 << 3)  /* I2S Clk */

#define DRV_NAME "tda19988"


struct tda19988_data {
	struct i2c_client *hdmi;
	struct i2c_client *cec;
    struct mutex mutex;
	struct delayed_work det_work;
	struct fb_info *fbi;
	bool waiting_for_fb;
	bool dft_mode_set;
	const char *mode_str;
	int bits_per_pixel;
	uint8_t current_page;
};

static struct tda19988_data *data = NULL;

extern const struct fb_videomode mxc_cea_mode[64];

/*
 * Ignore sync value when matching
 */
int mxc_lcd_fb_mode_is_equal(const struct fb_videomode *mode1,
			const struct fb_videomode *mode2)
{
	return (mode1->xres         == mode2->xres &&
		mode1->yres         == mode2->yres &&
		mode1->hsync_len    == mode2->hsync_len &&
		mode1->vsync_len    == mode2->vsync_len &&
		mode1->left_margin  == mode2->left_margin &&
		mode1->right_margin == mode2->right_margin &&
		mode1->upper_margin == mode2->upper_margin &&
		mode1->lower_margin == mode2->lower_margin &&
		/* refresh check, 59.94Hz and 60Hz have the same parameter
		 * in struct of mxc_cea_mode */
		abs(mode1->refresh - mode2->refresh) <= 1
	);
};

/*
 * Crude attempt at matching LCD mode to CEA mode
 */
int mxc_lcd_fb_var_to_vic(struct fb_var_screeninfo *var)
{
	int i;
	struct fb_videomode m;

	for (i = 0; i < ARRAY_SIZE(mxc_cea_mode); i++) {
		fb_var_to_videomode(&m, var);
		if (mxc_lcd_fb_mode_is_equal(&m, &mxc_cea_mode[i]))
			break;
	}

	if (i == ARRAY_SIZE(mxc_cea_mode))
		return 0;

	return i;
}

static void cec_write(struct tda19988_data *data, uint16_t addr, uint8_t val)
{
	struct i2c_client *client = data->cec;
    uint8_t buf[] = {addr, val};
    int ret;

	ret = i2c_master_send(client, buf, sizeof(buf));
    if (ret < 0)
		dev_err(&client->dev, "Error %d writing to cec:0x%x\n", ret, addr);
}

static int set_page(struct tda19988_data *data, uint16_t reg)
{
	if (REG2PAGE(reg) != data->current_page) {
		struct i2c_client *client = data->hdmi;
		uint8_t buf[] = { REG_CURPAGE, REG2PAGE(reg) };
		int ret = i2c_master_send(client, buf, sizeof(buf));
		if (ret < 0) {
			dev_err(&client->dev, "setpage %04x err %d\n", reg, ret);
			return ret;
		}

		data->current_page = REG2PAGE(reg);
    }
    return 0;
}

static int reg_read_range(struct tda19988_data *data, uint16_t reg, char *buf, int cnt)
{
	struct i2c_client *client = data->hdmi;
	uint8_t addr = REG2ADDR(reg);
	int ret;

	mutex_lock(&data->mutex);
	ret = set_page(data, reg);
	if (ret < 0)
		goto out;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, buf, cnt);
	if (ret < 0)
		goto fail;

	goto out;

fail:
	dev_err(&client->dev, "Error %d reading from 0x%x\n", ret, reg);
out:
	mutex_unlock(&data->mutex);
	return ret;
}

static int reg_read(struct tda19988_data *data, uint16_t reg)
{
	uint8_t val = 0;
	int ret;

	ret = reg_read_range(data, reg, &val, sizeof(val));
	if (ret < 0)
		return ret;
	return val;
}

static void reg_write(struct tda19988_data *data, uint16_t reg, uint8_t val)
{
	struct i2c_client *client = data->hdmi;
    uint8_t buf[] = {REG2ADDR(reg), val};
	int ret;

    mutex_lock(&data->mutex);
	ret = set_page(data, reg);
	if (ret < 0)
		goto out;

    ret = i2c_master_send(client, buf, sizeof(buf));
    if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
out:
    mutex_unlock(&data->mutex);
}

static void reg_write16(struct tda19988_data *data, uint16_t reg, uint16_t val)
{
	struct i2c_client *client = data->hdmi;
	uint8_t buf[] = {REG2ADDR(reg), val >> 8, val};
	int ret;

	mutex_lock(&data->mutex);
	ret = set_page(data, reg);
	if (ret < 0)
		goto out;

	ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
out:
	mutex_unlock(&data->mutex);
}

static void reg_set(struct tda19988_data *data, uint16_t reg, uint8_t val)
{
	int old_val;

	old_val = reg_read(data, reg);
	if (old_val >= 0)
		reg_write(data, reg, old_val | val);
}

static void reg_clear(struct tda19988_data *data, uint16_t reg, uint8_t val)
{
	int old_val;

	old_val = reg_read(data, reg);
	if (old_val >= 0)
		reg_write(data, reg, old_val & ~val);
}

static void reg_write_range(struct tda19988_data *data, uint16_t reg, uint8_t *p, int cnt)
{
	struct i2c_client *client = data->hdmi;
    uint8_t buf[cnt+1];
    int ret;

    buf[0] = REG2ADDR(reg);
    memcpy(&buf[1], p, cnt);

    mutex_lock(&data->mutex);
    ret = set_page(data, reg);
    if (ret < 0)
		goto out;

   ret = i2c_master_send(client, buf, cnt + 1);
   if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
out:
   mutex_unlock(&data->mutex);
}

#define HB(x) (x)
#define PB(x) (HB(2) + 1 + (x))

static uint8_t tda19988_cksum(uint8_t *buf, size_t bytes)
{
	int sum = 0;

	while (bytes--)
		sum -= *buf++;
    return sum;
}

static void tda19988_write_if(struct tda19988_data *data, uint8_t bit, uint16_t addr,
			uint8_t *buf, size_t size)
{
	buf[PB(0)] = tda19988_cksum(buf, size);

    reg_clear(data, REG_DIP_IF_FLAGS, bit);
    reg_write_range(data, addr, buf, size);
    reg_set(data, REG_DIP_IF_FLAGS, bit);
}

static void tda19988_write_avi(int cea_mode, struct fb_var_screeninfo *var)
{
	u8 buf[PB(HDMI_AVI_INFOFRAME_SIZE) + 1];

	memset(buf, 0, sizeof(buf));
	buf[HB(0)] = HDMI_INFOFRAME_TYPE_AVI;
	buf[HB(1)] = 0x02;
	buf[HB(2)] = HDMI_AVI_INFOFRAME_SIZE;
	buf[PB(1)] = HDMI_SCAN_MODE_UNDERSCAN;
	buf[PB(2)] = HDMI_ACTIVE_ASPECT_PICTURE;
	buf[PB(3)] = HDMI_QUANTIZATION_RANGE_FULL << 2;
	buf[PB(4)] = cea_mode;

	tda19988_write_if(data, DIP_IF_FLAGS_IF2, REG_IF2_HB0, buf,sizeof(buf));
}

static void tda19988_write_aif(struct tda19988_data *data)
{
	u8 buf[PB(HDMI_AUDIO_INFOFRAME_SIZE) + 1];

    memset(buf, 0, sizeof(buf));
    buf[HB(0)] = HDMI_INFOFRAME_TYPE_AUDIO;
    buf[HB(1)] = 0x01;
    buf[HB(2)] = HDMI_AUDIO_INFOFRAME_SIZE;
    buf[PB(1)] = 1 & 0x07; /* CC */
    buf[PB(2)] = 0 & 0x1c; /* SF */
    buf[PB(4)] = 0;
    buf[PB(5)] = 0 & 0xf8; /* DM_INH + LSV */

    tda19988_write_if(data, DIP_IF_FLAGS_IF4, REG_IF4_HB0, buf, sizeof(buf));
}

static void tda19988_audio_mute(struct tda19988_data *data, bool on)
{
	if (on) {
		reg_set(data, REG_SOFTRESET, SOFTRESET_AUDIO);
		reg_clear(data, REG_SOFTRESET, SOFTRESET_AUDIO);
		reg_set(data, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);
	} else {
		reg_clear(data, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);
    }
}

static void tda19988_configure_audio(struct tda19988_data *data,
           struct fb_var_screeninfo var)
{
	uint8_t buf[6], clksel_aip, clksel_fs, cts_n, adiv;
	uint32_t n;

	/* Enable audio ports for I2S */
	reg_write(data, REG_ENA_AP, 0x03);
	reg_write(data, REG_ENA_ACLK, 0x01);

	/* Set audio input source to I2S */
	reg_write(data, REG_MUX_AP, MUX_AP_SELECT_I2S);
	clksel_aip = AIP_CLKSEL_AIP_I2S;
	clksel_fs = AIP_CLKSEL_FS_ACLK;
	cts_n = CTS_N_M(3) | CTS_N_K(3);

    reg_write(data, REG_AIP_CLKSEL, clksel_aip);
    reg_clear(data, REG_AIP_CNTRL_0, AIP_CNTRL_0_LAYOUT | AIP_CNTRL_0_ACR_MAN);   /* auto CTS */
    reg_write(data, REG_CTS_N, cts_n);

    /*
	 * Audio input somehow depends on HDMI line rate which is
	 * related to pixclk. Testing showed that modes with pixclk
	 * >100MHz need a larger divider while <40MHz need the default.
	 * There is no detailed info in the datasheet, so we just
	 * assume 100MHz requires larger divider.
	 */
	adiv = AUDIO_DIV_SERCLK_8;
	if (PICOS2KHZ(var.pixclock)> 100000)
		adiv++;                 /* AUDIO_DIV_SERCLK_16 */

	reg_write(data, REG_AUDIO_DIV, adiv);

	/*
	 * This is the approximate value of N, which happens to be
	 * the recommended values for non-coherent clocks.
	 */
    n = 128 * 48000 / 1000; // Default Sample rate is 48000

    /* Write the CTS and N values */
    buf[0] = 0x44;
    buf[1] = 0x42;
    buf[2] = 0x01;
	buf[3] = n;
    buf[4] = n >> 8;
    buf[5] = n >> 16;
    reg_write_range(data, REG_ACR_CTS_0, buf, 6);

    /* Set CTS clock reference */
    reg_write(data, REG_AIP_CLKSEL, clksel_aip | clksel_fs);

	/* Reset CTS generator */
    reg_set(data, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_CTS);
	reg_clear(data, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_CTS);

    /* Write the channel status */
    buf[0] = IEC958_AES0_CON_NOT_COPYRIGHT;
    buf[1] = 0x00;
    buf[2] = IEC958_AES3_CON_FS_NOTID;
    buf[3] = IEC958_AES4_CON_ORIGFS_NOTID |
		IEC958_AES4_CON_MAX_WORDLEN_24;
	reg_write_range(data, REG_CH_STAT_B(0), buf, 4);

	tda19988_audio_mute(data, true);
	msleep(20);
	tda19988_audio_mute(data, false);

    /* Write the audio information packet */
    tda19988_write_aif(data);
}

static void tda19988_reset(struct tda19988_data *data)
{
	/* reset audio and i2c master: */
    reg_write(data, REG_SOFTRESET, SOFTRESET_AUDIO | SOFTRESET_I2C_MASTER);
    msleep(50);
    reg_write(data, REG_SOFTRESET, 0);
    msleep(50);
 
    /* reset transmitter: */
    reg_set(data, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);
    reg_clear(data, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);

    /* PLL registers common configuration */
    reg_write(data, REG_PLL_SERIAL_1, 0x00);
    reg_write(data, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(1));
    reg_write(data, REG_PLL_SERIAL_3, 0x00);
    reg_write(data, REG_SERIALIZER,   0x00);
    reg_write(data, REG_BUFFER_OUT,   0x00);
    reg_write(data, REG_PLL_SCG1,     0x00);
    reg_write(data, REG_AUDIO_DIV,    AUDIO_DIV_SERCLK_8);
    reg_write(data, REG_SEL_CLK,      SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);
    reg_write(data, REG_PLL_SCGN1,    0xfa);
    reg_write(data, REG_PLL_SCGN2,    0x00);
    reg_write(data, REG_PLL_SCGR1,    0x5b);
    reg_write(data, REG_PLL_SCGR2,    0x00);
    reg_write(data, REG_PLL_SCG2,     0x10);
 
    /* Write the default value MUX register */
    reg_write(data, REG_MUX_VP_VIP_OUT, 0x24);  

	/* Setup RGB Muxing for MCIMX28LCD */
	reg_write(data,REG_VIP_CNTRL_0,0x23);
	reg_write(data,REG_VIP_CNTRL_1,0x01);
	reg_write(data,REG_VIP_CNTRL_2,0x45);
}

static void tda19988_fb_var(struct tda19988_data *data, struct fb_var_screeninfo var) {

    char unsigned value; 
    uint hdisplay, hsync_start, hsync_end, htotal;
    uint vdisplay, vsync_start, vsync_end, vtotal;
	uint div, clk, rep, n_pix, n_line, hs_pix_s, hs_pix_e, de_pix_s, de_pix_e;
	uint ref_pix, ref_line, vwin1_line_s, vwin1_line_e, vs1_pix_s, vs1_pix_e;
	uint vs1_line_s, vs1_line_e, vwin2_line_s,vwin2_line_e;
	uint vs2_pix_s, vs2_pix_e, vs2_line_s, vs2_line_e;
	int cea_mode;
	struct fb_videomode mode;

    reg_write(data,REG_DDC_DISABLE, 0x00);
     /* set clock on DDC channel: */
    reg_write(data,REG_TX3, 39);

    hdisplay = var.xres;
    hsync_start = hdisplay + var.right_margin;
    hsync_end = hsync_start + var.hsync_len;
    htotal = hsync_end + var.left_margin;
    vdisplay = var.yres;
    vsync_start = vdisplay + var.lower_margin;
    vsync_end = vsync_start + var.vsync_len;
    vtotal = vsync_end + var.upper_margin;

    n_pix        = htotal;
    n_line       = vtotal;

    hs_pix_s     = hsync_start - hdisplay;
    hs_pix_e     = hsync_end - hdisplay;
    de_pix_s     = htotal - hdisplay;
    de_pix_e     = htotal;

    ref_pix      = 3 + hs_pix_s;
    
    ref_line     = 1 + vsync_start - vdisplay;
    vwin1_line_s = vtotal - vdisplay - 1;
    vwin1_line_e = vwin1_line_s + vdisplay;
    vs1_pix_s    = hs_pix_s;
    vs1_pix_e    = hs_pix_s;
    vs1_line_s   = vsync_start - vdisplay;
    vs1_line_e   = vs1_line_s + vsync_end - vsync_start;
    vwin2_line_s = 0;
    vwin2_line_e = 0;
    vs2_pix_s    = 0;
    vs2_pix_e    = 0;
    vs2_line_s   = 0;
    vs2_line_e   = 0; 

    /* mute the audio FIFO: */
    reg_set(data,REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);

    /* set HDMI HDCP mode off: */
    reg_write(data,REG_TBG_CNTRL_1, TBG_CNTRL_1_DWIN_DIS);
    reg_clear(data,REG_TX33, TX33_HDMI);
    reg_write(data,REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(0));

    div = 0;
    clk = (PICOS2KHZ(var.pixclock)/10)*10;
    div = 148500 / clk;
    if (div != 0) {
        div--;
        if (div > 3)
            div = 3;
    }

    reg_write(data,REG_HVF_CNTRL_0, HVF_CNTRL_0_PREFIL(0) |
                   HVF_CNTRL_0_INTPOL(0));
    reg_write(data,REG_VIP_CNTRL_5, VIP_CNTRL_5_SP_CNT(0));
    reg_write(data,REG_VIP_CNTRL_4, VIP_CNTRL_4_BLANKIT(0) |
                   VIP_CNTRL_4_BLC(0));

    reg_clear(data,REG_PLL_SERIAL_1, PLL_SERIAL_1_SRL_MAN_IZ);
    reg_clear(data,REG_PLL_SERIAL_3, PLL_SERIAL_3_SRL_CCIR |
                                             PLL_SERIAL_3_SRL_DE);
    reg_write(data,REG_SERIALIZER, 0);
    reg_write(data,REG_HVF_CNTRL_1, HVF_CNTRL_1_VQR(0));

    /* TODO enable pixel repeat for pixel rates less than 25Msamp/s */
    rep = 0;
    reg_write(data,REG_RPT_CNTRL, 0);
    reg_write(data,REG_SEL_CLK, SEL_CLK_SEL_VRF_CLK(0) |
                   SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);

    reg_write(data,REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(div) |
                   PLL_SERIAL_2_SRL_PR(rep));

    /* set color matrix bypass flag: */
    reg_write(data,REG_MAT_CONTRL, MAT_CONTRL_MAT_BP |
                                   MAT_CONTRL_MAT_SC(1));

    /* set BIAS tmds value: */
    reg_write(data,REG_ANA_GENERAL, 0x09);

    /* For MX28 LCD controller invert Hsync */
    value = VIP_CNTRL_3_SYNC_HS;
    value = value | VIP_CNTRL_3_H_TGL;
    reg_write(data,REG_VIP_CNTRL_3, value);

    reg_write(data,REG_VIDFORMAT, 0x00);
    reg_write16(data,REG_REFPIX_MSB, ref_pix);
    reg_write16(data,REG_REFLINE_MSB, ref_line);
    reg_write16(data,REG_NPIX_MSB, n_pix);
    reg_write16(data,REG_NLINE_MSB, n_line);
    reg_write16(data,REG_VS_LINE_STRT_1_MSB, vs1_line_s);
    reg_write16(data,REG_VS_PIX_STRT_1_MSB, vs1_pix_s);
    reg_write16(data,REG_VS_LINE_END_1_MSB, vs1_line_e);
    reg_write16(data,REG_VS_PIX_END_1_MSB, vs1_pix_e);
    reg_write16(data,REG_VS_LINE_STRT_2_MSB, vs2_line_s);
    reg_write16(data,REG_VS_PIX_STRT_2_MSB, vs2_pix_s);
    reg_write16(data,REG_VS_LINE_END_2_MSB, vs2_line_e);
    reg_write16(data,REG_VS_PIX_END_2_MSB, vs2_pix_e);
    reg_write16(data,REG_HS_PIX_START_MSB, hs_pix_s);
    reg_write16(data,REG_HS_PIX_STOP_MSB, hs_pix_e);
    reg_write16(data,REG_VWIN_START_1_MSB, vwin1_line_s);
    reg_write16(data,REG_VWIN_END_1_MSB, vwin1_line_e);
    reg_write16(data,REG_VWIN_START_2_MSB, vwin2_line_s);
    reg_write16(data,REG_VWIN_END_2_MSB, vwin2_line_e);
    reg_write16(data,REG_DE_START_MSB, de_pix_s);
    reg_write16(data,REG_DE_STOP_MSB, de_pix_e);

    reg_write(data,REG_ENABLE_SPACE, 0x00);

    /* For MX28 LCD controller invert Hsync */
    value =  TBG_CNTRL_1_DWIN_DIS | TBG_CNTRL_1_TGL_EN;
    value = value | TBG_CNTRL_1_H_TGL;
    reg_write(data,REG_TBG_CNTRL_1, value);

    /* must be last register set: */
    reg_write(data,REG_TBG_CNTRL_0, 0);

	/* We need to turn HDMI HDCP stuff on to get audio through */
	value &= ~TBG_CNTRL_1_DWIN_DIS;
	reg_write(data, REG_TBG_CNTRL_1, value);
	reg_write(data, REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(1));
	reg_set(data, REG_TX33, TX33_HDMI);

	fb_var_to_videomode(&mode,&var);
	cea_mode=mxc_lcd_fb_var_to_vic(&var);
	dev_info(&data->cec->dev,"Matching CEA mode is %d for fb mode %dx%d-%d",
		cea_mode,var.xres,var.yres,mode.refresh);

	tda19988_write_avi(cea_mode, &var);
	tda19988_configure_audio(data,var);

}

static int tda19988_fb_event(struct notifier_block *nb, unsigned long val, void *v)
{
	struct fb_event *event = v;
	struct fb_info *fbi = event->info;
		
	dev_dbg(&data->cec->dev, "%s event=0x%lx\n", __func__, val);

	switch (val) {
	case FB_EVENT_FB_REGISTERED:
		tda19988_fb_var(data,fbi->var);
		break;
	case FB_EVENT_MODE_CHANGE:
		/* We can't cope with a change in resolution */ 
		break;
	case FB_EVENT_BLANK:
		if (*((int *)event->data) == FB_BLANK_UNBLANK) {
			dev_dbg(&data->cec->dev, "FB_BLANK_UNBLANK\n");
			/* Enable video ports */
			reg_write(data,REG_ENA_VP_0, 0xff);
			reg_write(data,REG_ENA_VP_1, 0xff);
			reg_write(data,REG_ENA_VP_2, 0xff);
		} else {
			dev_dbg(&data->cec->dev, "FB_BLANK_BLANK\n");
			/* Disable video ports */
			reg_write(data, REG_ENA_VP_0, 0x00);
			reg_write(data, REG_ENA_VP_1, 0x00);
			reg_write(data, REG_ENA_VP_2, 0x00);
		}
		break;
	}
	return 0;
}

static struct notifier_block nb = {
	.notifier_call = tda19988_fb_event,
};

static int tda19988_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	
	uint16_t rev;
	int rev_lo, rev_hi;
	
	data = kzalloc(sizeof(struct tda19988_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Could not alloc data memory !\n");
		return-ENOMEM;
	}

	data->cec = client;
	
	// Ask CEC core to enable HDMI core
    cec_write(data, REG_CEC_ENAMODS, CEC_ENAMODS_EN_RXSENS | CEC_ENAMODS_EN_HDMI);
 
    data->hdmi = i2c_new_dummy(client->adapter, 0x70);	
	if (!data->hdmi) {
		dev_err(&client->dev, "Can't find HDMI core");
		kfree(data);
		return -ENODEV;	
	}

	mutex_init(&data->mutex);

    tda19988_reset(data);

	/* read version: */
	rev_lo = reg_read(data, REG_VERSION_LSB);
	rev_hi = reg_read(data, REG_VERSION_MSB);
	if (rev_lo < 0 || rev_hi < 0) {	
		dev_err(&client->dev,"Invalid chip revision MSB %d LSB %d",rev_lo,rev_hi);
		i2c_unregister_device(data->hdmi);
		kfree(data)	;
		return -ENODEV;	
	}

	rev = (uint16_t) (rev_lo | rev_hi << 8);

	/* mask off feature bits: */
	rev &= ~0x30; /* not-hdcp and not-scalar bit */

	if (rev != 0x0301) {
		dev_err(&client->dev,"Invalid chip revision %x",rev);
		i2c_unregister_device(data->hdmi);
		kfree(data);
		return -ENODEV;	
	}

	fb_register_client(&nb);
			
	dev_info(&client->dev,"Sucessfully initialised chip revision %x",rev);

	return 0;
}

static int tda19988_remove(struct i2c_client *client)
{

	if (!data->hdmi) {
		i2c_unregister_device(data->hdmi);
	}

	fb_unregister_client(&nb);
	
	if (!data) {
		kfree(data);
	}

	return 0;
}

static const struct i2c_device_id tda19988_id[] = {
	{ DRV_NAME, 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, tda19988_id);

static const struct of_device_id tda19988_dt_ids[] = {
	{ .compatible = "udoo,tda19988", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, tda19988_dt_ids);

static struct i2c_driver tda19988_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tda19988_dt_ids,
	},
	.probe		= tda19988_probe,
	.remove		= tda19988_remove,
	.id_table	= tda19988_id,
};

module_i2c_driver(tda19988_driver);

MODULE_AUTHOR("Jasbir Matharu");
MODULE_DESCRIPTION("UDOO NEO TDA19988 HDMI driver");
MODULE_LICENSE("GPL");
