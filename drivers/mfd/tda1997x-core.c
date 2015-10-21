/*
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
 * TODO
 *  - add gpio reset pin
 *  - add gpio pwrdn pin
 *  - document devicetree bindings
 *  - unload/reload module interrupts never fire (something not getting reset)
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/mfd/tda1997x-core.h>
#include <linux/time.h>

#include <drm/drm_edid.h>
#include <drm/drm_crtc.h>

/* Voltage regulators */
#define TDA1997X_VOLTAGE_DIGITAL_IO	3300000
#define TDA1997X_VOLTAGE_DIGITAL_CORE	1800000
#define TDA1997X_VOLTAGE_ANALOG		1800000

static struct regulator *dvddio_regulator;
static struct regulator *dvdd_regulator;
static struct regulator *avdd_regulator;

/* Page 0x00 */
#define REG_VERSION             0x0000
#define REG_INPUT_SEL           0x0001
#define REG_SERVICE_MODE        0x0002
#define REG_HPD_MAN_CTRL        0x0003
#define REG_RT_MAN_CTRL         0x0004
#define REG_STANDBY_SOFT_RST    0x000A
#define REG_HDMI_SOFT_RST       0x000B
#define REG_HDMI_INFO_RST       0x000C
#define REG_INT_FLG_CLR_TOP     0x000E
#define REG_INT_FLG_CLR_SUS     0x000F
#define REG_INT_FLG_CLR_DDC     0x0010
#define REG_INT_FLG_CLR_RATE    0x0011
#define REG_INT_FLG_CLR_MODE    0x0012
#define REG_INT_FLG_CLR_INFO    0x0013
#define REG_INT_FLG_CLR_AUDIO   0x0014
#define REG_INT_FLG_CLR_HDCP    0x0015
#define REG_INT_FLG_CLR_AFE     0x0016
#define REG_INT_MASK_TOP        0x0017
#define REG_INT_MASK_SUS        0x0018
#define REG_INT_MASK_DDC        0x0019
#define REG_INT_MASK_RATE       0x001A
#define REG_INT_MASK_MODE       0x001B
#define REG_INT_MASK_INFO       0x001C
#define REG_INT_MASK_AUDIO      0x001D
#define REG_INT_MASK_HDCP       0x001E
#define REG_INT_MASK_AFE        0x001F
#define REG_DETECT_5V           0x0020
#define REG_SUS_STATUS          0x0021
#define REG_V_PER               0x0022
#define REG_H_PER               0x0025
#define REG_HS_WIDTH            0x0027
#define REG_FMT_H_TOT           0x0029
#define REG_FMT_H_ACT           0x002b
#define REG_FMT_H_FRONT         0x002d
#define REG_FMT_H_SYNC          0x002f
#define REG_FMT_H_BACK          0x0031
#define REG_FMT_V_TOT           0x0033
#define REG_FMT_V_ACT           0x0035
#define REG_FMT_V_FRONT_F1      0x0037
#define REG_FMT_V_FRONT_F2      0x0038
#define REG_FMT_V_SYNC          0x0039
#define REG_FMT_V_BACK_F1       0x003a
#define REG_FMT_V_BACK_F2       0x003b
#define REG_FMT_DE_ACT          0x003c
#define REG_RATE_CTRL           0x0040
#define REG_CLK_MIN_RATE        0x0043
#define REG_CLK_MAX_RATE        0x0046
#define REG_CLK_A_STATUS        0x0049
#define REG_CLK_A_RATE          0x004A
#define REG_DRIFT_CLK_A_REG     0x004D
#define REG_CLK_B_STATUS        0x004E
#define REG_CLK_B_RATE          0x004F
#define REG_DRIFT_CLK_B_REG     0x0052
#define REG_HDCP_CTRL           0x0060
#define REG_HDCP_KDS            0x0061
#define REG_HDCP_BCAPS          0x0063
#define REG_HDCP_KEY_CTRL       0x0064
#define REG_INFO_CTRL           0x0076
#define REG_INFO_EXCEED         0x0077
#define REG_PIX_REPEAT          0x007B
#define REG_AUDIO_PATH          0x007C
#define REG_AUDIO_SEL           0x007D
#define REG_AUDIO_OUT_ENABLE    0x007E
#define REG_AUDIO_OUT_HIZ       0x007F
#define REG_VDP_CTRL            0x0080
#define REG_VHREF_CTRL          0x00A0
#define REG_PXCNT_PR            0x00A2
#define REG_PXCNT_NPIX          0x00A4
#define REG_LCNT_PR             0x00A6
#define REG_LCNT_NLIN           0x00A8
#define REG_HREF_S              0x00AA
#define REG_HREF_E              0x00AC
#define REG_HS_S                0x00AE
#define REG_HS_E                0x00B0
#define REG_VREF_F1_S           0x00B2
#define REG_VREF_F1_WIDTH       0x00B4
#define REG_VREF_F2_S           0x00B5
#define REG_VREF_F2_WIDTH       0x00B7
#define REG_VS_F1_LINE_S        0x00B8
#define REG_VS_F1_LINE_WIDTH    0x00BA
#define REG_VS_F2_LINE_S        0x00BB
#define REG_VS_F2_LINE_WIDTH    0x00BD
#define REG_VS_F1_PIX_S         0x00BE
#define REG_VS_F1_PIX_E         0x00C0
#define REG_VS_F2_PIX_S         0x00C2
#define REG_VS_F2_PIX_E         0x00C4
#define REG_FREF_F1_S           0x00C6
#define REG_FREF_F2_S           0x00C8
#define REG_FDW_S               0x00ca
#define REG_FDW_E               0x00cc
#define REG_BLK_GY              0x00da
#define REG_BLK_BU              0x00dc
#define REG_BLK_RV              0x00de
#define REG_FILTERS_CTRL        0x00e0
#define REG_DITHERING_CTRL      0x00E9
#define REG_OF_CTRL             0x00EA
#define REG_CLKOUT_CTRL         0x00EB
#define REG_HS_HREF_SEL         0x00EC
#define REG_VS_VREF_SEL         0x00ED
#define REG_DE_FREF_SEL         0x00EE
#define REG_VP35_32_CTRL        0x00EF
#define REG_VP31_28_CTRL        0x00F0
#define REG_VP27_24_CTRL        0x00F1
#define REG_VP23_20_CTRL        0x00F2
#define REG_VP19_16_CTRL        0x00F3
#define REG_VP15_12_CTRL        0x00F4
#define REG_VP11_08_CTRL        0x00F5
#define REG_VP07_04_CTRL        0x00F6
#define REG_VP03_00_CTRL        0x00F7
#define REG_CURPAGE_00H         0xFF
#define MASK_VPER               0x3fffff
#define MASK_HPER               0x0fff
#define MASK_HSWIDTH            0x03ff

/* Page 0x01 */
#define REG_HDMI_FLAGS          0x0100
#define REG_DEEP_COLOR_MODE     0x0101
#define REG_AUDIO_FLAGS         0x0108
#define REG_AUDIO_FREQ          0x0109
#define REG_ACP_PACKET_TYPE     0x0141
#define REG_ISRC1_PACKET_TYPE   0x0161
#define REG_ISRC2_PACKET_TYPE   0x0181
#define REG_GBD_PACKET_TYPE     0x01a1
#define ISRC_PACKET_HDR_LEN     3
#define ISRC_PACKET_DAT_LEN     16
#define GDB_PACKET_HDR_LEN      3
#define GDB_PACKET_DAT_LEN      28
#define ACP_PACKET_HDR_LEN      3
#define ACP_PACKET_DAT_LEN      16
#define MASK_AUDIO_DST_RATE     0x80
#define MASK_AUDIO_FREQ         0x07
#define MASK_DC_PIXEL_PHASE     0xf0
#define MASK_DC_COLOR_DEPTH     0x0f

/* Page 0x12 */
#define REG_CLK_CFG             0x1200
#define REG_CLK_OUT_CFG         0x1201
#define REG_CFG1                0x1202
#define REG_CFG2                0x1203
#define REG_WDL_CFG             0x1210
#define REG_DELOCK_DELAY        0x1212
#define REG_PON_OVR_EN          0x12A0
#define REG_PON_CBIAS           0x12A1
#define REG_PON_RESCAL          0x12A2
#define REG_PON_RES             0x12A3
#define REG_PON_CLK             0x12A4
#define REG_PON_PLL             0x12A5
#define REG_PON_EQ              0x12A6
#define REG_PON_DES             0x12A7
#define REG_PON_OUT             0x12A8
#define REG_PON_MUX             0x12A9
#define REG_MODE_RECOVER_CFG1   0x12F8
#define REG_MODE_RECOVER_CFG2   0x12F9
#define REG_MODE_RECOVER_STS    0x12FA
#define REG_AUDIO_LAYOUT        0x12D0

/* Page 0x13 */
#define REG_DEEP_COLOR_CTRL     0x1300
#define REG_CGU_DEBUG_SEL       0x1305
#define REG_HDCP_DDC_ADDR       0x1310
#define REG_HDCP_KIDX           0x1316
#define REG_DEEP_PLL7           0x1347
#define REG_HDCP_DE_CTRL        0x1370
#define REG_HDCP_EP_FILT_CTRL   0x1371
#define REG_HDMI_CTRL           0x1377
#define REG_HMTP_CTRL           0x137a
#define REG_TIMER_D             0x13CF
#define REG_SUS_SET_RGB0        0x13E1
#define REG_SUS_SET_RGB1        0x13E2
#define REG_SUS_SET_RGB2        0x13E3
#define REG_SUS_SET_RGB3        0x13E4
#define REG_SUS_SET_RGB4        0x13E5
#define REG_MAN_SUS_HDMI_SEL    0x13E8
#define REG_MAN_HDMI_SET        0x13E9
#define REG_SUS_CLOCK_GOOD      0x13EF

/* CGU_DEBUG_SEL bits */
#define CGU_DEBUG_CFG_CLK_MASK  0x18
#define CGU_DEBUG_XO_FRO_SEL    (1<<2)
#define CGU_DEBUG_VDP_CLK_SEL   (1<<1)
#define CGU_DEBUG_PIX_CLK_SEL   (1<<0)

/* REG_MAN_SUS_HDMI_SEL / REG_MAN_HDMI_SET bits */
#define MAN_DIS_OUT_BUF         (1<<7)
#define MAN_DIS_ANA_PATH        (1<<6)
#define MAN_DIS_HDCP            (1<<5)
#define MAN_DIS_TMDS_ENC        (1<<4)
#define MAN_DIS_TMDS_FLOW       (1<<3)
#define MAN_RST_HDCP            (1<<2)
#define MAN_RST_TMDS_ENC        (1<<1)
#define MAN_RST_TMDS_FLOW       (1<<0)

/* Page 0x14 */
#define REG_FIFO_LATENCY_VAL    0x1403
#define REG_AUDIO_CLOCK_MODE    0x1411
#define REG_TEST_NCTS_CTRL      0x1415
#define REG_TEST_AUDIO_FREQ     0x1426
#define REG_TEST_MODE           0x1437

/* Page 0x20 */
#define REG_EDID_IN_BYTE0       0x2000 /* EDID base */
#define REG_EDID_IN_VERSION     0x2080
#define REG_EDID_ENABLE         0x2081
#define REG_HPD_POWER           0x2084
#define REG_HPD_AUTO_CTRL       0x2085
#define REG_HPD_DURATION        0x2086
#define REG_RX_HPD_HEAC         0x2087

/* Page 0x21 */
#define REG_EDID_IN_BYTE128     0x2100 /* CEA Extension block */
#define REG_EDID_IN_SPA_SUB     0x2180
#define REG_EDID_IN_SPA_AB_A    0x2181
#define REG_EDID_IN_SPA_CD_A    0x2182
#define REG_EDID_IN_CKSUM_A     0x2183
#define REG_EDID_IN_SPA_AB_B    0x2184
#define REG_EDID_IN_SPA_CD_B    0x2185
#define REG_EDID_IN_CKSUM_B     0x2186

/* Page 0x30 */
#define REG_RT_AUTO_CTRL        0x3000
#define REG_EQ_MAN_CTRL0        0x3001
#define REG_EQ_MAN_CTRL1        0x3002
#define REG_OUTPUT_CFG          0x3003
#define REG_MUTE_CTRL           0x3004
#define REG_SLAVE_ADDR          0x3005
#define REG_CMTP_REG6           0x3006
#define REG_CMTP_REG7           0x3007
#define REG_CMTP_REG8           0x3008
#define REG_CMTP_REG9           0x3009
#define REG_CMTP_REGA           0x300A
#define REG_CMTP_REGB           0x300B
#define REG_CMTP_REGC           0x300C
#define REG_CMTP_REGD           0x300D
#define REG_CMTP_REGE           0x300E
#define REG_CMTP_REGF           0x300F
#define REG_CMTP_REG10          0x3010
#define REG_CMTP_REG11          0x3011

/* CEC */
#define REG_PWR_CONTROL         0x80F4
#define REG_OSC_DIVIDER         0x80F5
#define REG_EN_OSC_PERIOD_LSB   0x80F8
#define REG_CONTROL             0x80FF

/* global interrupt flags (INT_FLG_CRL_TOP) */
#define INTERRUPT_AFE           (1<<7) /* AFE module */
#define INTERRUPT_HDCP          (1<<6) /* HDCP module */
#define INTERRUPT_AUDIO         (1<<5) /* Audio module */
#define INTERRUPT_INFO          (1<<4) /* Infoframe module */
#define INTERRUPT_MODE          (1<<3) /* HDMI mode module */
#define INTERRUPT_RATE          (1<<2) /* rate module */
#define INTERRUPT_DDC           (1<<1) /* DDC module */
#define INTERRUPT_SUS           (1<<0) /* SUS module */

/* INT_FLG_CLR_HDCP bits */
#define MASK_HDCP_MTP           (1<<7) /* HDCP MTP busy */
#define MASK_HDCP_DLMTP         (1<<4) /* HDCP end download MTP to SRAM */
#define MASK_HDCP_DLRAM         (1<<3) /* HDCP end download keys from SRAM */
#define MASK_HDCP_ENC           (1<<2) /* HDCP ENC */
#define MASK_STATE_C5           (1<<1) /* HDCP State C5 reached */
#define MASK_AKSV               (1<<0) /* AKSV received (start of auth) */

/* INT_FLG_CLR_RATE bits */
#define MASK_RATE_B_DRIFT       (1<<7) /* Rate measurement drifted */
#define MASK_RATE_B_ST          (1<<6) /* Rate measurement stability change */
#define MASK_RATE_B_ACT         (1<<5) /* Rate measurement activity change */
#define MASK_RATE_B_PST         (1<<4) /* Rate measreument presence change */
#define MASK_RATE_A_DRIFT       (1<<3) /* Rate measurement drifted */
#define MASK_RATE_A_ST          (1<<2) /* Rate measurement stability change */
#define MASK_RATE_A_ACT         (1<<1) /* Rate measurement presence change */
#define MASK_RATE_A_PST         (1<<0) /* Rate measreument presence change */

/* INT_FLG_CLR_SUS (Start Up Sequencer) bits */
#define MASK_MPT_BIT            (1<<7) /* Config MTP end of process */
#define MASK_FMT_BIT            (1<<5) /* Video format changed */
#define MASK_RT_PULSE_BIT       (1<<4) /* End of termination resistance pulse */
#define MASK_SUS_END_BIT        (1<<3) /* SUS last state reached */
#define MASK_SUS_ACT_BIT        (1<<2) /* Activity of selected input changed */
#define MASK_SUS_CH_BIT         (1<<1) /* Selected input changed */
#define MASK_SUS_ST_BIT         (1<<0) /* SUS state changed */

/* INT_FLG_CLR_DDC bits */
#define MASK_EDID_MTP           (1<<7) /* EDID MTP end of process */
#define MASK_DDC_ERR            (1<<6) /* master DDC error */
#define MASK_DDC_CMD_DONE       (1<<5) /* master DDC cmd send correct */
#define MASK_READ_DONE          (1<<4) /* End of down EDID read */
#define MASK_RX_DDC_SW          (1<<3) /* Output DDC switching finished */
#define MASK_HDCP_DDC_SW        (1<<2) /* HDCP DDC switching finished */
#define MASK_HDP_PULSE_END      (1<<1) /* End of Hot Plug Detect pulse */
#define MASK_DET_5V             (1<<0) /* Detection of +5V */

/* INT_FLG_CLR_MODE bits */
#define MASK_HDMI_FLG           (1<<7) /* HDMI mode, avmute, encrypt-on, FIFO fail */
#define MASK_GAMUT              (1<<6) /* Gamut packet */
#define MASK_ISRC2              (1<<5) /* ISRC2 packet */
#define MASK_ISRC1              (1<<4) /* ISRC1 packet */
#define MASK_ACP                (1<<3) /* Audio Content Protection packet */
#define MASK_DC_NO_GCP          (1<<2) /* GCP not recieved in 5 frames */
#define MASK_DC_PHASE           (1<<1) /* deep color mode pixel phase needs update */
#define MASK_DC_MODE            (1<<0) /* deep color mode color depth changed */

/* INT_FLG_CLR_INFO bits */
#define MASK_MPS_IF             (1<<6) /* MPEG Source Product IF change */
#define MASK_AUD_IF             (1<<5) /* Audio IF change */
#define MASK_SPD_IF             (1<<4) /* Source Product Descriptor IF change */
#define MASK_AVI_IF             (1<<3) /* Auxiliary Video information IF change */
#define MASK_VS_IF_OTHER_BK2    (1<<2) /* Vendor Specific IF (bank2) change */
#define MASK_VS_IF_OTHER_BK1    (1<<1) /* Vendor Specific IF (bank1) change */
#define MASK_VS_IF_HDMI         (1<<0) /* Vendor Specific IF (with HDMI LLC reg code) change */

/* INT_FLG_CLR_AUDIO bits */
#define MASK_AUDIO_FREQ_FLG     (1<<5) /* Audio freq change */
#define MASK_AUDIO_FLG          (1<<4) /* DST, OBA, HBR, ASP change */
#define MASK_MUTE_FLG           (1<<3) /* Audio Mute */
#define MASK_CH_STATE           (1<<2) /* Channel status */
#define MASK_UNMUTE_FIFO        (1<<1) /* Audio Unmute */
#define MASK_ERROR_FIFO_PT      (1<<0) /* Audio FIFO pointer error */

/* INT_FLG_CLR_AFE bits */
#define MASK_AFE_WDL_UNLOCKED   (1<<7) /* Wordlocker was unlocked */
#define MASK_AFE_GAIN_DONE      (1<<6) /* Gain calibration done */
#define MASK_AFE_OFFSET_DONE    (1<<5) /* Offset calibration done */
#define MASK_AFE_ACTIVITY_DET   (1<<4) /* Activity detected on data */
#define MASK_AFE_PLL_LOCK       (1<<3) /* TMDS PLL is locked */
#define MASK_AFE_TRMCAL_DONE    (1<<2) /* Termination calibration done */
#define MASK_AFE_ASU_STATE      (1<<1) /* ASU state is reached */
#define MASK_AFE_ASU_READY      (1<<0) /* AFE calibration done: TMDS ready */

/* OF_CTRL bits */
#define VP_OUT                  (1<<7) /* enable VP[35:0], HS, VS, DE, V_CLK */
#define VP_HIZ                  (1<<6) /* unused VP pins Hi-Z */
#define VP_BLK                  (1<<4) /* Insert blanking code in data */
#define VP_TRC                  (1<<3) /* Insert timing code (SAV/EAV) in data*/
#define VP_FORMAT_SEL_MASK      0x7    /* format selection */

/* HDMI_SOFT_RST bits */
#define RESET_DC                (1<<7) /* Reset deep color module */
#define RESET_HDCP              (1<<6) /* Reset HDCP module */
#define RESET_KSV               (1<<5) /* Reset KSV-FIFO */
#define RESET_SCFG              (1<<4) /* Reset HDCP and repeater function */
#define RESET_HCFG              (1<<3) /* Reset HDCP DDC part */
#define RESET_PA                (1<<2) /* Reset polarity adjust */
#define RESET_EP                (1<<1) /* Reset Error protection */
#define RESET_TMDS              (1<<0) /* Reset TMDS (calib, encoding, flow) */

/* HDMI_INFO_RST bits */
#define NACK_HDCP               (1<<7) /* No ACK on HDCP request */
#define RESET_FIFO              (1<<4) /* Reset Audio FIFO control */
#define RESET_GAMUT             (1<<3) /* Clear Gamut packet */
#define RESET_AI                (1<<2) /* Clear ACP and ISRC packets */
#define RESET_IF                (1<<1) /* Clear all Audio infoframe packets */
#define RESET_AUDIO             (1<<0) /* Reset Audio FIFO control */

/* HDCP_BCAPS bits */
#define HDCP_HDMI               (1<<7) /* HDCP suports HDMI (vs DVI only) */
#define HDCP_REPEATER           (1<<6) /* HDCP supports repeater function */
#define HDCP_READY              (1<<5) /* set by repeater function */
#define HDCP_FAST               (1<<4) /* Up to 400kHz */
#define HDCP_11                 (1<<1) /* HDCP 1.1 supported */
#define HDCP_FAST_REAUTH        (1<<0) /* fast reauthentication suported */

/* masks for interrupt status registers */
#define MASK_SUS_STATE_VALUE    0x1F
#define LAST_STATE_REACHED      0x1B
#define MASK_CLK_STABLE         0x04
#define MASK_CLK_ACTIVE         0x02
#define MASK_SUS_STATE_BIT      0x10
#define MASK_SR_FIFO_FIFO_CTRL  0x30
#define MASK_AUDIO_FLAG         0x10

/* Power Control */
#define MASK_OF_CTRL_OUT_HIZ    0x80
#define MASK_AUDIO_PLL_PD       0x80
#define DC_PLL_PD               0x01
#define DC_PLL_PON              0x00
#define MASK_XTAL_OSC_PD        0x02
#define MASK_TMDS_CLK_DIS       0x08
#define CBIAS_PON               0x01
#define CBIAS_POFF              0x00
#define TMDS_AUTO_PON           0x00
#define TMDS_MAN_PON            0x01
#define MASK_LOW_PW_EDID        0x01

/* Rate measurement */
#define RATE_REFTIM_ENABLE      0x01
#define CLK_MIN_RATE            0x0057e4
#define CLK_MAX_RATE            0x0395f8
#define WDL_CFG_VAL             0x82
#define DC_FILTER_VAL           0x31

/* Infoframe */
#define VS_HDMI_IF_UPDATE       0x0200
#define VS_HDMI_IF_TYPE         0x0201
#define VS_BK1_IF_UPDATE        0x0220
#define VS_BK1_IF_TYPE          0x0221
#define VS_BK2_IF_UPDATE        0x0240
#define VS_BK2_IF_TYPE          0x0241
#define AVI_IF_UPDATE           0x0260
#define AVI_IF_TYPE             0x0261
#define AVI_IF_NB_DATA          17
#define SPD_IF_UPDATE           0x0280
#define SPD_IF_TYPE             0x0281
#define SPD_IF_NB_DATA          31
#define AUD_IF_UPDATE           0x02a0
#define AUD_IF_TYPE             0x02a1
#define AUD_IF_NB_DATA          14
#define MPS_IF_UPDATE           0x02c0
#define MPS_IF_TYPE             0x02c1
#define MPS_IF_NB_DATA          14
#define MAX_IF_DATA             40
#define VS_IF_NB                31

/* Input Selection */
#define MASK_DIG_INPUT          0x01
#define MASK_DIG_INPUT_VDPR_FMT 0x85
#define MASK_HDMIOUTMODE        0x02
#define FORMAT_RESET            0x80

/* Colorspace Conversion Registers */
#define MAT_OFFSET_NB           3
#define MAT_COEFF_NB            9
#define OFFSET_LOOP_NB          2
#define MIN_VAL_OFFSET          -4096
#define MAX_VAL_OFFSET          4095
#define MIN_VAL_COEFF           -16384
#define MAX_VAL_COEFF           16383
#define MASK_MAT_COEFF_LSB      0x00FF

/* Blanking code values depend on output colorspace (RGB or YUV) */
typedef struct
{
	s16 blankingCodeGy;
	s16 blankingCodeBu;
	s16 blankingCodeRv;
} blankingcodes_t;	

blankingcodes_t RGBBlankingCode = {64, 64, 64};
blankingcodes_t YUVBlankingCode = {64, 512, 512};

/* Video Colorspace formats */
typedef enum {
	COLORSPACE_RGB,
	COLORSPACE_YCBCR_422,
	COLORSPACE_YCBCR_444,
	COLORSPACE_FUTURE,
} tda1997x_colorspace_t;

/* Video Colorimetry formats */
typedef enum {
	COLORIMETRY_NONE,
	COLORIMETRY_ITU601,
	COLORIMETRY_ITU709,
	COLORIMETRY_XVYCC,
} tda1997x_colorimetry_t;

/* Video colormode formats */
typedef enum {
	DEEPCOLORMODE_NOT_INDICATED = 0x00,
	DEEPCOLORMODE_24            = 0x04,
	DEEPCOLORMODE_30            = 0x05,
	DEEPCOLORMODE_36            = 0x06,
	DEEPCOLORMODE_48            = 0x07,
} tda1997x_deepcolor_t;

/* resolution type */
typedef enum {
	RESTYPE_SDTV,
	RESTYPE_HDTV,
	RESTYPE_PC,
} tda1997x_restype_t;

/* Video output port format */
const char *vidfmt_names[] = {
	"RGB444/YUV444",        /* RGB/YUV444 16bit data bus, 8bpp */
	"YUV422 semi-planar", /* YUV422 16bit data base, 8bpp */
	"YUV422 CCIR656",     /* BT656 (YUV 8bpp 2 clock per pixel) */
};

static char *colorspace_names[] = {
	"RGB", "YUV422", "YUV444", "Future"
};

static char *colorimetry_names[] = {
	"", "ITU601", "ITU709", "XVYCC"
};

/* HDCP */
#define RX_SEED_TABLE_LEN       10 /* HDCP Seed */
typedef enum
{
	HDCP_DECRYPTKEY_OFF = 0x00,
	HDCP_DECRYPTKEY_ON  = 0x02
} hdcp_key_t;
typedef enum
{
	DISABLE = 0x00,
	ENABLE  = 0x01
} enable_t;

/* MTP */
typedef enum {
	MTP_START_DOWNLOAD,
	MTP_START_READ,
} mtp_command_t;

/* HPD modes */
typedef enum {
	HPD_LOW,        /* HPD low and pulse of at least 100ms */
	HPD_LOW_OTHER,  /* HPD low and pulse of at least 100ms */
	HPD_HIGH,       /* HIGH */
	HPD_HIGH_OTHER,
	HPD_PULSE,      /* HPD low pulse */
} hpdmode_t;

/** configure colorspace conversion matrix
 * The color conversion matrix will convert between the colorimetry of the
 * HDMI input to the desired output format RGB|YUV
 */
typedef enum {
	ITU709_RGBLimited,
	RGBLimited_ITU601,
	ITU601_RGBLimited,
} colorconversion_t;

/* Colorspace conversion matrix coefficients and offsets
 */
typedef struct
{
	/* Input offsets */
	s16 offInt1;
	s16 offInt2;
	s16 offInt3;
	/* Coeficients */
	s16 P11Coef;
	s16 P12Coef;
	s16 P13Coef;
	s16 P21Coef;
	s16 P22Coef;
	s16 P23Coef;
	s16 P31Coef;
	s16 P32Coef;
	s16 P33Coef;
	/* Output offsets */
	s16 offOut1;
	s16 offOut2;
	s16 offOut3;
} colormatrixcoefs_t;

/* Conversion matrixes */
colormatrixcoefs_t conversion_matrix[] = {
	/* ITU709 -> RGBLimited */
	{
		-256, -2048,  -2048,  /*Input Offset*/
		4096, -1875,   -750,
		4096,  6307,      0,
		4096,     0,   7431,
		 256,   256,    256   /*Output Offset*/
	},
	/* RGBLimited -> ITU601 */
	{
		-256,  -256,   -256,  /*Input Offset*/
		2404,  1225,    467,
		-1754, 2095,   -341,
		-1388, -707,   2095,  /*RGB limited range => ITU-601 YUV limited range */
		256,   2048,   2048   /*Output Offset*/
	},
	/* YUV601 -> RGBLimited */
	{
		-256, -2048,  -2048,  /*Input Offset*/
		4096, -2860,  -1378,
		4096,  5615,      0,
		4096,     0,   7097,  /*ITU-601 YUV limited range => RGB limited range */
		256,    256,    256   /*Output Offset*/
	}
};

/* HDCP seed table, arranged as pairs of 16bit integrers:  lookup val, seed val
 * If no table is programmed or KEY_SED in config file is null, HDCP will be
 * disabled
 */
typedef struct {
	u16 lookUpVal;
	u16 seedVal;
} hdmi_cfg_seed_t;

const hdmi_cfg_seed_t rx_seed_table[RX_SEED_TABLE_LEN] = {
	{0xF0, 0x1234},
	{0xF1, 0xDBE6},
	{0xF2, 0xDBE6},
	{0, 0x1234},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0}
};

/** Video Input mode database
 * TODO: can I use something like modedb instead?
 *       More recent kernels have some CEA data
 */
static char *restype_names[] = {
	"SDTV", "HDTV", "PC",
};

typedef enum
{
	VIDEORES_1280_720p_24HZ,
	VIDEORES_1280_720p_25HZ,
	VIDEORES_1280_720p_30HZ,
	VIDEORES_1920_1080p_24HZ,
	VIDEORES_1920_1080p_25HZ,
	VIDEORES_1920_1080p_30HZ,
	
	VIDEORES_720_480p_60HZ,
	VIDEORES_1280_720p_60HZ,
	VIDEORES_1920_1080i_60HZ,
	VIDEORES_720_480i_60HZ,
	VIDEORES_1920_1080p_60HZ,
	
	VIDEORES_720_576p_50HZ,
	VIDEORES_1280_720p_50HZ,
	VIDEORES_1920_1080i_50HZ,
	VIDEORES_720_576i_50HZ,
	VIDEORES_1920_1080p_50HZ,
	
	VIDEORES_640_480p_60HZ,      /* VGA   */
	VIDEORES_800_600p_60HZ,      /* SVGA  */
	VIDEORES_1024_768p_60HZ,     /* XGA   */
	VIDEORES_1280_768p_60HZ,     /* WXGA  */
	VIDEORES_1280_960p_60HZ,     /* ????  */
	VIDEORES_1280_1024p_60HZ,    /* SXGA  */
	VIDEORES_1440_900p_60HZ,     /* ????  */
	VIDEORES_1600_1200p_60HZ,    /* UGA   */
	VIDEORES_1680_1050p_60HZ_RB, /* WSXGA */
	VIDEORES_1920_1200p_60HZ_RB, /* WUXGA */

	VIDEORES_640_480p_75HZ,      /* VGA   */
	VIDEORES_800_600p_75HZ,      /* SVGA  */
	VIDEORES_1024_768p_75HZ,     /* XGA   */
	VIDEORES_1280_768p_75HZ,     /* WXGA  */
	VIDEORES_1280_1024p_75HZ,    /* SXGA  */

	VIDEORES_640_480p_85HZ,      /* VGA   */
	VIDEORES_800_600p_85HZ,      /* SVGA  */
	VIDEORES_1024_768p_85HZ,     /* XGA   */
	VIDEORES_1280_768p_85HZ,     /* WXGA  */
	VIDEORES_1280_1024p_85HZ,    /* SXGA  */

	VIDEORES_720_240p_60HZ_M1,   /* 720(1440, 2880)x240p 60Hz mode 1 */
	VIDEORES_720_240p_60HZ_M2,   /* 720(1440, 2880)x240p 60Hz mode 2 */

	VIDEORES_720_288p_50HZ_M1,   /* 720(1440)x288p 50Hz mode 1 */
	VIDEORES_720_288p_50HZ_M2,   /* 720(1440)x288p 50Hz mode 1 */
	VIDEORES_720_288p_50HZ_M3,   /* 720(1440)x288p 50Hz mode 1 */

	VIDEORES_1360_768p_60HZ,     /* 1360x768p 60Hz (PC resolution) */
	VIDEORES_1400_1050p_60HZ,    /* 1400x1050p 60Hz (PC resolution) */
	VIDEORES_1400_1050p_60HZ_RB, /* 1400x1050p 60Hz Reduced Blanking (PC) */

	VIDEORES_1024_768p_70HZ,     /* XGA  */
	VIDEORES_640_480p_72HZ,      /* VGA  */
	VIDEORES_800_600p_72HZ,      /* SVGA */

	VIDEORES_640_350p_85HZ,      /* 640x350p 85Hz (PC) */
	VIDEORES_640_400p_85HZ,      /* 640x400p 85Hz (PC) */
	VIDEORES_720_400p_85HZ,	     /* 720x400p 85Hz (PC) */
	VIDEORES_UNKNOWN
} resolutionid_t;

/* structure for video format measurements */
typedef struct
{
	u8 videoFormat;                 /* 1=interlaced or 0=progressive */
	u8 vsPolarity;                  /* 1=negative 0=positive */
	u8 hsPolarity;                  /* 1=negative 0=positive */
	u16 horizontalTotalPeriod;      /* period of 1 line (pixel clocks) */
	u16 horizontalVideoActiveWidth; /* period of 1 active line (pixel clocks) */
	u16 horizontalFrontPorchWidth;  /* width of front porch */
	u16 horizontalBackPorchWidth;   /* width of back porch */
	u16 horizontalSyncWidthPixClk;
	u16 verticalTotalPeriod;        /* period of a frame in line numbers */
	u16 verticalVideoActiveWidth;
	u16 verticalFrontPorchWidthF1;  /* vertical front porch width of field 1 */
	u16 verticalFrontPorchWidthF2;  /* vertical front porch width of field 2*/
	u16 verticalSyncWidth;          /* width of the VS in line numbers */
	u16 verticalBackPorchWidthF1;   /* vertical back porch width of field 1 */
	u16 verticalBackPorchWidthF2;   /* vertical back porch width of field 2 */
	u16 dataEnablePresent;          /* 1=DE signal present */
} videoFormatDetails;

typedef struct
{
	u8 resolutionID;
	u16 width;
	u16 height;
	u8  horizfreq;
	u8  interlaced;
	u32 verticalPeriodMin;      /* = MCLK(27MHz) / VFreq minus 0.7% */
	u32 verticalPeriodMax;      /* same + 0.7% */
	u16 horizontalPeriodMin;    /* = MCLK(27MHz) / HFreq minus 1% */
	u16 horizontalPeriodMax;
	u16 hsWidthMin;             /* = MCLK(27MHz) / pixclk * hWidth minux ...% */
	u16 hsWidthMax;
} resolution_data_t;

typedef struct
{
	u16 href_start;
	u16 href_end;
	u16 vref_f1_start;
	u8  vref_f1_width;
	u16 vref_f2_start;
	u8  vref_f2_width;
	u16 fieldref_f1_start;
	u8  fieldPolarity;
	u16 fieldref_f2_start;
} vhref_values_t;

typedef struct
{
	u8 resolutionID;
	u16 pixCountPreset;
	u16 pixCountNb;
	u16 lineCountPreset;
	u16 lineCountNb;
	vhref_values_t vhref_values;
} resolution_timings_t;

const resolution_timings_t resolution_timings[] = {
	/* Low TV */
	{VIDEORES_1280_720p_24HZ, 1, 3300, 1, 750,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1280_720p_25HZ, 1, 3960, 1, 750,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1280_720p_30HZ, 1, 3300, 1, 750,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1920_1080p_24HZ, 1, 2750, 1, 1125,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1920_1080p_25HZ, 1, 2640, 1, 1125,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1920_1080p_30HZ, 1, 2200, 1, 1125,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},

	/* 60 Hz TV */
	{VIDEORES_720_480p_60HZ, 1, 858, 1, 525,
		{123, 843, 516, 45, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1280_720p_60HZ, 1, 1650, 1, 750,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1920_1080i_60HZ, 1, 2200, 1, 1125,
		{193, 2113, 1123, 22, 560, 23, 1, 0, 563}
	},
	{VIDEORES_720_480i_60HZ, 1, 858, 1, 525,
		{120, 840, 521, 22, 258, 23, 1, 0, 263}
	},
	{VIDEORES_1920_1080p_60HZ, 1, 2200, 1, 1125,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},
	
	/* 50 Hz TV */
	{VIDEORES_720_576p_50HZ, 1, 864, 1, 625,
		{133, 853, 620, 49, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1280_720p_50HZ, 1, 1980, 1, 750,
		{261, 1541, 745, 30, 0, 0, 1, 0, 0}
	},
	{VIDEORES_1920_1080i_50HZ, 1, 2640, 1, 1125,
		{193, 2113, 1123, 22, 560, 23, 1, 0, 563}
	},
	{VIDEORES_720_576i_50HZ, 1, 864, 1, 625,
		{133, 853, 623, 24, 310, 25, 1, 0, 313 }
	},
	{VIDEORES_1920_1080p_50HZ, 1, 2640, 1, 1125,
		{193, 2113, 1121, 45, 0, 0, 1, 0, 0}
	},

	/* 60 Hz PC */
	{VIDEORES_640_480p_60HZ, 1, 800, 1, 525,
		{145, 785, 515, 45, 0, 0, 1, 0, 0}
	},
	{VIDEORES_800_600p_60HZ, 1, 1056, 1, 628,
		{217, 1017, 627, 28, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1024_768p_60HZ, 1, 1344, 1, 806,
		{297, 1321, 803, 38, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1280_768p_60HZ, 1, 1440, 1, 790,
		{321, 1601, 795, 30, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1280_960p_60HZ, 1, 1800, 1, 1000,
		{425, 1705, 999, 40, 0, 0, 0, 0, 0}
	},
 	{VIDEORES_1280_1024p_60HZ, 1, 1688, 1, 1066,
		{361, 1641, 1065, 42, 0, 0, 0, 0, 0}
 	},
	{VIDEORES_1440_900p_60HZ, 1, 1904, 1, 934,
		{385, 1825, 931, 34, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1600_1200p_60HZ, 1, 2160, 1, 1250,
		{497, 2097, 1249, 50, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1680_1050p_60HZ_RB, 1, 1840, 1, 1080,
		{113, 1793, 1077, 30, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1920_1200p_60HZ_RB, 1, 2080, 1, 1235,
		{113, 2033, 1232, 35, 0, 0, 0, 0, 0}
	},

 	/* 75 HZ PC */
	{VIDEORES_640_480p_75HZ, 1, 840, 1, 500,
		{185, 825, 499, 20, 0, 0, 1, 0, 0}
	},
	{VIDEORES_800_600p_75HZ, 1, 1056, 1, 625,
		{241, 1041, 624, 25, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1024_768p_75HZ, 1, 1312, 1, 800,
		{273, 1297, 799, 32, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1280_768p_75HZ, 1, 1696, 1, 805,
		{337, 1617, 802, 37, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1280_1024p_75HZ, 1, 1688, 1, 1066,
		{393, 1673, 1065, 42, 0, 0, 0, 0, 0}
	},
	
	/* 85 HZ PC */
	{VIDEORES_640_480p_85HZ, 1, 832, 1, 509,
		{137, 777, 508, 29, 0, 0, 1, 0, 0}
	},
	{VIDEORES_800_600p_85HZ, 1, 1048, 1, 631,
		{217, 1017, 630, 31, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1024_768p_85HZ, 1, 1376, 1, 808,
		{305, 1329, 807, 40, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1280_768p_85HZ, 1, 1712, 1, 908,
		{353, 1633, 905, 140, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1280_1024p_85HZ, 1, 1728, 1, 1072,
		{385, 1665, 1071, 48, 0, 0, 0, 0, 0}
	},
	
	/* Other resolutions */
	{VIDEORES_720_240p_60HZ_M1, 1, 858, 1, 262,
		{120, 840, 258, 22, 0, 0, 0, 0, 0}
	},
	{VIDEORES_720_240p_60HZ_M2, 1, 858, 1, 263,
		{120, 840, 258, 23, 0, 0, 0, 0, 0}
	},
	{VIDEORES_720_288p_50HZ_M1, 1, 864, 1, 312,
		{133, 853, 310, 24, 0, 0, 0, 0, 0}
	},
	{VIDEORES_720_288p_50HZ_M2, 1, 864, 1, 313,
		{133, 853, 310, 25, 0, 0, 0, 0, 0}
	},
	{VIDEORES_720_288p_50HZ_M3, 1, 864, 1, 314,
		{133, 853, 310, 26, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1360_768p_60HZ, 1, 1792, 1, 795,
		{369, 1729, 792, 27, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1400_1050p_60HZ, 1, 1864, 1, 1089,
		{377, 1777, 1086, 39, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1400_1050p_60HZ_RB, 1, 1560, 1, 1080,
		{113, 1513, 1077, 30, 0, 0, 0, 0, 0}
	},
	{VIDEORES_1024_768p_70HZ, 1, 1328, 1, 806,
		{281, 1305, 803, 38, 0, 0, 0, 0, 0}
	},
	{VIDEORES_640_480p_72HZ, 1, 832, 1, 520,
		{169, 809, 511, 40, 0, 0, 0, 0, 0}
	},
	{VIDEORES_800_600p_72HZ, 1, 1040, 1, 666,
		{185, 985, 629, 66, 0, 0, 0, 0, 0}
	},
	{VIDEORES_640_350p_85HZ, 1, 832, 1, 445,
		{161, 801, 413, 95, 0, 0, 0, 0, 0}
	},
	{VIDEORES_640_400p_85HZ, 1, 832, 1, 445,
 		{161, 801, 444, 45, 0, 0, 0, 0, 0}
	},
	{VIDEORES_720_400p_85HZ, 1, 936, 1, 446,
		{181, 901, 445, 46, 0, 0, 0, 0, 0}
	}
};

const resolution_data_t supported_res[] =
{
	/* Low TV */
	{VIDEORES_1280_720p_24HZ, 1280,720,24,0, 1117178, 1134065, 1488, 1513, 17, 19},
	{VIDEORES_1280_720p_25HZ, 1280,720,25,0, 1072491, 1087614, 1428, 1451, 13, 15},
	{VIDEORES_1280_720p_30HZ, 1280,720,30,0, 893742, 907252, 1190, 1210, 13, 15},
	{VIDEORES_1920_1080p_24HZ, 1920,1080,24,0, 1117178, 1134065, 992, 1009, 14, 17},
	{VIDEORES_1920_1080p_25HZ, 1920,1080,25,0, 1072491, 1087614, 952, 967, 14, 17},
	{VIDEORES_1920_1080p_30HZ, 1920,1080,30,0, 893742, 907252, 794, 806, 14, 17},

	/* 60 Hz TV */
	{VIDEORES_720_480p_60HZ, 720,480,60,0, 446870, 453626, 850, 865, 60, 63},
	{VIDEORES_1280_720p_60HZ, 1280,720,60,0, 446870, 453626, 594, 605, 13, 15},
	{VIDEORES_1920_1080i_60HZ, 1920,1080,60,1, 446870, 453626, 793, 807, 14, 17},
	{VIDEORES_720_480i_60HZ, 720,480,60,1, 446870, 453626, 1701, 1729, 122, 125},
	{VIDEORES_1920_1080p_60HZ, 1920,1080,60,0, 446870, 453626, 396, 404, 6, 9},

	/* 50 Hz TV */
	{VIDEORES_720_576p_50HZ, 720,576,50,0, 536245, 543807, 856, 871, 62, 65},
	{VIDEORES_1280_720p_50HZ, 1280,720,50,0, 536245, 543807, 713, 726, 13, 15},
	{VIDEORES_1920_1080i_50HZ, 1920,1080,50,1, 536245, 543807, 952, 967, 14, 17},
	{VIDEORES_720_576i_50HZ, 720,576,50,1, 536245, 543807, 1714, 1741, 124, 127},
	{VIDEORES_1920_1080p_50HZ, 1920,1080,50,0, 536245, 543807, 475, 484, 6, 9},

	/* 60 HZ PC */
	{VIDEORES_640_480p_60HZ,	640,480,60,0, 446870, 453626, 850, 865, 101, 104},
	{VIDEORES_800_600p_60HZ,	800,600,60,0, 444523, 450791, 708, 718, 84, 88},
	{VIDEORES_1024_768p_60HZ, 1024,768,60,0, 446842, 453142, 554, 562, 54, 58},
	{VIDEORES_1280_768p_60HZ, 1280,768,60,0, 447842, 454156, 561, 569, 41, 46},
	{VIDEORES_1280_960p_60HZ, 1280,960,60,0, 446872, 453172, 447, 453, 26, 30},
	{VIDEORES_1280_1024p_60HZ, 1280,1024,60,0, 446723, 453021, 419, 425, 26, 30},
	{VIDEORES_1440_900p_60HZ, 1440,900,60,0, 446723, 453021, 478, 486, 35, 40},
	{VIDEORES_1600_1200p_60HZ, 1600,1200,60,0, 446872, 453172, 357, 363, 30, 34},
	{VIDEORES_1680_1050p_60HZ_RB, 1680,1050,60,0, 447745, 454058, 415, 420, 5, 9},
	{VIDEORES_1920_1200p_60HZ_RB, 1920,1200,60,0, 447235, 453550, 362, 367, 4, 8},

	/* 75 HZ PC */
	{VIDEORES_640_480p_75HZ,	640,480,75,0, 357498, 362538, 715, 725, 53, 57},
	{VIDEORES_800_600p_75HZ,	800,600,75,0, 357498, 362538, 572, 580, 42, 46},
	{VIDEORES_1024_768p_75HZ, 1024,768,75,0, 357359, 362398, 447, 453, 31, 35},
	{VIDEORES_1280_768p_75HZ, 1280,768,75,0, 357480, 362520, 444, 450, 32, 36},
	{VIDEORES_1280_1024p_75HZ, 1280,1024,75,0, 357378, 362417, 335, 340, 27, 31},

	/* 85 HZ PC */
	{VIDEORES_640_480p_85HZ,	640,480,85,0, 315409, 319856, 620, 628, 40, 44},
	{VIDEORES_800_600p_85HZ,	800,600,85,0, 315213, 319657, 500, 507, 29, 33},
	{VIDEORES_1024_768p_85HZ, 1024,768,85,0, 315450, 319898, 390, 396, 25, 29},
	{VIDEORES_1280_768p_85HZ, 1280,768,85,0, 315423, 319871, 391, 396, 29, 33},
	{VIDEORES_1280_1024p_85HZ, 1280,1024,85,0, 315350, 319796, 294, 298, 26, 30},

	/* Other resolutions */
	{VIDEORES_720_240p_60HZ_M1, 720,240,60,0, 446017, 452305, 1702, 1726, 122, 126},
	{VIDEORES_720_240p_60HZ_M2, 720,240,60,0, 447723, 454035, 1702, 1726, 122, 126},
	{VIDEORES_720_288p_50HZ_M1, 720,288,50,0, 535390, 542938, 1716, 1740, 124, 128},
	{VIDEORES_720_288p_50HZ_M2, 720,288,50,0, 537106, 544678, 1716, 1740, 124, 128},
	{VIDEORES_720_288p_50HZ_M3, 720,288,50,0, 538822, 546419, 1716, 1740, 124, 128},
	{VIDEORES_1360_768p_60HZ,	 1360,768,60,0, 446760, 453059, 562, 570, 33, 37},
	{VIDEORES_1400_1050p_60HZ,	1400,1050,60,0, 447036, 453338, 411, 416, 30, 34},
	{VIDEORES_1400_1050p_60HZ_RB, 1400,1050,60,0, 447260, 453565, 414, 420, 7, 11},

	{VIDEORES_1024_768p_70HZ,	 1024,768,70,0, 382656, 388051, 475, 481, 47, 51},
	{VIDEORES_640_480p_72HZ,	 640,480,72,0, 368255, 373447, 708, 718, 32, 36},
	{VIDEORES_800_600p_72HZ,	 800,600,72,0, 371423, 376660, 558, 566, 63, 67},

	{VIDEORES_640_350p_85HZ,	 640,350,85,0, 315142, 319585, 708, 718, 53, 57},
	{VIDEORES_640_400p_85HZ,	 640,400,85,0, 315142, 319585, 708, 718, 53, 57},
	{VIDEORES_720_400p_85HZ,	 720,400,85,0, 315294, 319740, 707, 717, 53, 57}
};

/* Platform Data */
struct tda1997x_platform_data {

	/* Misc */
	char hdcp;          /* enable HDCP */
	char external_edid; /* use external EDID */
	char ddc_slave;     /* DDC i2c slave address */

	/* Audio */
	tda1997x_audiofmt_t audout_format;    /* output data format */
	tda1997x_audiosysclk_t audout_sysclk; /* clock config */
	tda1997x_audiolayout_t audout_layout; /* physical bus layout */
	bool audio_force_channel_assignment;  /* use AUD IF info if unset */
	bool audio_auto_mute;                 /* enable hardware audio auto-mute */
	bool audout_invert_clk;               /* data valid on rising edge of BCLK */

	/* Video */
	tda1997x_videofmt_t vidout_format;    /* video output data format */
	bool vidout_blc;                      /* insert blanking codes (SAV/EAV) */
	bool vidout_trc;                      /* insert timing codes */
	tda1997x_videoclkmode_t vidout_clkmode; /* clock mode */
	/* pin polarity (1=invert) */
	bool vidout_invert_de;
	bool vidout_invert_hs;
	bool vidout_invert_vs;
	/* clock delays (0=-8, 1=-7 ... 15=+7 pixels) */
	char vidout_delay_hs;
	char vidout_delay_vs;
	char vidout_delay_de;
	char vidout_delay_clk;
	/* sync selections (controls how sync pins are derived) */
	tda1997x_sync_output_hs_t vidout_sel_hs;
	tda1997x_sync_output_vs_t vidout_sel_vs;
	tda1997x_sync_output_de_t vidout_sel_de;
	/* Video port configs */
	u8   vidout_port_config[9];
	u8   vidout_port_config_no;
	/* Max pixel rate (MP/sec) */
	int max_pixel_rate;
};

/**
 * Maintains the information on the current state of the chip
 */
struct tda1997x_data {
	struct i2c_client *client;
	struct i2c_client *client_cec;
	char page;
	struct work_struct work;
	int irq;
	tda1997x_state_t state;
	tda1997x_input_t input;
	spinlock_t lock;
	struct tda1997x_platform_data *pdata;
	struct mutex page_lock;
	struct mutex cec_lock;

	/* detected info from chip */
	int chip;
	int chip_version;
	int chip_revision;
	char eess_detected;
	char hdmi_detected;
	char hdcp_detected;
	char internal_edid;
	char port_30bit;
	char output_2p5;
	char tmdsb_clk;
	char tmdsb_soc;
	char cec_enabled;
	char cec_slave;

	/* status info */
	char hdmi_status;
	char vsi_received;
	char mptrw_in_progress;
	char state_c5_reached;
	char activity_status_reg;
	char input_detect[2];
	char vendor[12];
	char product[18];
	u16 key_decryption_seed;

	/* video source */
	tda1997x_colorspace_t colorspace;
	tda1997x_colorimetry_t colorimetry;
	tda1997x_restype_t resolutiontype;

	/* video source and output format */
	tda1997x_vidout_fmt_t video_mode;

	/* audio source */
	u8  channel_assignment;
	int source_channels;

	/* audio output format */
	tda1997x_audout_fmt_t audio_mode;
};

#ifdef DEBUG
/* kernel parameters */
static int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level");

#define DPRINTK(x, fmt, args...) { if (debug>=x) printk(KERN_DEBUG fmt, ## args); }

/* HPD modes */
const char *hpd_names[] = {
	"HPD_LOW",
	"HPD_LOW_OTHER",
	"HPD_HIGH",
	"HPD_HIGH_OTHER",
	"HPD_PULSE",
};

/* Audio output port layout (hardware pins) */
const char *audlayout_names[] = {
	"AUDIO_LAYOUT_AUTO",
	"AUDIO_LAYOUT_0",
	"AUDIO_LAYOUT_1",
};

/* Audio format */
const char *audfmt_names[] = {
	"AUDIO_FMT_I2S16",
	"AUDIO_FMT_I2S32",
	"AUDIO_FMT_SPDIF",
	"AUDIO_FMT_OBA",
	"AUDIO_FMT_I2S16_HBR_STRAIGHT",
	"AUDIO_FMT_I2S16_HBR_DEMUX",
	"AUDIO_FMT_I2S32_HBR_DEMUX",
	"AUDIO_FMT_SPDIF_HBR_DEMUX",
	"AUDIO_FMT_DST",
};
#else
#define DPRINTK(x, fmt, args...)
#endif

static struct tda1997x_data tda1997x_data;

static int tda1997x_manual_hpd(struct tda1997x_data *tda1997x, hpdmode_t mode);

/** API for CEC
 */
int tda1997x_cec_read(u8 reg)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	int val;

	mutex_lock(&tda1997x->cec_lock);
	val = i2c_smbus_read_byte_data(tda1997x_data.client_cec, reg);
	if (val < 0) {
		dev_dbg(&tda1997x_data.client_cec->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg);
		val = -1;
	}

	mutex_unlock(&tda1997x->cec_lock);
	return val;
}
EXPORT_SYMBOL(tda1997x_cec_read);

int tda1997x_cec_write(u8 reg, u8 val)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	int ret = 0;

	mutex_unlock(&tda1997x->cec_lock);
	ret = i2c_smbus_write_byte_data(tda1997x_data.client_cec, reg, val);
	if (ret < 0) {
		dev_dbg(&tda1997x_data.client_cec->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg, val);
		ret = -1;
	}
	mutex_unlock(&tda1997x->cec_lock);

	return ret;
}
EXPORT_SYMBOL(tda1997x_cec_write);

/** API for MFD children
 */

/** get current state
 */
tda1997x_state_t
tda1997x_get_state(void)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	tda1997x_state_t state;
	unsigned long flags;

	dev_dbg(&tda1997x_data.client->dev, "%s\n", __func__);
	spin_lock_irqsave(&tda1997x->lock, flags);
	state = tda1997x->state;
	spin_unlock_irqrestore(&tda1997x->lock, flags);

	return state;
}
EXPORT_SYMBOL(tda1997x_get_state);

int
tda1997x_get_vidout_fmt(tda1997x_vidout_fmt_t *fmt)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	unsigned long flags;

	dev_dbg(&tda1997x_data.client->dev, "%s\n", __func__);
	spin_lock_irqsave(&tda1997x->lock, flags);
	memcpy(fmt, &(tda1997x->video_mode), sizeof(*fmt));
	fmt->sensor_vidfmt = tda1997x->pdata->vidout_format;
	fmt->sensor_clkmode = tda1997x->pdata->vidout_clkmode;
	spin_unlock_irqrestore(&tda1997x->lock, flags);

	return 0;
}
EXPORT_SYMBOL(tda1997x_get_vidout_fmt);

int
tda1997x_get_audout_fmt(tda1997x_audout_fmt_t *fmt)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	unsigned long flags;

	dev_dbg(&tda1997x_data.client->dev, "%s\n", __func__);
	spin_lock_irqsave(&tda1997x->lock, flags);
	memcpy(fmt, &(tda1997x->audio_mode), sizeof(*fmt));
	spin_unlock_irqrestore(&tda1997x->lock, flags);

	return 0;
}
EXPORT_SYMBOL(tda1997x_get_audout_fmt);


/***********************************************************************
 * I2C transfer
 ***********************************************************************/

/** set the current page
 *
 *  @param page number
 *  @returns 0 if success, an error code otherwise.
 */
static int tda1997x_setpage(u8 page) {
	int ret;

	if (tda1997x_data.page != page) {
		ret = i2c_smbus_write_byte_data(tda1997x_data.client,
			REG_CURPAGE_00H, page);
		if (ret < 0) {
			dev_dbg(&tda1997x_data.client->dev,
				"%s:write reg error:reg=%2x,val=%2x\n", __func__,
				REG_CURPAGE_00H, page);
			return -1;
		}
		tda1997x_data.page = page;
	}
	return 0;
}

/** Read one register from a tda1997x i2c slave device.
 *
 *  @param reg register in the device we wish to access.
 *         high byte is page, low byte is reg
 *  @returns 0 if success, an error code otherwise.
 */
static inline int io_read(u16 reg)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	int val;

	/* page 80 denotes CEC reg which needs to go out cec_client */
	BUG_ON(reg>>8 == 0x80);

	mutex_lock(&tda1997x->page_lock);
	if (tda1997x_setpage(reg>>8)) {
		val = -1;
		goto out;
	}

	val = i2c_smbus_read_byte_data(tda1997x_data.client, reg&0xff);
	if (!(reg == REG_INT_FLG_CLR_TOP && val == 0x00)) {
		DPRINTK(3, "<< 0x%04x=0x%02x\n", reg, val);
	}
	if (val < 0) {
		dev_dbg(&tda1997x_data.client->dev,
			"%s:read reg error: reg=%2x\n", __func__, reg&0xff);
		val = -1;
		goto out;
	}

out:
	mutex_unlock(&tda1997x->page_lock);
	return val;
}
/* 16bit read */
static inline long io_read16(u16 reg)
{
	u8 val;
	long lval = 0;

	if ( (val = io_read(reg)) < 0)
		return -1;
	lval |= (val<<8);
	if ( (val = io_read(reg+1)) < 0)
		return -1;
	lval |= val;

	return lval;
}
/* 24bit read */
static inline long io_read24(u16 reg)
{
	u8 val;
	long lval = 0;

	if ( (val = io_read(reg)) < 0)
		return -1;
	lval |= (val<<16);
	if ( (val = io_read(reg+1)) < 0)
		return -1;
	lval |= (val<<8);
	if ( (val = io_read(reg+2)) < 0)
		return -1;
	lval |= val;

	return lval;
}
/* n-byte read */
static unsigned int io_readn(u16 reg, u8 len, u8 *data)
{
	int i;
	int sz = 0;
	u8 val;

	for (i = 0; i < len; i++) {
		if ( (val = io_read(reg + i)) < 0)
			break;
		data[i] = val;
		sz++;	
	}

	return sz;
}

/** Write one register of a tda1997x i2c slave device.
 *
 *  @param reg register in the device we wish to access.
 *         high byte is page, low byte is reg
 *  @returns 0 if success, an error code otherwise.
 */
static int io_write(u16 reg, u8 val)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	s32 ret = 0;

	/* page 80 denotes CEC reg which needs to go out cec_client */
	BUG_ON(reg>>8 == 0x80);

	mutex_lock(&tda1997x->page_lock);
	if (tda1997x_setpage(reg>>8)) {
		ret = -1;
		goto out;
	}

	DPRINTK(3, ">> 0x%04x=0x%02x\n", reg, val);
	ret = i2c_smbus_write_byte_data(tda1997x_data.client, reg&0xff, val);
	if (ret < 0) {
		dev_dbg(&tda1997x_data.client->dev,
			"%s:write reg error:reg=%2x,val=%2x\n", __func__,
			reg&0xff, val);
		ret = -1;
		goto out;
	}

out:
	mutex_unlock(&tda1997x->page_lock);
	return ret;
}
/* 16bit write */
static int io_write16(u16 reg, u16 val)
{
	if (io_write(reg, (val>>8)&0xff) < 0)
		return -1;
	if (io_write(reg+1, val&0xff) < 0)
		return -1;
	return 0;
}
/* 24bit write */
static int io_write24(u16 reg, u32 val)
{
	if (io_write(reg, (val>>16)&0xff) < 0)
		return -1;
	if (io_write(reg+1, (val>>8)&0xff) < 0)
		return -1;
	if (io_write(reg+2, val&0xff) < 0)
		return -1;
	return 0;
}
/* n-byte write */
static unsigned int io_writen(u16 reg, u8 len, u8 *data)
{
	int i;
	int sz = 0;

	for (i = 0; i < len; i++) {
		io_write(reg + i, data[i]);
		sz++;
	}

	return sz;
}


/***********************************************************************
 * EDID
 ***********************************************************************/

/* Modified from the NXP exapp71a application for Ventana */
u8 edid_block[256] = {
 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00,
 0x1e, 0xe3, 0x00, 0x50, 0x01, 0x00, 0x00, 0x00,
 0x01, 0x17, 0x01, 0x03, 0xa2, 0x00, 0x00, 0x00,
 0x02, 0xee, 0x95, 0xa3, 0x54, 0x4c, 0x99, 0x26,
 0x0f, 0x50, 0x54, 0x20, 0x00, 0x00, 0x01, 0x01,
 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a,
 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
 0x45, 0x00, 0xc4, 0x8e, 0x21, 0x00, 0x00, 0x1e,
 0x00, 0x00, 0x00, 0xfc, 0x00, 0x47, 0x57, 0x20,
 0x56, 0x65, 0x6e, 0x74, 0x61, 0x6e, 0x61, 0x0a,
 0x20, 0x20, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x1e,
 0x3c, 0x0f, 0x44, 0x09, 0x00, 0x0a, 0x20, 0x20,
 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfe,
 0x00, 0x31, 0x30, 0x38, 0x30, 0x70, 0x44, 0x43,
 0x78, 0x76, 0x43, 0x33, 0x44, 0x0a, 0x01, 0x89,
 0x02, 0x03, 0x21, 0xf0, 0x4c, 0x22, 0x20, 0x21,
 0x04, 0x13, 0x03, 0x12, 0x05, 0x14, 0x07, 0x16,
 0x01, 0x23, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00,
 0x00, 0x67, 0x03, 0x0c, 0x00, 0x00, 0x00, 0xb8,
 0x2d, 0x01, 0x1d, 0x80, 0x18, 0x71, 0x38, 0x2d,
 0x40, 0x58, 0x2c, 0x45, 0x00, 0x80, 0x38, 0x74,
 0x00, 0x00, 0x3f, 0x01, 0x1d, 0x00, 0x72, 0x51,
 0xd0, 0x1e, 0x20, 0x6e, 0x50, 0x55, 0x00, 0x00,
 0xd0, 0x52, 0x00, 0x00, 0x39, 0xa0, 0x0f, 0x20,
 0x00, 0x31, 0x58, 0x1c, 0x20, 0x28, 0x80, 0x14,
 0x00, 0x20, 0x58, 0x32, 0x00, 0x00, 0x3f, 0xd7,
 0x09, 0x80, 0xa0, 0x20, 0xe0, 0x2d, 0x10, 0x08,
 0x60, 0x22, 0x01, 0x80, 0xe0, 0x21, 0x00, 0x00,
 0x39, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa7,
};

/* DDC config 0 (page 0x20) */
u8 ddc_config0[8] = {
	/* 0x80: EDID_VERSION */
	0x19, /* main=0x19 */

	/* 0x81: EDID_ENABLE bits:
	 *   7 - unused
	 *   6 - edid_only
	 * 5:2 - unused
	 *   1 - edid_b_en
	 *   0 - edid_a_en
	 */
	0x41, /* enable EDID for A only */

	/* 0x82: EDID_BLOCK_SELECT bits:
	 * 7:6 - ddc_b_blk1_sel
	 * 5:4 - ddc_b_blk0_sel
	 * 3:2 - ddc_a_blk1_sel
	 * 1:0 - ddc_a_blk0_sel
	 */
	0x44, /* ddc_b_blk1_sel=1 ddc_a_blk1_sel=1 */

	/* 0x83: empty */
	0x00,

	/* 0x84: HPD_POWER bits:
	 * 7:5 - unused
	 * 4:3 - hpd_bp
	 *   2 - hpd_edid_only
	 * 1:0 - unused
	 */
	0x06, /* hpd_bp=1 hpd_edid_only=1 */

	/* 0x85: HPD_AUTO_CTRL bits
	 *   7 - read_edid
	 *   6 - unused
	 *   5 - hpd_f3tech
	 *   4 - hpd_other
	 *   3 - hpd_unsel
	 *   2 - hpd_all_ch
	 *   1 - hpd_prv_ch
	 *   0 - hpd_new_ch
	 */
	0x18, /* hpd_other=1, hdp_unsel=1 */

	/* 0x86: HPD_DURATION */
	0x00, /* hpd_duration=0 */

	/* 0x87: RX_HPD_HEAC bits
	 * 7:2 - unused
	 *   1 - heac_b_en
	 *   0 - heac_a_en
	 */
	0x00,
};

/* RT config (page 0x30) */
u8 rt_config[6] = {
	/* 0x00: RT_AUTO_CTRL bits:
	 *   7 - unused
	 *   6 - rt_other
	 *   5 - rt_hpd_low
	 *   4 - rt_no_5v
	 *   3 - rt_unsel
	 *   2 - rt_all_ch
	 *   1 - rt_prv_ch
	 *   0 - rt_new_ch
	 */
	0x78,

	/* 0x01: EQ_MAN_CTRL0 bits:
	 * 7-3 - unused
	 * 2-0 - ch0_man_gain
	 */
	0x03, /* ch0_man_gain=3 */

	/* 0x02: EQ_MAN_CTRL1 bits:
	 *   7 - unused
	 * 6-4 - ch1_man_gain
	 *   3 - unused
	 * 2-0 - ch2_man_gain
	 */
	0x33, /* ch1_man_gain=3 ch2_man_gain=3 */

	/* 0x03: OUTPUT_CFG bits
	 *   7 - eq_cal_act
	 *   6 - eq_cal_ch
	 *   5 - eq_cal_5v
	 *   4 - eq_cal_cond
	 *   3 - unused
	 *   2 - out_idle
	 *   1 - ap_idle
	 *   0 - vp_idle
	 */
	0xf0, /* eq_cal_act=1 eq_cal_ch=1 eq_cal_5v=1 eq_cal_cond=1 */

	/* 0x04: MUTE_CTRL bits
	 * 7:3 - unused
	 *   2 - mute_man
	 *   1 - mute_pol
	 *   0 - mute_ena
	 */
	0x00,

	/* 0x05: SLAVE_ADDR bits
	 * 7:6 - unused
	 *   5 - i2c_cec_a1
	 *   4 - i2c_cec_a0
	 *   3 - unused
	 *   2 - i2c_a2
	 *   1 - i2c_a1
	 *   0 - i2c_a0
	 */
	0x10, /* i2c_cec_a0=1 */
};

static u32 reg = 0;

/** Loads EDID data into embedded EDID memory of receiver device
 * @edid - pointer to two block EDID (array of 256 bytes) common to both inputs
 *
 * A common EDID block is used for both inputs with the exception of the
 * SPA (Source Physical Address) used for CEC. Therefore, we locate the SPA
 * within the EDID passed and treating it as the SPA for InputA while adding
 * 1 to it for the SPA for InputB. The EDID is written into nvram as well
 * as the SPA offset, SPA's and adjusted checksums for both inputs.
 */
static int tda1997x_load_edid_data(u8 *edid)
{
	u8 chksum, chksum_spa;
	int i, n, spa_offset = 0;
	u16 spa;

	DPRINTK(0,"%s\n", __func__);

	/* sanity check EDID data: base block, extensions, checksums */
	if (!drm_edid_is_valid((struct edid *) edid)) {
		printk(KERN_ERR "edid data is invalid\n");
		return -EINVAL;
	}
	if (edid[0x7e] != 1) {
		printk(KERN_ERR "edid requires a single CEA extension block\n");
		return -EINVAL;
	}

	/* Find SPA offset within CEA extentsion */
	for (i = 1; i <= edid[0x7e]; i++) {
		u8 *ext = edid + (i * EDID_LENGTH);

		/* look for CEA v3 extension */
		if (ext[0] != 0x02 || ext[1] != 0x03)
			continue;

		/* make sure we have a DBC */
		if (ext[2] < 5)
			continue;

		/* iterate through DBC's until we find the Vendor block */
		for (n = 4; n < ext[2]; n++) {
			char type = (ext[n] & 0xe0) >> 5;
			char len = (ext[n] & 0x1f);
			if (type == 3) {
				if (ext[n+1] == 0x03 && ext[n+2] == 0x0c) {
					spa_offset = n+4;
					spa = ext[spa_offset] << 8 |
					      ext[spa_offset+1];
				}
			}
			n += len;
		}
	}
	if (!spa_offset) {
		printk(KERN_ERR
		       "EDID requires an HDMI Vendor Specific Data Block\n");
		return -EINVAL;
	}

	/* calculate ext block checksum w/o SPA */
	chksum = 0;
	for (i = 0; i < 127; i++)
		if (i != spa_offset && i != (spa_offset + 1))
			chksum += edid[i+128];

	/* write base EDID */
	for (i = 0; i < 128; i++)
		io_write(REG_EDID_IN_BYTE0 + i, edid[i]);

	/* write CEA Extension */
	for (i = 0; i < 128; i++)
		io_write(REG_EDID_IN_BYTE128 + i, edid[i+128]);

	/* SPA for InputA */
	io_write16(REG_EDID_IN_SPA_AB_A, spa);
	chksum_spa = (u8)((spa & 0xff00)>>8) + (u8)(spa & 0x00ff) + chksum;
	chksum_spa = (u8)((0xff - chksum_spa) + 0x01); /* 2's complement */
	io_write(REG_EDID_IN_CKSUM_A, chksum_spa);

	/* SPA for InputB */
	spa += 1;
	io_write16(REG_EDID_IN_SPA_AB_B, spa);
	chksum_spa = (u8)((spa & 0xff00)>>8) + (u8)(spa & 0x00ff) + chksum;
	chksum_spa = (u8)((0xff - chksum_spa) + 0x01); /* 2's comp */
	io_write(REG_EDID_IN_CKSUM_B, chksum_spa);

	/* write source physical address subaddress offset */
	io_write(REG_EDID_IN_SPA_SUB, spa_offset);

	return 0;
}

/*
 * sysfs hooks
 */
static ssize_t b_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	int rz = 0;
	const char *name = attr->attr.name;
	struct tda1997x_data *tda1997x = &tda1997x_data;
	unsigned long flags;
	const char *sname = "";

	spin_lock_irqsave(&tda1997x->lock, flags);
	if (strcasecmp(name, "state") == 0) {
		switch(tda1997x->state) {
			case STATE_NOT_INITIALIZED: sname = "not initialized"; break;
			case STATE_INITIALIZED:     sname = "initalized"; break;
			case STATE_LOCKED:          sname = "locked"; break;
			case STATE_UNLOCKED:        sname = "unlocked"; break;
			case STATE_CONFIGURED:      sname = "configured"; break;	
		}
		rz = sprintf(buf, "%s\n", sname);
	}
	else if (strcasecmp(name, "vidmode") == 0) {
		if (tda1997x->state == STATE_LOCKED) {
			rz = sprintf(buf, "%dx%d%c@%dHz\n",
				tda1997x->video_mode.width, tda1997x->video_mode.height,
				(tda1997x->video_mode.interlaced)?'i':'p',
				tda1997x->video_mode.fps);
		} else
			rz = sprintf(buf, "no signal\n");
	} else if (strcasecmp(name, "colorspace") == 0) {
		rz = sprintf(buf, "%s %s\n",
			     colorspace_names[tda1997x->colorspace],
			     colorimetry_names[tda1997x->colorimetry]);
	} else if (strcasecmp(name, "audmode") == 0) {
		if (tda1997x->state == STATE_LOCKED) {
			rz = sprintf(buf, "%dHz\n", tda1997x->audio_mode.samplerate);
		} else
			rz = sprintf(buf, "no signal\n");
	} else if (strcasecmp(name, "vendor") == 0) {
		rz = sprintf(buf, "%s\n", tda1997x->vendor);
	} else if (strcasecmp(name, "product") == 0) {
		rz = sprintf(buf, "%s\n", tda1997x->product);
	} else if (strcasecmp(name, "info") == 0) {
		rz = sprintf(buf, "%s%s%s\n", 
			(tda1997x->hdmi_detected)?"HDMI ":"",
			(tda1997x->eess_detected)?"EESS ":"",
			(tda1997x->hdcp_detected)?"HDCP ":"");
	} else if (strcasecmp(name, "edid") == 0) {
		for (rz = 0; rz < sizeof(edid_block); rz++)
			buf[rz] = edid_block[rz];
	} else if (strcasecmp(name, "reg") == 0) {
		rz = sprintf(buf, "%02x\n", io_read(reg) );
		printk(KERN_INFO "TDA1997x-core: Register 0x%04x=%s\n", reg, buf);
	} else {
		rz = sprintf(buf, "invalid attr\n");
	}
	spin_unlock_irqrestore(&tda1997x->lock, flags);

	return rz;
}

static ssize_t b_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	u32 val = 0;
	const char *name = attr->attr.name;
	struct tda1997x_data *tda1997x = &tda1997x_data;
	int i;


	if (strcasecmp(name, "edid") == 0) {
		for (i = 0; i < count; i++)
			edid_block[i] = buf[i];
		printk(KERN_INFO "TDA1997x-core: New EDID being loaded\n");
		/* Set HPD low */
		tda1997x_manual_hpd(tda1997x, HPD_LOW);
		tda1997x_load_edid_data(edid_block);
		/* Set HPD high (now that EDID is ready) */
		tda1997x_manual_hpd(tda1997x, HPD_HIGH);
	} else if (strcasecmp(name, "reg") == 0) {
			i = sscanf(buf, "%x %x", &reg, &val);
			if (i == 2) {
				io_write( (u16)reg, (u8)val);
				printk(KERN_INFO "TDA1997x-core: Register 0x%04x=0x%02x\n", reg, val);
			} else {
				printk(KERN_INFO "TDA1997x-core: Register 0x%04x\n", reg);
			}
	} else {
		printk(KERN_ERR "invalid name '%s'\n", attr->attr.name);
	}

	return count;
}

/*
 * Create a group of attributes so that we can create and destory them all
 * at once.
 */
static struct device_attribute attr_state =
	__ATTR(state, 0660, b_show, b_store);
static struct device_attribute attr_vidmode =
	__ATTR(vidmode, 0660, b_show, NULL);
static struct device_attribute attr_audmode =
	__ATTR(audmode, 0660, b_show, NULL);
static struct device_attribute attr_vendor =
	__ATTR(vendor, 0660, b_show, NULL);
static struct device_attribute attr_product =
	__ATTR(product, 0660, b_show, NULL);
static struct device_attribute attr_info =
	__ATTR(info, 0660, b_show, NULL);
static struct device_attribute attr_edid =
	__ATTR(edid, 0770, b_show, b_store);
static struct device_attribute attr_colorspace =
	__ATTR(colorspace, 0660, b_show, NULL);
static struct device_attribute attr_reg =
	__ATTR(reg, 0660, b_show, b_store);

static struct attribute *tda1997x_attrs[] = {
	&attr_state.attr,
	&attr_vidmode.attr,
	&attr_audmode.attr,
	&attr_vendor.attr,
	&attr_product.attr,
	&attr_info.attr,
	&attr_edid.attr,
	&attr_colorspace.attr,
	&attr_reg.attr,
	NULL
};

static struct attribute_group attr_group = {
	.attrs = tda1997x_attrs,
};

/** Loads DDC and RT configuration data into embedded memory of receiver device
 * @ddc_config - pointer to the DDC block configuration (8 bytes)
 * @rt_config  - pointer to the RT block configuration (6 bytes)
 */
static int tda1997x_load_config_data(struct tda1997x_data *tda1997x,
	const u8 *ddc, const u8 *rt)
{
	int i;

	if (!tda1997x->internal_edid)
		return -EPERM;

	/* DDC to page 20h */
	for (i = 0; i < 8; i++)
		io_write(REG_EDID_IN_VERSION + i, ddc[i]);
	/* RT to page 30h */
	for (i = 0; i < 6; i++)
		io_write(REG_RT_AUTO_CTRL + i, rt[i]);

	return 0;
}


/***********************************************************************
 * Signal Control
 ***********************************************************************/

/** manual HPD (Hot Plug Detect) control
 * @param hdp_mode
 * @returns 0 on success
 */
static int tda1997x_manual_hpd(struct tda1997x_data *tda1997x, hpdmode_t mode)
{
	u8 hpd_auto, hpd_pwr, hpd_man;

	DPRINTK(0, "%s: %s\n", __func__, hpd_names[mode]);

	/* HPD_POWER bits:
	 * 7:4 - unused
	 * 3:2 - hpd_bp[1:0]
	 *   1 - hpd_edid_only
	 *   0 - unused
	 *
	 * HPD_AUTO_CTRL bits:
	 *   7 - read_edid
	 *   6 - unused
	 *   5 - hpd_f3tech
	 *   4 - hp_other
	 *   3 - hpd_unsel
	 *   2 - hpd_all_ch
	 *   1 - hpd_prv_ch
	 *   0 - hpd_new_ch
	 *
	 * HPD_MAN bits:
	 *   7 - ?
	 * 6-3 - unused
	 * 2:0 - man_gain
	 */
	hpd_auto = io_read(REG_HPD_AUTO_CTRL);
	hpd_pwr = io_read(REG_HPD_POWER);
	hpd_man = io_read(REG_HPD_MAN_CTRL);
	hpd_man &= 0x87;

	switch(mode) {
		/* HPD low and pulse of at least 100ms */
		case HPD_LOW:
			hpd_man &= ~0x03; /* man_gain=0 */
			hpd_pwr &= ~0x0c; /* hpd_bp=0 */
			io_write(REG_HPD_POWER, hpd_pwr);
			io_write(REG_HPD_MAN_CTRL, hpd_man);
			break;
		/* HPD high */
		case HPD_HIGH:
			/* hpd_bp=1 */
			hpd_pwr = (hpd_pwr & ~0x0c) | 0x04;
			io_write(REG_HPD_POWER, hpd_pwr);
			break;
		/* HPD low and pulse of at least 100ms */
		case HPD_LOW_OTHER:
			hpd_man &= ~0x03; /* man_gain=0 */
			hpd_auto &= ~0x10; /* hp_other=0 */
			io_write(REG_HPD_AUTO_CTRL, hpd_auto);
			io_write(REG_HPD_MAN_CTRL, hpd_man);
			break;
		/* HPD high */
		case HPD_HIGH_OTHER:
			hpd_auto |= 0x10; /* hp_other=1 */
			io_write(REG_HPD_AUTO_CTRL, hpd_auto);
			break;
		/* HPD low pulse */
		case HPD_PULSE:
			hpd_man &= ~0x03; /* man_gain=0 */
			/* write HPD_MAN to have HPD low pulse */
			io_write(REG_HPD_MAN_CTRL, hpd_man);
			break;
	}
	
	return 0;
}

/** configure the receiver with the new colorspace
 * @returns 0 on success
 * colorspace conversion depends on input format and output format
 * blanking codes depend on the output colorspace
 */
static int tda1997x_configure_conversion(struct tda1997x_data *tda1997x,
	tda1997x_colorspace_t colorspace, tda1997x_colorimetry_t colorimetry,
	tda1997x_videofmt_t vidout_format)
{
	colormatrixcoefs_t *pCoefficients = NULL;
	blankingcodes_t *pBlankingCodes = NULL;
	u8 data[31];

	printk(KERN_INFO "%s: %s %s => %s\n", KBUILD_MODNAME,
	       colorspace_names[colorspace],
	       (colorspace == COLORSPACE_RGB) ? "" :
	       colorimetry_names[colorimetry],
	       vidfmt_names[vidout_format]);

	switch (vidout_format) {
	/* RGB 4:4:4 output */
	case VIDEOFMT_444:
		pBlankingCodes = &RGBBlankingCode;
		if (colorspace != COLORSPACE_RGB) {
			if (colorimetry == COLORIMETRY_ITU709)
				pCoefficients = &conversion_matrix[ITU709_RGBLimited];
			else
				pCoefficients = &conversion_matrix[ITU601_RGBLimited];
		}
		break;

	/* YUV422 output */
	case VIDEOFMT_422_SMP: /* YUV422 semi-planar */
	case VIDEOFMT_422_CCIR:/* YUV422 CCIR656 */
		pBlankingCodes = &YUVBlankingCode;
		if (colorspace == COLORSPACE_RGB)
			pCoefficients = &conversion_matrix[RGBLimited_ITU601];
		break;
	}

	if (pCoefficients) {
		s16 *pTabCoeff = &(pCoefficients->offInt1);
		u8 i, j;

		/* test the range of the coefs */
		for (i = 0; i < OFFSET_LOOP_NB; i++) {
			for (j = 0; j < MAT_OFFSET_NB; j++) {
				if ( (pTabCoeff[j] < MIN_VAL_OFFSET) || pTabCoeff[j] > MAX_VAL_OFFSET)
					return -EINVAL;
			}
			pTabCoeff = &(pCoefficients->offOut1);
		}
		pTabCoeff = &(pCoefficients->P11Coef);
	
		for (i = 0; i < MAT_COEFF_NB; i++) {
			if ( (pTabCoeff[i] < MIN_VAL_OFFSET) || pTabCoeff[i] > MAX_VAL_OFFSET)
				return -EINVAL;
		}
	
		/* enable matrix conversion by disabling bypass: mat_bp=0 */
		data[0] = io_read(REG_VDP_CTRL);
		data[0] &= ~(1<<0);
		/* offset input 1 value */
		data[1] = (u8) ((u16)(pCoefficients->offInt1) >> 8);
		data[2] = (u8) (pCoefficients->offInt1 & MASK_MAT_COEFF_LSB);
		/* offset input 2 value */
		data[3] = (u8) ((u16)(pCoefficients->offInt2) >> 8);
		data[4] = (u8) (pCoefficients->offInt2 & MASK_MAT_COEFF_LSB);
		/* offset input 3 value */
		data[5] = (u8) ((u16)(pCoefficients->offInt3) >> 8);
		data[6] = (u8) (pCoefficients->offInt3 & MASK_MAT_COEFF_LSB);
		/* Coefficient (1,1) value */
		data[7] = (u8) ((u16)(pCoefficients->P11Coef) >> 8);
		data[8] = (u8) (pCoefficients->P11Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (1,2) value */
		data[9] = (u8) ((u16)(pCoefficients->P12Coef) >> 8);
		data[10] = (u8) (pCoefficients->P12Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (1,3) value */
		data[11] = (u8) ((u16)(pCoefficients->P13Coef) >> 8);
		data[12] = (u8) (pCoefficients->P13Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (2,1) value */
		data[13] = (u8) ((u16)(pCoefficients->P21Coef) >> 8);
		data[14] = (u8) (pCoefficients->P21Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (2,2) value */
		data[15] = (u8) ((u16)(pCoefficients->P22Coef) >> 8);
		data[16] = (u8) (pCoefficients->P22Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (2,3) value */
		data[17] = (u8) ((u16)(pCoefficients->P23Coef) >> 8);
		data[18] = (u8) (pCoefficients->P23Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (3,1) value */
		data[19] = (u8) ((u16)(pCoefficients->P31Coef) >> 8);
		data[20] = (u8) (pCoefficients->P31Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (3,2) value */
		data[21] = (u8) ((u16)(pCoefficients->P32Coef) >> 8);
		data[22] = (u8) (pCoefficients->P32Coef & MASK_MAT_COEFF_LSB);
		/* Coefficient (3,3) value */
		data[23] = (u8) ((u16)(pCoefficients->P33Coef) >> 8);
		data[24] = (u8) (pCoefficients->P33Coef & MASK_MAT_COEFF_LSB);
		/* Offset output 1 value */
		data[25] = (u8) ((u16)(pCoefficients->offOut1) >> 8);
		data[26] = (u8) (pCoefficients->offOut1 & MASK_MAT_COEFF_LSB);
		/* Offset output 2 value */
		data[27] = (u8) ((u16)(pCoefficients->offOut2) >> 8);
		data[28] = (u8) (pCoefficients->offOut2 & MASK_MAT_COEFF_LSB);
		/* Offset output 3 value */
		data[29] = (u8) ((u16)(pCoefficients->offOut3) >> 8);
		data[30] = (u8) (pCoefficients->offOut3 & MASK_MAT_COEFF_LSB);

		io_writen(REG_VDP_CTRL, 31, data);
	} else {
		/* enable matrix conversion bypass (mat_b) */
		data[0] = io_read(REG_VDP_CTRL);
		data[0] |= (1<<0); /* mat_bp=1 */
		io_write(REG_VDP_CTRL, data[0]);
	}

	/* SetBlankingCodes */
	if (pBlankingCodes) {
		io_write16(REG_BLK_GY, pBlankingCodes->blankingCodeGy);
		io_write16(REG_BLK_BU, pBlankingCodes->blankingCodeBu);
		io_write16(REG_BLK_RV, pBlankingCodes->blankingCodeRv);
	}

	return 0;
}

/** Configure the active input to the given resolution
 * @param resolution ID
 * @returns 0 on success
 */
static int
tda1997x_configure_input_resolution(resolutionid_t resolution)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	u8 reg;
	const resolution_timings_t *timings = NULL;
	const vhref_values_t *vh;
	int i;

	DPRINTK(0,"%s\n", __func__);

	/* scan through resolution table looking for a match for timings */
	for (i = 0; i < ARRAY_SIZE(resolution_timings); i++) {
		if (resolution == resolution_timings[i].resolutionID)
		{
			timings = &resolution_timings[i];
			vh = &timings->vhref_values;
			break;
		}
	}
	if (!timings) {
		printk(KERN_INFO "%s: resolution not supported\n", KBUILD_MODNAME);
		return -1;
	}

	/* Configure Frame Detection Window:
	 * define the horizontal area where the VHREF modules consider a VSYNC a
	 * new frame (values typically depend on the video mode being processed
	 */
	/* start position of the frame detection window */
	io_write16(REG_FDW_S, 0x2ef & 0x3fff);
	/* end position of the frame detection window */
	io_write16(REG_FDW_E, 0x141 & 0x3fff);

	/* Set Pixel And Line Counters */
	if (tda1997x->chip_revision == 0)
		/* add 1 line to lineCountPreset */
		io_write16(REG_PXCNT_PR, (timings->pixCountPreset + 3) & 0x3fff);
	else
		io_write16(REG_PXCNT_PR, timings->pixCountPreset & 0x3fff);
	io_write16(REG_PXCNT_NPIX, timings->pixCountNb & 0x3fff);
	io_write16(REG_LCNT_PR, timings->lineCountPreset & 0x3fff);
	io_write16(REG_LCNT_NLIN, timings->lineCountNb & 0x3fff);

	/* Configure VHRef:
	 * configure the VHRef timing generator responsible for rebuilding all
	 * horiz and vert synch and ref signals from its input allowing automatic
	 * detection algorithms and forcing predefined modes (480i & 576i)
	 *
	 * REG_VHREF_CTRL bits:
	 * bit7   - interlaced_det - interlace detect method: 1=alternate,0=framefield
	 * bit6:5 - vsync_type: 0=Auto,1=FDW,2=Even,3=Odd
	 * bit4:3 - std_det: 0=PAL,1=NTSC,2=AUTO,3=OFF
	 * bit2   - href_src - VREF source: 1=from standard, 0=manual
	 * bit1   - href_src - HREF source: 1=from standard, 0=manual
	 * bit0   - hsync_sel - HSYNC signal: 1=HS,0=VS
	 */
	reg = io_read(REG_VHREF_CTRL);
	io_write(REG_VHREF_CTRL, 0x3<<3 /* std_det=off */);

	/* Set VHRef:
	 * configure the VHRef timing values.  In case the VHREF generator has
	 * been configured in manual mode, this will allow to manually set all horiz
	 * and vert ref values (non-active pixel areas) of the generator
	 * and allows setting the frame reference params.  These values typically
	 * depend on the video mode being processed
	 */
	/* horizontal reference start/end */
	io_write16(REG_HREF_S, vh->href_start & 0x3fff);
	io_write16(REG_HREF_E, vh->href_end & 0x3fff);
	/* vertical reference f1 start/end */
	io_write16(REG_VREF_F1_S, vh->vref_f1_start & 0x3fff);
	io_write  (REG_VREF_F1_WIDTH, vh->vref_f1_width);
	/* vertical reference f2 start/end */
	io_write16(REG_VREF_F2_S, vh->vref_f2_start & 0x3fff);
	io_write  (REG_VREF_F2_WIDTH, vh->vref_f2_width);
	/* F1/F2 FREF, field polarity */
	io_write16(REG_FREF_F1_S, (vh->fieldref_f1_start & 0x3fff)
		|| (vh->fieldPolarity<<8));
	io_write16(REG_FREF_F2_S, vh->fieldref_f2_start & 0x3fff);

	/* set the state machine */
	tda1997x->state = STATE_CONFIGURED;

	return 0;
}

/** Configure Audio output formatter
 * @param pdata with audio output configuration
 * @param channel_assignment
 * @returns 0 on success
 */
static int
tda1997x_configure_audio_formatter(struct tda1997x_platform_data *pdata,
	u8 channel_assignment)
{
	u8 fifo_latency = 0x80;
	bool sp_used_by_fifo = 0;
	bool ws_active = 0;
	u8 reg;

	DPRINTK(0,"%s: %s %s channel_assignment=%d enable_auto_mute=%d\n", __func__,
		audlayout_names[pdata->audout_layout],
		audfmt_names[pdata->audout_format],
		channel_assignment, pdata->audio_auto_mute);

	/* AUDIO_PATH bits:
	 *  7:0 - channel assignment (CEA-861-D Table 20)
	 */
	io_write(REG_AUDIO_PATH, channel_assignment);

	/* AUDIO_SEL bits:
	 *    7 - aclk_inv     - Polarity of clock signal on A_CLK pin (1=invert)
	 *    6 - test_tone    - Audio test tone generator (1=on)
	 *    5 - i2s_spdif    - Selection of audio output format (0=I2S, 1=SPDIF)
	 *    4 - i2s_32       - Size of I2S samples (0=16bit, 1=32bit)
	 *    3 - hwmute_ena   - Automatic audio mute (1=enabled)
	 *    2 - hbr_demux    - High Bit Rate output mode:
	 *      0:straight via AP0
	 *      1:demuxed via AP0,AP1,AP2,AP3
	 *  1:0 - audio_type   - Selection of audio output packet mode
	 *     00: Audio samples
	 *     01: High-Bit Rate (HBR)
	 *     10: One Bit Audio (OBA)
	 *     11: Direct Stream Transfer (DST)
	 */
	reg = io_read(REG_AUDIO_SEL);
	if (pdata->audio_auto_mute)
		reg |= 1<<3;
	else
		reg &= ~(1<<3);
	switch(pdata->audout_format) {
		case AUDIO_FMT_I2S16:
			/* 16bit I2S mode, SP flag used by FIFO */
			reg &= ~(1<<4);
			reg &= ~(1<<5);
			sp_used_by_fifo = 1;
			ws_active = 1;
			break;
		case AUDIO_FMT_I2S32:
			/* 32bit I2S mode, SP flag used by FIFO */
			reg |= (1<<4);
			reg &= ~(1<<5);
			sp_used_by_fifo = 1;
			ws_active = 1;
			break;
		case AUDIO_FMT_OBA:
			/* One Bit Audio */
			reg &= ~(1<<5);
			sp_used_by_fifo = 1;
			ws_active = 1;
			break;
		case AUDIO_FMT_SPDIF:           /* SPDIF */
			/* SPDIF mode, SP flag used by FIFO */
			reg |= (1<<5);
			sp_used_by_fifo = 1;
			break;
		case AUDIO_FMT_I2S16_HBR_DEMUX:
			/* 16bit I2S High Bit Rate demux */
			reg &= ~(1<<4);
			reg &= ~(1<<5);
			sp_used_by_fifo = 1;
			ws_active = 1;
			break;
		case AUDIO_FMT_I2S32_HBR_DEMUX:
			/* 32bit I2S High Bit Rate demux */
			reg |= (1<<4);
			reg &= ~(1<<5);
			sp_used_by_fifo = 1;
			ws_active = 1;
			break;
		case AUDIO_FMT_DST:
			/* Direct Stream Transfer */
			sp_used_by_fifo = 0;
			break;
		case AUDIO_FMT_SPDIF_HBR_DEMUX:
			/* SPDIF High Bit Rate demux */
			reg |= (1<<5);
			sp_used_by_fifo = 0;
			break;
		default:
			break;
	}
	/* Clock polarity */
	reg &= ~(1<<7);
	if (pdata->audout_invert_clk)
		reg |= 1<<7;
	io_write(REG_AUDIO_SEL, reg);

	/* AUDIO_LAYOUT bits:
	 *  2 - sp_flag       :
	 *       0 - sp flag ignored by FIFO-control (4 subpackets written in FIFO)
	 *       1 - sp flag used by FIFO-control (Present samples subpackets written in FIFO)
	 *  1 - layout_man:
	 *       0 - layout defined by audio packet header
	 *       1 - manual control of layout
	 *  0 - layout: value of layout in case of manual selection
	 *       0 - layout0
	 *       1 - layout1
	 */
	io_write(REG_AUDIO_LAYOUT,
		((sp_used_by_fifo)?(1<<2):0) | pdata->audout_layout);

	/* FIFO Latency value */
	io_write(REG_FIFO_LATENCY_VAL, fifo_latency);

	/* AUDIO_OUT_ENABLE bits: enable AP, ACLK and WS if needed
	 *  5 - aclk_out     - 1=Audio clock port active
	 *  4 - ws_out       - 1=Word selection port active
	 *  3 - ap3_out      - 1=AP0 active
	 *  2 - ap2_out      - 1=AP1 active
	 *  1 - ap1_out      - 1=AP2 active
	 *  0 - ap0_out      - 1=AP3 active
	 */
	if (!sp_used_by_fifo) {
		reg = 0x0f;
	} else {
		/* TODO: ensure audio out DAI allows AP1,2,3? */
		reg = 0x01; /* AP0 always enabled */
		if (channel_assignment >= 0x01)
			reg |= 2; /* >1 also need AP1 */
		if (channel_assignment >= 0x04)
			reg |= 4; /* >4 also need AP2 */
		if (channel_assignment >= 0x0C)
			reg |= 8; /* >12 also need AP3 */
		/* specific cases where AP1 is not used */
		if ((channel_assignment == 0x04)
		 || (channel_assignment == 0x08)
		 || (channel_assignment == 0x0c)
		 || (channel_assignment == 0x10)
		 || (channel_assignment == 0x14)
		 || (channel_assignment == 0x18)
		 || (channel_assignment == 0x1c))
			reg &= ~2;
		/* specific cases where AP2 is not used */
		if ((channel_assignment >= 0x14)
		 && (channel_assignment <= 0x17))
			reg &= ~4;
	}
	if (ws_active)
		reg |= 0x30;
	io_write(REG_AUDIO_OUT_ENABLE, reg);

	/* reset test mode to normal audio freq auto selection */
	io_write(REG_TEST_MODE, 0x00);

	/* reset sw_ncts_en & ncts_en if i2s layout_0 */
	if (reg & 0x18) {
		reg = io_read(REG_TEST_NCTS_CTRL);
		reg &= ~0x03;
		io_write(REG_TEST_NCTS_CTRL, reg);
	}

	return 0;
}


/** Selects HDMI input
 * here as well.
 *
 * @param input
 * @returns 0 on success
 */
int
tda1997x_select_input(tda1997x_input_t input)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	u8 reg;

	DPRINTK(0,"%s: HDMI-%c\n", __func__, input?'B':'A');
	reg = io_read(REG_INPUT_SEL);
	/* keep loop mode for TDA19973 */
	if (tda1997x->chip == 19973)
		reg &= ~MASK_DIG_INPUT_VDPR_FMT;
	else
		reg &= ~(MASK_DIG_INPUT_VDPR_FMT | MASK_HDMIOUTMODE);
	reg |= 0x80; /* RESET_FTM - vdp reset */
	reg |= input;
	reg |= FORMAT_RESET;  /* Soft reset of format measurement timing */
	/* update INPUT_SEL */
	io_write(REG_INPUT_SEL, reg);

	return 0;
}
EXPORT_SYMBOL(tda1997x_select_input);


/** configure video output format (output pins and sync config)
 * @param pdata with video output configuration
 * @returns 0 on success
 */
static int
tda1997x_set_video_outputformat(struct tda1997x_platform_data *pdata)
{
	u8 reg;

	DPRINTK(0,"%s: %s\n", __func__, vidfmt_names[pdata->vidout_format]);

	/* Configure pixel clock generator:
	 *  - delay and polarity from platform data
	 *  - clock per output format
	 *
	 * CLKOUT_CTRL bits:
	 *   7 - unused
	 * 6:4 - clkout_del - Delay of clock signal on V_CLK pin
	 *   3 - unused
	 *   2 - clkout_tog - polarity of clock signal on V_CLK pin (1 - invert)
	 * 1:0 - clkout_sel: Selection of the clock signal output on CKP pin
	 *   00: Pix_clock (and selection for Chroma/Luma in case of CCIR-656, so
	 *                  clock for CCIR-656 DDR)
	 *   01: Pix_clock_x2 (for 422_CCIR)
	 *   10: Pix_clock_div2
	 *   11: Pix_clock_div4
	 */
	reg = pdata->vidout_delay_clk << 4;
	if (pdata->vidout_clkmode == CLOCK_SINGLE_EDGE) {  /* single edge */
		switch (pdata->vidout_format) {
			case VIDEOFMT_444:     /* RGB444/YUV444 */
			case VIDEOFMT_422_SMP: /* YUV422 semi-planar */
				break;
			case VIDEOFMT_422_CCIR:/* YUV422 CCIR656 */
				reg |= 0x01; /* clk_x2 */
				break;
		}
	} else {                               /* dual edge */
		switch (pdata->vidout_format) {
			case VIDEOFMT_444:     /* RGB444/YUV444 */
			case VIDEOFMT_422_SMP: /* YUV422 semi-planar */
				reg |= 0x02; /* clk_div2 */
				break;
			case VIDEOFMT_422_CCIR:/* YUV422 CCIR656 */
				break;
		}
	}
	io_write(REG_CLKOUT_CTRL, reg);

	/* Configure pre-filter:
	 *
	 * FILTERS_CTRL:
	 * 3:2 - Bu Filter control
	 *   00: Off
	 *   01: 2 Taps
	 *   10: 7 Taps
	 *   11: 2/7 Taps
	 * 1:0 - Rv Filter control
	 *   00: Off
	 *   01: 2 Taps
	 *   10: 7 Taps
	 *   11: 2/7 Taps
	 */
	reg = 0x00; /* filters off */
	/* 4:2:2 mode requires conversion */
	if ((pdata->vidout_format == VIDEOFMT_422_SMP)
	 || (pdata->vidout_format == VIDEOFMT_422_CCIR))
		reg = 0x0f; /* 27taps for Rv and Bu */
	io_write(REG_FILTERS_CTRL, reg);

	/* Enable Blanking code and timing ref (EAV/SAV) insertion */
	reg = VP_OUT | pdata->vidout_format;
	if (pdata->vidout_blc)
		reg |= VP_BLK;
	if (pdata->vidout_trc)
		reg |= VP_TRC;
	io_write(REG_OF_CTRL, reg);

	/* Configure bypass:
	 * VDP_CTRL bits:
	 *  5 - compdel_bp   - 1:bypass compdel
	 *  4 - formatter_bp - 1:bypass formatter
	 *  3 - ?
	 *  2 - ?
	 *  1 - prefilter_bp - 1:bypass prefilter
	 *  0 - mat_bp       - 1:bypass matrix conversion
	 */
	reg = io_read(REG_VDP_CTRL);
	/* bypass pre-filter if not needed (REG_FILTERS_CTRL == 0) */
	if ( (io_read(REG_FILTERS_CTRL) & 0x0f) == 0)
		reg |= (1<<1);  /* disable pre-filter */
	else
		reg &= ~(1<<1); /* enable pre-filter */
	/* bypass formatter if not needed:
	 * (444 mode and no insertion of timing/blanking codes */
	if ( (pdata->vidout_format == VIDEOFMT_444)
	  && !pdata->vidout_blc
	  && !pdata->vidout_trc
	) {
		reg |= (1<<4);  /* disable formatter */
	} else {
		/* enable formatter and compdel: needed for timing/blanking codes */
		reg &= ~((1<<4)|(1<<5));
	}
	/* activate compdel for small sync delays */
	if ((pdata->vidout_delay_vs < 4) || (pdata->vidout_delay_hs < 4)) {
		reg &= ~(1<<5);
	}
	io_write(REG_VDP_CTRL, reg);

	/* Configure DE output signal:
	 *  - delay, polarity, and source from platform data
	 *
	 * DE_FREF_SEL bits:
	 *  7:4 - de_del - DE from HDMI delay (Latency datapath + [-8..+7] pixels)
	 *    3 - de_pxq - DE pixel qualification (only when del_sel = 00)
	 *    0: Timing codes are not signaled by DE
	 *    1: Timing codes are signaled by DE
	 *    2 - de_pol - Polarity of signal output on DE/FREF pin
	 *    0: No specific action
	 *    1: Invert signal
	 *  1:0 - de_sel - Selection of signal output on DE/FREF pin
	 *    00: DE from VHREF [HREF and not(VREF)]
	 *    01: FREF from VHREF
	 *    10: FREF from HDMI
	 */
	io_write(REG_DE_FREF_SEL,
		(pdata->vidout_delay_de << 4) |
		(pdata->vidout_invert_de << 2) |
		(pdata->vidout_sel_de));

	/* HS_HREF_SEL bits:
	 *  7:4 - hs_del - HS from HDMI delay (Latency datapath + [-8..+7] pixels)
	 *    3 - href_pxq - HREF pixel qualifcation:
	 *    0 - Timing codes are not signaled by HREF
	 *    1 - Timing codes are signaled by HREF
	 *    2 - hsync_pol: Polarity of signal output on HS/HREF pin
	 *    0 - No specific action
	 *    1 - Invert signal
	 *  1:0 - hsync_sel - Selection of signal output on HS/HREF pin
	 *    00: HS from VHREF (Select HS signal output from internal sync generator
	 *        on HS_HREF hardware pin)
	 *    01: HREF from VHREF (Select HREF signal output from internal sync
	 *        generator on HS_HREF hardware pin)
	 *    10: HREF from HDMI (Select HREF signal output from HDMI input sync
	 *        processor on HS_HREF hardware pin)
	 */
	io_write(REG_HS_HREF_SEL,
		(pdata->vidout_delay_hs << 4) |
		(pdata->vidout_invert_hs << 2) |
		(pdata->vidout_sel_hs));

	/* VS_VREF_SEL bits:
	 *  7:4 - vs_del - VS from HDMI delay (Latency datapath + [-8..+7] pixels)
	 *    3 - unused
	 *    2 - vsync_pol: Polarity of signal output on VS/VREF pin
	 *    0 - No specific action
	 *    1 - Invert signal
	 *  1:0 - vsync_sel - Selection of signal output on VS/VREF pin
	 *    00: VS from VHREF (Select VS signal output from internal sync generator
	 *        on HS_VREF hardware pin) (BSLHDMIRX_SYNCOUTPUT_VSYNC_VHREF)
	 *    01: VREF from VHREF (Select VREF signal output from internal sync
	 *        generator on HS_VREF hardware pin) (BSLHDMIRX_SYNCOUTPUT_VREF_VHREF)
	 *    10: VREF from HDMI (Select HREF signal output from HDMI input sync
	 *        processor on HS_VREF hardware pin) (BSLHDMIRX_SYNCOUTPUT_VSYNC_HDMI)
	 */
	io_write(REG_VS_VREF_SEL,
		(pdata->vidout_delay_vs << 4) |
		(pdata->vidout_invert_vs << 2) |
		(pdata->vidout_sel_vs));

	return 0;
}


/** Soft Reset of specific hdmi info
 * @param info_rst - reset to apply to HDMI_INFO_RST
 * @param bresetSus - reset start-up sequencer
 * @returns 0 on success
 */
static int
tda1997x_hdmi_info_reset(u8 info_rst, bool bresetSus)
{
	u8 reg;

	DPRINTK(0,"%s: 0x%02x %s\n", __func__, info_rst, bresetSus?"RESET_SUS":"");

	reg = io_read(REG_HDMI_INFO_RST);
	io_write(REG_HDMI_INFO_RST, info_rst);

	/* if IF has been reset, clear INT_FLG_MODE as all ITs are raided by the
	 * reset IF */
	if (reg & RESET_IF) {
		reg = io_read(REG_INT_FLG_CLR_MODE);
		io_write(REG_INT_FLG_CLR_MODE, reg);
	}	

	/* Write SUS_RESET register (bit hdcp_dcc_man does not exist on TDA19972) */
	if (!bresetSus) {
		reg = io_read(REG_RATE_CTRL);
		reg |= RATE_REFTIM_ENABLE;
		reg = io_write(REG_RATE_CTRL, reg);
	} else {
		reg = io_read(REG_RATE_CTRL);
		reg &= ~RATE_REFTIM_ENABLE;
		reg = io_write(REG_RATE_CTRL, reg);
	}

	return 0;
}

/* Configure HDCP:  set basic configuration for HDCP module:
 *   enable/disable, key encryption, DDC address, key description seed
 *
 * @param decrypt_keys - Key internal decryption (on/off)
 * @param hdcp_enable - Enable/disable HDCP function
 * @param i2c_addr - Display data channel I2C slave address
 * @param key_decryption_seed - Key decryption Seed
 * @returns 0 on success
 */
static int
tda1997x_configure_hdcp(struct tda1997x_data *tda1997x,
	hdcp_key_t decrypt_keys,
	enable_t hdcp_enable,
	u8 ddc_i2c_addr,
	u16 key_decryption_seed)
{
	u8 reg;
	u8 regs[3];

	DPRINTK(0,"%s: enable=%d ddc=0x%02x seed=0x%04x\n",
		__func__, hdcp_enable, ddc_i2c_addr, key_decryption_seed);

	/* HDCP control */
	regs[0] = decrypt_keys | hdcp_enable;
	/* keys description seed MSB */
	regs[1] = key_decryption_seed >> 8;
	/* keys description seed LSB */
	regs[2] = key_decryption_seed & 0x00ff;

	if (tda1997x->chip_revision == 0) {
		/* enable clock on TMDS PLL by using FRO */
		io_write(REG_CLK_CFG, 0x03);
		io_write(REG_PON_CBIAS, 0x01);
		io_write(REG_PON_PLL, 0x01);
		io_write(REG_PON_OVR_EN, 0x01);
	}

	/* write HDCP_CTRL regs */
	io_writen(REG_HDCP_CTRL, 3, regs);

	/* write i2c addr */
	io_write(REG_HDCP_DDC_ADDR, ddc_i2c_addr);

	if (tda1997x->chip_revision == 0) {
		/* disable clock on TMDS PLL by using FRO */
		io_write(REG_CLK_CFG, 0x00);
		io_write(REG_PON_OVR_EN, 0x00);

		/* Restore clock */
		io_write(REG_PON_CBIAS, 0x00);
		io_write(REG_CGU_DEBUG_SEL, 0x08);

		/* Clear HDMI mode flag in BCAPS (for N1) */
		io_write(REG_CLK_CFG, 0x03);
		io_write(REG_PON_OVR_EN, 0x01);
		io_write(REG_PON_CBIAS, 0x01);
		io_write(REG_PON_PLL, 0x01);
		reg = io_read(REG_MODE_RECOVER_CFG1);
		reg &= ~0x06;
		reg |= 0x02;
		io_write(REG_MODE_RECOVER_CFG1, reg);
		io_write(REG_CLK_CFG, 0x00);
		io_write(REG_PON_OVR_EN, 0x00);
		reg = io_read(REG_MODE_RECOVER_CFG1);
		reg &= ~0x06;
		io_write(REG_MODE_RECOVER_CFG1, reg);
	}

	/* clear HDCP interrupt status bits that may have been raised during this
	 * process
	 */
	io_write(REG_INT_FLG_CLR_HDCP, 0x07);

	return 0;
}

/* Configure HDCP MTP
 * @param cmd - command to be sent (download or read)
 * @returns 0 on success
 */
static int
tda1997x_configure_mtp(struct tda1997x_data *tda1997x, mtp_command_t cmd)
{
	u8 reg;
	unsigned int timeout;

	switch (cmd) {
		case MTP_START_DOWNLOAD:
			DPRINTK(0,"%s: MTP_START_DOWNLOAD\n", __func__);
			if (tda1997x->chip_revision == 0) {
				/* disable termination and enable HDCP block */
				io_write(REG_RT_MAN_CTRL, 0x00);
				io_write(REG_MAN_SUS_HDMI_SEL, MAN_RST_HDCP | MAN_DIS_HDCP);

				/* enable clock on TMDS PLL by using FRO */
				io_write(REG_CLK_CFG, 0x03);
				io_write(REG_PON_CBIAS, 0x01);
				io_write(REG_PON_PLL, 0x01);
				io_write(REG_PON_OVR_EN, 0x01);
				io_write(REG_CGU_DEBUG_SEL, 0x00);

				/* reset HDCP block */
				io_write(REG_MAN_SUS_HDMI_SEL, MAN_RST_HDCP | MAN_DIS_HDCP);
				io_write(REG_MAN_HDMI_SET, 0x04);
				io_write(REG_MAN_HDMI_SET, 0x00);

				/* remove force */
				io_write(REG_PON_OVR_EN, 0x00);
				io_write(REG_CLK_CFG, 0x03);

				/* copy byte KEY_0(39) (0x4002) into private_area (0x425f) */
				reg = io_read(0x4002 /*REG_MTP_KEY39_LSB*/);
				io_write(0x425f /*REG_MTP_PRIVATE_AREA*/, reg);
			}
		
			/* Enable HDCP */
			io_write(REG_HDMI_INFO_RST, 0x00);
			io_write(REG_HDCP_CTRL, 0x03);

			/* clear flag hdcp_dlmtp: 0x0015:4 */
			io_write(REG_INT_FLG_CLR_HDCP, 0x18);

			/* download key into HDCP engine */
			io_write(REG_HDCP_KEY_CTRL, 0x01);

			/* check flag hdcp_dlram: 0x0015:3 */
			timeout = jiffies + msecs_to_jiffies(100);
			do {
				reg = io_read(REG_INT_FLG_CLR_HDCP);
			} while ( (jiffies < timeout) & ((reg & 0x08) != 0x08));

			/* download MTP into SRAM: hmtp_dl_all (0x137a) */
			io_write(REG_HMTP_CTRL, 0x01);

			/* check flag hdcp_dlmtp: 0x0015:4 */
			timeout = jiffies + msecs_to_jiffies(100);
			do {
				reg = io_read(REG_INT_FLG_CLR_HDCP);
			} while ( (jiffies < timeout) & ((reg & 0x10) != 0x10));

			/* clear HDCP interrupt status bits that may have been raised */
			io_write(REG_INT_FLG_CLR_HDCP, 0x07);
			break;

		case MTP_START_READ:
			DPRINTK(0,"%s: MTP_START_READ\n", __func__);
			/* clear flag hdcp_dlmtp */
			io_write(REG_INT_FLG_CLR_HDCP, 0x10);

			/* Download MTP into SRAM: hmtp_dl_all */
			io_write(REG_HMTP_CTRL, 0x01);

			/* check flag hdcp_dlmtp */
			timeout = jiffies + msecs_to_jiffies(100);
			do {
				reg = io_read(REG_INT_FLG_CLR_HDCP);
			} while ( (jiffies < timeout) & ((reg & 0x10) != 0x10));
			break;
	}

	return 0;
}


/** set power mode
 */
static void
tda1997x_power_mode(struct tda1997x_data *tda1997x, bool bEnable)
{
	u8 reg;

	dev_dbg(&tda1997x->client->dev, "%s %s\n", __func__, bEnable?"on":"off");
	DPRINTK(0,"%s %s\n", __func__, bEnable?"on":"off");

	if (bEnable) {
		/* Power on sequence */

#if 0
		/* Disable low power mode */
		reg = io_read(REG_EDID_POWER);
		reg &= ~MASK_LOW_PW_EDID; 		
		io_write(REG_EDID_POWER, reg);
#endif

		/* Automatic control of TMDS */
		io_write(REG_PON_OVR_EN, TMDS_AUTO_PON);

		/* Enable current bias unit */
		io_write(REG_CFG1, CBIAS_PON);

#if 0
		/* Enable TMDS clock in the digital equalizer */
		reg = io_read(REG_SUS_RESET);
		reg &= ~MASK_TMDS_CLK_DIS; 		
		io_write(REG_SUS_RESET, reg);

		/* Enable Xtal Osc */
		reg = io_read(REG_XOSC_CFG);
		reg &= ~MASK_XTAL_OSC_PD; 		
		io_write(REG_XOSC_CFG, reg);
#endif

		/* Enable deep color PLL */
		io_write(REG_DEEP_PLL7, DC_PLL_PON);

#if 0
		/* Enable audio PLL */
		reg = io_read(REG_CLOCKS_MODE);
		reg &= ~MASK_AUDIO_PLL_PD; 		
		io_write(REG_CLOCKS_MODE, reg);
#endif

		/* Output buffers active */
		reg = io_read(REG_OF_CTRL);
		reg &= ~MASK_OF_CTRL_OUT_HIZ; 		
		io_write(REG_OF_CTRL, reg);

	} else {
		/* Power down EDID mode sequence */

		/* Output buffers in HiZ */
		reg = io_read(REG_OF_CTRL);
		reg |= MASK_OF_CTRL_OUT_HIZ; 		
		io_write(REG_OF_CTRL, reg);

#if 0
		/* Disable audio PLL */
		reg = io_read(REG_CLOCKS_MODE);
		reg |= MASK_AUDIO_PLL_PD; 		
		io_write(REG_CLOCKS_MODE, reg);
#endif

		/* Disable deep color PLL */
		io_write(REG_DEEP_PLL7, DC_PLL_PD);

#if 0
		/* Disable Xtal Osc */
		reg = io_read(REG_XOSC_CFG);
		reg |= MASK_XTAL_OSC_PD; 		
		io_write(REG_XOSC_CFG, reg);

		/* Disable TMDS clock in the digital equalizer */
		reg = io_read(REG_SUS_RESET);
		reg |= MASK_TMDS_CLK_DIS; 		
		io_write(REG_SUS_RESET, reg);
#endif

		/* Disable current bias unit */
		io_write(REG_CFG1, CBIAS_POFF);

		/* Manual control of TMDS */
		io_write(REG_PON_OVR_EN, TMDS_MAN_PON);

#if 0
		/* Enable low power mode */
		reg = io_read(REG_EDID_POWER);
		reg |= MASK_LOW_PW_EDID; 		
		io_write(REG_EDID_POWER, reg);
#endif
	}
	
}


/* check the audio samplerate for change
 * @returns 0 on success
 */
static int
tda1997x_get_audio_frequency(struct tda1997x_data *tda1997x)
{
	u8 reg;
	long freq = 0;

	reg = io_read(REG_AUDIO_FREQ);
	switch(reg & MASK_AUDIO_FREQ) {
		case 0x00: break;
		case 0x01: freq= 32000; break;
		case 0x02: freq= 44100; break;
		case 0x03: freq= 48000; break;
		case 0x04: freq= 88200; break;
		case 0x05: freq= 96000; break;
		case 0x06: freq=176400; break;
		case 0x07: freq=192000; break;
	}

	DPRINTK(0, "REG_AUDIO_FREQ=0x%02x: %ldHz\n", reg, freq);
	tda1997x->audio_mode.samplerate = freq;

	return 0;
}

/** detect an appropriate video mode from:
 * video input verticalPeriod, horizontalPeriod, and hsWidth
 * @returns resolution_data *
 */
static const resolution_data_t *
tda1997x_detect_resolution(struct tda1997x_data *tda1997x)
{
	u32 verticalPeriod;
	u16 horizontalPeriod;
	u16 hsWidth;
	char vPerCmp = 0, hPerCmp = 0, hsWidthCmp = 0;
	const resolution_data_t *res;
	int i;

	/* Read the FMT registers */
	verticalPeriod = io_read24(REG_V_PER) & MASK_VPER;
	horizontalPeriod = io_read16(REG_H_PER) & MASK_HPER;
	hsWidth = io_read16(REG_HS_WIDTH) & MASK_HSWIDTH;
	DPRINTK(0,"verticalPeriod=%d horizontalPeriod=%d hsWidth=%d\n",
		verticalPeriod, horizontalPeriod, hsWidth);

#if 1 // more details
{
	videoFormatDetails fmt;
	videoFormatDetails *pFMT = &fmt;
	
	/* read the FMT registers */
	pFMT->vsPolarity = io_read(REG_V_PER) & 0x80;
	pFMT->hsPolarity = io_read(REG_H_PER) & 0x80;
	pFMT->videoFormat = io_read(REG_HS_WIDTH) & 0x80;
	pFMT->horizontalTotalPeriod = io_read16(REG_FMT_H_TOT)&0x3fff;
	pFMT->horizontalVideoActiveWidth = io_read16(REG_FMT_H_ACT)&0x3fff;
	pFMT->horizontalFrontPorchWidth = io_read16(REG_FMT_H_FRONT)&0x3fff;
	pFMT->horizontalSyncWidthPixClk = io_read16(REG_FMT_H_SYNC)&0x3fff;
	pFMT->horizontalBackPorchWidth = io_read16(REG_FMT_H_BACK)&0x3fff;
	pFMT->verticalTotalPeriod = io_read16(REG_FMT_V_TOT)&0x3fff;
	pFMT->verticalVideoActiveWidth = io_read16(REG_FMT_V_ACT)&0x3fff;
	pFMT->verticalFrontPorchWidthF1 = io_read(REG_FMT_V_FRONT_F1);
	pFMT->verticalFrontPorchWidthF2 = io_read(REG_FMT_V_FRONT_F2);
	pFMT->verticalSyncWidth = io_read(REG_FMT_V_SYNC);
	pFMT->verticalBackPorchWidthF1 = io_read(REG_FMT_V_BACK_F1);
	pFMT->verticalBackPorchWidthF2 = io_read(REG_FMT_V_BACK_F2);
	pFMT->dataEnablePresent = io_read(REG_FMT_DE_ACT)&0x01;

	DPRINTK(2,"vsPolarity=%d hsPolarity=%d videoFormat=%d\n"
		"horizTotalPeriod=%d horizVideoActiveWidth=%d horizFrontPorchWidth=%d\n"
		"horizSyncWidthPixClk=%d horizBackPorchWidth=%d\n"
		"vertTotalPeriod=%d vertVideoActiveWidth=%d vertFrontPorchWidthF1=%d"
		"vertFrontPorchWidthF2=%d\n"
		"vertBackPorchWidthF1=%d vertBackPorchWidthF1=%d dataEnablePresent=%d\n",
		pFMT->vsPolarity, pFMT->hsPolarity, pFMT->videoFormat,
		pFMT->horizontalTotalPeriod, pFMT->horizontalVideoActiveWidth,
		pFMT->horizontalFrontPorchWidth, pFMT->horizontalSyncWidthPixClk,
		pFMT->horizontalBackPorchWidth,
		pFMT->verticalTotalPeriod, pFMT->verticalVideoActiveWidth,
		pFMT->verticalFrontPorchWidthF1, pFMT->verticalFrontPorchWidthF2,
		pFMT->verticalBackPorchWidthF1,
		pFMT->verticalBackPorchWidthF2,
		pFMT->dataEnablePresent);
}
#endif

	/* iterate over list of supported resolutions and find best match */
	for (i = 0; i < ARRAY_SIZE(supported_res); i++) {
		res = &supported_res[i];

		vPerCmp = (char) ((verticalPeriod >= res->verticalPeriodMin) &&
						(verticalPeriod <= res->verticalPeriodMax));
		hPerCmp = (char) ((horizontalPeriod >= res->horizontalPeriodMin) &&
						(horizontalPeriod <= res->horizontalPeriodMax));
		hsWidthCmp = (char) ((hsWidth >= res->hsWidthMin) &&
						(hsWidth <= res->hsWidthMax));

		DPRINTK(1,"\t%02d: %dx%d@%d%c: %d/%d/%d\n", i, res->width, res->height,
			res->horizfreq, res->interlaced?'i':'p',
			vPerCmp, hPerCmp, hsWidthCmp);
		if (vPerCmp && hPerCmp && hsWidthCmp) {
			int pixrate;

			/* resolutiontype used to determine Default Colorimetry */
			switch (res->height) {
				case 480:
				case 576:
				case 240:
				case 288:
					tda1997x->resolutiontype = RESTYPE_SDTV;
					break;
				case 720:
				case 1080:
					tda1997x->resolutiontype = RESTYPE_HDTV;
					break;
				default:
					tda1997x->resolutiontype = RESTYPE_PC;
			}

			printk(KERN_INFO "%s: matched resolution: %dx%d%c@%d %s\n",
				KBUILD_MODNAME, res->width, res->height,
				res->interlaced?'i':'p',
				res->horizfreq,
				restype_names[tda1997x->resolutiontype]);

			/* validate input mode */
			pixrate = (res->width * res->height * res->horizfreq) /
				  1000000;
			if (res->interlaced)
				pixrate /= 2;
			switch(tda1997x->pdata->vidout_format) {
			case VIDEOFMT_444:
			case VIDEOFMT_422_SMP:
				/* FIXME: have not figured out how to get HS output asserted on 2nd field */ 
				if (res->interlaced) {
					printk(KERN_INFO "%s: Error %s: interlaced not supported\n", KBUILD_MODNAME,
					       vidfmt_names[tda1997x->pdata->vidout_format]);
					return NULL;
				}
				break;
			case VIDEOFMT_422_CCIR:
				/* BT656 requires 2-clocks per pixel */
				pixrate *= 2;
				break;
			}
			if (tda1997x->pdata->max_pixel_rate &&
			    (pixrate > tda1997x->pdata->max_pixel_rate))
			{
				printk(KERN_INFO "%s: Error: %dMP/s exceeds max of %dMP/s\n", KBUILD_MODNAME,
				       pixrate,
				       tda1997x->pdata->max_pixel_rate);
				return NULL;
			}

			return res;
		}
	}

	printk(KERN_ERR "%s: found no video resolution match for: %d/%d/%d\n",
		KBUILD_MODNAME, verticalPeriod, horizontalPeriod, hsWidth);
	return NULL;
}

/** read input activity registers
 *
 * @returns activity_sate bitmask:
 *   7: unused
 *   6: unused
 *   5: unused
 *   4: tmds_locked
 *   3: inputD_clock_stable (unsupported)
 *   2: inputC_clock_stable (unsupported)
 *   1: inputB_clock_stable
 *   0: inputA_clock_stable
 */
static u8
tda1997x_read_activity_status_regs(void)
{
	u8 reg, status = 0;

 /* Activity detection must only be notified when stable_clk_x AND active_x
  * bits are set to 1.  If only stable_clk_x bit is set to 1 but not
  * active_x, it means that the TMDS clock is not in the defined range,
  * so activity detection must not be notified
  * => regStatus must be set to 0 in that case.  If stable_clk_x bit is
  * set to 0, regStatus must also be set to 0
  */

	/* Read CLK_A_STATUS register */
	reg = io_read(REG_CLK_A_STATUS);
	/* when stable_clk_x is set to 1, check active_x bit */
	if ((reg & MASK_CLK_STABLE) && !(reg & MASK_CLK_ACTIVE) )
		reg &= ~MASK_CLK_STABLE;
	status |= ((reg & MASK_CLK_STABLE) >> 2);

	/* Read CLK_B_STATUS register */
	reg = io_read(REG_CLK_B_STATUS);
	/* when stable_clk_x is set to 1, check active_x bit */
	if ((reg & MASK_CLK_STABLE) && !(reg & MASK_CLK_ACTIVE) )
		reg &= ~MASK_CLK_STABLE;
	status |= ((reg & MASK_CLK_STABLE) >> 1);

	/* Read the SUS_STATUS register */
	reg = io_read(REG_SUS_STATUS);

	/* If state = 5 => TMDS is locked */
	if ( (reg & MASK_SUS_STATE_VALUE) == LAST_STATE_REACHED)
		status |= MASK_SUS_STATE_BIT;
	else
		status &= ~MASK_SUS_STATE_BIT;

	return status;
}


/** parse an infoframe and do some sanity checks on it
 *
 * @type of inforframe
 * @returns 0 on success
 */
static unsigned int
tda1997x_parse_infoframe(struct tda1997x_data *tda1997x, int type)
{
	u8 d[MAX_IF_DATA];
	u8 crc;
	int len = 0;
	int i;

	/* determine length based on type */
	switch(type) {
		case MPS_IF_TYPE: len = MPS_IF_NB_DATA; break;
		case AUD_IF_TYPE: len = AUD_IF_NB_DATA; break;
		case SPD_IF_TYPE: len = SPD_IF_NB_DATA; break;
		case AVI_IF_TYPE: len = AVI_IF_NB_DATA; break;
		case VS_HDMI_IF_TYPE:
		case VS_BK1_IF_TYPE:
		case VS_BK2_IF_TYPE:
			len = VS_IF_NB;
			break;
	}

	/* read data */
	if (io_readn(type, len, d) != len) {
		printk(KERN_ERR "infoframe type0x%02x failed read\n", type);
		return -1;
	}

	/* verify crc */
	for (i = 0, crc = 0; i < len; i++)
		crc += d[i];
	if (crc) {
		printk(KERN_ERR "infoframe type0x%02x failed CRC\n", type);
		return -2;
	}

	/* parse details */
	switch(type) {
		case MPS_IF_TYPE:
			/* parse infoframe to get bitrate, fieldrepeat, MPEG_Frame */
			DPRINTK(0,"\t\tbitrate=%d,%d,%d,%d fieldRepeat=%d MPEG_Frame=%d\n",
				d[4], d[5], d[6], d[7], d[8] & 0x10, d[8] & 0x03);
			break;

		/* Audio InfoFrame: see HDMI spec 8.2.2 */
		case AUD_IF_TYPE:
			/*
			 * CC0..CC2 - Channel Count. See CEA-861-D table 17
			 * CT0..CT3 - Coding Type. The CT bits shall always be 0 (use Stream Header)
			 * SS0..SS1 - Sample Size. The SS bits shall always be 0 (use Stream Header)
			 * SF0..SF2 - Sample Freq. See CEA-861-D table 18.
			 *            - For L-PCM and IEC 61937 compressed audio streams the SF bits
			 *              shall always be 0 (use Stream Header).
			 *            - For One Bit Audio and DST streams, the SF bits shall equal
			 *              the ACR fs value.
			 *            - For Super Audio CD, the SF bits are typically 0, 1, 0
			 *              indicating a sample freq of 2.8224MSamples/s (64*44.1kHz)
			 * CA0..CA7 - Channel/Speaker Allocation. See CEA-861-D Section 6.6.2
			 *            - this is not valid for IEC 61937 compressed audio streams
			 * LSV0..LSV3 - Level Shif Value (for downmixing). See CEA-861-D 6.6.2
			 *              and CEA-861-D table 21
			 * DM_INH     - Downmix inibit.  See CEA-861-D sec 6.6.2 and table 22
			 *              The DM_INH field is to be set only for DVD-Audio
			 *
			 * CT, SS, and SF values of 0 indicate that these items are carried in the
			 * audio stream itself.
			 */
			DPRINTK(0,"\t\tcodingType=%d channelCount=%d samplefrequency=%d "
				"samplesize=%d dataByte3=%d channelAllocation=%d downmixInhibit=%d "
				"levelShiftValue=%d\n",
				(d[4] & 0xf0) >> 4,  /* CT3, CT2, CT1, CT0 */
				(d[4] & 0x07),       /* CC2, CC1, CC0 */
				(d[5] & 0x1c) >> 2,  /* SF2, SF1, SF0 */
				(d[5] & 0x03),       /* SS1, SS0 */
				d[6],
				d[7],                /* CA7 .. CA0 */
				(d[8] & 0x80) >> 7,  /* DM_INH */
				(d[8] & 0x78) >> 3); /* LSV3, LSV2, LSV1, LSV0 */

			/* Channel Count */
			tda1997x->source_channels = (d[4] & 0x07) + 1;
			DPRINTK(0, "Audio Channels: %d\n",
				tda1997x->source_channels);

			/* Channel Assignment */
			if ( (d[7] <= 0x1f)    /* MAX_CHANNEL_ALLOC */
				&& (d[7] != tda1997x->channel_assignment)
			  && !tda1997x->pdata->audio_force_channel_assignment)
			{
				/* use the channel assignment from the audio infoframe */
				DPRINTK(0, "channel assignment changed: %d\n", d[7]);
				tda1997x->channel_assignment = d[7];

				/* configure audio output */
				tda1997x_configure_audio_formatter(tda1997x->pdata,
					tda1997x->channel_assignment);

				/* reset the audio FIFO */
				tda1997x_hdmi_info_reset(RESET_AUDIO, 0);
				//tda1997x_hdmi_info_reset(0x00, 0);
			}
			break;

		/* Source Product Descriptor information (SPD) */
		case SPD_IF_TYPE:
			for (i = 0; i < 8; i++)
				tda1997x->vendor[i] = d[4 + i];
			for (i = 0; i < 16; i++)
				tda1997x->product[i] = d[4 + 8 + i];
			printk(KERN_INFO "%s: Source Product Descriptor: %s %s\n", KBUILD_MODNAME,
				tda1997x->vendor, tda1997x->product);
			break;


		/* Auxiliary Video information (AVI) InfoFrame: see HDMI spec 8.2.1 */
		case AVI_IF_TYPE: {
			u8 pixel_repetitionfactor;
			u8 reg;
	
			if (d[0] != 0x82) {
				printk(KERN_ERR "INFOFRAME AVI: wrong packet type! (0x%02x)\n", d[0]);
			}
			DPRINTK(0,"\t\tcolorIndicator=%d activeInfoPresent=%d "
				"barInfomationDataValid=%d scanInformation=%d colorimetry=%d "
				"pictureAspectRatio=%d activeFormatAspectRation=%d "
				"nonUinformPictureScaling=%d videoFormatIdentificationCode=%d "
				"pixelRepetitionFactor=%d\n",
				(d[4] & 0x60) >> 5, /* colorspace: Y1, Y0 */
				(d[4] & 0x10) >> 4, /* activeInfoPresent: A0 */
				(d[4] & 0x0c) >> 2, /* barInformationValid: B1, B0 */
				(d[4] & 0x03),	    /* scanInformation: S1, S0 */
				(d[5] & 0xc0) >> 6, /* coloriemtry: C1, C0 */
				(d[5] & 0x30) >> 4, /* pictureAspectRatio: M1, M0 */
				(d[5] & 0x0f),	    /* activeFormatAspectRatio: R3, R2, R1, R0 */
				(d[6] & 0x03),	    /* nonUniformPictureScaling: SC1, SC0 */
				(d[7] & 0x7f),	    /* videoFormatID: VIC6, VIC5, VIC4 */
				(d[8] & 0x0f)	      /* pixelRepetitionFactor: PR3, PR2, PR1, PR0 */
			);
			tda1997x->colorspace = (d[4] & 0x60) >> 5;  /* Y1, Y0 */
			tda1997x->colorimetry = (d[5] & 0xc0) >> 6; /* C1, C0 */
			pixel_repetitionfactor = d[8] & 0x0f;       /* PR3, PR2, PR1, PR0 */
			/* If colorimetry not specified, conversion depends on resolutiontype:
			 *  - SDTV: ITU601 for SD (480/576/240/288 line resolution)
			 *  - HDTV: ITU709 for HD (720/1080 line resolution)
			 *  -   PC: sRGB
			 * see HDMI specification section 6.7
			 */
			if ( (tda1997x->colorspace == COLORSPACE_YCBCR_422 ||
			      tda1997x->colorspace == COLORSPACE_YCBCR_444) &&
			     (tda1997x->colorimetry == COLORIMETRY_XVYCC ||
			      tda1997x->colorimetry == COLORIMETRY_NONE) )
			{

				if (tda1997x->resolutiontype == RESTYPE_HDTV)
					tda1997x->colorimetry = COLORIMETRY_ITU709;
				else if (tda1997x->resolutiontype == RESTYPE_SDTV)
					tda1997x->colorimetry = COLORIMETRY_ITU601;
				else
					tda1997x->colorimetry = COLORIMETRY_NONE;
				dev_info(&tda1997x->client->dev,
					"invalid/undefined colorimetry defaulted to %s (%s)\n",
					colorimetry_names[tda1997x->colorimetry],
					restype_names[tda1997x->resolutiontype]);
			}

			/* configure upsampler per sample format */
			/* ConfigureUpDownSampler: 0=bypass 1=repeatchroma 2=interpolate */
			reg = io_read(REG_PIX_REPEAT);
			reg = (reg & ~0x30 /* MASK_UP_SEL */);
			if (tda1997x->colorspace == COLORSPACE_YCBCR_422)
 				reg |= (1 << 4); /* repeatchroma */
			io_write(REG_PIX_REPEAT, reg);
			
			/* ConfigurePixelRepeater - repeat n-times each pixel */
			reg = io_read(REG_PIX_REPEAT);
			reg = (reg & ~0x0f /*MASK_PIX_REP*/) | pixel_repetitionfactor;
			io_write(REG_PIX_REPEAT, reg);

			/* configure the receiver with the new colorspace */
			tda1997x_configure_conversion(tda1997x,
				tda1997x->colorspace,
				tda1997x->colorimetry,
				tda1997x->pdata->vidout_format);

		}	break;

		case VS_HDMI_IF_TYPE:
		case VS_BK1_IF_TYPE:
		case VS_BK2_IF_TYPE:
			/* read update flag and store at the end */
			d[VS_IF_NB] = io_read(type);
			if (type == VS_HDMI_IF_TYPE && d[VS_IF_NB] > 3) /* HDMI_INFO_EXCEED */
			{
				DPRINTK(0,"HDMI_INFO_EXCEED\n");
				return -3;
			}
			DPRINTK(0, "\t\tieee_id[0]=%d ieee_id[1]=%d ieee_id[2]=%d\n",d[0],d[1],d[2]);
			break;
	}

	return 0;
}


/* tda1997x_work - deferred work procedure for handling interrupt
 */
static void tda1997x_work(struct work_struct *work)
{
	struct tda1997x_data *tda1997x = &tda1997x_data;
	u8 reg, interrupt_top_flags, source;
	
	do {
		/* read interrupt flags */
		interrupt_top_flags = io_read(REG_INT_FLG_CLR_TOP);
		if (interrupt_top_flags == 0)
			break;
		DPRINTK(0,"interrupt:0x%02x\n", interrupt_top_flags);

		/* SUS interrupt source (Input activity events) */
		if (interrupt_top_flags & INTERRUPT_SUS) {
			source  = io_read(REG_INT_FLG_CLR_SUS);
			io_write(REG_INT_FLG_CLR_SUS, source);
			DPRINTK(0,"SUS: 0x%02x\n", source);

			if (source & MASK_MPT_BIT) {
				DPRINTK(0,"\tConfig MTP end of process\n");

				/* reset MTP in use flag if set */
				if (tda1997x->mptrw_in_progress)
					tda1997x->mptrw_in_progress = 0;
			}

			if (source & MASK_SUS_END_BIT) {
				/* reset audio FIFO */
				reg = io_read(REG_HDMI_INFO_RST);
				reg |= MASK_SR_FIFO_FIFO_CTRL;
				io_write(REG_HDMI_INFO_RST, reg);
				reg &= ~MASK_SR_FIFO_FIFO_CTRL;
				io_write(REG_HDMI_INFO_RST, reg);

				DPRINTK(0,"\tRESET AUDIO SUS_END\n");

				/* reset HDMI flags memory and vsi_received flag */
				tda1997x->hdmi_status = 0;
				tda1997x->vsi_received = 0;
			}

			/* filter FMT interrupt based on SUS state */
			reg = io_read(REG_SUS_STATUS);
			if ( ((reg & MASK_SUS_STATE_VALUE) != LAST_STATE_REACHED)
			 || (source & MASK_MPT_BIT))
			{
				DPRINTK(0,"sus_state_value=0x%02x filter video fmt changed\n",
					reg & MASK_SUS_STATE_VALUE);
				source &= ~MASK_FMT_BIT;
			}

			if (source & (MASK_FMT_BIT | MASK_SUS_END_BIT)) {
				const resolution_data_t *res;
				DPRINTK(0, "\tHDMI LOCKED\n");

				reg = io_read(REG_SUS_STATUS);
				if ((reg & MASK_SUS_STATE_VALUE) != LAST_STATE_REACHED) {
					printk(KERN_ERR "%s: BAD SUS STATUS\n", KBUILD_MODNAME);
					continue;
				}

				/* There is a new activity, the status for HDCP repeater state */
				tda1997x->state_c5_reached = 0;

				/* Detect the new resolution */
				res = tda1997x_detect_resolution(tda1997x);
				if (res) {
					tda1997x->video_mode.width = res->width;
					tda1997x->video_mode.height = res->height;
					tda1997x->video_mode.fps = res->horizfreq;
					tda1997x->video_mode.interlaced = res->interlaced;
					tda1997x->video_mode.signal = 1;

					/* configure the active input to the given resolution */
					tda1997x_configure_input_resolution(res->resolutionID);

				} else {
					tda1997x->video_mode.width = 0;
					tda1997x->video_mode.height = 0;
					tda1997x->video_mode.fps = 0;
					tda1997x->video_mode.interlaced = 0;
					tda1997x->video_mode.signal = 1;
				}

				/* on 'input locked' event, RGB colorspace is forced (the AVI infoframe
				 * is not received yet at this moment)
				 * if AVI infoframe is received later, the colorspace will be
				 * reconfigured in the AVI infoframe handler
				 */
				tda1997x->colorspace = COLORSPACE_RGB;
				tda1997x->colorimetry = COLORIMETRY_NONE;
				/* bypass colorspace conversion */
				io_write(REG_VDP_CTRL, io_read(REG_VDP_CTRL) | (1<<0));
				/* SetBlankingCodes */
				io_write16(REG_BLK_GY, RGBBlankingCode.blankingCodeGy);
				io_write16(REG_BLK_BU, RGBBlankingCode.blankingCodeBu);
				io_write16(REG_BLK_RV, RGBBlankingCode.blankingCodeRv);

				/* set the state machine */
				tda1997x->state = STATE_LOCKED;
			}

			if (source & MASK_RT_PULSE_BIT) {
				DPRINTK(0,"\tEnd of termination resistance pulse\n");
			}
			if (source & MASK_SUS_ACT_BIT) {
				DPRINTK(0,"\tActivity of selected input changed\n");
			}
			if (source & MASK_SUS_CH_BIT) {
				DPRINTK(0,"\tSelected input changed\n");
			}
			if (source & MASK_SUS_ST_BIT) {
				DPRINTK(0,"\tSUS state changed\n");
			}
		}
		
		/* DDC interrupt source (Display Data Channel) */
		else if (interrupt_top_flags & INTERRUPT_DDC ) {
			source = io_read(REG_INT_FLG_CLR_DDC );
			io_write(REG_INT_FLG_CLR_DDC, source);
			DPRINTK(0,"DDC: 0x%02x\n", source);

			if (source & MASK_EDID_MTP) {
				DPRINTK(0,"\tEDID MTP end of process\n");
				/* reset MTP in use flag if set */
				if (tda1997x->mptrw_in_progress)
					tda1997x->mptrw_in_progress = 0;
			}

			/* we don't care about these */
			if (source & MASK_DDC_ERR) {
				DPRINTK(0,"\tmaster DDC error\n");
			}
			if (source & MASK_DDC_CMD_DONE) {
				DPRINTK(0,"\tmaster DDC cmd send correct\n");
			}
			if (source & MASK_READ_DONE) {
				DPRINTK(0,"\tEnd of Down EDID read\n");
			}
			if (source & MASK_RX_DDC_SW) {
				DPRINTK(0,"\tOutput DDC switching finished\n");
			}
			if (source & MASK_HDCP_DDC_SW) {
				DPRINTK(0,"\tHDCP DDC switching finished\n");
			}
			if (source & MASK_HDP_PULSE_END) {
				DPRINTK(0,"\tEnd of Hot Plug Detect pulse\n");
			}
			if (source & MASK_DET_5V) {
				DPRINTK(0,"\tDetected +5V\n");
			}
		}
		
		/* RATE interrupt source (Digital Input A/B activity: rate/presence/drift) */
		else if (interrupt_top_flags & INTERRUPT_RATE) {
			u8 irq_status, last_irq_status;

			source = io_read(REG_INT_FLG_CLR_RATE);
			io_write(REG_INT_FLG_CLR_RATE, source);
			DPRINTK(0,"RATE: 0x%02x\n", source);

			/* read status regs */
			last_irq_status = irq_status = tda1997x_read_activity_status_regs();

			/* read clock status reg until INT_FLG_CLR_RATE is still 0
			 * after the read to make sure its the last one
			 */
			reg = source;
			while (reg != 0) {
				irq_status = tda1997x_read_activity_status_regs();
				reg = io_read(REG_INT_FLG_CLR_RATE);
				io_write(REG_INT_FLG_CLR_RATE, reg);
				source |= reg;
			}

			/* we don't use these indicators */
#if 0
			if (source & MASK_RATE_B_DRIFT) {
				DPRINTK(0, "\tRate measrement of input B drifted\n");
			}
			if (source & MASK_RATE_B_ACT) {
				DPRINTK(0, "\tRate measurement of input B activity change\n");
			}
			if (source & MASK_RATE_B_PST) {
				DPRINTK(0, "\tRate measurement of input B presence change\n");
			}
			if (source & MASK_RATE_A_DRIFT) {
				DPRINTK(0, "\tRate measrement of input A drifted\n");
			}
			if (source & MASK_RATE_A_ACT) {
				DPRINTK(0, "\tRate measurement of input A activity change\n");
			}
			if (source & MASK_RATE_A_PST) {
				DPRINTK(0, "\tRate measurement of input A presence change\n");
			}
#endif

			/* we only pay attention to stability change events */
			if (source & (MASK_RATE_A_ST | MASK_RATE_B_ST)) {
				int input = (source & MASK_RATE_A_ST)?0:1;
				u8 mask = 1<<input;

				DPRINTK(0, "\tHDMI-%c: Rate measurement stability change\n", input+'A');
				DPRINTK(0, "\tirq_status=0x%02x/0x%02x/0x%02x\n",
					irq_status, last_irq_status, tda1997x->activity_status_reg);

				/* state change */
				if ((irq_status & mask) != (tda1997x->activity_status_reg & mask)) {

					/* activity lost */
					if ( (irq_status & mask) == 0) {
						printk(KERN_INFO "%s: HDMI-%c: Digital Activity Lost\n",
							KBUILD_MODNAME, input+'A');
						tda1997x->state = STATE_UNLOCKED;
						tda1997x->input_detect[input] = 0;
						tda1997x->hdmi_status = 0;
						tda1997x->hdmi_detected = 0;
						tda1997x->hdcp_detected = 0;
						tda1997x->eess_detected = 0;

						/* ConfigureUpDownSampler: 0=bypass 1=repeatchroma 2=interpolate */
						reg = io_read(REG_PIX_REPEAT);
						reg = (reg & ~0x30 /*MASK_UP_SEL*/) | (0 << 4); /* bypass */
						io_write(REG_PIX_REPEAT, reg);

						/* ConfigurePixelRepeater - repeat n-times each pixel */
						reg = io_read(REG_PIX_REPEAT);
						reg = (reg & ~0x0f /*MASK_PIX_REP*/) | 0; /* 0-times: disable */
						io_write(REG_PIX_REPEAT, reg);

						if (tda1997x->chip_revision == 0) {
							/* Clear HDMI mode flag in BCAPS (for N1) */
							io_write(REG_CLK_CFG, 0x03);
							io_write(REG_PON_OVR_EN, 0x01);
							io_write(REG_PON_CBIAS, 0x01);
							io_write(REG_PON_PLL, 0x01);
							reg = io_read(REG_MODE_RECOVER_CFG1);
							reg &= ~0x06;
							reg |= 0x02;
							io_write(REG_MODE_RECOVER_CFG1, reg);
							io_write(REG_CLK_CFG, 0x00);
							io_write(REG_PON_OVR_EN, 0x00);
							reg = io_read(REG_MODE_RECOVER_CFG1);
							reg &= ~0x06;
							io_write(REG_MODE_RECOVER_CFG1, reg);
						}

						tda1997x->video_mode.width = 0;
						tda1997x->video_mode.height = 0;
						tda1997x->video_mode.fps = 0;
						tda1997x->video_mode.interlaced = 0;
						tda1997x->video_mode.signal = 0;
					}

					else {
						printk(KERN_INFO "%s: HDMI-%c: Digital Activity Detected\n",
							KBUILD_MODNAME, input+'A');
						tda1997x->input_detect[input] = 1;
					}

					/* hold onto current state */
					tda1997x->activity_status_reg = (irq_status & mask);
				}
			}
		}
		
		/* MODE interrupt source (Gamut, ISRC2, ISRC1, ACP, GCP, Deep color flags) */
		else if (interrupt_top_flags & INTERRUPT_MODE) {
			source = io_read(REG_INT_FLG_CLR_MODE);
			io_write(REG_INT_FLG_CLR_MODE, source);
			DPRINTK(0,"MODE: 0x%02x\n", source);

			/* special processing for HDMI Flag IT */
			if (source & MASK_HDMI_FLG) {
				u8 statusChange;

				reg = io_read(REG_HDMI_FLAGS);
				reg &= 0x7c; /* ignore fifo_fail and fifo_warning bits */
				statusChange = tda1997x->hdmi_status ^ reg;
				DPRINTK(0,"\tHDMI_FLAGS:0x%02x prev=0x%02x changed=0x%02x\n", reg,
					tda1997x->hdmi_status, statusChange);
				if (statusChange) {
					tda1997x->hdmi_status = reg;
					/* Get HDMI Status */
					if (statusChange & 0x80)
						DPRINTK(0,"\t\tAUDIOFLAG - Audio packet in last videoframe\n");
					if (statusChange & 0x40)
						DPRINTK(0,"\t\tHDMI mode detected\n");
						tda1997x->hdmi_detected = 1;
					if (statusChange & 0x20)
						DPRINTK(0,"\t\tEESS mode detected\n");
						tda1997x->eess_detected = 1;
					if (statusChange & 0x10)
						DPRINTK(0,"\t\tHDCP encryption detected\n");
						tda1997x->hdcp_detected = 1;
					if (statusChange & 0x08)
						DPRINTK(0,"\t\tAVMUTE\n");
					if (statusChange & 0x04)
						DPRINTK(0,"\t\tLayout status Audio Sample Packet\n");
					if (statusChange & 0x02)
						DPRINTK(0,"\t\tFIFO read/write pointers are crossed\n");
					if (statusChange & 0x01)
						DPRINTK(0,"\t\tFIFO read ptr closer than 2 samples from write ptr\n");
				}
			}
			if (source & MASK_GAMUT) {
				u8 d[GDB_PACKET_HDR_LEN + GDB_PACKET_DAT_LEN];
				io_readn(REG_GBD_PACKET_TYPE, sizeof(d), d);
				DPRINTK(0,"\tGamut packet: type=%d nextField=%d GBDProfile=%d "
					"affectedGamutSeqNum=%d noCrntGBD=%d packetSeq=%d curSeq=%d\n",
					d[0],               /* type */
					(d[1] & 0x80) >> 7, /* nextField */
					(d[1] & 0x70) >> 4, /* GBDProfile */
					(d[1] & 0x0f),      /* affected Gamut SeqNum */
					(d[2] & 0x80) >> 7, /* noCrntGBD */
					(d[2] & 0x30) >> 4, /* packet SeqNum */
					(d[2] & 0x0f)       /* current SeqNum */
				);
			}
#if 0
			if (source & MASK_ISRC2) {
				DPRINTK(0,"\tISRC2 packet\n");

				if (1) // untested
				{
					u8 d[ISRC_PACKET_HDR_LEN + ISRC_PACKET_DAT_LEN];
					u8 crc = 0, i;

					io_readn(REG_ISRC2_PACKET_TYPE, sizeof(d), d);
					for (i = 0; i < sizeof(d); i++)
						crc += d[i];
					if (crc) {
						printk(KERN_ERR "%s: ISRC2 packet CRC failed\n", KBUILD_MODNAME);
					}
				}
			}
			if (source & MASK_ISRC1) {
				DPRINTK(0,"\tISRC1 packet\n");

				if (1) // untested
				{
					u8 d[ISRC_PACKET_HDR_LEN + ISRC_PACKET_DAT_LEN];
					u8 crc = 0, i;

					io_readn(REG_ISRC1_PACKET_TYPE, sizeof(d), d);
					for (i = 0; i < sizeof(d); i++)
						crc += d[i];
					if (crc) {
						printk(KERN_ERR "%s: ISRC1 packet CRC failed\n", KBUILD_MODNAME);
					} else {
						DPRINTK(0,"\tISRC1 packet: type=%d ISRCCont=%d ISRCValid=%d "
							"ISRCStatus=%d\n",
							d[0],
							(d[1] & 0x80) >> 7,
							(d[1] & 0x40) >> 6,
							(d[1] & 0x07)
						);
					}
				}
			}
			if (source & MASK_ACP) {
				DPRINTK(0,"\tAudio Content Protection (ACP) Packet\n");

				if (1) // untested
				{
					u8 d[ACP_PACKET_HDR_LEN + ACP_PACKET_DAT_LEN];
					u8 crc = 0, i;

					io_readn(REG_ACP_PACKET_TYPE, sizeof(d), d);
					for (i = 0; i < sizeof(d); i++)
						crc += d[i];
					if (crc) {
						printk(KERN_ERR "%s: ACP packet CRC failed\n", KBUILD_MODNAME);
					}
				}
			}
#endif
			if (source & MASK_DC_NO_GCP) {
				DPRINTK(0,"\tGCP not received in 5 frames\n");
			}
			if (source & MASK_DC_PHASE) {
				DPRINTK(0,"\tDeep color mode pixel phase needs update\n");
			}
			if (source & MASK_DC_MODE) {
				reg = io_read(REG_DEEP_COLOR_MODE);
				DPRINTK(0,"\tDeep color mode color depth changed: "
					"pixelPackingPhase=0x%02x mode=%d\n",
					reg & MASK_DC_PIXEL_PHASE,
					reg & MASK_DC_COLOR_DEPTH);
			}
		}
		
		/* Infoframe change interrupt source */
		else if (interrupt_top_flags & INTERRUPT_INFO ) {
			source = io_read(REG_INT_FLG_CLR_INFO);
			io_write(REG_INT_FLG_CLR_INFO, source);
			DPRINTK(0,"INFO: 0x%02x\n", source);

			/* Vendor-Specific Infoframe */
			if (source & MASK_VS_IF_OTHER_BK2 ||
				source & MASK_VS_IF_OTHER_BK1 ||
				source & MASK_VS_IF_HDMI)
			{
				u8 VSIUpdate, VSOther1Update, VSOther2Update;

				/* Read the Update registers */
				VSIUpdate = io_read(VS_HDMI_IF_UPDATE);
				VSOther1Update = io_read(VS_BK1_IF_UPDATE);
				VSOther2Update = io_read(VS_BK2_IF_UPDATE);

				/* discard ITs that are too old */
				if (VSIUpdate >= 3)	  source &= ~MASK_VS_IF_HDMI;
				if (VSOther1Update >= 3) source &= ~MASK_VS_IF_OTHER_BK1;
				if (VSOther2Update >= 3) source &= ~MASK_VS_IF_OTHER_BK2;

				/* No new VSI has been received if these 3 registers are > 3 */
				if (tda1997x->vsi_received
					 && (VSIUpdate>=3) && (VSOther1Update>=3) && (VSOther2Update>=3))
				{
					/* false VSI received */
					tda1997x->vsi_received = 0;
				}
				else
				{
					tda1997x->vsi_received = 1;

					if (source & MASK_VS_IF_HDMI) {
						DPRINTK(0,"\tVendor-Specific InfoFrame: HDMI\n");
						tda1997x_parse_infoframe(tda1997x, VS_HDMI_IF_TYPE);
					}

					if (source & MASK_VS_IF_OTHER_BK1) {
						DPRINTK(0,"\tVendor-Specific InfoFrame: BK1\n");
						tda1997x_parse_infoframe(tda1997x, VS_BK1_IF_TYPE);
					}

					if (source & MASK_VS_IF_OTHER_BK2) {
						DPRINTK(0,"\tVendor-Specific InfoFrame: BK2\n");
						tda1997x_parse_infoframe(tda1997x, VS_BK2_IF_TYPE);
					}
				}
			}

			/* MPEG Source Product infoframe */
			if (source & MASK_MPS_IF) {
				DPRINTK(0, "\tMPEG Source Product InfoFrame content change\n");
				tda1997x_parse_infoframe(tda1997x, MPS_IF_TYPE);
			}

			/* Audio infoframe */
			if (source & MASK_AUD_IF) {
				DPRINTK(0, "\tAudio InfoFrame content change\n");
				tda1997x_parse_infoframe(tda1997x, AUD_IF_TYPE);
			}

			/* Source Product Descriptor infoframe change */
			if (source & MASK_SPD_IF) {
				DPRINTK(0, "\tSource Product Descriptor InfoFrame change\n");
				tda1997x_parse_infoframe(tda1997x, SPD_IF_TYPE);
			}

			/* Auxillary Video Information infoframe */
			if (source & MASK_AVI_IF) {
				DPRINTK(0, "\tAuxiliary Video information InfoFrame change\n");
				tda1997x_parse_infoframe(tda1997x, AVI_IF_TYPE);
			}
		}
		
		/* Audio interrupt source:
		 *   freq change, DST,OBA,HBR,ASP flags, mute, FIFO err
		 */
		else if (interrupt_top_flags & INTERRUPT_AUDIO ) {
			source = io_read(REG_INT_FLG_CLR_AUDIO);
			io_write(REG_INT_FLG_CLR_AUDIO, source);
			DPRINTK(0,"AUDIO: 0x%02x\n", source);

			if (source & MASK_ERROR_FIFO_PT || /* Audio FIFO pointer error */
			    source & MASK_MUTE_FLG)        /* Audio mute */
			{
				if (source & MASK_MUTE_FLG) {
					DPRINTK(0, "\tAudio MUTE\n");
				} else {
					DPRINTK(0, "\tAudio FIFO error\n");
				}

				/* audio reset audio FIFO */
				reg = io_read(REG_SUS_STATUS);
			  if ((reg & MASK_SUS_STATE_VALUE) == LAST_STATE_REACHED) {
					reg = io_read(REG_HDMI_INFO_RST);
					reg |= MASK_SR_FIFO_FIFO_CTRL;
					io_write(REG_HDMI_INFO_RST, reg);
					reg &= ~MASK_SR_FIFO_FIFO_CTRL;
					io_write(REG_HDMI_INFO_RST, reg);

					/* reset channel status IT if present */
					source &= ~(MASK_CH_STATE);
				}
			}
			if (source & MASK_AUDIO_FREQ_FLG) {
				DPRINTK(0, "\tAudio freq change\n");
				tda1997x_get_audio_frequency(tda1997x);
				printk(KERN_INFO "%s: Audio Frequency Change: %dHz\n",
				     KBUILD_MODNAME,
				     tda1997x->audio_mode.samplerate);
			}
			if (source & MASK_AUDIO_FLG) {
				reg = io_read(REG_AUDIO_FLAGS);
				DPRINTK(0, "\tAudio flag: 0x%02x\n", reg);
				if (reg & 0x08) {
					DPRINTK(0, "\t\tDTS packets detected\n");
				}
				if (reg & 0x04) {
					DPRINTK(0, "\t\tOBA packets detected\n");
				}
				if (reg & 0x02) {
					DPRINTK(0, "\t\tHBR packets detected\n");
				}
				if (reg & 0x01) {
					DPRINTK(0, "\t\tAudio sample packets detected\n");
				}
			}
			if (source & MASK_CH_STATE) {
				DPRINTK(0, "\tChannel status\n");
			}
			if (source & MASK_UNMUTE_FIFO) {
				DPRINTK(0, "\tUnmute audio FIFO\n");
			}
		}
		
		/* HDCP interrupt source (content protection) */
		if (interrupt_top_flags & INTERRUPT_HDCP) {
			source = io_read(REG_INT_FLG_CLR_HDCP);
			io_write(REG_INT_FLG_CLR_HDCP, source);
			DPRINTK(0,"HDCP: 0x%02x\n", source);

			/* reset MTP in use flag if set */
			if (source & MASK_HDCP_MTP) {
				DPRINTK(0, "\tHDCP MTP in use\n");
				tda1997x->mptrw_in_progress = 0;
			}
			if (source & MASK_HDCP_DLMTP) {
				DPRINTK(0,"\tHDCP end download MTP to SRAM\n");
			}
			if (source & MASK_HDCP_DLRAM) {
				DPRINTK(0,"\tHDCP end download keys from SRAM to HDCP core\n");
			}
			if (source & MASK_HDCP_ENC) {
				DPRINTK(0,"\tHDCP_ENC\n");
			}
			if (source & MASK_STATE_C5) {
				DPRINTK(0,"\tHDCP State C5 reached\n");

				/* REPEATER: mask AUDIO and IF interrupts to avoid IF during auth */
				reg = io_read(REG_INT_MASK_TOP);
				reg &= ~(INTERRUPT_AUDIO | INTERRUPT_INFO);
				io_write(REG_INT_MASK_TOP, reg);
				interrupt_top_flags &= (INTERRUPT_AUDIO | INTERRUPT_INFO);
			}
			if (source & MASK_AKSV) {
				DPRINTK(0,"\tAKSV received (Start of Authentication)\n");
			}
		}

		/* AFE interrupt source */
		else if (interrupt_top_flags & INTERRUPT_AFE ) {
			source = io_read(REG_INT_FLG_CLR_AFE);
			io_write(REG_INT_FLG_CLR_AFE, source);
			DPRINTK(0,"AFE: 0x%02x\n", source);
		}
	} while (interrupt_top_flags != 0);

	/* we handled all alerts; re-enable level-triggered IRQ */
	enable_irq(tda1997x->irq);
}

/** tda1997x interrupt handler
 */
static irqreturn_t tda1997x_isr(int irq, void *d)
{
	struct tda1997x_data *tda1997x = d;
	
	/* disable level-triggered IRQs until we handle them */
	disable_irq_nosync(irq);
	schedule_work(&tda1997x->work);

	return IRQ_HANDLED;
}

/***********************************************************************
 * I2C client and driver.
 ***********************************************************************/

/**
 * tda1997x I2C detach function.
 * Called on rmmod.
 *
 *  @param *client	struct i2c_client*.
 *  @return Error code indicating success or failure.
 */
static int tda1997x_remove(struct i2c_client *client)
{
	struct tda1997x_data *tda1997x = i2c_get_clientdata(client);
	dev_dbg(&tda1997x_data.client->dev,
		"%s:Removing tda1997x video decoder @ 0x%02X from adapter %s\n",
		__func__, client->addr << 1, client->adapter->name);

	tda1997x_power_mode(&tda1997x_data, 0);

	if (client->irq) {
		devm_free_irq(&client->dev, client->irq, &tda1997x_data);
	}

/*
	if (tda1997x->vid_child) {
		platform_device_del(tda1997x->vid_child);
	}
*/

	if (tda1997x->client_cec)
		i2c_unregister_device(tda1997x->client_cec);

	sysfs_remove_group(&client->dev.kobj, &attr_group);

	if (dvddio_regulator)
		regulator_disable(dvddio_regulator);
	if (dvdd_regulator)
		regulator_disable(dvdd_regulator);
	if (avdd_regulator)
		regulator_disable(avdd_regulator);

	return 0;
}


static int tda1997x_regulator_enable(struct device *dev,
				     struct tda1997x_platform_data *pdata)
{
	int ret = 0;

	dvddio_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(dvddio_regulator)) {
		regulator_set_voltage(dvddio_regulator,
				      TDA1997X_VOLTAGE_DIGITAL_IO,
				      TDA1997X_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(dvddio_regulator);
		if (ret) {
			dev_err(dev, "set io voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set io voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get io voltage\n");
	}

	dvdd_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(dvdd_regulator)) {
		ret = regulator_set_voltage(dvdd_regulator,
					    TDA1997X_VOLTAGE_DIGITAL_CORE,
					    TDA1997X_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(dvdd_regulator);
		if (ret) {
			dev_err(dev, "set core voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set core voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get core voltage\n");
	}

	avdd_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(avdd_regulator)) {
		ret = regulator_set_voltage(avdd_regulator,
					    TDA1997X_VOLTAGE_ANALOG,
					    TDA1997X_VOLTAGE_ANALOG);
		ret = regulator_enable(avdd_regulator);
		if (ret) {
			dev_err(dev, "set analog voltage failed\n");
			return ret;
		} else {
			dev_dbg(dev, "set analog voltage ok\n");
		}
	} else {
		dev_warn(dev, "cannot get analog voltage\n");
	}

	return ret;
}

static int parse_vidout_fmt(const char *mode)
{
	int clkmode;
	if (!strcmp(mode, "444"))
		clkmode = VIDEOFMT_444;
	else if (!strcmp(mode, "422_smp"))
		clkmode = VIDEOFMT_422_SMP;
	else if (!strcmp(mode, "422_ccir"))
		clkmode = VIDEOFMT_422_CCIR;
	else
		clkmode = -EINVAL;

	return clkmode; 
}

static int parse_vidout_clkmode(const char *mode)
{
	int clkmode;
	if (!strcmp(mode, "single_edge"))
		clkmode = CLOCK_SINGLE_EDGE;
	else if (!strcmp(mode, "dual_edge"))
		clkmode = CLOCK_DUAL_EDGE;
	else if (!strcmp(mode, "single_edge_toggled"))
		clkmode = CLOCK_SINGLE_EDGE_TOGGLED;
	else if (!strcmp(mode, "dual_edge_toggled"))
		clkmode = CLOCK_DUAL_EDGE_TOGGLED;
	else
		clkmode = -EINVAL;

	return clkmode; 
}

static int parse_audout_fmt(const char *mode)
{
	int clkmode;
	if (!strcmp(mode, "i2s16"))
		clkmode = AUDIO_FMT_I2S16;
	else if (!strcmp(mode, "i2s32"))
		clkmode = AUDIO_FMT_I2S32;
	else if (!strcmp(mode, "spdif"))
		clkmode = AUDIO_FMT_SPDIF;
	else if (!strcmp(mode, "oba"))
		clkmode = AUDIO_FMT_OBA;
	else if (!strcmp(mode, "i2s16_hbr_straight"))
		clkmode = AUDIO_FMT_I2S16_HBR_STRAIGHT;
	else if (!strcmp(mode, "i2s16_hbr_demux"))
		clkmode = AUDIO_FMT_I2S16_HBR_DEMUX;
	else if (!strcmp(mode, "i2s32_hbr_demux"))
		clkmode = AUDIO_FMT_I2S32_HBR_DEMUX;
	else if (!strcmp(mode, "dst"))
		clkmode = AUDIO_FMT_DST;
	else
		clkmode = -EINVAL;

	return clkmode; 
}

static int tda1997x_get_of_property(struct device *dev,
				    struct tda1997x_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	const char *vidout_fmt, *vidout_clk, *audout_fmt;
	u32 hdcp, ddc_slave, blc, trc;
	u32 port_configs;
	u32 audout_clk, audout_layout, max_pixel_rate = 0;
	int err;

	/* defaults (use inverted vs/hs/de) */
	/* TODO: add of bindings for these */
	pdata->vidout_sel_vs = 1;
	pdata->vidout_invert_vs =1;
	pdata->vidout_sel_hs = 1;
	pdata->vidout_invert_hs =1;
	pdata->vidout_sel_de = 1;
	pdata->vidout_invert_de =1;

	/* enable HDCP */
	err = of_property_read_u32(np, "hdcp", &hdcp);
	if (err) {
		dev_dbg(dev, "get of property hdcp fail\n");
		return err;
	}
	/* DDC Slave address */
	err = of_property_read_u32(np, "ddc_slave", &ddc_slave);
	if (err) {
		dev_dbg(dev, "get of property ddc_slave fail\n");
		return err;
	}

	/* Video output mode */
	err = of_property_read_string(np, "vidout_fmt", &vidout_fmt);
	if (err) {
		dev_dbg(dev, "get of property vidout_fmt fail\n");
		return err;
	}
	/* insert timing codes (SAV/EAV) in stream */
	err = of_property_read_u32(np, "vidout_trc", &trc);
	if (err) {
		dev_dbg(dev, "get of property vidout_trc fail\n");
		return err;
	}
	/* insert blanking codes in stream */
	err = of_property_read_u32(np, "vidout_blc", &blc);
	if (err) {
		dev_dbg(dev, "get of property vidout_blc fail\n");
		return err;
	}
	/* video output clock mode */
	err = of_property_read_string(np, "vidout_clkmode", &vidout_clk);
	if (err) {
		dev_dbg(dev, "get of property vidout_clkmode fail\n");
		return err;
	}
	/* video output port config */
	of_find_property(np, "vidout_portcfg", &port_configs);
	err = of_property_read_u8_array(np, "vidout_portcfg",
					pdata->vidout_port_config,
					port_configs);
	if (err) {
		dev_dbg(dev, "get of property vidout_portcfg fail\n");
		return err;
	}
	pdata->vidout_port_config_no = port_configs;
	/* max pixrate */
	of_property_read_u32(np, "max-pixel-rate", &max_pixel_rate);

	/* audio output format */
	err = of_property_read_string(np, "audout_fmt", &audout_fmt);
	if (err) {
		dev_dbg(dev, "get of property audout_fmt fail\n");
		return err;
	}
	/* audio output sysclk */
	err = of_property_read_u32(np, "audout_sysclk", &audout_clk);
	if (err) {
		dev_dbg(dev, "get of property audout_clkmode fail\n");
		return err;
	}
	/* audio layout */
	err = of_property_read_u32(np, "audout_layout", &audout_layout);
	if (err) {
		dev_dbg(dev, "get of property audout_layout fail\n");
		return err;
	}

	pdata->hdcp = hdcp;
	pdata->ddc_slave = ddc_slave;
	pdata->vidout_format = parse_vidout_fmt(vidout_fmt);
	pdata->vidout_trc = trc;
	pdata->vidout_blc = blc;
	pdata->vidout_clkmode = parse_vidout_clkmode(vidout_clk);
	pdata->max_pixel_rate = max_pixel_rate;
	pdata->audout_layout = audout_layout;
	pdata->audout_format = parse_audout_fmt(audout_fmt);
	switch (audout_clk) {
	default:
	case 128:
		pdata->audout_sysclk = AUDIO_SYSCLK_128FS;
		break;
	case 256:
		pdata->audout_sysclk = AUDIO_SYSCLK_256FS;
		break;
	case 512:
		pdata->audout_sysclk = AUDIO_SYSCLK_512FS;
		break;
	}

	return err;
}

/**
 * tda1997x I2C probe function.
 * Function set in i2c_driver struct.
 * Called by insmod.
 *
 *  @param *adapter	I2C adapter descriptor.
 *
 *  @return		Error code indicating success or failure.
 */
static int tda1997x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0;
	int i;
	u8 reg;
	struct tda1997x_data *tda1997x = &tda1997x_data;
	struct tda1997x_platform_data *pdata;
	struct device *dev = &client->dev;

	dev_dbg(dev, "%s\n", __func__);
	pdata = devm_kzalloc(dev, sizeof(struct tda1997x_platform_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;
	dev->platform_data = pdata;

	if (!client->irq) {
		dev_err(dev, "get tda1997x of interrupt fail\n");
		return -EINVAL;
	}

	ret = tda1997x_get_of_property(dev, pdata);
	if (ret < 0) {
		dev_err(dev, "get tda1997x of property fail\n");
		return ret;
	}

	/* probe chip */
/*
	ret = i2c_smbus_read_byte_data(client, REG_CMTP_REG10 & 0xff);
	if (ret < 0)
		return -ENODEV;
*/

	tda1997x_regulator_enable(dev, pdata);

	memset(tda1997x, 0, sizeof(tda1997x_data));
	i2c_set_clientdata(client, tda1997x);
	spin_lock_init(&tda1997x->lock);
	tda1997x->page = 0xff;
	tda1997x->client = client;
	tda1997x->irq = client->irq;
	tda1997x->pdata = pdata;
	tda1997x->internal_edid = !pdata->external_edid;
	INIT_WORK(&tda1997x->work, tda1997x_work);
	mutex_init(&tda1997x->page_lock);
	mutex_init(&tda1997x->cec_lock);
	switch(pdata->audout_layout) {
	case AUDIO_LAYOUT_FORCED_0:
		tda1997x->audio_mode.channels = 2;
		break;
	case AUDIO_LAYOUT_FORCED_1:
		tda1997x->audio_mode.channels = 8;
		break;
	default:
		tda1997x->audio_mode.channels = 8;
		break;
	}
	switch(pdata->audout_format) {
	case AUDIO_FMT_I2S16:
		tda1997x->audio_mode.samplesize = 2;
		break;
	case AUDIO_FMT_I2S32:
		tda1997x->audio_mode.samplesize = 2;
		break;
	default:
		tda1997x->audio_mode.samplesize = 0;
	}

	/* sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &attr_group);
	if (ret)
		printk(KERN_ERR "%s: failed creating sysfs attrs\n", KBUILD_MODNAME);

	tda1997x->colorspace = COLORSPACE_RGB;
	tda1997x->colorimetry = COLORIMETRY_NONE;
	tda1997x->resolutiontype = RESTYPE_SDTV;

	/* disable/reset HDCP to get correct I2C access to Rx HDMI */
	io_write(REG_MAN_SUS_HDMI_SEL, MAN_RST_HDCP | MAN_DIS_HDCP);

	/* Read chip configuration*/
	reg = io_read(REG_CMTP_REG10);
	tda1997x->tmdsb_clk       = (reg >> 6) & 0x01;  /* use tmds clock B_inv for B */
	tda1997x->tmdsb_soc       = (reg >> 5) & 0x01;  /* tmds of input B */
	tda1997x->port_30bit      = (reg >> 2) & 0x03;  /* 30bit vs 24bit */
	tda1997x->output_2p5      = (reg >> 1) & 0x01;  /* output supply 2.5v */
	tda1997x->cec_enabled     = (reg >> 0) & 0x01;  /* CEC enabled */
	switch((reg >> 4) & 0x03) {
		case 0x00:
			tda1997x->chip = 19971;
			tda1997x->input = INPUT_HDMI_A;
			break;
		case 0x01:
			tda1997x->chip = 19972;
			tda1997x->input = INPUT_AUTO_DIGITAL;
			break;
		case 0x02:
		case 0x03:
			tda1997x->chip = 19973;
			tda1997x->input = INPUT_HDMI_A;
			break;
	}

	/* read chip revision */
	tda1997x->chip_revision = io_read(REG_CMTP_REG11);

	/* if N2 version, reset compdel_bp as it may generate some small pixel
	 * shifts in case of embedded sync/or delay lower than 4 */
	if (tda1997x->chip_revision != 0) {
		io_write(REG_MAN_SUS_HDMI_SEL, 0x00);
		io_write (REG_VDP_CTRL, 0x1f);
	}

	/* The CEC I2C address is not yet correct.  We need to take into account
	 * possible config setting in SLAVE_ADDR register, however the Hw I2C address
	 */
	tda1997x->cec_slave = 0x34 + ((io_read(REG_SLAVE_ADDR)>>4) & 0x03);

	pr_info("NXP TDA%d N%d detected: %dbit VP%s\n",
		tda1997x->chip,
		tda1997x->chip_revision + 1,
		(tda1997x->port_30bit)?30:24,
		(tda1997x->cec_enabled)?", CEC ":"");
	pr_info("video out format: %s\n", vidfmt_names[pdata->vidout_format]);
	if (tda1997x->cec_enabled)
		pr_info("CEC slave address 0x%02x\n", tda1997x->cec_slave);
	if (tda1997x->pdata->max_pixel_rate)
		pr_info("max pixel rate: %d MP/sec\n", pdata->max_pixel_rate);

	/* Attach a second dummy i2c_client for CEC register access */
	tda1997x->client_cec = i2c_new_dummy(client->adapter, tda1997x->cec_slave);
	if (!tda1997x->client_cec) {
		printk(KERN_ERR "%s: failed to register CEC client\n", KBUILD_MODNAME);
	}

	/* disable HPD */
	io_write(REG_HPD_AUTO_CTRL, 0x08); /* hpd_unsel */
	
	if (tda1997x->chip_revision == 0) {
		io_write(REG_MAN_SUS_HDMI_SEL, MAN_DIS_HDCP | MAN_RST_HDCP);
		io_write(REG_CGU_DEBUG_SEL,	0x08);
	}

	/* reset infoframe at end of start-up-sequencer */
	io_write(REG_SUS_SET_RGB2, 0x06);
	io_write(REG_SUS_SET_RGB3, 0x06);

	/* update page 0 */
	io_write(REG_RT_MAN_CTRL, 0x43);

	/* CEC
	 */
	/* enable sync measurement timing */
	tda1997x_cec_write(REG_PWR_CONTROL & 0xff, 0x04);
	/* adjust CEC clock divider */
	tda1997x_cec_write(REG_OSC_DIVIDER & 0xff, 0x03);
	tda1997x_cec_write(REG_EN_OSC_PERIOD_LSB & 0xff, 0xa0);
	io_write(REG_TIMER_D, 0x54);
	/* enable power switch - SRAM content is always valid
	 * (in case E-MTP is not or mis-programmed)
	 */	
	reg = tda1997x_cec_read(REG_CONTROL & 0xff);
	reg |= 0x20;
	tda1997x_cec_write(REG_CONTROL & 0xff, reg);
	mdelay(50);

	/* read the chip version */
	reg = io_read(REG_VERSION);
	/* get the chip configuration */
	reg = io_read(REG_CMTP_REG10);

	/* init interrupt masks we care about */
	io_write(REG_INT_MASK_TOP,  0x7f); /* hdcp,audio,info,mode,rate,ddc,sus */
	io_write(REG_INT_MASK_SUS,  0xa8); /* config_mtp,fmt,sus_end,sus_st */
	io_write(REG_INT_MASK_DDC,  0x00); /* none */
	io_write(REG_INT_MASK_RATE, 0x44); /* rate_b_st, rate_a_st */
	io_write(REG_INT_MASK_MODE, 0xf9); /* hdmi_flg,gamut,isrc2,isrc1,acp,dc_mode */
	io_write(REG_INT_MASK_INFO, 0x7f); /* mps_if,aud_if,spd_if,avi_if,
	                                      vs_if_other_bk2,vs_if_other_bk1,
	                                      vs_if_hdmi */
	io_write(REG_INT_MASK_AUDIO,0x3f); /* audio_freq,audio_flg,mute_flg,
	                                      ch stat,unmount_fifo,fifo_err */
	io_write(REG_INT_MASK_HDCP, 0x02); /* state_c5 */
	io_write(REG_INT_MASK_AFE,  0x00); /* none */
	
	/* clear all interrupts */
	io_write(REG_INT_FLG_CLR_TOP,   0xff);
	io_write(REG_INT_FLG_CLR_SUS,   0xff);
	io_write(REG_INT_FLG_CLR_DDC,   0xff);
	io_write(REG_INT_FLG_CLR_RATE,  0xff);
	io_write(REG_INT_FLG_CLR_MODE,  0xff);
	io_write(REG_INT_FLG_CLR_INFO,  0xff);
	io_write(REG_INT_FLG_CLR_AUDIO, 0xff);
	io_write(REG_INT_FLG_CLR_HDCP,  0xff);
	io_write(REG_INT_FLG_CLR_AFE,   0xff);

	/* init TMDS equalizer */
	if (tda1997x->chip_revision == 0)
		io_write(REG_CGU_DEBUG_SEL, 0x08);
	io_write24(REG_CLK_MIN_RATE, CLK_MIN_RATE);
	io_write24(REG_CLK_MAX_RATE, CLK_MAX_RATE);
	if (tda1997x->chip_revision == 0)
		io_write(REG_WDL_CFG, WDL_CFG_VAL);
	/* DC filter */
	io_write(REG_DEEP_COLOR_CTRL, DC_FILTER_VAL);
	
	/* disable test pattern */
	io_write(REG_SERVICE_MODE, 0x00);

	/* update HDMI INFO CTRL */
	io_write(REG_INFO_CTRL, 0xff);

	/* write HDMI INFO EXCEED value */
	io_write(REG_INFO_EXCEED, 3);

	if (tda1997x->chip_revision == 0) {
		/* clear HDMI mode flag in BCAPS (for N1) */
		io_write(REG_CLK_CFG, 0x03);
		io_write(REG_PON_OVR_EN, 0x01);
		io_write(REG_PON_CBIAS, 0x01);
		io_write(REG_PON_PLL, 0x01);
		
		reg = io_read(REG_MODE_RECOVER_CFG1);
		reg &= ~0x06;
		reg |= 0x02;
		io_write(REG_MODE_RECOVER_CFG1, reg);
		io_write(REG_CLK_CFG, 0x00);
		io_write(REG_PON_OVR_EN, 0x00);
		reg = io_read(REG_MODE_RECOVER_CFG1);
		reg &= ~0x06;
		io_write(REG_MODE_RECOVER_CFG1, reg);
	}

	/* No HDCP acknowledge when HDCP is disabled
	 * and reset SUS to force format detection
	 */
	tda1997x_hdmi_info_reset(NACK_HDCP, 1);
	//tda1997x_hdmi_info_reset(NACK_HDCP, 0);

	/* get key description seed in fuction of seed table if provded */
	tda1997x_configure_mtp(tda1997x, MTP_START_READ); /* Start read */
	reg = io_read(0x4000); /* read from MTP */
	for (i = 0; i < RX_SEED_TABLE_LEN; i++) {
		if ((rx_seed_table[i].seedVal == 0) && (rx_seed_table[i].lookUpVal == 0))
			break;
		else if (rx_seed_table[i].lookUpVal == reg) { /* MTP_DATA_LSB */
			/* found seed replace seed in key_decryption_seed */
			tda1997x->key_decryption_seed = rx_seed_table[i].seedVal;
			break;
		}
	}
	DPRINTK(0,"HDCP decryption seed:0x%04x\n", tda1997x->key_decryption_seed);
		
	/* disable HDCP */
	tda1997x_configure_hdcp(tda1997x, HDCP_DECRYPTKEY_ON, DISABLE,
		tda1997x->pdata->ddc_slave, tda1997x->key_decryption_seed);

	/* set the state machine */
	tda1997x->state = STATE_INITIALIZED;

	/* Set HPD low */
	tda1997x_manual_hpd(tda1997x, HPD_LOW);

	/* Configure receiver capabilities */
	io_write(REG_HDCP_BCAPS, HDCP_HDMI | HDCP_FAST_REAUTH);

	/* Configure HDMI
	 *
	 * HDMI_CTRL bits:
	 * 3:2 - mute_mode:
	 *   00: use control packet
	 *   01: mute off
	 *   10: mute on
	 * 1:0 - hdcp_mode:
	 *   00: auto
	 *   01: oess
	 *   10: eess
	 */
	io_write(REG_HDMI_CTRL, 0x00); /* Auto HDCP mode, packet controlled mute */

	/* Configure EDID
	 *
	 * EDID_ENABLE bits:
	 *  7 - nack_off
	 *  6 - edid_only
	 *  1 - edid_b_en
	 *  0 - edid_a_en
	 */
	reg = io_read(REG_EDID_ENABLE);
	if (!tda1997x->internal_edid)
		reg &= ~0x83; /* EDID Nack ON */
	else
		reg |= 0x83;  /* EDID Nack OFF */
	io_write(REG_EDID_ENABLE, reg);

	/* HDCP Activation */
	if (pdata->hdcp) {
		DPRINTK(0,"%s: Activating HDCP\n", __func__);

		/* No HDCP acknowledge when HDCP is disabled */
		tda1997x_hdmi_info_reset(NACK_HDCP, 0);

		/* disable HDCP */
		tda1997x_configure_hdcp(tda1997x, HDCP_DECRYPTKEY_ON, DISABLE,
			tda1997x->pdata->ddc_slave, tda1997x->key_decryption_seed);

		/* index first secret key */
		io_write(REG_HDCP_KIDX, 0x00);

		/* download MTP */
		tda1997x_configure_mtp(tda1997x, MTP_START_DOWNLOAD);
		mdelay(20);

		/* enable HDCP */
		tda1997x_configure_hdcp(tda1997x, HDCP_DECRYPTKEY_OFF, ENABLE,
			tda1997x->pdata->ddc_slave, tda1997x->key_decryption_seed);

		/* Enable HDCP acknowledge when HDCP is enabled */
		//tda1997x_hdmi_info_reset(0x00, 0);

		/* Configure HDCP error protection
		 */
		DPRINTK(0,"Configure HDCP error protection\n");
		/* delockDelay - Delay before delocking the word locker */
		io_write(REG_DELOCK_DELAY, 0x07);  /* delockDelay (7=max) */
		/* HDCP_DE_CTRL bits:
		 *  7:6 - de_measurement_mode:
		 *   00: 1_VDP
		 *   01: 2_VDP
		 *   10: 3_VDP
		 *   11: 4_VDP
		 *    5 - de_regen_mode: 1-enable
		 *  4:3 - de_filter_sensitivity:
		 *  2:0 - de_composition_mode:
		 *   000: CH0
		 *   001: CH1
		 *   010: CH2
		 *   011: AND
		 *   100: OR
		 *   101: MIXED
		 */
		io_write(REG_HDCP_DE_CTRL, 3<<3); /* de_filter_sensitivity=3 */
		/* HDCP_EP_FILT_CTRL bits:
		 *  5:4 - ctl_filter_sensitivity
		 *  3:2 - vs_filter_sensitivity
		 *  1:0 -  hs_filter_sensitivity
		 */
		/* ctl_filter_sentivity = vs_filter_sensitivity = hs_filter_sensitivity = 3 */
		io_write(REG_HDCP_EP_FILT_CTRL, (0x3<<4) | (0x3<<2) | 0x3);
	}

	/* reset start-up-sequencer to force format detection */
	tda1997x_hdmi_info_reset(0x00, 1);
	//tda1997x_hdmi_info_reset(0x00, 0);

	/* Set HPD high */
	tda1997x_manual_hpd(tda1997x, HPD_HIGH_OTHER);

	/* Internal EDIDs are enabled - we can now load EDID */
	if (tda1997x->internal_edid) {
		/* Load EDID into embedded memory */
		tda1997x_load_edid_data(edid_block);

		/* Load DDC and RT data into embedded memory */
		tda1997x_load_config_data(tda1997x, ddc_config0, rt_config);
	}

	/* Set HPD high (now that EDID is ready) */
	tda1997x_manual_hpd(tda1997x, HPD_HIGH);

	/* Select input */
	tda1997x_select_input(tda1997x->input);

	/* enable matrix conversion bypass (no conversion) */
	io_write(REG_VDP_CTRL, io_read(REG_VDP_CTRL) | (1<<0));

	/* set video output mode */
	tda1997x_set_video_outputformat(tda1997x->pdata);
	for (i = 0; i < tda1997x->pdata->vidout_port_config_no; i++)
		io_write(REG_VP35_32_CTRL + i, tda1997x->pdata->vidout_port_config[i]);

	/* configure audio output mode */
	tda1997x_configure_audio_formatter(tda1997x->pdata, 0);
	/* configure audio clock mode:
	 * Audio clock generation from Xtal: 128fs, 256fs, 512fs and
	 * Fs = 32kHz, 44.1kHz, 48kHz, 88.2kHz, 96kHz, 172.4kHz, 192kHz
	 */
	//io_write(REG_AUDIO_CLOCK_MODE, tda1997x->pdata->audio_sysclk);
	io_write(REG_AUDIO_CLOCK_MODE, AUDIO_SYSCLK_128FS);

	/* reset advanced infoframes (ISRC1/ISRC2/ACP) */
	tda1997x_hdmi_info_reset(RESET_AI, 0);
	//tda1997x_hdmi_info_reset(0x00, 0);
	/* reset infoframe */
	tda1997x_hdmi_info_reset(RESET_IF, 0);
	//tda1997x_hdmi_info_reset(0x00, 0);
	/* reset audio infoframes */
	tda1997x_hdmi_info_reset(RESET_AUDIO, 0);
	//tda1997x_hdmi_info_reset(0x00, 0);
	/* reset gamut */
	tda1997x_hdmi_info_reset(RESET_GAMUT, 0);
	//tda1997x_hdmi_info_reset(0x00, 0);

	/* get initial HDMI status */
	tda1997x->hdmi_status = io_read(REG_HDMI_FLAGS);

	/* get DEEP color mode */
	reg = io_read(REG_DEEP_COLOR_MODE);

	/* register interrupt handler */
	if (client->irq) {
		ret = devm_request_irq(&client->dev, client->irq, tda1997x_isr, 0,
			"tda1997x_irq", tda1997x);
		if (ret < 0) {
			pr_err("%s:interrupt gpio%d registration failed, error=%d \n",
				__func__, client->irq, ret);
		}
		pr_debug("registered irq%d\n", client->irq);
	}

	return ret;
}

static const struct i2c_device_id tda1997x_id[] = {
	{"tda1997x", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, tda1997x_id);

//static const struct of_device_id tda1997x_dt_ids[] = {
//	{ .compatible = "nxp,tda1997x", },
//	{ /* sentinel */ }
//};
//MODULE_DEVICE_TABLE(of, tda1997x_dt_ids);

static struct i2c_driver tda1997x_i2c_driver = {
	.driver = {
		.name = "tda1997x",
		.owner = THIS_MODULE,
		//.of_match_table = tda1997x_dt_ids,
	},
	.probe = tda1997x_probe,
	.remove = tda1997x_remove,
	.id_table = tda1997x_id,
};

/**
 * tda1997x init function.
 * Called on insmod.
 *
 * @return Error code indicating success or failure.
 */
static __init int tda1997x_init(void)
{
	u8 err = 0;

	pr_debug("%s\n", __func__);

	/* Tells the i2c driver what functions to call for this driver. */
	err = i2c_add_driver(&tda1997x_i2c_driver);
	if (err != 0)
		pr_err("%s:driver registration failed, error=%d \n",
			__func__, err);

	return err;
}

/**
 * tda1997x cleanup function.
 * Called on rmmod.
 *
 * @return Error code indicating success or failure.
 */
static void __exit tda1997x_clean(void)
{
	dev_dbg(&tda1997x_data.client->dev, "%s\n", __func__);
	i2c_del_driver(&tda1997x_i2c_driver);
}

module_init(tda1997x_init);
module_exit(tda1997x_clean);

MODULE_DESCRIPTION("Core driver for TDA1997X HDMI Receiver");
MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_LICENSE("GPL");
