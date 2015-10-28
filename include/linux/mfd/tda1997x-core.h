/*
 * tda1997x.h - header for TDA1997X HDMI receiver device
 *
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
 */

#ifndef __TDA1997X_H_
#define __TDA1997X_H_

#include <linux/types.h>

/*
 * Video Port Configuration:
 *
 * 9 byte registers describe the video port output bit mapping (which bits from
 * the video data stream output on which pins). Each register controls a
 * 'pin group' of 4 bits (nibble) of the internal 36bit video data bus
 * with the following bit descriptions:
 *
 *   7 - vp_out - video port output enable: 1-active
 *   6 - vp_hiz - video port output level when not used: 1-HI-Z, 0-Low
 *   5 - unused
 *   4 - vp_swap - Swap bit allocation: 1-swap
 * 3:0 - vp_sel  - Select nibble to be routed to this pin-group 
 *    00: D[03:00]
 *    01: D[07:04]
 *    02: D[03:08]
 *    03: D[07:12]
 *    04: D[11:16]
 *    05: D[15:20]
 *    06: D[19:24]
 *    07: D[23:28]
 *    08: D[27:32]
 *    0a: D[31:35]
 *
 * The pingroups differ per chip and are as follows:
 *
 * Byte  Register      TDA19971(24bit)     TDA19972(36bit)
 *  1    VP35_32_CTRL  VP[24:20] Group8    VP[35:32] Group8
 *  2    VP31_28_CTRL  VP[19:16] Group7    VP[31:28] Group7
 *  3    VP27_24_CTRL            N/A       VP[27:24] Group6
 *  4    VP23_20_CTRL  VP[15:12] Group5    VP[23:20] Group5
 *  5    VP19_16_CTRL  VP[11:08] Group4    VP[19:16] Group4
 *  6    VP15_12_CTRL            N/A       VP[15:12] Group3
 *  7    VP11_08_CTRL  VP[07:04] Group2    VP[11:08] Group2
 *  8    VP07_04_CTRL  VP[03:00] Group1    VP[07:04] Group1
 *  9    VP03_00_CTRL            N/A       VP[03:00] Group0
 *
 * Note that the internal 36bit video bus is aligned to the top of the bus,
 * for example 8bit CCIR656 will be output on D[24:16] internally.
 *
 * If you have a TDA19971 (24bit output) pins VP[0:8] connected to
 * an Soc with an 8bit video port (ie for 8bit CCIR656 data) you would
 * assign the 9 registers as follows to map D[24:16] to VP[0:8].
 *                   
 * video_out_port = {
 *   0x00, // VP35_32_CTRL
 *   0x00, // VP31_28_CTRL
 *   0x00, // VP27_24_CTRL
 *   0x82, // VP23_18_CTRL: vp_out=1 vp_hiz=0 vp_swap=0 vp_sel=2 (Group2)
 *   0x81, // VP19_16_CTRL: vp_out=1 vp_hiz=0 vp_swap=0 vp_sel=1 (Group8)
 *   0x00, // VP15_12_CTRL
 *   0x00, // VP11_08_CTRL
 *   0x00, // VP07_04_CTRL
 *   0x00, // VP03_00_CTRL
 * };
 */

/* Video output clock modes */
typedef enum {
	CLOCK_SINGLE_EDGE,
	CLOCK_DUAL_EDGE,
	CLOCK_SINGLE_EDGE_TOGGLED,
	CLOCK_DUAL_EDGE_TOGGLED,
} tda1997x_videoclkmode_t;

/* Video output data formats */
typedef enum {
	VIDEOFMT_444         = 0x00,  /* RGB444/YUV444 */
	VIDEOFMT_422_SMP     = 0x01,  /* YUV422 semi-planar */
	VIDEOFMT_422_CCIR    = 0x02   /* YUV422 CCIR656 */
} tda1997x_videofmt_t;

/* HS/HREF signal source */
typedef enum
{
	SYNCOUTPUT_HSYNC_VHREF = 0x00, /* HS from VHREF - do not use (not programmed) */
	SYNCOUTPUT_HREF_VHREF  = 0x01, /* HREF from VHREF */
	SYNCOUTPUT_HREF_HDMI   = 0x02, /* HREF from HDMI */
} tda1997x_sync_output_hs_t;

/* VS/VREF signal source */
typedef enum
{
	SYNCOUTPUT_VSYNC_VHREF = 0x00, /* VS from VHREF - do not use (not programmed) */
	SYNCOUTPUT_VREF_VHREF  = 0x01, /* VREF from VHREF */
	SYNCOUTPUT_VREF_HDMI   = 0x02, /* VREF from HDMI */
} tda1997x_sync_output_vs_t;

/* DE/FREF signal source */
typedef enum
{
	SYNCOUTPUT_DE_VHREF    = 0x00, /* DE from VHREF (HREF and not VREF) */
	SYNCOUTPUT_FREF_VHREF  = 0x01, /* FREF from VHREF */
	SYNCOUTPUT_FREF_HDMI   = 0x02, /* FREF from HDMI */
} tda1997x_sync_output_de_t;

/* video details */
typedef struct {
	/* video input data (input to the HDMI receiver) */
	int	width;
	int height;
	int fps;
	bool interlaced;
	bool signal;

	/* video output data (output from the HDMI receiver) */
	tda1997x_videofmt_t sensor_vidfmt;
	tda1997x_videoclkmode_t sensor_clkmode;
} tda1997x_vidout_fmt_t;

/** Obtain current video format details from core */
int tda1997x_get_vidout_fmt(tda1997x_vidout_fmt_t *);

/*
 * Audio samples can be output in either S/PDIF or I2S bus formats.
 * In I2S mode, the TDF1997X is  the master with 16bit or 32bit per word.
 * In either modes, up to 8 audio channels can be controlled using the audio
 * port pins AP0 to AP3 and A_WS.  The audio port mapping depends on the
 * channel allocation, layout, and audio input type.
 *
 * The following table shows the audio port pin usage for the various modes
 * possible (pins missing should be unconnected)
 *
 *       |  SPDIF  |  SPDIF  |   I2S   |   I2S   |         HBR demux
 *       | Layout0 | Layout1 | Layout0 | Layout1 | SPDIF      | I2S  
 * ------+---------+---------+---------+---------+------------+------------
 * A_WS  | WS      | WS      | WS      | WS      | WS         | WS
 * AP3   |         | SPDIF3  |         | SD3     | SPDIF[x+3] | SD[x+3]
 * AP2   |         | SPDIF2  |         | SD2     | SPDIF[x+2] | SD[x+2]
 * AP1   |         | SPDIF1  |         | SD1     | SPDIF[x+1] | SD[x+1]
 * AP0   |         | SPDIF0  |         | SD0     | SPDIF[x]   | SD[x]
 * A_CLK | (32*Fs) | (32*Fs) |(32*Fs)  | (32*Fs) | (32*FsACR) | (32*FsACR)
 *       | (64*Fs) | (64*Fs) |(64*Fs)  | (64*Fs) | (64*FsACR) | (64*FsACR)
 *
 * Freq(Sysclk) = 2*freq(Aclk)
 */
typedef enum {
	AUDIO_LAYOUT_FORCED_0 = 0x00, /* Layout dictated by packet header? */
	AUDIO_LAYOUT_FORCED_1 = 0x01, /* layout1? */
	AUDIO_LAYOUT_FORCED   = 0x02, /* layout0? */
} tda1997x_audiolayout_t;

/* Audio output data formats */
typedef enum {
	AUDIO_FMT_I2S16,              /* I2S 16 bit */
	AUDIO_FMT_I2S32,              /* I2S 32 bit */
	AUDIO_FMT_SPDIF,              /* SPDIF */
	AUDIO_FMT_OBA,                /* One Bit Audio */
	AUDIO_FMT_I2S16_HBR_STRAIGHT, /* HBR straight in I2S 16bit mode */
	AUDIO_FMT_I2S16_HBR_DEMUX,    /* HBR demux in I2S 16bit mode */
	AUDIO_FMT_I2S32_HBR_DEMUX,    /* HBR demux in I2S 32bit mode */
	AUDIO_FMT_SPDIF_HBR_DEMUX,    /* HBR demux in SPDIF mode */
	AUDIO_FMT_DST,                /* Direct Stream Transfer */
} tda1997x_audiofmt_t;

/* Audio output clock frequencies */
typedef enum
{
	AUDIO_SYSCLK_128FS       = 0x03,
	AUDIO_SYSCLK_256FS       = 0x04,
	AUDIO_SYSCLK_512FS       = 0x05,
} tda1997x_audiosysclk_t;

/* Audio output info */
typedef struct {
	int samplerate;
	int channels;
	int samplesize;
} tda1997x_audout_fmt_t;

/** Obtain current audio format details from core */
int tda1997x_get_audout_fmt(tda1997x_audout_fmt_t *);

/* possible states of the state machine */
typedef enum
{
	STATE_NOT_INITIALIZED, /* Driver is not initialized */
	STATE_INITIALIZED,     /* Driver is initialized */
	STATE_UNLOCKED,        /* Driver is not locked on input signal */
	STATE_LOCKED,          /* Driver is locked on input signal */
	STATE_CONFIGURED       /* Driver is configured */
} tda1997x_state_t;

extern tda1997x_state_t tda1997x_get_state(void);

/* HDMI Inputs:
 *  TDA19971: HDMI-A (single input)
 *  TDA19972: HDMI-A|B (dual input)
 */
typedef enum
{
	INPUT_HDMI_A,
	INPUT_HDMI_B,        /* TDA19972 only */
	INPUT_AUTO_DIGITAL,  /* TDA19972 only */
} tda1997x_input_t;

extern int tda1997x_select_input(tda1997x_input_t);

#endif /* End of __TDA1997X_H */
