/*
 * vt1613.h - VT1613 audio codec interface
 *
 * Copyright 2010-2015 Seco s.r.l.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef _VT1613_H
#define _VT1613_H

#define AC97_VT1613_DAC_SLOT_MAP  0x6C
#define AC97_VT1613_ADC_SLOT_MAP  0x6E 

#define AC97_VT1613_GPIO_CTRL     0x78
#define AC97_VT1613_GPIO_STATUS   0x7A

#define AC97_VT1613_STEREO_MIC    0x5C

/* 	VT1613 DAI ID's */
#define VT1613_DAI_AC97_ANALOG          0
#define VT1613_DAI_AC97_DIGITAL         1


#endif
