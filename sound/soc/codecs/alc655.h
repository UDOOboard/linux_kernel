/*
 * vt1613.h - ALC655 audio codec interface
 *
 * Copyright 2010-2015 Seco s.r.l.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 */

#ifndef _ALC655_H
#define _ALC655_H

#define AC97_ALC655_DAC_SLOT_MAP  0x6C
#define AC97_ALC655_ADC_SLOT_MAP  0x6E 

#define AC97_ALC655_GPIO_CTRL     0x78
#define AC97_ALC655_GPIO_STATUS   0x7A

#define AC97_ALC655_STEREO_MIC    0x5C

/* 	ALC655 DAI ID's */
#define ALC655_DAI_AC97_ANALOG          0
#define ALC655_DAI_AC97_DIGITAL         1


#endif
