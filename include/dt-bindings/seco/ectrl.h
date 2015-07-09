/*
 * (C) Copyright 2015 Seco srl
 *
 * Author: Davide Cardillo <davide.cardillo@seco.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 *
 * This header provides constants for the binding Seco Embedded controller
 *
 */

#ifndef __DT_BINDINGS_SECO_ECTRL_H
#define __DT_BINDINGS_SECO_ECTRL_H




#define ECTRL_EVNT_PWR_BTN      0x01
#define ECTRL_EVNT_RST_BTN      0x02
#define ECTRL_EVNT_SLEEP        0x04
#define ECTRL_EVNT_BATTERY      0x08
#define ECTRL_EVNT_LID          0x10
#define ECTRL_EVNT_WAKE         0x20



#define ECTRL_BOOTDEV_USDHC4    0x01
#define ECTRL_BOOTDEV_USDHC1    0x02
#define ECTRL_BOOTDEV_EMMC      0x04
#define ECTRL_BOOTDEV_SPI       0x08


#endif   /*  __DT_BINDINGS_SECO_ECTRL_H  */
