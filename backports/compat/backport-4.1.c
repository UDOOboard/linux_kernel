/*
 * Copyright (c) 2015  Stefan Assmann <sassmann@kpanic.de>
 *
 * Backport functionality introduced in Linux 4.1.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/netdevice.h>

netdev_features_t passthru_features_check(struct sk_buff *skb,
					  struct net_device *dev,
					  netdev_features_t features)
{
	return features;
}
EXPORT_SYMBOL_GPL(passthru_features_check);
