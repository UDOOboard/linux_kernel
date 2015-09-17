#include <linux/module.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
#include <linux/interrupt.h>
#endif

#ifndef __CHECKER__
#define CREATE_TRACE_POINTS
#include "trace.h"

#endif
