

/*
 * rmtest_mod.c
 *
 * Linux specific driver module initialization for t23xrm test module
 *
 * Copyright (c) 2007, 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */


#include <linux/module.h>


extern int32_t execTestSeries(void);


static int32_t __init rmtest_init(void)
{
    return execTestSeries();
}

static void rmtest_exit(void)
{
}


module_init(rmtest_init);
module_exit(rmtest_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Extensible Crypto Driver - t23x RM quick-test module");
MODULE_AUTHOR("Freescale Semiconductor - NMG/STC");
