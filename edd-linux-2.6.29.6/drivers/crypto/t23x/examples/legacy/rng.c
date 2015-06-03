
/*
 * rng.c
 *
 * Linux specific test application, 
 *
 * Copyright (c) 2007, 2008 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <string.h>
#include <stdio.h>

#include <Sec2.h>
#include "examples.h"


#define RNG_DATASIZE (8192)

unsigned char rngBuf[RNG_DATASIZE];


#define BITSINABYTE (8)

int rndbitwt(unsigned char *buf, int size)
{
    unsigned int  totalbits, actualbits;
    unsigned int  i, j;

    totalbits = size * BITSINABYTE;
    actualbits = 0;

    for (i = 0; i < size; i++)
        for (j = 0; j < BITSINABYTE; j++)
            if ((buf[i] >> j) & 0x01)
                actualbits++;
    return((actualbits * 100) / totalbits);
}

int rngwt(int fd)
{
    RNG_REQ       rngRQ ;
    int           status;
    int           bitwt;
    unsigned long i;
    char          statmsg[32];

    /* precoat buffer so we can see where we've been */
    for (i = 0; i < RNG_DATASIZE; i++)
        rngBuf[i] = i;

    memset(&rngRQ, 0, sizeof(RNG_REQ));

    rngRQ.opId     = DPD_RNG_GETRN;
    rngRQ.rngBytes = RNG_DATASIZE;
    rngRQ.rngData  = (unsigned char *)rngBuf;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, &rngRQ);

    /* Error check before proceeding */
    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("rng", "rngwt", "", "not supported");
            return 0;
        }
        else
        {
            printf("rng: rngwt - driver error with 0x%08x\n", status);
            return -1;
        }
    }


    bitwt = rndbitwt(rngBuf, RNG_DATASIZE);
    if ((bitwt < 48) || (bitwt > 52))
    {
        sprintf(statmsg, "weight test = %dpct", bitwt);
        consolemsg("rng", "rngwt", statmsg, "error");
        return -1;
    }
    else
    {
        sprintf(statmsg, "bit density = %dpct", bitwt);
        consolemsg("rng", "rngwt", statmsg, "OK");
    }
    return 0;
}


int rng(int fd)
{
    int err, stat;

    err = 0;

    stat = rngwt(fd);
    if (stat) err++;

    if (err)
        return -1;
    else
        return 0;
}
