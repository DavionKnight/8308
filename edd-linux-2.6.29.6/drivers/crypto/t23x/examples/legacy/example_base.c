
/*
 * example_base.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>

/* Mask of intended tests to run */
#define RUN_AES      0x00000001
#define RUN_DES      0x00000002
#define RUN_PK       0x00000004
#define RUN_HASH     0x00000010
#define RUN_ARC4     0x00000020
#define RUN_RNG      0x00000040
#define RUN_KASUMI   0x00000080
#define RUN_CMAC     0x00000100



int main(int argc, char **argv)
{

    int          i, fd, stat, err;
    unsigned int exmask, exknown;

    fd = open("/dev/sec2", O_RDWR, 0);
    if (fd == -1)
    {
        printf("ex_base: can't open device, error = %d: \n", errno);
        perror("");
        return -1;
    }

    /* If no extra arguments, run everything possible */
    if (argc == 1)
        exmask = 0xffffffff;
    else /* user wants to see specific modules */
    {
        exmask = 0;
        for (i = 1; i < argc; i++)
        {
            exknown = 0;
            if (!strcmp(argv[i], "aes"))    { exmask |= RUN_AES;    exknown++; }
            if (!strcmp(argv[i], "des"))    { exmask |= RUN_DES;    exknown++; }
            if (!strcmp(argv[i], "pk"))     { exmask |= RUN_PK;     exknown++; }
            if (!strcmp(argv[i], "hash"))   { exmask |= RUN_HASH;   exknown++; }
            if (!strcmp(argv[i], "arc4"))   { exmask |= RUN_ARC4;   exknown++; }
            if (!strcmp(argv[i], "rng"))    { exmask |= RUN_RNG;    exknown++; }
            if (!strcmp(argv[i], "kasumi")) { exmask |= RUN_KASUMI; exknown++; }
            if (!exknown)
                printf("secexamples: example case <%s> unknown\n", argv[i]);
        }
    }


    err = 0;

    if (exmask & RUN_AES)
    {
        stat = aes(fd);
        if (stat) err++;
    }

    if (exmask & RUN_DES)
    {
        stat = des(fd);
        if (stat) err++;
    }

    if (exmask & RUN_PK)
    {
        stat = pk(fd);
        if (stat) err++;
    }

    if (exmask & RUN_HASH)
    {
        stat = hash(fd);
        if (stat) err++;
    }

    if (exmask & RUN_ARC4)
    {
        stat = arc4(fd);
        if (stat) err++;
    }

    if (exmask & RUN_RNG)
    {
        stat = rng(fd);
        if (stat) err++;
    }

    if (exmask & RUN_KASUMI)
    {
        stat = kasumi(fd);
        if (stat) err++;
    }


    close(fd);

    if (err)
        return -1;
    else
        return 0;
}



void dumpm(unsigned char *csData, unsigned int nBytes)
{
    int nHex, nAscii;
    unsigned int nIndex;
    char csOutputBuffer[80];
    unsigned char *csLocalPtr;

    if (csData == NULL) {
        printf("00000000 Null pointer passed to dumpm()\n");
        return;
    }

    nHex = nAscii = 0;

    for (nIndex = 0; nIndex < nBytes; nIndex++) {
        if (nIndex % 16 == 0) {
            csOutputBuffer[nAscii] = 0;
            if (nHex) printf("%s\n", csOutputBuffer);
            memset(csOutputBuffer, ' ', sizeof(csOutputBuffer) - 1);
            csLocalPtr = csData + nIndex;
            sprintf(csOutputBuffer, "%08lx", csLocalPtr);
            csOutputBuffer[8] = ' ';
            nHex = 11;
            nAscii = 63;
        }

        sprintf(&csOutputBuffer[nHex], "%02x", csData[nIndex]);
        csOutputBuffer[nHex+2] = ' ';

        if (csData[nIndex] >= 0x20 && csData[nIndex] < 0x7f)
            csOutputBuffer[nAscii] = csData[nIndex];
        else
            csOutputBuffer[nAscii] = '.';

        nAscii++, nHex += 3;
        if (nIndex % 16 == 7)
            nHex += 2;
    }

    if (nHex > 11) {
        csOutputBuffer[nAscii] = 0;
        printf("%s\n", csOutputBuffer);
    }
}
