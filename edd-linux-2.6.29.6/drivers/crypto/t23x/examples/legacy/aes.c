
/*
 * aes.c
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

#define AES_CTR_TESTSIZE (64)
#define AES_CTR_KEYSIZE  (16)



/*
 * Basic input values for comparison
 */
static const unsigned char keyIn[] =
{
  0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
  0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};


static const unsigned char counterIn[] =
{
  0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
  0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};


static const unsigned char inputBlk[] =
{
  0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
  0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
  0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
  0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
  0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
  0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
  0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
  0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10
};

/*
 * Local buffers for comparison and display
 */
static unsigned char decBuf[AES_CTR_TESTSIZE], encBuf[AES_CTR_TESTSIZE];
static unsigned char counter[AES_CTR_KEYSIZE];




int aesCTR128(int fd)
{
    AESA_CRYPT_REQ aesRQ;
    int            i, status;
    unsigned char  outCtx[AES_CTR_KEYSIZE];
    unsigned char *key, *ctx;
    unsigned char *textBuf;


    memset(&aesRQ, 0, sizeof(AESA_CRYPT_REQ));

    memset(decBuf, 0, AES_CTR_TESTSIZE);
    memset(encBuf, 0, AES_CTR_TESTSIZE);

    aesRQ.opId            = DPD_AESA_CTR_CRYPT;
    aesRQ.keyBytes        = AES_CTR_KEYSIZE;
    aesRQ.inIvBytes       = AES_CTR_KEYSIZE;
    aesRQ.outCtxBytes     = AES_CTR_KEYSIZE;
    aesRQ.inBytes         = AES_CTR_TESTSIZE;
    aesRQ.keyData         = (unsigned char *)keyIn;
    aesRQ.inIvData        = (unsigned char *)counterIn;
    aesRQ.outCtxData      = counter;
    aesRQ.inData          = (unsigned char *)inputBlk;
    aesRQ.outData         = encBuf;


    /* Make blocking encryption request */

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, &aesRQ);

    /* Error check before proceeding */
    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCTR128", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCTR128 - driver error with 0x%08x\n", status);
            return -1;
        }
    }


#ifdef SHOW_RESULTS
    printf("Key:\n");
    dumpm((unsigned char *)keyIn, AES_CTR_KEYSIZE);
    printf("Initial counter:\n");
    dumpm((unsigned char *)counterIn, AES_CTR_KEYSIZE);
    printf("Plaintext in:\n");
    dumpm((unsigned char *)inputBlk, AES_CTR_TESTSIZE);
    printf("Ciphertext out:\n");
    dumpm(encBuf, AES_CTR_TESTSIZE);
    printf("Updated counter:\n");
    dumpm(counter, AES_CTR_KEYSIZE);
#endif

    /*
     * Now run the decrypt operation. All you have to do is change the
     * input and output
     */
    aesRQ.inData  = encBuf;
    aesRQ.outData = decBuf;
    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, &aesRQ);

    /* Check for errors */
    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCTR128", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCTR128 - driver error with 0x%08x\n", status);
            return -1;
        }
    }



#ifdef SHOW_RESULTS
    printf("Key:\n");
    dumpm((unsigned char *)keyIn, AES_CTR_KEYSIZE);
    printf("Initial counter:\n");
    dumpm((unsigned char *)counterIn, AES_CTR_KEYSIZE);
    printf("Cleartext (decrypted) out:\n");
    dumpm(decBuf, AES_CTR_TESTSIZE);
    printf("Updated counter:\n");
    dumpm(counter, AES_CTR_KEYSIZE);
#endif

   /*
    * Compare the decrypted value with the initial cleartext
    * so we know it worked
    */

    if (memcmp(decBuf, inputBlk, AES_CTR_TESTSIZE) == 0)
    {
        consolemsg("aes", "aesctr", "", "OK");
        return 0;
    }
    else
    {
        consolemsg("aes", "aesctr", "", "error");
        return -1;
    }

    return status;
}




#define AES_TESTSIZE (32)
#define AES_KEYSIZE  (16)

static const unsigned char keyData[] = {
    0x01, 0x23, 0x45, 0x67, 0x89, 0xab, 0xcd, 0xef,
    0xf0, 0xe1, 0xd2, 0xc3, 0xb4, 0xa5, 0x96, 0x87
};

static const unsigned char AesaData[] =
{"Now is the time for all good men"}; /* 32 bytes */

static unsigned char decryptBuf[AES_TESTSIZE], encryptBuf[AES_TESTSIZE];


int aesECB(int fd)
{
    AESA_CRYPT_REQ aesaencReq;
    AESA_CRYPT_REQ aesadecReq;
    int            status;

    memset(decryptBuf, 0, AES_TESTSIZE);
    memset(encryptBuf, 0, AES_TESTSIZE);

    memset(&aesaencReq, 0, sizeof(aesaencReq));
    memset(&aesadecReq, 0, sizeof(aesadecReq));


    /* encrypt cycle */
    aesaencReq.opId     = DPD_AESA_ECB_ENCRYPT_CRYPT;
    aesaencReq.keyBytes = AES_KEYSIZE;
    aesaencReq.keyData  = (unsigned char *)keyData;
    aesaencReq.inBytes  = AES_TESTSIZE;
    aesaencReq.inData   = (unsigned char *)AesaData;
    aesaencReq.outData  = encryptBuf;


    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesaencReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesECB", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesECB - driver error with 0x%08x\n", status);
            return -1;
        }
    }



    /* decrypt cycle */
    aesadecReq.opId     = DPD_AESA_ECB_DECRYPT_CRYPT;
    aesadecReq.keyBytes = AES_KEYSIZE;
    aesadecReq.keyData  = (unsigned char *)keyData;
    aesadecReq.inBytes  = AES_TESTSIZE;
    aesadecReq.inData   = encryptBuf;
    aesadecReq.outData  = decryptBuf;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesadecReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesECB", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesECB - driver error with 0x%08x\n", status);
            return -1;
        }
    }




    /* run results comparison */
    if ((memcmp(AesaData, decryptBuf, AES_TESTSIZE)) == 0)
    {
        consolemsg("aes", "aesECB", "", "OK");
        status = 0;
    }
    else
    {
        consolemsg("aes", "aesECB", "", "error");
        status = -1;
    }

    return status;
}





int aesCBC(int fd)
{
    AESA_CRYPT_REQ aesaencReq;
    AESA_CRYPT_REQ aesadecReq;
    unsigned char  aesCtxOut[AES_KEYSIZE];
    unsigned char  iv_in[AES_KEYSIZE] = {0xfe,0xdc,0xba,0x98,0x76,0x54,0x32,0x10,
                                         0xfe,0xdc,0xba,0x98,0x76,0x54,0x32,0x10};
    int            status;

    memset(decryptBuf, 0, AES_TESTSIZE);
    memset(encryptBuf, 0, AES_TESTSIZE);

    memset(&aesaencReq, 0, sizeof(aesaencReq));
    memset(&aesadecReq, 0, sizeof(aesadecReq));


    /* encrypt cycle */
    aesaencReq.opId            = DPD_AESA_CBC_ENCRYPT_CRYPT;
    aesaencReq.inIvBytes       = AES_KEYSIZE;
    aesaencReq.inIvData        = (unsigned char *)iv_in;
    aesaencReq.keyBytes        = AES_KEYSIZE;
    aesaencReq.keyData         = (unsigned char *)keyData;
    aesaencReq.inBytes         = AES_TESTSIZE;
    aesaencReq.inData          = (unsigned char *)AesaData;
    aesaencReq.outData         = encryptBuf;
    aesaencReq.outCtxBytes     = AES_KEYSIZE;
    aesaencReq.outCtxData      = (unsigned char *)aesCtxOut;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesaencReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCBC", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCBC - driver error with 0x%08x\n", status);
            return -1;
        }
    }


    /* decrypt cycle */
    aesadecReq.opId            = DPD_AESA_CBC_DECRYPT_CRYPT;
    aesadecReq.inIvBytes       = AES_KEYSIZE;
    aesadecReq.inIvData        = (unsigned char *)iv_in;
    aesadecReq.keyBytes        = AES_KEYSIZE;
    aesadecReq.keyData         = (unsigned char *)keyData;
    aesadecReq.inBytes         = AES_TESTSIZE;
    aesadecReq.inData          = encryptBuf;
    aesadecReq.outData         = decryptBuf;
    aesadecReq.outCtxBytes     = AES_KEYSIZE;
    aesadecReq.outCtxData      = (unsigned char *)aesCtxOut;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesadecReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCBC", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCBC - driver error with 0x%08x\n", status);
            return -1;
        }
    }


    /* compare results */
    if ((memcmp(AesaData, decryptBuf, AES_TESTSIZE)) == 0)
    {
        consolemsg("aes", "aesCBC", "", "OK");
        status = 0;
    }
    else
    {
        consolemsg("aes", "aesCBC", "", "error");
        status = -1;
    }

    return status;
}



int aesCTR(int fd)
{
    AESA_CRYPT_REQ aesaencReq;
    AESA_CRYPT_REQ aesadecReq;
    int            status;

    memset(decryptBuf, 0, AES_TESTSIZE);
    memset(encryptBuf, 0, AES_TESTSIZE);

    memset(&aesaencReq, 0, sizeof(aesaencReq));
    memset(&aesadecReq, 0, sizeof(aesadecReq));


    aesaencReq.opId            = DPD_AESA_CTR_CRYPT;
    aesaencReq.keyBytes        = AES_KEYSIZE;
    aesaencReq.keyData         = (unsigned char *)keyData;
    aesaencReq.inBytes         = AES_TESTSIZE;
    aesaencReq.inData          = (unsigned char *)AesaData;
    aesaencReq.outData         = encryptBuf;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesaencReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCTR", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCTR - driver error with 0x%08x\n", status);
            return -1;
        }
    }

    aesadecReq.opId            = DPD_AESA_CTR_CRYPT;
    aesadecReq.keyBytes        = AES_KEYSIZE;
    aesadecReq.keyData         = (unsigned char *)keyData;
    aesadecReq.inBytes         = AES_TESTSIZE;
    aesadecReq.inData          = encryptBuf;
    aesadecReq.outData         = decryptBuf;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesadecReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCTR", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCTR - driver error with 0x%08x\n", status);
            return -1;
        }
    }


    if ((memcmp(AesaData, decryptBuf, AES_TESTSIZE)) == 0)
    {
        consolemsg("aes", "aesCTR", "", "OK");
        status = 0;
    }
    else
    {
        consolemsg("aes", "aesCTR", "", "error");
        status = -1;
    }

    return status;
}





#define AES_XCBCMAC_KEYSIZE    16
#define AES_XCBCMAC_TESTSIZE   32
#define AES_XCBCMAC_DIGESTSIZE 16


static const unsigned char aesxcbcmac_key[] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f
};

static const unsigned char aesxcbcmac_msg[] =
{
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f
};

static const unsigned char aesxcbcmac_digest[] =
{
    0xf5, 0x4f, 0x0e, 0xc8, 0xd2, 0xb9, 0xf3, 0xd3,
    0x68, 0x07, 0x73, 0x4b, 0xd5, 0x28, 0x3f, 0xd4
};


int aesXCBCMAC(int fd)
{
    AESA_MAC_REQ  aesmacReq;
    int           status;
    unsigned char digestBuf[AES_XCBCMAC_DIGESTSIZE];

    memset(digestBuf,  0, AES_XCBCMAC_DIGESTSIZE);
    memset(&aesmacReq, 0, sizeof(aesmacReq));

    aesmacReq.opId        = DPD_AESA_XCBCMAC;
    aesmacReq.keyBytes    = AES_XCBCMAC_KEYSIZE;
    aesmacReq.keyData     = (unsigned char *)aesxcbcmac_key;
    aesmacReq.inBytes     = AES_XCBCMAC_TESTSIZE;
    aesmacReq.inData      = (unsigned char *)aesxcbcmac_msg;
    aesmacReq.cmpBytes    = AES_XCBCMAC_DIGESTSIZE;
    aesmacReq.cmpOut      = digestBuf;

    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aesmacReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesXCBCMAC", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesXCBCMAC - driver error with 0x%08x\n", status);
            return status;
        }
    }


    if ((memcmp(aesxcbcmac_digest, digestBuf, AES_XCBCMAC_DIGESTSIZE)) == 0)
    {
        consolemsg("aes", "aesXCBCMAC", "", "OK");
        status = 0;
    }
    else
    {
        consolemsg("aes", "aesXCBCMAC", "", "error");
        status = -1;
    }

    return status;
}




#define AES_CMAC_DIGESTSIZE  2
#define AES_CMAC_KEYSIZE    16
#define AES_CMAC_TESTSIZE   16

static const unsigned char aescmac_key[] =
{
    0x66, 0xa9, 0xf8, 0xf0, 0xcd, 0x9b, 0xf0, 0xdf,
    0xbc, 0xd8, 0x5b, 0x83, 0x66, 0x5c, 0x8e, 0xeb
};

static const unsigned char aescmac_msg[] =
{
    0xf9, 0x46, 0xf3, 0x21, 0x27, 0xd5, 0x37, 0xc3,
    0x3b, 0xee, 0x31, 0x41, 0xb5, 0xdb, 0x96, 0xd1
};

static const unsigned char aescmac_digest[] =
{
    0x30, 0x9c
};


int aesCMAC(int fd)
{
    AESA_MAC_REQ  aescmacReq;
    int           status;
    unsigned char digestBuf[AES_CMAC_DIGESTSIZE];

    memset(digestBuf,   0, AES_CMAC_DIGESTSIZE);
    memset(&aescmacReq, 0, sizeof(aescmacReq));

    aescmacReq.opId        = DPD_AESA_CMAC;
    aescmacReq.keyBytes    = AES_CMAC_KEYSIZE;
    aescmacReq.keyData     = (unsigned char *)aescmac_key;
    aescmacReq.inBytes     = AES_CMAC_TESTSIZE;
    aescmacReq.inData      = (unsigned char *)aescmac_msg;
    aescmacReq.cmpBytes    = AES_CMAC_DIGESTSIZE;
    aescmacReq.cmpOut      = digestBuf;


    status = ioctl(fd, IOCTL_PROC_REQ_BLOCK_VIRTUAL, (int)&aescmacReq);

    if (status)
    {
        if ((status == SEC2_INVALID_CHA_TYPE) ||
            (status == SEC2_CHA_ERROR))
        {
            consolemsg("aes", "aesCMAC", "", "not supported");
            return 0;
        }
        else
        {
            printf("aes: aesCMAC - driver error with 0x%08x\n", status);
            return status;
        }
    }


    if ((memcmp(aescmac_digest, digestBuf, AES_CMAC_DIGESTSIZE)) == 0)
    {
        consolemsg("aes", "aesCMAC", "", "OK");
        status = 0;
    }
    else
    {
        consolemsg("aes", "aesCMAC", "", "error");
        status = -1;
    }

    return status;
}





int aes(int fd)
{
    int err, stat;

    err = 0;
    stat = aesCTR128(fd);
    if (stat) err++;

    stat = aesECB(fd);
    if (stat) err++;

    stat = aesCBC(fd);
    if (stat) err++;

    stat = aesCTR(fd);
    if (stat) err++;

    stat = aesXCBCMAC(fd);
    if (stat) err++;

    stat = aesCMAC(fd);
    if (stat) err++;
    
    if (err)
        return -1;
    else
        return 0;

}
