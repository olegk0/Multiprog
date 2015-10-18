/*
 * Name: stk500protocol.c
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-06-19
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

#include "hardware.h"
#include <avr/pgmspace.h>
#include <string.h>
#include "oddebug.h"
#include "stk500protocol.h"
#include "utils.h"
#include "isp.h"
#include "hvprog.h"
#include "timer.h"
#include "vreg.h"
#include "picprog.h"

/* The following versions are reported to the programming software: */
#define STK_VERSION_HW      1
#define STK_VERSION_MAJOR   2
#define STK_VERSION_MINOR   4

//#define BUFFER_SIZE     281 /* results in 275 bytes max body size */
#define RX_TIMEOUT      200 /* timeout in milliseconds */

#define STK_TXMSG_START 5


static uchar        rxBuffer[BUFFER_SIZE];
static uint         rxPos;
static utilWord_t   rxLen;
static uchar        rxBlockAvailable;

static uchar        txBuffer[BUFFER_SIZE];
static uint         txPos, txLen;

stkParam_t      stkParam = {{
                    0, 0, 0, 0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0, 0, 0, 0,
                    STK_VERSION_HW, STK_VERSION_MAJOR, STK_VERSION_MINOR, 0, 50, 0, 1, 0x80,
                    2, 0, 0xaa, 0, 0, 0, 0, 0,
                }};
utilDword_t     stkAddress;

/* ------------------------------------------------------------------------- */

void    stkIncrementAddress(void)
{
    stkAddress.dword++;
}

/* ------------------------------------------------------------------------- */

static void stkSetTxMessage(uint len)
{
uchar   *p = txBuffer, sum = 0;

    *p++ = STK_STX;
    *p++ = rxBuffer[1];  /* sequence number */
    *p++ = utilHi8(len);
    *p++ = len;
    *p++ = STK_TOKEN;
    txPos = 0;
    len += 6;
    txLen = len--;
    p = txBuffer;
    while(len--){
        sum ^= *p++;
    }
    *p = sum;
    DBG1(0xe0, txBuffer, txLen);
}

/* ------------------------------------------------------------------------- */

static void setParameter(uchar index, uchar value)
{
    if(index == STK_PARAM_OSC_PSCALE){
        HW_SET_T2_PRESCALER(value);
    }else if(index == STK_PARAM_OSC_CMATCH){
        OCR2 = value;
    }
    index &= 0x1f;
    stkParam.bytes[index] = value;
}

static uchar getParameter(uchar index)
{
    if(index == STK_PARAM_OSC_PSCALE)
        return HW_GET_T2_PRESCALER();
    if(index == STK_PARAM_OSC_CMATCH)
        return OCR2;
    index &= 0x1f;
    return stkParam.bytes[index];
}

/* ------------------------------------------------------------------------- */

/* Use defines for the switch statement so that we can choose between an
 * if()else if() and a switch/case based implementation. switch() is more
 * efficient for a LARGE set of sequential choices, if() is better in all other
 * cases.
 */
#if 0
#define SWITCH_START        if(0){
#define SWITCH_CASE(value)  }else if(cmd == (value)){
#define SWITCH_CASE2(v1,v2) }else if(cmd == (v1) || cmd == (v2)){
#define SWITCH_CASE3(v1,v2,v3) }else if(cmd == (v1) || cmd == (v2) || (cmd == v3)){
#define SWITCH_CASE4(v1,v2,v3,v4) }else if(cmd == (v1) || cmd == (v2) || cmd == (v3) || cmd == (v4)){
#define SWITCH_DEFAULT      }else{
#define SWITCH_END          }
#else
#define SWITCH_START        switch(cmd){{
#define SWITCH_CASE(value)  }break; case (value):{
#define SWITCH_CASE2(v1,v2) }break; case (v1): case(v2):{
#define SWITCH_CASE3(v1,v2,v3) }break; case (v1): case(v2): case(v3):{
#define SWITCH_CASE4(v1,v2,v3,v4) }break; case (v1): case(v2): case(v3): case(v4):{
#define SWITCH_DEFAULT      }break; default:{
#define SWITCH_END          }}
#endif

void stkEvaluateRxMessage(void) /* not static to prevent inlining */
{
uchar       i, cmd;
utilWord_t  len = {2};  /* defaults to cmd + error code */
void        *param;

    DBG1(0xf1, rxBuffer, rxLen.bytes[0]);
    cmd = rxBuffer[STK_TXMSG_START];
    txBuffer[STK_TXMSG_START] = cmd;
    txBuffer[STK_TXMSG_START + 1] = STK_STATUS_CMD_OK;
    param = &rxBuffer[STK_TXMSG_START + 1];
    SWITCH_START
    SWITCH_CASE(STK_CMD_SIGN_ON)
        static const PROGMEM char string[] = {8, 'S', 'T', 'K', '5', '0', '0', '_', '2', 0};
        char *p = (char *)&txBuffer[STK_TXMSG_START + 2];
        strcpy_P(p, string);
        len.bytes[0] = 11;
    SWITCH_CASE(STK_CMD_SET_PARAMETER)
        setParameter(rxBuffer[STK_TXMSG_START + 1], rxBuffer[STK_TXMSG_START + 2]);
    SWITCH_CASE(STK_CMD_GET_PARAMETER)
        txBuffer[STK_TXMSG_START + 2] = getParameter(rxBuffer[STK_TXMSG_START + 1]);
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_OSCCAL)
        txBuffer[STK_TXMSG_START + 1] = STK_STATUS_CMD_FAILED;
        /* not yet implemented */
    SWITCH_CASE(STK_CMD_LOAD_ADDRESS)
        for(i=0;i<4;i++){
            stkAddress.bytes[3-i] = rxBuffer[STK_TXMSG_START + 1 + i];
        }
    SWITCH_CASE(STK_CMD_SET_CONTROL_STACK)
        /* AVR Studio sends:
        1b 08 00 21 0e 2d 
        4c 0c 1c 2c 3c 64 74 66
        68 78 68 68 7a 6a 68 78
        78 7d 6d 0c 80 40 20 10
        11 08 04 02 03 08 04 00
        bf
        */
        /* dummy: ignore */
#if ENABLE_HVPROG
    SWITCH_CASE(STK_CMD_ENTER_PROGMODE_HVSP)
        hvspEnterProgmode(param);
    SWITCH_CASE(STK_CMD_LEAVE_PROGMODE_HVSP)
        hvspLeaveProgmode(param);
    SWITCH_CASE(STK_CMD_CHIP_ERASE_HVSP)
        txBuffer[STK_TXMSG_START + 1] = hvspChipErase(param);
    SWITCH_CASE(STK_CMD_PROGRAM_FLASH_HVSP)
        txBuffer[STK_TXMSG_START + 1] = hvspProgramMemory(param, 0);
    SWITCH_CASE(STK_CMD_READ_FLASH_HVSP)
        len.word = 1 + hvspReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 0);
    SWITCH_CASE(STK_CMD_PROGRAM_EEPROM_HVSP)
        txBuffer[STK_TXMSG_START + 1] = hvspProgramMemory(param, 1);
    SWITCH_CASE(STK_CMD_READ_EEPROM_HVSP)
        len.word = 1 + hvspReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 1);
    SWITCH_CASE(STK_CMD_PROGRAM_FUSE_HVSP)
        txBuffer[STK_TXMSG_START + 1] = hvspProgramFuse(param);
    SWITCH_CASE(STK_CMD_READ_FUSE_HVSP)
        txBuffer[STK_TXMSG_START + 2] = hvspReadFuse(param);
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_PROGRAM_LOCK_HVSP)
        txBuffer[STK_TXMSG_START + 1] = hvspProgramLock(param);
    SWITCH_CASE(STK_CMD_READ_LOCK_HVSP)
        txBuffer[STK_TXMSG_START + 2] = hvspReadLock();
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_READ_SIGNATURE_HVSP)
        txBuffer[STK_TXMSG_START + 2] = hvspReadSignature(param);
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_READ_OSCCAL_HVSP)
        txBuffer[STK_TXMSG_START + 2] = hvspReadOsccal();
        len.bytes[0] = 3;

    SWITCH_CASE(STK_CMD_ENTER_PROGMODE_PP)
        ppEnterProgmode(param);
    SWITCH_CASE(STK_CMD_LEAVE_PROGMODE_PP)
        ppLeaveProgmode(param);
    SWITCH_CASE(STK_CMD_CHIP_ERASE_PP)
        txBuffer[STK_TXMSG_START + 1] = ppChipErase(param);
    SWITCH_CASE(STK_CMD_PROGRAM_FLASH_PP)
        txBuffer[STK_TXMSG_START + 1] = ppProgramMemory(param, 0);
    SWITCH_CASE(STK_CMD_READ_FLASH_PP)
        len.word = 1 + ppReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 0);
    SWITCH_CASE(STK_CMD_PROGRAM_EEPROM_PP)
        txBuffer[STK_TXMSG_START + 1] = ppProgramMemory(param, 1);
    SWITCH_CASE(STK_CMD_READ_EEPROM_PP)
        len.word = 1 + ppReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 1);
    SWITCH_CASE(STK_CMD_PROGRAM_FUSE_PP)
        txBuffer[STK_TXMSG_START + 1] = ppProgramFuse(param);
    SWITCH_CASE(STK_CMD_READ_FUSE_PP)
        txBuffer[STK_TXMSG_START + 2] = ppReadFuse(param);
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_PROGRAM_LOCK_PP)
        txBuffer[STK_TXMSG_START + 1] = ppProgramLock(param);
    SWITCH_CASE(STK_CMD_READ_LOCK_PP)
        txBuffer[STK_TXMSG_START + 2] = ppReadLock();
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_READ_SIGNATURE_PP)
        txBuffer[STK_TXMSG_START + 2] = ppReadSignature(param);
        len.bytes[0] = 3;
    SWITCH_CASE(STK_CMD_READ_OSCCAL_PP)
        txBuffer[STK_TXMSG_START + 2] = ppReadOsccal();
        len.bytes[0] = 3;
#endif /* ENABLE_HVPROG */
    SWITCH_CASE(STK_CMD_ENTER_PROGMODE_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispEnterProgmode(param);
    SWITCH_CASE(STK_CMD_LEAVE_PROGMODE_ISP)
        ispLeaveProgmode(param);
    SWITCH_CASE(STK_CMD_CHIP_ERASE_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispChipErase(param);
    SWITCH_CASE(STK_CMD_PROGRAM_FLASH_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramMemory(param, 0);
    SWITCH_CASE(STK_CMD_READ_FLASH_ISP)
        len.word = 1 + ispReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 0);
    SWITCH_CASE(STK_CMD_PROGRAM_EEPROM_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramMemory(param, 1);
    SWITCH_CASE(STK_CMD_READ_EEPROM_ISP)
        len.word = 1 + ispReadMemory(param, (void *)&txBuffer[STK_TXMSG_START + 1], 1);
    SWITCH_CASE(STK_CMD_PROGRAM_FUSE_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramFuse(param);
    SWITCH_CASE4(STK_CMD_READ_FUSE_ISP, STK_CMD_READ_LOCK_ISP, STK_CMD_READ_SIGNATURE_ISP, STK_CMD_READ_OSCCAL_ISP)
        txBuffer[STK_TXMSG_START + 2] = ispReadFuse(param);
        txBuffer[STK_TXMSG_START + 3] = STK_STATUS_CMD_OK;
        len.bytes[0] = 4;
    SWITCH_CASE(STK_CMD_PROGRAM_LOCK_ISP)
        txBuffer[STK_TXMSG_START + 1] = ispProgramFuse(param);
    SWITCH_CASE(STK_CMD_SPI_MULTI)
        len.word = 1 + ispMulti(param, (void *)&txBuffer[STK_TXMSG_START + 1]);
//++++++++++++++++++ Pic prog  ISCP
    SWITCH_CASE(STK_CMD_PREPARE_PROGMODE_ISCP)
    	txBuffer[STK_TXMSG_START + 1] =	iscpPrepareProgmode(param);
    SWITCH_CASE(STK_CMD_LEAVE_PROGMODE_ISCP)
    	iscpLeaveProgmode();
    SWITCH_CASE(STK_CMD_RUN_ISCP)
    	len.word = 1 + iscpRunProgmode((void *)rxBuffer, (void *)&txBuffer[STK_TXMSG_START + 1]);
//++++++++++++++++++end pic prog
    SWITCH_DEFAULT  /* unknown command */
        DBG1(0xf8, 0, 0);
        txBuffer[STK_TXMSG_START + 1] = STK_STATUS_CMD_FAILED;
    SWITCH_END
    stkSetTxMessage(len.word);
}

/* ------------------------------------------------------------------------- */

void    stkSetRxChar(uchar c)
{
    if(timerLongTimeoutOccurred()){
        rxPos = 0;
    }
    if(rxPos == 0){     /* only accept STX as the first character */
        if(c == STK_STX)
            rxBuffer[rxPos++] = c;
    }else{
        if(rxPos < BUFFER_SIZE){
            rxBuffer[rxPos++] = c;
            if(rxPos == 4){     /* do we have length byte? */
                rxLen.bytes[0] = rxBuffer[3];
                rxLen.bytes[1] = rxBuffer[2];
                rxLen.word += 6;
                if(rxLen.word > BUFFER_SIZE){    /* illegal length */
                    rxPos = 0;      /* reset state */
                }
            }else if(rxPos == 5){   /* check whether this is the token byte */
                if(c != STK_TOKEN){
                    rxPos = 0;  /* reset state */
                }
            }else if(rxPos > 4 && rxPos == rxLen.word){ /* message is complete */
                uchar sum = 0;
                uchar *p = rxBuffer;
                while(rxPos){ /* decrement rxPos down to 0 -> reset state */
                    sum ^= *p++;
                    rxPos--;
                }
                if(sum == 0){   /* check sum is correct, evaluate rx message */
                    rxBlockAvailable = 1;
                }else{          /* checksum error */
                    DBG2(0xf2, rxBuffer, rxLen.word);
                    txBuffer[STK_TXMSG_START] = STK_ANSWER_CKSUM_ERROR;
                    txBuffer[STK_TXMSG_START + 1] = STK_ANSWER_CKSUM_ERROR;
                    stkSetTxMessage(2);
                }
            }
        }else{  /* overflow */
            rxPos = 0;  /* reset state */
        }
    }
    timerSetupLongTimeout(RX_TIMEOUT);
}

int stkGetTxCount(void)
{
    return txLen - txPos;
}

int stkGetTxByte(void)
{
uchar   c;

    if(txLen == 0){
        return -1;
    }
    c = txBuffer[txPos++];
    if(txPos >= txLen){         /* transmit ready */
        txPos = txLen = 0;
    }
    return c;
}

void    stkPoll(void)
{
    if(rxBlockAvailable){
        rxBlockAvailable = 0;
        stkEvaluateRxMessage();
    }
}
/*
void test()
{


	rxBuffer[0] = 0xc8;
	rxBuffer[1] = 0x01;

	iscpPrepareProgmode((void *)rxBuffer);
	timerMsDelay(50);

	rxBuffer[2] = 0;
	rxBuffer[3] = 7;

	rxBuffer[6] = 0x46;
	rxBuffer[7] = 0x00;
	rxBuffer[8] = 0xc8;
	rxBuffer[9] = 0x18;
	rxBuffer[10] = 0x29;
	rxBuffer[11] = 0x64;
	rxBuffer[12] = 0x21;

	iscpRunProgmode((void *)rxBuffer, (void *)txBuffer);

	timerMsDelay(50);

	rxBuffer[2] = 0;
	rxBuffer[3] = 48;

	rxBuffer[6] = 0x32;
	rxBuffer[7] =  0x06 ;
	rxBuffer[8] = 0x00 ;
	rxBuffer[9] = 0x2a ;
	rxBuffer[10] = 0x01 ;
	rxBuffer[11] = 0x32 ;
	rxBuffer[12] = 0x10;
	rxBuffer[13] =  0x00;
	rxBuffer[14] =  0x00;
	rxBuffer[15] =  0x2a;
	rxBuffer[16] =  0x01;
	rxBuffer[17] =  0x32;
	rxBuffer[18] = 0x06;
	rxBuffer[19] =  0x06;
	rxBuffer[20] =  0x2a;
	rxBuffer[21] =  0x01;
	rxBuffer[22] =  0x32;
	rxBuffer[23] =  0x06;
	rxBuffer[24] =  0x06;
	rxBuffer[25] =  0x2a;
	rxBuffer[26] =  0x01;
	rxBuffer[27] =  0x32;
	rxBuffer[28] =  0x06;
	rxBuffer[29] =  0x06;
	rxBuffer[30] =  0x2a;
	rxBuffer[31] =  0x01;
	rxBuffer[32] =  0x32;
	rxBuffer[33] =  0x06;
	rxBuffer[34] =  0x06;
	rxBuffer[35] =  0x2a;
	rxBuffer[36] =  0x01;
	rxBuffer[37] =  0x32;
	rxBuffer[38] =  0x06;
	rxBuffer[39] =  0x06;
	rxBuffer[40] =  0x2a;
	rxBuffer[41] =  0x01;
	rxBuffer[42] =  0x32;
	rxBuffer[43] =  0x06;
	rxBuffer[44] =  0x06;
	rxBuffer[45] =  0x2a;
	rxBuffer[46] =  0x01;
	rxBuffer[47] =  0x32;
	rxBuffer[48] =  0x06;
	rxBuffer[49] =  0x04;
	rxBuffer[50] =  0x2a;
	rxBuffer[51] =  0x01;
	rxBuffer[52] =  0x3d;

	iscpRunProgmode((void *)rxBuffer, (void *)txBuffer);
}
*/
/* ------------------------------------------------------------------------- */
