/*
 * Name: utils.h
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-07-02
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

/*
General Description:
This module contains general purpose macros and functions:
  - uchar and uint data types
  - highbyte/lowbyte access
  - string concatenation tricks
  - binary constant interpretation
  - port pin access
*/

#ifndef __utils_h_included__
#define __utils_h_included__

#ifndef uchar
#define uchar   unsigned char
#endif

#ifndef uint
#define uint    unsigned int
#endif

#ifndef __ASSEMBLER__

typedef union{
    uint    word;
    uchar   bytes[2];
}utilWord_t;

typedef union{
    unsigned long   dword;
    uchar           bytes[4];
}utilDword_t;

/* ------------------------------------------------------------------------- */

static inline uchar utilHi8(uint x)
{
utilWord_t converter;

    converter.word = x;
    return converter.bytes[1];
}

/* ------------------------------------------------------------------------- */

#endif  /* !defined(__ASSEMBLER__) */

#define UTIL_CONCAT(a, b)           a ## b
#define UTIL_CONCAT_EXPANDED(a, b)  UTIL_CONCAT(a, b)

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

/* ------------------------------------------------------------------------- */

/* PORT bit macros */

#define UTIL_ARG1(a, b) a
#define UTIL_ARG2(a, b) b

#define UTIL_PBIT_SET(varbase, pinspec)  UTIL_CONCAT_EXPANDED(varbase, UTIL_ARG1(pinspec)) |= (1 << (UTIL_ARG2(pinspec)))
#define UTIL_PBIT_CLR(varbase, pinspec)  UTIL_CONCAT_EXPANDED(varbase, UTIL_ARG1(pinspec)) &= ~(1 << (UTIL_ARG2(pinspec)))

#define PORT_OUT(pinspec)       UTIL_CONCAT_EXPANDED(PORT, UTIL_ARG1(pinspec))
#define PORT_IN(pinspec)        UTIL_CONCAT_EXPANDED(PIN, UTIL_ARG1(pinspec))
#define PORT_DDR(pinspec)       UTIL_CONCAT_EXPANDED(DDR, UTIL_ARG1(pinspec))
#define PORT_BIT(pinspec)       UTIL_ARG2(pinspec)

#define PORT_PIN_SET(pinspec)   UTIL_CONCAT_EXPANDED(PORT, UTIL_ARG1(pinspec)) |= (1 << (UTIL_ARG2(pinspec)))
#define PORT_PIN_CLR(pinspec)   UTIL_CONCAT_EXPANDED(PORT, UTIL_ARG1(pinspec)) &= ~(1 << (UTIL_ARG2(pinspec)))
#define PORT_PIN_SWITCH(pinspec)   UTIL_CONCAT_EXPANDED(PORT, UTIL_ARG1(pinspec)) ^= (1 << (UTIL_ARG2(pinspec)))
#define PORT_PIN_VALUE(pinspec) (UTIL_CONCAT_EXPANDED(PIN, UTIL_ARG1(pinspec)) & (1 << (UTIL_ARG2(pinspec))))

#define PORT_DDR_SET(pinspec)   UTIL_CONCAT_EXPANDED(DDR, UTIL_ARG1(pinspec)) |= (1 << (UTIL_ARG2(pinspec)))
#define PORT_DDR_CLR(pinspec)   UTIL_CONCAT_EXPANDED(DDR, UTIL_ARG1(pinspec)) &= ~(1 << (UTIL_ARG2(pinspec)))
#define PORT_DDR_VALUE(pinspec) (UTIL_CONCAT_EXPANDED(DDR, UTIL_ARG1(pinspec)) & (1 << (UTIL_ARG2(pinspec))))

#define PORT_DDR_OUT(pinspec)   UTIL_CONCAT_EXPANDED(DDR, UTIL_ARG1(pinspec)) |= (1 << (UTIL_ARG2(pinspec)))
#define PORT_DDR_IN(pinspec)   UTIL_CONCAT_EXPANDED(DDR, UTIL_ARG1(pinspec)) &= ~(1 << (UTIL_ARG2(pinspec)))

/* ------------------------------------------------------------------------- */
#ifndef __ASSEMBLER__

#define nop()    asm volatile("rjmp .+0"::)

//uchar ShByte;

uchar HVgenOn(uint uPRG);
void HVgenOff();
/*
void ClkOutOn();
void ClkOutOff();
*/
void IOReset();

//uchar GetSpeedSlowJPVal();

void OutShByte(uchar data);

#define SHPORT_PIN_SET(pinspec)   {ShByte |= pinspec;OutShByte();}
#define SHPORT_PIN_CLR(pinspec)   {ShByte &= ~pinspec;OutShByte();}

#define HWPINS_HVPP_SH		(1 << PORT_BIT(HWPIN_HVPP_DT_SH))|(1 << PORT_BIT(HWPIN_HVPP_DT_IN))|(1 << PORT_BIT(HWPIN_HVPP_DT_CL))

#define HVRESET_ENABLE		PORT_DDR_OUT(HWPIN_HVP_RESET)
//deactivate reset		+12 disconnect
#define HVRESET_OFF			{PORT_PIN_CLR(HWPIN_HVP_RESET);PORT_PIN_CLR(HWPIN_HVP_HVRESET);}
//+12 disconnect		activate reset (to 0
#define HVRESET_TO_RESET	{PORT_PIN_CLR(HWPIN_HVP_HVRESET);PORT_PIN_SET(HWPIN_HVP_RESET);}
//deactivate reset		apply 12v to reset
#define HVRESET_TO_HV		{PORT_PIN_CLR(HWPIN_HVP_RESET);PORT_PIN_SET(HWPIN_HVP_HVRESET);}

//void SwitchHVReset(uchar mode);

#define HVPP_DPORT_TO_OUT {PORT_DDR(HWPIN_HVPP_DT0) |= (1 << PORT_BIT(HWPIN_HVPP_DT0))|(1 << PORT_BIT(HWPIN_HVPP_DT1))|\
	(1 << PORT_BIT(HWPIN_HVPP_DT2))|(1 << PORT_BIT(HWPIN_HVPP_DT3))|(1 << PORT_BIT(HWPIN_HVPP_DT4))|(1 << PORT_BIT(HWPIN_HVPP_DT5));\
	PORT_DDR(HWPIN_HVPP_DT6) |= (1 << PORT_BIT(HWPIN_HVPP_DT6))|(1 << PORT_BIT(HWPIN_HVPP_DT7));}
#define HVPP_DPORT_TO_IN {PORT_DDR(HWPIN_HVPP_DT0) &= ~((1 << PORT_BIT(HWPIN_HVPP_DT0))|(1 << PORT_BIT(HWPIN_HVPP_DT1))|\
	(1 << PORT_BIT(HWPIN_HVPP_DT2))|(1 << PORT_BIT(HWPIN_HVPP_DT3))|(1 << PORT_BIT(HWPIN_HVPP_DT4))|(1 << PORT_BIT(HWPIN_HVPP_DT5)));\
	PORT_DDR(HWPIN_HVPP_DT6) &= ~((1 << PORT_BIT(HWPIN_HVPP_DT6))|(1 << PORT_BIT(HWPIN_HVPP_DT7)));}

#endif

#endif /* __utils_h_included__ */
