/**************************************************************************
*   adapted for MultiProg 2015 by olegvedi@gmail.com
*
*   Copyright (C) 2008 by Frans Schreuder                                 *
*   usbpicprog.sourceforge.net                                            *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
**************************************************************************/


#ifndef PROG_LOLVL_H
#define PROG_LOLVL_H


//#include "prog.h"



//#define enablePGC_D() { PORT_DDR_OUT(HWPIN_HVPIC_PGC); PORT_DDR_OUT(HWPIN_HVPIC_PGD); }
#define enablePGC_D() { PORT_DDR_OUT(HWPIN_HVPIC_PGC); PORT_PIN_CLR(HWPIN_HVPIC_PGD);}
#define disablePGC_D() { PORT_DDR_IN(HWPIN_HVPIC_PGC); PORT_DDR_IN(HWPIN_HVPIC_PGD); }

#define setPGDinput() PORT_DDR_IN(HWPIN_HVPIC_PGD)
#define setPGDoutput() PORT_DDR_OUT(HWPIN_HVPIC_PGD)

//#define PGDhigh() PORT_PIN_SET(HWPIN_HVPIC_PGD)
//#define PGDhigh() PORT_PIN_SET(HWPIN_HVPIC_PGD)
#define PGDhigh() PORT_DDR_IN(HWPIN_HVPIC_PGD)
//#define PGDlow() PORT_PIN_CLR(HWPIN_HVPIC_PGD)
#define PGDlow() PORT_DDR_OUT(HWPIN_HVPIC_PGD)

#define PGChigh() PORT_PIN_SET(HWPIN_HVPIC_PGC)
#define PGClow() PORT_PIN_CLR(HWPIN_HVPIC_PGC)

//#define enableVDD()
#define VDDon() PORT_PIN_CLR(HWPIN_HVP_iSUPPLY)
#define VDDoff() PORT_PIN_SET(HWPIN_HVP_iSUPPLY)

#define DelayMs(x) timerMsDelay(x)

//~ 0.5 us
#define Nop() asm volatile("rjmp .+0"::)


typedef union {
	struct{
		unsigned int byte0 : 8;
		unsigned int byte1 : 8;
		unsigned int byte2 : 8;
		unsigned int byte3 : 8;
	};
	struct{
		uint32_t a32b;
	};

} t32_t;

/**
    Would one Nop() cycle be enough delay for all PIC's?
    It works for PIC18F2550
*/

//#define clock_delay() {uint8_t cnt; for(cnt=0;cnt<clock_delay_cnt;cnt++)Nop();}
void clock_delay();

//#define I2C_delay() Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop()
void I2C_delay();


/**
    Sets or clears the VDD and VPP voltages.
*/
//void set_vdd_vpp(PICTYPE pictype, PICFAMILY picfamily, uint8_t level);
//void enter_ISCP( void );

/**
    Sets the address pointer to a certain address location.
*/
//void set_address(PICFAMILY picfamily, unsigned long address);
//void set_address_P16( unsigned long address );
//void set_address_P18( unsigned long address );


/**
    Reads 8 bits from a pic device with a given cmd_size bits command
    (only the 2nd byte is read).
*/
uint16_t pic_read_byte2(/*uint8_t cmd_size, uint8_t command*/);

/**
    Reads 14 bits from a pic16 device (0 payload 0).
*/
//uint pic_read_14_bits(uint8_t cmd_size, uint8_t command);
uint pic_read_14_bits();

/**
    Writes a n-bit command.
*/
void pic_send_n_bits( uint8_t cmd_size, uint8_t command );

/**
    Writes a 14 bit word with a start and a stop bit (16F devices).
*/
//void pic_send_word_14_bits(unsigned int payload);

/**
    Writes 16 bits to the PIC (18F devices).
*/
//void pic_send_word(unsigned int payload);

/**
    Writes a cmd_size bit command + 14 bit payload to a pic16 device.
*/
//void pic_send_14_bits(uint8_t cmd_size, uint8_t command, unsigned int payload);

/**
    Writes a cmd_size bit command + 16 bit payload to a pic18 device.
*/
//void pic_send(uint8_t cmd_size, uint8_t command, unsigned int payload);

/**
  reads a 16 bit word from a dsPic device using normal ICSP
*/
//uint dspic_read_16_bits(uint8_t isLV);
uint dspic_read_16_bits(void);


/**
  gives a 24 bit instruction to a dsPIC device
*/
void dspic_send_24_bits(t32_t *p);


/**
I2C Start
*/
void I2C_start(void);

/**
I2C Stop
*/
void I2C_stop(void);


/**
returns ack
*/
uint8_t I2C_write(uint8_t d);


uint8_t I2C_read(uint8_t ack);

void enter_ISCP_simple( void );
void enter_ISCP_P16_Vpp( void );
void enter_ISCP_dsPIC30( void );
void enter_ISCP_PIC18J( void );
void enter_ISCP_PIC18K( void );
void enter_ISCP_PIC24( void ) ;
void enter_ISCP_PIC24E( void ) ;
void enter_ISCP_PIC24K( void ) ;
void enter_ISCP_I2C_EE( void );
void exit_ISCP( void );

//	lastblock bits
//#define FIRST_BLOCK	0x01
//#define LAST_BLOCK	0x02
//#define CONFIG_BLOCK	0x04
#define BLOCKTYPE_FIRST	 0x01
#define BLOCKTYPE_LAST	 0x02
#define BLOCKTYPE_CONFIG 0x04

#define TABLE



typedef struct stkProgCMDIscp{
	uint8_t   stoken;//start token
	uint8_t   cs;//command_sequence
	uint8_t   sizeh;
    uint8_t   sizel;
    uint8_t   token;
    uint8_t   cmd;
    uint8_t   data;
}stkProgCMDIscp_t;

typedef struct stkReadFlashResult{
    uint8_t   status1;
    uint8_t   data[1];    // actually more than 1 byte
}stkReadFlashResult_t;

uint8_t iscpPrepareProgmode(uint *param);
void iscpLeaveProgmode(void);
uint8_t iscpRunProgmode(stkProgCMDIscp_t *param, stkReadFlashIspResult_t *result);



//----------------list of commands--------------------
//hi(5)+low(3) => 8bits - commands with his variants
//low - count of parameters > 0-7 bytes
//hi - code of command from list: (0-31)
enum{
c_nop=0,
c_enablePGC_D=5,
//c_setPGDinput=10,
//c_setPGDoutput=11,

c_PGDlow=20,
c_PGDhigh=21,
c_PGClow=22,
c_PGChigh=23,
c_VDDon=24,
c_VDDoff=25,

c_HVReset_ENABLE=30,
c_HVReset_OFF=31,
c_HVReset_TO_RESET=32,
c_HVReset_TO_HV=33,

c_clock_delay=40,
c_DelayMs=41,//+1 byte
c_DelayUs=42,//+1 byte

c_pic_send=50, // + 2-5 bytes (c_pic_send_8  c_pic_send_16  c_pic_send_24 c_pic_send_32)
c_dspic_send_24=51,//+3 bytes

c_pic_read=60,
c_pic_read_14_bits=61,
c_pic_read_byte2=62,
c_dspic_read_16_bits=63,

c_set_param=70	//+2 bytes - num param and value
};

//#define c_setPGDinput c_PGDhigh
//#define c_setPGDoutput c_PGDlow

//x<<3
#define CMD_MASK_HI 0b11111000
#define CMD_MASK_LO 0b111

// for c_set_param
enum{
	p_param_clock_delay = 0,
};

#endif// PROG_LOLVL_H
