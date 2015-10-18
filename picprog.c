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

#include "hardware.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include "timer.h"
#include "utils.h"
#include "hvprog.h"
#include "serial.h"
#include "picprog.h"
#include "vreg.h"


#define I2C_delay()	TIMER_US_DELAY(1)		// approx 2x 1.3us min  -> ~5us

uint8_t clock_delay_cnt=0;
//0 - 2us
//1 - 3us
//10 - 8us = 0.6
//50 -30us = 0.8

/*
unsigned uint8_t ConfigDisableVDD=0;
unsigned uint8_t ConfigLimitVPP=0;
unsigned uint8_t ConfigLimitPGDPGC=0;
*/
void clock_delay()
{
	uint8_t cnt;
//	uint8_t n;
	for(cnt=0;cnt<clock_delay_cnt;cnt++)
//		for(n=0;n<10;n++)
			Nop();
}

uint8_t iscpPrepareProgmode(uint *param)
{
	uint8_t ret;
	ret = HVgenOn(PRG13V);
//	ret = HVgenOn(*param);

/*	enablePGC_D(); //PGC/D output & PGC/D_LOW appropriate
	PGDlow(); // initial value for programming mode
	PGClow(); // initial value for programming mode
	HVRESET_ENABLE;
	HVRESET_TO_RESET;
*/
	return ret;
}


void iscpLeaveProgmode(void)
{
	HVgenOff();

	HVRESET_OFF;//VPPoff(); //low, (inverted)
	//VPP_RUNoff();
	HVRESET_TO_RESET;//VPP_RSTon(); //hard reset, low (inverted)
	DelayMs( 40 );
	HVRESET_OFF;//VPP_RSToff(); //hard reset, high (inverted)
	VDDoff();
//	disablePGC_D();
	IOReset();
	DelayMs( 200 );
}


//-------------------------------WRITE-----------------------------------

//Writes a n-bit command < 8 bit
void pic_send_n_bits( uint8_t cmd_size, uint8_t command )
{
	uint8_t i;
//	enablePGC_D()

//	PGClow();
//	PGDlow();
	for( i = 0; i < cmd_size; i++ )
	{
		if( command & 1 )
			PGDhigh();
		else
			PGDlow();
		PGChigh();
		command >>= 1;
		clock_delay();
		PGClow();
		clock_delay();
	}
	clock_delay();
	PGDlow();//?
//	for( i = 0; i < 10; i++ )
//		continue; //wait at least 1 us <<-- this could be tweaked to get the thing faster
}

//#define pic_send_word(x) pic_send_n_bits(16, x)
/*
// Writes a n-bit command + 14 bit payload to a pic16 device
void pic_send_14_bits( uint8_t cmd_size, uint8_t command, unsigned int payload )
{
	pic_send_n_bits( cmd_size, command );
	pic_send_word_14_bits( payload );
	PGDlow(); //  <=== Must be low at the end, at least when VPP and VDD go low.
}
*/
void send_clock( uint8_t count )
{
	while(count)
	{
		PGChigh();
		clock_delay();
		PGClow();
		clock_delay();
		count--;
	}
}

void dspic_send_24_bits( t32_t *p )
{

    PGDlow();
    /*
    for( i = 0; i < 4; i++ )
    {
        PGChigh();
        clock_delay();
        PGClow();
    }
    */
    send_clock(4);

    pic_send_n_bits( 8, p->byte0 );

    PGDlow();

    pic_send_n_bits( 8, p->byte1 );

    PGDlow();

    pic_send_n_bits( 8, p->byte2 );

    PGDlow();

}
//-----------------------READ---------------------------
uint16_t pic_read( uint8_t size) // 1 to 2 bytes
{
	uint8_t i;
	uint16_t result;

	result = 0;
	for( i = 0; i < size; i++ )
	{

/*		PGChigh();
		clock_delay();
		result |= ((uint) PORT_PIN_VALUE(HWPIN_HVPIC_PGD)) << i;
		clock_delay();
		PGClow();
*/
		PGChigh();
		clock_delay();
		PGClow();
		result |= ((uint) PORT_PIN_VALUE(HWPIN_HVPIC_PGD)) << i;
		clock_delay();

//		clock_delay();
	}

	return result;
}

//uint16_t pic_read_14_bits( uint8_t cmd_size, uint8_t command )
uint16_t pic_read_14_bits()
{
//	uint8_t i;
	uint16_t result;
//	pic_send_n_bits( cmd_size, command );
	//for(i=0;i<80;i++)continue;	//wait at least 1us
	///PIC10 only...

	setPGDinput(); //PGD = input
//	for( i = 0; i < 10; i++ )
//		continue;
	clock_delay();

	PGChigh();
	clock_delay();
	PGClow();
	clock_delay();

	result = pic_read(14);

	PGChigh();
	clock_delay();
	PGClow();
	clock_delay();
	setPGDoutput();
	clock_delay();
	return result;
}


// reads 8 bits from a pic device with a given cmd_size bits command

uint16_t pic_read_byte2(/* uint8_t cmd_size, uint8_t command */)
{
//	uint8_t i;
	uint8_t result;
//	pic_send_n_bits( cmd_size, command );
	//	for(i=0;i<80;i++)continue;	//wait at least 1us
//	for( i = 0; i < 8; i++ )
//	{
//		PGDlow();
//		clock_delay();
//		PGChigh();
//		clock_delay();
//		PGClow();
//		clock_delay();
//	}
	PGDlow();
	send_clock(8);

	setPGDinput();

	clock_delay();
//	for( i = 0; i < 10; i++ )
//		continue;

	result = pic_read(8);

	setPGDoutput();
	clock_delay();
	return result;
}

/// read a 16 bit "word" from a dsPIC
//uint dspic_read_16_bits( uint8_t isLV )
uint16_t dspic_read_16_bits(void)
{
//	uint8_t i;
	uint16_t result;

	PGDlow();
	PGDhigh(); //send 1
	PGChigh(); //clock pulse
	clock_delay();
	PGClow();
	PGDlow(); //send 3 zeroes
/*	for( i = 0; i < 3; i++ )
	{
		PGChigh();
		clock_delay();
		PGClow();
		clock_delay();
	}
*/
	send_clock(3);
//	result = 0;
/*	for( i = 0; i < 8; i++ )
	{
		PGChigh();
		clock_delay();
		PGClow();
		clock_delay();
	}
*/
	send_clock(8);

	setPGDinput();
	clock_delay();

	result = pic_read(16);

	setPGDoutput();
	PGDlow();
	return result;
}


//----------------I2C-----------------
/*
void I2C_start( void )
{
	//initial condition
	PGDhigh();
	I2C_delay();
	PGChigh();
	I2C_delay();
	PGDlow();
	I2C_delay();
	PGClow();
	I2C_delay();
}

void I2C_stop( void )
{
	PGDlow();
	I2C_delay();
	PGChigh();
	I2C_delay();
	PGDhigh();
	I2C_delay();
}

unsigned uint8_t I2C_write( unsigned uint8_t d )
{
	unsigned uint8_t i, j;
	j = d;
	for( i = 0; i < 8; i++ )
	{
		if( (j & 0x80) == 0x80 )
			PGDhigh();
		else
			PGDlow();
		j <<= 1;
		I2C_delay();
		PGChigh();
		I2C_delay();
		PGClow();
		I2C_delay();
	}
	setPGDinput();
	PGChigh();
	I2C_delay();
	i = (unsigned uint8_t) PGD_READ;
	PGClow();
	I2C_delay();
	setPGDoutput();
	return i;
}

unsigned uint8_t I2C_read( unsigned uint8_t ack ) {
    unsigned uint8_t i, d;
    setPGDinput();
	TRISPGD_LOW = 0;	// going to use the 1K res as a pull-up
	PGD_LOW = 1;

    d = 0;
    for( i = 0; i < 8; i++ ) {
        PGChigh();
        I2C_delay();
        d <<= 1;
        if( PGD_READ )
            d |= 0x01;
        PGClow();
        I2C_delay();
    }
	TRISPGD_LOW = 1;	// remove the pull-up
	PGD_LOW = 0;
    setPGDoutput();
    I2C_delay();
    if( ack == 1 )
        PGDhigh();
        else PGDlow();
    PGChigh();
    I2C_delay();
    PGClow();
    I2C_delay();
    return d;
}
*/
//************************** read block******************************
/*
void read_data_dsPIC30( uint32_t address, uint8_t* data, uint8_t blocksize, uint8_t lastblock )
{
	uint16_t payload;
	uint8_t i, blockcounter = 0;

	for( blockcounter = 0; blockcounter < blocksize; blockcounter += 8 )
	{
		//Step 3: Initialize the write pointer (W7) and store the next four locations of code memory to W0:W5.
		dspic_send_24_bits( 0xEB0380 ); //CLR W7
		dspic_send_24_bits( 0x000000 ); //NOP
		for( i = 0; i < 4; i++ )
		{
			dspic_send_24_bits( 0xBA1BB6 ); //TBLRDL [W6++], [W7++]
			dspic_send_24_bits( 0x000000 ); //NOP
			dspic_send_24_bits( 0x000000 ); //NOP
		}
		//Step 4: Output W0:W5 using the VISI register and REGOUT command.
		for( i = 0; i < 4; i++ )
		{
			dspic_send_24_bits( 0x883C20 | (uint32_t) i ); //MOV W0, VISI
			dspic_send_24_bits( 0x000000 ); //NOP
			payload = dspic_read_16_bits(); //VISI
			data[blockcounter + (i * 2)] = (uint8_t) (payload & 0xFF);
			data[blockcounter + ((i * 2) + 1)] = (uint8_t) ((payload & 0xFF00) >> 8);
			dspic_send_24_bits( 0x000000 ); //NOP
		}
		//Step 5: Reset device internal PC.
		dspic_send_24_bits( 0x040100 ); //GOTO 0x100
		dspic_send_24_bits( 0x000000 ); //NOP
	}
}

void read_data_PIC18( uint32_t address, uint8_t* data, uint8_t blocksize, uint8_t lastblock )
{
	uint8_t blockcounter = 0;
//	uint16_t d;

	for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		pic_send( 4, 0x00, (0x0E00 | ((address + (uint) blockcounter) & 0xFF)) ); //MOVLW Addr [7:0]
		pic_send( 4, 0x00, 0x6EA9 ); //MOVWF EEADR
		pic_send( 4, 0x00, (0x0E00 | (((address + (uint) blockcounter) >> 8) & 0xFF)) ); //MOVLW Addr [15:8]
		pic_send( 4, 0x00, 0x6EAA ); //MOVWF EEADRH
		pic_send( 4, 0x00, 0x80A6 ); //BSF EECON1, RD
		pic_send( 4, 0x00, 0x50A8 ); //MOVF EEDATA, W, 0
		pic_send( 4, 0x00, 0x6EF5 ); //MOVWF TABLAT
		pic_send( 4, 0x00, 0x0000 ); //Nop
		*(data + blockcounter) = pic_read_byte2( 4, 0x02 );
	}
}
void read_data_PIC16( uint32_t address, uint8_t* data, uint8_t blocksize, uint8_t lastblock )
{
	uint8_t blockcounter = 0;

	for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		data[blockcounter] = (uint8_t) pic_read_14_bits( 6, 0x05 ); //read data memory
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
void read_data_P16F18XX( uint32_t address, uint8_t* data, uint8_t blocksize, uint8_t lastblock )
{
	uint8_t blockcounter = 0;

	for( blockcounter = 0; blockcounter < blocksize; blockcounter++ )
	{
		data[blockcounter] = (uint8_t) pic_read_14_bits( 6, 0x05 ); //read data memory
		pic_send_n_bits( 6, 0x06 ); //increment address
	}
}
*/
//---------------------------------Run-------------------------------



uint8_t iscpRunProgmode(stkProgCMDIscp_t *param, stkReadFlashIspResult_t *result)
{
	uint8_t /*args,*/t8,retWords=0,cmd;
	uint8_t *pCmd;
	uint16_t *pRes;
//	utilWord_t  numBytes;
	t32_t	t32;

	pRes = (void *)&result->data;
	result->status1 = STK_STATUS_CMD_OK;
	pCmd = &param->data;
	param->sizel--;// - cmd byte
	PORT_PIN_CLR(HWPIN_LED);
	while((pCmd - &param->data) < param->sizel){
		wdt_reset();
//		args = (*pCmd)&CMD_MASK_LO;
		cmd = *pCmd;
		pCmd++;//pointer to next byte (next cmd or arg)
//		switch((*pCmd++)/*&CMD_MASK_HI*/){//pointer to next byte (next cmd or arg)
		switch(cmd){
		case c_nop:
				break;
/*		case c_setPGDinput):
				setPGDinput();
				break;
		case c_setPGDoutput):
				setPGDoutput();
				break;*/
		case c_enablePGC_D:
				enablePGC_D();
				break;
		case c_PGDlow:
				PGDlow();
				break;
		case c_PGDhigh:
				PGDhigh();
				break;
		case c_PGClow:
				PGClow();
				break;
		case c_PGChigh:
				PGChigh();
				break;
		case c_VDDon:
				VDDon();
				break;
		case c_VDDoff:
				VDDoff();
				break;
		case c_HVReset_ENABLE:
				HVRESET_ENABLE;
				break;
		case c_HVReset_OFF:
				HVRESET_OFF;
				break;
		case c_HVReset_TO_RESET:
				HVRESET_TO_RESET;
				break;
		case c_HVReset_TO_HV:
				HVRESET_TO_HV;
				break;
		case c_clock_delay:
				clock_delay();
				break;
		case c_DelayMs:// 1 parameter
				timerMsDelay(*pCmd);
				pCmd++;
			break;
		case c_DelayUs:// 1 parameter *5us
				timerTicksDelay(*pCmd);
				pCmd++;
				break;
		case c_pic_send:
				t8 = (*pCmd);
				pCmd++;
				while(t8 > 8){
					pic_send_n_bits(8, *pCmd);
					pCmd++;
					t8 -= 8;
				}
				pic_send_n_bits(t8, *pCmd);
				pCmd++;
				break;
		case c_dspic_send_24:
				t32.byte3 = 0;
				t32.byte0 = *pCmd;
				pCmd++;
				t32.byte1 = *pCmd;
				pCmd++;
				t32.byte2 = *pCmd;
				pCmd++;
				dspic_send_24_bits(&t32);
				break;
		case c_pic_read:
				*pRes = pic_read(*pCmd);
				pCmd++;
				pRes++;
				retWords++;
				break;
		case c_pic_read_14_bits:
				*pRes = pic_read_14_bits();
				pRes++;
				retWords++;
				break;
		case c_pic_read_byte2:
				*pRes = pic_read_byte2();
				pRes++;
				retWords++;
				break;
		case c_dspic_read_16_bits:
				*pRes = dspic_read_16_bits();
				pRes++;
				retWords++;
				break;
		case c_set_param:
				switch(*pCmd++){
				case p_param_clock_delay:
					clock_delay_cnt = *pCmd;
					break;
				default:
					result->status1 = STK_STATUS_SET_PARAM_MISSING;
				}
				pCmd++;
				break;
		default:
				result->status1 = STK_STATUS_CMD_UNKNOWN;
				*pRes = cmd;
				retWords++;
				goto end;
		}
//		pCmd += args;
		if(retWords > ((BUFFER_SIZE/2)-10)){//?
			result->status1 = STK_STATUS_CMD_FAILED;
			retWords = 0;
			break;
		}
	}
//	BUFFER_SIZE
end:
	PORT_PIN_SET(HWPIN_LED);
//	return numBytes.word +2;
	return (retWords<<1) + 1;
}
