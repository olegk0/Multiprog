/*
 * Name: utils.c
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-07-02
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

/* all functionality is in utils.h */

#include "utils.h"
#include "timer.h"
#include "vreg.h"
#include <avr/io.h>
#include <util/delay.h>
#include "hardware.h"
#include "stk500protocol.h"

uchar HVgenOn(uint uPRG)
{
	uchar to;

	vregInit(uPRG);
	TCCR1A = UTIL_BIN8(1000, 0010);  /* OC1A = PWM, OC1B disconnected, 9 bit */
	timerMsDelay(200);
/*	to = 10;// 2 sec
	while(!HVok){
		PORT_PIN_SWITCH(HWPIN_LED);
//		_delay_ms(100);
		timerMsDelay(100);
		to--;
		if(!to){
			break;
//			PORT_PIN_CLR(HWPIN_LED);
//			return(STK_STATUS_CMD_TOUT);//error
		}
	}*/
    PORT_PIN_SET(HWPIN_LED);
	return(STK_STATUS_CMD_OK);
}

void HVgenOff()
{
//	vregStop();
	ADCSRA = 0;
	TCCR1A = UTIL_BIN8(0000, 0010);  /* OC1A , OC1B disconnected, 9 bit */
	PORT_PIN_CLR(HWPIN_SMPS_OUT);
//	PORT_PIN_CLR(HWPIN_LED);
}
/*
void ClkOutOn()
{
	TCCR2 = UTIL_BIN8(0001, 1001); // activate clock
}

void ClkOutOff()
{
	TCCR2 = UTIL_BIN8(0000, 1001);  // OC2 disconnected
}
*/
void IOReset()
{
	//see hardware.h init table
	DDRB = (1 << PORT_BIT(HWPIN_SMPS_OUT))|HWPINS_HVPP_SH;//pins to out
    PORTB = 0;
//	ShByte = 0;
	OutShByte(0);

	DDRB = (1 << PORT_BIT(HWPIN_SMPS_OUT));
    PORTB = 0;

    PORTC = 0;
    DDRC = 0;

    DDRD &= ~((1 << PORT_BIT(HWPIN_HVPP_DT6))|(1 << PORT_BIT(HWPIN_HVPP_DT7))|(1 << PORT_BIT(HWPIN_HVPP_RDY)));
    DDRD |= (1 << PORT_BIT(HWPIN_HVP_iSUPPLY))|(1 << PORT_BIT(HWPIN_LED))|(1 << PORT_BIT(HWPIN_HVP_HVRESET));
    PORTD &= ~((1 << PORT_BIT(HWPIN_HVPP_DT6))|(1 << PORT_BIT(HWPIN_HVPP_DT7))|(1 << PORT_BIT(HWPIN_HVPP_RDY))|(1 << PORT_BIT(HWPIN_LED))|(1 << PORT_BIT(HWPIN_HVP_HVRESET)));
    PORTD |= (1 << PORT_BIT(HWPIN_HVP_iSUPPLY));

}
/*
uchar GetSpeedSlowJPVal()
{
	uchar ret;

	PORT_PIN_CLR(HWPIN_SS_JUMPER);//pullup off
	PORT_DDR_IN(HWPIN_SS_JUMPER);//input
	_delay_us(10);
	ret = (PORT_PIN_VALUE(HWPIN_SS_JUMPER) == 0);
	PORT_DDR_OUT(HWPIN_SS_JUMPER);//by def
	return ret;
}
*/
void OutShByte(uchar data)
{
	uchar ind=128;

	PORT_PIN_CLR(HWPIN_HVPP_DT_CL);
	PORT_PIN_CLR(HWPIN_HVPP_DT_SH);
	while(ind){
//		if(ind & ShByte)// out 1
		if(ind & data)// out 1
			PORT_PIN_SET(HWPIN_HVPP_DT_IN);
		else
			PORT_PIN_CLR(HWPIN_HVPP_DT_IN);
		nop();
		PORT_PIN_SET(HWPIN_HVPP_DT_CL);
		nop();
		PORT_PIN_CLR(HWPIN_HVPP_DT_CL);
		ind >>=1;
	}
	PORT_PIN_SET(HWPIN_HVPP_DT_SH);
	nop();
	PORT_PIN_CLR(HWPIN_HVPP_DT_SH);
}


