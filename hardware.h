/*
 * Name: hardware.h
 * Project: AVR-Doper
 * Author: Christian Starkjohann <cs@obdev.at>
 * Creation Date: 2006-07-05
 * Tabsize: 4
 * Copyright: (c) 2006 by Christian Starkjohann, all rights reserved.
 * License: GNU GPL v2 (see License.txt) or proprietary (CommercialLicense.txt)
 * Revision: $Id$
 */

/*
General Description:
This module defines hardware properties and configuration choices.
*/

#ifndef __hardware_h_included__
#define __hardware_h_included__


/* configuration options: */
#define USE_DCD_REPORTING       1
/* If this option is defined to 1, the driver will report carrier detect when
 * the serial device is opened. This is useful on some Unix platforms to allow
 * using the /dev/tty* instead of /dev/cu* devices.
 * Setting this option to 0 saves a couple of bytes in flash and RAM memory.
 */
#define ENABLE_DEBUG_INTERFACE  1
/* If this option is defined to 1, the device buffers serial data (read from
 * the the ISP connector) and makes it available through vendor specific
 * requests to host based software. This is useful to display debug information
 * sent by the target.
 * Setting this option to 0 saves a couple of bytes in flash memory.
 */
#define ENABLE_HID_INTERFACE    1
/* If this option is defined to 1, the device implements a custom HID type
 * interface to send and receive STK500 serial data. If both, the CDC-ACM and
 * HID interface are enabled, a jumper selects which one is used.
 */
#define ENABLE_CDC_INTERFACE    0
/* If this option is defined to 1, the device implements a CDC-ACM modem,
 * emulating the behavior of the STK500 board on a virtual COM port.
 */
#define ENABLE_HVPROG           1
/* If this option is defined to 1, the high voltage programmer is enabled.
 */
#define HW_CDC_PACKET_SIZE      8
/* Size of bulk transfer packets. The standard demands 8 bytes, but we may
 * be better off with less. Try smaller values if the communication hangs.
 */
#define HW_DEBUG_BAUDRATE       9600
/* Bit-rate for the UART (for reading debug data from the target).
 */
#ifndef F_CPU
#error "F_CPU must be defined in compiler command line!"
/* #define F_CPU                   12000000 */
/* Our CPU frequency.
 */
#endif


/*
Port        | Function                        | dir | value
------------+---------------------------------+-----+-------
			| ISP		HVPP	HVSP	PIC	  | on Boot
PORT B
  0         | !RESET 	RESET	 +		-		[I]    0
  1 OC1A    | -			SMPS(PWM)+		+		[O]    0
  2         | MOSI		D_SH	-		-		[I]    0
  3 OC2     | CLK		XTAL1	-		-		[I]    0
  4         | MISO		D_IN	-		-		[I]    0
  5         | SCK		D_CL	-		-		[I]    0
  6 XTAL1   | XTAL
  7 XTAL2   | XTAL
PORT C (ADC)
  0         | -			DATA0	SDI		PGD		[I]    0
  1         | -			DATA1	SII		PGC		[I]    0
  2         | -			DATA2	SCI		-		[I]    0
  3         | -			DATA3	-		-		[I]    0
  4         | -			DATA4	-		-		[I]    0
  5         | -			DATA5	-		-		[I]    0
  6 RESET   | Reset
  7 n/a     | *
PORT D
  0 RxD     | RX		DATA6	-		-		[I]    0
  1 TxD     | TX		DATA7	-		-		[I]    0
  2 Int0    | USB D+                            [W]    0
  3 Int1    | USB D-                            [W]    0
  4 T0      | -			RDY		SDO		-		[I]    0
  5 T1      | JP2 SS	Supply	+		+		[O]    1
  6 AIN0    | LED Prog active                   [O]    0
  7 AIN1    | JP1 UH	RESETHV	+		MCLR	[O]    0
  ADC6		| SMPS(+12) feedback
  ADC7		| Voltage sense
*/

/* The following defines can be used with the PORT_* macros from utils.h */
#define HWPIN_SS_JUMPER     D, 5
#define HWPIN_UH_JUMPER     D, 7
#define HWPIN_LED           D, 6
#define HWPIN_USB_DPLUS     D, 2
#define HWPIN_USB_DMINUS    D, 3
#define HWPIN_VOLT_CTRL		7

#define HWPIN_ISP_CLK       B, 3
#define HWPIN_ISP_SCK       B, 5
#define HWPIN_ISP_MISO      B, 4
#define HWPIN_ISP_MOSI      B, 2
#define HWPIN_ISP_RESET     B, 0
#define HWPIN_ISP_TXD       D, 0
#define HWPIN_ISP_RXD       D, 1

#define HWPIN_SMPS_OUT      B, 1
#define HWPIN_SMPS_CTRL		6

#define HWPIN_HVP_RESET     B, 0
#define HWPIN_HVP_iSUPPLY   D, 5
#define HWPIN_HVP_HVRESET   D, 7

#define HWPIN_HVSP_SCI      C, 2
#define HWPIN_HVSP_SII      C, 1
#define HWPIN_HVSP_SDI      C, 0
#define HWPIN_HVSP_SDO      D, 4

#define HWPIN_HVPP_XTAL     B, 3
#define HWPIN_HVPP_DT_SH    B, 2
#define HWPIN_HVPP_DT_IN    B, 4
#define HWPIN_HVPP_DT_CL    B, 5
#define HWPIN_HVPP_DT0    	C, 0
#define HWPIN_HVPP_DT1    	C, 1
#define HWPIN_HVPP_DT2    	C, 2
#define HWPIN_HVPP_DT3    	C, 3
#define HWPIN_HVPP_DT4    	C, 4
#define HWPIN_HVPP_DT5    	C, 5
#define HWPIN_HVPP_DT6    	D, 0
#define HWPIN_HVPP_DT7    	D, 1
#define HWPIN_HVPP_RDY    	D, 4


#define HWPIN_HVPIC_PGD    	C, 0
#define HWPIN_HVPIC_PGC    	C, 1


//#define HWPIN_ISP_DRIVER    B, 4

//#define HWPIN_ADC_SMPS      C, 0
//#define HWPIN_ADC_VTARGET   C, 1

/* Device compatibility: Allow both, ATMega8 and ATMega88/168. The following
 * macros (almost) mimic an ATMega8 based on the ATMega88/168 defines.
 */
#include <avr/io.h>

#ifdef TCCR2A
#   define TCCR2    TCCR2A
#   define COM20    COM2A0
#   define OCR2     OCR2A
#   define HW_SET_T2_PRESCALER(value)   (TCCR2B = (TCCR2B & ~7) | (value & 7))
#   define HW_GET_T2_PRESCALER()        (TCCR2B & 7)
#else
#   define HW_SET_T2_PRESCALER(value)   (TCCR2 = (TCCR2 & ~7) | (value & 7))
#   define HW_GET_T2_PRESCALER()        (TCCR2 & 7)
#endif

#ifdef TCCR0B
#   define TCCR0    TCCR0B
#   define TIMSK    TIMSK0
#endif

#ifdef UBRR0L
#   define UBRRH    UBRR0H
#   define UBRRL    UBRR0L
#   define UCSRA    UCSR0A
#   define UCSRB    UCSR0B
#   define UDR      UDR0
#   define RXEN     RXEN0
#   define TXEN     TXEN0
#   define UDRE     UDRE0
#   define RXCIE    RXCIE0
#endif


#endif /* __hardware_h_included__ */
