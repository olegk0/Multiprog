/*
 * Name: winusb.h
 * Project: avrdebug
 * Author: Sjors Hettinga
 * Creation Date: 2010-10-10
 * Tabsize: 4
 * License: GNU General Public License.
 * Originates form ser_avrdoper.c from the avrdude project
  */

#ifndef WINUSB_H
#define WINUSB_H

#include <stdio.h>

#define USB_HID_REPORT_TYPE_INPUT   1
#define USB_HID_REPORT_TYPE_OUTPUT  2
#define USB_HID_REPORT_TYPE_FEATURE 3
/* Numeric constants for 'reportType' parameters */

// feature number used for debug
#define USB_TYPE_HID_DEBUGDATA	4

typedef void    usbDevice_t;

// all the report sizes of the features
extern const int  reportDataSizes[5];

// function declaration
char *usbErrorText(int usbErrno);
int usbSendData(usbDevice_t **device, unsigned char *buf, size_t buflen);
int usbOpenDevice(usbDevice_t **device, int vendor, char *vendorName, int product, char *productName, int usesReportIDs);
void usbCloseDevice(usbDevice_t *device);
int usbGetReport(usbDevice_t *device, int reportType, int reportNumber, char *buffer, int *len);

#endif