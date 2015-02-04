Tool: avrdebug
Date: 10 October 2010
Author: Sjors Hettinga


This program connects the the programmer via libusb and uses vendor specific
requests (added: or by HID interface) to read data from the programmer's UART.
Since the UART RxD is connected to the programming socket, this is a way to
display debug information from the target.
Sending information to the target is currently only implemented in the HID
version this tool.

Sinnce we use Vendor class requests which can be sent to the default Control
endpoint, we don't need to claim an interface or set a configuration. We can
therefore run although the device is opened by another application such as
the CDC-ACM driver or the HID driver.


In linux the package libusb-dev is needed to compile the tool. 
Use "make" to build the tool.

In Windows the tool can be compiled with MinGW. Here also the libusb tool is
required. Use "make -f Makefile.windows" to build the tool.

When the programmer is in HID mode and avrdebug-hid.exe is used, no drivers
are required.
