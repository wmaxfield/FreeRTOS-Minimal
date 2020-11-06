# FreeRTOS-Minimal
A FreeRTOS implementation for the PSOC 5 which provides a minimal base for your projects:  UART, USB Serial Uart and an LED flash Task

The Project is in the "master" branch.  The main is the holder for the license and this readme.

If you purchase a CY8CKit-059, then this project will compile and load on it with no changes.
The "Finger" tab for the Debugger also hosts a USB Uart port which is accessed through the Serial UART on pins P12[6] and P12[7]
The USBUart is a USB Serial port which uses the Windows 7 driver generated when you compile the project.  Follow the Project URL's
in the TerminalDebug.c file to use the micro USB Uart Port.  A skeleton command line interface debugging program has been created
as an example.
Enjoy!
