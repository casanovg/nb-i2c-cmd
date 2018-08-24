// ********************************************************
// *  Nicebots ESP8266 I2C Tests                          *
// *  ==========================                          *
// *  [] I2C Commander Tests []                           *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.2 | MCU: ESP8266                *
// *  2018-08-12 gustavo.casanova@nicebots.com            *
// ********************************************************
//
// Run this master program on a NodeMCU, ESP-01 or ESP-12 Module
// Run the slave program on a Digispark or ATtiny85
//
// Basic command path to Attiny85:
// -------------------------------
// User (serial console) --> ESP8266 --> Attiny85
//
// Available commands:
// -------------------
// a - (STDPB1_1) Set ATtiny85 PB1 = 1
// s - (STDPB1_0) Set ATtiny85 PB1 = 0
// d - (STANAPB3) Set ATtiny85 PB3 = PWMx (the command asks for a PWM value input)
// f - (READADC2) Read ATtiny85 ADC2 (the reply has 2 data bytes + 1 CRC byte)
// g - (GET_INFO) Get useful information regarding various slave parameters.
// h - (READBUFF) Read N-bytes from the DSP buffer.
// j - (DUMPBUFF) Read all the DSP buffer.
// k - (DSPDEBUG) Digital Signal Processing debug data.
// z - (INITTINY) Reboot ESP-8266 and initialize ATtiny85newByte
// x - (RESETINY) Reset ATtiny85

#include <Wire.h>
#include "nb-i2c-cmd.h"
#include <pgmspace.h>

// Pluggie application
#define VCC				3.3		/* PSU VCC 3.3 Volts */
#define ADCTOP			1023	/* ADC Top Value @ 10-bit precision = 1023 (2^10) */
#define DSPBUFFERSIZE	100		/* 10-bit buffer size (16-bit elements) */
#define DSPBUFFTXSIZE	5		/* 10-bit buffer size transmit chunk size */
#define MAXBUFFERTXLN	7		/* Maximum buffer TX/RX size */
#define RXDATASIZE		4		/* RX data size for WRITBUFF command */
#define VOLTSADJUST		0.025	/* Measured volts adjust: 0.01 = 1% */
#define DATATYPEWORD	2		/* Buffer data type "Word" */
// Timonel bootloader
#define MCUTOTALMEM		8192	/* Slave MCU total flash memory*/
#define MAXCKSUMERRORS	100		/* Max number of checksum errors allowed in bootloader comms */
#define TXDATASIZE		4		/* TX data size for WRITBUFF command */
#define FLASHPGSIZE		64		/* Tiny85 flash page buffer size */
#define DATATYPEBYTE	1		/* Buffer data type "Byte" */

// Global Variables
byte slaveAddress = 0;
byte blockRXSize = 0;
bool newKey = false;
bool newByte = false;
bool newWord = false;
bool appMode = true;
char key = '\0';
word flashPageAddr = 0xFFFF;	/* Current flash  page address to be written. Tiny85 allowed values are 0 to 0x2000, so 0xFFFF means 'not set' */
word timonelStart = 0xFFFF;		/* Timonel start address, 0xFFFF means 'not set'. Use Timonel 'version' command to get it */
word bufferValue = 0;			/* Application 10-bit value to write into buffer */

								// CRC Table: Polynomial=0x9C, CRC size=8-bit, HD=5, Word Length=9 bytes
const byte crcTable[256] = {
	//const byte crcTable[256] PROGMEM = {
	0x00, 0x9c, 0xa4, 0x38, 0xd4, 0x48, 0x70, 0xec, 0x34, 0xa8,
	0x90, 0x0c, 0xe0, 0x7c, 0x44, 0xd8, 0x68, 0xf4, 0xcc, 0x50,
	0xbc, 0x20, 0x18, 0x84, 0x5c, 0xc0, 0xf8, 0x64, 0x88, 0x14,
	0x2c, 0xb0, 0xd0, 0x4c, 0x74, 0xe8, 0x04, 0x98, 0xa0, 0x3c,
	0xe4, 0x78, 0x40, 0xdc, 0x30, 0xac, 0x94, 0x08, 0xb8, 0x24,
	0x1c, 0x80, 0x6c, 0xf0, 0xc8, 0x54, 0x8c, 0x10, 0x28, 0xb4,
	0x58, 0xc4, 0xfc, 0x60, 0x3c, 0xa0, 0x98, 0x04, 0xe8, 0x74,
	0x4c, 0xd0, 0x08, 0x94, 0xac, 0x30, 0xdc, 0x40, 0x78, 0xe4,
	0x54, 0xc8, 0xf0, 0x6c, 0x80, 0x1c, 0x24, 0xb8, 0x60, 0xfc,
	0xc4, 0x58, 0xb4, 0x28, 0x10, 0x8c, 0xec, 0x70, 0x48, 0xd4,
	0x38, 0xa4, 0x9c, 0x00, 0xd8, 0x44, 0x7c, 0xe0, 0x0c, 0x90,
	0xa8, 0x34, 0x84, 0x18, 0x20, 0xbc, 0x50, 0xcc, 0xf4, 0x68,
	0xb0, 0x2c, 0x14, 0x88, 0x64, 0xf8, 0xc0, 0x5c, 0x78, 0xe4,
	0xdc, 0x40, 0xac, 0x30, 0x08, 0x94, 0x4c, 0xd0, 0xe8, 0x74,
	0x98, 0x04, 0x3c, 0xa0, 0x10, 0x8c, 0xb4, 0x28, 0xc4, 0x58,
	0x60, 0xfc, 0x24, 0xb8, 0x80, 0x1c, 0xf0, 0x6c, 0x54, 0xc8,
	0xa8, 0x34, 0x0c, 0x90, 0x7c, 0xe0, 0xd8, 0x44, 0x9c, 0x00,
	0x38, 0xa4, 0x48, 0xd4, 0xec, 0x70, 0xc0, 0x5c, 0x64, 0xf8,
	0x14, 0x88, 0xb0, 0x2c, 0xf4, 0x68, 0x50, 0xcc, 0x20, 0xbc,
	0x84, 0x18, 0x44, 0xd8, 0xe0, 0x7c, 0x90, 0x0c, 0x34, 0xa8,
	0x70, 0xec, 0xd4, 0x48, 0xa4, 0x38, 0x00, 0x9c, 0x2c, 0xb0,
	0x88, 0x14, 0xf8, 0x64, 0x5c, 0xc0, 0x18, 0x84, 0xbc, 0x20,
	0xcc, 0x50, 0x68, 0xf4, 0x94, 0x08, 0x30, 0xac, 0x40, 0xdc,
	0xe4, 0x78, 0xa0, 0x3c, 0x04, 0x98, 0x74, 0xe8, 0xd0, 0x4c,
	0xfc, 0x60, 0x58, 0xc4, 0x28, 0xb4, 0x8c, 0x10, 0xc8, 0x54,
	0x6c, 0xf0, 0x1c, 0x80, 0xb8, 0x24
};

// Payload Application test for ATtiny85 (SOS Timer)
const byte payload[1213] = {
	//0x0e, 0xc0, 0x28, 0xc0, 0x27, 0xc0, 0x26, 0xc0, /* SOS Original */ 
	0x3f, 0xcd, 0x28, 0xc0, 0x27, 0xc0, 0x26, 0xc0,   /* SOS with modified reset vector */
	0x25, 0xc0, 0x24, 0xc0, 0x23, 0xc0, 0x22, 0xc0,
	0x21, 0xc0, 0x20, 0xc0, 0x1f, 0xc0, 0x1e, 0xc0,
	0x1d, 0xc0, 0x1c, 0xc0, 0x1b, 0xc0, 0x11, 0x24,
	0x1f, 0xbe, 0xcf, 0xe5, 0xd2, 0xe0, 0xde, 0xbf,
	0xcd, 0xbf, 0x10, 0xe0, 0xa0, 0xe6, 0xb0, 0xe0,
	0xe6, 0xeb, 0xf4, 0xe0, 0x02, 0xc0, 0x05, 0x90,
	0x0d, 0x92, 0xa6, 0x36, 0xb1, 0x07, 0xd9, 0xf7,
	0x20, 0xe0, 0xa6, 0xe6, 0xb0, 0xe0, 0x01, 0xc0,
	0x1d, 0x92, 0xa8, 0x36, 0xb2, 0x07, 0xe1, 0xf7,
	0xdd, 0xd0, 0x2f, 0xc2, 0xd5, 0xcf, 0xf8, 0x94,
	0x80, 0xe8, 0x86, 0xbd, 0x83, 0xe0, 0x86, 0xbd,
	0x17, 0xba, 0x18, 0xba, 0xb9, 0x9a, 0x08, 0x95,
	0x8f, 0x92, 0x9f, 0x92, 0xaf, 0x92, 0xbf, 0x92,
	0xcf, 0x92, 0xdf, 0x92, 0xef, 0x92, 0xff, 0x92,
	0xc1, 0x9a, 0x79, 0x2f, 0x68, 0x2f, 0x99, 0x0f,
	0x88, 0x0b, 0x99, 0x0b, 0xf7, 0xd0, 0x97, 0x2e,
	0x86, 0x2e, 0xb9, 0x2e, 0xa8, 0x2e, 0x20, 0xe0,
	0x30, 0xe0, 0x4a, 0xef, 0x54, 0xe4, 0x7a, 0xd1,
	0xd7, 0x2e, 0xc6, 0x2e, 0xf9, 0x2e, 0xe8, 0x2e,
	0x20, 0xe0, 0x30, 0xe0, 0x40, 0xe8, 0x5f, 0xe3,
	0xb3, 0xd0, 0x87, 0xfd, 0x5b, 0xc0, 0x20, 0xe0,
	0x3f, 0xef, 0x4f, 0xe7, 0x57, 0xe4, 0x9f, 0x2d,
	0x8e, 0x2d, 0x7d, 0x2d, 0x6c, 0x2d, 0x62, 0xd1,
	0x18, 0x16, 0x0c, 0xf0, 0x45, 0xc0, 0x20, 0xe0,
	0x30, 0xe0, 0x40, 0xe2, 0x51, 0xe4, 0x9b, 0x2d,
	0x8a, 0x2d, 0x79, 0x2d, 0x68, 0x2d, 0x5a, 0xd1,
	0x9f, 0xd0, 0x28, 0xec, 0x30, 0xe0, 0x61, 0x15,
	0x71, 0x05, 0x79, 0xf5, 0xc1, 0x98, 0x20, 0xe0,
	0x30, 0xe0, 0x40, 0xe8, 0x5f, 0xe3, 0x9f, 0x2d,
	0x8e, 0x2d, 0x7d, 0x2d, 0x6c, 0x2d, 0x8c, 0xd0,
	0x87, 0xfd, 0x48, 0xc0, 0x20, 0xe0, 0x3f, 0xef,
	0x4f, 0xe7, 0x57, 0xe4, 0x9f, 0x2d, 0x8e, 0x2d,
	0x7d, 0x2d, 0x6c, 0x2d, 0x3b, 0xd1, 0x18, 0x16,
	0x9c, 0xf5, 0x20, 0xe0, 0x30, 0xe0, 0x40, 0xe2,
	0x51, 0xe4, 0x9b, 0x2d, 0x8a, 0x2d, 0x79, 0x2d,
	0x68, 0x2d, 0x34, 0xd1, 0x79, 0xd0, 0x28, 0xec,
	0x30, 0xe0, 0x61, 0x15, 0x71, 0x05, 0xe9, 0xf4,
	0xff, 0x90, 0xef, 0x90, 0xdf, 0x90, 0xcf, 0x90,
	0xbf, 0x90, 0xaf, 0x90, 0x9f, 0x90, 0x8f, 0x90,
	0x08, 0x95, 0x93, 0x2f, 0x82, 0x2f, 0x01, 0x97,
	0xf1, 0xf7, 0x61, 0x50, 0x71, 0x09, 0xc7, 0xcf,
	0x9f, 0x2d, 0x8e, 0x2d, 0x7d, 0x2d, 0x6c, 0x2d,
	0x5f, 0xd0, 0x97, 0x2f, 0x86, 0x2f, 0x01, 0x97,
	0xf1, 0xf7, 0xc0, 0xcf, 0x61, 0xe0, 0x70, 0xe0,
	0xf8, 0xcf, 0x93, 0x2f, 0x82, 0x2f, 0x01, 0x97,
	0xf1, 0xf7, 0x61, 0x50, 0x71, 0x09, 0xd9, 0xcf,
	0x9f, 0x2d, 0x8e, 0x2d, 0x7d, 0x2d, 0x6c, 0x2d,
	0x4b, 0xd0, 0x97, 0x2f, 0x86, 0x2f, 0x01, 0x97,
	0xf1, 0xf7, 0xd2, 0xcf, 0x61, 0xe0, 0x70, 0xe0,
	0xf8, 0xcf, 0xf9, 0x2f, 0xe8, 0x2f, 0x80, 0x81,
	0x83, 0x35, 0x51, 0xf0, 0x28, 0xf4, 0x80, 0x32,
	0xc9, 0xf0, 0x8f, 0x34, 0x71, 0xf0, 0x08, 0x95,
	0x8f, 0x36, 0x59, 0xf0, 0x83, 0x37, 0xe1, 0xf4,
	0x82, 0xe3, 0x90, 0xe0, 0x59, 0xdf, 0x82, 0xe3,
	0x90, 0xe0, 0x56, 0xdf, 0x82, 0xe3, 0x90, 0xe0,
	0x53, 0xcf, 0x84, 0xe6, 0x90, 0xe0, 0x50, 0xdf,
	0x84, 0xe6, 0x90, 0xe0, 0x4d, 0xdf, 0x84, 0xe6,
	0x90, 0xe0, 0xf6, 0xcf, 0x80, 0xed, 0x97, 0xe0,
	0x28, 0xec, 0x30, 0xe0, 0xf3, 0x2f, 0xe2, 0x2f,
	0x31, 0x97, 0xf1, 0xf7, 0x01, 0x97, 0xd1, 0xf7,
	0x08, 0x95, 0x10, 0x92, 0x67, 0x00, 0x10, 0x92,
	0x66, 0x00, 0x80, 0xe6, 0x90, 0xe0, 0xcd, 0xdf,
	0x82, 0xe6, 0x90, 0xe0, 0xca, 0xdf, 0x80, 0xe6,
	0x90, 0xe0, 0xc7, 0xdf, 0x84, 0xe6, 0x90, 0xe0,
	0xc4, 0xdf, 0xef, 0xcf, 0x24, 0xdf, 0xed, 0xdf,
	0x6c, 0xd0, 0x08, 0xf4, 0x81, 0xe0, 0x08, 0x95,
	0x94, 0xd0, 0x88, 0xf0, 0x9f, 0x57, 0x90, 0xf0,
	0xb9, 0x2f, 0x99, 0x27, 0xb7, 0x51, 0xa0, 0xf0,
	0xd1, 0xf0, 0x66, 0x0f, 0x77, 0x1f, 0x88, 0x1f,
	0x99, 0x1f, 0x1a, 0xf0, 0xba, 0x95, 0xc9, 0xf7,
	0x12, 0xc0, 0xb1, 0x30, 0x81, 0xf0, 0x9b, 0xd0,
	0xb1, 0xe0, 0x08, 0x95, 0x98, 0xc0, 0x67, 0x2f,
	0x78, 0x2f, 0x88, 0x27, 0xb8, 0x5f, 0x39, 0xf0,
	0xb9, 0x3f, 0xcc, 0xf3, 0x86, 0x95, 0x77, 0x95,
	0x67, 0x95, 0xb3, 0x95, 0xd9, 0xf7, 0x3e, 0xf4,
	0x90, 0x95, 0x80, 0x95, 0x70, 0x95, 0x61, 0x95,
	0x7f, 0x4f, 0x8f, 0x4f, 0x9f, 0x4f, 0x08, 0x95,
	0xe8, 0x94, 0x09, 0xc0, 0x97, 0xfb, 0x3e, 0xf4,
	0x90, 0x95, 0x80, 0x95, 0x70, 0x95, 0x61, 0x95,
	0x7f, 0x4f, 0x8f, 0x4f, 0x9f, 0x4f, 0x99, 0x23,
	0xa9, 0xf0, 0xf9, 0x2f, 0x96, 0xe9, 0xbb, 0x27,
	0x93, 0x95, 0xf6, 0x95, 0x87, 0x95, 0x77, 0x95,
	0x67, 0x95, 0xb7, 0x95, 0xf1, 0x11, 0xf8, 0xcf,
	0xfa, 0xf4, 0xbb, 0x0f, 0x11, 0xf4, 0x60, 0xff,
	0x1b, 0xc0, 0x6f, 0x5f, 0x7f, 0x4f, 0x8f, 0x4f,
	0x9f, 0x4f, 0x16, 0xc0, 0x88, 0x23, 0x11, 0xf0,
	0x96, 0xe9, 0x11, 0xc0, 0x77, 0x23, 0x21, 0xf0,
	0x9e, 0xe8, 0x87, 0x2f, 0x76, 0x2f, 0x05, 0xc0,
	0x66, 0x23, 0x71, 0xf0, 0x96, 0xe8, 0x86, 0x2f,
	0x70, 0xe0, 0x60, 0xe0, 0x2a, 0xf0, 0x9a, 0x95,
	0x66, 0x0f, 0x77, 0x1f, 0x88, 0x1f, 0xda, 0xf7,
	0x88, 0x0f, 0x96, 0x95, 0x87, 0x95, 0x97, 0xf9,
	0x08, 0x95, 0x99, 0x0f, 0x00, 0x08, 0x55, 0x0f,
	0xaa, 0x0b, 0xe0, 0xe8, 0xfe, 0xef, 0x16, 0x16,
	0x17, 0x06, 0xe8, 0x07, 0xf9, 0x07, 0xc0, 0xf0,
	0x12, 0x16, 0x13, 0x06, 0xe4, 0x07, 0xf5, 0x07,
	0x98, 0xf0, 0x62, 0x1b, 0x73, 0x0b, 0x84, 0x0b,
	0x95, 0x0b, 0x39, 0xf4, 0x0a, 0x26, 0x61, 0xf0,
	0x23, 0x2b, 0x24, 0x2b, 0x25, 0x2b, 0x21, 0xf4,
	0x08, 0x95, 0x0a, 0x26, 0x09, 0xf4, 0xa1, 0x40,
	0xa6, 0x95, 0x8f, 0xef, 0x81, 0x1d, 0x81, 0x1d,
	0x08, 0x95, 0x57, 0xfd, 0x90, 0x58, 0x44, 0x0f,
	0x55, 0x1f, 0x59, 0xf0, 0x5f, 0x3f, 0x71, 0xf0,
	0x47, 0x95, 0x88, 0x0f, 0x97, 0xfb, 0x99, 0x1f,
	0x61, 0xf0, 0x9f, 0x3f, 0x79, 0xf0, 0x87, 0x95,
	0x08, 0x95, 0x12, 0x16, 0x13, 0x06, 0x14, 0x06,
	0x55, 0x1f, 0xf2, 0xcf, 0x46, 0x95, 0xf1, 0xdf,
	0x08, 0xc0, 0x16, 0x16, 0x17, 0x06, 0x18, 0x06,
	0x99, 0x1f, 0xf1, 0xcf, 0x86, 0x95, 0x71, 0x05,
	0x61, 0x05, 0x08, 0x94, 0x08, 0x95, 0xe8, 0x94,
	0xbb, 0x27, 0x66, 0x27, 0x77, 0x27, 0xcb, 0x01,
	0x97, 0xf9, 0x08, 0x95, 0xb2, 0xdf, 0x08, 0xf4,
	0x8f, 0xef, 0x08, 0x95, 0x0a, 0xd0, 0x80, 0xc0,
	0x71, 0xd0, 0x28, 0xf0, 0x76, 0xd0, 0x18, 0xf0,
	0x95, 0x23, 0x09, 0xf0, 0x62, 0xc0, 0x67, 0xc0,
	0xeb, 0xcf, 0xc7, 0xdf, 0xa8, 0xf3, 0x99, 0x23,
	0xd9, 0xf3, 0x55, 0x23, 0xc9, 0xf3, 0x95, 0x0f,
	0x50, 0xe0, 0x55, 0x1f, 0xaa, 0x27, 0xee, 0x27,
	0xff, 0x27, 0xbb, 0x27, 0x00, 0x24, 0x08, 0x94,
	0x67, 0x95, 0x20, 0xf4, 0xe2, 0x0f, 0xf3, 0x1f,
	0xb4, 0x1f, 0x0a, 0x1e, 0x22, 0x0f, 0x33, 0x1f,
	0x44, 0x1f, 0xaa, 0x1f, 0x66, 0x95, 0xa9, 0xf7,
	0x77, 0x95, 0x30, 0xf4, 0xf3, 0x0f, 0xb4, 0x1f,
	0x0a, 0x1e, 0x12, 0x1e, 0x08, 0xf4, 0x63, 0x95,
	0x33, 0x0f, 0x44, 0x1f, 0xaa, 0x1f, 0x22, 0x1f,
	0x76, 0x95, 0x99, 0xf7, 0x87, 0x95, 0x20, 0xf4,
	0xb4, 0x0f, 0x0a, 0x1e, 0x12, 0x1e, 0x63, 0x1f,
	0x44, 0x0f, 0xaa, 0x1f, 0x22, 0x1f, 0x33, 0x1f,
	0x86, 0x95, 0xa9, 0xf7, 0x86, 0x2f, 0x71, 0x2d,
	0x60, 0x2d, 0x11, 0x24, 0x9f, 0x57, 0x50, 0x40,
	0x8a, 0xf0, 0xe1, 0xf0, 0x88, 0x23, 0x4a, 0xf0,
	0xee, 0x0f, 0xff, 0x1f, 0xbb, 0x1f, 0x66, 0x1f,
	0x77, 0x1f, 0x88, 0x1f, 0x91, 0x50, 0x50, 0x40,
	0xa9, 0xf7, 0x9e, 0x3f, 0x51, 0x05, 0x70, 0xf0,
	0x14, 0xc0, 0x9e, 0xcf, 0x5f, 0x3f, 0xec, 0xf3,
	0x98, 0x3e, 0xdc, 0xf3, 0x86, 0x95, 0x77, 0x95,
	0x67, 0x95, 0xb7, 0x95, 0xf7, 0x95, 0xe7, 0x95,
	0x9f, 0x5f, 0xc1, 0xf7, 0xfe, 0x2b, 0x88, 0x0f,
	0x91, 0x1d, 0x96, 0x95, 0x87, 0x95, 0x97, 0xf9,
	0x08, 0x95, 0x97, 0xf9, 0x9f, 0x67, 0x80, 0xe8,
	0x70, 0xe0, 0x60, 0xe0, 0x08, 0x95, 0x9f, 0xef,
	0x80, 0xec, 0x08, 0x95, 0x00, 0x24, 0x0a, 0x94,
	0x16, 0x16, 0x17, 0x06, 0x18, 0x06, 0x09, 0x06,
	0x08, 0x95, 0x00, 0x24, 0x0a, 0x94, 0x12, 0x16,
	0x13, 0x06, 0x14, 0x06, 0x05, 0x06, 0x08, 0x95,
	0x09, 0x2e, 0x03, 0x94, 0x00, 0x0c, 0x11, 0xf4,
	0x88, 0x23, 0x52, 0xf0, 0xbb, 0x0f, 0x40, 0xf4,
	0xbf, 0x2b, 0x11, 0xf4, 0x60, 0xff, 0x04, 0xc0,
	0x6f, 0x5f, 0x7f, 0x4f, 0x8f, 0x4f, 0x9f, 0x4f,
	0x08, 0x95, 0xf8, 0x94, 0xff, 0xcf, 0x73, 0x00,
	0x6f, 0x00, 0x20, 0x00, 0xff
};

//volatile byte payload[FLASHPGSIZE * 2] = { 0 };

//
// ***************************
// * Setup Block (Runs once) *
// ***************************
//
void setup() {
	Serial.begin(9600); // Init the serial port
						// Init the Wire object for I2C
	Wire.begin(0, 2);   // GPIO0 - GPIO2 (ESP-01) // D3 - D4 (NodeMCU)
						//Wire.begin(); // Standard pins SDA on D2 and SCL on D1 (NodeMCU)
						//Wire.begin(D3, D4); // Set SDA on D3 and SCL on D4 (NodeMCU)
	delay(500);        // Wait 1/2 second for slave init sequence
					   // Search continuouly for slave addresses
	while (slaveAddress == 0) {
		slaveAddress = ScanI2C();
		delay(3000);
	}

	// Run ATtiny85 initialization command
	byte cmdTX[1] = { INITTINY };
	Wire.beginTransmission(slaveAddress);
	Wire.write(cmdTX[0]);
	Wire.endTransmission();
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	blockRXSize = 0;

	ClrScr();
	Serial.println("Nicebots Pluggie Commander for Application and Timonel Bootloader (v0.2)");
	Serial.println("========================================================================");
	ShowMenu();

	//for (byte i = 0; i < FLASHPGSIZE * 2; i++) {
	//	payload[i] = i;
	//}

	//---for (byte i = 0; i < FLASHPGSIZE * 2; i++) {
	//---	payload[i] = i;
	//---}

	//for (byte i = 0; i < FLASHPGSIZE * 2; i++) {
	//	payload[i] = i;
	//}
/*	for (byte i = 127; i < 192; i++) {
		payload[i] = 3;
	}*/	//(byte)i;
	  
}

//
// **********************************
// * Main Loop, (Runs continuously) *
// **********************************
//
void loop() {
	if (newKey == true) {
		newKey = false;
		Serial.println("");
		Serial.println("");
		switch (key) {
			// ********************
			// * STDPB1_1 Command *
			// ********************
			case 'a': case 'A': {
				SetPB1On();
				break;
			}
			// ********************
			// * STDPB1_0 Command *
			// ********************
			case 's': case 'S': {
				SetPB1Off();
				break;
			}
			// ********************
			// * STANAPB3 Command *
			// ********************
			case 'd': case 'D': {
				byte triacTriggerDelay = 0;
				Serial.print("Please enter a value between 0 and 255 for this command: ");
				while (newByte == false) {
					triacTriggerDelay = ReadByte();
				}
				if (newByte == true) {
					SetPB3Analog(triacTriggerDelay);
					newByte = false;
				}
				break;
			}
			// ********************
			// * READADC2 Command *
			// ********************
			case 'f': case 'F': {
				ReadADC2();
				break;
			}
			// ************************************
			// * GET_INFO Command (16 byte reply) *
			// ************************************
			case 'g': case 'G': {
				GetInfo();
				delay(250);
				Serial.println("");
				ReleaseAnalogData();
				break;
			}
			// ********************
			// * READBUFF Command *
			// ********************
			case 'h': case 'H': {
				byte dataSize = 0;	// 10-bit buffer data size requested to ATtiny85
				byte dataIX = 0;	// Requested 10-bit buffer data start position
				Serial.print("Please enter the 10-bit buffer data start position (1 to 100): ");
				while (newByte == false) {
					dataIX = ReadByte();
				}
				newByte = false;
				Serial.println("");
				Serial.print("Please enter the word amount to retrieve from the 10-bit buffer (1 to 5): ");
				while (newByte == false) {
					dataSize = ReadByte();
				}
				if (newByte == true) {
					Read10bitBuff(dataIX, dataSize);
					newByte = false;
				}
				break;
			}
			// ********************
			// * WRITBUFF Command *
			// ********************
			case 'u': case 'U': {
				newByte = false;
				newWord = false;
				byte dataIX = 0;	// Requested DSP buffer data start position
				Serial.print("Please enter the 10-bit buffer position to write (1 to 100): ");
				while (newByte == false) {
					dataIX = ReadByte();
				}
				if (dataIX > 100) {
					Serial.println("\n\rError: The buffer position must be between 0 and 100 ");
					newByte = false;
					break;
				}
				Serial.print("\n\rPlease enter the 10-bit buffer value to write: ");
				while (newWord == false) {
					bufferValue = ReadWord();
				}
				if (bufferValue > 1023) {
					Serial.println("\n\n\rError: The buffer only accepts values between 0 and 1023 ");
					newWord = false;
					break;
				}
				if ((newByte == true) & (newWord == true)) {
					Serial.println("");
					Serial.print("\n\r10 bit-buffer value to write: ");
					Serial.print(bufferValue);
					Serial.print(" into position: ");
					Serial.println(dataIX);
					Serial.print("10 bit-buffer high byte: ");
					Serial.print((bufferValue & 0xFF00) >> 8);
					Serial.print(" (<< 8) + 10 bit-buffer low byte: ");
					Serial.println(bufferValue & 0xFF);
					Write10bitBuff(dataIX, bufferValue);
					newWord = false;
				}
				break;
			}
			// ********************
			// * DUMPBUFF Command *
			// ********************
			case 'j': case 'J': {
				//void DumpBuffer(byte bufferSize, byte dataSize, byte dataType, byte valuesPerLine)
				Dump10bitBuff(DSPBUFFERSIZE, DSPBUFFTXSIZE, 10);
				delay(250);
				Serial.println("");
				//ReleaseAnalogData();
				break;
			}
			// ********************
			// * DSPDEBUG Command *
			// ********************
			case 'k': case 'K': {
				GetInfo();
				delay(250);
				Serial.println("");
				//void DumpBuffer(byte bufferSize, byte dataSize, byte dataType, byte valuesPerLine)
				Dump10bitBuff(DSPBUFFERSIZE, DSPBUFFTXSIZE, 10);
				delay(250);
				Serial.println("");
				ReleaseAnalogData();
				break;
			}
			// ********************
			// * FIXPOSIT Command *
			// ********************
			case 'p': case 'P': {
				FixPositiveHC();
				break;
			}
			// ********************
			// * FIXNEGAT Command *
			// ********************
			case 'n': case 'N': {
				FixNegativeHC();
				break;
			}
			// *******************
			// * Restart ESP8266 *
			// *******************
			case 'z': case 'Z': {
				Serial.println("\nResetting ESP8266 ...");
				Serial.println("\n.\n.\n.\n");
				ESP.restart();
				break;
			}
			// ********************
			// * RESETINY Command *
			// ********************
			case 'x': case 'X': {
				ResetTiny();
				Serial.println("\n  .\n\r . .\n\r. . .\n");
				delay(2000);
				ESP.restart();
				break;
			}
			// ********************************
			// * Timonel ::: GETTMNLV Command *
			// ********************************
			case 'v': case 'V': {
				//Serial.println("\nBootloader Cmd >>> Get bootloader version ...");
				GetTimonelVersion();
				break;
			}
			// ********************************
			// * Timonel ::: EXITTMNL Command *
			// ********************************
			case 'r': case 'R': {
				//Serial.println("\nBootloader Cmd >>> Run Application ...");
				RunApplication();
				Serial.println("\n. . .\n\r . .\n\r  .\n");
				delay(2000);
				ESP.restart();
				break;
			}
			// ********************************
			// * Timonel ::: DELFLASH Command *
			// ********************************
			case 'e': case 'E': {
				//Serial.println("\nBootloader Cmd >>> Delete app firmware from T85 flash memory ...");
				DeleteFlash();
				break;
			}
			// ********************************
			// * Timonel ::: STPGADDR Command *
			// ********************************
			case 'b': case 'B': {
				Serial.print("Please enter the flash memory page base address: ");
				while (newWord == false) {
					flashPageAddr = ReadWord();
				}
				if (timonelStart > MCUTOTALMEM) {
					Serial.println("\n\n\rWarning: Timonel bootloader start address unknown, please run 'version' command to find it !");
					//newWord = false;
					break;
				}
				if ((flashPageAddr > (timonelStart - 64)) | (flashPageAddr == 0xFFFF)) {
					Serial.print("\n\n\rWarning: The highest flash page addreess available is ");
					Serial.print(timonelStart - 64);
					Serial.print(" (0x");
					Serial.print(timonelStart - 64, HEX);
					Serial.println("), please correct it !!!");
					newWord = false;
					break;
				}
				if (newWord == true) {
					Serial.println("");
					Serial.print("Flash memory page base address: ");
					Serial.println(flashPageAddr);
					Serial.print("Address high byte: ");
					Serial.print((flashPageAddr & 0xFF00) >> 8);
					Serial.print(" (<< 8) + Address low byte: ");
					Serial.print(flashPageAddr & 0xFF);
					SetTmlPageAddr(flashPageAddr);
					newWord = false;
				}
				break;
			}
			// ********************************
			// * Timonel ::: WRITPAGE Command *
			// ********************************
			case 'w': case 'W': {
				//Serial.println("\nBootloader Cmd >>> Write new app firmware to T85 flash memory ...");
				WriteFlash();
				//WriteFlashTest();
				break;
			}
			// ********************************
			// * Timonel ::: READPAGE Command *
			// ********************************
			case 'q': case 'Q': {
				//ReadPageBuff(dataIX, dataSize)

				byte dataSize = 0;	// 10-bit buffer data size requested to ATtiny85
				byte dataIX = 0;	// Requested 10-bit buffer data start position
				Serial.print("Please enter the flash page data start position (1 to 64; If > 64, dump the whole page buffer): ");
				while (newByte == false) {
					dataIX = ReadByte();
				}
				if (dataIX <= FLASHPGSIZE) {
					newByte = false;
					Serial.println("");
					Serial.print("Please enter the byte amount to retrieve from the flash page buffer (1 to 10): ");
					while (newByte == false) {
						dataSize = ReadByte();
					}
					if (newByte == true) {
						ReadPageBuff(dataIX, dataSize);
						newByte = false;
					}
				}
				else {
					DumpPageBuff(FLASHPGSIZE, 4, 8);
					newByte = false;
				}
				break;
			}
			// ******************
			// * ? Help Command *
			// ******************
			case '?': case '¿': {
				Serial.println("\n\rNicebots Pluggie Help");
				Serial.println("=====================");
				//ShowHelp();
				break;
			}
			// *******************
			// * Unknown Command *
			// *******************
			default: {
				Serial.print("ESP8266 - Command '");
				Serial.print(key);
				Serial.println("' unknown ...");
				break;
			}
		}
		Serial.println("");
		ShowMenu();
	}
	ReadChar();           // PROD - REMOVE FOR TESTING
}

// Function ScanI2C
byte ScanI2C() {
	//clrscr();
	Serial.println("Scanning I2C bus ...");
	byte slaveAddr = 0, scanAddr = 8;
	while (scanAddr < 120) {
		Wire.beginTransmission(scanAddr);
		if (Wire.endTransmission() == 0) {
			if (scanAddr < 21) {
				Serial.print("T85 Timonel Bootloader found at address: ");
				appMode = false;
			}
			else {
				Serial.print("T85 Pluggie App Firmware found at address: ");
				appMode = true;
			}
			Serial.print(scanAddr, DEC);
			Serial.print(" (0x");
			Serial.print(scanAddr, HEX);
			Serial.println(")");
			delay(500);
			slaveAddr = scanAddr;
		}
		scanAddr++;
	}
	return slaveAddr;
}

// Function CalculateCRC (CRC-8)
byte CalculateCRC(byte* block, size_t blockLength) {
	int i;
	byte crc = 0, data = 0;
	for (i = 0; i < blockLength; i++) {
		data = (byte)(block[i] ^ crc); // XOR-in next input byte
		crc = (byte)(crcTable[data]);  // Get current CRC value = remainder 
	}
	return crc;
}

// Function ReadChar
void ReadChar() {
	if (Serial.available() > 0) {
		key = Serial.read();
		newKey = true;
	}
}

// Function ReadByte
byte ReadByte(void) {
	const byte dataLength = 16;
	char serialData[dataLength];	// an array to store the received data  
	static byte ix = 0;
	char rc, endMarker = 0xD;		//standard is: char endMarker = '\n'
	while (Serial.available() > 0 && newByte == false) {
		rc = Serial.read();
		if (rc != endMarker) {
			serialData[ix] = rc;
			Serial.print(serialData[ix]);
			ix++;
			if (ix >= dataLength) {
				ix = dataLength - 1;
			}
		}
		else {
			serialData[ix] = '\0';	// terminate the string
			ix = 0;
			newByte = true;
		}
	}
	if ((atoi(serialData) < 0 || atoi(serialData) > 255) && newByte == true) {
		Serial.println("");
		Serial.print("WARNING! Byte values must be 0 to 255 -> Truncating to ");
		Serial.println((byte)atoi(serialData));
	}
	return((byte)atoi(serialData));
}

// Function ReadWord
word ReadWord(void) {
	const byte dataLength = 16;
	char serialData[dataLength];	// an array to store the received data  
	static byte ix = 0;
	char rc, endMarker = 0xD;		//standard is: char endMarker = '\n'
	while (Serial.available() > 0 && newWord == false) {
		rc = Serial.read();
		if (rc != endMarker) {
			serialData[ix] = rc;
			Serial.print(serialData[ix]);
			ix++;
			if (ix >= dataLength) {
				ix = dataLength - 1;
			}
		}
		else {
			serialData[ix] = '\0';	// terminate the string
			ix = 0;
			newWord = true;
		}
	}
	if ((atoi(serialData) < 0 || atoi(serialData) > MCUTOTALMEM) && newWord == true) {
		for (int i = 0; i < dataLength; i++) {
			serialData[i] = 0;
		}
		Serial.println("");
		Serial.print("WARNING! Word memory positions must be between 0 and ");
		Serial.print(MCUTOTALMEM);
		Serial.print(" -> Changing to ");
		Serial.println((word)atoi(serialData));
	}
	return((word)atoi(serialData));
}

// Function Clear Screen
void ClrScr() {
	Serial.write(27);       // ESC command
	Serial.print("[2J");    // clear screen command
	Serial.write(27);       // ESC command
	Serial.print("[H");     // cursor to home command
}

// Function SetPB1On
void SetPB1On(void) {
	byte cmdTX[1] = { STDPB1_1 };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(STDPB1_1)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == AKDPB1_1) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function SetPB1Off
void SetPB1Off(void) {
	byte cmdTX[1] = { STDPB1_0 };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(STDPB1_0)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == AKDPB1_0) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function SetPB3Analog()
void SetPB3Analog(uint8_t triacTriggerDelay) {
	byte cmdTX[3] = { STANAPB3, 0, 0 };
	byte txSize = 3;
	Serial.println("");
	Serial.println("");
	cmdTX[1] = triacTriggerDelay;
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(STANAPB3)");
	cmdTX[2] = CalculateCRC(cmdTX, 2);
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		if (i > 0) {
			if (i < 2) {
				Serial.print("ESP8266 - Sending Operand >>> ");
				Serial.println(cmdTX[i]);
			}
			else {
				Serial.print("ESP8266 - Sending CRC >>> ");
				Serial.println(cmdTX[i]);
			}
		}
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)2);
	byte ackRX[2] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKANPB3) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		if (ackRX[1] == 0) {
			Serial.print("ESP8266 - Operand ");
			Serial.print(cmdTX[1]);
			Serial.print(" parsed OK by slave <<< ATtiny85 CRC Check = ");
			Serial.println(ackRX[1]);
		}
		else {
			Serial.print("ESP8266 - Operand ");
			Serial.print(cmdTX[1]);
			Serial.print(" parsed with {{{ERROR}}} <<< ATtiny85 CRC Check = ");
			Serial.println(ackRX[1]);
		}

	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function ReadADC2
void ReadADC2(void) {
	byte cmdTX[1] = { READADC2 };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(READADC2)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)4);
	byte ackRX[4] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	word analogValue = ((ackRX[1] << 8) + ackRX[2]);
	if (ackRX[0] == ACKNADC2) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		for (int i = 1; i < blockRXSize; i++) {
			Serial.print("ESP8266 - Data Byte ");
			Serial.print(i + 1);
			Serial.print(" received OK <<< ");
			Serial.println(ackRX[i]);
		}
		Serial.println("*************************");
		Serial.print("* Analog Value: ");
		Serial.print(analogValue);
		Serial.print("(");
		Serial.print(analogValue, HEX);
		Serial.println(") *");
		Serial.println("*************************");
		Serial.print("ESP8266 - MSB = ");
		Serial.print(ackRX[1]);
		Serial.print(" | LSB=");
		Serial.print(ackRX[2]);
		Serial.print(" | CRC=");
		Serial.println(ackRX[3]);
		Serial.print("CRC received ---> ");
		Serial.println(ackRX[sizeof(ackRX)]);
		byte checkCRC = CalculateCRC(ackRX, sizeof(ackRX));
		if (checkCRC == 0) {
			Serial.print("   >>> CRC OK! <<<   ");
			Serial.println(checkCRC);
		}
		else {
			Serial.print("   ### CRC ERROR! ###   ");
			Serial.println(checkCRC);
		}
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function GetInfo
void GetInfo(void) {
	byte cmdTX[1] = { GET_INFO };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(GET_INFO)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)16);
	byte ackRX[16] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACK_GETI) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		// -----------------------------------------------------
		// Fixed half-cycle for ADC calculations
		Serial.print("(((* Fixed half-cycle: ");
		switch (ackRX[1]) {
		case 1: {
			Serial.println("Positive [+] *)))");
			break;
		}
		case 0: {
			Serial.println("Negative [-] *)))");
			break;
		}
		default: {
			Serial.println("Unknown [?] *)))");
			break;
		}
		}
		// ADC half-cycle mid point
		Serial.print("%%%* ADC Mid Point: ");
		Serial.print((ackRX[2] << 8) + ackRX[3]);
		Serial.println(" *%%%");
		// -----------------------------------------------------
		// Half-cycle Vi Average
		Serial.print(":::* Half-cycle Vi Average: ");
		Serial.print((ackRX[4] << 8) + ackRX[5]);
		Serial.println(" *:::");
		// -----------------------------------------------------
		// Sum of squared Vi's (instantaneous voltages)
		Serial.print(">>>* Sum of Squared Vi's: ");
		Serial.print((unsigned)(ackRX[6] << 24) + (ackRX[7] << 16) + (ackRX[8] << 8) + ackRX[9]);
		Serial.print(" (");
		Serial.print(((ackRX[6] << 24) + (ackRX[7] << 16) + (ackRX[8] << 8) + ackRX[9]), HEX);
		Serial.println(") *<<<");
		// -----------------------------------------------------
		// VRMS (Square root of sum of sqred VI's / ADC samples
		uint16_t vRMS = (ackRX[10] << 8) + ackRX[11];
		vRMS = vRMS;
		float volts = (vRMS * VCC) / ADCTOP;
		volts = volts + (volts * VOLTSADJUST);								/* VOLTS COMPENSATION ADJUST */
		Serial.print("}}}* Vrms (AC): ");
		Serial.print(vRMS);
		Serial.print(" *{{{ -----------------------> {{ ");
		Serial.print(volts, 3);
		Serial.println(" Volts RMS }}");
		// -----------------------------------------------------
		// Last analog value readout (DC)
		Serial.print("[[[* Analog Value (Last): ");
		Serial.print((ackRX[12] << 8) + ackRX[13]);
		Serial.println(" *]]]");
		// -----------------------------------------------------
		// ADC conversions per AC half-cycle
		Serial.print("~~~ ADC count: ");
		Serial.print(ackRX[14]);
		Serial.println(" ~~~");
		// --------------------------------------------------
		byte checkCRC = CalculateCRC(ackRX, sizeof(ackRX));
		if (checkCRC == 0) {
			Serial.print("   >>> CRC OK! <<<   ");
			Serial.println(checkCRC);
		}
		else {
			Serial.print("   ### CRC ERROR! ###   ");
			Serial.println(checkCRC);
		}
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function Read10bitBuff
void Read10bitBuff(uint8_t dataIX, uint8_t dataSize) {
	byte cmdTX[3] = { READBUFF, 0, 0 };
	byte txSize = 3;
	Serial.println("");
	Serial.println("");
	cmdTX[1] = dataIX;
	cmdTX[2] = dataSize;
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(READBUFF)");
	Serial.print("ESP8266 - Sending position IX >>> ");
	Serial.println(cmdTX[1]);
	Serial.print("ESP8266 - Sending requested data size >>> ");
	Serial.println(cmdTX[2]);
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)((dataSize * 2) + 2));
	byte ackRX[(dataSize * 2) + 2];   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKRDBUF) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		for (uint8_t i = 1; i < (dataSize * 2) + 1; i += 2) {
			// DSP Buffer 2-Byte Word
			Serial.print("# %%% Buffer position ");
			if (dataIX < 10) {
				Serial.print("0");
			}
			Serial.print(dataIX++);
			Serial.print(": ");
			Serial.print((ackRX[i] << 8) + ackRX[i + 1]);
			Serial.println(" %%% #");
		}
		byte checkCRC = CalculateCRC(ackRX, sizeof(ackRX));
		if (checkCRC == 0) {
			Serial.print("   >>> CRC OK! <<<   ");
			Serial.println(checkCRC);
		}
		else {
			Serial.print("   ### CRC ERROR! ###   ");
			Serial.println(checkCRC);
		}
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function Dump10bitBuff
void Dump10bitBuff(byte bufferSize, byte dataSize, byte valuesPerLine) {
	byte cmdTX[3] = { READBUFF, 0, 0 };
	byte txSize = 3;
	uint8_t crcErrors = 0;
	int v = 1;
	cmdTX[2] = dataSize;
	byte transmitData[1] = { 0 };
	Serial.println("ESP8266 - Dumping Tiny85 Buffer ...");
	Serial.println("");
	for (uint8_t k = 1; k < bufferSize + 1; k += dataSize) {
		//byte dataSize = 0;	// Requested T85 buffer data size
		//byte dataIX = 0;		// Requested T85 buffer data start position
		cmdTX[1] = k;
		for (int i = 0; i < txSize; i++) {
			transmitData[i] = cmdTX[i];
			Wire.beginTransmission(slaveAddress);
			Wire.write(transmitData[i]);
			Wire.endTransmission();
		}
		// Receive acknowledgement
		blockRXSize = Wire.requestFrom(slaveAddress, (byte)((dataSize * 2) + 2));
		byte ackRX[(dataSize * 2) + 2];   // Data received from slave
		for (int i = 0; i < blockRXSize; i++) {
			ackRX[i] = Wire.read();
		}
		if (ackRX[0] == ACKRDBUF) {
			//Serial.print("ESP8266 - Command ");
			//Serial.print(cmdTX[0]);
			//Serial.print(" parsed OK <<< ");
			//Serial.println(ackRX[0]);
			for (uint8_t i = 1; i < (dataSize * 2) + 1; i += 2) {
				Serial.print((ackRX[i] << 8) + ackRX[i + 1]);	/* 2 = dataType word */
				if (v == valuesPerLine) {
					Serial.println("");
					v = 0;
				}
				else {
					Serial.print(" ");
				}
				v++;
				//Serial.println(" |");
			}
			byte checkCRC = CalculateCRC(ackRX, sizeof(ackRX));
			if (checkCRC == 0) {
				//Serial.print("   >>> CRC OK! <<<   ");
				//Serial.println(checkCRC);
			}
			else {
				Serial.print("ESP8266 - DUMPBUFF aborted due to Checksum ERROR! ");
				Serial.println(checkCRC);
				if (crcErrors++ == MAXCKSUMERRORS) {
					delay(1000);
					exit(1);
				}
			}
		}
		else {
			Serial.print("ESP8266 - Error parsing ");
			Serial.print(cmdTX[0]);
			Serial.print(" command! <<< ");
			Serial.println(ackRX[0]);
		}
		delay(500);
	}
}

// Function Write10bitBuff
void Write10bitBuff(byte bufferPosition, word bufferValue) {
	byte cmdTX[5] = { WRITBUFF, 0, 0, 0, 0 };
	byte txSize = 5;
	Serial.println("");
	cmdTX[1] = bufferPosition;						/* Buffer position to write */
	cmdTX[2] = ((bufferValue & 0xFF00) >> 8);		/* Buffer value high byte */
	cmdTX[3] = (bufferValue & 0xFF);				/* Buffer value low byte */
	Serial.print("\nWritting value to Attiny85 10-bit buffer >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(WRITBUFF)");
	cmdTX[4] = CalculateCRC(cmdTX, 4);				/* CRC-8 */
	// Transmit command
	byte transmitData[5] = { 0 };
	for (int i = 0; i < txSize; i++) {
		if (i > 0) {
			if (i < txSize - 1) {
				Serial.print("ESP8266 - Sending Operand >>> ");
				Serial.println(cmdTX[i]);
			}
			else {
				Serial.print("ESP8266 - Sending CRC >>> ");
				Serial.println(cmdTX[i]);
			}
		}
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)2);
	byte ackRX[2] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKWTBUF) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		if (ackRX[1] == 0) {
			Serial.print("ESP8266 - Operands ");
			Serial.print(cmdTX[1]);
			Serial.print(", ");
			Serial.print(cmdTX[2]);
			Serial.print(" and ");
			Serial.print(cmdTX[3]);
			Serial.print(" successfully parsed by slave <<< ATtiny85 CRC Check = ");
			Serial.println(ackRX[1]);
		}
		else {
			Serial.print("ESP8266 - Operand ");
			Serial.print(cmdTX[1]);
			Serial.print(" parsed with {{{ERROR}}} <<< ATtiny85 CRC Check = ");
			Serial.println(ackRX[1]);
		}

	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function ReadPageBuff
void ReadPageBuff(uint8_t dataIX, uint8_t dataSize) {
	byte cmdTX[3] = { READPAGE, 0, 0 };
	byte txSize = 3;
	Serial.println("");
	Serial.println("");
	cmdTX[1] = dataIX;
	cmdTX[2] = dataSize;
	Serial.print("[Timonel] - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(READPAGE)");
	Serial.print("[Timonel] - Sending position IX >>> ");
	Serial.println(cmdTX[1]);
	Serial.print("[Timonel] - Sending requested data size >>> ");
	Serial.println(cmdTX[2]);
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)(dataSize + 2));
	byte ackRX[dataSize + 2];   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKRDPAG) {
		Serial.print("[Timonel] - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		uint8_t checksum = 0;
		for (uint8_t i = 1; i < (dataSize + 1); i++) {
			// DSP Buffer 2-Byte Word
			Serial.print("# ~~~ Page position ");
			if (dataIX < 10) {
				Serial.print("0");
			}
			Serial.print(dataIX++);
			Serial.print(": ");
			Serial.print(ackRX[i]);
			Serial.println(" ~~~ #");
			checksum += (uint8_t)ackRX[i];
		}
		if (checksum == ackRX[dataSize + 1]) {
			Serial.print("   >>> Checksum OK! <<<   ");
			Serial.println(checksum);
		}
		else {
			Serial.print("   ### Checksum ERROR! ###   ");
			Serial.println(checksum);
		}
	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function DumpPageBuff
void DumpPageBuff(byte bufferSize, byte dataSize, byte valuesPerLine) {
	byte cmdTX[3] = { READPAGE, 0, 0 };
	byte txSize = 3;
	uint8_t checksumErr = 0;
	int v = 1;
	cmdTX[2] = dataSize;
	byte transmitData[1] = { 0 };
	Serial.println("\n\n\r[Timonel] - Dumping Flash Memory Page Buffer ...");
	Serial.println("");
	for (uint8_t k = 1; k < bufferSize + 1; k += dataSize) {
		//byte dataSize = 0;	// Requested T85 buffer data size
		//byte dataIX = 0;		// Requested T85 buffer data start position
		cmdTX[1] = k;
		for (int i = 0; i < txSize; i++) {
			transmitData[i] = cmdTX[i];
			Wire.beginTransmission(slaveAddress);
			Wire.write(transmitData[i]);
			Wire.endTransmission();
		}
		// Receive acknowledgement
		blockRXSize = Wire.requestFrom(slaveAddress, (byte)(dataSize + 2));
		byte ackRX[dataSize + 2];   // Data received from slave
		for (int i = 0; i < blockRXSize; i++) {
			ackRX[i] = Wire.read();
		}
		if (ackRX[0] == ACKRDPAG) {
			//Serial.print("ESP8266 - Command ");
			//Serial.print(cmdTX[0]);
			//Serial.print(" parsed OK <<< ");
			//Serial.println(ackRX[0]);
			uint8_t checksum = 0;
			for (uint8_t i = 1; i < (dataSize + 1); i++) {
				if (ackRX[i] < 16) {
					Serial.print("0x0");
				}
				else {
					Serial.print("0x");
				}
				Serial.print(ackRX[i], HEX);			/* Byte values */
				//checksum += (ackRX[i]);
				if (v == valuesPerLine) {
					Serial.println("");
					v = 0;
				}
				else {
					Serial.print(" ");
				}
				v++;
				//Serial.println(" |");
				checksum += (uint8_t)ackRX[i];
			}
			//if (checksum + 1 == ackRX[dataSize + 1]) {
			if (checksum == ackRX[dataSize + 1]) {
				//Serial.print("   >>> Checksum OK! <<<   ");
				//Serial.println(checksum);
			}
			else {
				Serial.print("\n\r   ### Checksum ERROR! ###   ");
				Serial.println(checksum);
				//Serial.print(checksum + 1);
				//Serial.print(" <-- calculated, received --> ");
				//Serial.println(ackRX[dataSize + 1]);
				if (checksumErr++ == MAXCKSUMERRORS) {
					Serial.println("[Timonel] - Too many Checksum ERRORS, aborting! ");
					delay(1000);
					exit(1);
				}
			}
		}
		else {
			Serial.print("[Timonel] - DumpPageBuff Error parsing ");
			Serial.print(cmdTX[0]);
			Serial.print(" command! <<< ");
			Serial.println(ackRX[0]);
		}
		delay(500);
	}
}

// Function WritePageBuff
int WritePageBuff(uint8_t dataArray[]) {
	const byte txSize = TXDATASIZE + 2;
	byte cmdTX[txSize] = { 0 };
	int commErrors = 0;					/* I2C communication error counter */
	uint8_t checksum = 0;
	Serial.println("");
	cmdTX[0] = WRITPAGE;
	for (int b = 1; b < txSize - 1; b++) {
		cmdTX[b] = dataArray[b - 1];
		checksum += (byte)dataArray[b - 1];
	}
	cmdTX[txSize - 1] = checksum;
	//Serial.print("[Timonel] Writting data to Attiny85 memory page buffer >>> ");
	//Serial.print(cmdTX[0]);
	//Serial.println("(WRITBUFF)");
	// Transmit command
	byte transmitData[txSize] = { 0 };
	//Serial.print("[Timonel] - Sending data >>> ");
	for (int i = 0; i < txSize; i++) {
		//if (i > 0) {
		//	if (i < txSize - 1) {
		//		Serial.print("0x");
		//		Serial.print(cmdTX[i], HEX);
		//		Serial.print(" ");
		//	}
		//	else {
		//		Serial.print("\n\r[Timonel] - Sending CRC >>> ");
		//		Serial.println(cmdTX[i]);
		//	}
		//}
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)2);
	byte ackRX[2] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKWTPAG) {
		//Serial.print("[Timonel] - Command ");
		//Serial.print(cmdTX[0]);
		//Serial.print(" parsed OK <<< ");
		//Serial.println(ackRX[0]);
		if (ackRX[1] == checksum) {
			//Serial.print("[Timonel] - Data parsed OK by slave <<< Checksum = 0x");
			//Serial.println(ackRX[1], HEX);
			//Serial.println("");
		}
		else {
			Serial.print("[Timonel] - Data parsed with {{{ERROR}}} <<< Checksum = 0x");
			Serial.println(ackRX[1], HEX);
			//Serial.println("");
			if (commErrors++ > 0) {					/* Checksum error detected ... */
				Serial.println("\n\r[Timonel] - WritePageBuff Checksum Errors, Aborting ...");
				exit(commErrors);
			}
		}

	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
		Serial.println("");
		if (commErrors++ > 0) {					/* Opcode error detected ... */
			Serial.println("\n\r[Timonel] - WritePageBuff Opcode Reply Errors, Aborting ...");
			exit(commErrors);
		}
	}
	return(commErrors);
}

// Function ReleaseAnalogData *
void ReleaseAnalogData(void) {
	byte cmdTX[1] = { REL_ANDT };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(REL_ANDT)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACK_RELD) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function FixPositiveHC
void FixPositiveHC(void) {
	byte cmdTX[1] = { FIXPOSIT };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(FIXPOSIT)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKFXPOS) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function FixNegativeHC
void FixNegativeHC(void) {
	byte cmdTX[1] = { FIXNEGAT };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(FIXNEGAT)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKFXNEG) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

//Function ResetTiny
void ResetTiny(void) {
	Serial.println("Sending ATtiny85 Reset Command ...");
	byte cmdTX[1] = { RESETINY };
	byte txSize = sizeof(cmdTX);
	Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(RESETINY)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKRESTY) {
		Serial.print("ESP8266 - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("ESP8266 - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function GetTimonelVersion
void GetTimonelVersion(void) {
	byte cmdTX[1] = { GETTMNLV };
	byte txSize = sizeof(cmdTX);
	Serial.print("\n[Timonel] Get Timonel Version >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(GETTMNLV)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)8);
	byte ackRX[8] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKTMNLV) {
		Serial.print("[Timonel] - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		Serial.print(">>> ");
		Serial.print((char)ackRX[1]);
		Serial.print((char)ackRX[2]);
		Serial.print((char)ackRX[3]);
		Serial.print(" <<< Version: ");
		Serial.print(ackRX[4]);
		Serial.print(".");
		Serial.print(ackRX[5]);
		Serial.print(" >>> Base address: 0x");
		Serial.print((ackRX[6] << 8) + ackRX[7], HEX);
		Serial.println(" <<<");
		timonelStart = (ackRX[6] << 8) + ackRX[7];
	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function RunApplication
void RunApplication(void) {
	byte cmdTX[1] = { EXITTMNL };
	byte txSize = sizeof(cmdTX);
	Serial.print("\n[Timonel] Exit bootloader & run application >>> ");
	//Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(EXITTMNL)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKEXITT) {
		Serial.print("[Timonel] - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function DeleteFlash
void DeleteFlash(void) {
	byte cmdTX[1] = { DELFLASH };
	byte txSize = sizeof(cmdTX);
	Serial.print("\n[Timonel] Delete Flash Memory >>> ");
	//Serial.print("ESP8266 - Sending Opcode >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(DELFLASH)");
	// Transmit command
	byte transmitData[1] = { 0 };
	for (int i = 0; i < txSize; i++) {
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)1);
	byte ackRX[1] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == ACKDELFL) {
		Serial.print("[Timonel] - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function SetTmlPageAddr
void SetTmlPageAddr(word pageAddr) {
	byte cmdTX[4] = { STPGADDR, 0, 0, 0 };
	byte txSize = 4;
	Serial.println("");
	cmdTX[1] = ((pageAddr & 0xFF00) >> 8);		/* Flash page address high byte */
	cmdTX[2] = (pageAddr & 0xFF);				/* Flash page address low byte */
	Serial.print("\n[Timonel] Setting flash page address on Attiny85 >>> ");
	Serial.print(cmdTX[0]);
	Serial.println("(STPGADDR)");
	cmdTX[3] = CalculateCRC(cmdTX, 2);
	// Transmit command
	byte transmitData[4] = { 0 };
	for (int i = 0; i < txSize; i++) {
		if (i > 0) {
			if (i < txSize - 1) {
				Serial.print("[Timonel] - Sending Operand >>> ");
				Serial.println(cmdTX[i]);
			}
			else {
				Serial.print("[Timonel] - Sending CRC >>> ");
				Serial.println(cmdTX[i]);
			}
		}
		transmitData[i] = cmdTX[i];
		Wire.beginTransmission(slaveAddress);
		Wire.write(transmitData[i]);
		Wire.endTransmission();
	}
	// Receive acknowledgement
	blockRXSize = Wire.requestFrom(slaveAddress, (byte)2);
	byte ackRX[2] = { 0 };   // Data received from slave
	for (int i = 0; i < blockRXSize; i++) {
		ackRX[i] = Wire.read();
	}
	if (ackRX[0] == AKPGADDR) {
		Serial.print("[Timonel] - Command ");
		Serial.print(cmdTX[0]);
		Serial.print(" parsed OK <<< ");
		Serial.println(ackRX[0]);
		if (ackRX[1] == (byte)(cmdTX[1] + cmdTX[2])) {
			Serial.print("[Timonel] - Operands ");
			Serial.print(cmdTX[1]);
			Serial.print(" and ");
			Serial.print(cmdTX[2]);
			Serial.print(" parsed OK by slave <<< ATtiny85 Flash Page Address Check = ");
			Serial.println(ackRX[1]);
		}
		else {
			Serial.print("[Timonel] - Operand ");
			Serial.print(cmdTX[1]);
			Serial.print(" parsed with {{{ERROR}}} <<< ATtiny85 Flash Page Address Check = ");
			Serial.println(ackRX[1]);
		}

	}
	else {
		Serial.print("[Timonel] - Error parsing ");
		Serial.print(cmdTX[0]);
		Serial.print(" command! <<< ");
		Serial.println(ackRX[0]);
	}
}

// Function WriteFlash
int WriteFlash(void) {
	int packet = 0;								/* Byte counter to be sent in a single I2C data packet */
	int padding = 0;							/* Amount of padding bytes to match the page size */
	int pageEnd = 0;							/* Byte counter to detect the end of flash mem page */
	int pageCount = 1;
	int wrtErrors = 0;
	uint8_t dataPacket[TXDATASIZE] = { 0xFF };
	int payloadSize = sizeof(payload);
	if ((payloadSize % FLASHPGSIZE) != 0) {		/* If the payload to be sent is smaller than flash page size, resize it to match */
		padding = ((((uint)(payloadSize / FLASHPGSIZE) + 1) * FLASHPGSIZE) - payloadSize);
		payloadSize += padding;
	}
	Serial.println("\n1-Deleting flash ...\n\r");
	Serial.println("\n2-Writing payload to flash ...\n\n\r");
	if (flashPageAddr == 0xFFFF) {
		Serial.println("Warning: Flash page start address no set, please use 'b' command to set it ...\n\r");
		return(1);
	}
	Serial.print("::::::::::::::::::: Page ");
	Serial.print(pageCount);
	Serial.print(" - Address ");
	Serial.println(flashPageAddr);
	for (int i = 0; i < payloadSize; i++) {
		if (i < (payloadSize - padding)) {
			dataPacket[packet] = payload[i];		/* If there are data to fill the page, use it ... */
		}
		else {
			dataPacket[packet] = 0xff;				/* If there are no more data, complete the page with padding (0xff) */
		}
		if (packet++ == (TXDATASIZE - 1)) {		/* When a data packet is completed to be sent ... */
			for (int b = 0; b < TXDATASIZE; b++) {
				Serial.print("0x");
				if (dataPacket[b] < 0x10) {
					Serial.print("0");
				}
				Serial.print(dataPacket[b], HEX);
				Serial.print(" ");
			}
			wrtErrors += WritePageBuff(dataPacket);	/* Send data to T85 through I2C */
			packet = 0;
			delay(200);
		}
		if (pageEnd++ == (FLASHPGSIZE - 1)) {	/* When a page end is detected ... */

			//DumpPageBuff(FLASHPGSIZE, TXDATASIZE, TXDATASIZE);
			delay(500);	/* DELAY BETWEEN PAGE WRITINGS ... */

			if (i < (payloadSize - 1)) {
				Serial.print("::::::::::::::::::: Page ");
				Serial.print(++pageCount);
				Serial.print(" - Address ");
				Serial.println(flashPageAddr + 1 + i);
				pageEnd = 0;
			}
		}
		if (wrtErrors > 10) {
			Serial.println("\n\r==== WriteFlash: too many errors, aborting ...");
			i = payloadSize;
		}
	}
	if (wrtErrors == 0) {
		Serial.println("\n\r==== WriteFlash: Firmware was successfully transferred to T85, please select 'run app' command to start it ...");
	}
	else {
		Serial.print("\n\r==== WriteFlash: Communication errors detected during firmware transfer, please retry !!! ErrCnt: ");
		Serial.print(wrtErrors);
		Serial.println(" ===");
	}
}

// Function WriteFlashTest
int WriteFlashTest(void) {
	int packet = 0;								/* Byte counter to be sent in a single I2C data packet */
	int padding = 0;							/* Amount of padding bytes to match the page size */
	int pageEnd = 0;							/* Byte counter to detect the end of flash mem page */
	int pageCount = 1;
	int wrtErrors = 0;
	uint8_t wrtBuff[TXDATASIZE] = { 0xFF };

	Serial.println("\n1-Deleting flash ...\n\r");
	Serial.println("\n2-Writing payload to flash ...\n\n\r");
	if (flashPageAddr == 0xFFFF) {
		Serial.println("Warning: Flash page start address no set, please use 'b' command to set it ...\n\r");
		return(1);
	}
	Serial.print("::::::::::::::::::: Page ");
	Serial.print(pageCount);
	Serial.print(" - Address ");
	Serial.println(flashPageAddr);
	for (int i = 0; i < FLASHPGSIZE; i++) {
		// ---	>>>
		wrtBuff[packet] = i + 1;				/* Store consecutive numbers in flash memory ... */
												// --- >>>
		if (packet++ == (TXDATASIZE - 1)) {		/* When a data packet is completed to be sent ... */
			for (int b = 0; b < TXDATASIZE; b++) {
				Serial.print("0x");
				if (wrtBuff[b] < 0x10) {
					Serial.print("0");
				}
				Serial.print(wrtBuff[b], HEX);
				Serial.print(" ");
			}
			wrtErrors += WritePageBuff(wrtBuff);	/* Send data to T85 through I2C */
			packet = 0;
			delay(50);
		}
		if (pageEnd++ == (FLASHPGSIZE - 1)) {	/* When a page end is detected ... */
			if (i < (FLASHPGSIZE - 1)) {
				Serial.print("::::::::::::::::::: Page ");
				Serial.print(++pageCount);
				Serial.print(" - Address ");
				Serial.println(flashPageAddr + 1 + i);
				pageEnd = 0;
			}
		}
		if (wrtErrors > 10) {
			Serial.println("\n\r==== WriteFlashTest: too many errors, aborting ...");
			i = FLASHPGSIZE;
		}
	}
	if (wrtErrors == 0) {
		Serial.println("\n\r==== WriteFlashTest: Firmware was successfully transferred to T85, please select 'run app' command to start it ...");
	}
	else {
		Serial.print("\n\r==== WriteFlashTest: Communication errors detected during firmware transfer, please retry !!! ErrCnt: ");
		Serial.print(wrtErrors);
		Serial.println(" ===");
	}
}

//Function ShowMenu
void ShowMenu(void) {
	if (appMode == true) {
		Serial.println("Application command ('a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'p', 'n', 'u', 'z' reboot, 'x' reset T85, '?' help): ");
	}
	else {
		Serial.print("Timonel booloader ('v' version, 'r' run app, 'e' erase flash, 'b' set address, 'w' write flash, 'q' read flash): ");
	}
}