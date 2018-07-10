// ********************************************************
// *  I2C Master ESP8266                                  *
// *  ==================                                  *
// *  Basic TX-RX command parser for I2C tests            *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.7 | MCU: ESP8266                *
// *  2018-06-19 gustavo.casanova@nicebots.com            *
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
// z - (INITTINY) Reboot ESP-8266 and initialize ATtiny85
// x - (RESETINY) Reset ATtiny85

#include <Wire.h>
#include "nb-i2c-cmd.h"
#include <pgmspace.h>

#define VCC				3.3				/* PSU VCC 3.3 Volts */
#define ADCTOP			1023			/* ADC Top Value @ 10-bit precision = 1023 (2^10) */
#define DSPBUFFERSIZE	100				/* DSP buffer size (16-bit elements) */
#define MAXBUFFERTXLN	7				/* Maximum DPS buffer TX/RX size */
#define VOLTSADJUST		0.025			/* Measured volts adjust: 0.01 = 1% */

// Global Variables
byte slaveAddress = 0;
byte blockRXSize = 0;
bool newKey = false;
bool newByte = false;
char key = '\0';

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
	Serial.println("Nicebots Pluggie I2C-PWM-ADC Test with Timonel Bootloader (v0.7)");
	Serial.println("================================================================");
	ShowMenu();
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
				byte dataSize = 0;	// DSP buffer data size requested to ATtiny85
				byte dataIX = 0;	// Requested DSP buffer data start position
				Serial.print("Please enter the DSP buffer data start position (1 to 100): ");
				while (newByte == false) {
					dataIX = ReadByte();
				}
				newByte = false;
				Serial.println("");
				Serial.print("Please enter the word amount to retrieve from the DSP buffer (1 to 5): ");
				while (newByte == false) {
					dataSize = ReadByte();
				}
				if (newByte == true) {
					ReadBuffer(dataIX, dataSize);
					newByte = false;
				}
				break;
			}
			// ********************
			// * DUMPBUFF Command *
			// ********************
			case 'j': case 'J': {
				DumpBuffer();
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
				DumpBuffer();
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
				ESP.restart();
				break;
			}
			// ********************
			// * RESETINY Command *
			// ********************
			case 'x': case 'X': {
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
				break;
			}
			// ********************
			// * EXITTMNL Command *
			// ********************
			case 'r': case 'R': {
				//Serial.println("\nBootloader Cmd >>> Run Application ...");
				RunApplication();
				break;
			}
			// ********************
			// * DELFLASH Command *
			// ********************
			case 'e': case 'E': {
				//Serial.println("\nBootloader Cmd >>> Run Application ...");
				DeleteFlash();
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
			Serial.print("Found ATtiny85 at address: ");
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
byte ReadByte() {
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

// Function ReadBuffer
void ReadBuffer(uint8_t dataIX, uint8_t dataSize) {
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
	blockRXSize = Wire.requestFrom(slaveAddress, (dataSize * 2) + 2);
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
			Serial.print("# %%% DSP position ");
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

// Function DumpBuffer
void DumpBuffer(void) {
	byte cmdTX[3] = { READBUFF, 0, 0 };
	byte txSize = 3;
	byte dataIX = 0;
	uint8_t dataSize = 5;
	cmdTX[2] = dataSize;
	byte transmitData[1] = { 0 };
	Serial.println("ESP8266 - Dumping DSP Buffer ...");
	Serial.println("");
	for (uint8_t k = 1; k < DSPBUFFERSIZE + 1; k += 5) {
		//byte dataSize = 0;	// DSP buffer data size requested to ATtiny85
		//byte dataIX = 0;	// Requested DSP buffer data start position
		dataIX = k;
		cmdTX[1] = k;
		for (int i = 0; i < txSize; i++) {
			transmitData[i] = cmdTX[i];
			Wire.beginTransmission(slaveAddress);
			Wire.write(transmitData[i]);
			Wire.endTransmission();
		}
		// Receive acknowledgement
		blockRXSize = Wire.requestFrom(slaveAddress, (dataSize * 2) + 2);
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
				// DSP Buffer 2-Byte Word
				//Serial.print("| DSP ");
				//if (dataIX < 100) {
				//	if (dataIX < 10) {
				//		Serial.print("  ");
				//	}
				//	else {
				//		Serial.print(" ");
				//	}
				//}
				//Serial.print(dataIX++);
				//Serial.print(": ");
				Serial.print((ackRX[i] << 8) + ackRX[i + 1]);
				Serial.println("");
				//Serial.println(" |");
			}
			byte checkCRC = CalculateCRC(ackRX, sizeof(ackRX));
			if (checkCRC == 0) {
				//Serial.print("   >>> CRC OK! <<<   ");
				//Serial.println(checkCRC);
			}
			else {
				Serial.print("ESP8266 - DUMPBUFF aborted due to CRC ERROR! ");
				Serial.println(checkCRC);
				k = 255;
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

//Function ShowMenu
void ShowMenu() {
	Serial.println("Pluggie command ('a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'p', 'n', 'z' reboot, 'x' reset t85)");
	Serial.println(" Booloader cmds ('v' version, 'r' run app, 'e' erase flash, 'b' set address):");
}