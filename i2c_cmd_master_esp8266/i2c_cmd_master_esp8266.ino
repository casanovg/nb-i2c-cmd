// ********************************************************
// *  I2C Master ESP8266                                  *
// *  ==================                                  *
// *  Basic TX-RX command parser for I2C tests            *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.1 | MCU: ESP8266                *
// *  2017-07-01 gustavo.casanova@nicebots.com            *
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

#include <Wire.h>

#define SLAVE_RESET_PIN 2

#define STDPB1_1 0xE9 // Command to Set ATtiny85 PB1 = 1
#define AKDPB1_1 0x16 // Acknowledge Command PB1 = 1

#define STDPB1_0 0xE1 // Command to Set ATtiny85 PB1 = 0
#define AKDPB1_0 0x1E // Acknowledge Command PB1 = 0

#define STANAPB3 0xFB // Command to Set ATtiny85 PB3 = PWMx
#define ACKNAPB3 0x04 // Acknowledge Command PB3 = PWMx

#define READADC2 0xDA // Command to Read ATtiny85 ADC2
#define ACKNADC2 0x25 // Acknowledge Command Read ADC2

//typedef uint8_t byte; //  8 bit data type
//typedef uint16_t word; // 16 bit data type

// Global Variables
word analogVal = 0;
byte slaveAddress = 0;
int blockRXSize = 0;
bool newKey = false, newByte = false;
char key = '\0';

// CRC Table: Polynomial=0x9C, CRC size=8-bit, HD=5, Word Length=9 bytes
byte crcTable[256] = {
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
//***************************
//* Setup Block (Runs once) *
//***************************
//
void setup() {
	pinMode(SLAVE_RESET_PIN, OUTPUT); // Set pin modes
	Serial.begin(9600); // Init the serial port
						// Init the Wire object for I2C
	Wire.begin(); // Standard pins SDA on D2 and SCL on D1 (NodeMCU)
				  //Wire.begin(D3, D4); // Set SDA on D3 and SCL on D4 (NodeMCU)
	digitalWrite(SLAVE_RESET_PIN, LOW); // Reset the slave
	delay(10);
	digitalWrite(SLAVE_RESET_PIN, HIGH);
	delay(1000); // Wait 2 seconds for slave init sequence
				 // Search continuouly for slave addresses
	while (slaveAddress == 0) {
		slaveAddress = scanI2C();
		delay(1000);
	}
	clrscr();
	Serial.println("Nicebots I2C Commands Test");
	Serial.println("==========================");
	Serial.println("Please type a command ('a', 's', 'd' or 'f'):");
}


//
//**********************************
//* Main Loop, (Runs continuously) *
//**********************************
//
void loop() {

	if (newKey == true) {
		newKey = false;
		Serial.println("");
		switch (key) {
			// ********************
			// * STDPB1_1 Command *
			//*********************
		case 'a': case 'A': {
			byte cmdTX[1] = { STDPB1_1 };
			byte txSize = sizeof(cmdTX), rxSize = 0;
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
			blockRXSize = Wire.requestFrom(slaveAddress, (int)txSize);
			Serial.print("ESP8266 - ?�?�?� RX BLOCKSIZE: ");
			Serial.print(blockRXSize);
			Serial.println("");
			byte ackRX[blockRXSize];   // Data received from slave
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
			break;
		}
				  // ********************
				  // * STDPB1_0 Command *
				  //*********************      
		case 's': case 'S': {
			byte cmdTX[1] = { STDPB1_0 };
			byte txSize = sizeof(cmdTX), rxSize = 0;
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
			blockRXSize = Wire.requestFrom(slaveAddress, (int)txSize);
			Serial.print("ESP8266 - ?�?�?� RX BLOCKSIZE: ");
			Serial.print(blockRXSize);
			Serial.println("");
			byte ackRX[blockRXSize];   // Data received from slave
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
			break;
		}
				  // ********************
				  // * STANAPB3 Command *
				  //*********************       
		case 'd': case 'D': {
			byte cmdTX[2] = { STANAPB3, 0x00 };
			//byte cmdTX[1] = {STANAPB3};
			//byte txSize = sizeof (cmdTX), rxSize = 0;
			byte txSize = 2, rxSize = 0;
			byte operandValue = 0;
			Serial.print("ESP8266 - Opcode ");
			Serial.print(cmdTX[0]);
			Serial.println("(STANAPB3)");
			Serial.print("Please enter a value from 0 to 255 for this command: ");
			while (newByte == false) {
				//operandValue = random(1, 255);
				operandValue = ReadByte();
			}
			if (newByte == true) {
				Serial.println("");
				Serial.print("Command STANAPB3 Operand Value (decimal): ");
				Serial.println(operandValue);
				Serial.println("");
				cmdTX[1] = operandValue;
				Serial.print("ESP8266 - Sending Opcode >>> ");
				Serial.print(cmdTX[0]);
				Serial.println("(STANAPB3)");
				Serial.print("ESP8266 - Sending Operand >>> ");
				Serial.print(cmdTX[1]);
				Serial.print("(");
				Serial.print(cmdTX[1]);
				Serial.println(")");
				// Transmit command
				byte transmitData[2] = { 0 };
				for (int i = 0; i < txSize; i++) {
					transmitData[i] = cmdTX[i];
					Wire.beginTransmission(slaveAddress);
					//Serial.print("ESP8266 - Sending Byte >>> ");
					//Serial.println(transmitData[i]);            
					Wire.write(transmitData[i]);
					Wire.endTransmission();
				}
				newByte = false;
			}
			// Receive acknowledgement
			blockRXSize = Wire.requestFrom(slaveAddress, (int)txSize);
			Serial.print("ESP8266 - ?�?�?� RX BLOCKSIZE: ");
			Serial.print(blockRXSize);
			Serial.println("");
			byte ackRX[blockRXSize];   // Data received from slave
			for (int i = 0; i < blockRXSize; i++) {
				ackRX[i] = Wire.read();
				//Serial.print("ESP8266 - ##### ");
				//Serial.print(">> ");
				//Serial.print(i);
				//Serial.print(" << ");
				//Serial.print(ackRX[i]);
				//Serial.println("");
			}
			if (ackRX[0] == ACKNAPB3) {
				Serial.print("ESP8266 - Command ");
				Serial.print(cmdTX[0]);
				Serial.print(" parsed OK <<< ");
				Serial.println(ackRX[0]);
				//Serial.print("ESP8266 - Operand &&&&&& ");
				//Serial.println((byte)~cmdTX[1]);
				if (ackRX[1] == (byte)~cmdTX[1]) {
					Serial.print("ESP8266 - Operand ");
					Serial.print(cmdTX[1]);
					Serial.print(" parsed OK <<< ");
					//Serial.print(" parsed +++++ <<< ");
					Serial.println(ackRX[1]);
				}
			}
			else {
				Serial.print("ESP8266 - Error parsing ");
				Serial.print(cmdTX[0]);
				Serial.print(" command! <<< ");
				Serial.println(ackRX[0]);
			}
			//free(ackRX);
			break;
		}
				  // ********************
				  // * READADC2 Command *
				  //*********************      
		case 'f': case 'F': {
			byte cmdTX[1] = { READADC2 };
			byte txSize = sizeof(cmdTX), rxSize = 0;
			Serial.print("ESP8266 - Opcode ");
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
			blockRXSize = Wire.requestFrom(slaveAddress, 4);
			byte ackRX[blockRXSize];   // Data received from slave
			for (int i = 0; i < blockRXSize; i++) {
				ackRX[i] = Wire.read();
			}
			word analogValue = ((ackRX[1] << 8) + ackRX[2]);
			if (ackRX[0] == ACKNADC2) {
				Serial.print("ESP8266 - Command ");
				Serial.print(cmdTX[0]);
				Serial.print(" parsed OK <<< ");
				Serial.println(ackRX[0]);
				for (int i = 1; i < rxSize; i++) {
					Serial.print("ESP8266 - Data Byte ");
					Serial.print(i + 1);
					Serial.print(" received OK <<< ");
					Serial.println(ackRX[i]);
				}
				Serial.print("ESP8266 - Analog Data=");
				Serial.print(analogValue);
				Serial.print("(");
				Serial.print(analogValue);
				Serial.print(") >>> | MSB=");
				Serial.print(ackRX[1]);
				Serial.print(" | LSB=");
				Serial.print(ackRX[2]);
				Serial.print(" | CRC=");
				Serial.println(ackRX[3]);


				//ackRX[2] = ackRX[2] & 0xDF; // Generate a CRC Error for testing

				byte checkCRC = CalculateCRC(ackRX, sizeof(ackRX));
				if (checkCRC == 0) {
					Serial.println("************************\n");
					Serial.println("****** CRC OK! *******\n");
					Serial.println(checkCRC);
					Serial.println("************************\n");
				}
				else {
					Serial.println("##########################\n");
					Serial.println("##### CRC ERROR #####\n");
					Serial.println(checkCRC);
					Serial.println("##########################\n");
				}
			}
			else {
				Serial.print("ESP8266 - Error parsing ");
				Serial.print(cmdTX[0]);
				Serial.print(" command! <<< ");
				Serial.println(ackRX[0]);
			}
			//free(ackRX);
			break;
		}
				  // *******************
				  // * Unknown Command *
				  //********************      
		default: {
			Serial.print("ESP8266 - Command '");
			Serial.print(key);
			Serial.println("' unknown ...");
			break;
		}
		}
		Serial.println("");
		Serial.println("Please type a command ('a', 's', 'd' or 'f'):");
		//Serial.println("DLY*");
		//delay(2000);
	}
	ReadChar();
}

// Function ScanI2C
byte scanI2C() {
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
			delay(1000);
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
		crc = (byte)(crcTable[data]); // Get current CRC value = remainder 
	}
	return crc;
}

// Function Clear Screen
void clrscr() {
	Serial.write(27);       // ESC command
	Serial.print("[2J");    // clear screen command
	Serial.write(27);       // ESC command
	Serial.print("[H");     // cursor to home command
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
	byte dataLength = 16;
	char serialData[dataLength]; // an array to store the received data  
	static byte ix = 0;
	char rc, endMarker = 0xD; //standard is: char endMarker = '\n'
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
			serialData[ix] = '\0'; // terminate the string
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
