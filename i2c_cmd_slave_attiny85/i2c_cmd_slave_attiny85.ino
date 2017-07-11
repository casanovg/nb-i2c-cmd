// ********************************************************
// *  I2C Slave ATtiny85                                  *
// *  ==================                                  *
// *  I2C Slave Stress Test                               *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.1 | MCU: ATtiny85               *
// *  2017-06-15 gustavo.casanova@nicebots.com            *
// ********************************************************
//
// Run this slave program on a Digispark or ATtiny85
// Run the master program on a NodeMCU, ESP-01 or ESP-12 Module
//
// The slave program using TinyWireS, running on a Attiny85, receives
// N bytes of random data in a single receiveEvent() callback and
// stores that data in a global buffer. It then responds the first requestEvent()
// callback with that same data. The requestEvent() callback overwrites the data
// buffer with zeros after responding so it will only respond correctly to the
// first requestEvent() callback after each receiveEvent() callback. Subsequent
// requestEvent() will respond with 0xff for all data bytes.
//
//
// SETUP:
// AtTiny Pin 5 (PB0/SDA) = I2C SDA
//     connect to SDA on master with external pull-up (~4.7K)
// AtTiny Pin 7 (PB2/SCL) = I2C SCL
//     connect to SCL on master with external pull-up (~4.7K)
// AtTiny Pin 1 (PB5/!RST)
//     connect to reset on master (or just pull-up)
//
// Please see credits and usage for usiTwiSlave and TinyWireS in the .h files of
// those libraries.

#include "TinyWireS.h"                  // wrapper class for I2C slave routines

#define I2C_SLAVE_ADDR 0x2E             // I2C slave address
#define LED_PIN 1

// global buffer to store data sent from the master.
byte master_data[16];

// global variable to number of bytes sent from the master.
int master_bytes;

// Gets called when the ATtiny receives an i2c write slave request
void receiveEvent(byte num_bytes)
{
	int i;

	// save the number of bytes sent from the master
	master_bytes = num_bytes;

	// store the data from the master into the data buffer
	for (i = 0; i < master_bytes; i++)
		master_data[i] = TinyWireS.receive();

}

// Gets called when the ATtiny receives an i2c read slave request
void requestEvent()
{
	int i;

	// send the data buffer back to the master
	for (i = 0; i < master_bytes; i++) {
		TinyWireS.send(~master_data[i]);
	}

	// corrupt the byte values in the data buffer
	// so that subsequent call won't match
	for (i = 0; i < master_bytes; i++) {
		master_data[i] += 0x5a;
	}
	// corrupt length of the request, but dont' make it zero
	master_bytes = 2;
}

void setup()
{
	// initialize the TinyWireS and usiTwiSlave libraries
	TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode

										  // register the onReceive() callback function
	TinyWireS.onReceive(receiveEvent);

	// register the onRequest() callback function
	TinyWireS.onRequest(requestEvent);
}

void loop() {
	heartbit();
	// This needs to be here
	TinyWireS_stop_check();
	// otherwise empty loop
}

// Function Heartbit
void heartbit() {
	digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(500);                    // wait for a second
	digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
	delay(500);                    // wait for a second
}

/*
// ********************************************************
// *  I2C Slave ATtiny85                                  *
// *  ==================                                  *
// *  I2C Slave Stress Test                               *
// *  ..................................................  *
// *  (c)2017 Nicebots.com                                *
// *  ..................................................  *
// *  Firmware Version: 0.1 | MCU: ATtiny85               *
// *  2017-06-15 gustavo.casanova@nicebots.com            *
// ********************************************************
//
// Run this slave program on a Digispark or ATtiny85
// Run the master program on a NodeMCU, ESP-01 or ESP-12 Module
//
// Basic command path to Attiny85:
// User (serial console) --> ESP8266 --> Attiny85
//
// Available commands:
// a - (STDPB1_1) Set ATtiny85 PB1 = 1
// s - (STDPB1_0) Set ATtiny85 PB1 = 0
// d - (STANAPB3) Set ATtiny85 PB3 = PWMx (the command asks for a PWM value input)
// f - (READADC2) Read ATtiny85 ADC2 (the reply has 2 data bytes + 1 CRC byte)
//
// Setup:
// AtTiny Pin 5 (PB0/SDA) = I2C SDA
//     connect to SDA on master with external pull-up (~4.7K)
// AtTiny Pin 7 (PB0/SCL) = I2C SCL
//     connect to SCL on master with external pull-up (~4.7K)
// AtTiny Pin 1 (PB5/!RST)
//     connect to reset on master (or just pull-up)
//

#include "TinyWireS.h"                  // wrapper class for I2C slave routines

#define I2C_SLAVE_ADDR 0x2E             // I2C slave address
#define LED_PIN 1

#define STDPB1_1 0xE9 // Command to Set ATtiny85 PB1 = 1
#define AKDPB1_1 0x16 // Acknowledge Command PB1 = 1

#define STDPB1_0 0xE1 // Command to Set ATtiny85 PB1 = 0
#define AKDPB1_0 0x1E // Acknowledge Command PB1 = 0

#define STANAPB3 0xFB // Command to Set ATtiny85 PB3 = PWMx
#define ACKNAPB3 0x04 // Acknowledge Command PB3 = PWMx

#define READADC2 0xDA // Command to Read ATtiny85 ADC2
#define ACKNADC2 0x25 // Acknowledge Command Read ADC2

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

// global buffer to store data sent from the master.
byte command[4] = {0};

// global variable to number of bytes sent from the master.
int commandLength = 0;

// Setup block
void setup() {
// initialize the TinyWireS and usiTwiSlave libraries
TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode
// register the onReceive() callback function
TinyWireS.onReceive(receiveEvent);
// register the onRequest() callback function
TinyWireS.onRequest(requestEvent);
}

// Main loop
void loop() {
//heartbit();
// This needs to be here
TinyWireS_stop_check();
// otherwise empty loop
}

// Gets called when the ATtiny receives an i2c write slave request
void receiveEvent(byte receivedLength) {
int i;
// save the number of bytes sent from the master
commandLength = receivedLength;
// store the data from the master into the data buffer
for (i = 0; i < commandLength; i++) {
command[i] = TinyWireS.receive();
}
}

// Gets called when the ATtiny receives an i2c read slave request
void requestEvent() {

//int i;

// send the data buffer back to the master
//for (i = 0; i < master_bytes; i++)
//  TinyWireS.send(~command[i]); // ######## GC COMMAND ACKNOWLEDGE !!! ########

int i;
byte opCodeAck = ~command[0]; // Command Operation Code acknowledge => Command Bitwise "Not".
switch (command[0]) {
case STDPB1_1: {
byte ackLng = 1;
byte acknowledge[1] = {0};
acknowledge[0] = opCodeAck;
digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
for (i = 0; i < ackLng; i++) {
TinyWireS.send(acknowledge[i]);
}
break;
}
case STDPB1_0: {
byte ackLng = 1;
byte acknowledge[1] = {0};
acknowledge[0] = opCodeAck;
digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
for (i = 0; i < ackLng; i++) {
TinyWireS.send(acknowledge[i]);
}
break;
}
case STANAPB3: {
byte ackLng = 2;
byte acknowledge[2] = {0};
acknowledge[0] = opCodeAck;
acknowledge[1] = ~command[1];
//acknowledge[1] = 0x33;
for (i = 0; i < ackLng; i++) {
TinyWireS.send(acknowledge[i]);
}
break;
}
case READADC2: {
byte ackLng = 4, analogMSB = 0, analogLSB = 0;
//word analogRead = 0;
word analogRead = rand() % (1023 + 1 - 0) + 0;
//if (analogVal < 1024) {
//    analogRead = ++analogVal;
//} else {
//    analogVal = 0;
//}
analogMSB = ((analogRead >> 8) & 0x03);
analogLSB = (analogRead & 0x0FF);
word analogValue = ((analogMSB << 8) + analogLSB);
byte acknowledge[4] = {0};
acknowledge[0] = opCodeAck;
acknowledge[1] = analogMSB;
acknowledge[2] = analogLSB;
acknowledge[3] = CalculateCRC(acknowledge, ackLng - 1); // Prepare CRC for Reply
for (i = 0; i < ackLng; i++) {
TinyWireS.send(acknowledge[i]);
}
break;
}
default: {
byte acknowledge[1] = {0};
acknowledge[0] = opCodeAck;
break;
}
}
}

// Function Heartbit
void heartbit() {
digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
delay(500);                    // wait for a second
digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
delay(500);                    // wait for a second
}

// Function CalculateCRC (CRC-8)
byte CalculateCRC(byte* block, size_t blockLength) {
int i;
byte crc = 0, data = 0;
for (i = 0; i < blockLength; i++) {
data = (byte) (block[i] ^ crc); // XOR-in next input byte
crc = (byte) (crcTable[data]); // Get current CRC value = remainder
}
return crc;
}
*/