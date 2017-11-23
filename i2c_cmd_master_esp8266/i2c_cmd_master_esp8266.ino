// ********************************************************
// *  I2C Master ESP8266                                  *
// *  ==================                                  *
// *  Basic TX-RX command parser for I2C tests            *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.3 | MCU: ESP8266                *
// *  2017-11-16 gustavo.casanova@nicebots.com            *
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
// z - (INITTINY) Reboot ESP-8266 and initialize ATtiny85
// x - (RESETINY) Reset ATtiny85

#include <Wire.h>

#define VCC 3.3			  // PSU VCC 3.3 Volts
#define ADCTOP 1023       // ADC Top Value @ 10-bit precision = 1023 (2^10)

#define STDPB1_1 0xE9     // Command to Set ATtiny85 PB1 = 1
#define AKDPB1_1 0x16     // Acknowledge Command PB1 = 1
#define STDPB1_0 0xE1     // Command to Set ATtiny85 PB1 = 0
#define AKDPB1_0 0x1E     // Acknowledge Command PB1 = 0
#define STANAPB3 0xFB     // Command to Set ATtiny85 PB3 = PWMx
#define ACKNAPB3 0x04     // Acknowledge Command PB3 = PWMx
#define READADC2 0xDA     // Command to Read ATtiny85 ADC2
#define ACKNADC2 0x25     // Acknowledge Command Read ADC2
#define GET_INFO 0x0D     // Command to Read Generic Info
#define ACK_GETI 0xF2     // Acknowledge Command Read Info
#define UNKNOWNC 0xFF     // Unknown Command Reply
#define INITTINY 0x01     // Command to initialize ATtiny85
#define RESETINY 0x02     // Command to Reset ATtiny85
#define ACKRESTY 0xFD     // Acknowledge Command Reset

//typedef uint8_t byte; //  8 bit data type
//typedef uint16_t word; // 16 bit data type

// Global Variables
byte slaveAddress = 0;
byte blockRXSize = 0;
bool newKey = false, newByte = false;
char key = '\0';
//long opcodeErrors = 0;    // This is for stress testing READADC2 only - REMOVE FOR PRODUCTION
//long loopsREADADC2 = 0;   // This is for stress testing READADC2 only - REMOVE FOR PRODUCTION

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

// Hex file to flash in ATtiny85
/*byte hexFile[238] = {
  0x3A, 0x31, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30,
  0x45, 0x43, 0x30, 0x31, 0x44, 0x43, 0x30, 0x31, 0x43, 0x43,
  0x30, 0x31, 0x42, 0x43, 0x30, 0x31, 0x41, 0x43, 0x30, 0x31,
  0x39, 0x43, 0x30, 0x31, 0x38, 0x43, 0x30, 0x31, 0x37, 0x43,
  0x30, 0x32, 0x43, 0x0D, 0x0A, 0x3A, 0x31, 0x30, 0x30, 0x30,
  0x31, 0x30, 0x30, 0x30, 0x31, 0x36, 0x43, 0x30, 0x31, 0x35,
  0x43, 0x30, 0x31, 0x34, 0x43, 0x30, 0x31, 0x33, 0x43, 0x30,
  0x31, 0x32, 0x43, 0x30, 0x31, 0x31, 0x43, 0x30, 0x31, 0x30,
  0x43, 0x30, 0x31, 0x31, 0x32, 0x34, 0x45, 0x36, 0x0D, 0x0A,
  0x3A, 0x31, 0x30, 0x30, 0x30, 0x32, 0x30, 0x30, 0x30, 0x31,
  0x46, 0x42, 0x45, 0x43, 0x46, 0x45, 0x35, 0x44, 0x32, 0x45,
  0x30, 0x44, 0x45, 0x42, 0x46, 0x43, 0x44, 0x42, 0x46, 0x31,
  0x30, 0x45, 0x30, 0x41, 0x30, 0x45, 0x36, 0x42, 0x30, 0x45,
  0x30, 0x35, 0x45, 0x0D, 0x0A, 0x3A, 0x31, 0x30, 0x30, 0x30,
  0x33, 0x30, 0x30, 0x30, 0x30, 0x31, 0x43, 0x30, 0x31, 0x44,
  0x39, 0x32, 0x41, 0x30, 0x33, 0x36, 0x42, 0x31, 0x30, 0x37,
  0x45, 0x31, 0x46, 0x37, 0x30, 0x32, 0x44, 0x30, 0x30, 0x37,
  0x43, 0x30, 0x45, 0x30, 0x43, 0x46, 0x41, 0x32, 0x0D, 0x0A,
  0x3A, 0x31, 0x30, 0x30, 0x30, 0x34, 0x30, 0x30, 0x30, 0x42,
  0x39, 0x39, 0x41, 0x39, 0x32, 0x45, 0x30, 0x38, 0x38, 0x42,
  0x33, 0x38, 0x39, 0x32, 0x37, 0x38, 0x38, 0x42, 0x42, 0x46,
  0x43, 0x43, 0x46, 0x46, 0x38, 0x39, 0x34, 0x46, 0x46, 0x43,
  0x46, 0x39, 0x38, 0x0D, 0x0A, 0x3A, 0x30, 0x30, 0x30, 0x30,
  0x30, 0x30, 0x30, 0x31, 0x46, 0x46, 0x0D, 0x0A
};*/

/*
3a 31 30 30 30 30 30 30 30 30 45 43 30 31 44 43
30 31 43 43 30 31 42 43 30 31 41 43 30 31 39 43
30 31 38 43 30 31 37 43 30 32 43 0d 0a 3a 31 30
30 30 31 30 30 30 31 36 43 30 31 35 43 30 31 34
43 30 31 33 43 30 31 32 43 30 31 31 43 30 31 30
43 30 31 31 32 34 45 36 0d 0a 3a 31 30 30 30 32
30 30 30 31 46 42 45 43 46 45 35 44 32 45 30 44
45 42 46 43 44 42 46 31 30 45 30 41 30 45 36 42
30 45 30 35 45 0d 0a 3a 31 30 30 30 33 30 30 30
30 31 43 30 31 44 39 32 41 30 33 36 42 31 30 37
45 31 46 37 30 32 44 30 30 37 43 30 45 30 43 46
41 32 0d 0a 3a 31 30 30 30 34 30 30 30 42 39 39
41 39 32 45 30 38 38 42 33 38 39 32 37 38 38 42
42 46 43 43 46 46 38 39 34 46 46 43 46 39 38 0d
0a 3a 30 30 30 30 30 30 30 31 46 46 0d 0a
*/

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
  Serial.println("Nicebots I2C Command Test (v0.2)");
  Serial.println("================================");
  Serial.println("Please type a command ('a', 's', 'd', 'f', 'g', 'z' reboot or 'x' reset tiny):");
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
    switch (key) {
      // ********************
      // * STDPB1_1 Command *
      // ********************
      case 'a': case 'A': {
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
        break;
      }
      // ********************
      // * STDPB1_0 Command *
      // ********************
      case 's': case 'S': {
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
        break;
      }
      // ********************
      // * STANAPB3 Command *
      // ********************
      case 'd': case 'D': {
        byte cmdTX[3] = { STANAPB3, 0, 0 };
        //byte txSize = sizeof(cmdTX);
        byte txSize = 3;
        byte operandValue = 0;
        Serial.print("Please enter a value between 0 and 255 for this command: ");
        while (newByte == false) {
          operandValue = ReadByte();
        }
        if (newByte == true) {
          Serial.println("");
          Serial.println("");
          cmdTX[1] = operandValue;
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
        newByte = false;
        }
        // Receive acknowledgement
        blockRXSize = Wire.requestFrom(slaveAddress, (byte)2);
        byte ackRX[2] = { 0 };   // Data received from slave
        for (int i = 0; i < blockRXSize; i++) {
          ackRX[i] = Wire.read();
        }
        if (ackRX[0] == ACKNAPB3) {
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
        break;
      }
      // ********************
      // * READADC2 Command *
      // ********************
      case 'f': case 'F': {
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
        break;
      }
      // ************************************
      // * GET_INFO Command (16 byte reply) *
      // ************************************
      case 'g': case 'G': {
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
          //for (int i = 1; i < blockRXSize - 1; i++) {   // ----->
          for (int i = 1; i < 6 - 1; i++) {              // ----->
            Serial.print("ESP8266 - Data Byte ");
            if (i < 9) {
              Serial.print("0");
            }
            Serial.print(i + 1);
            Serial.print(" received OK <<< ");
            Serial.print((char)ackRX[i]);
            Serial.print(" (numeric: ");
            Serial.print(ackRX[i]);
            Serial.println(")");
          }
          // --------------------------------------------------
          // Sum of Vi (instantaneous voltages)
          Serial.print("# <>* Sum of Vi's: ");
          Serial.print((unsigned)(ackRX[5] << 24) + (ackRX[6] << 16) + (ackRX[7] << 8) + ackRX[8]);
          Serial.print(" (");
          Serial.print(((ackRX[5] << 24) + (ackRX[6] << 16) + (ackRX[7] << 8) + ackRX[8]), HEX);
          Serial.println(") *<>");
          // --------------------------------------------------
          // Vrms
		  uint16_t vRMS = (ackRX[9] << 8) + ackRX[10];
		  float volts = (vRMS * VCC) / ADCTOP;
          Serial.print("# {}* Vrms (AC): ");
          Serial.print(vRMS);
          Serial.print(" *{} ---> { ");
		  Serial.print(volts, 3);
		  Serial.println(" Volts RMS }");
          // --------------------------------------------------
          // Analog value (DC)
          Serial.print("# []* Analog Value (DC): ");
          Serial.print((ackRX[11] << 8) + ackRX[12]);
          Serial.println(" *[]");
          // --------------------------------------------------
          // ADC conversions per AC half-cycle
          Serial.print("# ~~~ ADC count: ");
          Serial.print((ackRX[13] << 8) + ackRX[14]);
          Serial.println(" ~~~ #");
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
    Serial.println("Please type a command ('a', 's', 'd', 'f', 'g', 'z' reboot or 'x' reset tiny):");
  }
  ReadChar();           // PROD - REMOVE FOR TESTING
  //delay(150);         // TEST - REMOVE FOR PRODUCTION
  //key = 'f';          // TEST - REMOVE FOR PRODUCTION
  //newKey = true;      // TEST - REMOVE FOR PRODUCTION
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
    crc = (byte)(crcTable[data]); // Get current CRC value = remainder 
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

// Function Clear Screen
void ClrScr() {
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);       // ESC command
  Serial.print("[H");     // cursor to home command
}
