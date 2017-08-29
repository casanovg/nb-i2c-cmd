// ********************************************************
// *  I2C Master ESP8266                                  *
// *  ==================                                  *
// *  I2C Stress Test                                     *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.1 | MCU: ESP8266                *
// *  2017-06-15 gustavo.casanova@nicebots.com            *
// ********************************************************
//
// Run this master program on a NodeMCU, ESP-01 or ESP-12 Module
// Run the slave program on a Digispark or ATtiny85
//
// This program finds the max amount of bytes that can be exchanged
// within one transmission block among a master and a slave:
//   1-Scans I2C addresses looking for an ATtiny85 slaves.
//   2-Loops through a series of incremental quantity of random bytes
//     that are sent to the slave, starting with 1.
//   3-If the slave answers the same random value correctly, it
//     increments to 2 random bytes for the next loop, an so on.
//   4-If a wrong answers is found, it decrements 1 byte until is
//     stable again.
//   5-Reports the maximum safe transmission block size.
//

#include <Wire.h>

#define SLAVE_RESET_PIN 2

byte slaveAddress = 0;

//**************************
//* Setup Block, Runs Once *
//**************************
//
void setup() {
  pinMode(SLAVE_RESET_PIN, OUTPUT); // Set pin modes
  Serial.begin(115200); // Init the serial port
                        // Init the Wire object for I2C
  Wire.begin(); // Standard pins SDA on D2 and SCL on D1 (NodeMCU)
                //Wire.begin(D3, D4); // Set SDA on D3 and SCL on D4 (NodeMCU)
  digitalWrite(SLAVE_RESET_PIN, LOW); // Reset the slave
  delay(10);
  digitalWrite(SLAVE_RESET_PIN, HIGH);
  delay(2000); // Wait 2 seconds for slave init sequence
               // Search continuouly for slave addresses
  while (slaveAddress == 0) {
    slaveAddress = scanI2C();
    delay(1000);
  }
}
int blockTXSize = 1;
int blockRXSize = 0;
long blockNumber = 1;
int cntRXErrors = 0;
int maxBlockSize = blockTXSize;
int accRXErrors = cntRXErrors;
bool possibleLimit = true;

//**************************************
//* Main Loop Block, Runs Continuously *
//**************************************
//
void loop() {

  int i;
  byte masterTXData[blockTXSize];  // Data to be transmitted by master
  byte masterBuffer[blockTXSize];  // Data transmission data

  clrscr();
  Serial.print("Starting transmission block ");
  Serial.print(blockNumber);
  Serial.println(" ...");
  Serial.print("TX Block Size: ");
  Serial.print(blockTXSize);
  Serial.print(" (Max: ");
  Serial.print(maxBlockSize - 1);
  Serial.println(")");
  Serial.print("RX Errors: ");
  Serial.print(cntRXErrors);
  Serial.print(" (Acc: ");
  Serial.print(accRXErrors);
  Serial.println(")");

  // Transmission data block
  for (int i = 0; i < sizeof(masterTXData); ++i) {
    masterTXData[i] = 0;
  }
  for (i = 0; i < blockTXSize; i++) {
    masterTXData[i] = random(256);
    masterBuffer[i] = masterTXData[i];
    Wire.beginTransmission(slaveAddress);
    Wire.write(masterBuffer[i]);
    Wire.endTransmission();
  }

  //delay(10);

  // Reception data block
  blockRXSize = Wire.requestFrom(slaveAddress, (int)blockTXSize);
  byte slaveRXData[blockRXSize];   // Data received from slave
  for (i = 0; i < blockTXSize; i++) {
    slaveRXData[i] = Wire.read();
  }

  // Compare RX vs TX
  cntRXErrors = 0;
  for (i = 0; i < blockTXSize; i++) {
    if (slaveRXData[i] != masterTXData[i]) {
      cntRXErrors++;
    }
  }

  /*
  Serial.println("TX|RX Data:");
  for (i = 0; i < blockTXSize; i++) {
  Serial.print("[");
  Serial.print(masterTXData[i]);
  Serial.print("|");
  Serial.print(slaveRXData[i]);
  Serial.print("] ");
  }
  Serial.println("");
  */

  delay(10);

  if (cntRXErrors == 0) {
    blockTXSize++;
    maxBlockSize = blockTXSize;
  }
  else {
    blockTXSize -= cntRXErrors;
    if (blockTXSize < 1) {
      blockTXSize = 1;
      accRXErrors += cntRXErrors;
    }
  }


  blockNumber++;
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

// Function Clear Screen
void clrscr() {
  Serial.write(27);       // ESC command
  Serial.print("[2J");    // clear screen command
  Serial.write(27);       // ESC command
  Serial.print("[H");     // cursor to home command
}
