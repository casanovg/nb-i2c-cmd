// ********************************************************
// *  I2C Slave ATtiny85                                  *
// *  ==================                                  *
// *  I2C Stress Test                                     *
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
    TinyWireS.send(master_data[i]);
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
