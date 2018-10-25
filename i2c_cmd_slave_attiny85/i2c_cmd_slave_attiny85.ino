// ********************************************************
// *  I2C Slave ATtiny85                                  *
// *  ==================                                  *
// *  Basic TX-RX command parser for I2C tests            *
// *  ..................................................  *
// *  Author: Gustavo Casanova                            *
// *  ..................................................  *
// *  Firmware Version: 0.2 | MCU: ATtiny85               *
// *  2017-10-03 gustavo.casanova@nicebots.com            *
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
// g - (GET_INFO) Get useful information regarding various slave parameters.
//
// Connections:
// ATtiny Pin 5 (PB0) <---> ESP8266 Pin  (NodeMCU D3) SDA
//     connect to SDA on master with external pull-up (~4.7K)
// ATtiny Pin 7 (PB0/SCL) = I2C SCL
//     connect to SCL on master with external pull-up (~4.7K)
// Optional:
// AtTiny Pin 1 (PB5/!RST)
//     connect to reset on master (or just pull-up)
//
// Tested fuse configurations:
//  8 MHz clock (RC):  Low=0xE2, High=0xDD, Extended=0xFE
// 16 MHz clock (PLL): Low=0xE1, High=0xDD, Extended=0xFE
//

//extern "C" {
//#include "usiTwiSlave.h"
#include <avr/wdt.h>
#include <stdbool.h>
//}


  void    usiTwiSlaveInit(uint8_t);
  void    usiTwiTransmitByte(uint8_t);
  uint8_t usiTwiReceiveByte(void);
  //bool    usiTwiDataInReceiveBuffer(void);
  void(*_onTwiDataRequest)(void);
  bool    usiTwiDataInTransmitBuffer(void);
  uint8_t usiTwiAmountDataInReceiveBuffer(void);
  // on_XXX handler pointers
  void(*usi_onRequestPtr)(void);
  void(*usi_onReceiverPtr)(uint8_t);
  // permitted RX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256
  #ifndef TWI_RX_BUFFER_SIZE
      #define TWI_RX_BUFFER_SIZE  ( 16 )
   #endif
   #define TWI_RX_BUFFER_MASK  ( TWI_RX_BUFFER_SIZE - 1 )
   #if ( TWI_RX_BUFFER_SIZE & TWI_RX_BUFFER_MASK )
      #  error TWI RX buffer size is not a power of 2
   #endif
      // permitted TX buffer sizes: 1, 2, 4, 8, 16, 32, 64, 128 or 256
   #ifndef TWI_TX_BUFFER_SIZE
      #define TWI_TX_BUFFER_SIZE ( 16 )
   #endif
   #define TWI_TX_BUFFER_MASK ( TWI_TX_BUFFER_SIZE - 1 )
   #if ( TWI_TX_BUFFER_SIZE & TWI_TX_BUFFER_MASK )
      #  error TWI TX buffer size is not a power of 2
      #endif








// Type definitions
typedef uint8_t byte;
typedef uint16_t word;

// Defines
#define I2C_ADDR 0x30
#define LED_PIN PB1
#define TOGGLETIME 0xFFFF  // Pre-init led toggle delay

// I2C Command Set
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
#define INITTINY 0x01     // Command to Initialize ATtiny85
#define RESETINY 0x02     // Command to Reset ATtiny85
#define ACKRESTY 0xFD     // Acknowledge Command Reset

// Global variables
byte command[4] = { 0 };              // Command received from master
byte commandLength = 0;               // Command number of bytes
volatile word analogValue = 0;		    // ADC value
volatile word ledToggleTimer = 0;	    // Pre-init led toggle timer
volatile bool initialized = false;    // Keeps status of initialization by master
volatile bool resetNow = false;

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
// ***************************
// * Setup Block (Runs once) *
// ***************************
//
void setup() {
  // Disable watchdog to avoid continuous loop after reset
  MCUSR = 0;
  WDTCR = 1 << WDCE | 1 << WDE;
  WDTCR = 1 << WDP2 | 1 << WDP1 | 1 << WDP0;
  // Fixing Clock at 8 Mhz - Handle with care!
  //cli();
  //CLKPR = 0x80;
  //CLKPR = 0x00;
  // Initialize I2C
  UsiTwiSlaveInit(I2C_ADDR);
  Usi_onReceiverPtr = receiveEvent;
  Usi_onRequestPtr = requestEvent;
  // Set pin LED_PIN for output
  DDRB |= (1 << LED_PIN);
  // Enable Interrupts
  sei();
}

// **********************************
// * Main Loop, (Runs continuously) *
// **********************************
//
void loop() {
  for (;;) {
    // TODO
    if (initialized == false) {
      // =============================
      // = Not initialized by master =
      // =============================
      // Blinks on every main loop pass at TOGGLETIME interval
      if (ledToggleTimer++ >= TOGGLETIME) {
        PORTB ^= (1 << LED_PIN);
        ledToggleTimer = 0;
      }
    }
    if (resetNow == true) {
      wdt_enable(WDTO_2S);
      for (;;) {
        for (word i = 0; i < (TOGGLETIME - 0xFFF); i++);
        PORTB ^= (1 << LED_PIN);
      }
    }
  }
}

// Receive event
void receiveEvent(byte commandbytes) {
  // save the number of bytes sent from the master
  commandLength = commandbytes;
  // store the data from the master into the data buffer
  for (byte i = 0; i < commandLength; i++) {
    command[i] = UsiTwiReceiveByte();
  }
}

// Request event
void requestEvent(void) {
  byte opCodeAck = ~command[0];	// Command Operation Code acknowledge => Command Bitwise "Not".
  switch (command[0]) {
    // ******************
    // * STDPB1_1 Reply *
    // ******************
  case STDPB1_1: {
    const byte ackLng = 1;
    byte acknowledge[1] = { 0 };
    acknowledge[0] = opCodeAck;
    TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from LED_PIN I/O pin.
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
    PORTB |= (1 << PB1);  // turn PB1 on (led pin)
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    break;
  }
                 // ******************
                 // * STDPB1_0 Reply *
                 // ******************
  case STDPB1_0: {
    const byte ackLng = 1;
    byte acknowledge[1] = { 0 };
    acknowledge[0] = opCodeAck;
    TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from LED_PIN I/O pin.
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
    PORTB &= ~(1 << PB1); // turn PB1 off (led pin)
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    break;
  }
                 // ******************
                 // * STANAPB3 Reply *
                 // ******************
  case STANAPB3: {
    const byte ackLng = 2;
    byte acknowledge[2] = { 0 };
    acknowledge[0] = opCodeAck;
    acknowledge[1] = CalculateCRC(command, 3);
    switch (command[1]) {
    case 0: {
      TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from LED_PIN I/O pin.
      TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
      PORTB &= ~(1 << LED_PIN); // turn LED_PIN pin off
      break;
    }
    case 255: {
      TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from LED_PIN I/O pin.
      TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
      PORTB |= (1 << LED_PIN);  // turn LED_PIN pin on
      break;
    }
    default: {
      TCCR0A |= ((1 << WGM01) | (1 << WGM00));  // Set timer 0 fast PWM mode
      TCCR0B |= (1 << CS01);  // Set timer 0 prescaler = CPUclk / 8
      TCCR0A |= (1 << COM0B1);  // Connect timer-controlled OC0B output compare pins to override LED_PIN I/O pin
      OCR0B = command[1]; // Set PWM duty cycle with STANAPB3 operand value
      break;
    }
    }
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    break;
  }
                 // ******************
                 // * READADC2 Reply *
                 // ******************
  case READADC2: {
    const byte ackLng = 4;
    byte analogMSB = 0, analogLSB = 0;
    analogMSB = ((analogValue >> 8) & 0x03);
    analogLSB = (analogValue & 0x0FF);
    byte acknowledge[4] = { 0 };
    acknowledge[0] = opCodeAck;
    acknowledge[1] = analogMSB;
    acknowledge[2] = analogLSB;
    acknowledge[3] = CalculateCRC(acknowledge, ackLng - 1); // Prepare CRC for Reply
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    break;
  }
                 // ******************
                 // * GET_INFO Reply *
                 // ******************
  case GET_INFO: {
    const byte ackLng = 16;
    byte acknowledge[16] = { 0 };
    acknowledge[0] = opCodeAck;
    acknowledge[1] = 71;	// G
    acknowledge[2] = 85;	// U
    acknowledge[3] = 83;	// S
    acknowledge[4] = 84;	// T
    acknowledge[5] = 65;	// A
    acknowledge[6] = 86;	// V
    acknowledge[7] = 79;	// O
    acknowledge[8] = 42;	// *
    acknowledge[9] = 42;	// *
    acknowledge[10] = 42;	// *
    acknowledge[11] = USICR;		// USICR USI Control Register
    acknowledge[12] = TCCR0A;		// TCCR0A Timer/Counter Control Register A value
    acknowledge[13] = TCCR0B;		// TCCR0B Timer/Counter Control Register A value
    acknowledge[14] = CLKPR;		// CLKPR Clock Prescale Register value
    acknowledge[15] = CalculateCRC(acknowledge, ackLng - 1); // Prepare CRC for Reply
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    break;
  }
                 // ******************
                 // * INITTINY Reply *
                 // ******************
  case INITTINY: {
    const byte ackLng = 1;
    byte acknowledge[1] = { 0 };
    acknowledge[0] = opCodeAck;
    //PORTB |= (1 << LED_PIN);  // turn LED_PIN on, it will be turned off by next toggle
    PORTB &= ~(1 << PB1); // turn PB1 off (led pin)
    initialized = true;
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    break;
  }
                 // ******************
                 // * RESETINY Reply *
                 // ******************
  case RESETINY: {
    const byte ackLng = 1;
    byte acknowledge[1] = { 0 };
    acknowledge[0] = opCodeAck;
    PORTB &= ~(1 << PB1); // turn PB1 off (led pin)
                          //initialized = false;
    for (byte i = 0; i < ackLng; i++) {
      UsiTwiTransmitByte(acknowledge[i]);
    }
    resetNow = true;
    break;
  }
                 // *************************
                 // * Unknown Command Reply *
                 // *************************
  default: {
    for (byte i = 0; i < commandLength; i++) {
      UsiTwiTransmitByte(UNKNOWNC);
    }
    break;
  }
  }
  // TinyWireS_stop_check();
}

// Function CalculateCRC (CRC-8)
byte CalculateCRC(byte* block, byte blockLength) {
  //int i;
  byte crc = 0, data = 0;
  for (byte i = 0; i < blockLength; i++) {
    data = (byte)(block[i] ^ crc);	// XOR-in next input byte
    crc = (byte)(crcTable[data]);	// Get current CRC value = remainder
  }
  return crc;
}
