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



#include "TinyWireS.h"                  // wrapper class for I2C slave routines
//#include "usiTwiSlave.h"

#define I2C_SLAVE_ADDR 0x2F             // I2C slave address (47, can be changed) 0x2F

#define LED_PIN PB1       // Output Pin for STDPB1_1 command
#define PB3 3             // Output Pin for STANAPB3 command
#define AD2 A2            // Input Pin for READADC2 command
#define ADR 1023.0        // ADC Resolution (10 bit = 2^10)
#define SAMPLETIME 500    // Sampling time in ms per reading
#define TOGGLETIME 0xFF   // Pre-init led toggle delay

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

// Global Variables
bool testReplies = false;           // Activates test mode
bool initialized = false;           // Keeps status of initialization by master
byte command[4] = { 0 };            // Command received from master
int commandLength = 0;              // Command number of bytes
volatile uint16_t analogValue = 0;  // ADC value
uint8_t ledToggleTimer = 0;         // Pre-init led toggle timer
bool ledOnStatus = false;           // Pre-init led status

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
	// initialize the TinyWireS and usiTwiSlave libraries
	//TinyWireS.begin(I2C_SLAVE_ADDR);      // init I2C Slave mode

  TinyWireS.begin(I2C_SLAVE_ADDR);

	// register the onReceive() callback function
	TinyWireS.onReceive(receiveEvent);
	// register the onRequest() callback function
	TinyWireS.onRequest(requestEvent);
  InitADC();
  DDRB |= (1 << LED_PIN); // Set pin PB1 for output
  //pinMode(PB3, OUTPUT);
  pinMode(AD2, INPUT);
}

// **********************************
// * Main Loop, (Runs continuously) *
// **********************************
//
void loop() {
  if (initialized == false) {
    // �������������������������������
    // �� Not initialized by master ��
    // �������������������������������
    // Blinks on every main loop pass at TOGGLETIME interval
    if (ledToggleTimer++ >= TOGGLETIME) {
      PORTB ^= (1 << LED_PIN);
      ledToggleTimer = 0;
    }
  }
  else {
    // ���������������������������
    // �� Initialized by master ��
    // ���������������������������
    if (testReplies == true) {
      Heartbeat(500);
    }
 }

	// This needs to be here
	//TinyWireS_stop_check();
	// otherwise empty loop

  analogValue = analogRead(AD2);                // Direct analog ADC read for DC
  //analogValue = GetVPP(AD2, SAMPLETIME);          // ADC analog function for AC
}

// Gets called when the ATtiny receives an I2C write slave request
void receiveEvent(byte commandbytes) {
	// save the number of bytes sent from the master
	commandLength = commandbytes;
	// store the data from the master into the data buffer


	for (int i = 0; i < commandLength; i++) {
		command[i] = TinyWireS.receive();
	}
}

// Gets called when the ATtiny receives an I2C read slave request
void requestEvent() {

	if (testReplies == false) {
    // ������������������
    // � Operating mode .
    // ������������������
    byte opCodeAck = ~command[0]; // Command Operation Code acknowledge => Command Bitwise "Not".
		switch (command[0]) {
			// ******************
			// * STDPB1_1 Reply *
			// ******************     
			case STDPB1_1: {
				const byte ackLng = 1;
				byte acknowledge[1] = { 0 };
				acknowledge[0] = opCodeAck;
        TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from PB1 I/O pin.
        TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
        PORTB |= (1 << PB1);  // turn PB1 pin on (Led pin)
				for (int i = 0; i < ackLng; i++) {
					TinyWireS.send(acknowledge[i]);
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
        TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from PB1 I/O pin.
        TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
        PORTB &= ~(1 << PB1); // turn PB1 pin off (Led pin)
				for (int i = 0; i < ackLng; i++) {
					TinyWireS.send(acknowledge[i]);
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
            TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from PB1 I/O pin.
            //TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
            PORTB &= ~(1 << PB1); // turn PB1 pin off (Led pin)
            break;
          }
          case 255: {
            TCCR0A &= ~(1 << COM0B1); // Disconnects timer-controlled OC0B output compare pins from PB1 I/O pin.
            //TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00)); // Turn off timer PWM
            PORTB |= (1 << PB1);  // turn PB1 pin on (Led pin)
            break;
          }
          default: {
            //TCCR0A |= ((1 << WGM01) | (1 << WGM00));  // Set timer 0 fast PWM mode
            TCCR0B |= (1 << CS01);  // Set timer 0 prescaler = CPUclk / 8
            TCCR0A |= (1 << COM0B1);  // Connect timer-controlled OC0B output compare pins to override PB1 I/O pin
            OCR0B = command[1]; // Set PWM duty cycle with STANAPB3 operand value
            break;
          }
        }
				for (int i = 0; i < ackLng; i++) {
					TinyWireS.send(acknowledge[i]);
				}
				break;
			}
			// ******************
			// * READADC2 Reply *
			// ******************
			case READADC2: {
        const byte ackLng = 4;
        byte analogMSB = 0, analogLSB = 0;
        // analogValue = analogRead(AD2);
        // if (analogValue < 1024) {
        //   analogValue++;
				// } else {
				//   analogValue = 0;
				// }
        analogMSB = ((analogValue >> 8) & 0x03);
				analogLSB = (analogValue & 0x0FF);
				word analogValue = ((analogMSB << 8) + analogLSB);
				byte acknowledge[ackLng] = { 0 };
				acknowledge[0] = opCodeAck;
				acknowledge[1] = analogMSB;
				acknowledge[2] = analogLSB;
				acknowledge[3] = CalculateCRC(acknowledge, ackLng - 1); // Prepare CRC for Reply
        // int g = 0;                                // TEST - REMOVE FOR PRODUCTION
        // while (g < 32500) {                       // TEST - REMOVE FOR PRODUCTION
        //  g++;                                     // TEST - REMOVE FOR PRODUCTION
        // }                                         // TEST - REMOVE FOR PRODUCTION
        // digitalWrite(LED_PIN, LOW);                   // TEST - REMOVE FOR PRODUCTION
				for (int i = 0; i < ackLng; i++) {
					TinyWireS.send(acknowledge[i]);
          /*int g = 0;*/                            // TEST - REMOVE FOR PRODUCTION
          //while (g < 32500) {                     // TEST - REMOVE FOR PRODUCTION
          //  g++;                                  // TEST - REMOVE FOR PRODUCTION
          //}                                       // TEST - REMOVE FOR PRODUCTION
				}
				break;
			}
      // ******************
      // * GET_INFO Reply *
      // ******************
      case GET_INFO: {
        const byte ackLng = 16;
        byte acknowledge[ackLng] = { 0 };
        acknowledge[0] = opCodeAck;
        acknowledge[1] = 71;
        acknowledge[2] = 85;
        acknowledge[3] = 83;
        acknowledge[4] = 84;
        acknowledge[5] = 65;
        acknowledge[6] = 86;
        acknowledge[7] = 79;
        acknowledge[8] = 78;
        acknowledge[9] = 73;
        acknowledge[10] = 67;
        acknowledge[11] = 69;
        acknowledge[12] = 66;
        acknowledge[13] = 79;
        acknowledge[14] = 84;
        acknowledge[15] = CalculateCRC(acknowledge, ackLng - 1); // Prepare CRC for Reply
        for (int i = 0; i < ackLng; i++) {
          TinyWireS.send(acknowledge[i]);
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
        initialized = true;
        digitalWrite(LED_PIN, LOW);    // turn the LED off to show initialization, in case that the blink ended turned on
        for (int i = 0; i < ackLng; i++) {
          TinyWireS.send(acknowledge[i]);
        }
        break;
      }
			// *************************
			// * Unknown Command Reply *
			// *************************
			default: {
				//byte acknowledge[1] = { 0 };
				//acknowledge[0] = 0xFA;
        for (int i = 0; i < commandLength; i++) {
          //TinyWireS.send(~command[i]);
          TinyWireS.send(UNKNOWNC);
        }
				break;
			}
		}
    // TinyWireS_stop_check();
	}
	else {
    // �������������
		// � Test mode .
    // �������������
		// Just reply the command inverted (Not-command)
		for (int i = 0; i < commandLength; i++) {
			TinyWireS.send(~command[i]);
		}
	}
}

// Function CalculateCRC (CRC-8)
byte CalculateCRC(byte* block, size_t blockLength) {
	int i;
	byte crc = 0, data = 0;
	for (i = 0; i < blockLength; i++) {
		data = (byte)(block[i] ^ crc);	// XOR-in next input byte
		crc = (byte)(crcTable[data]);	// Get current CRC value = remainder
	}
	return crc;
}

// Function Heartbit
void Heartbeat(int blinkDelay) {
	digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
	delay(blinkDelay);                    // wait for a second
	digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
	delay(blinkDelay);                    // wait for a second
}

void InitADC() {

  /* this function initialises the ADC

  For more information, see table 17.5 "ADC Prescaler Selections" in
  chapter 17.13.2 "ADCSRA � ADC Control and Status Register A"
  (pages 140 and 141 on the complete ATtiny25/45/85 datasheet, Rev. 2586M�AVR�07/10)

  // 10-bit resolution
  // set ADLAR to 0 to disable left-shifting the result (bits ADC9 + ADC8 are in ADC[H/L] and
  // bits ADC7..ADC0 are in ADC[H/L])
  // use uint16_t variable to read ADC (intead of ADCH or ADCL)

  */

  ADMUX =
    (0 << ADLAR) |    // do not left shift result (for 10-bit values)
    (0 << REFS2) |    // Sets ref. voltage to internal 1.1V, bit 2
    (1 << REFS1) |    // Sets ref. voltage to internal 1.1V, bit 1   
    (0 << REFS0) |    // Sets ref. voltage to internal 1.1V, bit 0
    (0 << MUX3) |     // use ADC2 for input (PB4), MUX bit 3
    (0 << MUX2) |     // use ADC2 for input (PB4), MUX bit 2
    (1 << MUX1) |     // use ADC2 for input (PB4), MUX bit 1
    (0 << MUX0);      // use ADC2 for input (PB4), MUX bit 0
    //(0 << MUX3) |      // use ADC1 for input (PB2), MUX bit 3
    //(0 << MUX2) |      // use ADC1 for input (PB2), MUX bit 2
    //(0 << MUX1) |      // use ADC1 for input (PB2), MUX bit 1
    //(1 << MUX0);       // use ADC1 for input (PB2), MUX bit 0

  ADCSRA =
    (1 << ADEN) |     // Enable ADC 
    (1 << ADPS2) |     // set prescaler to 128, bit 2 
    (1 << ADPS1) |     // set prescaler to 128, bit 1 
    (1 << ADPS0);      // set prescaler to 128, bit 0  
}

// Function ReadADC
int AnalogRead2(void) {
  //initADC();
  uint8_t adc_lobyte; // to hold the low byte of the ADC register (ADCL)
  uint16_t raw_adc;
  //while (1) {
  ADCSRA |= (1 << ADSC);          // start ADC measurement
  while (ADCSRA & (1 << ADSC));   // wait till conversion complete 
                                  // for 10-bit resolution:
  adc_lobyte = ADCL; // get the sample value from ADCL
  raw_adc = ADCH << 8 | adc_lobyte;   // add lobyte and hibyte
  if (raw_adc > 512) {
    // ADC input voltage is more than half of the internal 1.1V reference voltage
  }
  else {
    // ADC input voltage is less than half of the internal 1.1V reference voltage
  }
  //}
  return 0;
}

// Fuction GetVPP
float GetVPP(int sensorPin, int sampleTime) {
  float result;
  float readValue;            //value read from the sensor
  float maxValue = 0;         // store max value, initialize with min
  float minValue = ADR;       // store min value, initialize with max
  uint32_t start_time = millis();
  while ((millis() - start_time) < sampleTime) {
    readValue = analogRead(sensorPin);
    if (readValue > maxValue) {
      maxValue = readValue;   // if there is a new maximum, record it 
    }
    if (readValue < minValue) {
      minValue = readValue;   // if there is a new minimum, record it
    }
  }
  //result = ((maxValue - minValue) * PSUVOLTAGE) / ADCRESOLUTION;
  result = (maxValue - minValue);
  return result;
}

// Function InitTimer
static inline void InitTimer(void) {
  TCCR0A |= (1 << WGM01); /* CTC mode */
  TCCR0A |= (1 << COM0A0); /* Toggles pin each cycle through */
  TCCR0B |= (1 << CS00) | (1 << CS01); /* CPU clock / 64 */
}