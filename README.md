ESP8266 ATtiny85 I2C Communications
===================================
This test implements a basic four I2C command set to control an ATtiny85 MCU from an ESP8266 through the serial console. In those commands where there are data replies, there is a plain CRC-8 check implemented.

Command path to Attiny85:
-------------------------------
User (serial console) --> ESP8266 --> Attiny85

Available commands:
-------------------
a - (STDPB1_1) Set ATtiny85 PB1 = 1

s - (STDPB1_0) Set ATtiny85 PB1 = 0

d - (STANAPB3) Set ATtiny85 PB3 = PWMx (the command asks for a PWM value input)

f - (READADC2) Read ATtiny85 ADC2 (the reply has 2 data bytes + 1 CRC byte)

Test mode:
----------
Loop continuously with command 'd' (STANAPB3) or 'f' (READADC2).

Errors simulated:
-----------------
Mask 0xDF applied to operand 2 of READADC2 command received in ESP8266

Mask 0xEF applied to operand of STANAPB3 command received in ATtiny85
