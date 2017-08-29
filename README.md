ESP8266 ATtiny85 I2C Communications
===================================
This program finds the max amount of bytes that can be exchanged within one transmission block among a master and a slave.

Steps:
------
1-Scans I2C addresses looking for an ATtiny85 slaves.

2-Loops through a series of incremental quantity of random bytes
  that are sent to the slave, starting with 1.

3-If the slave answers the same random value correctly, it
  increments to 2 random bytes for the next loop, an so on.

4-If a wrong answers is found, it decrements 1 byte until is
  stable again.

5-Reports the maximum safe transmission block size.

