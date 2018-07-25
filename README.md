ATtiny85 I2C Bootloader Tests
=============================
Timonel I2C Bootloader development tests ...

Timonel-Bootloader --> ATtiny85 (I2C slave)

Timonel-Uploader   --> ESP8266 (I2C master)

Test Payload: "SOS" blinker for Tiny85. Initially loaded from an ESP8266 array, in the final version it will be parsed from a real ".HEX" file residing on SPIFFS.
