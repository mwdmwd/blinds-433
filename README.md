# blinds-433
Firmware for a remote control transceiver for outdoor roller blinds/shades/???. Runs on an ATmega328P (i.e. Arduino Nano or similar) and acts as a bidirectional bridge between radio and serial.

Serial protocol description is in `main.c`. It works the same in both directions. You'll *probably* also need to change the header in `blinds.c`. A Universal Radio Hacker project file and example captures are provided to assist with troubleshooting (the decoder to use is called "Long 1_3").