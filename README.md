# Motivation

the shellylora was ok, but due to the local setup the signal strength was pretty weak, and at a better lora location wifi was not good. i.e. i had a lot of missing and corrupt data.   
this is an example of how to get simple lora receiver going using an esp32 p4 board and an sx1262 addon board

# Setup

 at first i tried a waveshare esp32 p4 nano board, but had problems with SPI, see below, then i used a waveshare esp32 p4 dev board with a POE hat
 
## hardware
 - waveshare ESP32-P4 POE ETH Entwicklungsboard mit PoE Modul, pinout https://www.waveshare.com/wiki/ESP32-P4-ETH?srsltid=AfmBOooyZypx7zT_zXIXvzyBKMioBOeLTpoagiUQXCFh9IVxTwZBkBLn#Pinout_Definition
 - waveshare sx1262 HF core board https://www.waveshare.com/wiki/Core1262-868M#Demo_Example
 - Nelawya 868MHz LoRa Antenne Omni-Directional 3dbi Gain SMA Male LoraWan Antenne

## software
base is the esp32bme code, but added ethernet capability and the lora receiver code.
yes, the code needs a lot of cleanup, this is a prototype proof of concept and has a lot of other esp32 sensor code still included, it uses
- arduino ide for development, v2.3.7
- [RadioLib](https://github.com/jgromes/RadioLib) 7.5.0 for the communication with the sx1262 board

it took quite a while to get it working, since at first i had tried a waveshare p4 nano board https://www.waveshare.com/wiki/ESP32-P4-Nano-StartPage#Pinout_Definition 
but did not have any luck getting it to work. BUSY stayed high, and despite attempts to find docs/examples and asking both chatgpt and claude vor suggestions i frist tried
the sx1262 on an esp32s3 board and lora worked without problems, and then move to the esp32 p4 board since at the location where the lora reception was good i had POE.

## signal
at the same location as the shelly lora addon reciever, here is a comparison, rough values, off course they fluctuate, but def. a lot better

- shelly add on   RSSI: -118 dBm, SNR: -11 dB
- esp32 p4 setup: RSSI: -105 dBm, SNR: -1,5 dB

time will tell how stable this is, and how much additional data filtering i have to add to the HA sensors where the data are consumed.

## notes
- do NOT run the lora rcv task on core 0, you will get errors
  
 >E (22096) task_wdt: Task watchdog got triggered. The following tasks/users did not reset the watchdog in time:  
 >E (22096) task_wdt:  - IDLE0 (CPU 0)`

# todo
- figure out a working set of pins on the nano board
