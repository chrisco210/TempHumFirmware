# TempHum Firmware
There are three versions of the temphum firmware in this repo:

## TempHum
This is the simplest firmware, with a timeout in between cycles.

## TempHumOptimized
This is an optimzed version with a sleep fuction as well as a decreased transmission interval.

## TempHumShutdown
This firmware enables functionality with a external power switch, and only transmits once.  This is the "final" version of the firmware

## TempHumShutdown WiFi
This firmware does the same thing as TempHumShutdown, but uses wifi to send data
### Building
The sketch is too large to fit on the default HelTec esp32 partition, so a custom partition must be used.  To create the custom partition, first navigate to wher4e the esp32 partition configurations are stored (On my installation, this was located in ~/.arduino15/packages/esp32/hardware/esp32/1.0.0/tools/partitions).  Then make a backup of the default.csv file.  After making the backup, update the default.csv file to the following:

#Name,   Type, SubType, Offset,  Size, Flags

nvs,      data, nvs,     0x9000,  0x5000,

otadata,  data, ota,     0xe000,  0x2000,

app0,     app,  ota_0,   0x10000, 0x200000,

eeprom,   data, 0x99,    0x210000,  0x2000,

spiffs,   data, spiffs,  0x212000,0x1EE000,

This partition configuration provides more space to the app0 partition by reducing the size of the spiffs partition.


More information on how to create the custom partition can be found [here](https://docs.google.com/document/d/1XtY1ogbXwr4GIBYinUUyv2_6-Hs1pM9L4s1Zih_2k7M/edit#heading=h.4d5ahrx8sf4d "Report Document Heading Uploading Sketch")
