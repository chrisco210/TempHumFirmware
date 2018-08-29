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
The firmware does not fit on the default HelTec partition, so a custom partition must be used.

This partition file can be found in th efolder with the firmware file

More information on how to create the custom partition can be found [here](https://docs.google.com/document/d/1XtY1ogbXwr4GIBYinUUyv2_6-Hs1pM9L4s1Zih_2k7M/edit#heading=h.4d5ahrx8sf4d "Report Document Heading Uploading Sketch")
