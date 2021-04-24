# Replacing the bootloader:

The bootloader is writeprotected, this is part of the device security and therefore there is no easy way to replace the loader. In fact it seems impossible to do that for v2.31 or v2.40.

The easy way would be to use a external SPI programer but thats impossible because WCH does not provide any infos about the protocol. 

## From the datasheet: 

1. Config Register: 
There is a 16 bit config register storing some bootup options. This isn't very well documented, but some of that options are accessable by the WCH tool. These options are stored within the bootloader space which means at some point during bootup the loader space isn't writeprotected at all. (Table 6.2 flash-ROM configuration information description)
Bit15 and Bit14 can't be accessed by the WCH Tool but Bit13 and Bit12 are available. So there is a way to access these bits in loader mode.

2. bBOOT_LOAD (in GLOBAL_CFG):
This bit which is readonly is set at powerup and gets reset by a SWReset. Remember these chips have several reset sources (POR,SW,Pin,WD) If you think about that you may realize that this bit might be the writeprotect bit for the bootloader.

So basically its just to find a way to execute some usercode while bBOOT_LOAD remains set which means prevent the bootloader doing a SW Reset.

The v1.1 bootloader has support for IAP. IAP is able to start a previous flashed program from loader mode at any address. That user prog will be executed with bBOOT_LOAD = 1 and with fSys=12MHz instead the normal 6MHz. Just write a software which can flash a loader image to 0x3800.. 0x3FEF. When done and verify succeeds just clear the IAP Marker to release IAP.

Warning: If you do something wrong the chip gets bricked. Its a good idea to have second loader in flash at 0x3000 which can be called in main as a emergency. Maybe additional checks and some crc is also usefull. Allways use 5V Vcc when flashing.

## IAP cmd:

Two commands will be used: 0xBA to prepare IAP and 0xA5 to run it:
- 0xBA  0x04  0x57  0xA8  0x00  0x00  
- 0xA5  0x02  0x01  0x00 
 
0xBA stores the IAP marker and the startaddress (0x0000 in this case).
0xA5 starts the the prog.

Since IAP is only available at v1.1 the above wont work on other loaders. 

~~Maybe Pin Reset can be used to exit the bootloader without clearing the bBOOT_LOAD bit. Then changing the loader would be much more easy and it would work with any loader. I havent checked that out yet.~~ 

