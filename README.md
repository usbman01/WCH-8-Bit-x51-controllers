# Preface
This is a add on to the datasheet of the Ch551-CH554 unfolding some facts which are missing in the datasheets. Basically the CH55x are fast MCS51 controllers with buildin extra features like USB or SPI. So far nothing special and many other companies have that too but, but these things are really cheap (less than 0.2 USD @LSC). 
The only disadvantage is that all documents are in Chinese only available. It seems WCH targets only the chinese markets which makes it difficult to find realable informations. 
Here its all about the CH552 but most applies for CH551 and CH554 too. In fact al of these chips uses the same die. {link} 
So its save in most cases to use the datasheet for the CH554 for the other MCUs too.

## Clocks:
The max clock speed is offically 24 MHz derived from some PLL. There is no need for a crystal. The 32MHz setting is offically forbitten but works too, exept for USB because the 48MHz USB clock cannot be reached.
The execution speed is 8..10 times higher than a normal 12 Clocker, which means efectively the CH552 is a 1.3 Clocker. The chips boot up at fsys=6MHz.
Timers can work at fsys/12 fsys/4 and fsys/1, all usual bautrates can be met up 115k2 @ 24MHz even when derived from T1. 

## Ports:
Ports differ from the original ones, they also can be configured as PUSH/PULL and OC besides the normal x51 mode.

## USB:
The USB Core supports max 8 endpoints (4 In / 4 Out) besides the control endpoint. The core supports FS (fullspeed 12Mbit) as well as LS (low speed 1.5Mbit). The CH554 supports USB hostmode too.

## Flash:
The flash (up to 16k) is a bit odd. It does not need to be errased prior writing and always stores 16 bits starting at even addresses. The last 2kb starting at 0x3800 usually contain a bootloader and might be in most cases WR protected. 

## Bootloader:
The chips contain a bootloader for flashing the firmware. This bootloader can be started by two different methods:

 1. by power on reset (when DP+ is pulled high by 10k during power on)
 2. by LJMP 0x3800

Both ways behave almost identical exept some commands will only work if the loader is called by harware contition. In fact it looks like at power on the CPU jumps to 0x3800 rather than to 0x0000.

In the wild at least 3 different bootloaders have been seen.

v1.1: 
 - can be replaced by any usr loader
 - iap cmd support 
v2.31: 
 - use a different cmd set
 - a alternate hw contition by pulling p1.5 low (if enabled)
 - not replaceable by usr
 - no IAP cmd support (exept on CH559)
v2.40: 
  - same cmd set as v2.31
  - much more secure than the others

common to all three loaders is support for program download by USB and UART. Exept for v2.40 all these loaders are not secure. The programcode can easily be read back by a simple application. WCH provides a app called WCHIspTool for Win to flash a Intel Hex file by USB or serial.

## WCHIspTool:
WCH offers a tool for flashing a firmware to the chips. This tool is also used setup some config options. Saving config options on the chip works only when the bootloader is activated with the hw contition.

The tool seems to have problems when loading Intel Hex files with missing cr/lf so be carefull with hexfiles from Linux. Gaps in the hexfile will be filled with 0x00 instead the usual 0xFF. This may become a problem on other controllers where pages need to be erased. Intel Hexfiles not starting at 0x0000 can't be processed because the way secutity is implemented. These problems can be solved by using some hex processing tools. The IAP option seems not to work correctly. 

# Device Header:
WCH provides device header files for Keil which can be used, but these
files are Keil only. Beside there are lots of extra declarations which should not be part of the device header there are some subtile problems when used on lsb compilers like IAR or SDCC. For example all int/uint  references to xdata should be reviewed. WCH implicitly asumes msb order which is ok for Keil. 

The new header files provided here can be used with Keil,IAR and SDCC. Other compilers can be used by extending compiler.h.