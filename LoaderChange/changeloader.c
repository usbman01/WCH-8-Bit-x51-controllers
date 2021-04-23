/* a simple demo for changing the bootloader

   1. start the bootloader by HW contition 
   2. download this changeloader.hex to the device 
   3. enable IAP with v1_tools.exe
   
   It will work on V1.1 only. The program first checks boootloader bit and 
   flashes the image to 0x3800. This image is a modified v1.1 loader it reports
   1.11 in the BCD_VER field of the usb device descriptor to be easily identified  
   
   Status LEDS:
   LED1 blinking   : bBOOT_LOAD not set -> flashing impossible or wrong chip id
   LED3 blinking   : Loader == Image    -> nothing todo
   LED1 + LED3     : same blinking      -> verify ok
                   : alternate blinking -> verify fails
   LED1 + LDE3     : permanent on       -> Loder started by SW1 or HW 

   Changelog: - cleared the IAP Entry directly after bBOOT_LOAD is found
              - check to avoid unneccesary flashing moved to WriteCodeFlash()
              - check the chip id to be 0x51 to 0x54 CH551 .. CH554
*/

#include <absacc.h>              // CBYTE[]
#include <intrins.h>             // _nop()
#include "ch552.h"               // sfrs


typedef unsigned short uint16_t; // Keil does not have these
typedef unsigned char  uint8_t;

  // from Aarons board
  #define LED1   TXD      
  #define LED3   RXD 
  #define SW1    INT0

  #define CALL(addr)   (((void(*)(void))(uint8_t code *)addr)()) //call any address

  #define DELAY 0xFFFF 

// a simple delay loop used for LED blinking   
void delay(uint16_t Time)
{
  while(Time)
  {
     _nop_();
     _nop_();
     _nop_();
     _nop_();  
     Time--;    
  }    
}  

// store one word into the codeflash
// new:  check the Data and if they are the same don't flash
//       to avoid unneccessary programming cycles
uint8_t WriteCodeFlash(uint16_t Address, uint16_t Data)
{
	 uint16_t old;

	 old  =((CBYTE[Address+1]) << 8) | CBYTE[Address];
   if (old==Data) return 0;
   ROM_ADDR = Address;
   ROM_DATA = Data;
   if (ROM_CTRL & bROM_ADDR_OK)
   {
      E_DIS = 1;
      ROM_CTRL = ROM_CMD_WRITE;
      E_DIS = 0;
      return ROM_CTRL ^ bROM_ADDR_OK;
   }           
   return bROM_ADDR_OK;
}

uint16_t crc;

// calculate a crc16
void crc16(uint8_t b)
{
   register uint8_t i;
   crc^=b;
   for(i=8;i;i--)
   {
     crc = (crc>>1)^((crc&1) ? 0xA001:0);
   }
}  

/*
   a modified v1.1 bootloader 
   start  : 0x3800
   end    : 0x3FEF
*/

code unsigned char Image[0x07F0] = {
   0x02, 0x3D, 0x24, 0x6F, 0xE4, 0xF5, 0x18, 0xF5, 0x1A, 0xF1, 0x15, 0x78, 0x21, 0xE6, 0xB1, 0xB7, 
   0x38, 0x5C, 0xA2, 0x3A, 0x98, 0xA5, 0x39, 0x66, 0xA6, 0x38, 0x77, 0xA7, 0x38, 0xC4, 0xA8, 0x39, 
   0x24, 0xA9, 0x38, 0x3E, 0xAC, 0x3A, 0x7B, 0xB5, 0x3A, 0x40, 0xB6, 0x3A, 0x17, 0xB7, 0x39, 0x74, 
   0xB8, 0x39, 0xE3, 0xB9, 0x39, 0x8A, 0xBA, 0x3A, 0x12, 0xBB, 0x00, 0x00, 0x3A, 0xA3, 0x78, 0x22, 
   0xE6, 0x64, 0x04, 0x70, 0x15, 0x08, 0xE6, 0x64, 0x55, 0x70, 0x0F, 0x08, 0xE6, 0x64, 0xAA, 0x70, 
   0x09, 0x08, 0xE6, 0xC0, 0xE0, 0x08, 0xE6, 0xC0, 0xE0, 0x22, 0x21, 0x3A, 0x85, 0xA1, 0x18, 0xE4, 
   0xF9, 0xE9, 0x90, 0x3D, 0xE3, 0x93, 0xFF, 0x74, 0x23, 0x29, 0xF8, 0xE6, 0x6F, 0x60, 0x02, 0x21, 
   0x3A, 0x09, 0xB9, 0x13, 0xEC, 0x41, 0xA6, 0x78, 0x24, 0xE6, 0xFE, 0x20, 0x05, 0x06, 0x94, 0x38, 
   0x40, 0x02, 0x21, 0x3A, 0x18, 0xE6, 0xFD, 0xED, 0x8E, 0x15, 0xF5, 0x16, 0xE4, 0xF5, 0x19, 0xE4, 
   0xF9, 0x74, 0x11, 0x29, 0xF8, 0xE6, 0xFF, 0x74, 0x25, 0x25, 0x19, 0xF8, 0xE6, 0x6F, 0xFF, 0x85, 
   0x16, 0x82, 0x85, 0x15, 0x83, 0xE4, 0x93, 0x6F, 0x60, 0x02, 0x21, 0x3A, 0x05, 0x19, 0x05, 0x16, 
   0xE5, 0x16, 0x70, 0x02, 0x05, 0x15, 0x09, 0xB9, 0x04, 0xD7, 0xE5, 0x19, 0xC3, 0x78, 0x22, 0x96, 
   0x40, 0xCD, 0x41, 0xA6, 0x30, 0x01, 0x02, 0x61, 0x07, 0x78, 0x24, 0xE6, 0xFE, 0x18, 0xE6, 0xFD, 
   0xEE, 0xF5, 0x1B, 0xED, 0xF5, 0x1C, 0xE4, 0xF5, 0x19, 0xE4, 0xF9, 0xE9, 0x25, 0xE0, 0x24, 0x12, 
   0xF8, 0xE6, 0xFF, 0x74, 0x26, 0x25, 0x19, 0xF8, 0xE6, 0x6F, 0xFE, 0xE9, 0x25, 0xE0, 0x24, 0x11, 
   0xF8, 0xE6, 0xFD, 0x74, 0x25, 0x25, 0x19, 0xF8, 0xE6, 0x6D, 0xFD, 0xEE, 0xFC, 0x7E, 0x00, 0xE5, 
   0x1C, 0x25, 0x19, 0xFF, 0xEE, 0x35, 0x1B, 0xFE, 0xD1, 0x6E, 0x8F, 0x18, 0xE5, 0x18, 0x60, 0x02, 
   0x41, 0xA6, 0x05, 0x19, 0x05, 0x19, 0x09, 0xB9, 0x02, 0xC1, 0xE5, 0x19, 0xC3, 0x78, 0x22, 0x96, 
   0x40, 0xB7, 0x41, 0xA6, 0x78, 0x24, 0xE6, 0xFE, 0x18, 0xE6, 0xFD, 0xEE, 0xF5, 0x1B, 0xED, 0xF5, 
   0x1C, 0x20, 0x01, 0x02, 0x41, 0xA6, 0x45, 0x1B, 0x60, 0x05, 0x75, 0x18, 0xFF, 0x41, 0xA6, 0x74, 
   0xFF, 0xFD, 0xFC, 0xAF, 0x1C, 0xAE, 0x1B, 0xD1, 0x6E, 0x8F, 0x18, 0xE5, 0x18, 0x60, 0x02, 0x41, 
   0xA6, 0x74, 0x02, 0x25, 0x1C, 0xF5, 0x1C, 0xE4, 0x35, 0x1B, 0xF5, 0x1B, 0x54, 0x03, 0x45, 0x1C, 
   0x70, 0xDD, 0xC2, 0x01, 0x41, 0xA6, 0x85, 0x23, 0x11, 0x85, 0x24, 0x12, 0x85, 0x25, 0x13, 0x85, 
   0x26, 0x14, 0x41, 0xA6, 0x90, 0x3F, 0xF8, 0xE4, 0x93, 0x78, 0x23, 0x66, 0x70, 0x08, 0xA3, 0x93, 
   0x08, 0x66, 0x70, 0x02, 0x41, 0xA6, 0x78, 0x24, 0x80, 0x48, 0x90, 0x3F, 0xF6, 0xE4, 0x93, 0x78, 
   0x25, 0x66, 0x70, 0x06, 0xA3, 0x93, 0x08, 0x66, 0x60, 0x1C, 0x78, 0x24, 0x7F, 0xF4, 0xD1, 0x65, 
   0x8F, 0x18, 0xE5, 0x18, 0x60, 0x02, 0x41, 0xA6, 0x78, 0x26, 0x7F, 0xF6, 0xD1, 0x65, 0x8F, 0x18, 
   0xE5, 0x18, 0x60, 0x02, 0x41, 0xA6, 0x78, 0x22, 0xE6, 0x64, 0x08, 0x60, 0x02, 0x41, 0xA6, 0x90, 
   0x3F, 0xF8, 0x93, 0x78, 0x29, 0x66, 0x70, 0x08, 0xA3, 0x93, 0x08, 0x66, 0x70, 0x02, 0x41, 0xA6, 
   0x78, 0x2A, 0xE6, 0xFE, 0x18, 0xE6, 0xFD, 0xEE, 0xFC, 0x7F, 0xF8, 0x7E, 0x3F, 0xD1, 0x6E, 0x8F, 
   0x18, 0x41, 0xA6, 0x75, 0x15, 0x3F, 0x75, 0x16, 0xF8, 0xE5, 0x1A, 0xC3, 0x94, 0x08, 0x40, 0x02, 
   0x41, 0xA6, 0xE5, 0x1A, 0x24, 0xF8, 0xFF, 0xE4, 0x34, 0x3F, 0x8F, 0x82, 0xF5, 0x83, 0xE4, 0x93, 
   0xFF, 0x74, 0x4C, 0x25, 0x1A, 0xF5, 0x82, 0xE4, 0x34, 0x00, 0xF5, 0x83, 0xEF, 0xF0, 0x05, 0x1A, 
   0x80, 0xD7, 0x75, 0x18, 0x11, 0x41, 0xA6, 0x75, 0x85, 0xC0, 0xE5, 0x1A, 0xC3, 0x94, 0x80, 0x50, 
   0x1B, 0xE5, 0x1A, 0x25, 0xE0, 0xF5, 0x84, 0x75, 0x86, 0x8E, 0x74, 0x4C, 0x25, 0x1A, 0xF5, 0x82, 
   0xE4, 0x34, 0x00, 0xF5, 0x83, 0xE5, 0x8E, 0xF0, 0x05, 0x1A, 0x80, 0xDE, 0xD2, 0x00, 0x80, 0x66, 
   0x78, 0x23, 0xE6, 0x75, 0x1B, 0x00, 0xF5, 0x1C, 0x75, 0x85, 0xC0, 0xE4, 0xF5, 0x19, 0xE4, 0xF9, 
   0xE5, 0x19, 0x25, 0x1C, 0xFF, 0x74, 0x11, 0x29, 0xF8, 0xE6, 0xFE, 0x74, 0x25, 0x25, 0x19, 0xF8, 
   0xE6, 0x6E, 0xFD, 0xD1, 0xEE, 0x8F, 0x18, 0xE5, 0x18, 0x70, 0x3B, 0x05, 0x19, 0x09, 0xB9, 0x04, 
   0xDF, 0xE5, 0x19, 0xC3, 0x78, 0x22, 0x96, 0x40, 0xD5, 0x80, 0x2B, 0x78, 0x23, 0xE6, 0xF9, 0x75, 
   0x85, 0xC0, 0xE9, 0xC3, 0x94, 0x80, 0x50, 0x1E, 0xCF, 0xE9, 0xCF, 0x7D, 0xFF, 0xD1, 0xEE, 0x8F, 
   0x18, 0xE5, 0x18, 0x70, 0x11, 0x09, 0x80, 0xEA, 0x78, 0x24, 0xE6, 0x18, 0x46, 0x60, 0x07, 0xD2, 
   0x02, 0x80, 0x03, 0x75, 0x18, 0xFE, 0xF1, 0x15, 0x20, 0x03, 0x37, 0xE5, 0x1A, 0x60, 0x11, 0xD3, 
   0x94, 0x40, 0x40, 0x04, 0x7F, 0x40, 0x80, 0x02, 0xAF, 0x1A, 0x8F, 0xD5, 0x53, 0xD4, 0xFC, 0x22, 
   0x90, 0x00, 0x4C, 0xE5, 0x18, 0xF0, 0xE4, 0xA3, 0xF0, 0x75, 0xD5, 0x02, 0x53, 0xD4, 0xFC, 0x30, 
   0x02, 0x35, 0x75, 0x18, 0x32, 0xAF, 0x18, 0x15, 0x18, 0xEF, 0x60, 0x2B, 0x7F, 0x64, 0xF1, 0x06, 
   0x80, 0xF3, 0xE5, 0x1A, 0x60, 0x19, 0xE4, 0xF9, 0xE9, 0xC3, 0x95, 0x1A, 0x50, 0x19, 0x74, 0x4C, 
   0x29, 0xF5, 0x82, 0xE4, 0x34, 0x00, 0xF5, 0x83, 0xE0, 0xFF, 0xF1, 0x22, 0x09, 0x80, 0xE9, 0xAF, 
   0x18, 0xF1, 0x22, 0xE4, 0xFF, 0xF1, 0x22, 0x22, 0x20, 0xD9, 0x02, 0x81, 0x31, 0xE5, 0xD9, 0x54, 
   0x3F, 0x70, 0x02, 0x81, 0x2B, 0x24, 0xE0, 0x70, 0x02, 0x81, 0x0C, 0x24, 0xFE, 0x60, 0x2C, 0x24, 
   0xF2, 0x60, 0x53, 0x24, 0x2E, 0x60, 0x02, 0x81, 0x2E, 0x20, 0xDE, 0x02, 0x81, 0x2E, 0x85, 0xDB, 
   0x17, 0x75, 0x1B, 0x01, 0x75, 0x1C, 0x00, 0x75, 0x1D, 0x0C, 0x85, 0x17, 0x1E, 0x7B, 0x00, 0x7A, 
   0x00, 0x79, 0x21, 0xD1, 0x9E, 0xC2, 0x03, 0x11, 0x04, 0x81, 0x2E, 0x30, 0x00, 0x1E, 0xC2, 0x00, 
   0x75, 0x1B, 0x01, 0x75, 0x1C, 0x00, 0x75, 0x1D, 0x8C, 0x75, 0x1E, 0x40, 0x7B, 0x01, 0x7A, 0x00, 
   0x79, 0x4C, 0xD1, 0x9E, 0x75, 0xD5, 0x40, 0x53, 0xD4, 0xFC, 0x81, 0x2E, 0xE5, 0xD4, 0x54, 0xFC, 
   0x44, 0x02, 0xF5, 0xD4, 0x81, 0x2E, 0x53, 0xDC, 0xF2, 0xE5, 0xDB, 0x64, 0x08, 0x70, 0x6F, 0x90, 
   0x00, 0x06, 0xE0, 0xF5, 0x09, 0xC2, 0x04, 0x90, 0x00, 0x00, 0xE0, 0x54, 0x60, 0x60, 0x04, 0xD2, 
   0x04, 0x80, 0x5D, 0x90, 0x00, 0x01, 0xE0, 0xF5, 0x0D, 0xE5, 0x0D, 0x24, 0xFB, 0x60, 0x36, 0x24, 
   0xFD, 0x60, 0x34, 0x14, 0x60, 0x3C, 0x24, 0x03, 0x70, 0x40, 0x90, 0x00, 0x03, 0xE0, 0x24, 0xFE, 
   0x60, 0x11, 0x04, 0x70, 0x1C, 0x75, 0x0E, 0xFF, 0x75, 0x0F, 0x3D, 0x75, 0x10, 0xF7, 0x75, 0x17, 
   0x12, 0x80, 0x2D, 0x75, 0x0E, 0xFF, 0x75, 0x0F, 0x3E, 0x75, 0x10, 0x09, 0x75, 0x17, 0x20, 0x80, 
   0x1F, 0xD2, 0x04, 0x80, 0x1B, 0x80, 0x0B, 0x90, 0x00, 0x00, 0xE5, 0x08, 0xF0, 0x75, 0x17, 0x01, 
   0x80, 0x0E, 0x90, 0x00, 0x02, 0xE0, 0xF5, 0x08, 0x80, 0x06, 0xD2, 0x04, 0x80, 0x02, 0xD2, 0x04, 
   0x30, 0x04, 0x08, 0x75, 0x0D, 0xFF, 0x75, 0xDC, 0xCF, 0x80, 0x33, 0xE5, 0x09, 0xD3, 0x95, 0x17, 
   0x40, 0x03, 0x85, 0x17, 0x09, 0xD1, 0x29, 0x75, 0xDC, 0xC0, 0x80, 0x22, 0xE5, 0x0D, 0x24, 0xFB, 
   0x60, 0x0A, 0x14, 0x70, 0x11, 0xD1, 0x29, 0x63, 0xDC, 0x40, 0x80, 0x12, 0xE5, 0xE3, 0x54, 0x80, 
   0x45, 0x08, 0xF5, 0xE3, 0x80, 0x05, 0xE4, 0xF5, 0xDD, 0x80, 0x00, 0x75, 0xDC, 0x02, 0xC2, 0xD9, 
   0x22, 0x30, 0xD8, 0x10, 0x75, 0xDC, 0x02, 0x75, 0xD4, 0x12, 0xE4, 0xF5, 0xE3, 0xC2, 0xDA, 0xC2, 
   0xD9, 0xC2, 0xD8, 0x22, 0x30, 0xDA, 0x03, 0xC2, 0xDA, 0x22, 0x75, 0xD8, 0xFF, 0x22, 0xF1, 0x34, 
   0x75, 0xA1, 0x55, 0x75, 0xA1, 0xAA, 0xE5, 0xB9, 0x54, 0xF8, 0x44, 0x04, 0xF5, 0xB9, 0xE4, 0xF5, 
   0xA1, 0xC2, 0x03, 0xC2, 0x02, 0xD2, 0x01, 0xC2, 0x00, 0xC2, 0xAF, 0xC2, 0x8C, 0xC2, 0x8D, 0xC2, 
   0xC7, 0xD2, 0xC5, 0xD2, 0xC4, 0x75, 0xC2, 0xF3, 0xC2, 0xC1, 0xC2, 0xC0, 0xE5, 0xB1, 0x30, 0xE5, 
   0x07, 0xD1, 0xD0, 0xEF, 0x64, 0x01, 0x70, 0x2B, 0xE4, 0xF5, 0xE2, 0x75, 0xEB, 0x0C, 0x75, 0xED, 
   0x00, 0x75, 0xEC, 0x00, 0x75, 0xE5, 0x00, 0x75, 0xE4, 0x0C, 0x75, 0xDC, 0x02, 0x75, 0xD4, 0x12, 
   0xF5, 0xE3, 0x75, 0xD1, 0x80, 0x75, 0xE2, 0x29, 0x43, 0xD1, 0x01, 0x75, 0xD8, 0xFF, 0x75, 0xE1, 
   0x07, 0x80, 0x0D, 0x43, 0x89, 0x01, 0x75, 0x8A, 0xC0, 0x75, 0x8C, 0x64, 0xC2, 0x8D, 0xD2, 0x8C, 
   0x30, 0xC0, 0x02, 0xB1, 0x2F, 0xE5, 0xD8, 0x54, 0x07, 0x60, 0x02, 0x71, 0x08, 0x30, 0x8D, 0x09, 
   0xC2, 0x8D, 0xC2, 0x8C, 0x20, 0x03, 0x02, 0xD2, 0x02, 0x30, 0x02, 0xE4, 0x90, 0x3F, 0xF4, 0xE4, 
   0x93, 0x64, 0x57, 0x70, 0x34, 0xA3, 0x93, 0x64, 0xA8, 0x70, 0x2E, 0x75, 0xA1, 0x55, 0x75, 0xA1, 
   0xAA, 0x75, 0xB9, 0x83, 0xF5, 0xA1, 0x75, 0xE2, 0x06, 0xC2, 0xC5, 0xC2, 0xC4, 0xC2, 0x8C, 0x90, 
   0x3F, 0xF7, 0x93, 0xFE, 0x90, 0x3F, 0xF6, 0xE4, 0x93, 0xFD, 0xED, 0xCA, 0xEE, 0xCA, 0xF9, 0x7B, 
   0x00, 0x8B, 0x0A, 0x8A, 0x0B, 0xF5, 0x0C, 0xB1, 0xDD, 0x75, 0xA1, 0x55, 0x75, 0xA1, 0xAA, 0x43, 
   0xB1, 0x10, 0x80, 0xFE, 0x78, 0x7F, 0xE4, 0xF6, 0xD8, 0xFD, 0x75, 0x81, 0x60, 0x81, 0x4E, 0x30, 
   0xC0, 0x59, 0xF1, 0x2C, 0xEF, 0x64, 0x61, 0x70, 0x52, 0xF1, 0x2C, 0xEF, 0x64, 0x9E, 0x70, 0x4B, 
   0xF1, 0x2C, 0x78, 0x21, 0xEF, 0xF6, 0xFE, 0xF1, 0x2C, 0x08, 0xEF, 0xF6, 0xFD, 0xE6, 0x2E, 0xFE, 
   0x18, 0xE6, 0x64, 0xA8, 0x60, 0x09, 0xE6, 0x64, 0xA7, 0x60, 0x04, 0xE6, 0xB4, 0xB6, 0x02, 0x0D, 
   0x0D, 0xE4, 0xFC, 0xEC, 0x6D, 0x60, 0x12, 0xF1, 0x2C, 0x74, 0x23, 0x2C, 0xF8, 0xEF, 0xF6, 0x74, 
   0x23, 0x2C, 0xF8, 0xE6, 0x2E, 0xFE, 0x0C, 0x80, 0xEA, 0xF1, 0x2C, 0xEE, 0x6F, 0x60, 0x08, 0x7F, 
   0x55, 0xF1, 0x22, 0x7F, 0xAA, 0xE1, 0x22, 0xD2, 0x03, 0x11, 0x04, 0x22, 0xBB, 0x01, 0x06, 0x89, 
   0x82, 0x8A, 0x83, 0xE0, 0x22, 0x50, 0x02, 0xE7, 0x22, 0xBB, 0xFE, 0x02, 0xE3, 0x22, 0x89, 0x82, 
   0x8A, 0x83, 0xE4, 0x93, 0x22, 0xBB, 0x01, 0x06, 0x89, 0x82, 0x8A, 0x83, 0xF0, 0x22, 0x50, 0x02, 
   0xF7, 0x22, 0xBB, 0xFE, 0x01, 0xF3, 0x22, 0xD0, 0x83, 0xD0, 0x82, 0xF8, 0xE4, 0x93, 0x70, 0x12, 
   0x74, 0x01, 0x93, 0x70, 0x0D, 0xA3, 0xA3, 0x93, 0xF8, 0x74, 0x01, 0x93, 0xF5, 0x82, 0x88, 0x83, 
   0xE4, 0x73, 0x74, 0x02, 0x93, 0x68, 0x60, 0xEF, 0xA3, 0xA3, 0xA3, 0x80, 0xDF, 0x8A, 0x83, 0x89, 
   0x82, 0xE4, 0x73, 0x55, 0x53, 0x42, 0x20, 0x44, 0x42, 0x47, 0x20, 0x43, 0x48, 0x35, 0x35, 0x39, 
   0x20, 0x26, 0x20, 0x49, 0x53, 0x50, 0x00, 0x12, 0x01, 0x10, 0x01, 0xFF, 0x80, 0x55, 0x08, 0x48, 
   0x43, 0xE0, 0x55, 0x11, 0x01, 0x00, 0x00, 0x00, 0x01, 0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 
   0x80, 0x32, 0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x80, 0x55, 0x00, 0x07, 0x05, 0x82, 0x02, 0x40, 
   0x00, 0x00, 0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00, 0xE5, 0x09, 0x60, 0x34, 0xC3, 0x94, 0x08, 
   0x40, 0x04, 0x7F, 0x08, 0x80, 0x02, 0xAF, 0x09, 0xCE, 0xEF, 0xCE, 0x85, 0x0E, 0x1B, 0x85, 0x0F, 
   0x1C, 0x85, 0x10, 0x1D, 0x8E, 0x1E, 0x7B, 0x01, 0x7A, 0x00, 0x79, 0x00, 0xD1, 0x9E, 0xC3, 0xE5, 
   0x09, 0x9E, 0xF5, 0x09, 0xEE, 0x25, 0x10, 0xF5, 0x10, 0xE4, 0x35, 0x0F, 0xF5, 0x0F, 0x8E, 0xDD, 
   0x22, 0xE4, 0xF5, 0xDD, 0x22, 0xE6, 0xFE, 0x18, 0xE6, 0xFD, 0xEE, 0xFC, 0x7E, 0x3F, 0xC3, 0xEE, 
   0x94, 0x38, 0x40, 0x11, 0xEF, 0x94, 0xF4, 0xEE, 0x94, 0x3F, 0x40, 0x1F, 0xD3, 0xEF, 0x94, 0xF8, 
   0xEE, 0x94, 0x3F, 0x50, 0x16, 0x8E, 0x85, 0x8F, 0x84, 0x8C, 0x8F, 0x8D, 0x8E, 0xE5, 0x86, 0x30, 
   0xE6, 0x09, 0x75, 0x86, 0x9A, 0xE5, 0x86, 0x64, 0x40, 0xFF, 0x22, 0x7F, 0x40, 0x22, 0x8B, 0x18, 
   0x8A, 0x19, 0x89, 0x1A, 0xAF, 0x1E, 0x15, 0x1E, 0xEF, 0x60, 0x24, 0xAB, 0x1B, 0x05, 0x1D, 0xE5, 
   0x1D, 0xAA, 0x1C, 0x70, 0x02, 0x05, 0x1C, 0x14, 0xF9, 0xB1, 0x8C, 0xFF, 0xAB, 0x18, 0x05, 0x1A, 
   0xE5, 0x1A, 0xAA, 0x19, 0x70, 0x02, 0x05, 0x19, 0x14, 0xF9, 0xEF, 0xB1, 0xA5, 0x80, 0xD5, 0x22, 
   0xE4, 0xFE, 0x20, 0xB6, 0x0C, 0x7F, 0x01, 0xF1, 0x06, 0x0E, 0xEE, 0x30, 0xE6, 0xF4, 0x7F, 0x00, 
   0x22, 0x7F, 0x01, 0xF1, 0x06, 0x30, 0xB6, 0x03, 0x7F, 0x01, 0x22, 0x7F, 0x00, 0x22, 0xEF, 0x25, 
   0xE0, 0xF5, 0x84, 0x8D, 0x8E, 0xE5, 0x86, 0x30, 0xE6, 0x09, 0x75, 0x86, 0x9A, 0xE5, 0x86, 0x64, 
   0x40, 0xFF, 0x22, 0x7F, 0x40, 0x22, 0xEF, 0x13, 0x13, 0x54, 0x3F, 0xFF, 0xEF, 0x60, 0x05, 0x05, 
   0xA1, 0x1F, 0x80, 0xF8, 0x22, 0x75, 0xA1, 0x55, 0x75, 0xA1, 0xAA, 0x63, 0xB1, 0x0C, 0xE4, 0xF5, 
   0xA1, 0x22, 0xC2, 0xC1, 0x8F, 0xC1, 0x30, 0xC1, 0xFD, 0xC2, 0xC1, 0x22, 0x30, 0xC0, 0xFD, 0xC2, 
   0xC0, 0xAF, 0xC1, 0x22, 0x75, 0x11, 0x54, 0x75, 0x12, 0x77, 0x75, 0x13, 0x63, 0x75, 0x14, 0x68, 
   0x22, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
   0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

// build a crc16 
uint16_t BuildCrc(uint8_t *p)
{
   uint16_t i;
   crc = 0; 
   for (i=0; i < sizeof(Image);i++) 
   {
      crc16(*p);
      p++;
   }
   return crc;
}


void main (void)
{
   uint16_t  image_crc;
   uint16_t  loader_crc;
   uint16_t  addr; 
   uint16_t  value;
   uint16_t  i;   
   uint8_t   chip_id = CHIP_ID;
   if (SW1 == 0) CALL (0x3800); 

   if ((chip_id < 0x51) || (chip_id > 0x54) || (!(GLOBAL_CFG & bBOOT_LOAD)))
   {
      while(1)
      {
         delay(DELAY);
         LED1 = 1; 
         delay(DELAY);
         LED1 = 0;
      }
   }
   
   image_crc =BuildCrc(Image);
   loader_crc=BuildCrc((uint8_t code *) BOOT_LOAD_ADDR);
   // unlock the flash and disable IAP
   SAFE_MOD    = 0x55;
	 SAFE_MOD    = 0xAA;
	 GLOBAL_CFG |= bCODE_WE;
   SAFE_MOD    = 0;
   WriteCodeFlash(ROM_CFG_ADDR-4,0xFFFF);
   WriteCodeFlash(ROM_CFG_ADDR-2,0xFFFF);
   
   if(image_crc!=loader_crc)
   {  
      
      // flash the bootloader 
      addr = BOOT_LOAD_ADDR;
      for (i=0; i<sizeof(Image); i+=2)
      {          
         value=(Image[i+1] << 8) | Image[i];  
         WriteCodeFlash(addr,value);
         addr+=2;
      }
      // lock the flash
      SAFE_MOD    =  0x55;
	    SAFE_MOD    =  0xAA;
	    GLOBAL_CFG &= ~bCODE_WE;
      
      // do a verify
      image_crc =BuildCrc(Image);
      loader_crc=BuildCrc((uint8_t code *) BOOT_LOAD_ADDR);
      if(image_crc==loader_crc)
      {
         // verify ok 
         while(1) 
         {
            delay(DELAY);
            LED1=1;LED3 = 1; 
            delay(DELAY);
            LED1=0;LED3 = 0;
         }  
      }
      else 
      {
         // verfiy failed
         while(1)
         {
            delay(DELAY);
            LED1=0;LED3 = 1; 
            delay(DELAY);
            LED1=1;LED3 = 0;
         }  
      }
   }
   // lock the flash
   SAFE_MOD    =  0x55;
	 SAFE_MOD    =  0xAA;
	 GLOBAL_CFG &= ~bCODE_WE;
   
   // nothing to do
   while(1)
   {
      delay(DELAY);
      LED3 = 1; 
      delay(DELAY);
      LED3 = 0;
   }
}