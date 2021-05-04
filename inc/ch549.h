/*
  removed all the typedefs, usb section and C++  
  all names and consts are still the same
*/

#ifndef CH549_H_
#include "compiler.h"

/*  System Registers  */
SFR_(PSW,0xD0);               // program status word
  SBIT_(CY,0xD0,7);           // carry flag
  SBIT_(AC,0xD0,6);           // auxiliary carry flag
  SBIT_(F0,0xD0,5);           // bit addressable general purpose flag 0
  SBIT_(RS1,0xD0,4);          // register R0-R7 bank selection high bit
  SBIT_(RS0,0xD0,3);          // register R0-R7 bank selection low bit
  SBIT_(OV,0xD0,2);           // overflow flag
  SBIT_(F1,0xD0,1);           // bit addressable general purpose flag 1
  SBIT_(P,0xD0,0);            // ReadOnly: parity flag

  #define MASK_PSW_RS 0x18    // bit mask of register R0-R7 bank selection
  // RS1 & RS0: register R0-R7 bank selection
  //    00 - bank 0, R0-R7 @ address 0x00-0x07
  //    01 - bank 1, R0-R7 @ address 0x08-0x0F
  //    10 - bank 2, R0-R7 @ address 0x10-0x17
  //    11 - bank 3, R0-R7 @ address 0x18-0x1F
SFR_(ACC,0xE0);               // accumulator
SFR_(A_INV,0xFD);             // ReadOnly: bit inversion of ACC (bit0<->bit7,bit1<->bit6,bit2<->bit5,bit3<->bit4, [7..0]->[0..7])
  SBIT_(ACC7,0xE0,7);
  SBIT_(ACC6,0xE0,6);  
  SBIT_(ACC5,0xE0,5);
  SBIT_(ACC4,0xE0,4);
  SBIT_(ACC3,0xE0,3);
  SBIT_(ACC2,0xE0,2);
  SBIT_(ACC1,0xE0,1);
  SBIT_(ACC0,0xE0,0);
SFR_(B,0xF0);                 // general purpose register B
  SBIT_(B07,0xF0,7);
  SBIT_(B06,0xF0,6);  
  SBIT_(B05,0xF0,5);
  SBIT_(B04,0xF0,4);
  SBIT_(B03,0xF0,3);
  SBIT_(B02,0xF0,2);
  SBIT_(B01,0xF0,1);
  SBIT_(B00,0xF0,0);
SFR_(SP,0x81);                // stack pointer
SFR_(DPL,0x82);               // data pointer low
SFR_(DPH,0x83);               // data pointer high
SFR_(SAFE_MOD,0xA1);          // WriteOnly: writing safe mode
SFR_(CHIP_ID,0xA1);           // ReadOnly: reading chip ID
SFR_(GLOBAL_CFG,0xB1);        // global config, Write@SafeMode
  #define bBOOT_LOAD     0x20 // ReadOnly: boot loader status for discriminating BootLoader or Application: set 1 by power on reset, clear 0 by software reset
  #define bSW_RESET      0x10 // software reset bit, auto clear by hardware
  #define bCODE_WE       0x08 // enable flash-ROM (include code & data area) being program or erasing: 0=writing protect, 1=enable program and erase
  #define bDATA_WE       0x04 // enable Data-Flash (flash-ROM data area) being program or erasing: 0=writing protect, 1=enable program and erase
  #define bWDOG_EN       0x01 // enable watch-dog reset if watch-dog timer overflow: 0=as timer only, 1=enable reset if timer overflow

/* Clock and Sleep and Power Registers */
SFR_(PCON,0x87);              // power control and reset flag
  #define SMOD           0x80 // baud rate selection for UART0 mode 1/2/3: 0=slow(Fsys/128 @mode2, TF1/32 @mode1/3, no effect for TF2),
                              //   1=fast(Fsys/32 @mode2, TF1/16 @mode1/3, no effect for TF2)
  #define bRST_FLAG1     0x20 // ReadOnly: recent reset flag high bit
  #define bRST_FLAG0     0x10 // ReadOnly: recent reset flag low bit
  #define MASK_RST_FLAG  0x30 // ReadOnly: bit mask of recent reset flag
  #define RST_FLAG_SW    0x00
  #define RST_FLAG_POR   0x10
  #define RST_FLAG_WDOG  0x20
  #define RST_FLAG_PIN   0x30
  #define GF1            0x08 // general purpose flag bit 1
  #define GF0            0x04 // general purpose flag bit 0
  #define PD             0x02 // power-down enable bit, auto clear by wake-up hardware
  // bPC_RST_FLAG1 & bPC_RST_FLAG0: recent reset flag
  //    00 - software reset, by bSW_RESET=1 @(bBOOT_LOAD=0 or bWDOG_EN=1)
  //    01 - power on reset
  //    10 - watch-dog timer overflow reset
  //    11 - external input manual reset by RST pin

SFR_(POWER_CFG,0xBA);         // power config, Write@SafeMode
  #define bPWR_DN_MODE   0x80 // power down mode: 0=deep sleep and slow waking, 1=standby and fast waking
  #define bUSB_PU_RES    0x40 // USB pullup resistance mode: 0=1.5K for 3.3V, 1=7K for 5V
  #define bLV_RST_OFF    0x20 // disable low voltage reset: 0=enable LVR, 1=disable LVR
  #define bLDO_3V3_OFF   0x10 // disable 5V->3.3V LDO: 0=enable LDO for USB I/O, 1=disable 3.3V LDO, short V33 and VDD power
  #define bLDO_CORE_VOL  0x08 // core voltage mode: 0=normal, 1=raised for performance
  #define MASK_ULLDO_VOL 0x07 // bit mask of ULLDO voltage selection

SFR_(CLOCK_CFG,0xB9);         // system clock config: lower 3 bits for system clock Fsys, Write@SafeMode
  #define bOSC_EN_INT     0x80// internal oscillator enable and original clock selection: 1=enable & select internal clock, 0=disable & select external clock
  #define bOSC_EN_XT      0x40// external oscillator enable, need quartz crystal or ceramic resonator between XI and XO pins
  #define bWDOG_IF_TO     0x20// ReadOnly: watch-dog timer overflow interrupt flag, cleared by reload watch-dog count or auto cleared when MCU enter interrupt routine
  #define MASK_SYS_CK_SEL 0x07// bit mask of system clock Fsys selection

/*
   Fxt  = 24MHz (12MHz~30MHz for non-USB application), from external oscillator @XI&XO
   Fosc = bOSC_EN_INT ? 24MHz : Fxt
   Fpll = Fosc * 4 => 96MHz (48MHz~120MHz for non-USB application)
   Fusb4x = Fpll / 2 => 48MHz (Fixed)
              MASK_SYS_CK_SEL[2] [1] [0]
   Fsys = Fpll/2   =  48MHz:  1   1   1
   Fsys = Fpll/3   =  32MHz:  1   1   0
   Fsys = Fpll/4   =  24MHz:  1   0   1
   Fsys = Fpll/6   =  16MHz:  1   0   0
   Fsys = Fpll/8   =  12MHz:  0   1   1
   Fsys = Fpll/32  =   3MHz:  0   1   0
   Fsys = Fpll/128 = 750KHz:  0   0   1
   Fsys = Fpll/512 =187.5KHz: 0   0   0
*/

SFR_(WAKE_CTRL, 0xA9);        // wake-up control, Write@SafeMode
  #define bWAK_BY_USB     0x80// enable wake-up by USB event
  #define bWAK_RXD1_LO    0x40// enable wake-up by RXD1 low level
  #define bWAK_P1_5_LO    0x20// enable wake-up by pin P1.5 low level
  #define bWAK_P1_4_LO    0x10// enable wake-up by pin P1.4 low level
  #define bWAK_P0_3_LO    0x08// enable wake-up by pin P0.3 low level
  #define bWAK_P57H_INT3L 0x04// enable wake-up by pin P5.7 high level or INT3 low level
  #define bWAK_INT0E_P33L 0x02// enable wake-up by pin INT0 edge or pin P3.3 (INT1) low level
  #define bWAK_RXD0_LO    0x01// enable wake-up by RXD0 low level
SFR_(RESET_KEEP,0xFE);        // value keeper during reset
SFR_(WDOG_COUNT,0xFF);        // watch-dog count, count by clock frequency Fsys/131072

/*  Interrupt Registers  */
SFR_(IE,0xA8);                // interrupt enable
  SBIT_(EA   ,0xA8,7);        // enable global interrupts: 0=disable, 1=enable if E_DIS=0
  SBIT_(E_DIS,0xA8,6);        // disable global interrupts, intend to inhibit interrupt during some flash-ROM operation: 0=enable if EA=1, 1=disable
  SBIT_(ET2  ,0xA8,5);        // enable timer2 interrupt
  SBIT_(ES   ,0xA8,4);        // enable UART0 interrupt
  SBIT_(ET1  ,0xA8,3);        // enable timer1 interrupt
  SBIT_(EX1  ,0xA8,2);        // enable external interrupt INT1
  SBIT_(ET0  ,0xA8,1);        // enable timer0 interrupt
  SBIT_(EX0  ,0xA8,0);        // enable external interrupt INT0
SFR_(IP,0xB8);                // interrupt priority and current priority
  SBIT_(PH_FLAG,0xB8,7);      // ReadOnly: high level priority action flag
  SBIT_(PL_FLAG,0xB8,6);      // ReadOnly: low level priority action flag
// PH_FLAG & PL_FLAG: current interrupt priority
//    00 - no interrupt now
//    01 - low level priority interrupt action now
//    10 - high level priority interrupt action now
//    11 - unknown error
  SBIT_(PT2 ,0xA8,5);         // timer2 interrupt priority level
  SBIT_(PS  ,0xA8,4);         // UART0 interrupt priority level
  SBIT_(PT1 ,0xA8,3);         // timer1 interrupt priority level
  SBIT_(PX1 ,0xA8,2);         // external interrupt INT1 priority level
  SBIT_(PT0 ,0xA8,1);         // timer0 interrupt priority level
  SBIT_(PX0 ,0xA8,0);         // external interrupt INT0 priority level
SFR_(IE_EX,0xE8);             // extend interrupt enable
  SBIT_(IE_WDOG ,0xE8,7);     // enable watch-dog timer interrupt
  SBIT_(IE_GPIO ,0xE8,6);     // enable GPIO input interrupt
  SBIT_(IE_PWMX ,0xE8,5);     // enable PWMX interrupt
  SBIT_(IE_UART3,0xE8,5);     // enable UART3 interrupt
  SBIT_(IE_UART1,0xE8,4);     // enable UART1 interrupt
  SBIT_(IE_ADC  ,0xE8,3);     // enable ADC interrupt
  SBIT_(IE_UART2,0xE8,3);     // enable UART2 interrupt
  SBIT_(IE_USB  ,0xE8,2);     // enable USB interrupt
  SBIT_(IE_INT3 ,0xE8,1);     // enable external interrupt INT3
  SBIT_(IE_SPI0 ,0xE8,0);     // enable SPI0 interrupt
SFR_(IP_EX,0xE9);             // extend interrupt priority
  #define bIP_LEVEL   0x80    // ReadOnly: current interrupt nested level: 0=no interrupt or two levels, 1=one level
  #define bIP_GPIO    0x40    // GPIO input interrupt priority level
  #define bIP_PWMX    0x20    // PWMX interrupt priority level
  #define bIP_UART3   0x20    // UART3 interrupt priority level
  #define bIP_UART1   0x10    // UART1 interrupt priority level
  #define bIP_ADC     0x08    // ADC interrupt priority level
  #define bIP_UART2   0x08    // UART2 interrupt priority level
  #define bIP_USB     0x04    // USB interrupt priority level
  #define bIP_INT3    0x02    // external interrupt INT3 priority level
  #define bIP_SPI0    0x01    // SPI0 interrupt priority level
SFR_(INTX,0xB3);              // external interrupt INTx control and interrupt flag
  #define bIX3        0x20    // INT3 interrupt polarity: 0=low level or falling edge action, 1=high level or rising edge action
  #define bIE3        0x08    // INT3 interrupt flag, auto cleared when MCU enter interrupt routine
  #define bIT3        0x04    // INT3 interrupt type: 0=low/high level action, 1=falling/rising edge action
SFR_(GPIO_IE,0xB2);           // GPIO interrupt enable
  #define bIE_IO_EDGE 0x80    // enable GPIO edge interrupt: 0=low/high level, 1=falling/rising edge
  #define bIE_RXD1_LO 0x40    // enable interrupt by RXD1 low level / falling edge
  #define bIE_P1_5_LO 0x20    // enable interrupt by pin P1.5 low level / falling edge
  #define bIE_P1_4_LO 0x10    // enable interrupt by pin P1.4 low level / falling edge
  #define bIE_P0_3_LO 0x08    // enable interrupt by pin P0.3 low level / falling edge
  #define bIE_P5_7_HI 0x04    // enable interrupt by pin P5.7 (RST) high level / rising edge
  #define bIE_P4_6_LO 0x02    // enable interrupt by pin P4.6 low level / falling edge
  #define bIE_RXD0_LO 0x01    // enable interrupt by RXD0 low level / falling edge

/*  FlashROM and Data-Flash Registers  */
#define ROM_PAGE_SIZE 0x40    // FlashROM page size ( number of bytes )
SFR16_(ROM_ADDR,0x84);        // address for flash-ROM, little-endian
SFR_(ROM_ADDR_L,0x84);        // address low byte for flash-ROM
SFR_(ROM_ADDR_H,0x85);        // address high byte for flash-ROM
SFR_16(ROM_DATA_LO,0x84);     // ReadOnly: low word data (16 bits) for flash-ROM reading, little-endian
SFR_(ROM_DATA_LL,0x84);       // ReadOnly: data low byte of low word for flash-ROM reading
SFR_(ROM_DATA_LH,0x85);       // ReadOnly: data high byte of low word for flash-ROM reading
SFR16_(ROM_DATA_HI,0x8E);     // ReadOnly: high word data (16 bits) for flash-ROM reading, little-endian
SFR_(ROM_DATA_HL,0x8E);       // ReadOnly: data low byte of high word for flash-ROM reading
SFR_(ROM_DATA_HH,0x8F);       // ReadOnly: data high byte of high word for flash-ROM reading
SFR_(ROM_DAT_BUF;0x8E);       // data byte buffer for flash-ROM program and erasing
SFR_(ROM_BUF_MOD,0x8F);       // data buffer mode and end address for flash-ROM program and erasing
  #define bROM_BUF_BYTE  0x80 // flash-ROM data buffer mode: 0=data block (1~64bytes) to program from xRAM pointed by DPTR, 1=only one byte for program or erasing from SFR ROM_DAT_BUF
  #define MASK_ROM_ADDR  0x3F // bit mask for end address for flash-ROM block program if bROM_BUF_BYTE=0
SFR_(ROM_CTRL,0x86);          // WriteOnly: flash-ROM control
  #define ROM_CMD_PROG   0x9A // WriteOnly: flash-ROM or Data-Flash byte/page program operation command, for changing some ROM bit of a byte from 0 to 1
  #define ROM_CMD_ERASE  0xA6 // WriteOnly: flash-ROM or Data-Flash page erase operation command, for changing all ROM bit of 64-Bytes from 1 to 0
  #define ROM_CMD_RD_OTP 0x8D // WriteOnly: OTP area dword read operation command
  #define ROM_CMD_PG_OTP 0x99 // WriteOnly: OTP area byte/page program operation command
SFR_(ROM_STATUS,0x86);        // ReadOnly: flash-ROM status
  #define bROM_ADDR_OK   0x40 // ReadOnly: flash-ROM operation address valid flag, can be reviewed before or after operation: 0=invalid parameter, 1=address valid
  #define bROM_CMD_ERR   0x02 // ReadOnly: flash-ROM operation command error flag: 0=command accepted, 1=unknown command or operation time out

/*  Port Registers  */
SFR_(P0,0x80);         // port 0 input & output
  SBIT_(P0_7 ,0x80,7);
  SBIT_(P0_6 ,0x80,6);
  SBIT_(P0_5 ,0x80,5);
  SBIT_(P0_4 ,0x80,4);
  SBIT_(P0_3 ,0x80,3);
  SBIT_(P0_2 ,0x80,2);
  SBIT_(P0_1 ,0x80,1);
  SBIT_(P0_0 ,0x80,0);
  SBIT_(TXD3 ,0x80,7);        // TXD output for UART3
  SBIT_(AIN15,0x80,7);        // AIN15 for ADC
  SBIT_(RXD3 ,0x80,6);        // RXD input for UART3
  SBIT_(AIN14,0x80,6);        // AIN14 for ADC
  SBIT_(TXD2 ,0x80,5);        // TXD output for UART2
  SBIT_(AIN13,0x80,5);        // AIN13 for ADC
  SBIT_(RXD2 ,0x80,4);        // RXD input for UART2
  SBIT_(AIN12,0x80,4);        // AIN12 for ADC
  SBIT_(TXD_ ,0x80,3);        // alternate pin for TXD of UART0
  SBIT_(AIN11,0x80,3);        // AIN11 for ADC
  SBIT_(RXD_ ,0x80,2);        // alternate pin for RXD of UART0
  SBIT_(AIN10,0x80,2);        // AIN10 for ADC
  SBIT_(AIN9 ,0x80,1);        // AIN9 for ADC
  SBIT_(AIN8 ,0x80,0);        // AIN8 for ADC
SFR_(P0_MOD_OC,0xC4);         // port 0 output mode: 0=push-pull, 1=open-drain
SFR_(P0_DIR_PU,0xC5);         // port 0 direction for push-pull or pullup enable for open-drain
  #define bTXD3     0x80      // TXD output for UART3
  #define bAIN15    0x80      // AIN15 for ADC
  #define bRXD3     0x40      // RXD input for UART3
  #define bAIN14    0x40      // AIN14 for ADC
  #define bTXD2     0x20      // TXD output for UART2
  #define bAIN13    0x20      // AIN13 for ADC
  #define bRXD2     0x10      // RXD input for UART2
  #define bAIN12    0x10      // AIN12 for ADC
  #define bTXD_     0x08      // alternate pin for TXD of UART0
  #define bAIN11    0x08      // AIN11 for ADC
  #define bRXD_     0x04      // alternate pin for RXD of UART0
  #define bAIN10    0x04      // AIN10 for ADC
  #define bAIN9     0x02      // AIN9 for ADC
  #define bAIN8     0x01      // AIN8 for ADC
SFR_(P1,0x90);                // port 1 input & output
  SBIT_(P1_7  ,0x90,7);
  SBIT_(P1_6  ,0x90,6);
  SBIT_(P1_5  ,0x90,5);
  SBIT_(P1_4  ,0x90,4);
  SBIT_(P1_3  ,0x90,3);
  SBIT_(P1_2  ,0x90,2);
  SBIT_(P1_1  ,0x90,1);
  SBIT_(P1_0  ,0x90,0);
  SBIT_(SCK   ,0x90,7);       // serial clock for SPI0
  SBIT_(TXD1_ ,0x90,7);       // alternate pin for TXD1
  SBIT_(AIN7  ,0x90,7);       // AIN7 for ADC
  SBIT_(MISO  ,0x90,6);       // master serial data input or slave serial data output for SPI0
  SBIT_(RXD1_ ,0x90,6);       // alternate pin for RXD1
  SBIT_(VBUS  ,0x90,6);       // VBUS for USB type-C
  SBIT_(AIN6  ,0x90,6);       // AIN6 for ADC
  SBIT_(MOSI  ,0x90,5);       // master serial data output or slave serial data input for SPI0
  SBIT_(PWM0_ ,0x90,5);       // alternate pin for PWM0
  SBIT_(UCC2  ,0x90,5);       // UCC2 for USB type-C
  SBIT_(AIN5  ,0x90,5);       // AIN5 for ADC
  SBIT_(SCS   ,0x90,4);       // slave chip-selection input for SPI0
  SBIT_(UCC1  ,0x90,4);       // UCC1 for USB type-C
  SBIT_(AIN4  ,0x90,4);       // AIN4 for ADC
  SBIT_(AIN3  ,0x90,3);       // AIN3 for ADC
  SBIT_(AIN2  ,0x90,2);       // AIN2 for ADC
  SBIT_(T2EX  ,0x90,1);       // external trigger input for timer2 reload & capture
  SBIT_(CAP2  ,0x90,1);       // capture2 input for timer2
  SBIT_(AIN1  ,0x90,1);       // AIN1 for ADC
  SBIT_(T2    ,0x90,0);       // external count input
  SBIT_(CAP1  ,0x90,0);       // capture1 input for timer2
  SBIT_(AIN0  ,0x90,0);       // AIN0 for ADC
SFR_(P1_MOD_OC,0x92);         // port 1 output mode: 0=push-pull, 1=open-drain
SFR_(P1_DIR_PU,0x93);         // port 1 direction for push-pull or pullup enable for open-drain
// Pn_MOD_OC & Pn_DIR_PU: pin input & output configuration for Pn (n=1/3)
//   0 0:  float input only, without pullup resistance
//   0 1:  push-pull output, strong driving high level and low level
//   1 0:  open-drain output and input without pullup resistance
//   1 1:  quasi-bidirectional (standard 8051 mode), open-drain output and input with pullup resistance, just driving high level strongly for 2 clocks if turning output level from low to high
  #define bSCK    0x80        // serial clock for SPI0
  #define bTXD1_  0x80        // alternate pin for TXD1
  #define bAIN7   0x80        // AIN7 for ADC
  #define bMISO   0x40        // master serial data input or slave serial data output for SPI0
  #define bRXD1_  0x40        // alternate pin for RXD1
  #define bVBUS   0x40        // VBUS for USB type-C
  #define bAIN6   0x40        // AIN6 for ADC
  #define bMOSI   0x20        // master serial data output or slave serial data input for SPI0
  #define bPWM0_  0x20        // alternate pin for PWM0
  #define bUCC2   0x20        // UCC2 for USB type-C
  #define bAIN5   0x20        // AIN5 for ADC
  #define bSCS    0x10        // slave chip-selection input for SPI0
  #define bUCC1   0x10        // UCC1 for USB type-C
  #define bAIN4   0x10        // AIN4 for ADC
  #define bAIN3   0x08        // AIN3 for ADC
  #define bAIN2   0x04        // AIN2 for ADC
  #define bT2EX   0x02        // external trigger input for timer2 reload & capture
  #define bCAP2   bT2EX       // capture2 input for timer2
  #define bAIN1   0x02        // AIN1 for ADC
  #define bT2     0x01        // external count input or clock output for timer2
  #define bCAP1   bT2         // capture1 input for timer2
  #define bAIN0   0x01        // AIN0 for ADC
SFR_(P2,0xA0);                // port 2 input & output
 SBIT_(P2_7  ,0xA0,7);
 SBIT_(P2_6  ,0xA0,6);
 SBIT_(P2_5  ,0xA0,5);
 SBIT_(P2_4  ,0xA0,4);
 SBIT_(P2_3  ,0xA0,3);
 SBIT_(P2_2  ,0xA0,2);
 SBIT_(P2_1  ,0xA0,1);
 SBIT_(P2_0  ,0xA0,0);
 SBIT_(PWM7  ,0xA0,7);        // PWM output for PWM7
 SBIT_(TXD1  ,0xA0,7);        // TXD output for UART1
 SBIT_(PWM6  ,0xA0,6);        // PWM output for PWM6
 SBIT_(RXD1  ,0xA0,6);        // RXD input for UART1
 SBIT_(PWM0  ,0xA0,5);        // PWM output for PWM0
 SBIT_(T2EX_ ,0xA0,5);        // alternate pin for T2EX
 SBIT_(CAP2_ ,0xA0,5);        // alternate pin for CAP2
 SBIT_(PWM1  ,0xA0,4);        // PWM output for PWM1
 SBIT_(T2_   ,0xA0,4);        // alternate pin for T2
 SBIT_(CAP1_ ,0xA0,4);        // alternate pin for CAP1
 SBIT_(PWM2  ,0xA0,3);        // PWM output for PWM2
 SBIT_(PWM3  ,0xA0,2);        // PWM output for PWM3
 SBIT_(INT0_ ,0xA0,2);        // alternate pin for INT0
 SBIT_(PWM4  ,0xA0,1);        // PWM output for PWM4
 SBIT_(PWM5  ,0xA0,0);        // PWM output for PWM5
SFR_(P2_MOD_OC,0x94);         // port 2 output mode: 0=push-pull, 1=open-drain
SFR_(P2_DIR_PU,0x95);         // port 2 direction for push-pull or pullup enable for open-drain
  #define bPWM7   0x80        // PWM output for PWM7
  #define bTXD1   0x80        // TXD output for UART1
  #define bPWM6   0x40        // PWM output for PWM6
  #define bRXD1   0x40        // RXD input for UART1
  #define bT2EX_  0x20        // alternate pin for T2EX
  #define bCAP2_  0x20        // alternate pin for CAP2
  #define bPWM0   0x20        // PWM output for PWM0
  #define bPWM1   0x10        // PWM output for PWM1
  #define bT2_    0x10        // alternate pin for T2
  #define bCAP1_  0x10        // alternate pin for CAP1
  #define bPWM2   0x08        // PWM output for PWM2
  #define bPWM3   0x04        // PWM output for PWM3
  #define bINT0_  0x04        // alternate pin for INT0
  #define bPWM4   0x02        // PWM output for PWM4
  #define bPWM5   0x01        // PWM output for PWM5
SFR_(P3,0xB0);                // port 3 input & output
  SBIT_(P3_7 ,0xB0,7);
  SBIT_(P3_6 ,0xB0,6);
  SBIT_(P3_5 ,0xB0,5);
  SBIT_(P3_4 ,0xB0,4);
  SBIT_(P3_3 ,0xB0,3);
  SBIT_(P3_2 ,0xB0,2);
  SBIT_(P3_1 ,0xB0,1);
  SBIT_(P3_0 ,0xB0,0);
  SBIT_(INT3 ,0xB0,7);        // external interrupt 3 input
  SBIT_(CAP0 ,0xB0,6);        // capture0 input for timer2
  SBIT_(T1   ,0xB0,5);        // external count input for timer1
  SBIT_(T0   ,0xB0,4);        // external count input for timer0
  SBIT_(INT1 ,0xB0,3);        // external interrupt 1 input
  SBIT_(INT0 ,0xB0,2);        // external interrupt 0 input
  SBIT_(TXD  ,0xB0,1);        // TXD output for UART0
  SBIT_(RXD  ,0xB0,0);        // RXD input for UART0
SFR_(P3_MOD_OC,0x96);         // port 3 output mode: 0=push-pull, 1=open-drain
SFR_(P3_DIR_PU,0x97);         // port 3 direction for push-pull or pullup enable for open-drain
  #define bINT3      0x80     // external interrupt 3 input
  #define bCAP0      0x40     // capture0 input for timer2
  #define bT1        0x20     // external count input for timer1
  #define bT0        0x10     // external count input for timer0
  #define bINT1      0x08     // external interrupt 1 input
  #define bINT0      0x04     // external interrupt 0 input
  #define bTXD       0x02     // TXD output for UART0
  #define bRXD       0x01     // RXD input for UART0
SFR_(P4,0xC0);                // port 4 input & output
  SBIT_(P4_6 ,0xC0,6);
  SBIT_(P4_5 ,0xC0,5);
  SBIT_(P4_4 ,0xC0,4);
  SBIT_(P4_3 ,0xC0,3);
  SBIT_(P4_2 ,0xC0,2);
  SBIT_(P4_1 ,0xC0,1);
  SBIT_(P4_0 ,0xC0,0);
  SBIT_(XO   ,0xC0,7);        // XO pin for external crystal oscillator
  SBIT_(XI   ,0xC0,6);        // XI pin for external crystal oscillator
SFR_(P4_MOD_OC,0xC2);         // port 4 output mode: 0=push-pull, 1=open-drain
SFR_(P4_DIR_PU,0xC3);         // port 4 direction for push-pull or pullup enable for open-drain
  #define bXO        0x80     // XO pin for external crystal oscillator
  #define bXI        0x40     // XI pin for external crystal oscillator
SFR_(P5,0xAB);                // port 5 input & output
  #define bRST       0x80     // ReadOnly: pin RST input
  #define bHVOD      0x20     // HVOD pin for high voltage open drain output
  #define bALE       0x10     // ALE pin for GPO
  #define bCKO       bALE     // ALE clock output
  #define bUDP       0x02     // ReadOnly: pin UDP input
  #define bDP        0x02     // ReadOnly: pin UDP input
  #define bUDM       0x01     // ReadOnly: pin UDM input
  #define bDM        0x01     // ReadOnly: pin UDM input
SFR_(PIN_FUNC,0xAA);          // pin function selection
  #define bPWM0_PIN_X    0x80 // PWM0 alternate pin enable: 0=PWM0 on P2.5, 1=PWM0 on P1.5
  #define bIO_INT_ACT    0x40 // ReadOnly: GPIO interrupt request action status
  #define bUART1_PIN_X   0x20 // UART1 alternate pin enable: 0=RXD1/TXD1 on P2.6/P2.7, 1=RXD1/TXD1 on P1.6/P1.7
  #define bUART0_PIN_X   0x10 // UART0 alternate pin enable: 0=RXD0/TXD0 on P3.0/P3.1, 1=RXD0/TXD0 on P0.2/P0.3
  #define bINT0_PIN_X    0x04 // INT0 alternate pin enable: 0=INT0 on P3.2, 1=INT0 on P2.2
  #define bT2EX_PIN_X    0x02 // T2EX/CAP2 alternate pin enable: 0=T2EX/CAP2 on P1.1, 1=T2EX/CAP2 on P2.5
  #define bT2_PIN_X      0x01 // T2/CAP1 alternate pin enable: 0=T2/CAP1 on P1.0, 1=T2/CAP1 on P2.4
SFR_(XBUS_AUX,0xA2);          // xBUS auxiliary setting
  #define bUART0_TX      0x80 // ReadOnly: indicate UART0 transmittal status
  #define bUART0_RX      0x40 // ReadOnly: indicate UART0 receiving status
  #define bSAFE_MOD_ACT  0x20 // ReadOnly: safe mode action status
  #define bALE_CLK_EN    0x10 // enable P5.4/ALE output clock
  #define bALE_CLK_SEL   0x08 // ALE clock frequency select if bALE_CLK_EN=1: 0-1/12 Fsys, 1-1/4 Fsys
  #define GF2            0x08 // general purpose flag bit 2 if bALE_CLK_EN=0
  #define bDPTR_AUTO_INC 0x04 // enable DPTR auto increase if finished MOVX_@DPTR instruction
  #define DPS            0x01 // dual DPTR selection: 0=DPTR0 selected, 1=DPTR1 selected

/*  Timer0/1 Registers  */
SFR_(TCON,0x88);              // timer 0/1 control and external interrupt control
  SBIT_(TF1 ,0x88,7);         // timer1 overflow & interrupt flag, auto cleared when MCU enter interrupt routine
  SBIT_(TR1 ,0x88,6);         // timer1 run enable
  SBIT_(TF0 ,0x88,5);         // timer0 overflow & interrupt flag, auto cleared when MCU enter interrupt routine
  SBIT_(TR0 ,0x88,4);         // timer0 run enable
  SBIT_(IE1 ,0x88,3);         // INT1 interrupt flag, auto cleared when MCU enter interrupt routine
  SBIT_(IT1 ,0x88,2);         // INT1 interrupt type: 0=low level action, 1=falling edge action
  SBIT_(IE0 ,0x88,1);         // INT0 interrupt flag, auto cleared when MCU enter interrupt routine
  SBIT_(IT0 ,0x88,0);         // INT0 interrupt type: 0=low level action, 1=falling edge action
SFR_(TMOD,0x89);              // timer 0/1 mode
  #define bT1_GATE    0x80    // gate control of timer1: 0=timer1 run enable while TR1=1, 1=timer1 run enable while P3.3 (INT1) pin is high and TR1=1
  #define bT1_CT      0x40    // counter or timer mode selection for timer1: 0=timer, use internal clock, 1=counter, use P3.5 (T1) pin falling edge as clock
  #define bT1_M1      0x20    // timer1 mode high bit
  #define bT1_M0      0x10    // timer1 mode low bit
  #define MASK_T1_MOD 0x30    // bit mask of timer1 mode
// bT1_M1 & bT1_M0: timer1 mode
//   00: mode 0, 13-bit timer or counter by cascaded TH1 and lower 5 bits of TL1, the upper 3 bits of TL1 are ignored
//   01: mode 1, 16-bit timer or counter by cascaded TH1 and TL1
//   10: mode 2, TL1 operates as 8-bit timer or counter, and TH1 provide initial value for TL1 auto-reload
//   11: mode 3, stop timer1
  #define bT0_GATE    0x08    // gate control of timer0: 0=timer0 run enable while TR0=1, 1=timer0 run enable while INT0 pin is high and TR0=1
  #define bT0_CT      0x04    // counter or timer mode selection for timer0: 0=timer, use internal clock, 1=counter, use P3.4 (T0) pin falling edge as clock
  #define bT0_M1      0x02    // timer0 mode high bit
  #define bT0_M0      0x01    // timer0 mode low bit
  #define MASK_T0_MOD 0x03    // bit mask of timer0 mode
// bT0_M1 & bT0_M0: timer0 mode
//   00: mode 0, 13-bit timer or counter by cascaded TH0 and lower 5 bits of TL0, the upper 3 bits of TL0 are ignored
//   01: mode 1, 16-bit timer or counter by cascaded TH0 and TL0
//   10: mode 2, TL0 operates as 8-bit timer or counter, and TH0 provide initial value for TL0 auto-reload
//   11: mode 3, TL0 is 8-bit timer or counter controlled by standard timer0 bits, TH0 is 8-bit timer using TF1 and controlled by TR1, timer1 run enable if it is not mode 3
SFR_(TL0,0x8A);               // low byte of timer 0 count
SFR_(TL1,0x8B);               // low byte of timer 1 count
SFR_(TH0,0x8C);               // high byte of timer 0 count
SFR_(TH1,0x8D);               // high byte of timer 1 count

/*  UART0 Registers  */
SFR_(SCON,0x98);              // UART0 control (serial port control)
  SBIT_(SM0 ,0x98,7);         // UART0 mode bit0, selection data bit: 0=8 bits data, 1=9 bits data
  SBIT_(SM1 ,0x98,6);         // UART0 mode bit1, selection baud rate: 0=fixed, 1=variable
// SM0 & SM1: UART0 mode
//    00 - mode 0, shift Register, baud rate fixed at: Fsys/12
//    01 - mode 1, 8-bit UART,     baud rate = variable by timer1 or timer2 overflow rate
//    10 - mode 2, 9-bit UART,     baud rate fixed at: Fsys/128@SMOD=0, Fsys/32@SMOD=1
//    11 - mode 3, 9-bit UART,     baud rate = variable by timer1 or timer2 overflow rate
  SBIT_( SM2,0x98,5);         // enable multi-device communication in mode 2/3
  #define MASK_UART0_MOD 0xE0 // bit mask of UART0 mode
  SBIT_(REN ,0x98,4);         // enable UART0 receiving
  SBIT_(TB8 ,0x98,3);         // the 9th transmitted data bit in mode 2/3
  SBIT_(RB8 ,0x98,2);         // 9th data bit received in mode 2/3, or stop bit received for mode 1
  SBIT_(TI  ,0x98,1);         // transmit interrupt flag, set by hardware after completion of a serial transmittal, need software clear
  SBIT_(RI  ,0x98,0);         // receive interrupt flag, set by hardware after completion of a serial receiving, need software clear
SFR_(SBUF ,0x99);             // UART0 data buffer: reading for receiving, writing for transmittal

/*  Timer2/Capture Registers  */
SFR_(T2CON2,0xC1);            // timer 2 extend control
  #define bT2_CAP0F    0x08   // timer2 capture 0 interrupt flag, set by CAP0 edge trigger if bT2_CAP0_EN=1, need software clear
  #define bT2_CAP0_EN  0x01   // enable CAP0 trigger function for capture 0 of timer2 if RCLK=0 & TCLK=0 & CP_RL2=1
SFR16_(T2CAP0,0xC6);          // ReadOnly: capture 0 value for timer2
SFR_(T2CAP0L,0xC6);           // ReadOnly: capture 0 value low byte for timer2
SFR_(T2CAP0H,0xC7);           // ReadOnly: capture 0 value high byte for timer2
SFR_(T2CON,0xC8);             // timer 2 control
  SBIT_(TF2    ,0xC8,7);      // timer2 overflow & interrupt flag, need software clear, the flag will not be set when either RCLK=1 or TCLK=1
  SBIT_(CAP1F  ,0xC8,7);      // timer2 capture 1 interrupt flag, set by T2 edge trigger if bT2_CAP1_EN=1, need software clear
  SBIT_(EXF2   ,0xC8,6);      // timer2 external flag, set by T2EX edge trigger if EXEN2=1, need software clear
  SBIT_(RCLK   ,0xC8,5);      // selection UART0 receiving clock: 0=timer1 overflow pulse, 1=timer2 overflow pulse
  SBIT_(TCLK   ,0xC8,4);      // selection UART0 transmittal clock: 0=timer1 overflow pulse, 1=timer2 overflow pulse
  SBIT_(EXEN2  ,0xC8,3);      // enable T2EX trigger function: 0=ignore T2EX, 1=trigger reload or capture by T2EX edge
  SBIT_(TR2    ,0xC8,2);      // timer2 run enable
  SBIT_(C_T2   ,0xC8,1);      // timer2 clock source selection: 0=timer base internal clock, 1=external edge counter base T2 falling edge
  SBIT_(CP_RL2 ,0xC8,0);      // timer2 function selection (force 0 if RCLK=1 or TCLK=1): 0=timer and auto reload if count overflow or T2EX edge, 1=capture by T2EX edge
SFR_(T2MOD,0xC9);             // timer 2 mode and timer 0/1/2 clock mode
  #define bTMR_CLK    0x80    // fastest internal clock mode for timer 0/1/2 under faster clock mode: 0=use divided clock, 1=use original Fsys as clock without dividing
  #define bT2_CLK     0x40    // timer2 internal clock frequency selection: 0=standard clock, Fsys/12 for timer mode, Fsys/4 for UART0 clock mode,
                              //   1=faster clock, Fsys/4 @bTMR_CLK=0 or Fsys @bTMR_CLK=1 for timer mode, Fsys/2 @bTMR_CLK=0 or Fsys @bTMR_CLK=1 for UART0 clock mode
  #define bT1_CLK     0x20    // timer1 internal clock frequency selection: 0=standard clock, Fsys/12, 1=faster clock, Fsys/4 if bTMR_CLK=0 or Fsys if bTMR_CLK=1
  #define bT0_CLK     0x10    // timer0 internal clock frequency selection: 0=standard clock, Fsys/12, 1=faster clock, Fsys/4 if bTMR_CLK=0 or Fsys if bTMR_CLK=1
  #define bT2_CAP_M1  0x08    // timer2 capture mode high bit
  #define bT2_CAP_M0  0x04    // timer2 capture mode low bit
// bT2_CAP_M1 & bT2_CAP_M0: timer2 capture point selection
//   x0: from falling edge to falling edge
//   01: from any edge to any edge (level changing)
//   11: from rising edge to rising edge
  #define T2OE         0x02   // enable timer2 generated clock output: 0=disable output, 1=enable clock output at T2 pin, frequency = TF2/2
  #define bT2_CAP1_EN  0x01   // enable T2 trigger function for capture 1 of timer2 if RCLK=0 & TCLK=0 & CP_RL2=1 & C_T2=0 & T2OE=0
SFR16_(RCAP2,0xCA);           // reload & capture value, little-endian
SFR_(RCAP2L,0xCA);            // low byte of reload & capture value
SFR_(RCAP2H,0xCB);            // high byte of reload & capture value
SFR16_(T2COUNT,0xCC);         // counter, little-endian
SFR_(TL2,0xCC);               // low byte of timer 2 count
SFR_(TH2,0xCD);               // high byte of timer 2 count
SFR16_(T2CAP1,0xCE);          // ReadOnly: capture 1 value for timer2
SFR_(T2CAP1L,0xCE);           // ReadOnly: capture 1 value low byte for timer2
SFR_(T2CAP1H,0xCF);           // ReadOnly: capture 1 value high byte for timer2

/*  PWMX Registers  */
SFR_(PWM_DATA2,0x9A);         // PWM data for PWM2
SFR_(PWM_DATA1,0x9B);         // PWM data for PWM1
SFR_(PWM_DATA0,0x9C);         // PWM data for PWM0
SFR_(PWM_CTRL,0x9D);          // PWM control
  #define bPWM1_POLAR   0x40  // PWM1 output polarity: 0=default low and high action, 1=default high and low action
  #define bPWM0_POLAR   0x20  // PWM0 output polarity: 0=default low and high action, 1=default high and low action
  #define bPWM_IF_END   0x10  // interrupt flag for cycle end, write 1 to clear or load new data into PWM_DATA0 to clear
  #define bPWM1_OUT_EN  0x08  // PWM1 output enable
  #define bPWM0_OUT_EN  0x04  // PWM0 output enable
  #define bPWM_CLR_ALL  0x02  // force clear FIFO and count of PWMX
  #define bPWM_MOD_6BIT 0x01  // PWM data width mode: 0=8 bits data, 1=6 bits data
SFR_(PWM_CK_SE,0x9E);         // clock divisor setting
SFR_(PWM_CTRL2,0x9F);         // PWM extend control
  #define bPWM7_OUT_EN  0x20  // PWM7 output enable
  #define bPWM6_OUT_EN  0x10  // PWM6 output enable
  #define bPWM5_OUT_EN  0x08  // PWM5 output enable
  #define bPWM4_OUT_EN  0x04  // PWM4 output enable
  #define bPWM3_OUT_EN  0x02  // PWM3 output enable
  #define bPWM2_OUT_EN  0x01  // PWM2 output enable
SFR_(PWM_DATA3,0xA3);         // PWM data for PWM3
SFR_(PWM_DATA4,0xA4);         // PWM data for PWM4
SFR_(PWM_DATA5,0xA5);         // PWM data for PWM5
SFR_(PWM_DATA6,0xA6);         // PWM data for PWM6
SFR_(PWM_DATA7,0xA7);         // PWM data for PWM7

/*  SPI0/Master0/Slave Registers  */
SFR_(SPI0_STAT,0xF8);         // SPI 0 status
 SBIT_(S0_FST_ACT ,0xF8,7);   // ReadOnly: indicate first byte received status for SPI0
 SBIT_(S0_IF_OV   ,0xF8,6);   // interrupt flag for slave mode FIFO overflow, direct bit address clear or write 1 to clear
 SBIT_(S0_IF_FIRST,0xF8,5);   // interrupt flag for first byte received, direct bit address clear or write 1 to clear
 SBIT_(S0_IF_BYTE ,0xF8,4);   // interrupt flag for a byte data exchanged, direct bit address clear or write 1 to clear or accessing FIFO to clear if bS0_AUTO_IF=1
 SBIT_(S0_FREE    ,0xF8,3);   // ReadOnly: SPI0 free status
 SBIT_(S0_T_FIFO  ,0xF8,2);   // ReadOnly: tx FIFO count for SPI0
 SBIT_(S0_R_FIFO  ,0xF8,0);   // ReadOnly: rx FIFO count for SPI0
SFR_(SPI0_DATA,0xF9);         // FIFO data port: reading for receiving, writing for transmittal
SFR_(SPI0_CTRL,0xFA);         // SPI 0 control
  #define bS0_MISO_OE   0x80  // SPI0 MISO output enable
  #define bS0_MOSI_OE   0x40  // SPI0 MOSI output enable
  #define bS0_SCK_OE    0x20  // SPI0 SCK output enable
  #define bS0_DATA_DIR  0x10  // SPI0 data direction: 0=out(master_write), 1=in(master_read)
  #define bS0_MST_CLK   0x08  // SPI0 master clock mode: 0=mode 0 with default low, 1=mode 3 with default high
  #define bS0_2_WIRE    0x04  // enable SPI0 two wire mode: 0=3 wire (SCK+MOSI+MISO), 1=2 wire (SCK+MISO)
  #define bS0_CLR_ALL   0x02  // force clear FIFO and count of SPI0
  #define bS0_AUTO_IF   0x01  // enable FIFO accessing to auto clear S0_IF_BYTE interrupt flag
SFR_(SPI0_CK_SE,0xFB);        // clock divisor setting
SFR_(SPI0_S_PRE,0xFB);        // preset value for SPI slave
SFR_(SPI0_SETUP,0xFC);        // SPI 0 setup
  #define bS0_MODE_SLV    0x80// SPI0 slave mode: 0=master, 1=slave
  #define bS0_IE_FIFO_OV  0x40// enable interrupt for slave mode FIFO overflow
  #define bS0_IE_FIRST    0x20// enable interrupt for first byte received for SPI0 slave mode
  #define bS0_IE_BYTE     0x10// enable interrupt for a byte received
  #define bS0_BIT_ORDER   0x08// SPI0 bit data order: 0=MSB first, 1=LSB first
  #define bS0_SLV_SELT    0x02// ReadOnly: SPI0 slave mode chip selected status: 0=unselected, 1=selected
  #define bS0_SLV_PRELOAD 0x01// ReadOnly: SPI0 slave mode data pre-loading status just after chip-selection

/*  UART1 Registers  */
SFR_(SCON1,0xBC);             // UART1 control (serial port control)
  #define bU1SM0       0x80   // UART1 mode, selection data bit: 0=8 bits data, 1=9 bits data
  #define bU1SMOD      0x20   // UART1 2X baud rate selection: 0=slow(Fsys/32/(256-SBAUD1)), 1=fast(Fsys/16/(256-SBAUD1))
  #define bU1REN       0x10   // enable UART1 receiving
  #define bU1TB8       0x08   // the 9th transmitted data bit in 9 bits data mode
  #define bU1RB8       0x04   // 9th data bit received in 9 bits data mode, or stop bit received for 8 bits data mode
  #define bU1TIS       0x02   // WriteOnly: write 1 to preset transmit interrupt flag
  #define bU1RIS       0x01   // WriteOnly: write 1 to preset receive interrupt flag
SFR_(SBUF1 ,0xBD);            // UART1 data buffer: reading for receiving, writing for transmittal
SFR_(SBAUD1,0xBE);            // UART1 baud rate setting
SFR_(SIF1  ,0xBF);            // UART1 interrupt flag
  #define bU1TI        0x02   // transmit interrupt flag, set by hardware after completion of a serial transmittal, need software write 1 to clear
  #define bU1RI        0x01   // receive interrupt flag, set by hardware after completion of a serial receiving, need software write 1 to clear

/*  UART2 Registers  */
SFR_(SCON2,0xB4);             // UART2 control (serial port control)
  #define bU2SM0       0x80   // UART2 mode, selection data bit: 0=8 bits data, 1=9 bits data
  #define bU2IE        0x40   // UART2 interrupt enable: 0=disable interrupt, 1=enable interrupt instead of ADC
  #define bU2SMOD      0x20   // UART2 2X baud rate selection: 0=slow(Fsys/32/(256-SBAUD2)), 1=fast(Fsys/16/(256-SBAUD2))
  #define bU2REN       0x10   // enable UART2 receiving
  #define bU2TB8       0x08   // the 9th transmitted data bit in 9 bits data mode
  #define bU2RB8       0x04   // 9th data bit received in 9 bits data mode, or stop bit received for 8 bits data mode
  #define bU2TIS       0x02   // WriteOnly: write 1 to preset transmit interrupt flag
  #define bU2RIS       0x01   // WriteOnly: write 1 to preset receive interrupt flag
SFR_(SBUF2 ,0xB5);            // UART2 data buffer: reading for receiving, writing for transmittal
SFR_(SBAUD2,0xB6);            // UART2 baud rate setting
SFR_(SIF2  ,0xB7);            // UART2 interrupt flag
  #define bU2TI        0x02   // transmit interrupt flag, set by hardware after completion of a serial transmittal, need software write 1 to clear
  #define bU2RI        0x01   // receive interrupt flag, set by hardware after completion of a serial receiving, need software write 1 to clear

/*  UART3 Registers  */
SFR_(SCON3,0xAC);             // UART3 control (serial port control)
  #define bU3SM0       0x80   // UART3 mode, selection data bit: 0=8 bits data, 1=9 bits data
  #define bU3IE        0x40   // UART3 interrupt enable: 0=disable interrupt, 1=enable interrupt instead of PWMX
  #define bU3SMOD      0x20   // UART3 2X baud rate selection: 0=slow(Fsys/32/(256-SBAUD3)), 1=fast(Fsys/16/(256-SBAUD3))
  #define bU3REN       0x10   // enable UART3 receiving
  #define bU3TB8       0x08   // the 9th transmitted data bit in 9 bits data mode
  #define bU3RB8       0x04   // 9th data bit received in 9 bits data mode, or stop bit received for 8 bits data mode
  #define bU3TIS       0x02   // WriteOnly: write 1 to preset transmit interrupt flag
  #define bU3RIS       0x01   // WriteOnly: write 1 to preset receive interrupt flag
SFR_(SBUF3 ,0xAD);            // UART3 data buffer: reading for receiving, writing for transmittal
SFR_(SBAUD3,0xAE);            // UART3 baud rate setting
SFR_(SIF3  ,0xAF);            // UART3 interrupt flag
  #define bU3TI        0x02   // transmit interrupt flag, set by hardware after completion of a serial transmittal, need software write 1 to clear
  #define bU3RI        0x01   // receive interrupt flag, set by hardware after completion of a serial receiving, need software write 1 to clear

/*  ADC and comparator and touch-key Registers  */
SFR_(TKEY_CTRL,0xF1);         // WriteOnly: touch-key charging pulse width control (only low 7 bits valid), auto cleared
SFR_(ADC_CTRL ,0xF2);         // ADC control and status
  #define bCMPDO       0x80   // ReadOnly: comparator result synchronous delay input
  #define bCMP_IF      0x40   // interrupt flag for comparator result changed, write 1 to clear
  #define bADC_IF      0x20   // interrupt flag for ADC finished, write 1 to clear or write TKEY_CTRL to clear
  #define bADC_START   0x10   // set 1 to start ADC, auto cleared when ADC finished
  #define bTKEY_ACT    0x08   // ReadOnly: indicate touch-key running status (charge then ADC)
  #define bCMPO        0x01   // ReadOnly: comparator result real time input
SFR_(ADC_CFG,0xF3);           // ADC config
  #define bADC_AIN_EN  0x20   // enable external AIN on ADC/comparator IN+ channel: 0=disable AIN on ADC/comparator IN+ channel, 1=enable external AIN by MASK_ADC_CHAN
  #define bVDD_REF_EN  0x10   // enable internal VDD reference voltage: 0=disable VDD resistances, 1=enable VDD series connection resistances to get internal reference voltage
  #define bADC_EN      0x08   // control ADC power: 0=shut down ADC, 1=enable power for ADC
  #define bCMP_EN      0x04   // control comparator power: 0=shut down comparator, 1=enable power for comparator, auto wake-up if comparator result changed
  #define bADC_CLK1    0x02   // ADC clock frequency selection high bit
  #define bADC_CLK0    0x01   // ADC clock frequency selection low bit
// bADC_CLK1 & bADC_CLK0: ADC clock frequency selection
//   00: slowest clock, 512 Fosc cycles for each ADC
//   01: slower clock, 256 Fosc cycles for each ADC
//   10: faster clock, 128 Fosc cycles for each ADC
//   11: fastest clock, 64 Fosc cycles for each ADC
SFR16_(ADC_DAT,0xF4);         // ReadOnly: ADC data
SFR_(ADC_DAT_L,0xF4);         // ReadOnly: ADC data low byte
SFR_(ADC_DAT_H,0xF5);         // ReadOnly: ADC data high byte (only low 4 bits valid)
SFR_(ADC_CHAN ,0xF6);         // ADC analog signal channel seletion
  #define MASK_CMP_CHAN 0xC0  // bit mask of comparator IN- input signal channel selection
  // bCMP_EN & bVDD_REF_EN & MASK_CMP_CHAN[1] & [0]: comparator IN- input signal channel selection
  //   0xxx: disconnect / float
  //   1000: disconnect / float
  //   1100: connect VDD/8
  //   1001: connect VDD
  //   1101: connect VDD/4
  //   1x10: connect AIN1(P1.1)
  //   1x11: connect AIN2(P1.2)
  #define MASK_ADC_I_CH  0x30 // bit mask of ADC/comparator IN+ input internal signal channel selection
  // bADC_EN & bADC_AIN_EN & bVDD_REF_EN & MASK_ADC_I_CH[1] & [0]: ADC/comparator IN+ input internal signal channel selection
  //   xx000: disconnect internal signal / float
  //   xx100: connect VDD/2
  //   xxx01: connect V33
  //   xxx10: connect V33/1.83
  //   10x11: connect temperature sense
  //   0xx11: disconnect internal signal / float
  //   x1x11: disconnect internal signal / float
  #define MASK_ADC_CHAN  0x0F // bit mask of ADC/comparator IN+ input external signal channel selection if bADC_AIN_EN=1
  // bADC_AIN_EN & MASK_ADC_CHAN[3] & [2] & [1] & [0]: ADC/comparator IN+ input external signal channel selection
  //   0xxxx: disconnect external signal AIN0~AIN15 / float
  //   10000: connect AIN0(P1.0)
  //   10001: connect AIN1(P1.1)
  //   10010: connect AIN2(P1.2)
  //   10011: connect AIN3(P1.3)
  //   10100: connect AIN4(P1.4)
  //   10101: connect AIN5(P1.5)
  //   10110: connect AIN6(P1.6)
  //   10111: connect AIN7(P1.7)
  //   11000: connect AIN8(P0.0)
  //   11001: connect AIN9(P0.1)
  //   11010: connect AIN10(P0.2)
  //   11011: connect AIN11(P0.3)
  //   11100: connect AIN12(P0.4)
  //   11101: connect AIN13(P0.5)
  //   11110: connect AIN14(P0.6)
  //   11111: connect AIN15(P0.7)
SFR_(ADC_PIN,0xF7);           // ADC pin digital input control
  #define bAIN14_15_DI_DIS 0x80 // control AIN14/AIN15 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN12_13_DI_DIS 0x40 // control AIN12/AIN13 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN10_11_DI_DIS 0x20 // control AIN10/AIN11 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN8_9_DI_DIS   0x10 // control AIN8/AIN9 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN6_7_DI_DIS   0x08 // control AIN6/AIN7 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN4_5_DI_DIS   0x04 // control AIN4/AIN5 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN2_3_DI_DIS   0x02 // control AIN2/AIN3 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN0_1_DI_DIS   0x01 // control AIN0/AIN1 digital input: 0=enable digital input, 1=disable digital input

/*  USB/Host/Device Registers  */
SFR_(USB_C_CTRL,0x91);        // USB type-C control
  #define bUCC_PD_MOD   0x80  // USB UCC1/UCC2 power delivery BMC output mode enable: 0=disable, 1=enable PD output mode
  #define bUCC2_PD_EN   0x40  // USB UCC2 5.1K pulldown resistance: 0=disable, 1=enable pulldown
  #define bUCC2_PU1_EN  0x20  // USB UCC2 pullup resistance control high bit
  #define bUCC2_PU0_EN  0x10  // USB UCC2 pullup resistance control low bit
  #define bVBUS_PD_EN   0x08  // USB VBUS 10K pulldown resistance: 0=disable, 1=enable pullup
  #define bUCC1_PD_EN   0x04  // USB UCC1 5.1K pulldown resistance: 0=disable, 1=enable pulldown
  #define bUCC1_PU1_EN  0x02  // USB UCC1 pullup resistance control high bit
  #define bUCC1_PU0_EN  0x01  // USB UCC1 pullup resistance control low bit
  // bUCC?_PU1_EN & bUCC?_PU0_EN: USB CC pullup resistance selection
  //   00: disable pullup resistance
  //   01: enable 56K pullup resistance for default USB power
  //   10: enable 22K pullup resistance for 1.5A USB power
  //   11: enable 10K pullup resistance for 3A USB power
SFR_(UDEV_CTRL,0xD1);         // USB device physical port control
  #define bUD_PD_DIS    0x80  // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
  #define bUD_DP_PIN    0x20  // ReadOnly: indicate current UDP pin level
  #define bUD_DM_PIN    0x10  // ReadOnly: indicate current UDM pin level
  #define bUD_LOW_SPEED 0x04  // enable USB physical port low speed: 0=full speed, 1=low speed
  #define bUD_GP_BIT    0x02  // general purpose bit
  #define bUD_PORT_EN   0x01  // enable USB physical port I/O: 0=disable, 1=enable
SFR_(UHOST_CTRL,0xD1);        // USB host physical port control
  #define bUH_PD_DIS     0x80 // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
  #define bUH_DP_PIN     0x20 // ReadOnly: indicate current UDP pin level
  #define bUH_DM_PIN     0x10 // ReadOnly: indicate current UDM pin level
  #define bUH_LOW_SPEED  0x04 // enable USB port low speed: 0=full speed, 1=low speed
  #define bUH_BUS_RESET  0x02 // control USB bus reset: 0=normal, 1=force bus reset
  #define bUH_PORT_EN    0x01 // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached
SFR_(UEP1_CTRL,0xD2);         // endpoint 1 control
  #define bUEP_R_TOG     0x80 // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
  #define bUEP_T_TOG     0x40 // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
  #define bUEP_AUTO_TOG  0x10 // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
  #define bUEP_R_RES1    0x08 // handshake response type high bit for USB endpoint X receiving (OUT)
  #define bUEP_R_RES0    0x04 // handshake response type low bit for USB endpoint X receiving (OUT)
  #define MASK_UEP_R_RES 0x0C // bit mask of handshake response type for USB endpoint X receiving (OUT)
  #define UEP_R_RES_ACK   0x00
  #define UEP_R_RES_TOUT  0x04
  #define UEP_R_RES_NAK   0x08
  #define UEP_R_RES_STALL 0x0C
  // bUEP_R_RES1 & bUEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
  //   00: ACK (ready)
  //   01: no response, time out to host, for non-zero endpoint isochronous transactions
  //   10: NAK (busy)
  //   11: STALL (error)
  #define bUEP_T_RES1     0x02// handshake response type high bit for USB endpoint X transmittal (IN)
  #define bUEP_T_RES0     0x01// handshake response type low bit for USB endpoint X transmittal (IN)
  #define MASK_UEP_T_RES  0x03// bit mask of handshake response type for USB endpoint X transmittal (IN)
  #define UEP_T_RES_ACK   0x00
  #define UEP_T_RES_TOUT  0x01
  #define UEP_T_RES_NAK   0x02
  #define UEP_T_RES_STALL 0x03
  // bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
  //   00: DATA0 or DATA1 then expecting ACK (ready)
  //   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
  //   10: NAK (busy)
  //   11: STALL (error)
SFR_(UEP1_T_LEN ,0xD3);       // endpoint 1 transmittal length
SFR_(UEP2_CTRL  ,0xD4);       // endpoint 2 control
SFR_(UEP2_T_LEN ,0xD5);       // endpoint 2 transmittal length
SFR_(UEP3_CTRL  ,0xD6);       // endpoint 3 control
SFR_(UEP3_T_LEN ,0xD7);       // endpoint 3 transmittal length
SFR_(USB_INT_FG ,0xD8);       // USB interrupt flag
  SBIT_(U_IS_NAK    ,0xD8,7); // ReadOnly: indicate current USB transfer is NAK received
  SBIT_(U_TOG_OK    ,0xD8,6); // ReadOnly: indicate current USB transfer toggle is OK
  SBIT_(U_SIE_FREE  ,0xD8,5); // ReadOnly: indicate USB SIE free status
  SBIT_(UIF_FIFO_OV ,0xD8,4); // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
  SBIT_(UIF_HST_SOF ,0xD8,3); // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
  SBIT_(UIF_SUSPEND ,0xD8,2); // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
  SBIT_(UIF_TRANSFER,0xD8,1); // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
  SBIT_(UIF_DETECT  ,0xD8,0); // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
  SBIT_(UIF_BUS_RST ,0xD8,0); // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear
SFR_(USB_INT_ST,0xD9);        // ReadOnly: USB interrupt status
  #define bUIS_IS_NAK     0x80// ReadOnly: indicate current USB transfer is NAK received for USB device mode
  #define bUIS_TOG_OK     0x40// ReadOnly: indicate current USB transfer toggle is OK
  #define bUIS_TOKEN1     0x20// ReadOnly: current token PID code bit 1 received for USB device mode
  #define bUIS_TOKEN0     0x10// ReadOnly: current token PID code bit 0 received for USB device mode
  #define MASK_UIS_TOKEN  0x30// ReadOnly: bit mask of current token PID code received for USB device mode
  #define UIS_TOKEN_OUT   0x00
  #define UIS_TOKEN_SOF   0x10
  #define UIS_TOKEN_IN    0x20
  #define UIS_TOKEN_SETUP 0x30
  // bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode
  //   00: OUT token PID received
  //   01: SOF token PID received
  //   10: IN token PID received
  //   11: SETUP token PID received
  #define MASK_UIS_ENDP   0x0F// ReadOnly: bit mask of current transfer endpoint number for USB device mode
  #define MASK_UIS_H_RES  0x0F// ReadOnly: bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received
SFR_(USB_MIS_ST,0xDA);        // ReadOnly: USB miscellaneous status
  #define bUMS_SOF_PRES   0x80// ReadOnly: indicate host SOF timer presage status
  #define bUMS_SOF_ACT    0x40// ReadOnly: indicate host SOF timer action status for USB host
  #define bUMS_SIE_FREE   0x20// ReadOnly: indicate USB SIE free status
  #define bUMS_R_FIFO_RDY 0x10// ReadOnly: indicate USB receiving FIFO ready status (not empty)
  #define bUMS_BUS_RESET  0x08// ReadOnly: indicate USB bus reset status
  #define bUMS_SUSPEND    0x04// ReadOnly: indicate USB suspend status
  #define bUMS_DM_LEVEL   0x02// ReadOnly: indicate UDM level saved at device attached to USB host
  #define bUMS_DEV_ATTACH 0x01// ReadOnly: indicate device attached status on USB host
SFR_(USB_RX_LEN,0xDB);        // ReadOnly: USB receiving length
SFR_(UEP0_CTRL ,0xDC);        // endpoint 0 control
SFR_(UEP0_T_LEN,0xDD);        // endpoint 0 transmittal length
SFR_(UEP4_CTRL ,0xDE);        // endpoint 4 control
SFR_(UEP4_T_LEN,0xDF);        // endpoint 4 transmittal length
SFR_(USB_INT_EN,0xE1);        // USB interrupt enable
  #define bUIE_DEV_SOF  0x80  // enable interrupt for SOF received for USB device mode
  #define bUIE_DEV_NAK  0x40  // enable interrupt for NAK responded for USB device mode
  #define bUIE_FIFO_OV  0x10  // enable interrupt for FIFO overflow
  #define bUIE_HST_SOF  0x08  // enable interrupt for host SOF timer action for USB host mode
  #define bUIE_SUSPEND  0x04  // enable interrupt for USB suspend or resume event
  #define bUIE_TRANSFER 0x02  // enable interrupt for USB transfer completion
  #define bUIE_DETECT   0x01  // enable interrupt for USB device detected event for USB host mode
  #define bUIE_BUS_RST  0x01  // enable interrupt for USB bus reset event for USB device mode
SFR_(USB_CTRL,0xE2);          // USB base control
  #define bUC_HOST_MODE 0x80  // enable USB host mode: 0=device mode, 1=host mode
  #define bUC_LOW_SPEED 0x40  // enable USB low speed: 0=full speed, 1=low speed
  #define bUC_DEV_PU_EN 0x20  // USB device enable and internal pullup resistance enable
  #define bUC_SYS_CTRL1 0x20  // USB system control high bit
  #define bUC_SYS_CTRL0 0x10  // USB system control low bit
  #define MASK_UC_SYS_CTRL 0x30      // bit mask of USB system control
  // bUC_HOST_MODE & bUC_SYS_CTRL1 & bUC_SYS_CTRL0: USB system control
  //   0 00: disable USB device and disable internal pullup resistance
  //   0 01: enable USB device and disable internal pullup resistance, need external pullup resistance
  //   0 1x: enable USB device and enable internal pullup resistance
  //   1 00: enable USB host and normal status
  //   1 01: enable USB host and force UDP/UDM output SE0 state
  //   1 10: enable USB host and force UDP/UDM output J state
  //   1 11: enable USB host and force UDP/UDM output resume or K state
  #define bUC_INT_BUSY  0x08  // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
  #define bUC_RESET_SIE 0x04  // force reset USB SIE, need software clear
  #define bUC_CLR_ALL   0x02  // force clear FIFO and count of USB
  #define bUC_DMA_EN    0x01  // DMA enable and DMA interrupt enable for USB
SFR_(USB_DEV_AD,0xE3);        // USB device address, lower 7 bits for USB device address
  #define bUDA_GP_BIT   0x80  // general purpose bit
  #define MASK_USB_ADDR 0x7F  // bit mask for USB device address
SFR16_(UEP2_DMA,0xE4);        // endpoint 2 buffer start address, little-endian
SFR_(UEP2_DMA_L,0xE4);        // endpoint 2 buffer start address low byte
SFR_(UEP2_DMA_H,0xE5);        // endpoint 2 buffer start address high byte
SFR16_(UEP3_DMA,0xE6);        // endpoint 3 buffer start address, little-endian
SFR_(UEP3_DMA_L,0xE6);        // endpoint 3 buffer start address low byte
SFR_(UEP3_DMA_H,0xE7);        // endpoint 3 buffer start address high byte
SFR_(UEP4_1_MOD,0xEA);        // endpoint 4/1 mode
  #define bUEP1_RX_EN    0x80 // enable USB endpoint 1 receiving (OUT)
  #define bUEP1_TX_EN    0x40 // enable USB endpoint 1 transmittal (IN)
  #define bUEP1_BUF_MOD  0x10 // buffer mode of USB endpoint 1
  // bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
  //   0 0 x:  disable endpoint and disable buffer
  //   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
  //   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
  //   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
  //   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
  //   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
  //   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
  #define bUEP4_RX_EN  0x08   // enable USB endpoint 4 receiving (OUT)
  #define bUEP4_TX_EN  0x04   // enable USB endpoint 4 transmittal (IN)
  // bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
  //   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
  //   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
  //   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
  //   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
  //           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes
SFR_(UEP2_3_MOD,0xEB);        // endpoint 2/3 mode
  #define bUEP3_RX_EN   0x80  // enable USB endpoint 3 receiving (OUT)
  #define bUEP3_TX_EN   0x40  // enable USB endpoint 3 transmittal (IN)
  #define bUEP3_BUF_MOD 0x10  // buffer mode of USB endpoint 3
  #define bUEP2_RX_EN   0x08  // enable USB endpoint 2 receiving (OUT)
  #define bUEP2_TX_EN   0x04  // enable USB endpoint 2 transmittal (IN)
  #define bUEP2_BUF_MOD 0x01  // buffer mode of USB endpoint 2
SFR16_(UEP0_DMA,0xEC);        // endpoint 0 buffer start address, little-endian
SFR_(UEP0_DMA_L,0xEC);        // endpoint 0 buffer start address low byte
SFR_(UEP0_DMA_H,0xED);        // endpoint 0 buffer start address high byte
SFR16_(UEP1_DMA,0xEE);        // endpoint 1 buffer start address, little-endian
SFR_(UEP1_DMA_L,0xEE);        // endpoint 1 buffer start address low byte
SFR_(UEP1_DMA_H,0xEF);        // endpoint 1 buffer start address high byte
SFR_(UH_SETUP  ,0xD2);        // host aux setup
  #define bUH_PRE_PID_EN 0x80 // USB host PRE PID enable for low speed device via hub
  #define bUH_SOF_EN     0x40 // USB host automatic SOF enable
SFR_(UH_RX_CTRL,0xD4);        // host receiver endpoint control
  #define bUH_R_TOG      0x80 // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
  #define bUH_R_AUTO_TOG 0x10 // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
  #define bUH_R_RES      0x04 // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions
SFR_(UH_EP_PID,0xD5);         // host endpoint and token PID, lower 4 bits for endpoint number, upper 4 bits for token PID
  #define MASK_UH_TOKEN 0xF0  // bit mask of token PID for USB host transfer
  #define MASK_UH_ENDP  0x0F  // bit mask of endpoint number for USB host transfer
SFR_(UH_TX_CTRL,0xD6);        // host transmittal endpoint control
  #define bUH_T_TOG      0x40 // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
  #define bUH_T_AUTO_TOG 0x10 // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
  #define bUH_T_RES      0x01 // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions
SFR_(UH_TX_LEN,0xD7);         // host transmittal endpoint transmittal length
SFR_(UH_EP_MOD,0xEB);         // host endpoint mode
  #define bUH_EP_TX_EN    0x40// enable USB host OUT endpoint transmittal
  #define bUH_EP_TBUF_MOD 0x10// buffer mode of USB host OUT endpoint
  // bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
  //   0 x:  disable endpoint and disable buffer
  //   1 0:  64 bytes buffer for transmittal (OUT endpoint)
  //   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
  #define bUH_EP_RX_EN    0x08// enable USB host IN endpoint receiving
  #define bUH_EP_RBUF_MOD 0x01// buffer mode of USB host IN endpoint
  // bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
  //   0 x:  disable endpoint and disable buffer
  //   1 0:  64 bytes buffer for receiving (IN endpoint)
  //   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes
SFR16_(UH_RX_DMA,0xE4);       // host rx endpoint buffer start address, little-endian
SFR_(UH_RX_DMA_L,0xE4);       // host rx endpoint buffer start address low byte
SFR_(UH_RX_DMA_H,0xE5);       // host rx endpoint buffer start address high byte
SFR16_(UH_TX_DMA,0xE6);       // host tx endpoint buffer start address, little-endian
SFR_(UH_TX_DMA_L,0xE6);       // host tx endpoint buffer start address low byte
SFR_(UH_TX_DMA_H,0xE7);       // host tx endpoint buffer start address high byte

/*----- XDATA: xRAM ------------------------------------------*/

#define XDATA_RAM_SIZE 0x0800 // size of expanded xRAM, xdata SRAM embedded chip

/*----- Reference Information --------------------------------------------*/
#define ID_CH549       0x49   // chip ID
#define ID_CH548       0x48   // chip ID

/* Interrupt routine address and interrupt number */
#define INT_ADDR_INT0     0x0003    // interrupt vector address for INT0
#define INT_ADDR_TMR0     0x000B    // interrupt vector address for timer0
#define INT_ADDR_INT1     0x0013    // interrupt vector address for INT1
#define INT_ADDR_TMR1     0x001B    // interrupt vector address for timer1
#define INT_ADDR_UART0    0x0023    // interrupt vector address for UART0
#define INT_ADDR_TMR2     0x002B    // interrupt vector address for timer2
#define INT_ADDR_SPI0     0x0033    // interrupt vector address for SPI0
#define INT_ADDR_INT3     0x003B    // interrupt vector address for INT3
#define INT_ADDR_USB      0x0043    // interrupt vector address for USB
#define INT_ADDR_ADC      0x004B    // interrupt vector address for ADC/UART2
#define INT_ADDR_UART2    0x004B    // interrupt vector address for ADC/UART2
#define INT_ADDR_UART1    0x0053    // interrupt vector address for UART1
#define INT_ADDR_PWMX     0x005B    // interrupt vector address for PWMX/UART3
#define INT_ADDR_UART3    0x005B    // interrupt vector address for PWMX/UART3
#define INT_ADDR_GPIO     0x0063    // interrupt vector address for GPIO
#define INT_ADDR_WDOG     0x006B    // interrupt vector address for watch-dog timer
#define INT_NO_INT0       0         // interrupt number for INT0
#define INT_NO_TMR0       1         // interrupt number for timer0
#define INT_NO_INT1       2         // interrupt number for INT1
#define INT_NO_TMR1       3         // interrupt number for timer1
#define INT_NO_UART0      4         // interrupt number for UART0
#define INT_NO_TMR2       5         // interrupt number for timer2
#define INT_NO_SPI0       6         // interrupt number for SPI0
#define INT_NO_INT3       7         // interrupt number for INT3
#define INT_NO_USB        8         // interrupt number for USB
#define INT_NO_ADC        9         // interrupt number for ADC/UART2
#define INT_NO_UART2      9         // interrupt number for ADC/UART2
#define INT_NO_UART1      10        // interrupt number for UART1
#define INT_NO_PWMX       11        // interrupt number for PWMX/UART3
#define INT_NO_UART3      11        // interrupt number for PWMX/UART3
#define INT_NO_GPIO       12        // interrupt number for GPIO
#define INT_NO_WDOG       13        // interrupt number for watch-dog timer

/* Special Program Space */
#define DATA_FLASH_ADDR   0xF000    // start address of Data-Flash
#define BOOT_LOAD_ADDR    0xF400    // start address of boot loader program
#define ROM_CFG_ADDR      0xFFFE    // chip configuration information address
#define ROM_CHIP_ID_LO    0x10      // chip ID number low dword
#define ROM_CHIP_ID_HI    0x14      // chip ID number high dword

/*
New Instruction:   MOVX @DPTR1,A
Instruction Code:  0xA5
Instruction Cycle: 1
Instruction Operation:
   step-1. write ACC @DPTR1 into xdata SRAM embedded chip
   step-2. increase DPTR1
ASM example:
       INC  XBUS_AUX
       MOV  DPTR,#TARGET_ADDR ;DPTR1
       DEC  XBUS_AUX
       MOV  DPTR,#SOURCE_ADDR ;DPTR0
       MOV  R7,#xxH
 LOOP: MOVX A,@DPTR ;DPTR0
       INC  DPTR    ;DPTR0, if need
       DB   0A5H    ;MOVX @DPTR1,A & INC DPTR1
       DJNZ R7,LOOP
*/
#define CH549_H_
#endif  // CH549_H_

