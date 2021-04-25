
#ifndef CH557_H_

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
  SBIT_(ACC7,0xE0,7);
  SBIT_(ACC6,0xE0,6);  
  SBIT_(ACC5,0xE0,5);
  SBIT_(ACC4,0xE0,4);
  SBIT_(ACC3,0xE0,3);
  SBIT_(ACC2,0xE0,2);
  SBIT_(ACC1,0xE0,1);
  SBIT_(ACC0,0xE0,0);
SFR_(A_INV,0xFD);             // ReadOnly: bit inversion of ACC (bit0<->bit7,bit1<->bit6,bit2<->bit5,bit3<->bit4, [7..0]->[0..7])
SFR_(B,0xF0);                 // general purpose register B
  SBIT_(B_07,0xF0,7);
  SBIT_(B_06,0xF0,6);  
  SBIT_(B_05,0xF0,5);
  SBIT_(B_04,0xF0,4);
  SBIT_(B_03,0xF0,3);
  SBIT_(B_02,0xF0,2);
  SBIT_(B_01,0xF0,1);
  SBIT_(B_00,0xF0,0);
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
  #define bXIR_XSFR      0x02 // force MOVX_@R0/@R1 only for xSFR area: 0=MOVX_@R0/@R1 for standard xdata area inclde xRAM&xBUS&xSFR, 1=MOVX_@R0/@R1 for xSFR only
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
  #define bCMP_RESULT    0x40 // ReadOnly: result of comparator: 0=lower then reference voltage, 1=higher then reference voltage
  #define bLV_RST_OFF    0x20 // disable low voltage reset: 0=enable LVR, 1=disable LVR
  #define bLDO_3V3_OFF   0x10 // disable 5V->3.3V LDO: 0=enable LDO for USB I/O, 1=disable 3.3V LDO, short V33 and VDD power
  #define bLDO_CORE_VOL  0x08 // core voltage mode: 0=normal, 1=raised for performance
  #define MASK_ULLDO_VOL 0x07 // bit mask of ULLDO voltage selection

SFR_(CLOCK_CFG,0xB9);         // system clock config: lower 3 bits for system clock Fsys, Write@SafeMode
  #define bOSC_EN_INT     0x80// internal oscillator enable and original clock selection: 1=enable & select internal clock, 0=disable & select external clock
  #define bOSC_EN_XT      0x40// external oscillator enable, need quartz crystal or ceramic resonator between XI and XO pins
  #define bWDOG_IF_TO     0x20// ReadOnly: watch-dog timer overflow interrupt flag, cleared by reload watch-dog count or auto cleared when MCU enter interrupt routine
  #define MASK_SYS_CK_SEL 0x07// bit mask of system clock Fsys selection
   //Fxt  = 24MHz (12MHz~30MHz for non-USB application), from external oscillator @XI&XO
   //Fosc = bOSC_EN_INT ? 24MHz : Fxt
   //Fpll = Fosc * 4 => 96MHz (48MHz~120MHz for non-USB application)
   //Fusb4x = Fpll / 2 => 48MHz (Fixed)
   //           MASK_SYS_CK_SEL[2] [1] [0]
   //Fsys = Fpll/2   =  48MHz:  1   1   1
   //Fsys = Fpll/3   =  32MHz:  1   1   0
   //Fsys = Fpll/4   =  24MHz:  1   0   1
   //Fsys = Fpll/6   =  16MHz:  1   0   0
   //Fsys = Fpll/8   =  12MHz:  0   1   1
   //Fsys = Fpll/32  =   3MHz:  0   1   0
   //Fsys = Fpll/128 = 750KHz:  0   0   1
   //Fsys = Fpll/512 =187.5KHz: 0   0   0

SFR_(WAKE_CTRL,0xA9);         // wake-up control, Write@SafeMode
  #define bWAK_BY_USB    0x80 // enable wake-up by USB event
  #define bWAK_RXD1_LO   0x40 // enable wake-up by RXD1 low level
  #define bWAK_P1_5_LO   0x20 // enable wake-up by pin P1.5 low level
  #define bWAK_P1_4_LO   0x10 // enable wake-up by pin P1.4 low level
  #define bWAK_P3_3_LO   0x04 // enable wake-up by pin P3.3 low level
  #define bWAK_INT0_EDGE 0x02 // enable wake-up by INT0 edge
  #define bWAK_RXD0_LO   0x01 // enable wake-up by RXD0 low level
SFR_(RESET_KEEP,0xFE);       // value keeper during reset
SFR_(WDOG_COUNT,0xFF);       // watch-dog count, count by clock frequency Fsys/131072

//EXTERN  UINT8XV CMP_DCDC       _AT_ 0x21EB;   // comparator and DC-DC control
  //#define pCMP_DCDC      PBYTE[0xEB]
  #define bDCDC_ACT      0x80 // ReadOnly: DC-DC output action status: 0=free, 1=action
  #define bDCDC_PIN      0x40 // DC-DC output pin and polarity selection: 0=DCO output bDCDC_ACT, 1=DCO output ! bDCDC_ACT, P6.4 output bDCDC_ACT
  #define bDCDC_FREQ1    0x20 // DC-DC reference clock frequency selection high bit
  #define bDCDC_FREQ0    0x10 // DC-DC reference clock frequency selection low bit
  #define MASK_DCDC_FREQ 0x30 // bit mask of DC-DC reference clock frequency selection
  // bDCDC_FREQ1 & bDCDC_FREQ0: DC-DC reference clock frequency if bCMP_VR_SEL2 & bCMP_VR_SEL1 & bCMP_VR_SEL0 != 000
  //    00 - disable DC-DC
  //    01 - reference 3MHz, max output frequency is 1MHz
  //    10 - reference 1.5MHz, max output frequency is 500KHz
  //    11 - reference 750KHz, max output frequency is 250KHz
  // bDCDC_FREQ1 & bDCDC_FREQ0: bDCDC_ACT control if bCMP_VR_SEL2 & bCMP_VR_SEL1 & bCMP_VR_SEL0 == 000
  //    00 - bDCDC_ACT=0
  //    x1/1x - bDCDC_ACT=1
  #define bCMP_PIN       0x08 // comparator input pin selection: 0=VDD, 1=analog pin input shared with ADC
  #define bCMP_VR_SEL2   0x04 // comparator reference voltage selection high bit
  #define bCMP_VR_SEL1   0x02 // comparator reference voltage selection middle bit
  #define bCMP_VR_SEL0   0x01 // comparator reference voltage selection low bit
  #define MASK_CMP_VREF  0x07 // bit mask of comparator reference voltage selection
  // bCMP_VR_SEL2 & bCMP_VR_SEL1 & bCMP_VR_SEL0: comparator reference voltage selection
  //    000 - disable comparator, bDCDC_ACT controlled by bDCDC_FREQ1|bDCDC_FREQ0
  //    001 - reference 1.2V without voltage divider, bDCDC_ACT controlled by DC-DC
  //    010 - voltage divider by resistance for compare 3.3V, bDCDC_ACT controlled by DC-DC
  //    011 - voltage divider by resistance for compare 5.0V, bDCDC_ACT controlled by DC-DC
  //    100 - voltage divider by resistance for compare 5.4V, bDCDC_ACT controlled by DC-DC
  //    101 - voltage divider by resistance for compare 5.8V, bDCDC_ACT controlled by DC-DC
  //    110 - voltage divider by resistance for compare 6.2V, bDCDC_ACT controlled by DC-DC
  //    111 - voltage divider by resistance for compare 6.6V, bDCDC_ACT controlled by DC-DC
  
/*  Interrupt Registers  */
SFR_(IE,0xA8);                // interrupt enable
  SBIT_(EA,0xA8,7);           // enable global interrupts: 0=disable, 1=enable if E_DIS=0
  SBIT_(E_DIS,0xA8,6);        // disable global interrupts, intend to inhibit interrupt during some flash-ROM operation: 0=enable if EA=1, 1=disable
  SBIT_(ET2,0xA8,5);          // enable timer2 interrupt
  SBIT_(ES,0xA8,4);           // enable UART0 interrupt
  SBIT_(ET1,0xA8,3);          // enable timer1 interrupt
  SBIT_(EX1,0xA8,2);          // enable external interrupt INT1
  SBIT_(ET0,0xA8,1);          // enable timer0 interrupt
  SBIT_(EX0,0xA8,0);          // enable external interrupt INT0 or LED interrupt
SFR_(IP,0xB8);                // interrupt priority and current priority
  SBIT_(PH_FLAG,0xB8,7);      // ReadOnly: high level priority action flag
  SBIT_(PL_FLAG,0xB8,6);      // ReadOnly: low level priority action flag0xA8// PH_FLAG & PL_FLAG: current interrupt priority
  SBIT_(PT2,0xB8,5);          // timer2 interrupt priority level
  SBIT_(PS ,0xB8,4);          // UART0 interrupt priority level
  SBIT_(PT1,0xB8,3);          // timer1 interrupt priority level
  SBIT_(PX1,0xB8,2);          // external interrupt INT1 priority level
  SBIT_(PT0,0xB8,1);          // timer0 interrupt priority level
  SBIT_(PX0,0xB8,0);          // external interrupt INT0 priority level
//    00 - no interrupt now
//    01 - low level priority interrupt action now
//    10 - high level priority interrupt action now
//    11 - unknown error
SFR_(IE_EX,0xE8);             // extend interrupt enable
  SBIT_(IE_WDOG ,0xE8,7);     // enable watch-dog timer interrupt
  SBIT_(IE_GPIO ,0xE8,6);     // enable GPIO input interrupt
  SBIT_(IE_PWM_I2C ,0xE8,5);  // enable PWMX/LED/I2C interrupt
  SBIT_(IE_UART1,0xE8,4);     // enable UART1 interrupt
  SBIT_(IE_ADC  ,0xE8,3);     // enable ADC interrupt
  SBIT_(IE_USB  ,0xE8,2);     // enable USB interrupt  
  SBIT_(IE_SPI0 ,0xE8,0);     // enable SPI0 interrupt

SFR_(IP_EX,0xE9);             // extend interrupt priority
  #define bIP_LEVEL    0x80   // ReadOnly: current interrupt nested level: 0=no interrupt or two levels, 1=one level
  #define bIP_GPIO     0x40   // GPIO input interrupt priority level
  #define bIP_PWM_I2C  0x20   // PWMX/LED/I2C interrupt priority level
  #define bIP_UART1    0x10   // UART1 interrupt priority level
  #define bIP_ADC      0x08   // ADC interrupt priority level
  #define bIP_USB      0x04   // USB interrupt priority level
  #define bIP_SPI0     0x01   // SPI0 interrupt priority level

SFR_(GPIO_IE,0xCF);           // GPIO interrupt enable Write@SafeMode
  #define bIE_IO_EDGE  0x80   // enable GPIO edge interrupt: 0=low/high level, 1=falling/rising edge
  #define bIE_RXD1_LO  0x40   // enable interrupt by RXD1 low level / falling edge
  #define bIE_P1_5_LO  0x20   // enable interrupt by pin P1.5 low level / falling edge
  #define bIE_P1_4_LO  0x10   // enable interrupt by pin P1.4 low level / falling edge
  #define bIE_P0_3_LO  0x08   // enable interrupt by pin P0.3 low level / falling edge
  #define bIE_P5_3X5X7 0x04   // enable interrupt by pin P5.3/P5.5/P5.7 level changing
  #define bIE_P7_1_LO  0x02   // enable interrupt by pin P7.1 low level / falling edge if bOSC_EN_XT=0
  #define bIE_CMP_RES_LO 0x02 // enable interrupt by bCMP_RESULT low / falling edge if MASK_CMP_VREF!=000
  #define bIE_RXD0_LO  0x01   // enable interrupt by RXD0 low level / falling edge


/*  FlashROM and Data-Flash Registers  */
#define ROM_PAGE_SIZE  0x40   // FlashROM page size ( number of bytes )

SFR16_(ROM_ADDR,   0x84);     // address for flash-ROM, little-endian
SFR16_(ROM_DATA_LO,0x84);     // ReadOnly: low word data (16 bits) for flash-ROM reading, little-endian  
SFR16_(ROM_DATA_HI,0x8E);     // ReadOnly: high word data (16 bits) for flash-ROM reading, little-endian

SFR_(ROM_ADDR_L, 0x84);       // address low byte for flash-ROM
SFR_(ROM_ADDR_H, 0x85);       // address high byte for flash-ROM
SFR_(ROM_DATA_LL,0x84);       // ReadOnly: data low byte of low word for flash-ROM reading 
SFR_(ROM_DATA_LH,0x84);       // ReadOnly: data low byte of low word for flash-ROM reading// ReadOnly: data high byte of low word for flash-ROM reading 
SFR_(ROM_DATA_L, 0x8E);       // data low byte for flash-ROM writing
SFR_(ROM_DATA_H, 0x8F);       // data high byte for flash-ROM writing
SFR_(ROM_DATA_HL,0x8E);       // ReadOnly: data low byte of high word for flash-ROM reading
SFR_(ROM_DATA_HH,0x8F);       // ReadOnly: data high byte of high word for flash-ROM reading
SFR_(ROM_DAT_BUF,0x8E);       // data byte buffer for flash-ROM program and erasing

SFR_(ROM_BUF_MOD,0x8F);       // data buffer mode and end address for flash-ROM program and erasing
  #define bROM_BUF_BYTE  0x80 // flash-ROM data buffer mode: 0=data block (1~64bytes) to program from xRAM pointed by DPTR, 1=only one byte for program or erasing from SFR ROM_DAT_BUF
  #define MASK_ROM_ADDR  0x3F // bit mask for end address for flash-ROM block program if bROM_BUF_BYTE=0

SFR_(ROM_CTRL  ,0x86);        // WriteOnly: flash-ROM control
  #define ROM_CMD_PROG   0x9A // WriteOnly: flash-ROM word program operation command, for changing some ROM bit of a word from 1 to 0
  #define ROM_CMD_ERASE  0xA6 // WriteOnly: flash-ROM sector erase operation command, for changing all ROM bit of 1KBytes from 0 to 1
  #define ROM_CMD_RD_OTP 0x8D // WriteOnly: OTP area dword read operation command
  #define ROM_CMD_PG_OTP 0x99 // WriteOnly: OTP area byte/page program operation command

SFR_(ROM_STATUS,0x86);        // ReadOnly: flash-ROM status
  #define bROM_ADDR_OK  0x40  // ReadOnly: flash-ROM operation address valid flag, can be reviewed before or after operation: 0=invalid parameter, 1=address valid
  #define bROM_CMD_ERR  0x02  // ReadOnly: flash-ROM operation command error flag: 0=command accepted, 1=unknown command

/*  Port Registers  */
SFR_(P0,0x80);                // port 0 input & output
  SBIT_(P0_7,0x80,7);         // generic port bits
  SBIT_(P0_6,0x80,6);
  SBIT_(P0_5,0x80,5);
  SBIT_(P0_4,0x80,4);
  SBIT_(P0_3,0x80,3);
  SBIT_(P0_2,0x80,2);
  SBIT_(P0_1,0x80,1);
  SBIT_(P0_0,0x80,0);
  SBIT_(SDA0,0x80,1);         // SDA input/output for I2CS
  SBIT_(SCL0,0x80,0);         // SCL input for I2CS
  SBIT_(TXD_,0x80,3);         // alternate pin for TXD of UART0
  SBIT_(RXD_,0x80,2);         // alternate pin for RXD of UART0
  SBIT_(AIN13,0x80,5);        // AIN13 for ADC
  SBIT_(AIN12,0x80,4);        // AIN12 for ADC
  SBIT_(AIN11,0x80,3);        // AIN11 for ADC
  SBIT_(AIN10,0x80,2);        // AIN10 for ADC
  SBIT_(AIN9 ,0x80,2);        // AIN9 for ADC
  SBIT_(AIN8 ,0x80,0);        // AIN8 for ADC

SFR_(P0_MOD_OC,0xC4);         // port 0 output mode: 0=push-pull, 1=open-drain
SFR_(P0_DIR_PU,0xC5);         // port 0 direction for push-pull or pullup enable for open-drain
  #define bSDA0     0x02      // SDA input/output for I2CS
  #define bSCL0     0x01      // SCL input for I2CS
  #define bTXD_     0x08      // alternate pin for TXD of UART0
  #define bRXD_     0x04      // alternate pin for RXD of UART0
  #define bAIN13    0x20      // AIN13 for ADC
  #define bAIN12    0x10      // AIN12 for ADC
  #define bAIN11    0x08      // AIN11 for ADC
  #define bAIN10    0x04      // AIN10 for ADC
  #define bAIN9     0x02      // AIN9 for ADC
  #define bAIN8     0x01      // AIN8 for ADC

SFR_(P1   ,0x90);             // port 1 input & output, not 5VT
  SBIT_(P1_0,0x90,0);         // generic port bits
  SBIT_(P1_1,0x90,1);
  SBIT_(P1_2,0x90,2);
  SBIT_(P1_3,0x90,3);
  SBIT_(P1_4,0x90,4);
  SBIT_(P1_5,0x90,5);
  SBIT_(P1_6,0x90,6);
  SBIT_(P1_7,0x90,7);

  SBIT_(AIN7,0x90,7);         // AIN7 for ADC, not 5VT
  SBIT_(AIN6,0x90,6);         // AIN6 for ADC, not 5VT
  SBIT_(AIN5,0x90,5);         // AIN5 for ADC, not 5VT
  SBIT_(AIN4,0x90,4);         // AIN4 for ADC, not 5VT
  SBIT_(AIN3,0x90,3);         // AIN3 for ADC, not 5VT
  SBIT_(AIN2,0x90,2);         // AIN2 for ADC, not 5VT
  SBIT_(AIN1,0x90,1);         // AIN1 for ADC, not 5VT
  SBIT_(AIN0,0x90,0);         // AIN0 for ADC, not 5VT

  SBIT_(SCK ,0x90,7);         // serial clock for SPI0, not 5VT
  SBIT_(MISO,0x90,6);         // master serial data input or slave serial data output for SPI0, not 5VT
  SBIT_(MOSI,0x90,5);         // master serial data output or slave serial data input for SPI0, not 5VT
  SBIT_(SCS ,0x90,4);         // slave chip-selection input for SPI0, not 5VT
  SBIT_(INT0_,0x90,2);        // alternate pin for INT0

  SBIT_(CAP2,0x90,2);         // capture2 input for timer2
  SBIT_(T2EX,0x90,1);         // external trigger input for timer2 reload & capture, not 5VT
  //SBIT_(CAP2,0x90,1);       // capture2 input for timer2, not 5VT
  SBIT_(T2  ,0x90,0);         // external count input, not 5VT
  SBIT_(CAP1,0x90,0);         // capture1 input for timer2, not 5VT

  SBIT_(TXD1_,0x90,7);        // alternate pin for TXD1
  SBIT_(RXD1_,0x90,6);        // alternate pin for RXD1
  SBIT_(PWM0_,0x90,5);        // alternate pin for PWM0
SFR_(P1_MOD_OC,0x92);         // port 1 output mode: 0=push-pull, 1=open-drain

SFR_(P1_DIR_PU,0x93);         // port 1 direction for push-pull or pullup enable for open-drain
  // Pn_MOD_OC & Pn_DIR_PU: pin input & output configuration for Pn (n=0/1/2/3)
  //   0 0:  float input only, without pullup resistance
  //   0 1:  push-pull output, strong driving high level and low level
  //   1 0:  open-drain output and input without pullup resistance
  //   1 1:  quasi-bidirectional (standard 8051 mode), open-drain output and input with pullup resistance, just driving high level strongly for 2 clocks if turning output level from low to high
  #define bTXD1_    0x80      // alternate pin for TXD1
  #define bRXD1_    0x40      // alternate pin for RXD1
  #define bPWM0_    0x20      // alternate pin for PWM0
  #define bSCK      0x80      // serial clock for SPI0
  #define bMISO     0x40      // master serial data input or slave serial data output for SPI0
  #define bMOSI     0x20      // master serial data output or slave serial data input for SPI0
  #define bSCS      0x10      // slave chip-selection input for SPI0
  #define bINT0_    0x04      // alternate pin for INT0
  #define bT2EX     0x02      // external trigger input for timer2 reload & capture
  #define bCAP2     bT2EX     // capture2 input for timer2
  #define bT2       0x01      // external count input or clock output for timer2
  #define bCAP1     bT2       // capture1 input for timer2
  #define bAIN7     0x80      // AIN7 for ADC
  #define bAIN6     0x40      // AIN6 for ADC
  #define bAIN5     0x20      // AIN5 for ADC
  #define bAIN4     0x10      // AIN4 for ADC
  #define bAIN3     0x08      // AIN3 for ADC
  #define bAIN2     0x04      // AIN2 for ADC
  #define bAIN1     0x02      // AIN1 for ADC
  #define bAIN0     0x01      // AIN0 for ADC

SFR_(P2,0xA0);                // port 2 input & output
  SBIT_(P2_7 ,0xA0,7);
  SBIT_(P2_6 ,0xA0,6);
  SBIT_(P2_5 ,0xA0,5);
  SBIT_(P2_4 ,0xA0,4);
  SBIT_(P2_3 ,0xA0,3);
  SBIT_(P2_2 ,0xA0,2);
  SBIT_(P2_1 ,0xA0,1);
  SBIT_(P2_0 ,0xA0,0);
  SBIT_(TXD1 ,0xA0,7);        // TXD output for UART1
  SBIT_(RXD1 ,0xA0,6);        // RXD input for UART1
  SBIT_(T2EX_,0xA0,5);        // alternate pin for T2EX
  SBIT_(CAP2_,0xA0,5);        // alternate pin for CAP2
  SBIT_(T2_  ,0xA0,4);        // alternate pin for T2
  SBIT_(CAP1_,0xA0,4);        // alternate pin for CAP1
  SBIT_(BUZZ ,0xA0,4);        // buzzer output
  SBIT_(PWM0 ,0xA0,5);        // PWM output for PWM0
  SBIT_(PWM1 ,0xA0,4);        // PWM output for PWM1
  SBIT_(PWM2 ,0xA0,3);        // PWM output for PWM2
  SBIT_(PWM3 ,0xA0,2);        // PWM output for PWM3
  SBIT_(PWM4 ,0xA0,1);        // PWM output for PWM4
  SBIT_(PWM5 ,0xA0,0);        // PWM output for PWM5
SFR_( P2_MOD_OC,0x94);        // port 2 output mode: 0=push-pull, 1=open-drain
SFR_( P2_DIR_PU,0x95);        // port 2 direction for push-pull or pullup enable for open-drain
  #define bTXD1       0x80    // TXD output for UART1
  #define bRXD1       0x40    // RXD input for UART1
  #define bT2EX_      0x20    // alternate pin for T2EX
  #define bCAP2_      0x20    // alternate pin for CAP2
  #define bT2_        0x10    // alternate pin for T2
  #define bCAP1_      0x10    // alternate pin for CAP1
  #define bBUZZ       0x10    // buzzer output
  #define bPWM0       0x20    // PWM output for PWM0
  #define bPWM1       0x10    // PWM output for PWM1
  #define bPWM2       0x08    // PWM output for PWM2
  #define bPWM3       0x04    // PWM output for PWM3
  #define bPWM4       0x02    // PWM output for PWM4
  #define bPWM5       0x01    // PWM output for PWM5

SFR_(P3,0xB0);                // port 3 input & output
 SBIT_(P3_7 ,0xB0,7);
 SBIT_(P3_6 ,0xB0,6);
 SBIT_(P3_5 ,0xB0,5);
 SBIT_(P3_4 ,0xB0,4);
 SBIT_(P3_3 ,0xB0,3);
 SBIT_(P3_2 ,0xB0,2);
 SBIT_(P3_1 ,0xB0,1);
 SBIT_(P3_0 ,0xB0,0);
 SBIT_(SCK1 ,0xB0,7);         // serial clock output for SPI1
 SBIT_(MISO1,0xB0,6);         // master serial data input for SPI1
 SBIT_(MOSI1,0xB0,5);         // master serial data output for SPI1
 SBIT_(T1   ,0xB0,5);         // external count input for timer1
 SBIT_(T0   ,0xB0,4);         // external count input for timer0
 SBIT_(MSDA ,0xB0,4);         // SDA input/output for I2C master
 SBIT_(MSCL ,0xB0,3);         // SCL output for I2C master
 SBIT_(INT1 ,0xB0,3);         // external interrupt 1 input
 SBIT_(INT0 ,0xB0,2);         // external interrupt 0 input
 SBIT_(TXD  ,0xB0,1);         // TXD output for UART0
 SBIT_(RXD  ,0xB0,0);         // RXD input for UART0
SFR_(P3_MOD_OC,0x96);         // port 3 output mode: 0=push-pull, 1=open-drain
SFR_(P3_DIR_PU,0x97);         // port 3 direction for push-pull or pullup enable for open-drain
  #define bSCK1    0x80       // serial clock output for SPI1
  #define bMISO1   0x40       // master serial data input for SPI1
  #define bMOSI1   0x20       // master serial data output for SPI1
  #define bT1      0x20       // external count input for timer1
  #define bT0      0x10       // external count input for timer0
  #define bMSDA    0x10       // SDA input/output for I2C master
  #define bMSCL    0x08       // SCL output for I2C master
  #define bINT1    0x08       // external interrupt 1 input
  #define bINT0    0x04       // external interrupt 0 input
  #define bTXD     0x02       // TXD output for UART0
  #define bRXD     0x01       // RXD input for UART0
SFR_(P4,0xC0);                // port 4 input & output
  SBIT_(P4_6,0xC0,6);
  SBIT_(P4_5,0xC0,5);
  SBIT_(P4_4,0xC0,4);
  SBIT_(P4_3,0xC0,3);
  SBIT_(P4_2,0xC0,2);
  SBIT_(P4_1,0xC0,1);
  SBIT_(P4_0,0xC0,0);
SFR_(P4_MOD_OC ,0xC2);        // port 4 output mode: 0=push-pull, 1=open-drain
SFR_(P4_DIR_PU ,0xC3);        // port 4 direction for push-pull or pullup enable for open-drain
SFR_(P4_LED_KEY,0xC1);        // port 4 LED drive mode or keyboard input mode
  // P4_MOD_OC & P4_DIR_PU & P4_LED_KEY: pin input & output configuration for P4
  //   0 0 x:  float input only, without pullup resistance
  //   0 1 0:  push-pull output, strong driving high level and low level
  //   1 0 0:  open-drain output and input without pullup resistance
  //   1 1 0:  quasi-bidirectional (standard 8051 mode), open-drain output and input with pullup resistance, just driving high level strongly for 2 clocks if turning output level from low to high
  //   0 1 1:  push-pull output, strong driving high level and limited driving low level for LED
  //   1 0 1:  open-drain output and input without pullup resistance, key input @high-resistance
  //   1 1 1:  quasi-bidirectional (standard 8051 mode), open-drain output and input with pullup resistance, just driving high level strongly for 2 clocks if turning output level from low to high, key input @pullup
SFR_(P5_IN    ,0xAA);         // ReadOnly: port 5 input
SFR_(P5_OUT_PU,0xAB);         // port 5 output data for output direction or pullup enable for input direction
SFR_(P5_DIR   ,0xAC);         // port 5 direction: 0=input, 1=output
  // P5_DIR P5_OUT_PU function for P5
  //  0      1       input with pullup resistance (7K5)
  //  1      0       push-pull output low
  //  1      1       push-pull output high
  //  0      0       float input without pullup/pulldown resistance (default) @P5.2~7
  //  0      0       input with pulldown resistance (15K) by bUH?_PD_EN of USBHUB1/2/3 @P5.2~7
  //  0      0       float input without pullup resistance (weak pulldown 1000K) (default) @P5.0/1
  //  0      0       input with pullup resistance (1K5) by bUC_DEV_PU_EN of USBHUB0 @P5.0/1
  //  0      0       input with pulldown resistance (15K) by bUH?_PD_EN of USBHUB0 @P5.0/1
  #define bHP3      0x80      // ReadOnly: pin HP3 input for USB hub3
  #define bHM3      0x40      // ReadOnly: pin HM3 input for USB hub3
  #define bHP2      0x20      // ReadOnly: pin HP2 input for USB hub2
  #define bHM2      0x10      // ReadOnly: pin HM2 input for USB hub2
  #define bHP1      0x08      // ReadOnly: pin HP1 input for USB hub1
  #define bHM1      0x04      // ReadOnly: pin HM1 input for USB hub1
  #define bHP0      0x02      // ReadOnly: pin DP/HP0 input for USB hub0
  #define bHM0      0x01      // ReadOnly: pin DM/HM0 input for USB hub0
  #define bALE_     0x04      // alternate pin for ALE/clock output if P5_DIR[2]=1 & bALE_CLK_EN=1
  #define bDP       0x02      // ReadOnly: pin DP/HP0 input for USB host/device
  #define bDM       0x01      // ReadOnly: pin DM/HM0 input for USB host/device
SFR_(P6_IN    ,0xAD);         // ReadOnly: port 6 input
SFR_(P6_OUT_PU,0xAE);         // port 6 output data for output direction or pullup enable for input direction
SFR_(P6_DIR   ,0xAF);         // port 6 direction: 0=input, 1=output
  // P6_dir P6_OUT_PU function for P6
  //  0      0       float input without pullup resistance (weak pulldown 1000K) (default)
  //  0      1       input with pullup resistance (7K5)
  //  1      0       push-pull output low
  //  1      1       push-pull output high
  #define bDCO_          0x10 // alternate pin for DCO output if bDCDC_PIN=1, polarity controlled by P6_OUT_PU[4]
SFR_(P7,0xF1);                // port 7 input & output & buzzer frequency
  #define bBUZZ_FREQ1    0x80 // buzzer output frequency selection high bit
  #define bBUZZ_FREQ0    0x40 // buzzer output frequency selection low bit
  #define MASK_BUZZ_FREQ 0xC0 // bit mask of buzzer output frequency selection
  // bBUZZ_FREQ1 & bBUZZ_FREQ0: buzzer frequency
  //    00 - disable buzzer output
  //    01 - buzzer output 1KHz
  //    10 - buzzer output 667KHz
  //    11 - buzzer output 500KHz
  #define bXO            0x20 // XO pin for external crystal oscillator
  #define bXI            0x10 // XI pin for external crystal oscillator
  #define bRST           0x20 // manual reset input, low action
  #define bALE           0x02 // ALE/clock output if P5_DIR[2]=0 & bALE_CLK_EN=1
  #define bPWM1_         0x01 // alternate pin for PWM1
  #define bP7_1_IN       0x20 // ReadOnly: P7.1 input
  #define bP7_0_IN       0x10 // ReadOnly: P7.0 input
  #define bP7_1_DIR      0x08 // P7.1 direction: 0=input, 1=output
  #define bP7_0_DIR      0x04 // P7.0 direction: 0=input, 1=output
  #define bP7_1_OUT_PU   0x02 // P7.1 output data for output direction or pullup enable for input direction
  #define bP7_0_OUT_PU   0x01 // P7.0 output data for output direction or pullup enable for input direction
  // bP7_n_DIR bP7_n_OUT_PU bOSC_EN_XT function for P7 (n=0/1)
  //  0      0       0             float input without pullup resistance
  //  0      1       0             input with pullup resistance (default)
  //  1      0       0             push-pull output low
  //  1      1       0             push-pull output high
  //  x/0    x/0     1             XI/XO for external crystal oscillator
SFR_(XBUS_AUX,0xA2);          // xBUS auxiliary setting
  #define bUART0_TX      0x80 // ReadOnly: indicate UART0 transmittal status
  #define bUART0_RX      0x40 // ReadOnly: indicate UART0 receiving status
  #define bSAFE_MOD_ACT  0x20 // ReadOnly: safe mode action status
  #define bALE_CLK_EN    0x10 // enable ALE(P5.2/P7.1) output clock
  #define bALE_CLK_SEL   0x08 // ALE clock frequency select if bALE_CLK_EN=1: 0-1/12 Fsys, 1-1/4 Fsys
  #define GF2            0x08 // general purpose flag bit 2 if bALE_CLK_EN=0
  #define bDPTR_AUTO_INC 0x04 // enable DPTR auto increase if finished MOVX_@DPTR instruction
  #define DPS            0x01 // dual DPTR selection: 0=DPTR0 selected, 1=DPTR1 selected

//EXTERN  UINT8XV PIN_FUNC       _AT_ 0x21E9;   // pin function selection
//#define pPIN_FUNC         PBYTE[0xE9]
  #define bPWM1_PIN_X    0x80 // PWM1 alternate pin enable: 0=PWM1 on P2.4, 1=PWM1 on P7.0
  #define bPWM0_PIN_X    0x40 // PWM0 alternate pin enable: 0=PWM0 on P2.5, 1=PWM0 on P1.5
  #define bUART1_PIN_X   0x20 // UART1 alternate pin enable: 0=RXD1/TXD1 on P2.6/P2.7, 1=RXD1/TXD1 on P1.6/P1.7
  #define bUART0_PIN_X   0x10 // UART0 alternate pin enable: 0=RXD0/TXD0 on P3.0/P3.1, 1=RXD0/TXD0 on P0.2/P0.3
  #define bIO_INT_ACT    0x08 // ReadOnly: GPIO interrupt request action status
  #define bINT0_PIN_X    0x04 // INT0 alternate pin enable: 0=INT0 on P3.2, 1=INT0 on P1.2
  #define bT2EX_PIN_X    0x02 // T2EX/CAP2 alternate pin enable: 0=T2EX/CAP2 on P1.1, 1=T2EX/CAP2 on P2.5
  #define bT2_PIN_X      0x01 // T2/CAP1 alternate pin enable: 0=T2/CAP1 on P1.0, 1=T2/CAP1 on P2.4

//EXTERN  UINT8XV PORT_CFG       _AT_ 0x21EA;   // port interrupt and wakeup and pulldown resistance config
//#define pPORT_CFG         PBYTE[0xEA]
  #define bP4_IE_LEVEL   0x80 // P4 level changing interrupt/wakeup enable
  #define bP2L_IE_LEVEL  0x40 // P2 low 4 bits level changing interrupt/wakeup enable
  #define bP1L_IE_LEVEL  0x20 // P1 low 4 bits level changing interrupt/wakeup enable
  #define bP0_IE_LEVEL   0x10 // P0 level changing interrupt/wakeup enable
  #define bP23_PDE       0x08 // P2.3 pulldown resistance enable
  #define bP22_PDE       0x04 // P2.2 pulldown resistance enable
  #define bP21_PDE       0x02 // P2.1 pulldown resistance enable
  #define bP20_PDE       0x01 // P2.0 pulldown resistance enable

//EXTERN  UINT8XV ANA_PIN        _AT_ 0x21E8;   // analog pin digital input control
//#define pANA_PIN          PBYTE[0xE8]
  #define bP70_P71_DI_DIS  0x80// control P7.0/P7.1 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN12_13_DI_DIS 0x40// control AIN12/AIN13 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN10_11_DI_DIS 0x20// control AIN10/AIN11 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN8_9_DI_DIS   0x10// control AIN8/AIN9 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN6_7_DI_DIS   0x08// control AIN6/AIN7 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN4_5_DI_DIS   0x04// control AIN4/AIN5 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN2_3_DI_DIS   0x02// control AIN2/AIN3 digital input: 0=enable digital input, 1=disable digital input
  #define bAIN0_1_DI_DIS   0x01// control AIN0/AIN1 digital input: 0=enable digital input, 1=disable digital input

/*  Timer0/1 Registers  */
SFR_(TCON,0x88);              // timer 0/1 control and external interrupt control
 SBIT_(TF1,0x88,7);           // timer1 overflow & interrupt flag, auto cleared when MCU enter interrupt routine
 SBIT_(TR1,0x88,6);           // timer1 run enable
 SBIT_(TF0,0x88,5);           // timer0 overflow & interrupt flag, auto cleared when MCU enter interrupt routine
 SBIT_(TR0,0x88,4);           // timer0 run enable
 SBIT_(IE1,0x88,3);           // INT1 interrupt flag, auto cleared when MCU enter interrupt routine
 SBIT_(IT1,0x88,2);           // INT1 interrupt type: 0=low level action, 1=falling edge action
 SBIT_(IE0,0x88,1);           // INT0 interrupt flag, auto cleared when MCU enter interrupt routine
 SBIT_(IT0,0x88,0);           // INT0 interrupt type: 0=low level action, 1=falling edge action
SFR_(TMOD,0x89);              // timer 0/1 mode
  #define bT1_GATE      0x80  // gate control of timer1: 0=timer1 run enable while TR1=1, 1=timer1 run enable while P3.3 (INT1) pin is high and TR1=1
  #define bT1_CT        0x40  // counter or timer mode selection for timer1: 0=timer, use internal clock, 1=counter, use P3.5 (T1) pin falling edge as clock
  #define bT1_M1        0x20  // timer1 mode high bit
  #define bT1_M0        0x10  // timer1 mode low bit
  #define MASK_T1_MOD   0x30  // bit mask of timer1 mode
  // bT1_M1 & bT1_M0: timer1 mode
  //   00: mode 0, 13-bit timer or counter by cascaded TH1 and lower 5 bits of TL1, the upper 3 bits of TL1 are ignored
  //   01: mode 1, 16-bit timer or counter by cascaded TH1 and TL1
  //   10: mode 2, TL1 operates as 8-bit timer or counter, and TH1 provide initial value for TL1 auto-reload
  //   11: mode 3, stop timer1
  #define bT0_GATE      0x08  // gate control of timer0: 0=timer0 run enable while TR0=1, 1=timer0 run enable while INT0 pin is high and TR0=1
  #define bT0_CT        0x04  // counter or timer mode selection for timer0: 0=timer, use internal clock, 1=counter, use P3.4 (T0) pin falling edge as clock
  #define bT0_M1        0x02  // timer0 mode high bit
  #define bT0_M0        0x01  // timer0 mode low bit
  #define MASK_T0_MOD   0x03  // bit mask of timer0 mode
  // bT0_M1 & bT0_M0: timer0 mode
  //   00: mode 0, 13-bit timer or counter by cascaded TH0 and lower 5 bits of TL0, the upper 3 bits of TL0 are ignored
  //   01: mode 1, 16-bit timer or counter by cascaded TH0 and TL0
  //   10: mode 2, TL0 operates as 8-bit timer or counter, and TH0 provide initial value for TL0 auto-reload
  //   11: mode 3, TL0 is 8-bit timer or counter controlled by standard timer0 bits, TH0 is 8-bit timer using TF1 and controlled by TR1, timer1 run enable if it is not mode 3

SFR_(L0 ,0x8A);         // low byte of timer 0 count
SFR_(L1 ,0x8B);         // low byte of timer 1 count
SFR_(H0 ,0x8C);         // high byte of timer 0 count
SFR_(H1 ,0x8D);         // high byte of timer 1 count

/*  UART0 Registers  */
SFR_(SCON, 0x98);             // UART0 control (serial port control)
  SBIT_(SM0, 0x98,7);         // UART0 mode bit0, selection data bit: 0=8 bits data, 1=9 bits data
  SBIT_(SM1, 0x98,6);         // UART0 mode bit1, selection baud rate: 0=fixed, 1=variable
  // SM0 & SM1: UART0 mode
  //    00 - mode 0, shift Register, baud rate fixed at: Fsys/12
  //    01 - mode 1, 8-bit UART,     baud rate = variable by timer1 or timer2 overflow rate
  //    10 - mode 2, 9-bit UART,     baud rate fixed at: Fsys/128@SMOD=0, Fsys/32@SMOD=1
  //    11 - mode 3, 9-bit UART,     baud rate = variable by timer1 or timer2 overflow rate
  SBIT_(SM2,0x98,5);          // enable multi-device communication in mode 2/3
  #define MASK_UART0_MOD 0xE0 // bit mask of UART0 mode
  SBIT_(REN,0x98,4);          // enable UART0 receiving
  SBIT_(TB8,0x98,3);          // the 9th transmitted data bit in mode 2/3
  SBIT_(RB8,0x98,2);          // 9th data bit received in mode 2/3, or stop bit received for mode 1
  SBIT_(TI ,0x98,1);          // transmit interrupt flag, set by hardware after completion of a serial transmittal, need software clear
  SBIT_(RI ,0x98,0);          // receive interrupt flag, set by hardware after completion of a serial receiving, need software clear
SFR_(SBUF,0x99);              // UART0 data buffer: reading for receiving, writing for transmittal

/*  Timer2/Capture Registers  */
SFR_(T2CON,0xC8);             // timer 2 control
  SBIT_(TF2   ,0xC8,7);       // timer2 overflow & interrupt flag, need software clear, the flag will not be set when either RCLK=1 or TCLK=1
  SBIT_(CAP1F ,0xC8,7);       // timer2 capture 1 interrupt flag, set by T2 edge trigger if bT2_CAP1_EN=1, need software clear
  SBIT_(EXF2  ,0xC8,6);       // timer2 external flag, set by T2EX edge trigger if EXEN2=1, need software clear
  SBIT_(RCLK  ,0xC8,5);       // selection UART0 receiving clock: 0=timer1 overflow pulse, 1=timer2 overflow pulse
  SBIT_(TCLK  ,0xC8,4);       // selection UART0 transmittal clock: 0=timer1 overflow pulse, 1=timer2 overflow pulse
  SBIT_(EXEN2 ,0xC8,3);       // enable T2EX trigger function: 0=ignore T2EX, 1=trigger reload or capture by T2EX edge
  SBIT_(TR2   ,0xC8,2);       // timer2 run enable
  SBIT_(C_T2  ,0xC8,1);       // timer2 clock source selection: 0=timer base internal clock, 1=external edge counter base T2 falling edge
  SBIT_(CP_RL2,0xC8,0);       // timer2 function selection (force 0 if RCLK=1 or TCLK=1): 0=timer and auto reload if count overflow or T2EX edge, 1=capture by T2EX edge
SFR_(T2MOD,0xC9);             // timer 2 mode and timer 0/1/2 clock mode
  #define bTMR_CLK      0x80  // fastest internal clock mode for timer 0/1/2 under faster clock mode: 0=use divided clock, 1=use original Fsys as clock without dividing
  #define bT2_CLK       0x40  // timer2 internal clock frequency selection: 0=standard clock, Fsys/12 for timer mode, Fsys/4 for UART0 clock mode,
                              //   1=faster clock, Fsys/4 @bTMR_CLK=0 or Fsys @bTMR_CLK=1 for timer mode, Fsys/2 @bTMR_CLK=0 or Fsys @bTMR_CLK=1 for UART0 clock mode
  #define bT1_CLK       0x20  // timer1 internal clock frequency selection: 0=standard clock, Fsys/12, 1=faster clock, Fsys/4 if bTMR_CLK=0 or Fsys if bTMR_CLK=1
  #define bT0_CLK       0x10  // timer0 internal clock frequency selection: 0=standard clock, Fsys/12, 1=faster clock, Fsys/4 if bTMR_CLK=0 or Fsys if bTMR_CLK=1
  #define bT2_CAP_M1    0x08  // timer2 capture mode high bit
  #define bT2_CAP_M0    0x04  // timer2 capture mode low bit
  #define MASK_CAP_MODE 0x0C  // bit mask of timer2 capture mode
  // bT2_CAP_M1 & bT2_CAP_M0: timer2 capture point selection
  //   x0: from falling edge to falling edge
  //   01: from any edge to any edge (level changing)
  //   11: from rising edge to rising edge
  #define T2OE          0x02  // enable timer2 generated clock output: 0=disable output, 1=enable clock output at T2 pin, frequency = TF2/2
  #define bT2_CAP1_EN   0x01  // enable T2 trigger function for capture 1 of timer2 if RCLK=0 & TCLK=0 & CP_RL2=1 & C_T2=0 & T2OE=0

SFR16_(RCAP2,0xCA);           // reload & capture value, little-endian
SFR16_(T2COUNT,0xCC);         // counter, little-endian
SFR16_(T2CAP1 ,0xCE);         // ReadOnly: capture 1 value for timer2

SFR_(RCAP2L ,0xCA);           // low byte of reload & capture value
SFR_(RCAP2H ,0xCB);           // high byte of reload & capture value
SFR_(TL2 ,0xCC);              // low byte of timer 2 count
SFR_(TH2 ,0xCD);              // high byte of timer 2 count
SFR_(T2CAP1L ,0xCE);          // ReadOnly: capture 1 value low byte for timer2
SFR_(T2CAP1H ,0xCF);          // ReadOnly: capture 1 value high byte for timer2

/*  PWMX Registers  */
SFR_(PWM_DATA2 ,0x9A);       // PWM data for PWM2 for 6/8 bits mode, low nibble is high 4 bits of PWM0 data for 12 bits mode
SFR_(PWM_DATA1 ,0x9B);       // PWM data for PWM1, low byte of PWM1 data for 12 bits mode
SFR_(PWM_DATA0 ,0x9C);       // PWM data for PWM0, low byte of PWM0 data for 12 bits mode
SFR_(PWM_CTRL  ,0x9D);       // PWM control
  #define bPWM_IE_END   0x80 // interrupt enable for cycle end
  #define bPWM1_POLAR   0x40 // PWM1 output polarity: 0=default low and high action, 1=default high and low action
  #define bPWM0_POLAR   0x20 // PWM0 output polarity: 0=default low and high action, 1=default high and low action
  #define bPWM_IF_END   0x10 // interrupt flag for cycle end, write 1 to clear or load new data into PWM_DATA0 to clear
  #define bPWM1_OUT_EN  0x08 // PWM1 output enable
  #define bPWM0_OUT_EN  0x04 // PWM0 output enable
  #define bPWM_CLR_ALL  0x02 // force clear FIFO and count of PWMX
  #define bPWM_MOD_6BIT 0x01 // PWM data 6 bits width mode: 0=8 bits data or 12 bits data, 1=6 bits data
SFR_(PWM_CK_SE ,0x9E);       // PWM clock divisor setting
SFR_(PWM_CTRL2 ,0x9F);       // PWM extend control
  #define bPWM_MOD_12BIT  0x80// PWM data 12 bits width mode: 0=8 bits data or 6 bits data, 1=12 bits data
  #define bPWM_STAG_STAT  0x40// ReadOnly: PWM stagger status: 0=inhibit PWM1/PWM3 if stagger enable, 1=inhibit PWM0/PWM2 if stagger enable
  #define bPWM2_3_STAG_EN 0x20// PWM2/PWM3 stagger output mode enable
  #define bPWM0_1_STAG_EN 0x10// PWM0/PWM1 stagger output mode enable
  #define bPWM5_OUT_EN    0x08// PWM5 output enable
  #define bPWM4_OUT_EN    0x04// PWM4 output enable
  #define bPWM3_OUT_EN    0x02// PWM3 output enable
  #define bPWM2_OUT_EN    0x01// PWM2 output enable
SFR_(PWM_DATA3 ,0xA3);       // PWM data for PWM3 for 6/8 bits mode, low nibble is high 4 bits of PWM1 data for 12 bits mode
SFR_(PWM_DATA4 ,0xA4);       // PWM data for PWM4 for 6/8 bits mode, low byte of cycle data for 12 bits mode
SFR_(PWM_DATA5 ,0xA5);       // PWM data for PWM5 for 6/8 bits mode, low nibble is high 4 bits of cycle data for 12 bits mode

/*  SPI0/Master0/Slave Registers  */
SFR_(SPI0_STAT ,0xF8);       // SPI0 status
  SBIT_(S0_FST_ACT  ,0xF8,7);// ReadOnly: indicate first byte received status for SPI0
  SBIT_(S0_IF_OV    ,0xF8,6);// interrupt flag for slave mode FIFO overflow, direct bit address clear or write 1 to clear
  SBIT_(S0_IF_FIRST ,0xF8,5);// interrupt flag for first byte received, direct bit address clear or write 1 to clear
  SBIT_(S0_IF_BYTE  ,0xF8,4);// interrupt flag for a byte data exchanged, direct bit address clear or write 1 to clear or accessing FIFO to clear if bS0_AUTO_IF=1
  SBIT_(S0_FREE     ,0xF8,3);// ReadOnly: SPI0 free status
  SBIT_(S0_T_FIFO   ,0xF8,2);// ReadOnly: tx FIFO count for SPI0
  SBIT_(S0_R_FIFO   ,0xF8,0);// ReadOnly: rx FIFO count for SPI0
SFR_(SPI0_DATA ,0xF9);       // FIFO data port: reading for receiving, writing for transmittal
SFR_(SPI0_CTRL ,0xFA);       // SPI0 control
  #define bS0_MISO_OE  0x80  // SPI0 MISO output enable
  #define bS0_MOSI_OE  0x40  // SPI0 MOSI output enable
  #define bS0_SCK_OE   0x20  // SPI0 SCK output enable
  #define bS0_DATA_DIR 0x10  // SPI0 data direction: 0=out(master_write), 1=in(master_read)
  #define bS0_MST_CLK  0x08  // SPI0 master clock mode: 0=mode 0 with default low, 1=mode 3 with default high
  #define bS0_2_WIRE   0x04  // enable SPI0 two wire mode: 0=3 wire (SCK+MOSI+MISO), 1=2 wire (SCK+MISO)
  #define bS0_CLR_ALL  0x02  // force clear FIFO and count of SPI0
  #define bS0_AUTO_IF  0x01  // enable FIFO accessing to auto clear S0_IF_BYTE interrupt flag
SFR_(SPI0_CK_SE ,0xFB);      // SPI0 clock divisor setting
SFR_(SPI0_S_PRE ,0xFB);      // preset value for SPI slave

SFR_(SPI0_SETUP ,0xFC);      // SPI0 setup
  #define bS0_MODE_SLV    0x80// SPI0 slave mode: 0=master, 1=slave
  #define bS0_IE_FIFO_OV  0x40// enable interrupt for slave mode FIFO overflow
  #define bS0_IE_FIRST    0x20// enable interrupt for first byte received for SPI0 slave mode
  #define bS0_IE_BYTE     0x10// enable interrupt for a byte received
  #define bS0_BIT_ORDER   0x08// SPI0 bit data order: 0=MSB first, 1=LSB first
  #define bS0_SLV_SELT    0x02// ReadOnly: SPI0 slave mode chip selected status: 0=unselected, 1=selected
  #define bS0_SLV_PRELOAD 0x01// ReadOnly: SPI0 slave mode data pre-loading status just after chip-selection

/*  UART1 Registers  */
SFR_(SCON1 ,0xBC);           // UART1 control (serial port control)
  #define bU1SM0       0x80  // UART1 mode, selection data bit: 0=8 bits data, 1=9 bits data
  #define bU1U0X2      0x40  // UART1/UART0 clock double frequency mode: 0=Fsys, 1=2*Fsys
  #define bU1SMOD      0x20  // UART1 2X baud rate selection: 0=slow(Fsys/32/(256-SBAUD1)), 1=fast(Fsys/16/(256-SBAUD1))
  #define bU1REN       0x10  // enable UART1 receiving
  #define bU1TB8       0x08  // the 9th transmitted data bit in 9 bits data mode
  #define bU1RB8       0x04  // 9th data bit received in 9 bits data mode, or stop bit received for 8 bits data mode
  #define bU1TIS       0x02  // WriteOnly: write 1 to preset transmit interrupt flag
  #define bU1RIS       0x01  // WriteOnly: write 1 to preset receive interrupt flag
SFR_(SBUF1  ,0xBD);         // UART1 data buffer: reading for receiving, writing for transmittal
SFR_(SBAUD1 ,0xBE);         // UART1 baud rate setting
SFR_(SIF1   ,0xBF);         // UART1 interrupt flag
  #define bU1TI        0x02 // transmit interrupt flag, set by hardware after completion of a serial transmittal, need software write 1 to clear
  #define bU1RI        0x01 // receive interrupt flag, set by hardware after completion of a serial receiving, need software write 1 to clear

/*  SPI1/Master1 Registers  */
SFR_(SPI1_STAT ,0xB4);      // SPI1 status
  #define bS1_IF_BYTE  0x10 // interrupt flag for a byte data exchanged, write 1 to clear or accessing FIFO to clear if bS1_AUTO_IF=1
  #define bS1_FREE     0x08 // ReadOnly: SPI1 free status
SFR_(SPI1_DATA ,0xB5);      // data port: reading for receiving, writing for transmittal
SFR_(SPI1_CTRL ,0xB6);      // SPI1 control
  #define bS1_MISO_OE  0x80 // SPI1 MISO output enable
  #define bS1_SCK_OE   0x20 // SPI1 SCK output enable, MOSI output enable if bS1_2_WIRE=0
  #define bS1_DATA_DIR 0x10 // SPI1 data direction: 0=out(master_write), 1=in(master_read)
  #define bS1_MST_CLK  0x08 // SPI1 master clock mode: 0=mode 0 with default low, 1=mode 3 with default high
  #define bS1_2_WIRE   0x04 // enable SPI1 two wire mode: 0=3 wire (SCK+MOSI+MISO), 1=2 wire (SCK+MISO)
  #define bS1_CLR_ALL  0x02 // force clear FIFO and count of SPI1
  #define bS1_AUTO_IF  0x01 // enable FIFO accessing to auto clear bS1_IF_BYTE interrupt flag
SFR_(SPI1_CK_SE ,0xB7);     // SPI1 clock divisor setting

/*  ADC and touch-key Registers  */
SFR_( ADC_CTRL ,0xF3);      // ADC/touch-key control and status
  #define bTKEY_ACT    0x80 // ReadOnly: indicate touch-key running status (charge then ADC)
  #define bADC_IF      0x20 // interrupt flag for ADC finished, write 1 to clear or write ADC_CHAN to clear or write TKEY_CTRL to clear
  #define bADC_START   0x10 // set 1 to start ADC, auto cleared when ADC finished
  #define bADC_EN      0x08 // control ADC power: 0=shut down ADC, 1=enable power for ADC
  #define bADC_CLK1    0x02 // ADC clock frequency selection high bit
  #define bADC_CLK0    0x01 // ADC clock frequency selection low bit
  #define MASK_ADC_CLK 0x03 // bit mask of ADC clock frequency selection
  // bADC_CLK1 & bADC_CLK0: ADC clock frequency selection
  //   00: slowest clock 750KHz, 512 Fosc cycles for each ADC
  //   01: slower clock 1.5MHz, 256 Fosc cycles for each ADC
  //   10: faster clock 3MHz, 128 Fosc cycles for each ADC
  //   11: fastest clock 6MHz, 64 Fosc cycles for each ADC

SFR16_(ADC_DAT ,0xF4);      // ReadOnly: ADC data

SFR_(ADC_DAT_L ,0xF4);      // ReadOnly: ADC data low byte
SFR_(ADC_DAT_H ,0xF5);      // ReadOnly: ADC data high byte
SFR_(TKEY_CTRL,0xF5);       // WriteOnly: touch-key charging pulse width control (only low 7 bits valid), auto cleared
SFR_(ADC_CHAN ,0xF6);       // analog signal channel seletion
  // ADC_CHAN[3:0]: ADC signal input channel selection if bADC_EN=1
  //   0000: connect AIN0(P1.0)
  //   0001: connect AIN1(P1.1)
  //   0010: connect AIN2(P1.2)
  //   0011: connect AIN3(P1.3)
  //   0100: connect AIN4(P1.4)
  //   0101: connect AIN5(P1.5)
  //   0110: connect AIN6(P1.6)
  //   0111: connect AIN7(P1.7)
  //   1000: connect AIN8(P0.0)
  //   1001: connect AIN9(P0.1)
  //   1010: connect AIN10(P0.2)
  //   1011: connect AIN11(P0.3)
  //   1100: connect AIN12(P0.4)
  //   1101: connect AIN13(P0.5)
  //   1110: connect V33
  //   1111: connect 1.8V reference voltage

/*  RGB LED Registers  */
SFR_(LED_COMMON ,0xA6);     // LED common drive pin selection
  // LED_COMMON[4:0]: LED common drive pin selection
  //   01110: act COM14(P7.0)
  //   01111: act COM15(P7.1)
  //   10000: act COM16(P0.0)
  //   10001: act COM17(P0.1)
  //   10010: act COM18(P0.2)
  //   10011: act COM19(P0.3)
  //   10100: act COM20(P0.4)
  //   10101: act COM21(P0.5)
  //   10110: act COM22(P0.6)
  //   10111: act COM23(P0.7)
  //   11000: act COM24(P3.0)
  //   11001: act COM25(P3.1)
  //   11010: act COM26(P3.2)
  //   11011: act COM27(P3.3)
  //   11100: act COM28(P3.4)
  //   11101: act COM29(P3.5)
  //   11110: act COM30(P3.6)
  //   11111: act COM31(P3.7)
  //   xxxxx: all inaction (default)
SFR_(LED_PWM_OE ,0xA7);      // LED RGB PWM output pin enable
  #define bLED_PWM7_OE  0x80 // LED PWM7 group output enable
  #define bLED_PWM6_OE  0x40 // LED PWM6 group output enable
  #define bLED_PWM5_OE  0x20 // LED PWM5 group output enable
  #define bLED_PWM4_OE  0x10 // LED PWM4 group output enable
  #define bLED_PWM3_OE  0x08 // LED PWM3 group output enable
  #define bLED_PWM2_OE  0x04 // LED PWM2 group output enable
  #define bLED_PWM1_OE  0x02 // LED PWM1 group output enable
  #define bLED_PWM0_OE  0x01 // LED PWM0 & global group output enable, auto clear at cycle end if bLED_PWM_INHIB=1

SFR16_(LED_DMA ,0xC6);       // LED buffer current address, little-endian

SFR_(LED_DMA_L ,0xC6);       // LED buffer current address low byte
SFR_(LED_DMA_H ,0xC7);       // LED buffer current address high byte
SFR_(LED_STATUS,0xF7);       // LED status
  #define bLED_IF        0x80// interrupt flag for LED inhibition, write 1 to clear or write LED_COMMON to clear, manual set by bLED_IF_SET
  #define bLED_IF_SET    0x40// WriteOnly: write 1 to force set bLED_IF
  #define bLED_INHIB     0x10// ReadOnly: LED inhibition status: 0=scanning, 1=inhibition for load new data and switch common
  #define MASK_LED_INTEN 0x0F// ReadOnly: bit mask of LED intenisy count high 4 bits

#define XSFR_LED_BASE  0x21D0// RGB PWM LED register base address

//EXTERN  UINT8XV LED_CTRL       _AT_ 0x21D1;   // LED control
//#define pLED_CTRL         PBYTE[0xD1]
  #define bLED_IE_INHIB  0x80// interrupt enable for LED inhibition
  #define bLED_BLUE_EN   0x40// blue color PWM group output enable
  #define bLED_GREEN_EN  0x20// green color PWM group output enable
  #define bLED_RED_EN    0x10// red color PWM group output enable
  #define bLED_COM_AHEAD 0x08// LED common output ahead mode: 0=normal, 1=ahead for MOS gate charging
  #define bLED_PWM_INHIB 0x04// LED PWM inhibition mode: 0=keep output, 1=auto inhibit at cycle end
  #define bLED_EN        0x01// LED enable

//EXTERN  UINT8XV LED_CYCLE      _AT_ 0x21D2;   // LED cycle config
//#define pLED_CYCLE        PBYTE[0xD2]
  #define bLED_COLOR_CYC    0x40// LED color PWM cycle: 0=256 intenisy PWM cycles, 1=128 intenisy PWM cycles
  #define bLED_INTEN_CYC1   0x20// LED intenisy PWM cycle selection high bit
  #define bLED_INTEN_CYC0   0x10// LED intenisy PWM cycle selection low bit
  #define MASK_LED_INT_CYC  0x30// bit mask of LED intenisy PWM cycle selection
  // bLED_INTEN_CYC1 & bLED_INTEN_CYC0: LED intenisy PWM cycle
  //   00: 256 LED clock cycles
  //   01: 128 LED clock cycles
  //   1x: 64 LED clock cycles
  #define bLED_CLK_FREQ1    0x02// LED clock frequency selection high bit
  #define bLED_CLK_FREQ0    0x01// LED clock frequency selection low bit
  #define MASK_LED_CLK_FREQ 0x03// bit mask of LED clock frequency selection
  // bLED_CLK_FREQ1 & bLED_CLK_FREQ0: LED clock frequency for intenisy PWM
  //   00: Fsys
  //   01: Fsys/2
  //   10: Fsys/3
  //   11: Fsys/4

//EXTERN  UINT8XV LED_FRAME      _AT_ 0x21D3;   // LED frame config
//#define pLED_FRAME        PBYTE[0xD3]
  #define bLED_INH_TMR2    0x40// LED inhibition timer selection high bit
  #define bLED_INH_TMR1    0x20// LED inhibition timer selection middle bit
  #define bLED_INH_TMR0    0x10// LED inhibition timer selection low bit
  #define MASK_LED_INH_TMR 0x70// bit mask of LED inhibition timer selection
  // bLED_INH_TMR2 & bLED_INH_TMR1 & bLED_INH_TMR0: LED inhibition timer (unit: intenisy PWM cycle)
  //   000~011: 1~4 intenisy PWM cycles
  //   100: 6 intenisy PWM cycles
  //   101: 8 intenisy PWM cycles
  //   110: 10 intenisy PWM cycles
  //   111: 12 intenisy PWM cycles
  #define bLED_PWM_REPT2    0x04// LED PWM repeat times selection high bit
  #define bLED_PWM_REPT1    0x02// LED PWM repeat times selection middle bit
  #define bLED_PWM_REPT0    0x01// LED PWM repeat times selection low bit
  #define MASK_LED_PWM_REPT 0x07// bit mask of LED PWM repeat times selection
  // bLED_PWM_REPT2 & bLED_PWM_REPT1 & bLED_PWM_REPT0: LED PWM repeat times
  //   000~111: same PWM data repeat 1~8 times

//EXTERN  UINT8XV LED_INT_ADJ    _AT_ 0x21D8;   // LED intensity adjustment
//EXTERN  UINT8XV LED_RED_ADJ    _AT_ 0x21D9;   // LED red color adjustment
//EXTERN  UINT8XV LED_GRE_ADJ    _AT_ 0x21DA;   // LED green color adjustment
//EXTERN  UINT8XV LED_BLU_ADJ    _AT_ 0x21DB;   // LED blue color adjustment
//#define pLED_INT_ADJ      PBYTE[0xD8]
//#define pLED_RED_ADJ      PBYTE[0xD9]
//#define pLED_GRE_ADJ      PBYTE[0xDA]
//#define pLED_BLU_ADJ      PBYTE[0xDB]

//EXTERN  UINT8XV LED_FRA_STA    _AT_ 0x21DC;   // ReadOnly: LED frame status
//#define pLED_FRA_STA      PBYTE[0xDC]
  #define MASK_LED_REPEAT 0x70// ReadOnly: bit mask of LED PWM repeat times
  #define MASK_LED_INHIB  0x0F// ReadOnly: bit mask of LED inhibition count

//EXTERN  UINT8XV LED_COL_CNT    _AT_ 0x21DD;   // ReadOnly: LED color PWM count
//#define pLED_COL_CNT      PBYTE[0xDD]

/*  I2C Registers  */
SFR_(I2CX_INT ,0xB3);         // I2C slave/master and PWM and LED interrupt request
  #define bI2CS_INT_ACT 0x20  // ReadOnly: I2C slave interrupt request: 0=free, 1=action
  #define bI2CM_INT_ACT 0x10  // ReadOnly: I2C master interrupt request: 0=free, 1=action
  #define bLED_INT_ACT  0x02  // ReadOnly: RGB LED interrupt request: 0=free, 1=action
  #define bPWMX_INT_ACT 0x01  // ReadOnly: PWMX interrupt request: 0=free, 1=action
SFR_(I2CS_INT_ST,0xBB);       // I2C slave interrupt status mapped from I2CS_STAT

/*  I2C master Registers  */
#define XSFR_I2CM_BASE    0x21C0    // I2C master register base address

//EXTERN  UINT8XV I2CM_CTRL      _AT_ 0x21C0;   // I2C master control
//#define pI2CM_CTRL        PBYTE[0xC0]
  #define bI2CM_IE          0x80 // interrupt enable for I2C master
  #define bI2CM_DEV_ACK     0x10 // ReadOnly: I2C device recent acknowledge status
  #define bI2CM_EN          0x08 // I2C master enable
  #define bI2CM_CMD1        0x02 // I2C master operation command high bit
  #define bI2CM_CMD0        0x01 // I2C master operation command low bit
  #define MASK_I2CM_CMD     0x03 // bit mask of I2C master operation command
  #define I2CM_CMD_STOP     0x09 // generate STOP condition
  #define I2CM_CMD_RX_ACK   0x0A // receive byte then generate ACK
  #define I2CM_CMD_RX_STOP  0x0B // receive byte then generate not-ACK and STOP condition
  // bI2CM_CMD1 & bI2CM_CMD0: I2C master operation command
  //   00: free
  //   01: generate STOP condition, auto clear after completion
  //   10: receive byte then generate ACK, auto clear after completion
  //   11: receive byte then generate not-ACK and STOP condition, auto clear after completion

//EXTERN  UINT8XV I2CM_CK_SE     _AT_ 0x21C1;   // I2C master clock divisor setting
//EXTERN  UINT8XV I2CM_START     _AT_ 0x21C2;   // I2C master start command, write byte to generate START condition then transmit first byte and receive device acknowledge
//EXTERN  UINT8XV I2CM_DATA      _AT_ 0x21C3;   // I2C master data buffer, write byte to transmit byte then receive device acknowledge, return received data if read
//#define pI2CM_CK_SE       PBYTE[0xC1]
//#define pI2CM_START       PBYTE[0xC2]
//#define pI2CM_DATA        PBYTE[0xC3]

//EXTERN  UINT8XV I2CM_STAT      _AT_ 0x21C4;   // I2C master status
//#define pI2CM_STAT        PBYTE[0xC4]
  #define bI2CM_IF          0x80      // interrupt flag for I2C master, write 1 to clear or write I2CM_CTRL/I2CM_START/I2CM_DATA to clear
  #define MASK_I2CM_STAT    0x70      // ReadOnly: bit mask of I2C master status machine
  // MASK_I2CM_STAT: I2C master status machine
  //   000: free
  //   001~011: START/STOP condition step 1/2/3
  //   100~111: byte transfer and acknowledge step 1/2/3/4
  #define MASK_I2CM_CNT     0x0F      // ReadOnly: bit count of I2C master

#define XSFR_I2CS_BASE    0x2230    // I2CS register base address

//EXTERN  UINT8XV I2CS_CTRL      _AT_ 0x2232;   // I2CS control
//#define pI2CS_CTRL        PBYTE[0x32]
  #define bI2CS_IE_RECV   0x80   // byte received interrupt enable for I2C slave
  #define bI2CS_IE_TRAN   0x40   // byte transmitted interrupt enable for I2C slave
  #define bI2CS_IE_ADDR   0x20   // data address received interrupt enable for I2C slave
  #define bI2CS_IE_DEV_A  0x10   // device address received interrupt enable for I2C slave, enable general address
  #define bI2CS_IE_STASTO 0x08   // START/STOP condition received interrupt enable for I2C slave
  #define bI2CS_SDA_IN    0x04   // ReadOnly: current SDA status after synchronization
  #define bI2CS_DMA_EN    0x02   // DMA enable for I2C slave
  #define bI2CS_EN        0x01   // I2C slave enable

//EXTERN  UINT8XV I2CS_DEV_A     _AT_ 0x2233;   // I2CS device address
//#define pI2CS_DEV_A       PBYTE[0x33]
  #define MASK_I2CS_DEV_A   0xFE // I2C slave device address: 00=general address, other=device address to match
  #define bI2CS_DA_4BIT     0x01 // I2C slave device address mode: 0=7 bits address, 1=4 bits address (ignore low 3 bits)

//EXTERN  UINT8XV I2CS_ADDR      _AT_ 0x2235;   // ReadOnly: I2CS data address
//EXTERN  UINT8XV I2CS_DATA      _AT_ 0x2236;   // I2CS data buffer
//#define pI2CS_ADDR        PBYTE[0x35]
//#define pI2CS_DATA        PBYTE[0x36]

//EXTERN  UINT8XV I2CS_STAT      _AT_ 0x223A;   // I2CS status
//#define pI2CS_STAT        PBYTE[0x3A]
  #define bI2CS_IF_STASTO   0x80 // START/STOP condition received interrupt flag for I2C slave, write 1 to clear
  #define bI2CS_IF_BYTE     0x40 // byte transferred interrupt flag for I2C slave, write 1 to clear
  #define bI2CS_IF_ADDR     0x20 // data address received interrupt flag for I2C slave, write 1 to clear
  #define bI2CS_IF_DEV_A    0x10 // device address received interrupt flag for I2C slave, write 1 to clear
  #define MASK_I2CS_STAT    0x0F // ReadOnly: bit mask of I2C slave status machine
  // MASK_I2CS_STAT: I2C slave status machine
  //   0000: free or receiving device address
  //   0001: acknowledging device address received
  //   0010: receiving data address
  //   0011: acknowledging data address received
  //   0100: receiving data byte
  //   0101: acknowledging data byte received
  //   0110: transmitting data byte
  //   0111: checking acknowledge
  //   1100: STOP condition, used to judge START/STOP if bI2CS_IF_STASTO=1
  //   xxxx: error or unknown

//EXTERN  UINT8XV I2CS_DMA_H     _AT_ 0x2138;   // I2CS buffer start address high byte, big-endian
//EXTERN  UINT8XV I2CS_DMA_L     _AT_ 0x2139;   // I2CS buffer start address low byte, big-endian
  #define MASK_I2CS_AH      0xE0 // ReadOnly: high 3 bits of data address @I2CS_DMA_H if bI2CS_DA_4BIT=1

/*  USB Host/Device Registers  */
SFR_(UDEV_CTRL,0xD1);            // USB device physical port control
  #define bUD_PD_EN         0x08 // USB DP/DM pulldown resistance enable: 0=disable pulldown, 1=enable
  #define bUD_LOW_SPEED     0x04 // enable USB physical port low speed: 0=full speed, 1=low speed
  #define bUD_GP_BIT        0x02 // general purpose bit
  #define bUD_PORT_EN       0x01 // enable USB physical port I/O: 0=disable, 1=enable
SFR_(UHOST_CTRL ,0xD1);          // USB host physical port control
SFR_(UHUB01_CTRL,0xD1);           // USB host root hub0 & hub1 physical port control     
  #define bUH1_PD_EN        0x80 // USB hub1 HP1/HM1 pulldown resistance enable: 0=disable pulldown, 1=enable
  #define bUH1_LOW_SPEED    0x40 // enable USB hub1 port low speed: 0=full speed, 1=low speed
  #define bUH1_BUS_RESET    0x20 // control USB hub1 bus reset: 0=normal, 1=force bus reset
  #define bUH1_PORT_EN      0x10 // enable USB hub1 port: 0=disable, 1=enable port, automatic disabled if USB device detached
  #define bUH0_PD_EN        0x08 // USB hub0 HP0/DP/HM0/DM pulldown resistance enable: 0=disable pulldown, 1=enable
  #define bUH0_LOW_SPEED    0x04 // enable USB hub0 port low speed: 0=full speed, 1=low speed
  #define bUH0_BUS_RESET    0x02 // control USB hub0 bus reset: 0=normal, 1=force bus reset
  #define bUH0_PORT_EN      0x01 // enable USB hub0 port: 0=disable, 1=enable port, automatic disabled if USB device detached
  #define bUH_PD_EN         0x08 // USB hub0 HP0/DP/HM0/DM pulldown resistance enable: 0=disable pulldown, 1=enable
  #define bUH_LOW_SPEED     0x04 // enable USB hub0 port low speed: 0=full speed, 1=low speed
  #define bUH_BUS_RESET     0x02 // control USB hub0 bus reset: 0=normal, 1=force bus reset
  #define bUH_PORT_EN       0x01 // enable USB hub0 port: 0=disable, 1=enable port, automatic disabled if USB device detached

SFR_(UHUB23_CTRL,0xE1);          // USB host root hub2 & hub3 physical port control
  #define bUH3_PD_EN        0x80 // USB hub3 HP3/HM3 pulldown resistance enable: 0=disable pulldown, 1=enable
  #define bUH3_LOW_SPEED    0x40 // enable USB hub3 port low speed: 0=full speed, 1=low speed
  #define bUH3_BUS_RESET    0x20 // control USB hub3 bus reset: 0=normal, 1=force bus reset
  #define bUH3_PORT_EN      0x10 // enable USB hub3 port: 0=disable, 1=enable port, automatic disabled if USB device detached
  #define bUH2_PD_EN        0x08 // USB hub2 HP2/HM2 pulldown resistance enable: 0=disable pulldown, 1=enable
  #define bUH2_LOW_SPEED    0x04 // enable USB hub2 port low speed: 0=full speed, 1=low speed
  #define bUH2_BUS_RESET    0x02 // control USB hub2 bus reset: 0=normal, 1=force bus reset
  #define bUH2_PORT_EN      0x01 // enable USB hub2 port: 0=disable, 1=enable port, automatic disabled if USB device detached

SFR_(UEP1_CTRL ,0xD2);           // endpoint 1 control
  #define bUEP_R_TOG        0x80 // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
  #define bUEP_T_TOG        0x40 // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
  #define bUEP_AUTO_TOG     0x10 // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
  #define bUEP_R_RES1       0x08 // handshake response type high bit for USB endpoint X receiving (OUT)
  #define bUEP_R_RES0       0x04 // handshake response type low bit for USB endpoint X receiving (OUT)
  #define MASK_UEP_R_RES    0x0C // bit mask of handshake response type for USB endpoint X receiving (OUT)
  #define UEP_R_RES_ACK     0x00
  #define UEP_R_RES_TOUT    0x04
  #define UEP_R_RES_NAK     0x08
  #define UEP_R_RES_STALL   0x0C
  // bUEP_R_RES1 & bUEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
  //   00: ACK (ready)
  //   01: no response, time out to host, for non-zero endpoint isochronous transactions
  //   10: NAK (busy)
  //   11: STALL (error)
  #define bUEP_T_RES1       0x02 // handshake response type high bit for USB endpoint X transmittal (IN)
  #define bUEP_T_RES0       0x01 // handshake response type low bit for USB endpoint X transmittal (IN)
  #define MASK_UEP_T_RES    0x03 // bit mask of handshake response type for USB endpoint X transmittal (IN)
  #define UEP_T_RES_ACK     0x00
  #define UEP_T_RES_TOUT    0x01
  #define UEP_T_RES_NAK     0x02
  #define UEP_T_RES_STALL   0x03
  // bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
  //   00: DATA0 or DATA1 then expecting ACK (ready)
  //   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
  //   10: NAK (busy)
  //   11: STALL (error)
SFR_(UEP1_T_LEN ,0xD3);          // endpoint 1 transmittal length
SFR_(UEP2_CTRL  ,0xD4);          // endpoint 2 control
SFR_(UEP2_T_LEN ,0xD5);          // endpoint 2 transmittal length
SFR_(UEP3_CTRL  ,0xD6);          // endpoint 3 control
SFR_(UEP3_T_LEN ,0xD7);          // endpoint 3 transmittal length

SFR_(USB_INT_FG ,0xD8);          // USB interrupt flag
  SBIT_(U_IS_NAK    ,0xD8,7);    // ReadOnly: indicate current USB transfer is NAK received
  SBIT_(U_TOG_OK    ,0xD8,6);    // ReadOnly: indicate current USB transfer toggle is OK
  SBIT_(UIF_FIFO_OV ,0xD8,4);    // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
  SBIT_(UIF_HST_SOF ,0xD8,3);    // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
  SBIT_(UIF_SUSPEND ,0xD8,2);    // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
  SBIT_(UIF_TRANSFER,0xD8,1);    // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
  SBIT_(UIF_DETECT  ,0xD8,0);    // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
  SBIT_(UIF_BUS_RST ,0xD8,0);    // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear

SFR_(USB_INT_ST ,0xD9);          // ReadOnly: USB interrupt status
  #define bUIS_SETUP_ACT    0x80 // ReadOnly: indicate SETUP token & 8 bytes setup request received for USB device mode
  #define bUIS_TOG_OK       0x40 // ReadOnly: indicate current USB transfer toggle is OK, keep last status during SETUP token
  #define bUIS_TOKEN1       0x20 // ReadOnly: current token PID code high bit received for USB device mode, clear UIF_TRANSFER to set free
  #define bUIS_TOKEN0       0x10 // ReadOnly: current token PID code low bit received for USB device mode, clear UIF_TRANSFER to set free
  #define MASK_UIS_TOKEN    0x30 // ReadOnly: bit mask of current token PID code received for USB device mode
  #define UIS_TOKEN_OUT     0x00
  #define UIS_TOKEN_SOF     0x10
  #define UIS_TOKEN_IN      0x20
  #define UIS_TOKEN_FREE    0x30
  // bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode, keep last status during SETUP token, clear UIF_TRANSFER ( UIF_TRANSFER from 1 to 0 ) to set free
  //   00: OUT token PID received
  //   01: SOF token PID received
  //   10: IN token PID received
  //   11: free
  #define MASK_UIS_ENDP     0x0F // ReadOnly: bit mask of current transfer endpoint number for USB device mode, keep last status during SETUP token
  #define MASK_UIS_H_RES    0x0F // ReadOnly: bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received
SFR_(USB_MIS_ST,0xDA);           // ReadOnly: USB miscellaneous status
  #define bUMS_SOF_PRES     0x80 // ReadOnly: indicate host SOF timer presage status
  #define bUMS_SOF_ACT      0x40 // ReadOnly: indicate host SOF timer action status for USB host
  #define bUMS_SIE_FREE     0x20 // ReadOnly: indicate USB SIE free status
  #define bUMS_R_FIFO_RDY   0x10 // ReadOnly: indicate USB receiving FIFO ready status (not empty)
  #define bUMS_BUS_RESET    0x08 // ReadOnly: indicate USB bus reset status
  #define bUMS_SUSPEND      0x04 // ReadOnly: indicate USB suspend status
  #define bUMS_DM_LEVEL     0x02 // ReadOnly: indicate HM0/DM level saved at device attached to USB host/hub0
  #define bUMS_DEV_ATTACH   0x01 // ReadOnly: indicate device attached status on USB host/hub0
SFR_(USB_RX_LEN ,0xDB);          // ReadOnly: USB receiving length, keep last data during SETUP token
SFR_(UEP0_CTRL  ,0xDC);          // endpoint 0 control
SFR_(UEP0_T_LEN ,0xDD);          // endpoint 0 transmittal length
SFR_(UEP4_CTRL  ,0xDE);          // endpoint 4 control
SFR_(UEP4_T_LEN ,0xDF);          // endpoint 4 transmittal length
SFR_(USB_CTRL   ,0xE2);          // USB base control
  #define bUC_HOST_MODE     0x80 // enable USB host mode: 0=device mode, 1=host mode
  #define bUC_LOW_SPEED     0x40 // enable USB low speed: 0=full speed, 1=low speed
  #define bUC_DEV_PU_EN     0x20 // USB device enable and internal pullup resistance (1K5) enable
  #define bUC_DEV_EN        0x10 // USB device enable only
  #define bUC_SYS_CTRL1     0x20 // USB system control high bit
  #define bUC_SYS_CTRL0     0x10 // USB system control low bit
  #define MASK_UC_SYS_CTRL  0x30 // bit mask of USB system control
  // bUC_HOST_MODE & bUC_SYS_CTRL1 & bUC_SYS_CTRL0: USB system control
  //   0 00: disable USB device and disable internal pullup resistance
  //   0 01: enable USB device and disable internal pullup resistance, need external pullup resistance
  //   0 1x: enable USB device and enable internal pullup resistance
  //   1 00: enable USB host and normal status
  //   1 01: enable USB host and force DP/DM output SE0 state
  //   1 10: enable USB host and force DP/DM output J state
  //   1 11: enable USB host and force DP/DM output resume or K state
  #define bUC_INT_BUSY      0x08 // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
  #define bUC_RESET_SIE     0x04 // force reset USB SIE, need software clear
  #define bUC_CLR_ALL       0x02 // force clear FIFO and count of USB
SFR_(USB_DEV_AD ,0xE3);          // USB device address, lower 7 bits for USB device address
  #define bUDA_GP_BIT       0x80 // general purpose bit
  #define MASK_USB_ADDR     0x7F // bit mask for USB device address

SFR16_(UEP2_DMA ,0xE4);          // endpoint 2 buffer start address, little-endian
SFR16_(UEP3_DMA ,0xE6);          // endpoint 3 buffer start address, little-endian
SFR16_(UEP1_DMA ,0xEE);          // endpoint 1 buffer start address, little-endian
SFR16_(UEP0_DMA ,0xEC);          // endpoint 0 buffer start address, little-endian

SFR_(UEP2_DMA_L ,0xE4);          // endpoint 2 buffer start address low byte
SFR_(UEP2_DMA_H ,0xE5);          // endpoint 2 buffer start address high byte
SFR_(UEP3_DMA_L ,0xE6);          // endpoint 3 buffer start address low byte
SFR_(UEP3_DMA_H ,0xE7);          // endpoint 3 buffer start address high byte
SFR_(USB_HUB_ST ,0xEB);          // ReadOnly: USB host hub status
  #define bUHS_HM3_LEVEL    0x80 // ReadOnly: indicate HM3 level saved at device attached to USB hub3
  #define bUHS_HM2_LEVEL    0x40 // ReadOnly: indicate HM2 level saved at device attached to USB hub2
  #define bUHS_HM1_LEVEL    0x20 // ReadOnly: indicate HM1 level saved at device attached to USB hub1
  #define bUHS_HM0_LEVEL    0x10 // ReadOnly: indicate HM0/DM level saved at device attached to USB hub0
  #define bUHS_H3_ATTACH    0x08 // ReadOnly: indicate device attached status on USB hub3
  #define bUHS_H2_ATTACH    0x04 // ReadOnly: indicate device attached status on USB hub2
  #define bUHS_H1_ATTACH    0x02 // ReadOnly: indicate device attached status on USB hub1
  #define bUHS_H0_ATTACH    0x01 // ReadOnly: indicate device attached status on USB hub0

SFR_(UEP0_DMA_L ,0xEC);          // endpoint 0 buffer start address low byte
SFR_(UEP0_DMA_H ,0xED);          // endpoint 0 buffer start address high byte
SFR_(UEP1_DMA_L ,0xEE);          // endpoint 1 buffer start address low byte
SFR_(UEP1_DMA_H ,0xEF);          // endpoint 1 buffer start address high byte
SFR_(UH_SETUP   ,0xD2);          // host aux setup
  #define bUH_PRE_PID_EN   0x80  // USB host PRE PID enable for low speed device via hub
  #define bUH_SOF_EN       0x40  // USB host automatic SOF enable
SFR_(UH_RX_CTRL,0xD4);           // host receiver endpoint control
  #define bUH_R_TOG        0x80  // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
  #define bUH_R_AUTO_TOG   0x10  // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
  #define bUH_R_RES        0x04  // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions
SFR_(UH_EP_PID ,0xD5);           // host endpoint and token PID, lower 4 bits for endpoint number, upper 4 bits for token PID
  #define MASK_UH_TOKEN     0xF0 // bit mask of token PID for USB host transfer
  #define MASK_UH_ENDP      0x0F // bit mask of endpoint number for USB host transfer
SFR_(UH_TX_CTRL ,0xD6);          // host transmittal endpoint control
  #define bUH_T_TOG         0x40 // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
  #define bUH_T_AUTO_TOG    0x10 // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
  #define bUH_T_RES         0x01 // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions
SFR_(UH_TX_LEN   ,0xD7);         // host transmittal endpoint transmittal length

SFR16_(UH_RX_DMA ,0xE4);         // host rx endpoint buffer start address, little-endian
SFR16_(UH_TX_DMA ,0xE6);         // host tx endpoint buffer start address, little-endian

SFR_(UH_RX_DMA_L ,0xE4);         // host rx endpoint buffer start address low byte
SFR_(UH_RX_DMA_H ,0xE5);         // host rx endpoint buffer start address high byte
SFR_(UH_TX_DMA_L ,0xE6);         // host tx endpoint buffer start address low byte
SFR_(UH_TX_DMA_H ,0xE7);         // host tx endpoint buffer start address high byte

#define XSFR_USB_BASE 0x21E0    // USB register base address

//EXTERN  UINT8XV UEP4_1_MOD     _AT_ 0x21E0;   // endpoint 4/1 mode
//#define pUEP4_1_MOD       PBYTE[0xE0]
  #define bUEP1_RX_EN       0x80 // enable USB endpoint 1 receiving (OUT)
  #define bUEP1_TX_EN       0x40 // enable USB endpoint 1 transmittal (IN)
  #define bUEP1_BUF_MOD     0x10 // buffer mode of USB endpoint 1
  // bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
  //   0 0 x:  disable endpoint and disable buffer
  //   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
  //   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
  //   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
  //   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
  //   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
  //   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
  #define bUEP4_RX_EN       0x08 // enable USB endpoint 4 receiving (OUT)
  #define bUEP4_TX_EN       0x04 // enable USB endpoint 4 transmittal (IN)
  // bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
  //   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
  //   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
  //   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
  //   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
  //           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes

//EXTERN  UINT8XV UEP2_3_MOD     _AT_ 0x21E1;   // endpoint 2/3 mode
//#define pUEP2_3_MOD       PBYTE[0xE1]
  #define bUEP3_RX_EN       0x80      // enable USB endpoint 3 receiving (OUT)
  #define bUEP3_TX_EN       0x40      // enable USB endpoint 3 transmittal (IN)
  #define bUEP3_BUF_MOD     0x10      // buffer mode of USB endpoint 3
  #define bUEP2_RX_EN       0x08      // enable USB endpoint 2 receiving (OUT)
  #define bUEP2_TX_EN       0x04      // enable USB endpoint 2 transmittal (IN)
  #define bUEP2_BUF_MOD     0x01      // buffer mode of USB endpoint 2
  
//EXTERN  UINT8XV UH_EP_MOD      _AT_ 0x21E1;   // host endpoint mode
  #define UH_EP_MOD         UEP2_3_MOD
  //#define pUH_EP_MOD       pUEP2_3_MOD
  #define bUH_EP_TX_EN      0x40      // enable USB host OUT endpoint transmittal
  #define bUH_EP_TBUF_MOD   0x10      // buffer mode of USB host OUT endpoint
  // bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
  //   0 x:  disable endpoint and disable buffer
  //   1 0:  64 bytes buffer for transmittal (OUT endpoint)
  //   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
  #define bUH_EP_RX_EN      0x08      // enable USB host IN endpoint receiving
  #define bUH_EP_RBUF_MOD   0x01      // buffer mode of USB host IN endpoint
  // bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
  //   0 x:  disable endpoint and disable buffer
  //   1 0:  64 bytes buffer for receiving (IN endpoint)
  //   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes
  
//EXTERN  UINT8XV USB_INT_EN     _AT_ 0x21E2;   // USB interrupt enable
//#define pUSB_INT_EN       PBYTE[0xE2]
  #define bUIE_DEV_SOF      0x80      // enable interrupt for SOF received for USB device mode
  #define bUIE_DEV_NAK      0x40      // enable interrupt for NAK responded for USB device mode
  #define bUIE_FIFO_OV      0x10      // enable interrupt for FIFO overflow
  #define bUIE_HST_SOF      0x08      // enable interrupt for host SOF timer action for USB host mode
  #define bUIE_SUSPEND      0x04      // enable interrupt for USB suspend or resume event
  #define bUIE_TRANSFER     0x02      // enable interrupt for USB transfer completion
  #define bUIE_DETECT       0x01      // enable interrupt for USB device detected event for USB host mode
  #define bUIE_BUS_RST      0x01      // enable interrupt for USB bus reset event for USB device mode


#define I2C_MASTER_XSFR_BASE 0x21C0
#define LED_XSFR_BASE        0x21D0
#define USB_XSFR_BASE        0x21E0 
#define IO_XSFR_BASE         0x21E8
#define I2C_SLAVE_XSFR_BASE  0x2230
 
typedef struct _I2CMASTER
{
   unsigned charI2CM_CTRL  ;// XBYTE[0x21C0] // I2C master control
   unsigned charI2CM_CK_SE ;// XBYTE[0x21C1] // I2C master clock divisor setting
   unsigned charI2CM_START ;// XBYTE[0x21C2] // I2C master start command, write byte to generate START condition then transmit first byte and receive device acknowledge
   unsigned charI2CM_DATA  ;// XBYTE[0x21C3] // I2C master data buffer, write byte to transmit byte then receive device acknowledge, return received data if read
   unsigned charI2CM_STAT  ;// XBYTE[0x21C4] // I2C master status
   unsigned char res[11];
}I2CMASTER;

typedef struct _XLED
{
   unsigned char res0[2]    ;
   unsigned char LED_CYCLE  ; // XBYTE[0x21D2] // LED cycle config
   unsigned char LED_FRAME  ; // XBYTE[0x21D3] // LED frame config
   unsigned char res1[4]    ; 
   unsigned char LED_INT_ADJ; // XBYTE[0x21D8] // LED intensity adjustment
   unsigned char LED_RED_ADJ; // XBYTE[0x21D9] // LED red color adjustment
   unsigned char LED_GRE_ADJ; // XBYTE[0x21DA] // LED green color adjustment
   unsigned char LED_BLU_ADJ; // XBYTE[0x21DB] // LED blue color adjustment
   unsigned char LED_FRA_STA; // XBYTE[0x21DC] // ReadOnly: LED frame status
   unsigned char LED_COL_CNT; // XBYTE[0x21DD] // ReadOnly: LED color PWM count
   unsigned char res2[2];
}XLED;

typedef struct _USBDEVICE
{
   unsigned char UEP4_1_MOD;
   unsigned char UEP2_3_MOD;
   unsigned char USB_INT_EN;
   unsigned char res0[5];
}USBDEVICE;

typedef struct _USBHOST
{
   unsigned char UEP4_1_MOD;
   unsigned char UH_EP_MOD;
   unsigned char USB_INT_EN;
   unsigned char res0[5];
}USBHOST;

typedef struct _CHIPIO
{
   unsigned char ANA_PIN;  //XBYTE[0x21E8] // analog pin digital input control
   unsigned char PIN_FUNC; //XBYTE[0x21E9] // pin function selection
   unsigned char PORT_CFG; //XBYTE[0x21EA] // port interrupt and wakeup and pulldown resistance config
   unsigned char CMP_DCDC; //XBYTE[0x21EB] // comparator and DC-DC control
   unsigned char res0[4];
}CHIPIO;

typedef struct _I2CSLAVE
{
   unsigned char res0[2];
   unsigned char I2CS_CTRL  ;// XBYTE[0x2232] // I2CS control
   unsigned char I2CS_DEV_A ;// XBYTE[0x2233] // I2CS device address
   unsigned char res1;
   unsigned char I2CS_ADDR  ;// XBYTE[0x2235] // ReadOnly: I2CS data address
   unsigned char I2CS_DATA  ;// XBYTE[0x2236] // I2CS data buffer
   unsigned char res2;
   unsigned char I2CS_DMA_H ;// XBYTE[0x2138] // I2CS buffer start address high byte, big-endian
   unsigned char I2CS_DMA_L ;// XBYTE[0x2139] // I2CS buffer start address low byte, big-endian   
   unsigned char I2CS_STAT  ;// XBYTE[0x223A] // I2CS status
   unsigned char res3[5];
}I2CSLAVE;

typedef struct _CH557XSFR
{
   I2CMASTER     MASTER; //mastermode @ 0x21C0
   XLED          LED;    //leds       @ 0x21D0
   union                 //usb        @ 0x21E0  
   {
      USBDEVICE  DEVICE; 
      USBHOST    HOST;   
   }USB;
   CHIPIO        IO;      //io        @ 0x21E8
   unsigned char reserved[0x40];
   I2CSLAVE      SLAVE;   //slavemode @ 0x2230
}XSFR;

/*----- XDATA: xRAM ------------------------------------------*/

#define XDATA_RAM_SIZE    0x2000    // size of expanded xRAM, xdata SRAM embedded chip

/*----- Reference Information --------------------------------------------*/
#define ID_CH557          0x57      // chip ID
#define ID_CH556          0x56      // chip ID

/* Interrupt routine address and interrupt number */
#define INT_ADDR_INT0     0x0003    // interrupt vector address for INT0
#define INT_ADDR_TMR0     0x000B    // interrupt vector address for timer0
#define INT_ADDR_INT1     0x0013    // interrupt vector address for INT1
#define INT_ADDR_TMR1     0x001B    // interrupt vector address for timer1
#define INT_ADDR_UART0    0x0023    // interrupt vector address for UART0
#define INT_ADDR_TMR2     0x002B    // interrupt vector address for timer2
#define INT_ADDR_SPI0     0x0033    // interrupt vector address for SPI0
#define INT_ADDR_USB      0x0043    // interrupt vector address for USB
#define INT_ADDR_ADC      0x004B    // interrupt vector address for ADC
#define INT_ADDR_UART1    0x0053    // interrupt vector address for UART1
#define INT_ADDR_PWM_I2C  0x005B    // interrupt vector address for PWMX/LED/I2C
#define INT_ADDR_GPIO     0x0063    // interrupt vector address for GPIO
#define INT_ADDR_WDOG     0x006B    // interrupt vector address for watch-dog timer
#define INT_NO_INT0       0         // interrupt number for INT0
#define INT_NO_TMR0       1         // interrupt number for timer0
#define INT_NO_INT1       2         // interrupt number for INT1
#define INT_NO_TMR1       3         // interrupt number for timer1
#define INT_NO_UART0      4         // interrupt number for UART0
#define INT_NO_TMR2       5         // interrupt number for timer2
#define INT_NO_SPI0       6         // interrupt number for SPI0
#define INT_NO_USB        8         // interrupt number for USB
#define INT_NO_ADC        9         // interrupt number for ADC
#define INT_NO_UART1      10        // interrupt number for UART1
#define INT_NO_PWM_I2C    11        // interrupt number for PWMX/LED/I2C
#define INT_NO_GPIO       12        // interrupt number for GPIO
#define INT_NO_WDOG       13        // interrupt number for watch-dog timer

/* Special Program Space */
#define DATA_FLASH_ADDR   0xF000    // start address of Data-Flash
#define BOOT_LOAD_ADDR    0xF400    // start address of boot loader program
#define ROM_CFG_ADDR      0xFFFE    // chip configuration information address
#define ROM_CHIP_ID_LO    0x10      // chip ID number low dword
#define ROM_CHIP_ID_HI    0x14      // chip ID number high dword

#define CH557_H_
#endif  // CH557_H_

