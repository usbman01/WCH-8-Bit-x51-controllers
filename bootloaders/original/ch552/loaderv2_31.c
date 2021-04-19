// Bootloader original v2.31 blob
// Start: 0x3800
// End  : 0x3FEF
// usable for CH551, CH552,CH554,(CH553)
const unsigned char code LoaderImage[]=
{
   0x02,0x3F,0xC7,0x00,0xE5,0x75,0x64,0x5A,0x60,0x02,0x80,0xFE,0xE4,0x90,0x00,0x51,
   0xF0,0x75,0x2B,0x06,0x79,0xFE,0xE5,0x2D,0x24,0x5F,0xB4,0x0B,0x00,0x40,0x03,0x02,
   0x3B,0x6E,0x90,0x38,0x30,0x75,0xF0,0x03,0xA4,0xC5,0x83,0x25,0xF0,0xC5,0x83,0x73,
   0x02,0x38,0x51,0x02,0x3B,0x61,0x02,0x39,0xE0,0x02,0x39,0x90,0x02,0x38,0x75,0x02,
   0x39,0x0F,0x02,0x3A,0xA5,0x02,0x3A,0x60,0x02,0x39,0xCC,0x02,0x38,0x7B,0x02,0x39,
   0x5E,0xA9,0xA1,0xE4,0xFB,0xEB,0x90,0x3E,0x65,0x93,0xFF,0x74,0x32,0x2B,0xF8,0xE6,
   0x6F,0x60,0x05,0x79,0xF1,0x02,0x3B,0x6E,0x0B,0xBB,0x10,0xE9,0x90,0x00,0x51,0x74,
   0x11,0xF0,0x02,0x3B,0x6E,0x30,0x00,0x03,0x02,0x3B,0x87,0xE5,0x2E,0x24,0xFB,0xF5,
   0x0A,0xE5,0x31,0xFE,0xE5,0x30,0xFD,0xEE,0xF5,0x0B,0xED,0xF5,0x0C,0xE5,0x2D,0x64,
   0xA5,0x70,0x4F,0xFB,0xEB,0xC3,0x95,0x0A,0x40,0x03,0x02,0x3B,0x6E,0xEB,0x7E,0x00,
   0x25,0x0C,0xFF,0xEE,0x35,0x0B,0xFE,0xEE,0xC0,0xE0,0xEF,0xC0,0xE0,0xEB,0x54,0x07,
   0xFD,0x24,0x23,0xF8,0xE6,0xFF,0x74,0x36,0x2B,0xF8,0xE6,0x6F,0xFE,0x74,0x22,0x2D,
   0xF8,0xE6,0xFD,0x74,0x35,0x2B,0xF8,0xE6,0x6D,0xFD,0xEE,0xFC,0xD0,0xE0,0xFF,0xD0,
   0xE0,0xFE,0x12,0x3E,0xAC,0xC9,0xEF,0xC9,0xE9,0x60,0x03,0x02,0x3B,0x6E,0x0B,0x0B,
   0x80,0xB2,0xE4,0xFB,0xEB,0xC3,0x95,0x0A,0x40,0x03,0x02,0x3B,0x6E,0xE5,0x0C,0x2B,
   0xFF,0xEB,0x54,0x07,0x24,0x22,0xF8,0xE6,0xFE,0x74,0x35,0x2B,0xF8,0xE6,0x6E,0xFD,
   0x12,0x3F,0x7A,0xC9,0xEF,0xC9,0xE9,0x60,0x03,0x02,0x3B,0x6E,0x0B,0x80,0xD5,0xE5,
   0x2E,0x24,0xFB,0xF5,0x0A,0x54,0x07,0x60,0x03,0x02,0x3B,0x6E,0xE5,0x31,0xFE,0xE5,
   0x30,0xFD,0xED,0x8E,0x73,0xF5,0x74,0xE4,0xFB,0xEB,0xC3,0x95,0x0A,0x50,0x2A,0xEB,
   0x54,0x07,0x24,0x22,0xF8,0xE6,0xFF,0x74,0x35,0x2B,0xF8,0xE6,0x6F,0xFF,0x85,0x74,
   0x82,0x85,0x73,0x83,0xE4,0x93,0x6F,0x60,0x05,0x79,0xF5,0x02,0x3B,0x6E,0x05,0x74,
   0xE5,0x74,0x70,0x02,0x05,0x73,0x0B,0x80,0xD0,0xE4,0xF9,0x02,0x3B,0x6E,0xE5,0x34,
   0x25,0x2B,0xF5,0x2B,0x75,0x85,0xC0,0x7B,0x06,0xEB,0xC3,0x95,0x2B,0x50,0x1C,0xE5,
   0x30,0x2B,0x24,0xFA,0x25,0xE0,0xF5,0x84,0x75,0x86,0x8E,0x74,0x4C,0x2B,0xF5,0x82,
   0xE4,0x34,0x00,0xF5,0x83,0xE5,0x8E,0xF0,0x0B,0x80,0xDE,0xE4,0xF9,0x02,0x3B,0x6E,
   0xE5,0x30,0xC3,0x94,0x08,0x50,0x03,0x02,0x3B,0x6E,0x75,0x30,0x04,0xE4,0xF5,0x0B,
   0xF5,0x0C,0x74,0xFF,0xFD,0xFC,0xAF,0x0C,0xAE,0x0B,0x12,0x3E,0xAC,0xC9,0xEF,0xC9,
   0x74,0x04,0x25,0x0C,0xF5,0x0C,0xE4,0x35,0x0B,0xF5,0x0B,0x54,0x03,0x45,0x0C,0x70,
   0x02,0x15,0x30,0xE5,0x30,0x70,0xDB,0xC2,0x00,0x02,0x3B,0x6E,0xE4,0xFB,0xCF,0xEB,
   0xCF,0x7D,0xFF,0x12,0x3F,0x7A,0xC9,0xEF,0xC9,0x0B,0xBB,0x80,0xF1,0x02,0x3B,0x6E,
   0xE5,0x2E,0xC3,0x94,0x1E,0x50,0x03,0x02,0x3B,0x6E,0xE5,0x2E,0x75,0xF0,0x07,0x84,
   0xFB,0x25,0xE0,0x25,0xE0,0x24,0x30,0xF8,0xE6,0x65,0x70,0xF5,0x22,0x74,0x30,0x2B,
   0xF8,0xE6,0x65,0x70,0xF5,0x24,0xEB,0x75,0xF0,0x06,0xA4,0x24,0x30,0xF8,0xE6,0x65,
   0x70,0xF5,0x25,0xEB,0x75,0xF0,0x03,0xA4,0x24,0x30,0xF8,0xE6,0x65,0x70,0xF5,0x26,
   0xEB,0x75,0xF0,0x05,0xA4,0x24,0x30,0xF8,0xE6,0x65,0x70,0xF5,0x28,0xE5,0x2E,0x75,
   0xF0,0x05,0x84,0xFB,0x24,0x30,0xF8,0xE6,0x65,0x70,0xF5,0x23,0xEB,0x75,0xF0,0x03,
   0xA4,0x24,0x30,0xF8,0xE6,0x65,0x70,0xF5,0x27,0xE5,0xA1,0x25,0x22,0xF5,0x29,0xE4,
   0xFB,0xF9,0x74,0x22,0x2B,0xF8,0xE6,0x29,0xF9,0x0B,0xBB,0x08,0xF5,0x02,0x3B,0x6E,
   0xE5,0x30,0x54,0x07,0x64,0x07,0x60,0x03,0x02,0x3B,0x6E,0x53,0x3B,0x7F,0x43,0x3B,
   0x40,0xFB,0xEB,0x24,0xF0,0xFF,0xE4,0x34,0x3F,0xFE,0xEE,0xC0,0xE0,0xEF,0xC0,0xE0,
   0x74,0x33,0x2B,0xF8,0xE6,0xFE,0x74,0x32,0x2B,0xF8,0xE6,0xFD,0xEE,0xFC,0xD0,0xE0,
   0xFF,0xD0,0xE0,0xFE,0x12,0x3E,0xAC,0xC9,0xEF,0xC9,0x0B,0x0B,0xEB,0xC3,0x94,0x0A,
   0x40,0xD0,0x02,0x3B,0x6E,0xE4,0xF9,0xE5,0x30,0x54,0x07,0x64,0x07,0x70,0x3F,0x79,
   0x07,0x75,0x73,0x3F,0x75,0x74,0xF0,0xFB,0x85,0x74,0x82,0x85,0x73,0x83,0xE4,0x93,
   0xFF,0x74,0x4C,0x25,0x2B,0xF5,0x82,0xE4,0x34,0x00,0xF5,0x83,0xEF,0xF0,0x05,0x74,
   0xE5,0x74,0x70,0x02,0x05,0x73,0x05,0x2B,0x0B,0xBB,0x0A,0xDC,0x05,0x2B,0x05,0x2B,
   0x90,0x00,0x56,0xE0,0x54,0xDF,0xFF,0xF0,0xE5,0xB1,0x54,0x20,0x4F,0xF0,0xE5,0x30,
   0x30,0xE3,0x1F,0xC9,0x44,0x08,0xC9,0xE4,0xFB,0xEB,0x90,0x3E,0xA8,0x93,0xFF,0x74,
   0x4C,0x25,0x2B,0xF5,0x82,0xE4,0x34,0x00,0xF5,0x83,0xEF,0xF0,0x05,0x2B,0x0B,0xBB,
   0x04,0xE7,0xE5,0x30,0x30,0xE4,0x3C,0xC9,0x44,0x10,0xC9,0x75,0x73,0x3F,0x75,0x74,
   0xFC,0xE4,0xF5,0x70,0xFB,0x85,0x74,0x82,0x85,0x73,0x83,0xE4,0x93,0xFF,0x74,0x4C,
   0x25,0x2B,0xF5,0x82,0xE4,0x34,0x00,0xF5,0x83,0xEF,0xF0,0x05,0x2B,0x25,0x70,0xF5,
   0x70,0x05,0x74,0xE5,0x74,0x70,0x02,0x05,0x73,0x0B,0xBB,0x04,0xD8,0x74,0x04,0x25,
   0x2B,0xF5,0x2B,0x75,0xA1,0x55,0x75,0xA1,0xAA,0x75,0xB1,0x0C,0xE4,0xF5,0xA1,0x80,
   0x0D,0xE5,0x30,0xB4,0x01,0x04,0xD2,0x01,0x80,0x02,0xD2,0x00,0xE4,0xF9,0x90,0x00,
   0x4C,0xE5,0x2D,0xF0,0xE5,0x2B,0x24,0xFC,0x90,0x00,0x4E,0xF0,0xE4,0xA3,0xF0,0xA3,
   0xE9,0xF0,0xE4,0xF5,0x75,0xF5,0x2A,0x22,0x20,0xD9,0x03,0x02,0x3C,0x99,0xE5,0xD9,
   0x54,0x3F,0x70,0x03,0x02,0x3C,0x93,0x24,0xE0,0x70,0x03,0x02,0x3C,0x73,0x24,0xFE,
   0x60,0x38,0x24,0xF2,0x60,0x3F,0x24,0x2E,0x60,0x03,0x02,0x3C,0x96,0x20,0xDE,0x03,
   0x02,0x3C,0x96,0x85,0xDB,0x09,0x75,0x0D,0x01,0x75,0x0E,0x00,0x75,0x0F,0x0C,0x85,
   0x09,0x10,0x7B,0x00,0x7A,0x00,0x79,0x2D,0x12,0x3F,0x1B,0x75,0x75,0x5A,0x12,0x38,
   0x04,0x85,0x2B,0xD5,0x53,0xD4,0xFC,0x02,0x3C,0x96,0xE5,0xD4,0x54,0xFC,0x44,0x02,
   0xF5,0xD4,0x02,0x3C,0x96,0xE5,0xDB,0x64,0x08,0x70,0x69,0x90,0x00,0x06,0xE0,0xF5,
   0x2C,0xC2,0x03,0x90,0x00,0x00,0xE0,0x54,0x60,0x60,0x04,0xD2,0x03,0x80,0x57,0x90,
   0x00,0x01,0xE0,0xF5,0x6F,0xE5,0x6F,0x24,0xFB,0x60,0x30,0x24,0xFD,0x60,0x2E,0x14,
   0x60,0x36,0x24,0x03,0x70,0x3A,0x90,0x00,0x03,0xE0,0x24,0xFE,0x60,0x0E,0x04,0x70,
   0x16,0x75,0x71,0x3E,0x75,0x72,0x76,0x75,0x09,0x12,0x80,0x2A,0x75,0x71,0x3E,0x75,
   0x72,0x88,0x75,0x09,0x20,0x80,0x1F,0xD2,0x03,0x80,0x1B,0x80,0x0B,0x90,0x00,0x00,
   0xE5,0x21,0xF0,0x75,0x09,0x01,0x80,0x0E,0x90,0x00,0x02,0xE0,0xF5,0x21,0x80,0x06,
   0xD2,0x03,0x80,0x02,0xD2,0x03,0x30,0x03,0x08,0x75,0x6F,0xFF,0x75,0xDC,0xCF,0x80,
   0x35,0xE5,0x2C,0xD3,0x95,0x09,0x40,0x03,0x85,0x09,0x2C,0x12,0x3E,0xE4,0x75,0xDC,
   0xC0,0x80,0x23,0xE5,0x6F,0x24,0xFB,0x60,0x0B,0x14,0x70,0x12,0x12,0x3E,0xE4,0x63,
   0xDC,0x40,0x80,0x12,0xE5,0xE3,0x54,0x80,0x45,0x21,0xF5,0xE3,0x80,0x05,0xE4,0xF5,
   0xDD,0x80,0x00,0x75,0xDC,0x02,0xC2,0xD9,0x22,0x30,0xD8,0x0B,0x75,0xDC,0x02,0x75,
   0xD4,0x12,0xE4,0xF5,0xE3,0x80,0x06,0x30,0xDA,0x03,0xC2,0xDA,0x22,0x75,0xD8,0xFF,
   0x22,0x75,0xA1,0x55,0x75,0xA1,0xAA,0xE5,0xB9,0x54,0xF8,0x44,0x04,0xF5,0xB9,0xC2,
   0x01,0xD2,0x00,0xE5,0xA1,0x13,0x92,0x02,0xC2,0xAF,0xC2,0x8C,0xC2,0x8D,0x30,0x02,
   0x16,0x75,0x98,0x50,0xE4,0xF5,0xC8,0x75,0x87,0x80,0x75,0x89,0x20,0x75,0xC9,0xA0,
   0x75,0x8D,0xF3,0xD2,0x8E,0x80,0x06,0x75,0xC0,0x30,0x75,0xC2,0xF3,0x90,0x3F,0xF4,
   0xE4,0x93,0xFF,0x30,0xE1,0x07,0xA2,0xB6,0xE4,0x33,0xFE,0x80,0x09,0x30,0x95,0x04,
   0x7E,0x00,0x80,0x02,0x7E,0x01,0xEE,0x70,0x13,0xE5,0xB1,0x30,0xE5,0x0E,0x90,0x00,
   0x00,0xE4,0x93,0xB4,0xFF,0x23,0xA3,0xE4,0x93,0xB4,0xFF,0x1D,0xE4,0xF5,0xE2,0x75,
   0xEB,0x0C,0xF5,0xED,0xF5,0xEC,0xF5,0xE5,0x75,0xE4,0x0C,0x75,0xE2,0x29,0x75,0xD1,
   0x81,0x75,0xD8,0xFF,0x75,0xE1,0x07,0x80,0x13,0xEF,0x30,0xE0,0x0D,0x43,0x89,0x01,
   0x75,0x8A,0xC0,0x75,0x8C,0x64,0xD2,0x8C,0x80,0x02,0xD2,0x01,0xE4,0xF5,0x08,0x12,
   0x3E,0x18,0x74,0x22,0x25,0x08,0xF8,0xC6,0xEF,0xC6,0x05,0x08,0xE5,0x08,0xC3,0x94,
   0x08,0x40,0xEC,0x30,0x01,0x0B,0x75,0xA1,0x55,0x75,0xA1,0xAA,0x75,0xB1,0x10,0x80,
   0xFE,0x20,0xC0,0x03,0x30,0x98,0x06,0x75,0x2A,0x96,0x12,0x3D,0x94,0xE5,0xD8,0x54,
   0x07,0x60,0x06,0x75,0x2A,0x96,0x12,0x3B,0x88,0x30,0x8D,0xD7,0xC2,0x8D,0xC2,0x8C,
   0xD2,0x01,0x80,0xCF,0x12,0x3F,0xB4,0xEF,0x64,0x57,0x70,0x7B,0x12,0x3F,0xB4,0xEF,
   0x64,0xAB,0x70,0x73,0x12,0x3F,0xB4,0x8F,0x2D,0xAE,0x2D,0x12,0x3F,0xB4,0x8F,0x2E,
   0xE5,0x2E,0x2E,0xFE,0x12,0x3F,0xB4,0x8F,0x2F,0xE4,0xFD,0xED,0x65,0x2E,0x60,0x0E,
   0x12,0x3F,0xB4,0x74,0x30,0x2D,0xF8,0xEF,0xF6,0x2E,0xFE,0x0D,0x80,0xED,0x12,0x3F,
   0xB4,0xEF,0x6E,0x70,0x42,0xC2,0x8C,0xC2,0x8D,0x75,0x75,0x5A,0x12,0x38,0x04,0xE4,
   0xFE,0x7F,0x55,0x12,0x3F,0x9D,0x7F,0xAA,0x12,0x3F,0x9D,0xFD,0xED,0xC3,0x95,0x2B,
   0x50,0x1F,0x74,0x4C,0x2D,0xF5,0x82,0xE4,0x34,0x00,0xF5,0x83,0xE0,0xFF,0x12,0x3F,
   0x9D,0x74,0x4C,0x2D,0xF5,0x82,0xE4,0x34,0x00,0xF5,0x83,0xE0,0x2E,0xFE,0x0D,0x80,
   0xDB,0xCF,0xEE,0xCF,0x12,0x3F,0x9D,0x22,0xAC,0x11,0xAD,0x12,0xAE,0x13,0xAF,0x14,
   0x78,0x10,0xEC,0x4D,0x4E,0x4F,0x70,0x04,0x7C,0xA5,0x7D,0xA5,0xC3,0xEC,0x13,0xFC,
   0xED,0x13,0xFD,0xEE,0x13,0xFE,0xEF,0x13,0xFF,0x50,0x10,0xEC,0x64,0xCC,0xFC,0xED,
   0x64,0x4C,0xFD,0xEE,0x64,0x4E,0xFE,0xEF,0x64,0xCE,0xFF,0xD8,0xDF,0x8C,0x11,0x8D,
   0x12,0x8E,0x13,0x8F,0x14,0xEE,0x54,0x7F,0xFE,0x22,0x8E,0x11,0x8F,0x12,0x74,0xA5,
   0xF5,0x13,0xF5,0x14,0x22,0x4D,0x43,0x55,0x20,0x49,0x53,0x50,0x20,0x26,0x20,0x57,
   0x43,0x48,0x2E,0x43,0x4E,0x00,0x12,0x01,0x10,0x01,0xFF,0x80,0x55,0x08,0x48,0x43,
   0xE0,0x55,0x00,0x01,0x00,0x00,0x00,0x01,0x09,0x02,0x20,0x00,0x01,0x01,0x00,0x80,
   0x32,0x09,0x04,0x00,0x00,0x02,0xFF,0x80,0x55,0x00,0x07,0x05,0x82,0x02,0x40,0x00,
   0x00,0x07,0x05,0x02,0x02,0x40,0x00,0x00,0x00,0x02,0x03,0x01,0xE5,0x2A,0x64,0x96,
   0x60,0x02,0x80,0xFE,0xC3,0xEE,0x94,0x38,0x40,0x11,0xEF,0x94,0xF0,0xEE,0x94,0x3F,
   0x40,0x1F,0xD3,0xEF,0x94,0xF8,0xEE,0x94,0x3F,0x50,0x16,0x8E,0x85,0x8F,0x84,0x8C,
   0x8F,0x8D,0x8E,0xE5,0x86,0x30,0xE6,0x09,0x75,0x86,0x9A,0xE5,0x86,0x64,0x40,0xFF,
   0x22,0x7F,0x40,0x22,0xE5,0x2C,0xC3,0x94,0x08,0x40,0x04,0x7F,0x08,0x80,0x02,0xAF,
   0x2C,0xCE,0xEF,0xCE,0x75,0x0D,0xFF,0x85,0x71,0x0E,0x85,0x72,0x0F,0x8E,0x10,0x7B,
   0x01,0x7A,0x00,0x79,0x00,0x12,0x3F,0x1B,0xC3,0xE5,0x2C,0x9E,0xF5,0x2C,0xEE,0x25,
   0x72,0xF5,0x72,0xE4,0x35,0x71,0xF5,0x71,0x8E,0xDD,0x22,0x8B,0x0A,0x8A,0x0B,0x89,
   0x0C,0xAF,0x10,0x15,0x10,0xEF,0x60,0x26,0xAB,0x0D,0x05,0x0F,0xE5,0x0F,0xAA,0x0E,
   0x70,0x02,0x05,0x0E,0x14,0xF9,0x12,0x3F,0x4F,0xFF,0xAB,0x0A,0x05,0x0C,0xE5,0x0C,
   0xAA,0x0B,0x70,0x02,0x05,0x0B,0x14,0xF9,0xEF,0x12,0x3F,0x68,0x80,0xD3,0x22,0xBB,
   0x01,0x06,0x89,0x82,0x8A,0x83,0xE0,0x22,0x50,0x02,0xE7,0x22,0xBB,0xFE,0x02,0xE3,
   0x22,0x89,0x82,0x8A,0x83,0xE4,0x93,0x22,0xBB,0x01,0x06,0x89,0x82,0x8A,0x83,0xF0,
   0x22,0x50,0x02,0xF7,0x22,0xBB,0xFE,0x01,0xF3,0x22,0xE5,0x2A,0x64,0x96,0x60,0x02,
   0x80,0xFE,0x75,0x85,0xC0,0xEF,0x25,0xE0,0xF5,0x84,0x8D,0x8E,0xE5,0x86,0x30,0xE6,
   0x09,0x75,0x86,0x9A,0xE5,0x86,0x64,0x40,0xFF,0x22,0x7F,0x40,0x22,0x30,0x02,0x0A,
   0xC2,0x99,0x8F,0x99,0x30,0x99,0xFD,0xC2,0x99,0x22,0xC2,0xC1,0x8F,0xC1,0x30,0xC1,
   0xFD,0xC2,0xC1,0x22,0x30,0x02,0x08,0x30,0x98,0xFD,0xC2,0x98,0xAF,0x99,0x22,0x30,
   0xC0,0xFD,0xC2,0xC0,0xAF,0xC1,0x22,0x78,0x7F,0xE4,0xF6,0xD8,0xFD,0x75,0x81,0x75,
   0x02,0x3C,0xB1,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
   0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

