// Bootloader original v1.1 blob
// Start: 0x3800
// End  : 0x3FEF
// usable for CH551, CH552,CH554,(CH553)
const unsigned char code LoaderImage[2032]=
{
   0x02,0x3D,0x30,0x00,0xE4,0xF5,0x18,0xF5,0x1A,0x12,0x3F,0xB0,0x78,0x21,0xE6,0x12,
   0x3E,0x4D,0x38,0x3D,0xA2,0x3A,0x90,0xA5,0x39,0x4B,0xA6,0x38,0x5A,0xA7,0x38,0xA0,
   0xA8,0x39,0x04,0xA9,0x3A,0x72,0xB5,0x3A,0x36,0xB6,0x3A,0x0D,0xB7,0x39,0x5F,0xB8,
   0x39,0xD7,0xB9,0x39,0x76,0xBA,0x3A,0x07,0xBB,0x00,0x00,0x3A,0x9B,0x85,0xA1,0x18,
   0xE4,0xF9,0xE9,0x90,0x3E,0x79,0x93,0xFF,0x74,0x23,0x29,0xF8,0xE6,0x6F,0x60,0x03,
   0x02,0x39,0x1B,0x09,0xB9,0x13,0xEB,0x02,0x3A,0x9E,0x78,0x24,0xE6,0xFE,0x18,0xE6,
   0xFD,0xED,0x8E,0x15,0xF5,0x16,0xE4,0xF5,0x19,0xE4,0xF9,0x74,0x11,0x29,0xF8,0xE6,
   0xFF,0x74,0x25,0x25,0x19,0xF8,0xE6,0x6F,0xFF,0x85,0x16,0x82,0x85,0x15,0x83,0xE4,
   0x93,0x6F,0x60,0x03,0x02,0x39,0x1B,0x05,0x19,0x05,0x16,0xE5,0x16,0x70,0x02,0x05,
   0x15,0x09,0xB9,0x04,0xD6,0xE5,0x19,0xC3,0x78,0x22,0x96,0x40,0xCC,0x02,0x3A,0x9E,
   0x30,0x01,0x03,0x02,0x3B,0x04,0x78,0x24,0xE6,0xFE,0x18,0xE6,0xFD,0xEE,0xF5,0x1B,
   0xED,0xF5,0x1C,0xE4,0xF5,0x19,0xE4,0xF9,0xE9,0x25,0xE0,0x24,0x12,0xF8,0xE6,0xFF,
   0x74,0x26,0x25,0x19,0xF8,0xE6,0x6F,0xFE,0xE9,0x25,0xE0,0x24,0x11,0xF8,0xE6,0xFD,
   0x74,0x25,0x25,0x19,0xF8,0xE6,0x6D,0xFD,0xEE,0xFC,0x7E,0x00,0xE5,0x1C,0x25,0x19,
   0xFF,0xEE,0x35,0x1B,0xFE,0x12,0x3F,0x05,0x8F,0x18,0xE5,0x18,0x60,0x03,0x02,0x3A,
   0x9E,0x05,0x19,0x05,0x19,0x09,0xB9,0x02,0xBF,0xE5,0x19,0xC3,0x78,0x22,0x96,0x40,
   0xB5,0x02,0x3A,0x9E,0x78,0x24,0xE6,0xFE,0x18,0xE6,0xFD,0xEE,0xF5,0x1B,0xED,0xF5,
   0x1C,0x20,0x01,0x03,0x02,0x3A,0x9E,0x45,0x1B,0x60,0x06,0x75,0x18,0xFF,0x02,0x3A,
   0x9E,0x74,0xFF,0xFD,0xFC,0xAF,0x1C,0xAE,0x1B,0x12,0x3F,0x05,0x8F,0x18,0xE5,0x18,
   0x60,0x03,0x02,0x3A,0x9E,0x74,0x02,0x25,0x1C,0xF5,0x1C,0xE4,0x35,0x1B,0xF5,0x1B,
   0x54,0x03,0x45,0x1C,0x70,0xDB,0xC2,0x01,0x02,0x3A,0x9E,0x78,0x23,0xE6,0xF5,0x11,
   0x08,0xE6,0xF5,0x12,0x08,0xE6,0xF5,0x13,0x08,0xE6,0xF5,0x14,0x02,0x3A,0x9E,0x90,
   0x3F,0xF8,0xE4,0x93,0x78,0x23,0x66,0x70,0x09,0xA3,0x93,0x08,0x66,0x70,0x03,0x02,
   0x3A,0x9E,0x78,0x24,0x80,0x4E,0x90,0x3F,0xF6,0xE4,0x93,0x78,0x25,0x66,0x70,0x06,
   0xA3,0x93,0x08,0x66,0x60,0x20,0x78,0x24,0x7F,0xF4,0x12,0x3E,0xFC,0x8F,0x18,0xE5,
   0x18,0x60,0x03,0x02,0x3A,0x9E,0x78,0x26,0x7F,0xF6,0x12,0x3E,0xFC,0x8F,0x18,0xE5,
   0x18,0x60,0x03,0x02,0x3A,0x9E,0x78,0x22,0xE6,0x64,0x08,0x60,0x03,0x02,0x3A,0x9E,
   0x90,0x3F,0xF8,0x93,0x78,0x29,0x66,0x70,0x09,0xA3,0x93,0x08,0x66,0x70,0x03,0x02,
   0x3A,0x9E,0x78,0x2A,0xE6,0xFE,0x18,0xE6,0xFD,0xEE,0xFC,0x7F,0xF8,0x7E,0x3F,0x12,
   0x3F,0x05,0x8F,0x18,0x02,0x3A,0x9E,0x75,0x15,0x3F,0x75,0x16,0xF8,0xE5,0x1A,0xC3,
   0x94,0x08,0x40,0x03,0x02,0x3A,0x9E,0xE5,0x1A,0x24,0xF8,0xFF,0xE4,0x34,0x3F,0x8F,
   0x82,0xF5,0x83,0xE4,0x93,0xFF,0x74,0x4C,0x25,0x1A,0xF5,0x82,0xE4,0x34,0x00,0xF5,
   0x83,0xEF,0xF0,0x05,0x1A,0x80,0xD6,0x75,0x18,0x11,0x02,0x3A,0x9E,0x75,0x85,0xC0,
   0xE5,0x1A,0xC3,0x94,0x80,0x50,0x1B,0xE5,0x1A,0x25,0xE0,0xF5,0x84,0x75,0x86,0x8E,
   0x74,0x4C,0x25,0x1A,0xF5,0x82,0xE4,0x34,0x00,0xF5,0x83,0xE5,0x8E,0xF0,0x05,0x1A,
   0x80,0xDE,0xD2,0x00,0x80,0x68,0x78,0x23,0xE6,0x75,0x1B,0x00,0xF5,0x1C,0x75,0x85,
   0xC0,0xE4,0xF5,0x19,0xE4,0xF9,0xE5,0x19,0x25,0x1C,0xFF,0x74,0x11,0x29,0xF8,0xE6,
   0xFE,0x74,0x25,0x25,0x19,0xF8,0xE6,0x6E,0xFD,0x12,0x3F,0x89,0x8F,0x18,0xE5,0x18,
   0x70,0x3C,0x05,0x19,0x09,0xB9,0x04,0xDE,0xE5,0x19,0xC3,0x78,0x22,0x96,0x40,0xD4,
   0x80,0x2C,0x78,0x23,0xE6,0xF9,0x75,0x85,0xC0,0xE9,0xC3,0x94,0x80,0x50,0x1F,0xCF,
   0xE9,0xCF,0x7D,0xFF,0x12,0x3F,0x89,0x8F,0x18,0xE5,0x18,0x70,0x11,0x09,0x80,0xE9,
   0x78,0x24,0xE6,0x18,0x46,0x60,0x07,0xD2,0x02,0x80,0x03,0x75,0x18,0xFE,0x12,0x3F,
   0xB0,0x20,0x03,0x38,0xE5,0x1A,0x60,0x11,0xD3,0x94,0x40,0x40,0x04,0x7F,0x40,0x80,
   0x02,0xAF,0x1A,0x8F,0xD5,0x53,0xD4,0xFC,0x22,0x90,0x00,0x4C,0xE5,0x18,0xF0,0xE4,
   0xA3,0xF0,0x75,0xD5,0x02,0x53,0xD4,0xFC,0x30,0x02,0x39,0x75,0x18,0x32,0xAF,0x18,
   0x15,0x18,0xEF,0x60,0x2F,0x7F,0x64,0x12,0x3F,0xA1,0x80,0xF2,0xE5,0x1A,0x60,0x1A,
   0xE4,0xF9,0xE9,0xC3,0x95,0x1A,0x50,0x1C,0x74,0x4C,0x29,0xF5,0x82,0xE4,0x34,0x00,
   0xF5,0x83,0xE0,0xFF,0x12,0x3F,0xBD,0x09,0x80,0xE8,0xAF,0x18,0x12,0x3F,0xBD,0xE4,
   0xFF,0x12,0x3F,0xBD,0x22,0x20,0xD9,0x03,0x02,0x3C,0x38,0xE5,0xD9,0x54,0x3F,0x70,
   0x03,0x02,0x3C,0x32,0x24,0xE0,0x70,0x03,0x02,0x3C,0x12,0x24,0xFE,0x60,0x31,0x24,
   0xF2,0x60,0x5B,0x24,0x2E,0x60,0x03,0x02,0x3C,0x35,0x20,0xDE,0x03,0x02,0x3C,0x35,
   0x85,0xDB,0x17,0x75,0x1B,0x01,0x75,0x1C,0x00,0x75,0x1D,0x0C,0x85,0x17,0x1E,0x7B,
   0x00,0x7A,0x00,0x79,0x21,0x12,0x3F,0x35,0xC2,0x03,0x12,0x38,0x04,0x02,0x3C,0x35,
   0x30,0x00,0x20,0xC2,0x00,0x75,0x1B,0x01,0x75,0x1C,0x00,0x75,0x1D,0x8C,0x75,0x1E,
   0x40,0x7B,0x01,0x7A,0x00,0x79,0x4C,0x12,0x3F,0x35,0x75,0xD5,0x40,0x53,0xD4,0xFC,
   0x02,0x3C,0x35,0xE5,0xD4,0x54,0xFC,0x44,0x02,0xF5,0xD4,0x02,0x3C,0x35,0xE5,0xDB,
   0x64,0x08,0x70,0x6F,0x90,0x00,0x06,0xE0,0xF5,0x09,0xC2,0x04,0x90,0x00,0x00,0xE0,
   0x54,0x60,0x60,0x04,0xD2,0x04,0x80,0x5D,0x90,0x00,0x01,0xE0,0xF5,0x0D,0xE5,0x0D,
   0x24,0xFB,0x60,0x36,0x24,0xFD,0x60,0x34,0x14,0x60,0x3C,0x24,0x03,0x70,0x40,0x90,
   0x00,0x03,0xE0,0x24,0xFE,0x60,0x11,0x04,0x70,0x1C,0x75,0x0E,0xFF,0x75,0x0F,0x3E,
   0x75,0x10,0x8D,0x75,0x17,0x12,0x80,0x2D,0x75,0x0E,0xFF,0x75,0x0F,0x3E,0x75,0x10,
   0x9F,0x75,0x17,0x20,0x80,0x1F,0xD2,0x04,0x80,0x1B,0x80,0x0B,0x90,0x00,0x00,0xE5,
   0x08,0xF0,0x75,0x17,0x01,0x80,0x0E,0x90,0x00,0x02,0xE0,0xF5,0x08,0x80,0x06,0xD2,
   0x04,0x80,0x02,0xD2,0x04,0x30,0x04,0x08,0x75,0x0D,0xFF,0x75,0xDC,0xCF,0x80,0x35,
   0xE5,0x09,0xD3,0x95,0x17,0x40,0x03,0x85,0x17,0x09,0x12,0x3E,0xBF,0x75,0xDC,0xC0,
   0x80,0x23,0xE5,0x0D,0x24,0xFB,0x60,0x0B,0x14,0x70,0x12,0x12,0x3E,0xBF,0x63,0xDC,
   0x40,0x80,0x12,0xE5,0xE3,0x54,0x80,0x45,0x08,0xF5,0xE3,0x80,0x05,0xE4,0xF5,0xDD,
   0x80,0x00,0x75,0xDC,0x02,0xC2,0xD9,0x22,0x30,0xD8,0x13,0x75,0xDC,0x02,0x75,0xD2,
   0x10,0x75,0xD4,0x12,0xE4,0xF5,0xE3,0xC2,0xDA,0xC2,0xD9,0xC2,0xD8,0x22,0x30,0xDA,
   0x03,0xC2,0xDA,0x22,0x75,0xD8,0xFF,0x22,0x75,0xA1,0x55,0x75,0xA1,0xAA,0xE5,0xB9,
   0x54,0xF8,0x44,0x04,0xF5,0xB9,0xE4,0xF5,0xA1,0xC2,0x03,0xC2,0x02,0xD2,0x01,0xC2,
   0x00,0xC2,0xAF,0xC2,0x8C,0xC2,0x8D,0xC2,0xC7,0xD2,0xC5,0xD2,0xC4,0x75,0xC2,0xF3,
   0xC2,0xC1,0xC2,0xC0,0xE5,0xB1,0x30,0xE5,0x08,0x12,0x3F,0x69,0xEF,0x64,0x01,0x70,
   0x2B,0xE4,0xF5,0xE2,0x75,0xEB,0x0C,0x75,0xED,0x00,0x75,0xEC,0x00,0x75,0xE5,0x00,
   0x75,0xE4,0x0C,0x75,0xDC,0x02,0x75,0xD4,0x12,0xF5,0xE3,0x75,0xD1,0x80,0x75,0xE2,
   0x29,0x43,0xD1,0x01,0x75,0xD8,0xFF,0x75,0xE1,0x07,0x80,0x0D,0x43,0x89,0x01,0x75,
   0x8A,0xC0,0x75,0x8C,0x64,0xC2,0x8D,0xD2,0x8C,0x30,0xC0,0x03,0x12,0x3D,0xBC,0xE5,
   0xD8,0x54,0x07,0x60,0x03,0x12,0x3B,0x05,0x30,0x8D,0x09,0xC2,0x8D,0xC2,0x8C,0x20,
   0x03,0x02,0xD2,0x02,0x30,0x02,0xE2,0x90,0x3F,0xF4,0xE4,0x93,0x64,0x57,0x70,0x35,
   0xA3,0x93,0x64,0xA8,0x70,0x2F,0x75,0xA1,0x55,0x75,0xA1,0xAA,0x75,0xB9,0x83,0xF5,
   0xA1,0x75,0xE2,0x06,0xC2,0xC5,0xC2,0xC4,0xC2,0x8C,0x90,0x3F,0xF7,0x93,0xFE,0x90,
   0x3F,0xF6,0xE4,0x93,0xFD,0xED,0xCA,0xEE,0xCA,0xF9,0x7B,0x00,0x8B,0x0A,0x8A,0x0B,
   0xF5,0x0C,0x12,0x3E,0x73,0x75,0xA1,0x55,0x75,0xA1,0xAA,0x43,0xB1,0x10,0x80,0xFE,
   0x78,0x7F,0xE4,0xF6,0xD8,0xFD,0x75,0x81,0x60,0x02,0x3D,0x77,0x02,0x3C,0x58,0xE4,
   0x93,0xA3,0xF8,0xE4,0x93,0xA3,0x40,0x03,0xF6,0x80,0x01,0xF2,0x08,0xDF,0xF4,0x80,
   0x29,0xE4,0x93,0xA3,0xF8,0x54,0x07,0x24,0x0C,0xC8,0xC3,0x33,0xC4,0x54,0x0F,0x44,
   0x20,0xC8,0x83,0x40,0x04,0xF4,0x56,0x80,0x01,0x46,0xF6,0xDF,0xE4,0x80,0x0B,0x01,
   0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x90,0x3F,0xCF,0xE4,0x7E,0x01,0x93,0x60,0xBC,
   0xA3,0xFF,0x54,0x3F,0x30,0xE5,0x09,0x54,0x1F,0xFE,0xE4,0x93,0xA3,0x60,0x01,0x0E,
   0xCF,0x54,0xC0,0x25,0xE0,0x60,0xA8,0x40,0xB8,0xE4,0x93,0xA3,0xFA,0xE4,0x93,0xA3,
   0xF8,0xE4,0x93,0xA3,0xC8,0xC5,0x82,0xC8,0xCA,0xC5,0x83,0xCA,0xF0,0xA3,0xC8,0xC5,
   0x82,0xC8,0xCA,0xC5,0x83,0xCA,0xDF,0xE9,0xDE,0xE7,0x80,0xBE,0x30,0xC0,0x62,0x12,
   0x3F,0xC7,0xEF,0x64,0x61,0x70,0x5A,0x12,0x3F,0xC7,0xEF,0x64,0x9E,0x70,0x52,0x12,
   0x3F,0xC7,0x78,0x21,0xEF,0xF6,0xFE,0x12,0x3F,0xC7,0x08,0xEF,0xF6,0xFD,0xE6,0x2E,
   0xFE,0x18,0xE6,0x64,0xA8,0x60,0x09,0xE6,0x64,0xA7,0x60,0x04,0xE6,0xB4,0xB6,0x02,
   0x0D,0x0D,0xE4,0xFC,0xEC,0x6D,0x60,0x13,0x12,0x3F,0xC7,0x74,0x23,0x2C,0xF8,0xEF,
   0xF6,0x74,0x23,0x2C,0xF8,0xE6,0x2E,0xFE,0x0C,0x80,0xE9,0x12,0x3F,0xC7,0xEE,0x6F,
   0x60,0x0A,0x7F,0x55,0x12,0x3F,0xBD,0x7F,0xAA,0x02,0x3F,0xBD,0xD2,0x03,0x12,0x38,
   0x04,0x22,0xBB,0x01,0x06,0x89,0x82,0x8A,0x83,0xE0,0x22,0x50,0x02,0xE7,0x22,0xBB,
   0xFE,0x02,0xE3,0x22,0x89,0x82,0x8A,0x83,0xE4,0x93,0x22,0xBB,0x01,0x06,0x89,0x82,
   0x8A,0x83,0xF0,0x22,0x50,0x02,0xF7,0x22,0xBB,0xFE,0x01,0xF3,0x22,0xD0,0x83,0xD0,
   0x82,0xF8,0xE4,0x93,0x70,0x12,0x74,0x01,0x93,0x70,0x0D,0xA3,0xA3,0x93,0xF8,0x74,
   0x01,0x93,0xF5,0x82,0x88,0x83,0xE4,0x73,0x74,0x02,0x93,0x68,0x60,0xEF,0xA3,0xA3,
   0xA3,0x80,0xDF,0x8A,0x83,0x89,0x82,0xE4,0x73,0x55,0x53,0x42,0x20,0x44,0x42,0x47,
   0x20,0x43,0x48,0x35,0x35,0x39,0x20,0x26,0x20,0x49,0x53,0x50,0x00,0x12,0x01,0x10,
   0x01,0xFF,0x80,0x55,0x08,0x48,0x43,0xE0,0x55,0x00,0x01,0x00,0x00,0x00,0x01,0x09,
   0x02,0x20,0x00,0x01,0x01,0x00,0x80,0x32,0x09,0x04,0x00,0x00,0x02,0xFF,0x80,0x55,
   0x00,0x07,0x05,0x82,0x02,0x40,0x00,0x00,0x07,0x05,0x02,0x02,0x40,0x00,0x00,0xE5,
   0x09,0x60,0x35,0xC3,0x94,0x08,0x40,0x04,0x7F,0x08,0x80,0x02,0xAF,0x09,0xCE,0xEF,
   0xCE,0x85,0x0E,0x1B,0x85,0x0F,0x1C,0x85,0x10,0x1D,0x8E,0x1E,0x7B,0x01,0x7A,0x00,
   0x79,0x00,0x12,0x3F,0x35,0xC3,0xE5,0x09,0x9E,0xF5,0x09,0xEE,0x25,0x10,0xF5,0x10,
   0xE4,0x35,0x0F,0xF5,0x0F,0x8E,0xDD,0x22,0xE4,0xF5,0xDD,0x22,0xE6,0xFE,0x18,0xE6,
   0xFD,0xEE,0xFC,0x7E,0x3F,0xC3,0xEE,0x94,0x38,0x40,0x11,0xEF,0x94,0xF4,0xEE,0x94,
   0x3F,0x40,0x1F,0xD3,0xEF,0x94,0xF8,0xEE,0x94,0x3F,0x50,0x16,0x8E,0x85,0x8F,0x84,
   0x8C,0x8F,0x8D,0x8E,0xE5,0x86,0x30,0xE6,0x09,0x75,0x86,0x9A,0xE5,0x86,0x64,0x40,
   0xFF,0x22,0x7F,0x40,0x22,0x8B,0x18,0x8A,0x19,0x89,0x1A,0xAF,0x1E,0x15,0x1E,0xEF,
   0x60,0x26,0xAB,0x1B,0x05,0x1D,0xE5,0x1D,0xAA,0x1C,0x70,0x02,0x05,0x1C,0x14,0xF9,
   0x12,0x3E,0x22,0xFF,0xAB,0x18,0x05,0x1A,0xE5,0x1A,0xAA,0x19,0x70,0x02,0x05,0x19,
   0x14,0xF9,0xEF,0x12,0x3E,0x3B,0x80,0xD3,0x22,0xE4,0xFE,0x20,0xB6,0x0D,0x7F,0x01,
   0x12,0x3F,0xA1,0x0E,0xEE,0x30,0xE6,0xF3,0x7F,0x00,0x22,0x7F,0x01,0x12,0x3F,0xA1,
   0x30,0xB6,0x03,0x7F,0x01,0x22,0x7F,0x00,0x22,0xEF,0x25,0xE0,0xF5,0x84,0x8D,0x8E,
   0xE5,0x86,0x30,0xE6,0x09,0x75,0x86,0x9A,0xE5,0x86,0x64,0x40,0xFF,0x22,0x7F,0x40,
   0x22,0xEF,0x13,0x13,0x54,0x3F,0xFF,0xEF,0x60,0x05,0x05,0xA1,0x1F,0x80,0xF8,0x22,
   0x75,0xA1,0x55,0x75,0xA1,0xAA,0x63,0xB1,0x0C,0xE4,0xF5,0xA1,0x22,0xC2,0xC1,0x8F,
   0xC1,0x30,0xC1,0xFD,0xC2,0xC1,0x22,0x30,0xC0,0xFD,0xC2,0xC0,0xAF,0xC1,0x22,0x04,
   0x11,0x54,0x77,0x63,0x68,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
   0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF
};

