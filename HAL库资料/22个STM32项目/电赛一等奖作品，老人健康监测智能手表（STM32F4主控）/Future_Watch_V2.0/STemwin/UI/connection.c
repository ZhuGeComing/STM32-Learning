/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Bitmap Converter for emWin V5.28.                           *
*        Compiled Jan 30 2015, 16:40:04                              *
*                                                                    *
*        (c) 1998 - 2015 Segger Microcontroller GmbH & Co. KG        *
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: connection                                            *
* Dimensions:  30 * 30                                               *
* NumColors:   31                                                    *
*                                                                    *
**********************************************************************
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

extern GUI_CONST_STORAGE GUI_BITMAP bmconnection;

/*********************************************************************
*
*       Palette
*
*  Description
*    The following are the entries of the palette table.
*    The entries are stored as a 32-bit values of which 24 bits are
*    actually used according to the following bit mask: 0xBBGGRR
*
*    The lower   8 bits represent the Red   component.
*    The middle  8 bits represent the Green component.
*    The highest 8 bits represent the Blue  component.
*/
static GUI_CONST_STORAGE GUI_COLOR _Colorsconnection[] = {
  0xFFFFFF, 0x000000, 0xF6F6F6, 0x292929,
  0xDEDEDE, 0x202020, 0x313131, 0xD5D5D5,
  0x080808, 0xCDCDCD, 0xB4B4B4, 0xE6E6E6,
  0x101010, 0xC5C5C5, 0x181818, 0x393939,
  0x626262, 0x9C9C9C, 0xA4A4A4, 0xACACAC,
  0xEEEEEE, 0x737373, 0x838383, 0xBDBDBD,
  0x6A6A6A, 0x4A4A4A, 0x525252, 0x7B7B7B,
  0x8B8B8B, 0x414141, 0x5A5A5A
};

static GUI_CONST_STORAGE GUI_LOGPALETTE _Palconnection = {
  31,  // Number of entries
  0,   // No transparency
  &_Colorsconnection[0]
};

static GUI_CONST_STORAGE unsigned char _acconnection[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x13, 0x11, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x18, 0x0C, 0x1B, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x09, 0x0D, 0x09, 0x09, 0x09, 0x09, 0x07, 0x09, 0x12, 0x06, 0x01, 0x08, 0x16, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x19, 0x05, 0x05, 0x05, 0x03, 0x03, 0x03, 0x03, 0x03, 0x0E, 0x01, 0x01, 0x01, 0x08, 0x1E, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x06, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0C, 0x13, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x14, 0x1A, 0x01, 0x01, 0x05, 0x0F, 0x06, 0x06, 0x06, 0x06, 0x06, 0x0F, 0x05, 0x01, 0x01, 0x01, 0x0C, 0x10, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x11, 0x01, 0x01, 0x0F, 0x17, 0x04, 0x04, 0x07, 0x07, 0x07, 0x04, 0x04, 0x0A, 0x06, 0x01, 0x0C, 0x1C, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x15, 0x01, 0x01, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x15, 0x0E, 0x15, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x13, 0x05, 0x0E, 0x11, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x02, 0x0A, 0x12, 0x04, 0x00, 0x00, 0x04, 0x12, 0x17, 0x02, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x14, 0x17, 0x0A, 0x0B, 0x00, 0x00, 0x07, 0x11, 0x17, 0x02, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x1C, 0x0C, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x0E, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x01, 0x01, 0x16, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x1B, 0x08, 0x01, 0x1D, 0x0A, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x13, 0x06, 0x01, 0x08, 0x12, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x1A, 0x08, 0x01, 0x01, 0x01, 0x05, 0x03, 0x03, 0x05, 0x05, 0x05, 0x03, 0x03, 0x0E, 0x01, 0x01, 0x10, 0x14, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0x08, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x10, 0x08, 0x01, 0x01, 0x08, 0x05, 0x06, 0x06, 0x03, 0x03, 0x03, 0x03, 0x05, 0x03, 0x19, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x16, 0x08, 0x01, 0x0F, 0x13, 0x07, 0x09, 0x0D, 0x0D, 0x09, 0x0D, 0x0D, 0x0D, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x18, 0x0C, 0x15, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x11, 0x0A, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

GUI_CONST_STORAGE GUI_BITMAP bmconnection = {
  30, // xSize
  30, // ySize
  30, // BytesPerLine
  8, // BitsPerPixel
  _acconnection,  // Pointer to picture data (indices)
  &_Palconnection   // Pointer to palette
};


static GUI_CONST_STORAGE GUI_COLOR _Colorsunconnection[] = {
  0xEEA400, 0xFFFFFF, 0xFFBD52, 0xEE9C00,
  0xEE9400, 0xFFC562, 0xFFBD4A, 0xF6BD5A,
  0xEEAC00, 0xE69400, 0xE6FFFF, 0xF6EECD,
  0xE68B00, 0xFFB439, 0xEEB44A, 0xFFB441,
  0xEE9C10, 0xEEA408, 0xF6B431, 0xFFB431,
  0xFFBD5A, 0xFFBD62, 0xEECD8B, 0xEED5A4,
  0xDEFFFF, 0xEE9C08, 0xE6A400, 0xF6AC20,
  0xFFB429, 0xFFC55A, 0xEED594, 0xF6E6BD,
  0xDEF6FF, 0xE6F6EE, 0xFFFFEE, 0xE69410,
  0xE6A410, 0xE6AC10, 0xEEAC29, 0xF6AC29,
  0xE6AC39, 0xEEBD5A, 0xF6BD52, 0xFFC56A,
  0xEED59C, 0xFFF6E6, 0xFFF6EE, 0xE6F6F6,
  0xE6F6FF, 0xE69C00, 0xE69C08, 0xE69C10,
  0xE69C18, 0xFFA400, 0xEEAC10, 0xE6AC18,
  0xF6A420, 0xFFAC20, 0xE6A431, 0xF6B441,
  0xE6C573, 0xEECD94, 0xE6D59C, 0xE6D5A4,
  0xF6D5A4, 0xEEE6D5, 0xEEE6DE, 0xDEEEFF,
  0xEEF6FF, 0xE69408, 0xEE9C18, 0xF6A400,
  0xE6A418, 0xEEA418, 0xF6A418, 0xEEA420,
  0xFFAC29, 0xE6AC31, 0xEEAC39, 0xE6B431,
  0xEEAC41, 0xE6B44A, 0xF6BD41, 0xE6BD62,
  0xFFC552, 0xE6C56A, 0xF6C562, 0xE6C57B,
  0xFFC573, 0xFFCD73, 0xE6CD94, 0xE6CD9C,
  0xF6D59C, 0xDED5B4, 0xE6CDAC, 0xEEDEA4,
  0xEED5AC, 0xE6DEAC, 0xF6DEA4, 0xEEDEB4,
  0xE6DEC5, 0xF6EEC5, 0xEEEED5, 0xEEEEDE,
  0xF6F6DE, 0xFFF6DE, 0xDEEEF6, 0xDEF6F6,
  0xE6E6E6, 0xE6EEE6, 0xEEEEE6, 0xE6E6EE,
  0xEEEEEE, 0xE6EEFF, 0xE6FFF6
};

static GUI_CONST_STORAGE GUI_LOGPALETTE _Palunconnection = {
  115,  // Number of entries
  0,    // No transparency
  &_Colorsunconnection[0]
};

static GUI_CONST_STORAGE unsigned char _acunconnection[] = {
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x69, 0x17, 0x0E, 0x37, 0x1A, 0x00, 0x1A, 0x37, 0x0E, 0x17, 0x68, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x22, 0x3D, 0x32, 0x09, 0x10, 0x1B, 0x12, 0x52, 0x12, 0x1B, 0x10, 0x09, 0x32, 0x3D, 0x22, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x0B, 0x3A, 0x0C, 0x27, 0x06, 0x05, 0x05, 0x15, 0x15, 0x15, 0x05, 0x05, 0x06, 0x27, 0x0C, 0x3A, 0x0B, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x1F, 0x23, 0x04, 0x06, 0x05, 0x14, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x15, 0x2B, 0x06, 0x04, 0x23, 0x1F, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x0B, 0x33, 0x19, 0x05, 0x14, 0x0D, 0x13, 0x0F, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x06, 0x02, 0x05, 0x1D, 0x19, 0x33, 0x0B, 0x01, 0x01,
  0x01, 0x01, 0x28, 0x04, 0x1D, 0x02, 0x06, 0x1E, 0x5F, 0x07, 0x13, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x14, 0x1D, 0x04, 0x28, 0x01, 0x01,
  0x01, 0x16, 0x0C, 0x06, 0x05, 0x0F, 0x64, 0x0A, 0x0A, 0x6D, 0x07, 0x1C, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x05, 0x06, 0x0C, 0x16, 0x01,
  0x2E, 0x24, 0x4A, 0x58, 0x0D, 0x53, 0x20, 0x16, 0x56, 0x0A, 0x21, 0x07, 0x1C, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x2B, 0x38, 0x24, 0x2E,
  0x17, 0x09, 0x54, 0x2B, 0x47, 0x4F, 0x20, 0x60, 0x35, 0x07, 0x2F, 0x21, 0x07, 0x0D, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x06, 0x15, 0x06, 0x09, 0x5C,
  0x0E, 0x46, 0x59, 0x27, 0x31, 0x03, 0x5A, 0x18, 0x2C, 0x2A, 0x42, 0x0A, 0x41, 0x0F, 0x0F, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x05, 0x10, 0x0E,
  0x25, 0x12, 0x3B, 0x31, 0x00, 0x03, 0x03, 0x3E, 0x18, 0x6F, 0x6C, 0x44, 0x6E, 0x0D, 0x13, 0x06, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x05, 0x1B, 0x25,
  0x11, 0x36, 0x11, 0x00, 0x00, 0x00, 0x03, 0x03, 0x3F, 0x20, 0x43, 0x0A, 0x70, 0x0F, 0x39, 0x13, 0x0D, 0x02, 0x02, 0x02, 0x02, 0x02, 0x05, 0x12, 0x1A,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x03, 0x03, 0x57, 0x61, 0x5E, 0x30, 0x66, 0x17, 0x63, 0x07, 0x13, 0x02, 0x02, 0x02, 0x02, 0x14, 0x3B, 0x00,
  0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x04, 0x3C, 0x0A, 0x44, 0x0A, 0x21, 0x07, 0x1C, 0x02, 0x02, 0x02, 0x05, 0x12, 0x00,
  0x26, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x51, 0x43, 0x71, 0x42, 0x30, 0x2F, 0x07, 0x1C, 0x02, 0x02, 0x05, 0x1B, 0x25,
  0x29, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0E, 0x20, 0x6A, 0x2A, 0x07, 0x0A, 0x21, 0x07, 0x0D, 0x02, 0x05, 0x10, 0x0E,
  0x62, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x3E, 0x18, 0x2C, 0x35, 0x2A, 0x72, 0x41, 0x0F, 0x14, 0x02, 0x09, 0x2C,
  0x2E, 0x49, 0x03, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x03, 0x3F, 0x18, 0x17, 0x39, 0x67, 0x2F, 0x06, 0x05, 0x38, 0x24, 0x2D,
  0x01, 0x1E, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x03, 0x04, 0x5B, 0x18, 0x6B, 0x30, 0x1E, 0x06, 0x06, 0x0C, 0x16, 0x01,
  0x01, 0x01, 0x50, 0x09, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x03, 0x3C, 0x5D, 0x55, 0x06, 0x1D, 0x04, 0x28, 0x01, 0x01,
  0x01, 0x01, 0x0B, 0x34, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x04, 0x4C, 0x05, 0x10, 0x45, 0x65, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x1F, 0x34, 0x04, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x36, 0x06, 0x19, 0x23, 0x1F, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x0B, 0x4E, 0x09, 0x03, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x19, 0x09, 0x4D, 0x0B, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x22, 0x1E, 0x4B, 0x04, 0x04, 0x03, 0x00, 0x00, 0x00, 0x03, 0x04, 0x04, 0x48, 0x16, 0x22, 0x01, 0x01, 0x01, 0x01, 0x01,
  0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x2D, 0x40, 0x29, 0x26, 0x11, 0x00, 0x11, 0x26, 0x29, 0x40, 0x2D, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01
};

GUI_CONST_STORAGE GUI_BITMAP bmunconnection = {
  25, // xSize
  25, // ySize
  25, // BytesPerLine
  8, // BitsPerPixel
  _acunconnection,  // Pointer to picture data (indices)
  &_Palunconnection   // Pointer to palette
};

/*************************** End of file ****************************/

