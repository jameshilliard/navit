/* vim: set tabstop=4 expandtab: */
/**
 * Navit, a modular navigation system.
 * Copyright (C) 2005-2017 Navit Team
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

/*
   This plugin is able to drive a ssd1306 OLED i2c screen
   */

#include <math.h>
#include <stdio.h>
#include <glib.h>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include "config.h"
#include <navit/item.h>
#include <navit/xmlconfig.h>
#include <navit/main.h>
#include <navit/debug.h>
#include <navit/map.h>
#include <navit/navit.h>
#include <navit/callback.h>
#include <navit/file.h>
#include <navit/plugin.h>
#include <navit/event.h>
#include <navit/command.h>
#include <navit/config_.h>
#include "graphics.h"
#include "color.h"
#include "vehicle.h"
#include "transform.h"
#include "track.h"
#include "vehicleprofile.h"
#include "roadprofile.h"

#include <linux/i2c-dev.h>
#include <linux/unistd.h>	/* for _syscallX macros/related stuff */
#include <linux/kernel.h>	/* for struct sysinfo */
#include <sys/sysinfo.h>

//#include "ArduiPi_OLED.h"
//#include "Adafruit_GFX.h"
//#include "Adafruit_SSD1306.h"

extern char *version;

// Arduino Compatible type
typedef uint8_t boolean;

#define swap(a, b) { int16_t t = a; a = b; b = t; }

class Adafruit_GFX {
 public:

  //Adafruit_GFX();
  // i have no idea why we have to formally call the constructor. kinda sux
  void constructor(int16_t w, int16_t h);
  virtual ~Adafruit_GFX(){};
  // this must be defined by the subclass
  virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0;
  virtual void invertDisplay(boolean i);

	// the printf function
	void printf( const char * format, ...);
	void vprintf( const char * format, va_list ap);
	void print( const char * string) ;

  // these are 'generic' drawing functions, so we can share them!
  virtual uint16_t drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, uint16_t pattern=0xffff);
  virtual void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
  virtual void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
  virtual void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t pattern=0xffff);
  virtual void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t pattern=0xffff);

	void drawVerticalBargraph(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t percent) ;
	void drawHorizontalBargraph(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t percent) ;

  virtual void fillScreen(uint16_t color);

  void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint16_t pattern=0xffff);
  uint16_t drawCircleHelper(int16_t x0, int16_t y0,	int16_t r, uint8_t cornername, uint16_t color, uint16_t pattern=0xffff);
  void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint16_t pattern=0xffff);
  void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color, uint16_t pattern=0xffff);

  void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint16_t pattern=0xffff);
  void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint16_t pattern=0xffff);
  void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,  int16_t radius, uint16_t color, uint16_t pattern=0xffff);
  void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h,  int16_t radius, uint16_t color, uint16_t pattern=0xffff);

  void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
  void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
  virtual size_t write(uint8_t);

  void setCursor(int16_t x, int16_t y);
  void setTextColor(uint16_t c);
  void setTextColor(uint16_t c, uint16_t bg);
  void setTextSize(uint8_t s);
  void setTextWrap(boolean w);

  int16_t height(void);
  int16_t width(void);

 protected:
  int16_t  WIDTH, HEIGHT;   // this is the 'raw' display w/h - never changes
  int16_t  _width, _height; // dependent on rotation
  int16_t  cursor_x, cursor_y;
  uint16_t textcolor, textbgcolor;
  uint8_t  textsize;
  uint8_t  rotation;
  boolean  wrap; // If set, 'wrap' text at right edge of display
};

// standard ascii 5x7 font

const unsigned char  font[] = {
        0x00, 0x00, 0x00, 0x00, 0x00,   
	0x3E, 0x5B, 0x4F, 0x5B, 0x3E, 	
	0x3E, 0x6B, 0x4F, 0x6B, 0x3E, 	
	0x1C, 0x3E, 0x7C, 0x3E, 0x1C, 
	0x18, 0x3C, 0x7E, 0x3C, 0x18, 
	0x1C, 0x57, 0x7D, 0x57, 0x1C, 
	0x1C, 0x5E, 0x7F, 0x5E, 0x1C, 
	0x00, 0x18, 0x3C, 0x18, 0x00, 
	0xFF, 0xE7, 0xC3, 0xE7, 0xFF, 
	0x00, 0x18, 0x24, 0x18, 0x00, 
	0xFF, 0xE7, 0xDB, 0xE7, 0xFF, 
	0x30, 0x48, 0x3A, 0x06, 0x0E, 
	0x26, 0x29, 0x79, 0x29, 0x26, 
	0x40, 0x7F, 0x05, 0x05, 0x07, 
	0x40, 0x7F, 0x05, 0x25, 0x3F, 
	0x5A, 0x3C, 0xE7, 0x3C, 0x5A, 
	0x7F, 0x3E, 0x1C, 0x1C, 0x08, 
	0x08, 0x1C, 0x1C, 0x3E, 0x7F, 
	0x14, 0x22, 0x7F, 0x22, 0x14, 
	0x5F, 0x5F, 0x00, 0x5F, 0x5F, 
	0x06, 0x09, 0x7F, 0x01, 0x7F, 
	0x00, 0x66, 0x89, 0x95, 0x6A, 
	0x60, 0x60, 0x60, 0x60, 0x60, 
	0x94, 0xA2, 0xFF, 0xA2, 0x94, 
	0x08, 0x04, 0x7E, 0x04, 0x08, 
	0x10, 0x20, 0x7E, 0x20, 0x10, 
	0x08, 0x08, 0x2A, 0x1C, 0x08, 
	0x08, 0x1C, 0x2A, 0x08, 0x08, 
	0x1E, 0x10, 0x10, 0x10, 0x10, 
	0x0C, 0x1E, 0x0C, 0x1E, 0x0C, 
	0x30, 0x38, 0x3E, 0x38, 0x30, 
	0x06, 0x0E, 0x3E, 0x0E, 0x06, 
	0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x5F, 0x00, 0x00, 
	0x00, 0x07, 0x00, 0x07, 0x00, 
	0x14, 0x7F, 0x14, 0x7F, 0x14, 
	0x24, 0x2A, 0x7F, 0x2A, 0x12, 
	0x23, 0x13, 0x08, 0x64, 0x62, 
	0x36, 0x49, 0x56, 0x20, 0x50, 
	0x00, 0x08, 0x07, 0x03, 0x00, 
	0x00, 0x1C, 0x22, 0x41, 0x00, 
	0x00, 0x41, 0x22, 0x1C, 0x00, 
	0x2A, 0x1C, 0x7F, 0x1C, 0x2A, 
	0x08, 0x08, 0x3E, 0x08, 0x08, 
	0x00, 0x80, 0x70, 0x30, 0x00, 
	0x08, 0x08, 0x08, 0x08, 0x08, 
	0x00, 0x00, 0x60, 0x60, 0x00, 
	0x20, 0x10, 0x08, 0x04, 0x02, 
	0x3E, 0x51, 0x49, 0x45, 0x3E, 
	0x00, 0x42, 0x7F, 0x40, 0x00, 
	0x72, 0x49, 0x49, 0x49, 0x46, 
	0x21, 0x41, 0x49, 0x4D, 0x33, 
	0x18, 0x14, 0x12, 0x7F, 0x10, 
	0x27, 0x45, 0x45, 0x45, 0x39, 
	0x3C, 0x4A, 0x49, 0x49, 0x31, 
	0x41, 0x21, 0x11, 0x09, 0x07, 
	0x36, 0x49, 0x49, 0x49, 0x36, 
	0x46, 0x49, 0x49, 0x29, 0x1E, 
	0x00, 0x00, 0x14, 0x00, 0x00, 
	0x00, 0x40, 0x34, 0x00, 0x00, 
	0x00, 0x08, 0x14, 0x22, 0x41, 
	0x14, 0x14, 0x14, 0x14, 0x14, 
	0x00, 0x41, 0x22, 0x14, 0x08, 
	0x02, 0x01, 0x59, 0x09, 0x06, 
	0x3E, 0x41, 0x5D, 0x59, 0x4E, 
	0x7C, 0x12, 0x11, 0x12, 0x7C, 
	0x7F, 0x49, 0x49, 0x49, 0x36, 
	0x3E, 0x41, 0x41, 0x41, 0x22, 
	0x7F, 0x41, 0x41, 0x41, 0x3E, 
	0x7F, 0x49, 0x49, 0x49, 0x41, 
	0x7F, 0x09, 0x09, 0x09, 0x01, 
	0x3E, 0x41, 0x41, 0x51, 0x73, 
	0x7F, 0x08, 0x08, 0x08, 0x7F, 
	0x00, 0x41, 0x7F, 0x41, 0x00, 
	0x20, 0x40, 0x41, 0x3F, 0x01, 
	0x7F, 0x08, 0x14, 0x22, 0x41, 
	0x7F, 0x40, 0x40, 0x40, 0x40, 
	0x7F, 0x02, 0x1C, 0x02, 0x7F, 
	0x7F, 0x04, 0x08, 0x10, 0x7F, 
	0x3E, 0x41, 0x41, 0x41, 0x3E, 
	0x7F, 0x09, 0x09, 0x09, 0x06, 
	0x3E, 0x41, 0x51, 0x21, 0x5E, 
	0x7F, 0x09, 0x19, 0x29, 0x46, 
	0x26, 0x49, 0x49, 0x49, 0x32, 
	0x03, 0x01, 0x7F, 0x01, 0x03, 
	0x3F, 0x40, 0x40, 0x40, 0x3F, 
	0x1F, 0x20, 0x40, 0x20, 0x1F, 
	0x3F, 0x40, 0x38, 0x40, 0x3F, 
	0x63, 0x14, 0x08, 0x14, 0x63, 
	0x03, 0x04, 0x78, 0x04, 0x03, 
	0x61, 0x59, 0x49, 0x4D, 0x43, 
	0x00, 0x7F, 0x41, 0x41, 0x41, 
	0x02, 0x04, 0x08, 0x10, 0x20, 
	0x00, 0x41, 0x41, 0x41, 0x7F, 
	0x04, 0x02, 0x01, 0x02, 0x04, 
	0x40, 0x40, 0x40, 0x40, 0x40, 
	0x00, 0x03, 0x07, 0x08, 0x00, 
	0x20, 0x54, 0x54, 0x78, 0x40, 
	0x7F, 0x28, 0x44, 0x44, 0x38, 
	0x38, 0x44, 0x44, 0x44, 0x28, 
	0x38, 0x44, 0x44, 0x28, 0x7F, 
	0x38, 0x54, 0x54, 0x54, 0x18, 
	0x00, 0x08, 0x7E, 0x09, 0x02, 
	0x18, 0xA4, 0xA4, 0x9C, 0x78, 
	0x7F, 0x08, 0x04, 0x04, 0x78, 
	0x00, 0x44, 0x7D, 0x40, 0x00, 
	0x20, 0x40, 0x40, 0x3D, 0x00, 
	0x7F, 0x10, 0x28, 0x44, 0x00, 
	0x00, 0x41, 0x7F, 0x40, 0x00, 
	0x7C, 0x04, 0x78, 0x04, 0x78, 
	0x7C, 0x08, 0x04, 0x04, 0x78, 
	0x38, 0x44, 0x44, 0x44, 0x38, 
	0xFC, 0x18, 0x24, 0x24, 0x18, 
	0x18, 0x24, 0x24, 0x18, 0xFC, 
	0x7C, 0x08, 0x04, 0x04, 0x08, 
	0x48, 0x54, 0x54, 0x54, 0x24, 
	0x04, 0x04, 0x3F, 0x44, 0x24, 
	0x3C, 0x40, 0x40, 0x20, 0x7C, 
	0x1C, 0x20, 0x40, 0x20, 0x1C, 
	0x3C, 0x40, 0x30, 0x40, 0x3C, 
	0x44, 0x28, 0x10, 0x28, 0x44, 
	0x4C, 0x90, 0x90, 0x90, 0x7C, 
	0x44, 0x64, 0x54, 0x4C, 0x44, 
	0x00, 0x08, 0x36, 0x41, 0x00, 
	0x00, 0x00, 0x77, 0x00, 0x00, 
	0x00, 0x41, 0x36, 0x08, 0x00, 
	0x02, 0x01, 0x02, 0x04, 0x02, 
	0x3C, 0x26, 0x23, 0x26, 0x3C, 
	0x1E, 0xA1, 0xA1, 0x61, 0x12, 
	0x3A, 0x40, 0x40, 0x20, 0x7A, 
	0x38, 0x54, 0x54, 0x55, 0x59, 
	0x21, 0x55, 0x55, 0x79, 0x41, 
	0x21, 0x54, 0x54, 0x78, 0x41, 
	0x21, 0x55, 0x54, 0x78, 0x40, 
	0x20, 0x54, 0x55, 0x79, 0x40, 
	0x0C, 0x1E, 0x52, 0x72, 0x12, 
	0x39, 0x55, 0x55, 0x55, 0x59, 
	0x39, 0x54, 0x54, 0x54, 0x59, 
	0x39, 0x55, 0x54, 0x54, 0x58, 
	0x00, 0x00, 0x45, 0x7C, 0x41, 
	0x00, 0x02, 0x45, 0x7D, 0x42, 
	0x00, 0x01, 0x45, 0x7C, 0x40, 
	0xF0, 0x29, 0x24, 0x29, 0xF0, 
	0xF0, 0x28, 0x25, 0x28, 0xF0, 
	0x7C, 0x54, 0x55, 0x45, 0x00, 
	0x20, 0x54, 0x54, 0x7C, 0x54, 
	0x7C, 0x0A, 0x09, 0x7F, 0x49, 
	0x32, 0x49, 0x49, 0x49, 0x32, 
	0x32, 0x48, 0x48, 0x48, 0x32, 
	0x32, 0x4A, 0x48, 0x48, 0x30, 
	0x3A, 0x41, 0x41, 0x21, 0x7A, 
	0x3A, 0x42, 0x40, 0x20, 0x78, 
	0x00, 0x9D, 0xA0, 0xA0, 0x7D, 
	0x39, 0x44, 0x44, 0x44, 0x39, 
	0x3D, 0x40, 0x40, 0x40, 0x3D, 
	0x3C, 0x24, 0xFF, 0x24, 0x24, 
	0x48, 0x7E, 0x49, 0x43, 0x66, 
	0x2B, 0x2F, 0xFC, 0x2F, 0x2B, 
	0xFF, 0x09, 0x29, 0xF6, 0x20, 
	0xC0, 0x88, 0x7E, 0x09, 0x03, 
	0x20, 0x54, 0x54, 0x79, 0x41, 
	0x00, 0x00, 0x44, 0x7D, 0x41, 
	0x30, 0x48, 0x48, 0x4A, 0x32, 
	0x38, 0x40, 0x40, 0x22, 0x7A, 
	0x00, 0x7A, 0x0A, 0x0A, 0x72, 
	0x7D, 0x0D, 0x19, 0x31, 0x7D, 
	0x26, 0x29, 0x29, 0x2F, 0x28, 
	0x26, 0x29, 0x29, 0x29, 0x26, 
	0x30, 0x48, 0x4D, 0x40, 0x20, 
	0x38, 0x08, 0x08, 0x08, 0x08, 
	0x08, 0x08, 0x08, 0x08, 0x38, 
	0x2F, 0x10, 0xC8, 0xAC, 0xBA, 
	0x2F, 0x10, 0x28, 0x34, 0xFA, 
	0x00, 0x00, 0x7B, 0x00, 0x00, 
	0x08, 0x14, 0x2A, 0x14, 0x22, 
	0x22, 0x14, 0x2A, 0x14, 0x08, 
	0xAA, 0x00, 0x55, 0x00, 0xAA, 
	0xAA, 0x55, 0xAA, 0x55, 0xAA, 
	0x00, 0x00, 0x00, 0xFF, 0x00, 
	0x10, 0x10, 0x10, 0xFF, 0x00, 
	0x14, 0x14, 0x14, 0xFF, 0x00, 
	0x10, 0x10, 0xFF, 0x00, 0xFF, 
	0x10, 0x10, 0xF0, 0x10, 0xF0, 
	0x14, 0x14, 0x14, 0xFC, 0x00, 
	0x14, 0x14, 0xF7, 0x00, 0xFF, 
	0x00, 0x00, 0xFF, 0x00, 0xFF, 
	0x14, 0x14, 0xF4, 0x04, 0xFC, 
	0x14, 0x14, 0x17, 0x10, 0x1F, 
	0x10, 0x10, 0x1F, 0x10, 0x1F, 
	0x14, 0x14, 0x14, 0x1F, 0x00, 
	0x10, 0x10, 0x10, 0xF0, 0x00, 
	0x00, 0x00, 0x00, 0x1F, 0x10, 
	0x10, 0x10, 0x10, 0x1F, 0x10, 
	0x10, 0x10, 0x10, 0xF0, 0x10, 
	0x00, 0x00, 0x00, 0xFF, 0x10, 
	0x10, 0x10, 0x10, 0x10, 0x10, 
	0x10, 0x10, 0x10, 0xFF, 0x10, 
	0x00, 0x00, 0x00, 0xFF, 0x14, 
	0x00, 0x00, 0xFF, 0x00, 0xFF, 
	0x00, 0x00, 0x1F, 0x10, 0x17, 
	0x00, 0x00, 0xFC, 0x04, 0xF4, 
	0x14, 0x14, 0x17, 0x10, 0x17, 
	0x14, 0x14, 0xF4, 0x04, 0xF4, 
	0x00, 0x00, 0xFF, 0x00, 0xF7, 
	0x14, 0x14, 0x14, 0x14, 0x14, 
	0x14, 0x14, 0xF7, 0x00, 0xF7, 
	0x14, 0x14, 0x14, 0x17, 0x14, 
	0x10, 0x10, 0x1F, 0x10, 0x1F, 
	0x14, 0x14, 0x14, 0xF4, 0x14, 
	0x10, 0x10, 0xF0, 0x10, 0xF0, 
	0x00, 0x00, 0x1F, 0x10, 0x1F, 
	0x00, 0x00, 0x00, 0x1F, 0x14, 
	0x00, 0x00, 0x00, 0xFC, 0x14, 
	0x00, 0x00, 0xF0, 0x10, 0xF0, 
	0x10, 0x10, 0xFF, 0x10, 0xFF, 
	0x14, 0x14, 0x14, 0xFF, 0x14, 
	0x10, 0x10, 0x10, 0x1F, 0x00, 
	0x00, 0x00, 0x00, 0xF0, 0x10, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 
	0xFF, 0xFF, 0xFF, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xFF, 0xFF, 
	0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 
	0x38, 0x44, 0x44, 0x38, 0x44, 
	0x7C, 0x2A, 0x2A, 0x3E, 0x14, 
	0x7E, 0x02, 0x02, 0x06, 0x06, 
	0x02, 0x7E, 0x02, 0x7E, 0x02, 
	0x63, 0x55, 0x49, 0x41, 0x63, 
	0x38, 0x44, 0x44, 0x3C, 0x04, 
	0x40, 0x7E, 0x20, 0x1E, 0x20, 
	0x06, 0x02, 0x7E, 0x02, 0x02, 
	0x99, 0xA5, 0xE7, 0xA5, 0x99, 
	0x1C, 0x2A, 0x49, 0x2A, 0x1C, 
	0x4C, 0x72, 0x01, 0x72, 0x4C, 
	0x30, 0x4A, 0x4D, 0x4D, 0x30, 
	0x30, 0x48, 0x78, 0x48, 0x30, 
	0xBC, 0x62, 0x5A, 0x46, 0x3D, 
	0x3E, 0x49, 0x49, 0x49, 0x00, 
	0x7E, 0x01, 0x01, 0x01, 0x7E, 
	0x2A, 0x2A, 0x2A, 0x2A, 0x2A, 
	0x44, 0x44, 0x5F, 0x44, 0x44, 
	0x40, 0x51, 0x4A, 0x44, 0x40, 
	0x40, 0x44, 0x4A, 0x51, 0x40, 
	0x00, 0x00, 0xFF, 0x01, 0x03, 
	0xE0, 0x80, 0xFF, 0x00, 0x00, 
	0x08, 0x08, 0x6B, 0x6B, 0x08,
	0x36, 0x12, 0x36, 0x24, 0x36, 
	0x06, 0x0F, 0x09, 0x0F, 0x06, 
	0x00, 0x00, 0x18, 0x18, 0x00, 
	0x00, 0x00, 0x10, 0x10, 0x00, 
	0x30, 0x40, 0xFF, 0x01, 0x01, 
	0x00, 0x1F, 0x01, 0x01, 0x1E, 
	0x00, 0x19, 0x1D, 0x17, 0x12, 
	0x00, 0x3C, 0x3C, 0x3C, 0x3C, 
	0x00, 0x00, 0x00, 0x00, 0x00, 
};

void Adafruit_GFX::constructor(int16_t w, int16_t h) 
{
	_width = WIDTH = w;
	_height = HEIGHT = h;

	rotation = 0;		
	cursor_y = cursor_x = 0;
	textsize = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap = true;
}

// the printf function
void Adafruit_GFX::printf( const char * format, ...){
	va_list args;
	va_start (args, format);

	this->vprintf(format, args);

	va_end (args);
}

void Adafruit_GFX::vprintf( const char * format, va_list args){
	char buffer[64];
	char * p = buffer;
	int n;
	vsnprintf (buffer, sizeof(buffer)-1, format, args);
	n = strlen(buffer);
		
	while (*p != 0 && n-->0)
		write ( (uint8_t) *p++);
}

// the print function
void Adafruit_GFX::print( const char * string) 
{

	const char * p = string;
	int n = strlen(string);
	
	while (*p != 0 && n-->0)
	{
		write ( (uint8_t) *p++);
	}

}


// draw a circle outline
void Adafruit_GFX::drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint16_t pattern){
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawPixel(x0, y0+r, color);
	drawPixel(x0, y0-r, color);
	drawPixel(x0+r, y0, color);
	drawPixel(x0-r, y0, color);

	while (x<y) 
	{
		if (f >= 0) 
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		
		x++;
		ddF_x += 2;
		f += ddF_x;
	
		if(pattern & 1){
			drawPixel(x0 + x, y0 + y, color);
			drawPixel(x0 - x, y0 + y, color);
			drawPixel(x0 + x, y0 - y, color);
			drawPixel(x0 - x, y0 - y, color);
			drawPixel(x0 + y, y0 + x, color);
			drawPixel(x0 - y, y0 + x, color);
			drawPixel(x0 + y, y0 - x, color);
			drawPixel(x0 - y, y0 - x, color);
		}
		
		uint16_t carry = pattern & 1;
		pattern >>= 1;
		if(carry)
			pattern |= 0x8000;		
	}
}

uint16_t Adafruit_GFX::drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color, uint16_t pattern) 
{
	int16_t f		 = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x		 = 0;
	int16_t y		 = r;

	while (x<y) 
	{
		if (f >= 0) 
		{
			y--;
			ddF_y += 2;
			f		 += ddF_y;
		}
		
		x++;
		ddF_x += 2;
		f		 += ddF_x;

		if(pattern & 1){
			if(cornername & 0x4){
				drawPixel(x0 + x, y0 + y, color);
				drawPixel(x0 + y, y0 + x, color);
			} 
			if(cornername & 0x2){
				drawPixel(x0 + x, y0 - y, color);
				drawPixel(x0 + y, y0 - x, color);
			}
			if(cornername & 0x8){
				drawPixel(x0 - y, y0 + x, color);
				drawPixel(x0 - x, y0 + y, color);
			}
			if(cornername & 0x1){
				drawPixel(x0 - y, y0 - x, color);
				drawPixel(x0 - x, y0 - y, color);
			}
		}
		uint16_t carry = pattern & 1;
		pattern >>= 1;
		if(carry)
			pattern |= 0x8000;		
	}

	return pattern;
}

void Adafruit_GFX::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color, uint16_t pattern){
	if(pattern == 0xffff)
		drawFastVLine(x0, y0-r, 2*r+1, color);
	else
		pattern = drawLine(x0, y0-r, x0, y0-r + 2*r+1, color, pattern);
	fillCircleHelper(x0, y0, r, 3, 0, color, pattern);
}

// used to do circles and roundrects!
void Adafruit_GFX::fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color, uint16_t pattern){
	int16_t f		 = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x		 = 0;
	int16_t y		 = r;
	while(x<y){
		if (f >= 0){
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		
		x++;
		ddF_x += 2;
		f += ddF_x;

		if(cornername & 0x1){
			if(pattern == 0xffff){
				drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
				drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
			} else {
				pattern = drawLine(x0+x, y0-y, x0+x, y0-y + 2*y+1+delta, color, pattern);
				pattern = drawLine(x0+y, y0-x, x0+y, y0-x + 2*x+1+delta, color, pattern);
			}
		}
		
		if (cornername & 0x2){
			if(pattern == 0xffff){
				drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
				drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
			} else {
				pattern = drawLine(x0-x, y0-y, x0-x, y0-y + 2*y+1+delta, color, pattern);
				pattern = drawLine(x0-y, y0-x, x0-y, y0-x + 2*x+1+delta, color, pattern);
			}
		}
	}
}

// bresenham's algorithm - thx wikpedia
uint16_t Adafruit_GFX::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color, uint16_t pattern){
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if(steep){
		swap(x0, y0);
		swap(x1, y1);
	}

	if(x0 > x1){
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if(y0 < y1){
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0<=x1; x0++){
		if(pattern & 1){
			if (steep)
				drawPixel(y0, x0, color);
			else 
				drawPixel(x0, y0, color);
		}
		err -= dy;
		
		if(err < 0){
			y0 += ystep;
			err += dx;
		}
		uint16_t carry = pattern & 1;
		pattern >>= 1;
		if(carry)
			pattern |= 0x8000;
	}

	return pattern;
}


// draw a rectangle
void Adafruit_GFX::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t pattern){
	if(pattern == 0xffff){
		drawFastHLine(x, y, w, color);
		drawFastHLine(x, y+h-1, w, color);
		drawFastVLine(x, y, h, color);
		drawFastVLine(x+w-1, y, h, color);
	} else {
		pattern = drawLine(x,y,     x+w,y,   color, pattern);
		pattern = drawLine(x+w,y,   x+w,y+h, color, pattern);
		pattern = drawLine(x+w,y+h, x,y+h,   color, pattern);
		drawLine(x,y+h,   x,y,     color, pattern);
	}
}

void Adafruit_GFX::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) 
{
	// stupidest version - update in subclasses if desired!
	drawLine(x, y, x, y+h-1, color);
}


void Adafruit_GFX::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) 
{
	// stupidest version - update in subclasses if desired!
	drawLine(x, y, x+w-1, y, color);
}

void Adafruit_GFX::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t pattern) 
{
	// stupidest version - update in subclasses if desired!
	for (int16_t i=x; i<x+w; i++){
		if(pattern == 0xffff)
			drawFastVLine(i, y, h, color);
		else
			pattern = drawLine(i,y, i,y+h, color, pattern);
	}
}

// draw a vertical bargraph and fill it with percent value (0%..100%)
void Adafruit_GFX::drawVerticalBargraph(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t percent) 
{
	uint16_t vsize;

	// Create rectangle
	drawRect(x,y, w, h, color)	;

	// Do not do stupid job
	if ( h>2 && w>2 )
	{
		// calculate pixel size of bargraph
		vsize = ( ( h - 2) * percent ) / 100	;

		// Fill it from bottom (0%) to top (100%)
		fillRect(x+1,y+1 + (( h-2)-vsize), w - 2, vsize, color);
	}
}

// draw a horizontal bargraph and fill it with percent value (0%..100%)
void Adafruit_GFX::drawHorizontalBargraph(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, uint16_t percent) 
{
	uint16_t hsize;

	// Create rectangle
	drawRect(x,y, w, h, color)	;

	// Do not do stupid job
	if ( h>2 && w>2 )
	{
		// calculate pixel size of bargraph
		hsize = ( ( w - 2) * percent ) / 100	;

		// Fill it from left (0%) to right (100%)
		fillRect(x+1 , y+1 , hsize, h - 2, color);
	}
}


void Adafruit_GFX::fillScreen(uint16_t color) 
{
	fillRect(0, 0, _width, _height, color);
}

// draw a rounded rectangle!
void Adafruit_GFX::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, uint16_t pattern){

	if(pattern == 0xffff){
		// smarter version
		drawFastHLine(x+r	, y		, w-2*r, color); // Top
		drawFastHLine(x+r	, y+h-1, w-2*r, color); // Bottom
		drawFastVLine(	x		, y+r	, h-2*r, color); // Left
		drawFastVLine(	x+w-1, y+r	, h-2*r, color); // Right

		// draw four corners
		drawCircleHelper(x+r		, y+r		, r, 1, color);
		drawCircleHelper(x+w-r-1, y+r		, r, 2, color);
		drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
		drawCircleHelper(x+r		, y+h-r-1, r, 8, color);
	} else {
		pattern = drawLine(x+r, y, x+w-r, y,   color, pattern);
		pattern = drawCircleHelper(x+w-r-1, y+r		, r, 2, color, pattern);
		pattern = drawLine(x+w-1, y+r, x+w-1, y+h-r,   color, pattern);
		pattern = drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color, pattern);
		pattern = drawLine(x+r, y+h-1, x+w-r, y+h-1, color, pattern);
		pattern = drawCircleHelper(x+r, y+h-r-1, r, 8, color, pattern);
		pattern = drawLine(x, y+r, x, y+h-r,   color, pattern);
		pattern = drawCircleHelper(x+r, y+r, r, 1, color, pattern);
	}
}

// fill a rounded rectangle!
void Adafruit_GFX::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, uint16_t pattern){
	// smarter version
	fillRect(x+r, y, w-2*r, h, color, pattern);

	// draw four corners
	fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color, pattern);
	fillCircleHelper(x+r		, y+r, r, 2, h-2*r-1, color, pattern);
}

// draw a triangle!
void Adafruit_GFX::drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint16_t pattern) 
{
	pattern = drawLine(x0, y0, x1, y1, color, pattern);
	pattern = drawLine(x1, y1, x2, y2, color, pattern);
	drawLine(x2, y2, x0, y0, color, pattern);
}

// fill a triangle!
void Adafruit_GFX::fillTriangle ( int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color, uint16_t pattern){

	int16_t a, b, y, last;

	// Sort coordinates by Y order (y2 >= y1 >= y0)
	if(y0 > y1) 
		swap(y0, y1); swap(x0, x1);
	if(y1 > y2) 
		swap(y2, y1); swap(x2, x1);
	if(y0 > y1) 
		swap(y0, y1); swap(x0, x1);

	if(y0 == y2){ // Handle awkward all-on-same-line case as its own thing
		a = b = x0;
		if(x1 < a)
			a = x1;
		else if(x1 > b)
			b = x1;
		if(x2 < a)
			a = x2;
		else if(x2 > b)
			b = x2;

		if(pattern == 0xffff)
			drawFastHLine(a, y0, b-a+1, color);
		else
			drawLine(a, y0, b+1, y0, color, pattern);
		return;
	}

	int16_t
		dx01 = x1 - x0,
		dy01 = y1 - y0,
		dx02 = x2 - x0,
		dy02 = y2 - y0,
		dx12 = x2 - x1,
		dy12 = y2 - y1,
		sa	 = 0,
		sb	 = 0;

	// For upper part of triangle, find scanline crossings for segments
	// 0-1 and 0-2.	If y1=y2 (flat-bottomed triangle), the scanline y1
	// is included here (and second loop will be skipped, avoiding a /0
	// error there), otherwise scanline y1 is skipped here and handled
	// in the second loop...which also avoids a /0 error here if y0=y1
	// (flat-topped triangle).
	if(y1 == y2) 
		last = y1;	 // Include y1 scanline
	else
		last = y1-1; // Skip it

	for(y=y0; y<=last; y++){
		a	 = x0 + sa / dy01;
		b	 = x0 + sb / dy02;
		sa += dx01;
		sb += dx02;
		/* longhand:
		a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
		b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
		if(a > b) 
			swap(a,b);
			
		if(pattern == 0xffff)
			drawFastHLine(a, y, b-a+1, color);
		else
			pattern = drawLine(a, y, b+1, y, color, pattern);
	}

	// For lower part of triangle, find scanline crossings for segments
	// 0-2 and 1-2.	This loop is skipped if y1=y2.
	sa = dx12 * (y - y1);
	sb = dx02 * (y - y0);
	for(; y<=y2; y++) 
	{
		a	 = x1 + sa / dy12;
		b	 = x0 + sb / dy02;
		sa += dx12;
		sb += dx02;
		/* longhand:
		a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
		b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
		*/
		if(a > b) 
			swap(a,b);
			
		if(pattern == 0xffff)
			drawFastHLine(a, y, b-a+1, color);
		else
			pattern = drawLine(a, y, b+1, y, color, pattern);
	}
}

void Adafruit_GFX::drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color) 
{

	int16_t i, j, byteWidth = (w + 7) / 8;

	for(j=0; j<h; j++) 
	{
		for(i=0; i<w; i++ ) 
		{
			if( *(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) 
			{
				drawPixel(x+i, y+j, color);
			}
		}
	}
}


size_t Adafruit_GFX::write(uint8_t c) 
{
	if (c == '\n') 
	{
		cursor_y += textsize*8;
		cursor_x = 0;
	} 
	else if (c == '\r') 
	{
		// skip em
	} 
	else 
	{
		drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
		cursor_x += textsize*6;
		
		if (wrap && (cursor_x > (_width - textsize*6))) 
		{
			cursor_y += textsize*8;
			cursor_x = 0;
		}
	}
	return 1;
}

// draw a character
void Adafruit_GFX::drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) 
{

	if((x >= _width)						|| // Clip right
		 (y >= _height)					 || // Clip bottom
		 ((x + 5 * size - 1) < 0) || // Clip left
		 ((y + 8 * size - 1) < 0))	 // Clip top
		return;

	for (int8_t i=0; i<6; i++ ) 
	{
		uint8_t line;
		if (i == 5) 
			line = 0x0;
		else 
			//line = pgm_read_byte(font+(c*5)+i);
			line = font[(c*5)+i];
		for (int8_t j = 0; j<8; j++) 
		{
			if (line & 0x1) 
			{
				if (size == 1) // default size
					drawPixel(x+i, y+j, color);
				else 
				{	// big size
					fillRect(x+(i*size), y+(j*size), size, size, color);
				} 
			} 
			else if (bg != color) 
			{
				if (size == 1) // default size
					drawPixel(x+i, y+j, bg);
				else 
				{	// big size
					fillRect(x+i*size, y+j*size, size, size, bg);
				} 	
			}
			
			line >>= 1;
		}
	}
}

void Adafruit_GFX::setCursor(int16_t x, int16_t y) 
{
	cursor_x = x;
	cursor_y = y;
}


void Adafruit_GFX::setTextSize(uint8_t s) 
{
	textsize = (s > 0) ? s : 1;
}


void Adafruit_GFX::setTextColor(uint16_t c) 
{
	textcolor = c;
	textbgcolor = c; 
	// for 'transparent' background, we'll set the bg 
	// to the same as fg instead of using a flag
}

void Adafruit_GFX::setTextColor(uint16_t c, uint16_t b) 
{
	 textcolor = c;
	 textbgcolor = b; 
 }

void Adafruit_GFX::setTextWrap(boolean w) 
{
	wrap = w;
}

void Adafruit_GFX::invertDisplay(boolean i) 
{
	// do nothing, can be subclassed
}


// return the size of the display which depends on the rotation!
int16_t Adafruit_GFX::width(void) 
{ 
	return _width; 
}
 
int16_t Adafruit_GFX::height(void) 
{ 
	return _height; 
}

class ArduiPi_OLED : public Adafruit_GFX
{
 public:
  ArduiPi_OLED();

  boolean init(uint8_t OLED_TYPE, const char *device);

  boolean oled_is_spi_proto(uint8_t OLED_TYPE); /* to know protocol before init */
  boolean select_oled(uint8_t OLED_TYPE, const char *device);

  void begin(void);
  void close(void);

  void sendCommand(uint8_t c);
  void sendCommand(uint8_t c0, uint8_t c1);
  void sendCommand(uint8_t c0, uint8_t c1, uint8_t c2);
  void sendData(uint8_t c);

  void clearDisplay(void);
  void setGrayLevel(uint8_t grayLevel);
  void setBrightness(uint8_t Brightness);
  void invertDisplay(uint8_t i);
  void display();
  void OnOff( boolean );
  void Flip( boolean );

  boolean SaveToPBM(const char *);

  void setSeedTextXY(unsigned char Row, unsigned char Column);
  void putSeedChar(char C);
  void putSeedString(const char *String);


  int16_t getOledWidth(void);
  int16_t getOledHeight(void);

  void startscrollright(uint8_t start, uint8_t stop);
  void startscrollleft(uint8_t start, uint8_t stop);

  void startscrolldiagright(uint8_t start, uint8_t stop);
  void startscrolldiagleft(uint8_t start, uint8_t stop);
  void setHorizontalScrollProperties(bool direction,uint8_t startRow, uint8_t endRow,uint8_t startColumn, uint8_t endColumn, uint8_t scrollSpeed);
  void stopscroll(void);

  void drawPixel(int16_t x, int16_t y, uint16_t color);
  uint16_t getPixel(int16_t x, int16_t y);

  private:
  uint8_t *poledbuff; // Pointer to OLED data buffer in memory
  int8_t _i2c_addr, dc, rst, spi;
  int16_t oled_width, oled_height;
  int16_t oled_buff_size;
  uint8_t vcc_type;
  uint8_t oled_type;
  uint8_t grayH, grayL;

  inline boolean isI2C(void);
  inline boolean isSPI(void);


  //volatile uint8_t *dcport;
  //uint8_t dcpinmask;
};

// Oled supported display
enum { 
	OLED_ADAFRUIT_I2C_128x32 = 0,
	OLED_ADAFRUIT_I2C_128x64,
	OLED_SEEED_I2C_128x64,
	OLED_SEEED_I2C_96x96,
	OLED_SH1106_I2C_128x64,
	OLED_LAST_OLED
};

// Arduino Compatible Macro
#define _BV(bit) (1 << (bit))

#define BLACK 0
#define WHITE 1


/*=========================================================================
    SSDxxxx Common Displays
    -----------------------------------------------------------------------
    Common values to all displays
=========================================================================*/

//#define SSD_Command_Mode      0x80  /* DC bit is 0 */ Seeed set C0 to 1 why ?
#define SSD_Command_Mode      0x00  /* C0 and DC bit are 0         */
#define SSD_Data_Mode         0x40  /* C0 bit is 0 and DC bit is 1 */

#define SSD_Set_Segment_Remap   0xA0
#define SSD_Inverse_Display     0xA7
#define SSD_Set_Muliplex_Ratio  0xA8

#define SSD_Display_Off         0xAE
#define SSD_Display_On          0xAF

#define SSD_Set_ContrastLevel 0x81

#define SSD_External_Vcc      0x01
#define SSD_Internal_Vcc      0x02

#define SSD_Set_Column_Address  0x21
#define SSD_Set_Page_Address    0x22

#define SSD_Activate_Scroll   0x2F
#define SSD_Deactivate_Scroll 0x2E

#define SSD_Right_Horizontal_Scroll   0x26
#define SSD_Left_Horizontal_Scroll    0x27


#define Scroll_Left           0x00
#define Scroll_Right          0x01

#define Scroll_2Frames    0x07
#define Scroll_3Frames    0x04
#define Scroll_4Frames    0x05
#define Scroll_5Frames    0x00
#define Scroll_25Frames   0x06
#define Scroll_64Frames   0x01
#define Scroll_128Frames  0x02
#define Scroll_256Frames  0x03

#define VERTICAL_MODE           01
#define PAGE_MODE               01
#define HORIZONTAL_MODE         02


/*=========================================================================
    SSD1306 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
=========================================================================*/

#define SSD1306_Entire_Display_Resume 0xA4
#define SSD1306_Entire_Display_On     0xA5

#define SSD1306_Normal_Display  0xA6

#define SSD1306_Set_Display_Offset      0xD3
#define SSD1306_Set_Com_Pins        0xDA
#define SSD1306_Set_Vcomh_Deselect_Level      0xDB
#define SSD1306_Set_Display_Clock_Div 0xD5
#define SSD1306_Set_Precharge_Period    0xD9
#define SSD1306_Set_Lower_Column_Start_Address        0x00
#define SSD1306_Set_Higher_Column_Start_Address       0x10
#define SSD1306_Set_Start_Line      0x40
#define SSD1306_Set_Memory_Mode     0x20
#define SSD1306_Set_Seg_Direction_Nomal 0xA0
#define SSD1306_Set_Seg_Direction_Rever 0xA1
#define SSD1306_Set_Com_Output_Scan_Direction_Normal  0xC0
#define SSD1306_Set_Com_Output_Scan_Direction_Remap   0xC8
#define SSD1306_Charge_Pump_Setting 0x8D

// Scrolling #defines
#define SSD1306_SET_VERTICAL_SCROLL_AREA              0xA3
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL  0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL   0x2A

/*=========================================================================
    SSD1308 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
=========================================================================*/
#define SSD1308_Normal_Display  0xA6

/*=========================================================================
    SSD1327 Displays
    -----------------------------------------------------------------------
    The driver is used in Seeed 96x96 display
=========================================================================*/
#define SSD1327_Set_Display_Start_Line  0xA1
#define SSD1327_Set_Display_Offset      0xA2
#define SSD1327_Normal_Display      0xA4
#define SSD1327_Set_Display_Clock_Div 0xB3
#define SSD1327_Set_Command_Lock    0xFD
#define SSD1327_Set_Column_Address  0x15
#define SSD1327_Set_Row_Address     0x75

#define SSD1327_Set_Row_Address     0x75

/*=========================================================================
    SH1106 Displays
    -----------------------------------------------------------------------
    The driver is used in multiple displays (128x64, 128x32, etc.).
=========================================================================*/
#define SH1106_Set_Page_Address 0xB0

#define DEV_TYPE_I2C 1
#define DEV_TYPE_SPI 2

#ifdef BANANAPI
#	define I2C_DEV "/dev/i2c-2"
#else
#	define I2C_DEV "/dev/i2c-0"
#endif
#define I2C_ADDR 0x3c

#define DEV_TYPE 1

#if defined (__cplusplus)
extern "C" {
#endif

    int lcd_dev_open(const char *dev);
    int lcd_dev_write(uint8_t* data, int len);
    void lcd_dev_close();

#if defined (__cplusplus)
}
#endif

int i2c_fd = -1;

int lcd_dev_open(const char *dev) {
 
    switch(DEV_TYPE) {
        
        case DEV_TYPE_I2C:
            i2c_fd = open(dev, O_RDWR);
            if(ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR) < 0) {
                printf("I2C ioctl error : %s\r\n", strerror(errno));
                return 0;
            }
            return 1;
            
        case DEV_TYPE_SPI:
            
            break;
        
    }
    
    return 0;
    
}

int lcd_dev_write(uint8_t* data, int len) {
    
    switch(DEV_TYPE) {
        
        case DEV_TYPE_I2C:
            if(write(i2c_fd, data, len) != len) {
                printf("I2C write error : %s\r\n", strerror(errno));
                    return 0;
            }
            return len;
            
        case DEV_TYPE_SPI:
            
            break;
        
    }

    return 0;
    
}

void lcd_dev_close() {
 
    switch(DEV_TYPE) {
        
        case DEV_TYPE_I2C:
            close(i2c_fd);
            return;
            
        case DEV_TYPE_SPI:
            break;
    }
    
}

  // 8x8 Font ASCII 32 - 127 Implemented
// Users can modify this to support more characters(glyphs)
// BasicFont is placed in code memory.

// This font can be freely used without any restriction(It is placed in public domain)
const unsigned char seedfont[][8] =
{
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x5F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00},
  {0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00},
  {0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00},
  {0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00},
  {0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00},
  {0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x22,0x1C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x2A,0x1C,0x2A,0x08,0x00,0x00},
  {0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00},
  {0x00,0xA0,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00},
  {0x00,0x60,0x60,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00},
  {0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00},
  {0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00},
  {0x00,0x62,0x51,0x49,0x49,0x46,0x00,0x00},
  {0x00,0x22,0x41,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00},
  {0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00},
  {0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00},
  {0x00,0x01,0x71,0x09,0x05,0x03,0x00,0x00},
  {0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00},
  {0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00},
  {0x00,0x00,0xAC,0x6C,0x00,0x00,0x00,0x00},
  {0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00},
  {0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00},
  {0x00,0x41,0x22,0x14,0x08,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00},
  {0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00},
  {0x00,0x7E,0x09,0x09,0x09,0x7E,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x36,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x22,0x1C,0x00,0x00},
  {0x00,0x7F,0x49,0x49,0x49,0x41,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x51,0x72,0x00,0x00},
  {0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00},
  {0x00,0x41,0x7F,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00},
  {0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00},
  {0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00},
  {0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00},
  {0x00,0x7F,0x04,0x08,0x10,0x7F,0x00,0x00},
  {0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00},
  {0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00},
  {0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00},
  {0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00},
  {0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00},
  {0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00},
  {0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00},
  {0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00},
  {0x00,0x3F,0x40,0x38,0x40,0x3F,0x00,0x00},
  {0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00},
  {0x00,0x03,0x04,0x78,0x04,0x03,0x00,0x00},
  {0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00},
  {0x00,0x7F,0x41,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00},
  {0x00,0x41,0x41,0x7F,0x00,0x00,0x00,0x00},
  {0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00},
  {0x00,0x80,0x80,0x80,0x80,0x80,0x00,0x00},
  {0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00},
  {0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00},
  {0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x28,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00},
  {0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00},
  {0x00,0x08,0x7E,0x09,0x02,0x00,0x00,0x00},
  {0x00,0x18,0xA4,0xA4,0xA4,0x7C,0x00,0x00},
  {0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00},
  {0x00,0x00,0x7D,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x80,0x84,0x7D,0x00,0x00,0x00,0x00},
  {0x00,0x7F,0x10,0x28,0x44,0x00,0x00,0x00},
  {0x00,0x41,0x7F,0x40,0x00,0x00,0x00,0x00},
  {0x00,0x7C,0x04,0x18,0x04,0x78,0x00,0x00},
  {0x00,0x7C,0x08,0x04,0x7C,0x00,0x00,0x00},
  {0x00,0x38,0x44,0x44,0x38,0x00,0x00,0x00},
  {0x00,0xFC,0x24,0x24,0x18,0x00,0x00,0x00},
  {0x00,0x18,0x24,0x24,0xFC,0x00,0x00,0x00},
  {0x00,0x00,0x7C,0x08,0x04,0x00,0x00,0x00},
  {0x00,0x48,0x54,0x54,0x24,0x00,0x00,0x00},
  {0x00,0x04,0x7F,0x44,0x00,0x00,0x00,0x00},
  {0x00,0x3C,0x40,0x40,0x7C,0x00,0x00,0x00},
  {0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00},
  {0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00},
  {0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00},
  {0x00,0x1C,0xA0,0xA0,0x7C,0x00,0x00,0x00},
  {0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00},
  {0x00,0x08,0x36,0x41,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x7F,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x41,0x36,0x08,0x00,0x00,0x00,0x00},
  {0x00,0x02,0x01,0x01,0x02,0x01,0x00,0x00},
  {0x00,0x02,0x05,0x05,0x02,0x00,0x00,0x00} 
};

// the most basic function, set a single pixel
void ArduiPi_OLED::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
  uint8_t * p = poledbuff ;
  uint8_t c;
  
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

    /*
    // check rotation, move pixel around if necessary
  switch (getRotation()) 
  {
    case 1:
      swap(x, y);
      x = WIDTH - x - 1;
    break;
    
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
    break;
    
    case 3:
      swap(x, y);
      y = HEIGHT - y - 1;
    break;
  }  
*/
  if (oled_type == OLED_SEEED_I2C_96x96 )
  {
    // Get where to do the change in the buffer
    p = poledbuff + (x + (y/2)*oled_width );
    
    // Get old value to not touch the other nible
    c = *p;
    
    // We are on High nible ?
    if ( ((y/2)&1) == 1 )
    {
      c &= 0x0F;
      c|= (color==WHITE ? grayH:0x00) << 4;
    } 
    else
    {
      c &= 0xF0;
      c|= (color==WHITE ? grayL:0x00) ;
    } 
    
    // set new nible value leaving the other untouched
    *p = c; 
  }
  else
  {
    // Get where to do the change in the buffer
    p = poledbuff + (x + (y/8)*oled_width );
    
    // x is which column
    if (color == WHITE) 
      *p |=  _BV((y%8));  
    else
      *p &= ~_BV((y%8)); 
  }
}

// Get the value of a pixel
uint16_t ArduiPi_OLED::getPixel(int16_t x, int16_t y){
	uint8_t *p = poledbuff;
	if((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
		return 0;

	if(oled_type == OLED_SEEED_I2C_96x96){
		fputs("Not implemented\n", stderr);
		return 0;
	} else {
		p = poledbuff + (x + (y/8)*oled_width );
		return(!!(*p & _BV((y%8))));
	}
}

// Display instantiation
ArduiPi_OLED::ArduiPi_OLED() 
{
  // Init all var, and clean
  // Command I/O
  rst = 0 ;
  dc  = 0 ;
  spi =  0 ;
  
  // Lcd size
  oled_width  = 0;
  oled_height = 0;
  
  // Empty pointer to OLED buffer
  poledbuff = NULL;
}


// initializer for OLED Type
boolean ArduiPi_OLED::select_oled(uint8_t OLED_TYPE, const char *dev){
  // Default type
  oled_width  = 128;
  oled_height = 64;
  //_i2c_addr = 0x00;
  oled_type = OLED_TYPE;
  
  // default OLED are using internal boost VCC converter
  vcc_type = SSD_Internal_Vcc;

  // Oled supported display
  // Setup size and I2C address
  switch (OLED_TYPE)
  {
    case OLED_ADAFRUIT_I2C_128x32:
      oled_height = 32;
      //_i2c_addr = ADAFRUIT_I2C_ADDRESS;
    break;

    case OLED_ADAFRUIT_I2C_128x64:
      //_i2c_addr = ADAFRUIT_I2C_ADDRESS;
    break;
    
    case OLED_SEEED_I2C_128x64:
      //_i2c_addr = SEEED_I2C_ADDRESS ;
      vcc_type = SSD_External_Vcc;
    break;

    case OLED_SEEED_I2C_96x96:
      oled_width  = 96;
      oled_height = 96;
      //_i2c_addr = SEEED_I2C_ADDRESS ;
    break;
    
    case OLED_SH1106_I2C_128x64:
      //_i2c_addr = SH1106_I2C_ADDRESS;
    break;
    
    // houston, we have a problem
    default:
	  fputs("Unknown display", stderr);
      return false;
    break;
  }
  
  // Buffer size differ from OLED type, 1 pixel is one bit 
  // execpt for 96x96 seed, 1 pixel is 1 nible
  oled_buff_size = oled_width * oled_height ;
  
  if ( OLED_TYPE == OLED_SEEED_I2C_96x96 )
    oled_buff_size = oled_buff_size / 2 ;
  else
    oled_buff_size = oled_buff_size / 8;

  // De-Allocate memory for OLED buffer if any
  if (poledbuff)
    free(poledbuff);
    
  // Allocate memory for OLED buffer
  poledbuff = (uint8_t *) malloc ( oled_buff_size ); 
  
  if (!poledbuff){
  	perror("buffer malloc()");
    return false;
  }

  // Init IO
  if (!lcd_dev_open(dev)){
  	fputs("lcd_dev_open() failed\n", stderr);
    return false;
  }

  return true;
}

boolean ArduiPi_OLED::init(uint8_t oled_type, const char *dev){
    return select_oled(oled_type, dev);
}

void ArduiPi_OLED::close(void) 
{
  // De-Allocate memory for OLED buffer if any
  if (poledbuff)
    free(poledbuff);
    
  poledbuff = NULL;

  lcd_dev_close();
  
}

  
void ArduiPi_OLED::begin( void ) 
{
  uint8_t multiplex;
  uint8_t chargepump;
  uint8_t compins;
  uint8_t contrast;
  uint8_t precharge;
  
  constructor(oled_width, oled_height);
  
  // depends on OLED type configuration
  if (oled_height == 32)
  {
    multiplex = 0x1F;
    compins   = 0x02;
    contrast  = 0x8F;
  }
  else
  {
    if (oled_type == OLED_SEEED_I2C_96x96 )
    {
      multiplex = 0x5F;
      compins   = 0x12;
      contrast  = 0x53;
    }
    // So 128x64
    else
    {
      multiplex = 0x3F;
      compins   = 0x12;
      
      if (oled_type == OLED_SH1106_I2C_128x64)
        contrast = 0x80;
      else
        contrast = (vcc_type==SSD_External_Vcc?0x9F:0xCF);
    }
      
  }
  
  if (vcc_type == SSD_External_Vcc)
  {
    chargepump = 0x10; 
    precharge  = 0x22;
  }
  else
  {
    chargepump = 0x14; 
    precharge  = 0xF1;
  }
  
  if (oled_type == OLED_SEEED_I2C_96x96 )
    sendCommand(SSD1327_Set_Command_Lock, 0x12); // Unlock OLED driver IC MCU interface from entering command. i.e: Accept commands
  
  sendCommand(SSD_Display_Off);                    
  sendCommand(SSD_Set_Muliplex_Ratio, multiplex); 
  
  if (oled_type == OLED_SEEED_I2C_96x96 )
  {
    sendCommand(SSD1327_Set_Display_Clock_Div, 0x01); 
    sendCommand(SSD1327_Set_Display_Start_Line    , 0   ); 
    sendCommand(SSD1327_Set_Display_Offset, 96  ); 
    sendCommand(SSD_Set_Segment_Remap     , 0x46); 

    sendCommand(0xAB); // set vdd internal
    sendCommand(0x01); //
    
    sendCommand(0xB1); // Set Phase Length
    sendCommand(0X51); //
    
    sendCommand(0xB9); //
    
    sendCommand(0xBC); // set pre_charge voltage/VCOMH
    sendCommand(0x08); // (0x08);
    
    sendCommand(0xBE); // set VCOMH
    sendCommand(0X07); // (0x07);
    
    sendCommand(0xB6); // Set second pre-charge period
    sendCommand(0x01); //
    
    sendCommand(0xD5); // enable second precharge and enternal vsl
    sendCommand(0X62); // (0x62);

    // Set Normal Display Mode
    sendCommand(SSD1327_Normal_Display); 

    // Row Address
    // Start 0 End 95
    sendCommand(SSD1327_Set_Row_Address, 0, 95);

    // Column Address
    // Start from 8th Column of driver IC. This is 0th Column for OLED 
    // End at  (8 + 47)th column. Each Column has 2 pixels(segments)
    sendCommand(SSD1327_Set_Column_Address, 8, 0x37 );  

    // Map to horizontal mode
    sendCommand(0xA0); // remap to
    sendCommand(0x46); // Vertical mode

    // Init gray level for text. Default:Brightest White
    grayH= 0xF0;
    grayL= 0x0F;
  }
  else if (oled_type == OLED_SH1106_I2C_128x64)
  {
    sendCommand(SSD1306_Set_Lower_Column_Start_Address|0x02); /*set lower column address*/
    sendCommand(SSD1306_Set_Higher_Column_Start_Address);     /*set higher column address*/
    sendCommand(SSD1306_Set_Start_Line);                      /*set display start line*/
    sendCommand(SH1106_Set_Page_Address);    /*set page address*/
    sendCommand(SSD_Set_Segment_Remap|0x01); /*set segment remap*/
    sendCommand(SSD1306_Normal_Display);     /*normal / reverse*/
    sendCommand(0xad);    /*set charge pump enable*/
    sendCommand(0x8b);    /*external VCC   */
    sendCommand(0x30);    /*0X30---0X33  set VPP   9V liangdu!!!!*/
    sendCommand(SSD1306_Set_Com_Output_Scan_Direction_Remap);    /*Com scan direction*/
    sendCommand(SSD1306_Set_Display_Offset);    /*set display offset*/
    sendCommand(0x00);   /*   0x20  */
    sendCommand(SSD1306_Set_Display_Clock_Div);    /*set osc division*/
    sendCommand(0x80);
    sendCommand(SSD1306_Set_Precharge_Period);    /*set pre-charge period*/
    sendCommand(0x1f);    /*0x22*/
    sendCommand(SSD1306_Set_Com_Pins);    /*set COM pins*/
    sendCommand(0x12);
    sendCommand(SSD1306_Set_Vcomh_Deselect_Level);    /*set vcomh*/
    sendCommand(0x40);
  }
  else
  {
    sendCommand(SSD1306_Charge_Pump_Setting, chargepump); 
    sendCommand(SSD1306_Set_Memory_Mode, 0x00);              // 0x20 0x0 act like ks0108
    sendCommand(SSD1306_Set_Display_Clock_Div, 0x80);      // 0xD5 + the suggested ratio 0x80
    sendCommand(SSD1306_Set_Display_Offset, 0x00);        // no offset
    sendCommand(SSD1306_Set_Start_Line | 0x0);            // line #0
    // use this two commands to flip display
    sendCommand(SSD_Set_Segment_Remap | 0x1);
    sendCommand(SSD1306_Set_Com_Output_Scan_Direction_Remap);
    
    sendCommand(SSD1306_Set_Com_Pins, compins);  
    sendCommand(SSD1306_Set_Precharge_Period, precharge); 
    sendCommand(SSD1306_Set_Vcomh_Deselect_Level, 0x40); // 0x40 -> unknown value in datasheet
    sendCommand(SSD1306_Entire_Display_Resume);    
    sendCommand(SSD1306_Normal_Display);         // 0xA6

    // Reset to default value in case of 
    // no reset pin available on OLED, 
    sendCommand( SSD_Set_Column_Address, 0, 127 ); 
    sendCommand( SSD_Set_Page_Address, 0,   7 ); 
  }

  sendCommand(SSD_Set_ContrastLevel, contrast);

  stopscroll();
  
  // Empty uninitialized buffer
  clearDisplay();
  
  // turn on oled panel
  sendCommand(SSD_Display_On);              
  
  // wait 100ms
  usleep(100000);
}

// Turn the display On and Off
void ArduiPi_OLED::OnOff( boolean i ){
	sendCommand(i ? SSD_Display_On : SSD_Display_Off);
}

// Turn the display upside-down 
void ArduiPi_OLED::Flip( boolean i ){
	if(i){
		sendCommand( SSD1306_Set_Seg_Direction_Nomal );
		sendCommand( SSD1306_Set_Com_Output_Scan_Direction_Normal );
	} else {
		sendCommand( SSD1306_Set_Seg_Direction_Rever );
		sendCommand( SSD1306_Set_Com_Output_Scan_Direction_Remap );
	}
}

// Only valid for Seeed 96x96 OLED
void ArduiPi_OLED::setGrayLevel(uint8_t grayLevel)
{
    grayH = (grayLevel << 4) & 0xF0;
    grayL =  grayLevel & 0x0F;
}

void ArduiPi_OLED::setSeedTextXY(unsigned char Row, unsigned char Column)
{
    //Column Address
    sendCommand(0x15);             /* Set Column Address */
    sendCommand(0x08+(Column*4));  /* Start Column: Start from 8 */
    sendCommand(0x37);             /* End Column */
    // Row Address
    sendCommand(0x75);             /* Set Row Address */
    sendCommand(0x00+(Row*8));     /* Start Row*/
    sendCommand(0x07+(Row*8));     /* End Row*/
}

void ArduiPi_OLED::putSeedChar(char C)
{
    if(C < 32) //Ignore non-printable ASCII characters. This can be modified for multilingual font.
    {
        C=' '; //Space
    } 

    for(int i=0;i<8;i=i+2)
    {
        for(int j=0;j<8;j++)
        {
            // Character is constructed two pixel at a time using vertical mode from the default 8x8 font
            char c=0x00;
            char bit1=( seedfont[(int)C-32][(int)i]   >> j) & 0x01;  
            char bit2=( seedfont[(int)C-32][(int)i+1] >> j) & 0x01;

           // Each bit is changed to a nibble
            c|=(bit1)?grayH:0x00;
            c|=(bit2)?grayL:0x00;
            sendData(c);
        }
    }
}

void ArduiPi_OLED::putSeedString(const char *String)
{
    unsigned char i=0;
    while(String[i])
    {
        putSeedChar( String[i]);     
        i++;
    }
}

void ArduiPi_OLED::setBrightness(uint8_t Brightness)
{
   sendCommand(SSD_Set_ContrastLevel);
   sendCommand(Brightness);
}


void ArduiPi_OLED::invertDisplay(uint8_t i) 
{
  if (i) 
    sendCommand(SSD_Inverse_Display);
  else 
    sendCommand(oled_type==OLED_SEEED_I2C_96x96 ? SSD1327_Normal_Display : SSD1306_Normal_Display);
}

void ArduiPi_OLED::sendCommand(uint8_t c) 
{ 
	uint8_t buff[2] ;
    
	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode ; 
	buff[1] = c;
    
	// Write Data on I2C
	lcd_dev_write(buff, 2);
}

void ArduiPi_OLED::sendCommand(uint8_t c0, uint8_t c1) 
{ 
	uint8_t buff[3] ;
	buff[1] = c0;
	buff[2] = c1;

 	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode ;

	// Write Data on I2C
	lcd_dev_write(buff, 3) ;
}

void ArduiPi_OLED::sendCommand(uint8_t c0, uint8_t c1, uint8_t c2) 
{ 
	uint8_t buff[4] ;
    
	buff[1] = c0;
	buff[2] = c1;
	buff[3] = c2;

	// Clear D/C to switch to command mode
	buff[0] = SSD_Command_Mode; 

	// Write Data on I2C
	lcd_dev_write(buff, 4)  ;
}


// startscrollright
// Activate a right handed scroll for rows start throufastI2Cwritegh stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void ArduiPi_OLED::startscrollright(uint8_t start, uint8_t stop)
{
  sendCommand(SSD_Right_Horizontal_Scroll);
  sendCommand(0X00);
  sendCommand(start);
  sendCommand(0X00);
  sendCommand(stop);
  sendCommand(0X01);
  sendCommand(0XFF);
  sendCommand(SSD_Activate_Scroll);
}

// startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void ArduiPi_OLED::startscrollleft(uint8_t start, uint8_t stop)
{
  sendCommand(SSD_Left_Horizontal_Scroll);
  sendCommand(0X00);
  sendCommand(start);
  sendCommand(0X00);
  sendCommand(stop);
  sendCommand(0X01);
  sendCommand(0XFF);
  sendCommand(SSD_Activate_Scroll);
}

// startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void ArduiPi_OLED::startscrolldiagright(uint8_t start, uint8_t stop)
{
  sendCommand(SSD1306_SET_VERTICAL_SCROLL_AREA);  
  sendCommand(0X00);
  sendCommand(oled_height);
  sendCommand(SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
  sendCommand(0X00);
  sendCommand(start);
  sendCommand(0X00);
  sendCommand(stop);
  sendCommand(0X01);
  sendCommand(SSD_Activate_Scroll);
}

// startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F) 
void ArduiPi_OLED::startscrolldiagleft(uint8_t start, uint8_t stop)
{
  sendCommand(SSD1306_SET_VERTICAL_SCROLL_AREA);  
  sendCommand(0X00);
  sendCommand(oled_height);
  sendCommand(SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
  sendCommand(0X00);
  sendCommand(start);
  sendCommand(0X00);
  sendCommand(stop);
  sendCommand(0X01);
  sendCommand(SSD_Activate_Scroll);
}


void ArduiPi_OLED::setHorizontalScrollProperties(bool direction,uint8_t startRow, uint8_t endRow,uint8_t startColumn, uint8_t endColumn, uint8_t scrollSpeed)
{
  if(Scroll_Right == direction)
  {
      //Scroll Right
      sendCommand(SSD_Left_Horizontal_Scroll);
  }
  else
  {
      //Scroll Left  
      sendCommand(SSD_Right_Horizontal_Scroll);
  }
  sendCommand(0x00);       //Dummmy byte
  sendCommand(startRow);
  sendCommand(scrollSpeed);
  sendCommand(endRow);
  sendCommand(startColumn+8);
  sendCommand(endColumn+8);
  sendCommand(0x00);      //Dummmy byte

}

void ArduiPi_OLED::stopscroll(void)
{
  sendCommand(SSD_Deactivate_Scroll);
}

void ArduiPi_OLED::sendData(uint8_t c) 
{
	uint8_t buff[2] ;
    
	// Setup D/C to switch to data mode
	buff[0] = SSD_Data_Mode; 
	buff[1] = c;

	// Write on i2c
	lcd_dev_write( buff, 2) ;
}

void ArduiPi_OLED::display(void) 
{

  if (oled_type == OLED_SEEED_I2C_96x96 )
  {
    sendCommand(SSD1327_Set_Row_Address   , 0x00, 0x5F);
    sendCommand(SSD1327_Set_Column_Address, 0x08, 0x37);
  }
  else
  {
    sendCommand(SSD1306_Set_Lower_Column_Start_Address  | 0x0); // low col = 0
    sendCommand(SSD1306_Set_Higher_Column_Start_Address | 0x0); // hi col = 0
    sendCommand(SSD1306_Set_Start_Line  | 0x0); // line #0
  }

  uint16_t i=0 ;
  
  // pointer to OLED data buffer
  uint8_t * p = poledbuff;

  uint8_t buff[17] ;
  uint8_t x ;

  // Setup D/C to switch to data mode
  buff[0] = SSD_Data_Mode; 
    
  if (oled_type == OLED_SH1106_I2C_128x64)
  {
    for (uint8_t k=0; k<8; k++) 
    {
      sendCommand(0xB0+k);//set page addressSSD_Data_Mode;
      sendCommand(0x02) ;//set lower column address
      sendCommand(0x10) ;//set higher column address

     for( i=0; i<8; i++)
     {
        for (x=1; x<=16; x++) 
          buff[x] = *p++;

        lcd_dev_write(buff, 17);
      }
    }
  }
  else
  {
    // loop trough all OLED buffer and 
    // send a bunch of 16 data byte in one xmission
    for ( i=0; i<oled_buff_size; i+=16 ) 
    {
      for (x=1; x<=16; x++) 
        buff[x] = *p++;

      lcd_dev_write(buff, 17);
    }
  }
}

// Save the display's buffer as a PBM
boolean ArduiPi_OLED::SaveToPBM(const char *fn){
	FILE *f = fopen(fn, "w");
	int16_t x,y;

	if(!f)
		return false;

	fprintf(f, "P1\n%d %d\n", width(), height());

	
	for(y=0;y<height();y++){
		for(x=0;x<width();x++){
			if(!(x%8))
				fputc(' ',f);
			fputc(getPixel(x,y) ? '0':'1', f);
		}
		fputc('\n',f);
	}

	fclose(f);
	return true;
}

// clear everything (in the buffer)
void ArduiPi_OLED::clearDisplay(void) 
{
  memset(poledbuff, 0, oled_buff_size);
}

ArduiPi_OLED display;

struct ssd1306
{
  struct navit *nav;
  struct callback *callback;
  struct event_idle *idle;
  int frames;
  long tick;
  int fps;
};


long
get_uptime ()
{
  struct sysinfo s_info;
  int error = sysinfo (&s_info);
  if (error != 0)
    {
      printf ("code error = %d\n", error);
    }
  return s_info.uptime;
}

// Configuration Pin for ArduiPi board
#define RPI_V2_GPIO_P1_22 25 /* Version 2, Pin P1-22 */
#define OLED_I2C_RESET RPI_V2_GPIO_P1_22 /* GPIO 25 pin 12 */

static void
ssd1306_idle (gpointer data)	// (struct ssd1306 *ssd1306)
{
  struct ssd1306 *ssd1306 = (struct ssd1306 *) data;

  display.clearDisplay ();
  display.setTextSize (1);
  display.setTextColor (WHITE);
  display.setCursor (0, 0);

  char snum[32];

  struct attr attr, attr2, vattr;
  struct attr position_attr, position_fix_attr;
  struct attr_iter *iter;
  enum projection pro;
  struct coord c1;
  long current_tick = get_uptime ();

  struct attr speed_attr;
  double speed = -1;
  int strength = -1;

  iter = navit_attr_iter_new(NULL);
  if (navit_get_attr (ssd1306->nav, attr_vehicle, &attr, iter)
      && !navit_get_attr (ssd1306->nav, attr_vehicle, &attr2, iter))
    {
      vehicle_get_attr (attr.u.vehicle, attr_name, &vattr, NULL);
      navit_attr_iter_destroy (iter);


      if (vehicle_get_attr
	  (attr.u.vehicle, attr_position_fix_type, &position_fix_attr, NULL))
	{
	  switch (position_fix_attr.u.num)
	    {
	    case 1:
	    case 2:
	      strength = 2;
	      if (vehicle_get_attr
		  (attr.u.vehicle, attr_position_sats_used,
		   &position_fix_attr, NULL))
		{
		  if (position_fix_attr.u.num >= 3)
		    strength = position_fix_attr.u.num - 1;
		  if (strength > 5)
		    strength = 5;
		  if (strength > 3)
		    {
		      if (vehicle_get_attr
			  (attr.u.vehicle, attr_position_hdop,
			   &position_fix_attr, NULL))
			{
			  if (*position_fix_attr.u.numd > 2.0 && strength > 4)
			    strength = 4;
			  if (*position_fix_attr.u.numd > 4.0 && strength > 3)
			    strength = 3;
			}
		    }
		}
	      break;
	    default:
	      strength = 1;
	    }
	}
        display.drawLine( 0, display.height()-1, strength * 5, display.height()-1, WHITE);

      if (strength > 2)
	{
	  if (vehicle_get_attr
	      (attr.u.vehicle, attr_position_coord_geo, &position_attr, NULL))
	    {
	      pro = position_attr.u.pcoord->pro;
	      transform_from_geo (pro, position_attr.u.coord_geo, &c1);
	      dbg (lvl_error, "%f %f\n", position_attr.u.coord_geo->lat,
		   position_attr.u.coord_geo->lng);
	      sprintf (snum, "%f %f\n", position_attr.u.coord_geo->lat,
		       position_attr.u.coord_geo->lng);
	      display.printf (snum);

	      vehicle_get_attr (attr.u.vehicle, attr_position_speed,
				  &speed_attr, NULL);
	      speed = *speed_attr.u.numd;
	      display.setTextSize (2);
	      dbg (lvl_error, "speed : %0.0f (%f)\n", speed, speed);
	    }
	  else
	    {
	      dbg (lvl_error, "nope\n");
	      navit_attr_iter_destroy (iter);
	    }

	  struct tracking *tracking = NULL;
	  double routespeed = -1;
	  int *flags;
	  struct attr maxspeed_attr;
	  tracking = navit_get_tracking (ssd1306->nav);

	  if (tracking)
	    {
	      struct item *item;

	      flags = tracking_get_current_flags (tracking);
	      if (flags && (*flags & AF_SPEED_LIMIT)
		  && tracking_get_attr (tracking, attr_maxspeed,
					&maxspeed_attr, NULL))
		{
		  routespeed = maxspeed_attr.u.num;
		}
	      item = tracking_get_current_item (tracking);
	      if (routespeed == -1 and item)
		{

		  struct vehicleprofile *prof =
		    navit_get_vehicleprofile (ssd1306->nav);
		  struct roadprofile *rprof = NULL;
		  if (prof)
		    rprof = vehicleprofile_get_roadprofile (prof, item->type);
		  if (rprof)
		    {
		      if (rprof->maxspeed != 0)
			routespeed = rprof->maxspeed;
		    }
		}

	      sprintf (snum, "%0.0f", speed);
	      display.setCursor(10,12);
	      if (routespeed == -1)
		{
		  display.printf (snum);
		  display.setTextColor (BLACK, WHITE);
                  display.setCursor(64,8);
		  display.printf (" ?? ");
		  display.setTextColor (WHITE, BLACK);

		}
	      else
		{
		  dbg (lvl_error, "route speed : %0.0f\n", routespeed);
		  if (speed > routespeed)
		    {
		      display.setTextColor (BLACK, WHITE);	// 'inverted' text
		      display.printf (snum);
		      display.setTextColor (WHITE, BLACK);	// 'inverted' text
		    }
		  else
		    {
		      display.printf (snum);
		    }
                  display.setCursor(64,12);
		  sprintf (snum, " %0.0f", routespeed);
		  display.printf (snum);
		}
	    }

          display.drawLine( display.width() - 1 - ssd1306->fps , display.height()-1, display.width() - 1, display.height()-1, WHITE);

	  if (current_tick == ssd1306->tick)
	    {
	      ssd1306->frames++;
	    }
	  else
	    {
	      ssd1306->fps = ssd1306->frames;
	      ssd1306->frames = 0;
	      ssd1306->tick = current_tick;
	    }
	}
      else
	{
	  if (current_tick % 2)
	    {
	      display.printf ("Waiting for GPS...");
	    }
	  else
	    {
	      display.printf ("Waiting for GPS.");
	    }

		if ( strength > 0 )
			display.fillRect(36, display.height()-9, 6, 8, WHITE);
		else
			display.drawRect(36, display.height()-9, 6, 8, WHITE);
		if ( strength > 1 )
			display.fillRect(44, display.height()-13, 6, 12, WHITE);
		else
			display.fillRect(44, display.height()-13, 6, 12, WHITE);
		if ( strength > 2 )
			display.drawRect(52, display.height()-17, 6, 16, WHITE);
		else
			display.fillRect(52, display.height()-17, 6, 16, WHITE);
		if ( strength > 3 )
			display.fillRect(60, display.height()-21, 6, 20, WHITE);
		else
			display.drawRect(60, display.height()-21, 6, 20, WHITE);
	}
      display.display ();
      // g_timeout_add (10, ssd1306_idle, ssd1306);
    }
}

static void
ssd1306_init (struct ssd1306 *ssd1306, struct navit *nav)
{
  dbg (lvl_error, "here\n");
  // ssd1306->callback=callback_new_1(callback_cast(ssd1306_idle), ssd1306);
  // ssd1306->idle=event_add_timeout(100, 1, ssd1306->callback);
  // ssd1306->idle=event_add_idle(125, ssd1306->callback);
  ssd1306_idle (ssd1306);
  // g_timeout_add (10, ssd1306_idle, ssd1306);
}

/**
 * @brief	Creates the ssd1306 and set some default properties
 * @param[in]	nav	- the navit object
 *              meth    - the osd_methods
 * 		attrs	- pointer to the attributes
 *
 * @return	nothing
 *
 * Creates the ssd1306 OSD and set some default properties
 *
 */
static struct osd_priv *
ssd1306_new (struct navit *nav, struct osd_methods *meth, struct attr **attrs)
{
  dbg (lvl_error, "here\n");
  if (!display.init (OLED_I2C_RESET, "/dev/i2c-2"))
    exit (-1);

  display.begin ();
  display.clearDisplay ();

  display.setTextSize (1);
  display.setTextColor (WHITE);
  display.setCursor (0, 0);
  display.print ("Navit\n");
  // display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.printf (version);
  display.setTextSize (2);
  display.setTextColor (WHITE);
  display.display ();
  struct ssd1306 *ssd1306 = g_new0 (struct ssd1306, 1);
  ssd1306->nav = nav;
  ssd1306->frames = 0;
  ssd1306->fps = 0;
  ssd1306->tick = get_uptime ();
  navit_add_callback (nav,
		      callback_new_attr_1 (callback_cast (ssd1306_init),
					   attr_graphics_ready, ssd1306));
  return (struct osd_priv *) ssd1306;
}

/**
 * @brief	The plugin entry point
 *
 * @return	nothing
 *
 * The plugin entry point
 *
 */
void
plugin_init (void)
{
  dbg (lvl_error, "here\n");

  plugin_register_category_osd ("ssd1306", ssd1306_new);
}
