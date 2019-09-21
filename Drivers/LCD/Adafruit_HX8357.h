/*!
 * @file Adafruit_HX8357.h
 *
 * This is the documentation for Adafruit's ILI9341 driver for the Arduino
 * platform.
 *
 * This library works with the Adafruit 3.5" TFT 320x480 + Touchscreen Breakout
 *    http://www.adafruit.com/products/2050
 *
 * Adafruit TFT FeatherWing - 3.5" 480x320 Touchscreen for Feathers
 *    https://www.adafruit.com/product/3651
 *
 * These displays use SPI to communicate. This requires 4 pins (MOSI,
 * SCK, select, data/command) and optionally a reset pin. Hardware SPI
 * or 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * This library depends on <a href="https://github.com/adafruit/Adafruit_GFX">
 * Adafruit_GFX</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * BSD license, all text here must be included in any redistribution.
 */
 
/* Modified version for use in STM32H7 project by
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */ 

#ifndef _ADAFRUIT_HX8357_H
#define _ADAFRUIT_HX8357_H

#include "stm32h7xx_hal.h"
#include "Adafruit_SPITFT.h"

#define HX8357D                    0xD
#define HX8357B                    0xB

#define HX8357_TFTWIDTH            320
#define HX8357_TFTHEIGHT           480

#define HX8357_NOP                0x00
#define HX8357_SWRESET            0x01
#define HX8357_RDDID              0x04
#define HX8357_RDDST              0x09

#define HX8357_RDPOWMODE          0x0A
#define HX8357_RDMADCTL           0x0B
#define HX8357_RDCOLMOD           0x0C
#define HX8357_RDDIM              0x0D
#define HX8357_RDDSDR             0x0F

#define HX8357_SLPIN              0x10
#define HX8357_SLPOUT             0x11
#define HX8357B_PTLON             0x12
#define HX8357B_NORON             0x13

#define HX8357_INVOFF             0x20
#define HX8357_INVON              0x21
#define HX8357_DISPOFF            0x28
#define HX8357_DISPON             0x29

#define HX8357_CASET              0x2A
#define HX8357_PASET              0x2B
#define HX8357_RAMWR              0x2C
#define HX8357_RAMRD              0x2E

#define HX8357B_PTLAR             0x30
#define HX8357_TEON               0x35
#define HX8357_TEARLINE           0x44
#define HX8357_MADCTL             0x36
#define HX8357_COLMOD             0x3A

#define HX8357_SETOSC             0xB0
#define HX8357_SETPWR1            0xB1
#define HX8357B_SETDISPLAY        0xB2
#define HX8357_SETRGB             0xB3
#define HX8357D_SETCOM            0xB6

#define HX8357B_SETDISPMODE       0xB4
#define HX8357D_SETCYC            0xB4
#define HX8357B_SETOTP            0xB7
#define HX8357D_SETC              0xB9

#define HX8357B_SET_PANEL_DRIVING 0xC0
#define HX8357D_SETSTBA           0xC0
#define HX8357B_SETDGC            0xC1
#define HX8357B_SETID             0xC3
#define HX8357B_SETDDB            0xC4
#define HX8357B_SETDISPLAYFRAME   0xC5
#define HX8357B_GAMMASET          0xC8
#define HX8357B_SETCABC           0xC9
#define HX8357_SETPANEL           0xCC

#define HX8357B_SETPOWER          0xD0
#define HX8357B_SETVCOM           0xD1
#define HX8357B_SETPWRNORMAL      0xD2

#define HX8357B_RDID1             0xDA
#define HX8357B_RDID2             0xDB
#define HX8357B_RDID3             0xDC
#define HX8357B_RDID4             0xDD

#define HX8357D_SETGAMMA          0xE0

#define HX8357B_SETGAMMA          0xC8
#define HX8357B_SETPANELRELATED   0xE9

// Plan is to move this to GFX header (with different prefix), though
// defines will be kept here for existing code that might be referencing
// them. Some additional ones are in the ILI9341 lib -- add all in GFX!
// Color definitions
#define	HX8357_BLACK   0x0000 ///< BLACK color for drawing graphics
#define	HX8357_BLUE    0x001F ///< BLUE color for drawing graphics
#define	HX8357_RED     0xF800 ///< RED color for drawing graphics
#define	HX8357_GREEN   0x07E0 ///< GREEN color for drawing graphics
#define HX8357_CYAN    0x07FF ///< CYAN color for drawing graphics
#define HX8357_MAGENTA 0xF81F ///< MAGENTA color for drawing graphics
#define HX8357_YELLOW  0xFFE0 ///< YELLOW color for drawing graphics
#define HX8357_WHITE   0xFFFF ///< WHITE color for drawing graphics

/*!
  @brief  Class to manage hardware interface with HX8357 chipset.
*/
class Adafruit_HX8357 : public Adafruit_SPITFT {
  public:
    Adafruit_HX8357(SPI * spi, IO * dcPin, IO * rstPin, uint8_t type = HX8357D);
    ~Adafruit_HX8357(void);

    void    begin(uint32_t freq = 0),
            setRotation(uint8_t r),
            invertDisplay(bool i),
            setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
    uint8_t readcommand8(uint8_t command, uint8_t index = 0);
  private:
    uint8_t displayType; // HX8357D vs HX8357B
};

#endif // _ADAFRUIT_HX8357_H
