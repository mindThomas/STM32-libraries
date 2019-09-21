/* Modified version for use in STM32H7 project by
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef _ADAFRUIT_SPITFT_
#define _ADAFRUIT_SPITFT_

#include "SPI.h"
#include "IO.h"
#include "Adafruit_GFX.h"

// Estimated RAM usage:
// 4 bytes/pixel on display major axis + 8 bytes/pixel on minor axis,
// e.g. 320x240 pixels = 320 * 4 + 240 * 8 = 3,200 bytes.
typedef volatile uint32_t RwReg;
#include "Adafruit_SPITFT_Macros.h"

/// A heavily optimized SPI display subclass of GFX. Manages SPI bitbanging, transactions, DMA, etc! Despite being called SPITFT, the classic SPI data/command interface is also used by OLEDs.
class Adafruit_SPITFT : public Adafruit_GFX {

    public:
        Adafruit_SPITFT(uint16_t w, uint16_t h, SPI * spi, IO * dcPin, IO * rstPin = 0);

        void reset(void);

        // Required Non-Transaction
        void      drawPixel(int16_t x, int16_t y, uint16_t color);

        // Transaction API
        void      startWrite(void);
        void      endWrite(void);

        void      writePixel(int16_t x, int16_t y, uint16_t color);
        void      writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
        void      writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
        void      writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);

        // Transaction API not used by GFX

	/*!
	  @brief   SPI displays set an address window rectangle for blitting pixels
	  @param  x  Top left corner x coordinate
	  @param  y  Top left corner x coordinate
	  @param  w  Width of window
	  @param  h  Height of window
	*/
	virtual void      setAddrWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) = 0;

	/*!
	  @brief   Write a 2-byte color  (must have a transaction in progress)
	  @param    color 16-bit 5-6-5 Color to draw
	*/
	void      inline writePixel(uint16_t color) { SPI_WRITE16(color); }
        void      writePixels(uint16_t * colors, uint32_t len);
        void      writeColor(uint16_t color, uint32_t len);
	void      pushColor(uint16_t color);

        // Recommended Non-Transaction
        void      drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
        void      drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
        void      fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

        using     Adafruit_GFX::drawRGBBitmap; // Check base class first
        void      drawRGBBitmap(int16_t x, int16_t y,
                    uint16_t *pcolors, int16_t w, int16_t h);
	void      invertDisplay(bool i);

        uint16_t  color565(uint8_t r, uint8_t g, uint8_t b);
        void      writeCommand(uint8_t cmd);
        void      spiWrite(uint8_t v);
        uint8_t   spiRead(void);

    protected:
        SPI * _spi;         ///< The SPI device we want to use (set in constructor)
        IO *  _dc;
		IO * _rst;

	uint8_t   invertOnCommand = 0,    ///<  SPI command byte to turn on invert
	  invertOffCommand = 0;           ///<  SPI command byte to turn off invert
	int16_t   _xstart = 0;   ///< Many displays don't have pixels starting at (0,0) of the internal framebuffer, this is the x offset from 0 to align
	int16_t   _ystart = 0;   ///< Many displays don't have pixels starting at (0,0) of the internal framebuffer, this is the y offset from 0 to align

#ifdef USE_SPI_DMA
        Adafruit_ZeroDMA dma;                  ///< DMA instance
        DmacDescriptor  *dptr          = NULL; ///< 1st descriptor
        DmacDescriptor  *descriptor    = NULL; ///< Allocated descriptor list
        uint16_t        *pixelBuf[2];          ///< Working buffers
        uint16_t         maxFillLen;           ///< Max pixels per DMA xfer
        uint16_t         lastFillColor = 0;    ///< Last color used w/fill
        uint32_t         lastFillLen   = 0;    ///< # of pixels w/last fill
        uint8_t          onePixelBuf;          ///< For hi==lo fill
#endif
};

#endif // !_ADAFRUIT_SPITFT_
