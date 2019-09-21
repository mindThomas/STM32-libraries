/* Modified version for use in STM32H7 project by
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#ifndef _ADAFRUIT_SPITFT_MACROS
#define _ADAFRUIT_SPITFT_MACROS

/*
 * Control Pins
 * */
#define SPI_DC_HIGH()           _dc->High()
#define SPI_DC_LOW()            _dc->Low()

#define SPI_CS_HIGH()			// handled by BEGIN_TRANSACTION
#define SPI_CS_LOW()			// handled by BEGIN_TRANSACTION

/*
 * Hardware SPI Macros
 * */
#define SPI_HAS_WRITE_PIXELS	true
#define HSPI_SET_CLOCK()
#define HSPI_BEGIN_TRANSACTION() _spi->BeginTransaction()
#define HSPI_END_TRANSACTION()	 _spi->EndTransaction()

// Optimized SPI
#define HSPI_READ()              _spi->TransactionRead()
#define HSPI_WRITE(b)            _spi->TransactionWrite8(b)
#define HSPI_WRITE16(s)          _spi->TransactionWrite16(s)
#define HSPI_WRITE32(l)          _spi->TransactionWrite32(l)
#define SPI_MAX_PIXELS_AT_ONCE  32
//#define HSPI_WRITE_PIXELS(c,l)  for(uint32_t i=0; i<(l); i+=2){ HSPI_WRITE(((uint8_t*)(c))[i+1]); SSPI_WRITE(((uint8_t*)(c))[i]); }
#define HSPI_WRITE_PIXELS(c,l)  for(uint32_t i=0; i<(l); i+=2){ HSPI_WRITE16( *((uint16_t*)((uint8_t*)c + i)) ); }  //     ((uint8_t*)(c))[i]);


#define SPI_BEGIN_TRANSACTION() HSPI_BEGIN_TRANSACTION()
#define SPI_END_TRANSACTION()   HSPI_END_TRANSACTION()
#define SPI_WRITE16(s)          HSPI_WRITE16(s)
#define SPI_WRITE32(l)          HSPI_WRITE32(l)
#define SPI_WRITE_PIXELS(c,l)   HSPI_WRITE_PIXELS(c,l)

#endif // _ADAFRUIT_SPITFT_MACROS
