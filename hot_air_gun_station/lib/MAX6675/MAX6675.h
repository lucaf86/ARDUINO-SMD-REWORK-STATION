#include <SPI.h>

//#define MAX6675_Settings (SPISettings(1000000, MSBFIRST, SPI_MODE1))
#define MAX6675_Settings (SPISettings(1000000, MSBFIRST, SPI_MODE0)) // 1MHz spi speed = 16MHz / 16
//#define MAX6675_Settings (SPISettings(250000, MSBFIRST, SPI_MODE0)) // 0.25MHz spi speed = 16MHz / 64
//#define MAX6675_Settings (SPISettings(125000, MSBFIRST, SPI_MODE0)) // 0.125MHz spi speed = 16MHz / 128

#define MAX6675_THERMOCOUPLE_OPEN_BIT 0x04
#define MAX6675_THERMOCOUPLE_OPEN -1.0
#define MAX6675_READ_PERIOD 220
#define MAX6675_CONVERSION_RATIO 0.25

/* uncomment to use Software emulated spi interface. Works on any pin.
 * Default to Hardware SPI on pins CLK=13, MISO=12, CS=any*/

//#define MAX6675_SW_SPI


/* uncomment to use 16bit SPI transfer instead of two 8bit transfer */
#define MAX6675_HW_SPI_TRANSFER16

class MAX6675
{
    private:
        uint32_t _lastCallTime;
        int8_t _SSPin;
#ifdef MAX6675_SW_SPI
        int8_t _miso, _sclk;
#endif
        uint16_t _incomingMessage;
        float _currentTemp;
#ifdef MAX6675_SW_SPI
        byte spiread(void);
#endif

    public:
#ifdef MAX6675_SW_SPI
        MAX6675 (int8_t SCLKpin, int8_t SSpin, int8_t MISOpin);
        MAX6675 (int8_t SSpin)     { MAX6675(13, SSpin, 12); }
#else
        MAX6675 (int8_t SSpin);
#endif
        float readTempC();
        float readCelsius(void)    { return readTempC(); }
        float readFahrenheit(void) { return readCelsius() * 9.0 / 5.0 + 32; }

};
