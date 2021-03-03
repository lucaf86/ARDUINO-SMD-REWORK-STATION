#include <MAX6675.h>

#ifdef MAX6675_SW_SPI
MAX6675::MAX6675(int8_t SCLKpin, int8_t MISOpin, int8_t SSpin)
{
    _SSPin = SSpin;
    _sclk = SCLKpin;
    _miso = MISOpin;
    pinMode(_SSPin, OUTPUT);
    digitalWrite(_SSPin, HIGH);
    pinMode(_sclk, OUTPUT);
    pinMode(_miso, INPUT);

    //_lastCallTime = 0;
}
#else
MAX6675::MAX6675(int8_t SSpin)
{
    _SSPin = SSpin;
    pinMode(_SSPin, OUTPUT);
    digitalWrite(_SSPin, HIGH);
    SPI.begin();

    //_lastCallTime = 0;
}
#endif

float MAX6675::readTempC()
#ifdef MAX6675_SW_SPI
{
    digitalWrite(_SSPin, LOW);
    delayMicroseconds(10);

    _incomingMessage = spiread();
    _incomingMessage <<= 8;
    _incomingMessage |= spiread();
        
    digitalWrite(_SSPin, HIGH); 

    if (_incomingMessage & MAX6675_THERMOCOUPLE_OPEN_BIT)
        return MAX6675_THERMOCOUPLE_OPEN;
    float _currentTemp = (_incomingMessage >> 3) * MAX6675_CONVERSION_RATIO;
    return _currentTemp;
}
#else
{
    //if (millis() - _lastCallTime >= MAX6675_READ_PERIOD)
    //{
        //SPI.usingInterrupt(digitalPinToInterrupt(3));  // disable all interrupt (valid input 0-7)
        //SPI.usingInterrupt(digitalPinToInterrupt(2));
        SPI.beginTransaction(MAX6675_Settings);
        digitalWrite(_SSPin, LOW);
#ifdef MAX6675_HW_SPI_TRANSFER16
        _incomingMessage = SPI.transfer16(0x00);
#else
        _incomingMessage = SPI.transfer(0x0);
        _incomingMessage <<= 8;
        _incomingMessage |= SPI.transfer(0x0);
#endif
        digitalWrite(_SSPin, HIGH);
        SPI.endTransaction();
        //SPI.notUsingInterrupt(digitalPinToInterrupt(3)); // restore interrupt
        //SPI.notUsingInterrupt(digitalPinToInterrupt(2)); // restore interrupt
        //_lastCallTime = millis();

        if (_incomingMessage & MAX6675_THERMOCOUPLE_OPEN_BIT)
            return MAX6675_THERMOCOUPLE_OPEN;
        float _currentTemp = (_incomingMessage >> 3) * MAX6675_CONVERSION_RATIO;
        return _currentTemp;
    //}
    //return _currentTemp;
}
#endif

#ifdef MAX6675_SW_SPI
byte MAX6675::spiread(void) {
  int i;
  byte d = 0;

  for (i = 7; i >= 0; i--) {
    digitalWrite(_sclk, LOW);
    delayMicroseconds(10);
    if (digitalRead(_miso)) {
      // set the bit to 0 no matter what
      d |= (1 << i);
    }

    digitalWrite(_sclk, HIGH);
    delayMicroseconds(10);
  }

  return d;
}
#endif