//
//    FILE: dht.h
//  AUTHOR: Rob Tillaart
// VERSION: 0.1.13
// PURPOSE: DHT Temperature & Humidity Sensor library for Arduino
//     URL: http://arduino.cc/playground/Main/DHTLib
//
// HISTORY:
// see dht.cpp file
//

#ifndef dht_h
#define dht_h

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#define DHT_LIB_VERSION "0.1.13"

#define DHTLIB_OK                0
#define DHTLIB_ERROR_CHECKSUM   -1
#define DHTLIB_ERROR_TIMEOUT    -2
#define DHTLIB_INVALID_VALUE    -999

#define DHTLIB_DHT11_WAKEUP     18
#define DHTLIB_DHT_WAKEUP       1

#if defined(ARDUINO_ARCH_MBED)//改动处开始
#define DHTLIB_TIMEOUT (133 * 1000000 / 40000) 
#else
#define DHTLIB_TIMEOUT (F_CPU/40000)
#endif//改动处结束

class dht
{
public:
    // return values:
    // DHTLIB_OK
    // DHTLIB_ERROR_CHECKSUM
    // DHTLIB_ERROR_TIMEOUT
    int read11(uint8_t pin);
    int read(uint8_t pin);

    inline int read21(uint8_t pin) { return read(pin); };
    inline int read22(uint8_t pin) { return read(pin); };
    inline int read33(uint8_t pin) { return read(pin); };
    inline int read44(uint8_t pin) { return read(pin); };

    double humidity;
    double temperature;

private:
    uint8_t bits[5];  // buffer to receive data
    int _readSensor(uint8_t pin, uint8_t wakeupDelay);
};
#endif
//
// END OF FILE
//