#ifndef AT24C32_H
#define AT24C32_H

#include <Arduino.h> // Toujours inclure ça !
#include <Wire.h>

class AT24C32 {
  public:
    static void begin(const uint8_t addr, const bool wireBegan, Stream* serial);              // Méthode d'initialisation
    static bool hasbegun();
    static uint8_t readAll(uint8_t* bufferNU, uint8_t* bufferU);
    static uint8_t readActive(uint8_t* bufferNU, uint8_t* bufferU);
    static bool createNewNU(uint8_t* buffer);
    static bool createNewU(uint8_t* buffer);
    static void modifyNU(uint8_t nb, uint8_t* buffer);
    static void modifyU(uint8_t nb, uint8_t* buffer);
  
  private:
    static uint8_t address;
    static bool boolhasbegun;
    static const uint8_t RegBytes;
    static const uint8_t NUBytes;
    static const uint8_t UBytes;
    static Stream* _serial;
};

#endif