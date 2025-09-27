#ifndef DS3231_H
#define DS3231_H

#include <Arduino.h> // Toujours inclure ça !
#include <Wire.h>

class DS3231 {
  public:
    static void begin(const uint8_t addr, const bool wireBegan, Stream* serial);              // Méthode d'initialisation
    static bool hasbegun();
    static void readDisplayTime(uint8_t* buffer);
    static void readAlarmCheck(uint8_t* buffer);
    static void readFullDate(uint8_t* buffer);
    static void setupFullDateRead(uint8_t* buffer);
    static void write1byte(const uint8_t Register, const uint8_t data);
    static void writeMbyte(const uint8_t FirstRegister, const uint8_t* data, const uint8_t nbBytes);
    static void writeSec(const uint8_t second);
    static void writeMin(const uint8_t minute);
    static void writeHou(const uint8_t hour);
    static void writeDay(const uint8_t day);
    static void writeDat(const uint8_t date);
    static void writeMon(const uint8_t month);
    static void writeYea(const uint8_t year);
  
  private:
    static uint8_t address;
    static bool boolhasbegun;
    static Stream* _serial;
};

#endif