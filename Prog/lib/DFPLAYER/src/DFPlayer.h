#ifndef DFPLAYER_H
#define DFPLAYER_H

#include <Arduino.h> // Always include this!
//#include <NeoSWSerial.h>

class DFPLAYER {
  public:
    // Basic functions
    static void begin(Stream* DFPport, uint8_t rx, uint8_t tx, uint8_t powerPin, Stream* serial); // Configure and initialise de DFPlayer
    static void play(); // Play the track
    static void volume(uint8_t vol); // Volume
    static void pause(); // Pause the track
    static void toSleep(); // Going to low power mode
    static void wakeUp(); // Going to normal mode
    static void wakeUpReset(); // Must call this function 1-2 second after wake up

  private:
    // Variables
    static Stream* _DFPport;
    static Stream* _serial;
    // Functions
    static void sendDFPCommand(uint8_t cmd, uint8_t param = 0x00); // Send data to the serial port
    static void _setVolume();
};

#endif