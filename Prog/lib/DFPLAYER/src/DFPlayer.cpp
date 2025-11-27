#include "DFPlayer.h"

Stream* DFPLAYER::_serial = nullptr;    // Serial port for debug
Stream* DFPLAYER::_DFPport = nullptr;   // Serial port for the DFPlayer
uint8_t _rx;   // for sleep mode
uint8_t _tx;   // for sleep mode
uint8_t _powerPin; // for sleep mode
uint8_t _volume = 15; // base volume

void DFPLAYER::begin(Stream* DFPport, uint8_t rx, uint8_t tx, uint8_t powerPin, Stream* serial) {
  _serial = serial;
  _serial->println("librairie DFPlayer");   // Checking the librairy is started
  _DFPport = DFPport;
  _powerPin = powerPin;
  _rx = rx;
  _tx = tx;
  pinMode(_powerPin, OUTPUT);
  pinMode(_rx, OUTPUT);
  pinMode(_tx, OUTPUT);
  sendDFPCommand(0x3F);   // Initialise DFPlayer
  delay(15);
  volume(_volume);
}

void DFPLAYER::sendDFPCommand(uint8_t cmd, uint8_t param = 0x00){ // If unspecified, then it's un-needed = 0
  // This function send data to the DFPlayer
  uint8_t version = 0xFF; // It work with this so I'm keeping it
  uint8_t length  = 0x06; // Cmd, feedback, param size, param, checksum1, checksum2 -> 6 ; /!\ must count the checksums byte event if their site says otherwise

  uint16_t checksum = (version + length + cmd + param); // I removed the feedback = 0 and the param size = 0

  uint8_t trame[10] = {
    0x7E,   // Start bit is always 0x7E
    version,
    length,
    cmd,
    0x00, // No feedback expected
    0x00, // Alway 8 bit param
    param,
    (uint8_t)(-checksum >> 8),    // Checksum is sent to 2 bytes
    (uint8_t)(-checksum & 0xFF),
    0xEF    // End bit is always  0xEF
  };

  _DFPport->write(trame, 10); // Actually sending data
  delay(10); // Small delay as the DFPlayer is slow
}

void DFPLAYER::play(){ // I only have one track so no param needed
  sendDFPCommand(0x03, 0x01);
}

void DFPLAYER::volume(uint8_t vol){ // Volume
  if(vol<31){ // Checking volume isn't out of bound
    _volume = vol;
    _setVolume();
  }
}

void DFPLAYER::_setVolume(){
  sendDFPCommand(0x06, _volume);
}

void DFPLAYER::pause(){ // Pause the track
  sendDFPCommand(0x0E);
}

void DFPLAYER::toSleep(){ // Going to low power mode
  digitalWrite(_powerPin, HIGH);
  //_DFPport->stopListening(); // Doesn't work anymore with NeoSWSerial
  digitalWrite(_rx, LOW);
  digitalWrite(_tx, LOW);
}

void DFPLAYER::wakeUp(){ // Going to normal mode
  digitalWrite(_powerPin, LOW);
  digitalWrite(_rx, HIGH);
  digitalWrite(_tx, HIGH);
  //_DFPport->listen(); // Doesn't work anymore with NeoSWSerial
  //_serial->println("must call DFPLAYER::wakeUpReset() in 0.9 seconds");
}

void DFPLAYER::wakeUpReset(){ // Must call this function 1-2 second after wake up
  DFPLAYER::_setVolume();
}