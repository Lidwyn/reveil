#include "MAX7219.h"
/*
|SEG|BIT|  binary  |
| A | 6 |0b01000000|
| B | 5 |0b00100000|
| C | 4 |0b00010000|
| D | 3 |0b00001000|
| E | 2 |0b00000100|
| F | 1 |0b00000010|
| G | 0 |0b00000001|
| DP| 7 |0b10000000|
*/

uint8_t MAX7219::_din = 0;
uint8_t MAX7219::_clk = 0;
Stream* MAX7219::_serial = nullptr;

MAX7219::MAX7219(uint8_t cs) : _cs(cs) {                // constructeur avec un cs unique a chaque instance
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
}

void MAX7219::begin(uint8_t din, uint8_t clk, Stream* serial) {         // configure DIN/CLK partagés
  MAX7219::_din = din;
  MAX7219::_clk = clk;
  pinMode(_din, OUTPUT);
  pinMode(_clk, OUTPUT);
  digitalWrite(_clk, LOW);
  digitalWrite(_din, LOW);
  _serial = serial;
  _serial->println("librairie MAX7219");
}

void MAX7219::spi_write(uint8_t data) {                 // gestion de l'écriture bit par bit dans le SPI
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(_din, (data & 0x80) ? HIGH : LOW);
    digitalWrite(_clk, LOW);
    delayMicroseconds(1);
    digitalWrite(_clk, HIGH);
    delayMicroseconds(1);
    data <<= 1;
  }
}

void MAX7219::send(uint8_t address, uint8_t data) {     // fonction d'écriture : address = DIGx et data = SEG0-7
  digitalWrite(_cs, LOW);
  spi_write(address);
  spi_write(data);
  digitalWrite(_cs, HIGH);
}

void MAX7219::init() {                                  // initialisation du MAX7219CNG
  send(0x0F, 0x00); // Pas de test
  send(0x0C, 0x01); // Normal mode
  send(0x0B, 0x04); // Scan limit digits 0-4
  send(0x0A, 0x03); // Intensité moyenne
  send(0x09, 0x00); // **Mode décodage désactivé**
  
  // Éteindre tous les digits
  for (uint8_t i = 1; i <= 4; i++) {
    send(i, 0x00);
  }
}

void MAX7219::Brightness(uint8_t brightness){
  send(0x0A, 0x0F & brightness);
}