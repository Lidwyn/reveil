#ifndef MAX7219_H
#define MAX7219_H

#include <Arduino.h> // Toujours inclure ça !

class MAX7219 {
  public:
    //fonction de base
    MAX7219(uint8_t cs);                          // constructeur avec un cs unique a chaque instance
    static void begin(uint8_t din, uint8_t clk);  // configure DIN/CLK partagés

    //fonction
    void send(uint8_t address, uint8_t data);          // fonction d'écriture : address = DIGx et data = SEG0-7
    void init();                                       // initialisation du MAX7219CNG

  private:
    //variable
    uint8_t _cs;                                  // cs associé à chaque instance pour l'écriture en SPI
    //statique
    static uint8_t _din;                          // din global pour la liaison SPI
    static uint8_t _clk;                          // clk global pour la liaison SPI

    //fonction
    void spi_write(uint8_t data);                      // gestion de l'écriture bit par bit dans le SPI

};

#endif