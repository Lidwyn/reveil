# Alarm Clock Project

A fully custom-built **embedded alarm clock** designed from scratch, featuring a hardware-level implementation using an **ATmega328P**, **7-segment displays**, a **real-time clock (RTC)** module, and a modular audio system with microSD song storage.
This project integrates **embedded firmware**, **electronics design**, **PCB layout**, **CAD**, and **technical documentation**.

> This project is **still in progress**, every features or documents mentionned here might **not be complete or avaible yet**.

## Features
+ **Times display**
+ **Next alarm** on secondary display
+ Up to **15 one time (unique) and 15 weekly (non unique) independent alarms**
+ **Night Mode** - complete display blackout
+ **RTC-based accurate timekeeping** (with battery backup)

## Tech Stack & Tools

### Firmware
+ **Atmega328P** (bare-metal Arduino-style development)
+ **Arduino language** via **PlatformIO**
+ Interrupt-drivern logic
+ Low-power modes

### Elecrtonics & PCB
+ **Kicad** for schematic capture and PCB layout
+ Multi-voltage power stage (5V and 3.3V)

### Mechanical Design
+ **CATIA** for enclosure & mechanical parts

### Documentation
+ **LaTeX** for the user and technical manuals

## Project Architure
```
/Prog/          -> PlatformIO project for MCU firmware
/Schematic/     -> KiCad schematics & PCB files
/cad/           -> CATIA models for enclosure
/Latex/         -> Latex (and pdf) manual
```

## How It Works
1. The **ATmega328P** handles alarm logic, display control, and power management.
2. A **DS3231 RTC** maintains accurate time with battery backup. Through **I2C bus**.
3. On the same chips as the DS3231, we have an **AT24C32 EEPROM** that store the multiples alarms, the volume, and the brightness setup. Also with battery backup and through the same **I2C bus**.
4. Two four **7-segment display** and **9 leds**, driven by two **MAX7219** are in charge of the display. Through **SPI bus**.
5. A **DFPlayer** handle the audio, stored on a **microSD card**, with an integrated amplifier and an external speaker. Through a **Serial Port bus**.
5. 6 buttons for setup.

Find more information about the GUI on the manual:
- [Manuel fr](Latex/Manuel.pdf)



## Learning Objectives & Personal Development
This project in an opportunity to build **foundational embedded-systems skills** from scratch, inculding:
+ **Embedded C++ developpement** on microcontrollers
+ Designing and debugging **digital and mixed-signal electronics**
+ Understanting **RTCs**, **SD interfaces**, **multiplexing**, and **interrupt-driven systems**
+ **PCB worklow** from schematic to layout to fabrication
+ **Mechanical design** for electronics
+ Writing **clear and precise technical documentation** in **LaTeX**

## What I'm Most Proud Of
+ Implementing **deep sleep and interrupt-driven wake-up**, dramatically
reducing power consumption
+ Designing a **clean dual-voltage system** that support both 5V and 3.3V as the ATmega328P consumme less current at 3.3V further improving power consumption
+ Using **logical Mosfet** to cut unactive chips such as the audio chips, reducing noise to zero and reducing consumption
+ **Creating the entire device end-to-end** : firmware, electonics, PCB, mechanical parts, and documentation
+ Gaining practical **embedded engineering experience** from low to no prior knowledge

## How to Build / Run
### Hardware
1. Burn bootloader on the ATmega328P
2. Flash your microSD on FAT32
3. Insert microSD with you preferred alarm audio file with this typo `0001WhateverName.mp3`

### Firmware
1. Install **PlatformIO** (VS Codium extension)
2. Plug the alarm clock
3. Plug your computer into the USB C to seral adapter of the card
4. Adjust `upload_port = COM15` to your specific usb port in `/Prog/platformio.ini`
5. Upload firmware with `pio run --target upload`

## Status
This project is **still in development** :
+ Firmware : **working**, new features should be added and bugs fixed
+ PCB : **schematic is done**, PCB layout is not
+ Mechanical part : not started
+ Documenation : just the base of the french user manual version