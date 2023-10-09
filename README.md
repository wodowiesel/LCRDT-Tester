# LCRDT-Tester

The Tester is a tool for identification of the component and its values!

![PCB](/docs/PCB-laypout-Wodowiesel-LCRDT-tester-schematics.png/)

Compatibility list:

- Inductors
- Capacitors
- Resistors
- Diodes
- Transistors
- ESR

## DISCLAIMERS

**This is still "work-in-progress (WIP)" so I do NOT take any kind of responsibility for damages/data losses!**

**You can use this at your own risk without guarantees and put device on non conductive surface or enclosure!**

If you like the new design yo are free to share it!

Have fun :)

## HARDWARE

- 1x Arduino Nano v3 (or Uno for old versions) + USB-Cable
- 1x Custom PCB for Arduino Nano v3 by WodoWiesel
- 3x 10 kOhm Resistors
- 3x 470 Ohm Resistors
- 1x 100 nF capacitor
- 1x 4-pin display for +3.3V (0.96" OLED)
- 2x Push-Button
- 1x 3-Pin Male Pin Header (for component testing)
- 1x 4-Pin Male Pin Header (for display)

- 1x Soldering iron
- Solder + flux + IPA

## SOFTWARE

Download my adapted [code](/code-aduino/LCRDT-Tester-WodoWiesel.ino) and configure your setup and load it via Arduino IDE.

## HOW TO USE

1. configure code adapted to your setup regarding used parts and display type

2. upload code to your arduino

3. solder the new pcb

4. put component of choice in the testing headers and read values

## CREDITS

:copyright: 10/10/2023 WodoWiesel

Github (https://github.com/wodowiesel/)

Twitch (https://www.twitch.com/wodowiesel/)

Instagram (https://www.instagram.com/wodowiesel/)

YT (https://www.youtube.com/@wodowiesel/)

E-Mail (wodowiesel@quantentunnel.de)

The new design was checked by the electrical engineer theBrutzler

(https://github.com/theBrutzler/ & https://www.twitch.com/theBrutzler/)

The new optimized PCB was designed with free KiCAD software (https://www.kicad.org/)

:heavy_exclamation_mark: The PCB should be produced at a manufacturer

(like PCBWay or similar) with the RoHS lead-free option for environmental protection.

Originals:

Proekty (https://arduino.ru/forum/proekty/transistor-tester-arduino/)

Vlad (https://github.com/vlad-gheorghe/ardutester/)

Code: Karl-Heinz Kubbeler (version 1.08k, 2013 / 1.10, 2014) 

engl. Journal [PDF](/docs/ttester_eng108k.pdf)

## LICENSES

- Hardware

CERN Open Hardware License Version 2 - Weakly Reciprocal

Short identifier CERN-OHL-W-2.0

[LICENSE Source](https://spdx.org/licenses/CERN-OHL-W-2.0.html)

- Software

This work is licensed under a Creative Commons Attribution-NonCommercial 3.0 Unported License.

[CCANC](http://creativecommons.org/licenses/by-nc/3.0/)
