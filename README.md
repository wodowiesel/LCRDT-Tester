#LCRD-Tester

The Tester is a Tool for identification ofthe part and its values!

![PCB](/docs/electronic-component-tester-nano.png/)

Compatibility list:

- Inductors
- Capacitors
- Resistors
- Diodes
(- Tranistors)

# DISCLAIMERS

**This is still "work-in-progress (WIP)" so I do NOT take any kind of warranty/responsibility for damages/data losses!**

**You can use this at your own risk without guarantees and put device on non conductive surface or enclosure!**

If you like the new design yo are free to share it!

Have fun :)

## HARDWARE

- 1x Arduino Nano v3 (or Uno for old versions) + USB-Cable
- 1x Custom PCB for Arduino Nano v3 by WodoWiesel
- 3x 330 Ohm Resistors
- 3x 470 Ohm Resistors
- 1x ~1,5 nF capacitor
- 1x 4-pin display for +3.3v
- 1x 3-Pin Male Pin Header (for component testing)
- 1x 4-Pin Male Pin Header (for display)

- 1x Soldering iron
- Solder + flux + IPA

## SOFTWARE

Download my adapted ![code] (/code-aduino/LCRD-Tester-WodoWiesel.ino)

and configure your setup and load it via  Arduino IDE.

## HOW TO USE

1. Configure code adapted to your setup regarding used partss and display typee

2. upload code to your arduino

3. solder the new pcb

4. put component of choice in the tsting headers an read value

## CREDITS

 :copyright: 2023 WodoWiesel 

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


## LICENSES

- Hardware

CERN Open Hardware License Version 2 - Weakly Reciprocal

Short identifier CERN-OHL-W-2.0

[LICENSE Source](https://spdx.org/licenses/CERN-OHL-W-2.0.html)

- Software

This work is licensed under a Creative Commons Attribution-NonCommercial 3.0 Unported License.

[CCANC](http://creativecommons.org/licenses/by-nc/3.0/)
