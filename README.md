# PIC32GC #

Read game controllers and joysticks connected to a PIC32MX550F256L.
SPI3 clock at 1MHz, connects to an OLED display module, 128x32 pixel,
based on the SSD1306 chip.

## Connections ##

OLED display connections:

| Signal | Chip  | Name      | Pin | Chip    | Name | Pin |
|--------|-------|-----------|-----|---------|------|-----|
| SCK3   | PIC32 | RF13/SCK3 | 39  | SSD1306 | SCK  | 3   |
| MOSI3  | PIC32 | RG8/SDO3  | 12  | SSD1306 | SDA  | 4   |
| SS3    | PIC32 | RA0       | 17  | SSD1306 | CS   | 7   |
| DC     | PIC32 | RG9       | 14  | SSD1306 | DC   | 6   |
| RES    | PIC32 | RE8       | 18  | SSD1306 | RES  | 5   |

Digital pin connected to GameCube controller:

* GC RA4 pin 60 via diode, to pull signal line down
* GC RA5 pin 61, direct, to sense state of signal line

Debugging LEDs on dev board:

* LED1 RE6 pin 4
* LED2 RE7 pin 5
* LED3 RE1 pin 94
* LED4 RA7 pin 91
* LED5 RA6 pin 92

PIC32 pin numbers are for the 100-pin package.

LEDs light when the pin is pulled LOW.

## PIC32 Toolchain ##

MPLAB X V5.20 and 'xc32' V2.15. These are quite old versions now, so I really ought to upgrade!

## PIC32 Programmer ##

Microchip ICD3. Other programmers should work, e.g. ICD4 or PICkit4.

