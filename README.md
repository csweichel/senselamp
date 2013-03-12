SenseLamp
============================

SenseLamp is a drop-in lampshade replacement that can remotely turn the light on and off, runs Linux, is WiFi connected and has a bunch of sensors:
 * __Temperature and humidity sensor:__ recommended DHT11/DHT12 or anything pin-compatible
 * __PIR motion sensor:__ typically HC-SR501 PIR, easily found on eBay
 * __LDR light sensor:__ any light-dependent resistor works as long as the voltage divider is adjusted

In addition to a custom PCB, the SenseLamp is powered by a [TL WR703N](http://wiki.openwrt.org/toh/tp-link/tl-wr703n) WiFi router, which can be bought for cheap of eBay. The total power consumption is around 120-150mA/5V.

## How does this thing look like?
Images can be found in my blog: [32leav.es](http://32leav.es/?tag=senselamp)

## What's in this repository?
 * <tt>board</tt> the [Eagle](http://www.cadsoftusa.com/) files for the custom PCB going on top of the WR703N
 * <tt>frame</tt> AutoDesk Inventor sources and DXF files for two laser cut frame variants. The RectangularFrame could be ordered from [Ponoko](https://www.ponoko.com/), while the AngeledFrame requires some advanced laser-cutting techniques.
 * <tt>msp430</tt> contains the firmware for the MSP430 controlling the sensors - written using [Energia](https://github.com/energia/Energia/)

## What is __NOT__ in this repository?
The OpenWRT user-land to actually speak with the sensors. I use SenseLamps in combination with [busfarhn](https://github.com/32leaves/busfahrn), and have some code there that might help: [scripts/native_clients](https://github.com/32leaves/busfahrn/tree/master/scripts/native_clients).

## How do I build my own?
1. Get all the parts: a TL-WR703N, a TI Launchpad incl. its MSP430G2553, all the sensors and parts needed for the board (see schematics) and some form of relay module.
2. Solder thin wires to the serial testpads of the WR703N - see existing sources how to do this: [here](http://forums.openpilot.org/blog/52/entry-92-unbrick-wr703n-wifi-router/), [here](http://www.designspark.com/blog/hacking-the-tp-link-tl-wr703n) and [here](http://www.instructables.com/id/TL-WR703N-serial-port/). In the end you should have two wires that you could solder onto the SenseLamp board.
3. Solder all components on the SenseLamp board.
4. Laser-cut a frame or go to town on some tupperware with a Dremel
5. Assemble all components onto the frame
6. Power the whole contraption and play with some commands (have a look at the firmware for what commands there are)
7. Hook it up to your busfarhn installation.

# License - "MIT License"
Copyright (c) 2013 Christian Weichel, 32leaves

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
