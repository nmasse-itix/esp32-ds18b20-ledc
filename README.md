# esp32-ds18b20-ledc

## Introduction

This is an example application for the Maxim Integrated DS18B20 Programmable Resolution 1-Wire Digital Thermometer 
device and LEDs.

Ensure that submodules are cloned:

    $ git clone --recursive https://github.com/nmasse-itix/esp32-ds18b20-ledc.git

Build the application with:

    $ cd esp32-ds18b20-ledc
    $ idf.py menuconfig    # set your serial configuration and the 1-Wire GPIO - see below
    $ idf.py build
    $ idf.py -p (PORT) flash monitor

The program should detect your connected devices and periodically obtain temperature readings from them, displaying them
on the console.

## Dependencies

This application makes use of the following components (included as submodules):

 * components/[esp32-owb](https://github.com/DavidAntliff/esp32-owb)
 * components/[esp32-ds18b20](https://github.com/DavidAntliff/esp32-ds18b20)

## Hardware

To run this example, connect one or more DS18B20 devices to a single GPIO on the ESP32. Use the recommended pull-up 
resistor of 4.7 KOhms, connected to the 3.3V supply.

`idf.py menuconfig` can be used to set the 1-Wire GPIO.

Connect two leds on GPIO 19 and 21 with a small resistor (69 ohms).

## Acknowledgements

"1-Wire" is a registered trademark of Maxim Integrated.
