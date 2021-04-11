# !
# Version 5.0.0 AOG Arduino Code is WIP!
The cross track error is currently simulated using a calculation based on the desired steer angle sent from agopengps (the cross track error measurement was removed without warning in V5 of AOG). This should work for following an A-B line but may not perform well on tight corners or u-turns.
# !

# AutosteerPCBv2LB
forked from  mnltake/AutosteerPCBv2LB

This is a porting of mnltake's lightbar code to the latest version of the AgOpenGPS machine code (4.3.10).

Requires the [Adafruit Neopixel](https://github.com/adafruit/Adafruit_NeoPixel) library (version 1.4 or higher)

The master branch is as close to the standard 4.3.10 arduino file as possible.


## Software
### Installing the Adafruit_Neopixel library

Open the [Arduino client](https://www.arduino.cc/en/software) (version 1.12 or higher is recomended), and open the Tools menu. Select the "Manage Libraries..." option:

<p align="center">
<img src="https://github.com/MattWoodhead/AutosteerPCBv2LB/blob/master/resources/install_arduino_library.png" height="400">
</p>

Using the library manager, search for "Neopixel". Find the library in the results called "Adafruit Neopixel" as shown in the third item in the window below. Note that there are several libraries with similar names - for the Arduino Nano the standard Adafruit Neopixel library is the correct one.

<p align="center">
<img src="https://github.com/MattWoodhead/AutosteerPCBv2LB/blob/master/resources/install_neopixel_lib.png" height="400">
</p>


## Hardware
### Arduino Nano
If you are using the AgOpenGPS PCB, the WS2812B strip may get its 5V supply from the PCB power supply. This is not a problem for a 5V Nano, but if you use a 3.3V Nano or one of the form factor compatible boards (such as the ST Nucleo STM32G431) the WS2812B LEDs can behave erratically. This is due to a requirement for the data signal to be >0.7x the supply voltage. This problem can be solved by placing a diode between the 5V supply from the PCB and the WS2812 strip to cause a 0.7V drop of the supply to 4.3V.

<p align="center">
<img src="https://github.com/MattWoodhead/AutosteerPCBv2LB/blob/master/resources/diode_trick.png" width="400">
</p>

### Capacitor
If you have a large string of LEDs that have high brightness settings, the Nano (or similar microcontroller) can be overwhelmed by the initial current spike when the LEDs are first powered on. This can be solved by placing a large electrolytic capacitor (220 to 1000 uF) between the 5V and 0V supply to the LED strip. If you use the AgOpenGPS PCB to supply the power to the LEDs and the m,icrocontroller is only supplying the control signal, this should not be a problem.
