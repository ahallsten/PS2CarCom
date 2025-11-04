This is a microcontroller project where one can wire the signals of a PS2 controller to an adafruit feather RFM9x module and create their DIY remote control car.

If you want to try this project for yourself, I have been using PlatformIO for development, so it is recommended that you download the PlatformIO extension in VScode if you want to do anything more than read the code. 

As you will be able to tell in the platformio.ini file I currently only have support for the Adafruit feather 32u4. I think in the future. I will design a more custom PCB that uses a microcontroller that will have enough pins for this project without the need for a GPIO expander or logic level converters.