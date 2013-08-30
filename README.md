ADF41020-Arduino-SPI-Controller
================================

Arduino Uno based SPI Controller for the ADF41020 Development Board which allows sending custom frequency sweep commands at a much higher rate than the off-the-shelf Analog Devices GUI allows.

This Arduino project was built and tested for the Arduino Uno but should work for other Arduinos as well.

Instructions
------------
Replace the "Precalculated SPI Data" section with your own data.

Important Notes
---------------
* You must desolder the resistors leading from the USB controller section of the ADF41020 board to the Data, CLK, and LE testpoints in order for this to work properly. Read the datasheet for the development board carefully.
* The Arduino is at 5 Volts, the ADF41020 requires a max of 3. So when you solder the wires across, use a resistor divider. I used a 1.0k and a 1.5k.

