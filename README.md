ADF41020-Arduiono-SPI-Controller
================================

Arduino Uno based SPI Controller for the ADF41020 Developmebt Board which allows sending frequency sweep commands at a much higher rate than the off-the-shelf Analog Devices GUI allows.

The Arduino Project was built and tested for the Arduino Uno but should work for other Arduinos as well.

Important Notes!
----------------
* You must desolder the resistors leading from the USB controller section of the ADF41020 board to the Data, CLK, and LE testpoints.
* The Arduino is at 5 Volts, the ADF41020 requires a max of 3. So when you solder the wires across, use a resistor divider. I used a 1.k and a 1.5k.
