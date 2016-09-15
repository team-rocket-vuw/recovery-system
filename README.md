# Metis-2 Onboard code
Code and various other files relating to the second Team Rocket flight. Electronics will fly on the TR-DU2 launch vehicle.

### Hardware code
Included is a lib folder within the ```/src``` directory, which much be copied into the Arduino global library directory.

Other libraries needed can be found in the following repos:
  * https://github.com/itead/ITEADLIB_Arduino_WeeESP8266 - ESP8266 library

To run, build circuit and upload code to Teensy of other Arduino compatable microcontroller, with pin numbers adjusted in code to suit your circuit.

### Hardware schematics
All schematics and PCB design conducted in KiCad. For parts, libraries used can be found here:
* Teensy library: https://github.com/XenGi/kicad_teensy
* ESP8266-01 library: https://github.com/jdunmire/kicad-ESP8266
* RFM22b library (Other useful libraries can be found here also): https://github.com/davepeake/kicad_libraries
