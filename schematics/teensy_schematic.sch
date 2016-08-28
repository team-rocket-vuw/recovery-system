EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:teensy
LIBS:DataStorage
LIBS:rfm22
LIBS:teensy_schematic-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Teensy3.2 U?
U 1 1 57BA64F7
P 8050 3750
F 0 "U?" H 8050 5750 60  0000 C CNN
F 1 "Teensy3.2" H 8050 1750 60  0000 C CNN
F 2 "" H 8050 3450 60  0000 C CNN
F 3 "" H 8050 3450 60  0000 C CNN
	1    8050 3750
	1    0    0    -1  
$EndComp
$Comp
L MicroSD J?
U 1 1 57BA6618
P 1200 5300
F 0 "J?" H 1000 6000 60  0000 C CNN
F 1 "MicroSD" H 1000 4600 60  0000 C CNN
F 2 "" H 1300 5450 60  0000 C CNN
F 3 "" H 1300 5450 60  0000 C CNN
	1    1200 5300
	1    0    0    -1  
$EndComp
$Comp
L RFM22 U?
U 1 1 57BA6711
P 3850 6100
F 0 "U?" H 3750 5400 60  0000 C CNN
F 1 "RFM22" H 4100 6600 60  0000 C CNN
F 2 "" H 3850 6100 60  0000 C CNN
F 3 "" H 3850 6100 60  0000 C CNN
	1    3850 6100
	1    0    0    -1  
$EndComp
$Comp
L L3G4200D-breakout U?
U 1 1 57C24F63
P 4450 2000
F 0 "U?" H 4500 2050 60  0000 C CNN
F 1 "L3G4200D-breakout" H 16850 4350 60  0000 C CNN
F 2 "" H 16850 4350 60  0001 C CNN
F 3 "" H 16850 4350 60  0001 C CNN
	1    4450 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2250 5200 1400 5200
Wire Wire Line
	2250 5800 3350 5800
Connection ~ 2250 5200
Wire Wire Line
	2250 5900 3350 5900
Connection ~ 2250 5800
Wire Wire Line
	2250 6000 3350 6000
Connection ~ 2250 5900
Wire Wire Line
	1400 5000 3000 5000
Wire Wire Line
	3000 5700 3350 5700
Connection ~ 3000 5000
Wire Wire Line
	2250 1550 2250 6000
Connection ~ 3000 4200
Wire Wire Line
	3000 4200 3000 5700
Wire Wire Line
	7050 3750 4950 3750
Wire Wire Line
	4950 3750 4950 5950
Wire Wire Line
	4950 5950 4550 5950
Wire Wire Line
	4550 6050 5050 6050
Wire Wire Line
	5050 6050 5050 3900
Wire Wire Line
	5050 3900 7050 3900
Wire Wire Line
	1400 4900 4950 4900
Connection ~ 4950 4900
Wire Wire Line
	1400 5300 5050 5300
Connection ~ 5050 5300
Wire Wire Line
	7050 4650 5150 4650
Wire Wire Line
	5150 4650 5150 5850
Wire Wire Line
	5150 5850 4550 5850
Wire Wire Line
	1400 5100 5150 5100
Connection ~ 5150 5100
Wire Wire Line
	1400 4800 4850 4800
Wire Wire Line
	4850 4800 4850 3600
Wire Wire Line
	4550 6150 5300 6150
Wire Wire Line
	5300 6150 5300 3450
Wire Wire Line
	3350 6150 3000 6150
Wire Wire Line
	3000 6150 3000 7250
Wire Wire Line
	4550 6600 4800 6600
Wire Wire Line
	4800 6600 4800 7250
Wire Wire Line
	4800 7250 3000 7250
Wire Wire Line
	3350 6250 3350 7150
Wire Wire Line
	3350 7150 4700 7150
Wire Wire Line
	4700 7150 4700 6700
Wire Wire Line
	4700 6700 4550 6700
Wire Wire Line
	5400 6450 4550 6450
Wire Wire Line
	3000 4200 7050 4200
Wire Wire Line
	3700 1550 2250 1550
Wire Wire Line
	7050 1950 5300 1950
Wire Wire Line
	5300 650  5300 3400
Wire Wire Line
	5300 3400 5200 3400
Wire Wire Line
	5200 3400 5200 3550
Wire Wire Line
	5200 3550 4800 3550
Wire Wire Line
	4800 3550 4800 4100
Wire Wire Line
	4800 4100 2250 4100
Connection ~ 2250 4100
Wire Wire Line
	7050 3150 2400 3150
Wire Wire Line
	2400 3150 2400 2000
Wire Wire Line
	2400 2000 3700 2000
Wire Wire Line
	7050 3300 2450 3300
Wire Wire Line
	2450 3300 2450 2150
Wire Wire Line
	2450 2150 3700 2150
Wire Wire Line
	2350 4500 2350 1850
Wire Wire Line
	2350 1850 3700 1850
Wire Wire Line
	3700 1700 2300 1700
Wire Wire Line
	2300 1700 2300 5000
Connection ~ 2300 5000
Wire Wire Line
	3700 2300 2500 2300
Wire Wire Line
	2500 2300 2500 3000
$Comp
L eSP8266-01 U?
U 1 1 57C27924
P 6750 1600
F 0 "U?" H 8100 2350 60  0000 C CNN
F 1 "eSP8266-01" H 8100 2350 60  0000 C CNN
F 2 "" H 8100 2350 60  0001 C CNN
F 3 "" H 8100 2350 60  0001 C CNN
	1    6750 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 650  5300 650 
Connection ~ 5300 1950
Wire Wire Line
	7350 1300 5300 1300
Connection ~ 5300 1300
$Comp
L R R?
U 1 1 57C27F50
P 6450 900
F 0 "R?" V 6530 900 50  0000 C CNN
F 1 "R" V 6450 900 50  0000 C CNN
F 2 "" V 6380 900 50  0000 C CNN
F 3 "" H 6450 900 50  0000 C CNN
	1    6450 900 
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 750  6450 1400
Wire Wire Line
	6450 1400 7350 1400
Wire Wire Line
	7350 750  3500 750 
Wire Wire Line
	3500 750  3500 1700
Connection ~ 3500 1700
Connection ~ 6450 1050
Connection ~ 6450 750 
Wire Wire Line
	6200 3000 6200 2700
Wire Wire Line
	6200 2700 7050 2700
Connection ~ 6200 3000
Wire Wire Line
	2500 3000 6200 3000
Wire Wire Line
	5300 3450 6250 3450
Wire Wire Line
	6250 3450 6250 2850
Wire Wire Line
	6250 2850 7050 2850
Wire Wire Line
	4850 3600 6300 3600
Wire Wire Line
	6300 3600 6300 3000
Wire Wire Line
	6300 3000 7050 3000
Wire Wire Line
	7350 850  6650 850 
Wire Wire Line
	6650 850  6650 3600
Wire Wire Line
	6650 3600 7050 3600
Wire Wire Line
	7350 950  6800 950 
Wire Wire Line
	6800 950  6800 3450
Wire Wire Line
	6800 3450 7050 3450
Wire Wire Line
	7350 1100 7050 1100
Wire Wire Line
	7050 1100 7050 650 
Connection ~ 7050 650 
Wire Wire Line
	7050 2400 5000 2400
Wire Wire Line
	5000 2400 5000 2950
Wire Wire Line
	5000 2950 3600 2950
Wire Wire Line
	3600 2950 3600 2700
Wire Wire Line
	3600 2700 3700 2700
Wire Wire Line
	3700 2550 3550 2550
Wire Wire Line
	3550 2550 3550 3050
Wire Wire Line
	3550 3050 5050 3050
Wire Wire Line
	5050 3050 5050 2550
Wire Wire Line
	5050 2550 7050 2550
Wire Wire Line
	4550 6300 4950 6300
Wire Wire Line
	4950 6300 4950 7500
Wire Wire Line
	4950 7500 2700 7500
Wire Wire Line
	2700 7500 2700 6000
Connection ~ 2700 6000
Wire Wire Line
	7050 4800 5800 4800
Wire Wire Line
	5800 4800 5800 4500
Wire Wire Line
	5800 4500 2350 4500
Wire Wire Line
	7050 4950 5400 4950
Wire Wire Line
	5400 4950 5400 6450
$EndSCHEMATC
