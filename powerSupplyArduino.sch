EESchema Schematic File Version 4
EELAYER 30 0
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
L MCU_Microchip_ATmega:ATmega8A-AU U?
U 1 1 5DB673AE
P 5350 3600
F 0 "U?" H 5350 5181 50  0000 C CNN
F 1 "ATmega8A-AU" H 5350 5090 50  0000 C CNN
F 2 "Package_QFP:TQFP-32_7x7mm_P0.8mm" H 5350 3600 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Microchip%208bit%20mcu%20AVR%20ATmega8A%20data%20sheet%2040001974A.pdf" H 5350 3600 50  0001 C CNN
	1    5350 3600
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Controller:TL494 U?
U 1 1 5DB68C48
P 8000 3000
F 0 "U?" H 8000 3881 50  0000 C CNN
F 1 "TL494" H 8000 3790 50  0000 C CNN
F 2 "" H 8000 3000 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl494.pdf" H 8000 3000 50  0001 C CNN
	1    8000 3000
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U?
U 1 1 5DB6A13C
P 2400 1750
F 0 "U?" H 2400 2117 50  0000 C CNN
F 1 "LM324" H 2400 2026 50  0000 C CNN
F 2 "" H 2350 1850 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 2450 1950 50  0001 C CNN
	1    2400 1750
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U?
U 2 1 5DB6B6A9
P 3950 1600
F 0 "U?" H 3950 1967 50  0000 C CNN
F 1 "LM324" H 3950 1876 50  0000 C CNN
F 2 "" H 3900 1700 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 4000 1800 50  0001 C CNN
	2    3950 1600
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U?
U 5 1 5DB6C9E5
P 9600 1250
F 0 "U?" H 9558 1296 50  0000 L CNN
F 1 "LM324" H 9558 1205 50  0000 L CNN
F 2 "" H 9550 1350 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 9650 1450 50  0001 C CNN
	5    9600 1250
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:LM324 U?
U 3 1 5DB6DD6D
P 5450 1400
F 0 "U?" H 5450 1767 50  0000 C CNN
F 1 "LM324" H 5450 1676 50  0000 C CNN
F 2 "" H 5400 1500 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/lm2902-n.pdf" H 5500 1600 50  0001 C CNN
	3    5450 1400
	1    0    0    -1  
$EndComp
$Comp
L Display_Character:RC1602A U?
U 1 1 5DB706EC
P 8000 5050
F 0 "U?" H 8000 5931 50  0000 C CNN
F 1 "RC1602A" H 8000 5840 50  0000 C CNN
F 2 "Display:RC1602A" H 8100 4250 50  0001 C CNN
F 3 "http://www.raystar-optronics.com/down.php?ProID=18" H 8100 4950 50  0001 C CNN
	1    8000 5050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DB7146F
P 8600 5250
F 0 "#PWR?" H 8600 5000 50  0001 C CNN
F 1 "GND" H 8605 5077 50  0000 C CNN
F 2 "" H 8600 5250 50  0001 C CNN
F 3 "" H 8600 5250 50  0001 C CNN
	1    8600 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 5250 8600 5250
$Comp
L Connector_Generic:Conn_01x10 J?
U 1 1 5DB74870
P 850 3050
F 0 "J?" H 768 2325 50  0000 C CNN
F 1 "Conn_01x10" H 768 2416 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 850 3050 50  0001 C CNN
F 3 "~" H 850 3050 50  0001 C CNN
	1    850  3050
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DB77870
P 4950 6200
F 0 "#PWR?" H 4950 5950 50  0001 C CNN
F 1 "GND" H 4955 6027 50  0000 C CNN
F 2 "" H 4950 6200 50  0001 C CNN
F 3 "" H 4950 6200 50  0001 C CNN
	1    4950 6200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5DB78ACE
P 4000 6100
F 0 "#PWR?" H 4000 5850 50  0001 C CNN
F 1 "GND" H 4005 5927 50  0000 C CNN
F 2 "" H 4000 6100 50  0001 C CNN
F 3 "" H 4000 6100 50  0001 C CNN
	1    4000 6100
	1    0    0    -1  
$EndComp
$Comp
L Device:Rotary_Encoder_Switch SW?
U 1 1 5DB6F79F
P 4500 6100
F 0 "SW?" H 4500 6467 50  0000 C CNN
F 1 "Rotary_Encoder_Switch" H 4500 6376 50  0000 C CNN
F 2 "" H 4350 6260 50  0001 C CNN
F 3 "~" H 4500 6360 50  0001 C CNN
	1    4500 6100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4200 6100 4000 6100
Wire Wire Line
	4800 6200 4950 6200
$Comp
L Device:C C?
U 1 1 5DB7AC07
P 3600 5950
F 0 "C?" H 3715 5996 50  0000 L CNN
F 1 "0.1uF" H 3715 5905 50  0000 L CNN
F 2 "" H 3638 5800 50  0001 C CNN
F 3 "~" H 3600 5950 50  0001 C CNN
	1    3600 5950
	1    0    0    -1  
$EndComp
Connection ~ 4000 6100
Wire Wire Line
	4000 6100 3600 6100
Wire Wire Line
	3600 5800 3950 5800
Wire Wire Line
	3950 5800 3950 6000
Wire Wire Line
	3950 6000 4200 6000
Wire Wire Line
	4200 6200 4200 6400
Wire Wire Line
	4200 6400 3600 6400
Text GLabel 3600 5800 0    50   Input ~ 0
Encoder_A
Text GLabel 3600 6400 0    50   Input ~ 0
Encoder_B
Text GLabel 4950 5900 2    50   Input ~ 0
Encoder_S
$Comp
L Device:C C?
U 1 1 5DB7E85A
P 3600 6250
F 0 "C?" H 3715 6296 50  0000 L CNN
F 1 "0.1uF" H 3715 6205 50  0000 L CNN
F 2 "" H 3638 6100 50  0001 C CNN
F 3 "~" H 3600 6250 50  0001 C CNN
	1    3600 6250
	1    0    0    -1  
$EndComp
Connection ~ 3600 6100
$Comp
L Device:C C?
U 1 1 5DB7EE31
P 4950 6050
F 0 "C?" H 5065 6096 50  0000 L CNN
F 1 "0.1uF" H 5065 6005 50  0000 L CNN
F 2 "" H 4988 5900 50  0001 C CNN
F 3 "~" H 4950 6050 50  0001 C CNN
	1    4950 6050
	1    0    0    -1  
$EndComp
Connection ~ 4950 6200
Wire Wire Line
	4800 6000 4800 5900
Wire Wire Line
	4800 5900 4950 5900
$EndSCHEMATC
