EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 11
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 1200 4900 2150 1350
U 621ED315
F0 "IR_LED" 50
F1 "IR_LED.sch" 50
F2 "InputLedAblock" I L 1200 5550 50 
F3 "InputLedBblock" I L 1200 5800 50 
$EndSheet
$Sheet
S 4050 4900 2150 1350
U 62353F20
F0 "sheet62353F20" 50
F1 "IR_LED.sch" 50
F2 "InputLedAblock" I L 4050 5550 50 
F3 "InputLedBblock" I L 4050 5800 50 
$EndSheet
$Sheet
S 6900 4900 2150 1350
U 6238A8B8
F0 "sheet6238A8B8" 50
F1 "IR_LED.sch" 50
F2 "InputLedAblock" I L 6900 5600 50 
F3 "InputLedBblock" I L 6900 5800 50 
$EndSheet
$Sheet
S 6900 3900 2850 700 
U 6239F98B
F0 "Sheet6239F98A" 50
F1 "PhotodiodeBlock.sch" 50
F2 "OutputPDCircuit" I R 9750 4250 50 
F3 "Jumper1" I L 6900 3950 50 
F4 "Jumper2" I L 6900 4050 50 
F5 "Jumper3" I L 6900 4150 50 
F6 "Jumper4" I L 6900 4250 50 
F7 "Jumper5" I L 6900 4350 50 
F8 "Jumper6" I L 6900 4450 50 
$EndSheet
$Comp
L Connector:Conn_01x13_Female J1
U 1 1 629B96C1
P 1300 3450
F 0 "J1" H 1328 3476 50  0000 L CNN
F 1 "Conn_01x13_Female" H 1328 3385 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x13_P2.54mm_Horizontal" H 1300 3450 50  0001 C CNN
F 3 "~" H 1300 3450 50  0001 C CNN
	1    1300 3450
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x03_Female J2
U 1 1 629BCC72
P 1450 1100
F 0 "J2" H 1342 775 50  0000 C CNN
F 1 "Conn_01x03_Female" H 1342 866 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 1450 1100 50  0001 C CNN
F 3 "~" H 1450 1100 50  0001 C CNN
	1    1450 1100
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0101
U 1 1 629BD6B6
P 2100 900
F 0 "#PWR0101" H 2100 750 50  0001 C CNN
F 1 "+5V" H 2115 1073 50  0000 C CNN
F 2 "" H 2100 900 50  0001 C CNN
F 3 "" H 2100 900 50  0001 C CNN
	1    2100 900 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0102
U 1 1 629BDED0
P 2450 900
F 0 "#PWR0102" H 2450 750 50  0001 C CNN
F 1 "+3.3V" H 2465 1073 50  0000 C CNN
F 2 "" H 2450 900 50  0001 C CNN
F 3 "" H 2450 900 50  0001 C CNN
	1    2450 900 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 629BE0FF
P 2100 1250
F 0 "#PWR0103" H 2100 1000 50  0001 C CNN
F 1 "GND" H 2105 1077 50  0000 C CNN
F 2 "" H 2100 1250 50  0001 C CNN
F 3 "" H 2100 1250 50  0001 C CNN
	1    2100 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1650 1000 1850 1000
Wire Wire Line
	2100 1000 2100 900 
Wire Wire Line
	1650 1100 2250 1100
Wire Wire Line
	2450 1100 2450 1000
Wire Wire Line
	1650 1200 1900 1200
Wire Wire Line
	2100 1200 2100 1250
Wire Wire Line
	1500 4050 1500 4650
Wire Wire Line
	1500 4650 700  4650
Wire Wire Line
	700  4650 700  5800
Wire Wire Line
	700  5800 1200 5800
Wire Wire Line
	1500 3950 1600 3950
Wire Wire Line
	1600 3950 1600 4600
Wire Wire Line
	1600 4600 850  4600
Wire Wire Line
	850  4600 850  5550
Wire Wire Line
	850  5550 1200 5550
Wire Wire Line
	1500 3850 3500 3850
Wire Wire Line
	3500 3850 3500 5800
Wire Wire Line
	3500 5800 4050 5800
Wire Wire Line
	1500 3750 3750 3750
Wire Wire Line
	3750 3750 3750 5550
Wire Wire Line
	3750 5550 4050 5550
Wire Wire Line
	1500 3650 6350 3650
Wire Wire Line
	6350 3650 6350 5800
Wire Wire Line
	6350 5800 6900 5800
Wire Wire Line
	1500 3550 6500 3550
Wire Wire Line
	6500 3550 6500 5600
Wire Wire Line
	6500 5600 6900 5600
Wire Wire Line
	1500 3450 6100 3450
Wire Wire Line
	6100 3450 6100 4450
Wire Wire Line
	6100 4450 6900 4450
Wire Wire Line
	1500 3350 5950 3350
Wire Wire Line
	5950 3350 5950 4350
Wire Wire Line
	5950 4350 6900 4350
Wire Wire Line
	1500 3250 5750 3250
Wire Wire Line
	5750 3250 5750 4250
Wire Wire Line
	5750 4250 6900 4250
Wire Wire Line
	1500 3150 5650 3150
Wire Wire Line
	5650 3150 5650 4150
Wire Wire Line
	5650 4150 6900 4150
Wire Wire Line
	1500 3050 5500 3050
Wire Wire Line
	5500 3050 5500 4050
Wire Wire Line
	5500 4050 6900 4050
Wire Wire Line
	1500 2950 5350 2950
Wire Wire Line
	5350 2950 5350 3950
Wire Wire Line
	5350 3950 6900 3950
Wire Wire Line
	1500 2850 9900 2850
Wire Wire Line
	9900 2850 9900 4250
Wire Wire Line
	9900 4250 9750 4250
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 629C994C
P 1850 1000
F 0 "#FLG0101" H 1850 1075 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 1173 50  0000 C CNN
F 2 "" H 1850 1000 50  0001 C CNN
F 3 "~" H 1850 1000 50  0001 C CNN
	1    1850 1000
	1    0    0    -1  
$EndComp
Connection ~ 1850 1000
Wire Wire Line
	1850 1000 2000 1000
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 629C9C84
P 2250 1100
F 0 "#FLG0102" H 2250 1175 50  0001 C CNN
F 1 "PWR_FLAG" H 2250 1273 50  0000 C CNN
F 2 "" H 2250 1100 50  0001 C CNN
F 3 "~" H 2250 1100 50  0001 C CNN
	1    2250 1100
	1    0    0    -1  
$EndComp
Connection ~ 2250 1100
Wire Wire Line
	2250 1100 2450 1100
$Comp
L power:PWR_FLAG #FLG0103
U 1 1 629C9DB9
P 1900 1200
F 0 "#FLG0103" H 1900 1275 50  0001 C CNN
F 1 "PWR_FLAG" H 1900 1373 50  0000 C CNN
F 2 "" H 1900 1200 50  0001 C CNN
F 3 "~" H 1900 1200 50  0001 C CNN
	1    1900 1200
	1    0    0    -1  
$EndComp
Connection ~ 1900 1200
Wire Wire Line
	1900 1200 2100 1200
$Comp
L pspice:C C15
U 1 1 623B84DD
P 2700 1000
F 0 "C15" V 2385 1000 50  0000 C CNN
F 1 "C" V 2476 1000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2700 1000 50  0001 C CNN
F 3 "~" H 2700 1000 50  0001 C CNN
	1    2700 1000
	0    1    1    0   
$EndComp
Connection ~ 2450 1000
Wire Wire Line
	2450 1000 2450 900 
$Comp
L pspice:C C16
U 1 1 623B8EE2
P 3150 850
F 0 "C16" H 3328 896 50  0000 L CNN
F 1 "C" H 3328 805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3150 850 50  0001 C CNN
F 3 "~" H 3150 850 50  0001 C CNN
	1    3150 850 
	1    0    0    -1  
$EndComp
$Comp
L pspice:C C17
U 1 1 623B9E3F
P 3600 850
F 0 "C17" H 3778 896 50  0000 L CNN
F 1 "C" H 3778 805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3600 850 50  0001 C CNN
F 3 "~" H 3600 850 50  0001 C CNN
	1    3600 850 
	1    0    0    -1  
$EndComp
$Comp
L pspice:C C18
U 1 1 623BA89C
P 4050 850
F 0 "C18" H 4228 896 50  0000 L CNN
F 1 "C" H 4228 805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4050 850 50  0001 C CNN
F 3 "~" H 4050 850 50  0001 C CNN
	1    4050 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 600  2300 600 
Wire Wire Line
	2300 600  2300 1000
Wire Wire Line
	2300 1000 2450 1000
Wire Wire Line
	3600 600  3600 550 
Wire Wire Line
	3600 550  2000 550 
Wire Wire Line
	2000 550  2000 1000
Connection ~ 2000 1000
Wire Wire Line
	2000 1000 2100 1000
Wire Wire Line
	4050 600  3600 600 
Connection ~ 3600 600 
Wire Wire Line
	4050 1100 3600 1100
Wire Wire Line
	3600 1100 3150 1100
Connection ~ 3600 1100
Wire Wire Line
	3150 1100 2950 1100
Wire Wire Line
	2950 1100 2950 1000
Connection ~ 3150 1100
Wire Wire Line
	2950 1100 2950 1200
Wire Wire Line
	2950 1200 2100 1200
Connection ~ 2950 1100
Connection ~ 2100 1200
$EndSCHEMATC
