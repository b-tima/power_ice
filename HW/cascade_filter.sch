EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 7 7
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
L power:GND #PWR0159
U 1 1 621545AD
P 4350 4150
AR Path="/6215432B/621545AD" Ref="#PWR0159"  Part="1" 
AR Path="/6215D384/621545AD" Ref="#PWR0160"  Part="1" 
AR Path="/62163678/621545AD" Ref="#PWR0161"  Part="1" 
AR Path="/621688FA/621545AD" Ref="#PWR0162"  Part="1" 
AR Path="/6216EEB1/621545AD" Ref="#PWR0163"  Part="1" 
AR Path="/62181AF8/621545AD" Ref="#PWR0164"  Part="1" 
F 0 "#PWR0164" H 4350 3900 50  0001 C CNN
F 1 "GND" H 4355 3977 50  0000 C CNN
F 2 "" H 4350 4150 50  0001 C CNN
F 3 "" H 4350 4150 50  0001 C CNN
	1    4350 4150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R18
U 1 1 62154779
P 3900 3750
AR Path="/6215432B/62154779" Ref="R18"  Part="1" 
AR Path="/6215D384/62154779" Ref="R19"  Part="1" 
AR Path="/62163678/62154779" Ref="R20"  Part="1" 
AR Path="/621688FA/62154779" Ref="R21"  Part="1" 
AR Path="/6216EEB1/62154779" Ref="R22"  Part="1" 
AR Path="/62181AF8/62154779" Ref="R23"  Part="1" 
F 0 "R23" V 3700 3650 50  0000 L CNN
F 1 "R" V 3800 3700 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3830 3750 50  0001 C CNN
F 3 "~" H 3900 3750 50  0001 C CNN
	1    3900 3750
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C15
U 1 1 621548ED
P 4350 3900
AR Path="/6215432B/621548ED" Ref="C15"  Part="1" 
AR Path="/6215D384/621548ED" Ref="C16"  Part="1" 
AR Path="/62163678/621548ED" Ref="C17"  Part="1" 
AR Path="/621688FA/621548ED" Ref="C18"  Part="1" 
AR Path="/6216EEB1/621548ED" Ref="C19"  Part="1" 
AR Path="/62181AF8/621548ED" Ref="C20"  Part="1" 
F 0 "C20" H 4465 3946 50  0000 L CNN
F 1 "C" H 4465 3855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4388 3750 50  0001 C CNN
F 3 "~" H 4350 3900 50  0001 C CNN
	1    4350 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4050 3750 4350 3750
Wire Wire Line
	4350 4050 4350 4150
Text HLabel 4900 3750 2    50   Input ~ 0
out
Text HLabel 3300 3750 0    50   Input ~ 0
in
Wire Wire Line
	3300 3750 3750 3750
Wire Wire Line
	4350 3750 4900 3750
Connection ~ 4350 3750
$EndSCHEMATC
