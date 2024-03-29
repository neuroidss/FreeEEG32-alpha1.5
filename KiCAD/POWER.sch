EESchema Schematic File Version 4
LIBS:FreeEEG32-alpha1.5-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 12 22
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
L da2303:da2303 T1
U 1 1 5CDCD766
P 4400 3250
F 0 "T1" H 4400 3965 50  0000 C CNN
F 1 "da2303" H 4400 3874 50  0000 C CNN
F 2 "SMT_Power_Transformer:SMT Power Transformer" H 4400 3783 50  0000 C CNN
F 3 "" H 4400 3250 50  0000 C CNN
	1    4400 3250
	1    0    0    -1  
$EndComp
$Comp
L sn6505a:SN6505A U12
U 1 1 5CDCD7FC
P 3150 3250
F 0 "U12" H 3150 2807 60  0000 C CNN
F 1 "SN6505A" H 3150 2913 60  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6_Handsoldering" H 3150 3019 60  0000 C CNN
F 3 "" H 3150 3250 60  0000 C CNN
	1    3150 3250
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Schottky D2
U 1 1 5CDCD872
P 5100 3650
F 0 "D2" H 5100 3434 50  0000 C CNN
F 1 "D_Schottky" H 5100 3525 50  0000 C CNN
F 2 "SOD-123:SOD−123" H 5100 3650 50  0001 C CNN
F 3 "~" H 5100 3650 50  0001 C CNN
	1    5100 3650
	-1   0    0    1   
$EndComp
$Comp
L Device:D_Schottky D1
U 1 1 5CDCD8EC
P 5100 2850
F 0 "D1" H 5100 2634 50  0000 C CNN
F 1 "D_Schottky" H 5100 2725 50  0000 C CNN
F 2 "SOD-123:SOD−123" H 5100 2850 50  0001 C CNN
F 3 "~" H 5100 2850 50  0001 C CNN
	1    5100 2850
	-1   0    0    1   
$EndComp
Text HLabel 5350 4650 3    50   Input ~ 0
3_OUT1A2B_DIODES
Text HLabel 5550 4650 3    50   Input ~ 0
4_OUT1B2A
$Comp
L Device:C_Small C?
U 1 1 5CDCDA82
P 5450 4200
AR Path="/5CDBF57D/5CDCDA82" Ref="C?"  Part="1" 
AR Path="/5CE92451/5CDCDA82" Ref="C?"  Part="1" 
AR Path="/5CE92453/5CDCDA82" Ref="C?"  Part="1" 
AR Path="/5CE92455/5CDCDA82" Ref="C?"  Part="1" 
AR Path="/5CDCDA82" Ref="C?"  Part="1" 
AR Path="/5CDACD1E/5CDCDA82" Ref="C?"  Part="1" 
AR Path="/5CDCD748/5CDCDA82" Ref="C114"  Part="1" 
F 0 "C114" H 5542 4246 50  0000 L CNN
F 1 "100n" H 5542 4155 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5450 4200 50  0001 C CNN
F 3 "~" H 5450 4200 50  0001 C CNN
	1    5450 4200
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5CDCDA89
P 5450 4550
AR Path="/5CDBF57D/5CDCDA89" Ref="C?"  Part="1" 
AR Path="/5CE92451/5CDCDA89" Ref="C?"  Part="1" 
AR Path="/5CE92453/5CDCDA89" Ref="C?"  Part="1" 
AR Path="/5CE92455/5CDCDA89" Ref="C?"  Part="1" 
AR Path="/5CDCDA89" Ref="C?"  Part="1" 
AR Path="/5CDACD1E/5CDCDA89" Ref="C?"  Part="1" 
AR Path="/5CDCD748/5CDCDA89" Ref="C115"  Part="1" 
F 0 "C115" H 5542 4596 50  0000 L CNN
F 1 "10u" H 5542 4505 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5450 4550 50  0001 C CNN
F 3 "~" H 5450 4550 50  0001 C CNN
	1    5450 4550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5350 4650 5350 4550
Wire Wire Line
	5550 4650 5550 4550
Wire Wire Line
	5350 4550 5350 4200
Connection ~ 5350 4550
Wire Wire Line
	5550 4550 5550 4200
Connection ~ 5550 4550
Text HLabel 2600 4000 3    50   Input ~ 0
1_GND
Text HLabel 2800 4000 3    50   Input ~ 0
2_VCC
$Comp
L Device:C_Small C?
U 1 1 5CDCDB92
P 2700 3900
AR Path="/5CDBF57D/5CDCDB92" Ref="C?"  Part="1" 
AR Path="/5CE92451/5CDCDB92" Ref="C?"  Part="1" 
AR Path="/5CE92453/5CDCDB92" Ref="C?"  Part="1" 
AR Path="/5CE92455/5CDCDB92" Ref="C?"  Part="1" 
AR Path="/5CDCDB92" Ref="C?"  Part="1" 
AR Path="/5CDACD1E/5CDCDB92" Ref="C?"  Part="1" 
AR Path="/5CDCD748/5CDCDB92" Ref="C113"  Part="1" 
F 0 "C113" H 2792 3946 50  0000 L CNN
F 1 "10u" H 2792 3855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 2700 3900 50  0001 C CNN
F 3 "~" H 2700 3900 50  0001 C CNN
	1    2700 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2600 4000 2600 3900
Wire Wire Line
	2800 4000 2800 3900
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 5CDCDD1E
P 5800 1900
AR Path="/5CDAD75F/5CDCDD1E" Ref="J?"  Part="1" 
AR Path="/5CDCD748/5CDCDD1E" Ref="J36"  Part="1" 
F 0 "J36" H 5880 1892 50  0000 L CNN
F 1 "Conn_01x02" H 5880 1801 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 5800 1900 50  0001 C CNN
F 3 "~" H 5800 1900 50  0001 C CNN
	1    5800 1900
	1    0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J?
U 1 1 5CDCDD25
P 3850 1800
AR Path="/5CDAD75F/5CDCDD25" Ref="J?"  Part="1" 
AR Path="/5CDCD748/5CDCDD25" Ref="J35"  Part="1" 
F 0 "J35" H 3930 1792 50  0000 L CNN
F 1 "Conn_01x02" H 3930 1701 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 3850 1800 50  0001 C CNN
F 3 "~" H 3850 1800 50  0001 C CNN
	1    3850 1800
	-1   0    0    -1  
$EndComp
Text HLabel 4200 1900 2    50   Input ~ 0
2_VCC
Text HLabel 4200 1800 2    50   Input ~ 0
1_GND
Wire Wire Line
	4200 1900 4100 1900
Wire Wire Line
	4200 1800 4100 1800
Text HLabel 5450 1900 0    50   Input ~ 0
3_OUT1A2B_DIODES
Wire Wire Line
	5450 1900 5550 1900
Text HLabel 5450 1800 0    50   Input ~ 0
4_OUT1B2A
Wire Wire Line
	5450 1800 5550 1800
Text HLabel 2650 3150 0    50   Input ~ 0
1_GND
Text HLabel 2650 3250 0    50   Input ~ 0
2_VCC
Wire Wire Line
	2650 3150 2750 3150
Wire Wire Line
	2650 3250 2750 3250
Text HLabel 3650 3250 2    50   Input ~ 0
2_VCC
Wire Wire Line
	3650 3250 3550 3250
Wire Wire Line
	3550 3150 3550 2850
Wire Wire Line
	3550 2850 4000 2850
Wire Wire Line
	3550 3350 3550 3650
Wire Wire Line
	3550 3650 4000 3650
Wire Wire Line
	3650 3250 3650 3150
Wire Wire Line
	3650 3150 4000 3150
Wire Wire Line
	3650 3250 3650 3350
Wire Wire Line
	3650 3350 4000 3350
Connection ~ 3650 3250
Wire Wire Line
	4800 2850 4950 2850
Wire Wire Line
	4800 3650 4950 3650
Wire Wire Line
	4800 3150 4900 3150
Wire Wire Line
	4900 3150 4900 3250
Wire Wire Line
	4900 3350 4800 3350
Text HLabel 5700 3250 2    50   Input ~ 0
3_OUT1A2B_DIODES
Text HLabel 5000 3250 2    50   Input ~ 0
4_OUT1B2A
Wire Wire Line
	4900 3250 5000 3250
Connection ~ 4900 3250
Wire Wire Line
	4900 3250 4900 3350
Wire Wire Line
	5650 3250 5700 3250
Connection ~ 5650 3250
Wire Wire Line
	5650 3250 5650 3650
Wire Wire Line
	5650 2850 5650 3250
Wire Wire Line
	5250 2850 5650 2850
Wire Wire Line
	5250 3650 5650 3650
$EndSCHEMATC
