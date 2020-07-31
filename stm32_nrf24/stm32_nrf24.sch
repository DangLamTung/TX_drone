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
L MCU_ST_STM32F1:STM32F103C8Tx U3
U 1 1 5F1A61C4
P 3150 3350
F 0 "U3" H 3100 1761 50  0000 C CNN
F 1 "STM32F103C8Tx" H 3100 1670 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 2550 1950 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 3150 3350 50  0001 C CNN
	1    3150 3350
	1    0    0    -1  
$EndComp
$Comp
L RF:nRF24L01P U5
U 1 1 5F1A6F1D
P 8400 2450
F 0 "U5" H 8400 3431 50  0000 C CNN
F 1 "nRF24L01P" H 8400 3340 50  0000 C CNN
F 2 "Package_DFN_QFN:QFN-20-1EP_4x4mm_P0.5mm_EP2.5x2.5mm" H 8600 3250 50  0001 L CIN
F 3 "http://www.nordicsemi.com/eng/content/download/2726/34069/file/nRF24L01P_Product_Specification_1_0.pdf" H 8400 2550 50  0001 C CNN
	1    8400 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:C 1.5pF1
U 1 1 5F1A843A
P 10100 1800
F 0 "1.5pF1" V 10352 1800 50  0000 C CNN
F 1 "C" V 10261 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10138 1650 50  0001 C CNN
F 3 "~" H 10100 1800 50  0001 C CNN
	1    10100 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:L 8.2nH1
U 1 1 5F1A9930
P 9300 1950
F 0 "8.2nH1" H 9353 1996 50  0000 L CNN
F 1 "L" H 9353 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9300 1950 50  0001 C CNN
F 3 "~" H 9300 1950 50  0001 C CNN
	1    9300 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 1650 8500 1550
Wire Wire Line
	8500 1550 8400 1550
Wire Wire Line
	8300 1550 8300 1650
Wire Wire Line
	8400 1650 8400 1550
Connection ~ 8400 1550
Wire Wire Line
	8400 1550 8300 1550
Wire Wire Line
	8400 1550 8400 1150
Wire Wire Line
	8300 3250 8300 3350
Wire Wire Line
	8300 3350 8400 3350
Wire Wire Line
	8600 3350 8600 3250
Wire Wire Line
	8500 3250 8500 3350
Connection ~ 8500 3350
Wire Wire Line
	8500 3350 8600 3350
Wire Wire Line
	8400 3250 8400 3350
Connection ~ 8400 3350
Wire Wire Line
	8400 3350 8450 3350
$Comp
L power:GND #PWR0101
U 1 1 5F1AA95F
P 8450 3350
F 0 "#PWR0101" H 8450 3100 50  0001 C CNN
F 1 "GND" H 8455 3177 50  0000 C CNN
F 2 "" H 8450 3350 50  0001 C CNN
F 3 "" H 8450 3350 50  0001 C CNN
	1    8450 3350
	1    0    0    -1  
$EndComp
Connection ~ 8450 3350
Wire Wire Line
	8450 3350 8500 3350
$Comp
L power:+3.3V #PWR0102
U 1 1 5F1AACD7
P 8400 1050
F 0 "#PWR0102" H 8400 900 50  0001 C CNN
F 1 "+3.3V" H 8415 1223 50  0000 C CNN
F 2 "" H 8400 1050 50  0001 C CNN
F 3 "" H 8400 1050 50  0001 C CNN
	1    8400 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C16
U 1 1 5F1AB75E
P 7800 1300
F 0 "C16" H 7915 1346 50  0000 L CNN
F 1 "C" H 7915 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7838 1150 50  0001 C CNN
F 3 "~" H 7800 1300 50  0001 C CNN
	1    7800 1300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C14
U 1 1 5F1ABD70
P 7450 1300
F 0 "C14" H 7565 1346 50  0000 L CNN
F 1 "C" H 7565 1255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7488 1150 50  0001 C CNN
F 3 "~" H 7450 1300 50  0001 C CNN
	1    7450 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8400 1150 7800 1150
Connection ~ 8400 1150
Wire Wire Line
	8400 1150 8400 1050
Connection ~ 7800 1150
Wire Wire Line
	7800 1150 7450 1150
$Comp
L power:GND #PWR0103
U 1 1 5F1AC4CA
P 7800 1500
F 0 "#PWR0103" H 7800 1250 50  0001 C CNN
F 1 "GND" H 7805 1327 50  0000 C CNN
F 2 "" H 7800 1500 50  0001 C CNN
F 3 "" H 7800 1500 50  0001 C CNN
	1    7800 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5F1AC843
P 7450 1500
F 0 "#PWR0104" H 7450 1250 50  0001 C CNN
F 1 "GND" H 7455 1327 50  0000 C CNN
F 2 "" H 7450 1500 50  0001 C CNN
F 3 "" H 7450 1500 50  0001 C CNN
	1    7450 1500
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1450 7800 1500
Wire Wire Line
	7450 1450 7450 1500
Text GLabel 7650 2450 0    50   Input ~ 0
CE
Text GLabel 7650 2250 0    50   Input ~ 0
CSN
Text GLabel 7650 2150 0    50   Input ~ 0
SCK
Text GLabel 7650 2050 0    50   Input ~ 0
MISO
Text GLabel 7650 1950 0    50   Input ~ 0
MOSI
Wire Wire Line
	7650 2450 7800 2450
Wire Wire Line
	7650 2250 7800 2250
Wire Wire Line
	7800 2150 7650 2150
Wire Wire Line
	7650 2050 7800 2050
Wire Wire Line
	7650 1950 7800 1950
$Comp
L Device:R R6
U 1 1 5F1AF34A
P 7500 2900
F 0 "R6" H 7570 2946 50  0000 L CNN
F 1 "R" H 7570 2855 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 7430 2900 50  0001 C CNN
F 3 "~" H 7500 2900 50  0001 C CNN
	1    7500 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2750 7500 2750
$Comp
L power:GND #PWR0105
U 1 1 5F1B038B
P 7500 3150
F 0 "#PWR0105" H 7500 2900 50  0001 C CNN
F 1 "GND" H 7505 2977 50  0000 C CNN
F 2 "" H 7500 3150 50  0001 C CNN
F 3 "" H 7500 3150 50  0001 C CNN
	1    7500 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 3050 7500 3150
Text GLabel 7650 2550 0    50   Input ~ 0
IRQ
Wire Wire Line
	7650 2550 7800 2550
Text GLabel 3900 3550 2    50   Input ~ 0
IRQ
Wire Wire Line
	3900 3550 3750 3550
Wire Wire Line
	9300 2700 9300 2750
Wire Wire Line
	9300 2750 9000 2750
Wire Wire Line
	9000 2950 9300 2950
Wire Wire Line
	9300 2950 9300 3000
Wire Wire Line
	9400 2850 9400 3150
$Comp
L power:GND #PWR0106
U 1 1 5F1B7983
P 9600 3600
F 0 "#PWR0106" H 9600 3350 50  0001 C CNN
F 1 "GND" H 9605 3427 50  0000 C CNN
F 2 "" H 9600 3600 50  0001 C CNN
F 3 "" H 9600 3600 50  0001 C CNN
	1    9600 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 3450 9400 3600
Wire Wire Line
	9400 3600 9600 3600
Wire Wire Line
	9600 3600 9800 3600
Connection ~ 9600 3600
$Comp
L Device:R R7
U 1 1 5F1B9696
P 9350 2850
F 0 "R7" V 9143 2850 50  0000 C CNN
F 1 "1M" V 9234 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 9280 2850 50  0001 C CNN
F 3 "~" H 9350 2850 50  0001 C CNN
	1    9350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 2950 7800 3550
Wire Wire Line
	7800 3550 7100 3550
Wire Wire Line
	7100 3550 7100 1150
Wire Wire Line
	7100 1150 7450 1150
Connection ~ 7450 1150
$Comp
L Device:C C12
U 1 1 5F1BD611
P 6850 1350
F 0 "C12" H 6965 1396 50  0000 L CNN
F 1 "C" H 6965 1305 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T_Pad1.50x2.35mm_HandSolder" H 6888 1200 50  0001 C CNN
F 3 "~" H 6850 1350 50  0001 C CNN
	1    6850 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7100 1150 6850 1150
Wire Wire Line
	6850 1150 6850 1200
Connection ~ 7100 1150
$Comp
L power:GND #PWR0107
U 1 1 5F1BF13F
P 6850 1500
F 0 "#PWR0107" H 6850 1250 50  0001 C CNN
F 1 "GND" H 6855 1327 50  0000 C CNN
F 2 "" H 6850 1500 50  0001 C CNN
F 3 "" H 6850 1500 50  0001 C CNN
	1    6850 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:L 3.9nH1
U 1 1 5F1C1FA6
P 9650 1800
F 0 "3.9nH1" V 9840 1800 50  0000 C CNN
F 1 "L" V 9749 1800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9650 1800 50  0001 C CNN
F 3 "~" H 9650 1800 50  0001 C CNN
	1    9650 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:C 1pF2
U 1 1 5F1C46EF
P 10400 1950
F 0 "1pF2" H 10515 1996 50  0000 L CNN
F 1 "C" H 10515 1905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10438 1800 50  0001 C CNN
F 3 "~" H 10400 1950 50  0001 C CNN
	1    10400 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9000 2150 9100 2150
Wire Wire Line
	9100 2150 9100 2100
Wire Wire Line
	9150 1800 9300 1800
Connection ~ 9300 1800
Wire Wire Line
	9300 1800 9500 1800
Wire Wire Line
	9800 1800 9950 1800
Wire Wire Line
	10250 1800 10400 1800
Connection ~ 10400 1800
Wire Wire Line
	10400 1800 10650 1800
Wire Wire Line
	9000 1950 9050 1950
Wire Wire Line
	3050 1850 3050 1750
Wire Wire Line
	3050 1750 3150 1750
Wire Wire Line
	3350 1750 3350 1850
Wire Wire Line
	3250 1850 3250 1750
Connection ~ 3250 1750
Wire Wire Line
	3250 1750 3350 1750
Wire Wire Line
	3150 1850 3150 1750
Connection ~ 3150 1750
Wire Wire Line
	3150 1750 3200 1750
$Comp
L power:+3.3V #PWR0109
U 1 1 5F1DFD71
P 3200 1600
F 0 "#PWR0109" H 3200 1450 50  0001 C CNN
F 1 "+3.3V" H 3215 1773 50  0000 C CNN
F 2 "" H 3200 1600 50  0001 C CNN
F 3 "" H 3200 1600 50  0001 C CNN
	1    3200 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3200 1600 3200 1750
Connection ~ 3200 1750
Wire Wire Line
	3200 1750 3250 1750
$Comp
L Device:C C2
U 1 1 5F1E5D99
P 1350 2950
F 0 "C2" H 1465 2996 50  0000 L CNN
F 1 "18pF" H 1465 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1388 2800 50  0001 C CNN
F 3 "~" H 1350 2950 50  0001 C CNN
	1    1350 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5F1E5D9F
P 1150 3250
F 0 "#PWR0110" H 1150 3000 50  0001 C CNN
F 1 "GND" H 1155 3077 50  0000 C CNN
F 2 "" H 1150 3250 50  0001 C CNN
F 3 "" H 1150 3250 50  0001 C CNN
	1    1150 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	950  3100 950  3250
Wire Wire Line
	950  3250 1150 3250
Wire Wire Line
	1150 3250 1350 3250
Wire Wire Line
	1350 3250 1350 3100
Connection ~ 1150 3250
$Comp
L Device:R R1
U 1 1 5F1E5DAA
P 1450 2350
F 0 "R1" V 1243 2350 50  0000 C CNN
F 1 "1M" V 1334 2350 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1380 2350 50  0001 C CNN
F 3 "~" H 1450 2350 50  0001 C CNN
	1    1450 2350
	-1   0    0    1   
$EndComp
$Comp
L Device:Crystal_GND24 Y1
U 1 1 5F1E5D85
P 1150 2350
F 0 "Y1" V 1104 2594 50  0000 L CNN
F 1 "Crystal_GND24" V 1195 2594 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_2520-4Pin_2.5x2.0mm" H 1150 2350 50  0001 C CNN
F 3 "~" H 1150 2350 50  0001 C CNN
	1    1150 2350
	0    1    1    0   
$EndComp
Wire Wire Line
	1350 2350 1350 2800
Wire Wire Line
	950  2350 950  2800
Wire Wire Line
	2150 2450 2450 2450
Wire Wire Line
	2450 2550 2100 2550
Wire Wire Line
	1150 2200 1450 2200
Wire Wire Line
	1600 2150 1600 2400
Wire Wire Line
	1600 2400 2150 2400
Wire Wire Line
	2150 2400 2150 2450
Text GLabel 3900 3750 2    50   Input ~ 0
MISO
Text GLabel 3900 3850 2    50   Input ~ 0
MOSI
Wire Wire Line
	3750 3650 3900 3650
Wire Wire Line
	3900 3750 3750 3750
Wire Wire Line
	3900 3850 3750 3850
Text GLabel 3900 3650 2    50   Input ~ 0
SCK
$Comp
L Connector:USB_B_Micro J1
U 1 1 5F23AAA0
P 900 6700
F 0 "J1" H 957 7167 50  0000 C CNN
F 1 "USB_B_Micro" H 957 7076 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_GCT_USB3076-30-A" H 1050 6650 50  0001 C CNN
F 3 "~" H 1050 6650 50  0001 C CNN
	1    900  6700
	1    0    0    -1  
$EndComp
Text GLabel 1450 6700 2    50   Input ~ 0
D+
Text GLabel 1450 6800 2    50   Input ~ 0
D-
Wire Wire Line
	1200 6700 1450 6700
Wire Wire Line
	1450 6800 1200 6800
Wire Wire Line
	1200 6500 1400 6500
Wire Wire Line
	1400 6500 1400 6300
$Comp
L power:+5V #PWR0111
U 1 1 5F247937
P 1400 6300
F 0 "#PWR0111" H 1400 6150 50  0001 C CNN
F 1 "+5V" H 1415 6473 50  0000 C CNN
F 2 "" H 1400 6300 50  0001 C CNN
F 3 "" H 1400 6300 50  0001 C CNN
	1    1400 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5F2480AE
P 900 7100
F 0 "#PWR0112" H 900 6850 50  0001 C CNN
F 1 "GND" H 905 6927 50  0000 C CNN
F 2 "" H 900 7100 50  0001 C CNN
F 3 "" H 900 7100 50  0001 C CNN
	1    900  7100
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:AMS1117-3.3 U2
U 1 1 5F248C24
P 4600 6450
F 0 "U2" H 4600 6692 50  0000 C CNN
F 1 "AMS1117-3.3" H 4600 6601 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 4600 6650 50  0001 C CNN
F 3 "http://www.advanced-monolithic.com/pdf/ds1117.pdf" H 4700 6200 50  0001 C CNN
	1    4600 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D1
U 1 1 5F24A4FD
P 5950 6750
F 0 "D1" V 5989 6632 50  0000 R CNN
F 1 "LED" V 5898 6632 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 5950 6750 50  0001 C CNN
F 3 "~" H 5950 6750 50  0001 C CNN
	1    5950 6750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5F24B22B
P 5950 6450
F 0 "R3" V 5743 6450 50  0000 C CNN
F 1 "R" V 5834 6450 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 5880 6450 50  0001 C CNN
F 3 "~" H 5950 6450 50  0001 C CNN
	1    5950 6450
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 5F24F516
P 5950 6900
F 0 "#PWR0113" H 5950 6650 50  0001 C CNN
F 1 "GND" H 5955 6727 50  0000 C CNN
F 2 "" H 5950 6900 50  0001 C CNN
F 3 "" H 5950 6900 50  0001 C CNN
	1    5950 6900
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0114
U 1 1 5F24FD13
P 5950 6300
F 0 "#PWR0114" H 5950 6150 50  0001 C CNN
F 1 "+3.3V" H 5965 6473 50  0000 C CNN
F 2 "" H 5950 6300 50  0001 C CNN
F 3 "" H 5950 6300 50  0001 C CNN
	1    5950 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C7
U 1 1 5F254B2B
P 5000 6600
F 0 "C7" H 5115 6646 50  0000 L CNN
F 1 "C" H 5115 6555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 5038 6450 50  0001 C CNN
F 3 "~" H 5000 6600 50  0001 C CNN
	1    5000 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5F25537C
P 4150 6600
F 0 "C5" H 4265 6646 50  0000 L CNN
F 1 "C" H 4265 6555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4188 6450 50  0001 C CNN
F 3 "~" H 4150 6600 50  0001 C CNN
	1    4150 6600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5F255BE0
P 4600 6850
F 0 "#PWR0115" H 4600 6600 50  0001 C CNN
F 1 "GND" H 4605 6677 50  0000 C CNN
F 2 "" H 4600 6850 50  0001 C CNN
F 3 "" H 4600 6850 50  0001 C CNN
	1    4600 6850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 6750 4600 6750
Connection ~ 4600 6750
Wire Wire Line
	4600 6750 5000 6750
Wire Wire Line
	5000 6450 4900 6450
Wire Wire Line
	4300 6450 4150 6450
$Comp
L power:+5V #PWR0116
U 1 1 5F262038
P 3800 6450
F 0 "#PWR0116" H 3800 6300 50  0001 C CNN
F 1 "+5V" H 3815 6623 50  0000 C CNN
F 2 "" H 3800 6450 50  0001 C CNN
F 3 "" H 3800 6450 50  0001 C CNN
	1    3800 6450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0117
U 1 1 5F262A55
P 5400 6450
F 0 "#PWR0117" H 5400 6300 50  0001 C CNN
F 1 "+3.3V" H 5415 6623 50  0000 C CNN
F 2 "" H 5400 6450 50  0001 C CNN
F 3 "" H 5400 6450 50  0001 C CNN
	1    5400 6450
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C4
U 1 1 5F26360B
P 3800 6600
F 0 "C4" H 3918 6646 50  0000 L CNN
F 1 "CP" H 3918 6555 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T_Pad1.50x2.35mm_HandSolder" H 3838 6450 50  0001 C CNN
F 3 "~" H 3800 6600 50  0001 C CNN
	1    3800 6600
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C9
U 1 1 5F26443B
P 5400 6600
F 0 "C9" H 5518 6646 50  0000 L CNN
F 1 "CP" H 5518 6555 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T_Pad1.50x2.35mm_HandSolder" H 5438 6450 50  0001 C CNN
F 3 "~" H 5400 6600 50  0001 C CNN
	1    5400 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	3800 6450 4150 6450
Connection ~ 4150 6450
Wire Wire Line
	4150 6750 3800 6750
Connection ~ 4150 6750
Wire Wire Line
	5000 6450 5400 6450
Connection ~ 5000 6450
Wire Wire Line
	5400 6750 5000 6750
Connection ~ 5000 6750
Connection ~ 5400 6450
Connection ~ 3800 6450
$Comp
L Power_Protection:USBLC6-2SC6 U1
U 1 1 5F29ACE3
P 2550 6750
F 0 "U1" H 2550 7331 50  0000 C CNN
F 1 "USBLC6-2SC6" H 2550 7240 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 2550 6250 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/usblc6-2.pdf" H 2750 7100 50  0001 C CNN
	1    2550 6750
	1    0    0    -1  
$EndComp
Text GLabel 2050 6650 0    50   Input ~ 0
D+
Text GLabel 2050 6850 0    50   Input ~ 0
USB_D+
Wire Wire Line
	2050 6650 2150 6650
Wire Wire Line
	2150 6850 2050 6850
Text GLabel 3050 6650 2    50   Input ~ 0
D-
Text GLabel 3050 6850 2    50   Input ~ 0
USB_D-
Wire Wire Line
	3050 6650 2950 6650
Wire Wire Line
	2950 6850 3050 6850
$Comp
L power:GND #PWR0118
U 1 1 5F2F7EFA
P 10450 5900
F 0 "#PWR0118" H 10450 5650 50  0001 C CNN
F 1 "GND" H 10455 5727 50  0000 C CNN
F 2 "" H 10450 5900 50  0001 C CNN
F 3 "" H 10450 5900 50  0001 C CNN
	1    10450 5900
	1    0    0    -1  
$EndComp
$Comp
L stm32_nrf24-rescue:RFX2401C-RFX2401C U4
U 1 1 5F311FA0
P 7350 5600
F 0 "U4" H 7350 6167 50  0000 C CNN
F 1 "RFX2401C" H 7350 6076 50  0000 C CNN
F 2 "stm32_nrf24:QFN50P300X300X60-17N" H 7350 5600 50  0001 L BNN
F 3 "https://snapeda.com/shop?store=Mouser&id=556493" H 7350 5600 50  0001 L BNN
F 4 "None" H 7350 5600 50  0001 L BNN "Field4"
F 5 "Unavailable" H 7350 5600 50  0001 L BNN "Field5"
F 6 "RF Front End 2.4GHz ~ 2.5GHz 802.15.4, Zigbee® 16-QFN (3x3)" H 7350 5600 50  0001 L BNN "Field6"
F 7 "RFX2401C" H 7350 5600 50  0001 L BNN "Field7"
F 8 "https://snapeda.com/shop?store=DigiKey&id=556493" H 7350 5600 50  0001 L BNN "Field8"
F 9 "UFQFN-16 Skyworks Solutions" H 7350 5600 50  0001 L BNN "Field9"
F 10 "Skyworks Solutions" H 7350 5600 50  0001 L BNN "Field10"
	1    7350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 5300 8050 4500
$Comp
L power:+3.3V #PWR0119
U 1 1 5F327E7B
P 8050 4500
F 0 "#PWR0119" H 8050 4350 50  0001 C CNN
F 1 "+3.3V" H 8065 4673 50  0000 C CNN
F 2 "" H 8050 4500 50  0001 C CNN
F 3 "" H 8050 4500 50  0001 C CNN
	1    8050 4500
	1    0    0    -1  
$EndComp
Connection ~ 8050 4500
$Comp
L Device:C C17
U 1 1 5F328877
P 7800 4650
F 0 "C17" H 7915 4696 50  0000 L CNN
F 1 "C" H 7915 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7838 4500 50  0001 C CNN
F 3 "~" H 7800 4650 50  0001 C CNN
	1    7800 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:C C15
U 1 1 5F32929D
P 7450 4650
F 0 "C15" H 7565 4696 50  0000 L CNN
F 1 "C" H 7565 4605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 7488 4500 50  0001 C CNN
F 3 "~" H 7450 4650 50  0001 C CNN
	1    7450 4650
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C13
U 1 1 5F329D17
P 7050 4650
F 0 "C13" H 7168 4696 50  0000 L CNN
F 1 "CP" H 7168 4605 50  0000 L CNN
F 2 "Capacitor_Tantalum_SMD:CP_EIA-3528-12_Kemet-T_Pad1.50x2.35mm_HandSolder" H 7088 4500 50  0001 C CNN
F 3 "~" H 7050 4650 50  0001 C CNN
	1    7050 4650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5F32A3AB
P 7800 4800
F 0 "#PWR0120" H 7800 4550 50  0001 C CNN
F 1 "GND" H 7805 4627 50  0000 C CNN
F 2 "" H 7800 4800 50  0001 C CNN
F 3 "" H 7800 4800 50  0001 C CNN
	1    7800 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5F32ACE2
P 7450 4800
F 0 "#PWR0121" H 7450 4550 50  0001 C CNN
F 1 "GND" H 7455 4627 50  0000 C CNN
F 2 "" H 7450 4800 50  0001 C CNN
F 3 "" H 7450 4800 50  0001 C CNN
	1    7450 4800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5F32B43A
P 7050 4800
F 0 "#PWR0122" H 7050 4550 50  0001 C CNN
F 1 "GND" H 7055 4627 50  0000 C CNN
F 2 "" H 7050 4800 50  0001 C CNN
F 3 "" H 7050 4800 50  0001 C CNN
	1    7050 4800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 4500 7450 4500
Connection ~ 7450 4500
Wire Wire Line
	7450 4500 7800 4500
Connection ~ 7800 4500
Wire Wire Line
	7800 4500 8050 4500
$Comp
L power:GND #PWR0123
U 1 1 5F336507
P 8200 5950
F 0 "#PWR0123" H 8200 5700 50  0001 C CNN
F 1 "GND" H 8205 5777 50  0000 C CNN
F 2 "" H 8200 5950 50  0001 C CNN
F 3 "" H 8200 5950 50  0001 C CNN
	1    8200 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8050 5900 8200 5900
Wire Wire Line
	8200 5900 8200 5950
$Comp
L Device:L L1
U 1 1 5F354D4D
P 8850 5700
F 0 "L1" V 9040 5700 50  0000 C CNN
F 1 "L" V 8949 5700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8850 5700 50  0001 C CNN
F 3 "~" H 8850 5700 50  0001 C CNN
	1    8850 5700
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C18
U 1 1 5F355AD9
P 8650 5900
F 0 "C18" H 8765 5946 50  0000 L CNN
F 1 "C" H 8765 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8688 5750 50  0001 C CNN
F 3 "~" H 8650 5900 50  0001 C CNN
	1    8650 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C19
U 1 1 5F3563EF
P 9050 5900
F 0 "C19" H 9165 5946 50  0000 L CNN
F 1 "C" H 9165 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9088 5750 50  0001 C CNN
F 3 "~" H 9050 5900 50  0001 C CNN
	1    9050 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C21
U 1 1 5F356E2A
P 9400 5900
F 0 "C21" H 9515 5946 50  0000 L CNN
F 1 "C" H 9515 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9438 5750 50  0001 C CNN
F 3 "~" H 9400 5900 50  0001 C CNN
	1    9400 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C23
U 1 1 5F35C9A2
P 9950 5900
F 0 "C23" H 10065 5946 50  0000 L CNN
F 1 "C" H 10065 5855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9988 5750 50  0001 C CNN
F 3 "~" H 9950 5900 50  0001 C CNN
	1    9950 5900
	1    0    0    -1  
$EndComp
$Comp
L Device:L L2
U 1 1 5F35D20C
P 9700 5700
F 0 "L2" V 9890 5700 50  0000 C CNN
F 1 "L" V 9799 5700 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9700 5700 50  0001 C CNN
F 3 "~" H 9700 5700 50  0001 C CNN
	1    9700 5700
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8650 5750 8650 5700
Wire Wire Line
	8650 5700 8700 5700
Wire Wire Line
	9000 5700 9050 5700
Wire Wire Line
	9400 5750 9400 5700
Connection ~ 9400 5700
Wire Wire Line
	9400 5700 9550 5700
Wire Wire Line
	9050 5750 9050 5700
Connection ~ 9050 5700
Wire Wire Line
	9050 5700 9400 5700
Wire Wire Line
	9950 5750 9950 5700
Connection ~ 9950 5700
Wire Wire Line
	9950 5700 9850 5700
$Comp
L power:GND #PWR0124
U 1 1 5F38A74E
P 9950 6050
F 0 "#PWR0124" H 9950 5800 50  0001 C CNN
F 1 "GND" H 9955 5877 50  0000 C CNN
F 2 "" H 9950 6050 50  0001 C CNN
F 3 "" H 9950 6050 50  0001 C CNN
	1    9950 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5F38AF7B
P 9400 6050
F 0 "#PWR0125" H 9400 5800 50  0001 C CNN
F 1 "GND" H 9405 5877 50  0000 C CNN
F 2 "" H 9400 6050 50  0001 C CNN
F 3 "" H 9400 6050 50  0001 C CNN
	1    9400 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 5F38B805
P 9050 6050
F 0 "#PWR0126" H 9050 5800 50  0001 C CNN
F 1 "GND" H 9055 5877 50  0000 C CNN
F 2 "" H 9050 6050 50  0001 C CNN
F 3 "" H 9050 6050 50  0001 C CNN
	1    9050 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5F38C018
P 8650 6050
F 0 "#PWR0127" H 8650 5800 50  0001 C CNN
F 1 "GND" H 8655 5877 50  0000 C CNN
F 2 "" H 8650 6050 50  0001 C CNN
F 3 "" H 8650 6050 50  0001 C CNN
	1    8650 6050
	1    0    0    -1  
$EndComp
Text GLabel 10650 1800 2    50   Input ~ 0
RF_out
Text GLabel 8200 5500 2    50   Input ~ 0
RF_out
Wire Wire Line
	8050 5500 8200 5500
Text GLabel 6200 5500 0    50   Input ~ 0
CE
Wire Wire Line
	6200 5500 6350 5500
Wire Wire Line
	6350 5600 6200 5600
Wire Wire Line
	2950 4850 2950 5150
Wire Wire Line
	2950 5150 3050 5150
Wire Wire Line
	3250 5150 3250 4850
Wire Wire Line
	3150 4850 3150 5150
Connection ~ 3150 5150
Wire Wire Line
	3150 5150 3250 5150
Wire Wire Line
	3050 4850 3050 5150
Connection ~ 3050 5150
Wire Wire Line
	3050 5150 3100 5150
$Comp
L power:GND #PWR0129
U 1 1 5F45E150
P 3100 5150
F 0 "#PWR0129" H 3100 4900 50  0001 C CNN
F 1 "GND" H 3105 4977 50  0000 C CNN
F 2 "" H 3100 5150 50  0001 C CNN
F 3 "" H 3100 5150 50  0001 C CNN
	1    3100 5150
	1    0    0    -1  
$EndComp
Connection ~ 3100 5150
Wire Wire Line
	3100 5150 3150 5150
$Comp
L Device:C C8
U 1 1 5F471439
P 3750 1000
F 0 "C8" H 3865 1046 50  0000 L CNN
F 1 "C" H 3865 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3788 850 50  0001 C CNN
F 3 "~" H 3750 1000 50  0001 C CNN
	1    3750 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5F47143F
P 3400 1000
F 0 "C6" H 3515 1046 50  0000 L CNN
F 1 "C" H 3515 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3438 850 50  0001 C CNN
F 3 "~" H 3400 1000 50  0001 C CNN
	1    3400 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1150 3750 1200
Wire Wire Line
	3400 1150 3400 1200
$Comp
L Device:C C11
U 1 1 5F478AEB
P 4500 1000
F 0 "C11" H 4615 1046 50  0000 L CNN
F 1 "C" H 4615 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4538 850 50  0001 C CNN
F 3 "~" H 4500 1000 50  0001 C CNN
	1    4500 1000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5F478AF1
P 4150 1000
F 0 "C10" H 4265 1046 50  0000 L CNN
F 1 "C" H 4265 955 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 4188 850 50  0001 C CNN
F 3 "~" H 4150 1000 50  0001 C CNN
	1    4150 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4500 1150 4500 1200
Wire Wire Line
	4150 1150 4150 1200
Wire Wire Line
	3400 1200 3750 1200
Connection ~ 3750 1200
Wire Wire Line
	3750 1200 3950 1200
Connection ~ 4150 1200
Wire Wire Line
	4150 1200 4500 1200
$Comp
L power:GND #PWR0130
U 1 1 5F488B10
P 3950 1200
F 0 "#PWR0130" H 3950 950 50  0001 C CNN
F 1 "GND" H 3955 1027 50  0000 C CNN
F 2 "" H 3950 1200 50  0001 C CNN
F 3 "" H 3950 1200 50  0001 C CNN
	1    3950 1200
	1    0    0    -1  
$EndComp
Connection ~ 3950 1200
Wire Wire Line
	3950 1200 4150 1200
$Comp
L power:+3.3V #PWR0131
U 1 1 5F4895A0
P 3950 850
F 0 "#PWR0131" H 3950 700 50  0001 C CNN
F 1 "+3.3V" H 3965 1023 50  0000 C CNN
F 2 "" H 3950 850 50  0001 C CNN
F 3 "" H 3950 850 50  0001 C CNN
	1    3950 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	3400 850  3750 850 
Connection ~ 3750 850 
Wire Wire Line
	3750 850  3950 850 
Connection ~ 3950 850 
Wire Wire Line
	3950 850  4150 850 
Connection ~ 4150 850 
Wire Wire Line
	4150 850  4500 850 
$Comp
L Switch:SW_Push SW1
U 1 1 5F4A3EC7
P 1550 1350
F 0 "SW1" H 1550 1635 50  0000 C CNN
F 1 "SW_Push" H 1550 1544 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 1550 1550 50  0001 C CNN
F 3 "~" H 1550 1550 50  0001 C CNN
	1    1550 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5F4AD302
P 1750 1600
F 0 "C3" H 1865 1646 50  0000 L CNN
F 1 "C" H 1865 1555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 1788 1450 50  0001 C CNN
F 3 "~" H 1750 1600 50  0001 C CNN
	1    1750 1600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5F4AE03A
P 1750 1100
F 0 "R2" H 1820 1146 50  0000 L CNN
F 1 "R" H 1820 1055 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 1680 1100 50  0001 C CNN
F 3 "~" H 1750 1100 50  0001 C CNN
	1    1750 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 5F4AE9D9
P 1750 1850
F 0 "#PWR0132" H 1750 1600 50  0001 C CNN
F 1 "GND" H 1755 1677 50  0000 C CNN
F 2 "" H 1750 1850 50  0001 C CNN
F 3 "" H 1750 1850 50  0001 C CNN
	1    1750 1850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0133
U 1 1 5F4AF34F
P 1750 850
F 0 "#PWR0133" H 1750 700 50  0001 C CNN
F 1 "+3.3V" H 1765 1023 50  0000 C CNN
F 2 "" H 1750 850 50  0001 C CNN
F 3 "" H 1750 850 50  0001 C CNN
	1    1750 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 1750 1750 1800
Wire Wire Line
	1750 1450 1750 1350
Connection ~ 1750 1350
Wire Wire Line
	1750 1350 1750 1250
Wire Wire Line
	1750 850  1750 950 
Wire Wire Line
	1350 1350 1350 1800
Wire Wire Line
	1350 1800 1750 1800
Connection ~ 1750 1800
Wire Wire Line
	1750 1800 1750 1850
Text GLabel 2000 1350 2    50   Input ~ 0
NRST
Wire Wire Line
	1750 1350 2000 1350
Text GLabel 2200 2050 0    50   Input ~ 0
NRST
Wire Wire Line
	2450 2050 2200 2050
Wire Wire Line
	4600 6750 4600 6850
$Comp
L Device:R R4
U 1 1 5F525865
P 6500 5500
F 0 "R4" V 6293 5500 50  0000 C CNN
F 1 "R" V 6384 5500 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6430 5500 50  0001 C CNN
F 3 "~" H 6500 5500 50  0001 C CNN
	1    6500 5500
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 5F5450A5
P 6500 5600
F 0 "R5" V 6293 5600 50  0000 C CNN
F 1 "R" V 6384 5600 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 6430 5600 50  0001 C CNN
F 3 "~" H 6500 5600 50  0001 C CNN
	1    6500 5600
	0    1    1    0   
$EndComp
Text GLabel 6200 5600 0    50   Input ~ 0
TXEN
Wire Notes Line
	6550 4000 11000 4000
Wire Notes Line
	11000 4000 11000 800 
Wire Notes Line
	11000 800  6550 800 
Wire Notes Line
	6550 800  6550 4000
Text GLabel 2250 2250 0    50   Input ~ 0
BOOT0
Wire Wire Line
	2450 2250 2250 2250
Text GLabel 4200 1800 0    50   Input ~ 0
BOOT0
Wire Wire Line
	4400 1800 4200 1800
$Comp
L Connector:Conn_01x03_Male J2
U 1 1 5F5FB6A0
P 4600 1800
F 0 "J2" H 4572 1732 50  0000 R CNN
F 1 "Conn_01x03_Male" H 4572 1823 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 4600 1800 50  0001 C CNN
F 3 "~" H 4600 1800 50  0001 C CNN
	1    4600 1800
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 5F5FC520
P 4250 2000
F 0 "#PWR0135" H 4250 1750 50  0001 C CNN
F 1 "GND" H 4255 1827 50  0000 C CNN
F 2 "" H 4250 2000 50  0001 C CNN
F 3 "" H 4250 2000 50  0001 C CNN
	1    4250 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 2000 4250 1900
Wire Wire Line
	4250 1900 4400 1900
$Comp
L power:+3.3V #PWR0136
U 1 1 5F60944F
P 4250 1600
F 0 "#PWR0136" H 4250 1450 50  0001 C CNN
F 1 "+3.3V" H 4265 1773 50  0000 C CNN
F 2 "" H 4250 1600 50  0001 C CNN
F 3 "" H 4250 1600 50  0001 C CNN
	1    4250 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 1700 4250 1700
Wire Wire Line
	4250 1700 4250 1600
Text GLabel 3850 4250 2    50   Input ~ 0
USB_D-
Wire Wire Line
	3750 4250 3850 4250
Text GLabel 3850 4350 2    50   Input ~ 0
USB_D+
Wire Wire Line
	3750 4350 3850 4350
Wire Wire Line
	10350 2850 10550 2850
Connection ~ 10350 2850
Wire Wire Line
	10550 2850 10550 2800
Wire Wire Line
	10150 2850 10350 2850
Wire Wire Line
	10150 2800 10150 2850
$Comp
L power:GND #PWR0108
U 1 1 5F1D78A0
P 10350 2850
F 0 "#PWR0108" H 10350 2600 50  0001 C CNN
F 1 "GND" H 10355 2677 50  0000 C CNN
F 2 "" H 10350 2850 50  0001 C CNN
F 3 "" H 10350 2850 50  0001 C CNN
	1    10350 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1950 9050 2600
Wire Wire Line
	9100 2100 9300 2100
Connection ~ 10550 2500
Wire Wire Line
	10200 2150 10550 2150
Wire Wire Line
	10200 2100 10200 2150
Wire Wire Line
	9800 2100 10200 2100
Wire Wire Line
	10400 2100 10600 2100
$Comp
L power:GND #PWR0128
U 1 1 5F38D115
P 10600 2100
F 0 "#PWR0128" H 10600 1850 50  0001 C CNN
F 1 "GND" V 10605 1972 50  0000 R CNN
F 2 "" H 10600 2100 50  0001 C CNN
F 3 "" H 10600 2100 50  0001 C CNN
	1    10600 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10150 2500 10550 2500
Connection ~ 10150 2500
Wire Wire Line
	10000 2500 10150 2500
$Comp
L Device:C 1pF3
U 1 1 5F1D4D67
P 10550 2650
F 0 "1pF3" H 10665 2696 50  0000 L CNN
F 1 "C" H 10665 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10588 2500 50  0001 C CNN
F 3 "~" H 10550 2650 50  0001 C CNN
	1    10550 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:C 1pF1
U 1 1 5F1D3542
P 10150 2650
F 0 "1pF1" H 10265 2696 50  0000 L CNN
F 1 "C" H 10265 2605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 10188 2500 50  0001 C CNN
F 3 "~" H 10150 2650 50  0001 C CNN
	1    10150 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	10000 2600 10000 2500
Wire Wire Line
	9050 2600 10000 2600
$Comp
L Device:L 2.7nH1
U 1 1 5F1CFDF6
P 9650 2100
F 0 "2.7nH1" V 9840 2100 50  0000 C CNN
F 1 "L" V 9749 2100 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 9650 2100 50  0001 C CNN
F 3 "~" H 9650 2100 50  0001 C CNN
	1    9650 2100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10550 2150 10550 2450
Connection ~ 9300 2100
Wire Wire Line
	9300 2100 9500 2100
Wire Wire Line
	9000 2350 9150 2350
Wire Wire Line
	9150 2350 9150 1800
Wire Wire Line
	10550 2450 10700 2450
Text GLabel 10700 2450 2    50   Input ~ 0
TXEN
Connection ~ 10550 2450
Wire Wire Line
	10550 2450 10550 2500
Text GLabel 3900 4550 2    50   Input ~ 0
SWCLK
Wire Wire Line
	3900 4550 3750 4550
Wire Wire Line
	3750 4450 3900 4450
Text GLabel 3900 4450 2    50   Input ~ 0
SWDIO
$Comp
L Connector:Conn_01x10_Male J4
U 1 1 5F2B694D
P 4900 3000
F 0 "J4" H 5008 3581 50  0000 C CNN
F 1 "Conn_01x10_Male" H 5008 3490 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x10_P2.54mm_Vertical" H 4900 3000 50  0001 C CNN
F 3 "~" H 4900 3000 50  0001 C CNN
	1    4900 3000
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J5
U 1 1 5F310E28
P 5150 4450
F 0 "J5" H 5178 4426 50  0000 L CNN
F 1 "Conn_01x04_Female" H 5178 4335 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 5150 4450 50  0001 C CNN
F 3 "~" H 5150 4450 50  0001 C CNN
	1    5150 4450
	1    0    0    -1  
$EndComp
Text GLabel 4800 4450 0    50   Input ~ 0
SWCLK
Wire Wire Line
	4800 4450 4950 4450
Wire Wire Line
	4950 4550 4800 4550
Text GLabel 4800 4550 0    50   Input ~ 0
SWDIO
$Comp
L power:+3.3V #PWR0137
U 1 1 5F33BFFB
P 4850 4250
F 0 "#PWR0137" H 4850 4100 50  0001 C CNN
F 1 "+3.3V" H 4865 4423 50  0000 C CNN
F 2 "" H 4850 4250 50  0001 C CNN
F 3 "" H 4850 4250 50  0001 C CNN
	1    4850 4250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0138
U 1 1 5F33C780
P 4800 4750
F 0 "#PWR0138" H 4800 4500 50  0001 C CNN
F 1 "GND" H 4805 4577 50  0000 C CNN
F 2 "" H 4800 4750 50  0001 C CNN
F 3 "" H 4800 4750 50  0001 C CNN
	1    4800 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	4850 4250 4850 4350
Wire Wire Line
	4850 4350 4950 4350
Wire Wire Line
	4950 4650 4800 4650
Wire Wire Line
	4800 4650 4800 4750
$Comp
L Device:C C22
U 1 1 5F1B7162
P 9800 3300
F 0 "C22" H 9915 3346 50  0000 L CNN
F 1 "18pF" H 9915 3255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9838 3150 50  0001 C CNN
F 3 "~" H 9800 3300 50  0001 C CNN
	1    9800 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 2850 9800 3150
Wire Wire Line
	9800 3600 9800 3450
$Comp
L Device:C C20
U 1 1 5F1B606E
P 9400 3300
F 0 "C20" H 9515 3346 50  0000 L CNN
F 1 "18pF" H 9515 3255 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 9438 3150 50  0001 C CNN
F 3 "~" H 9400 3300 50  0001 C CNN
	1    9400 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9350 3000 9600 3000
Wire Wire Line
	9600 2700 9350 2700
$Comp
L Device:Crystal_GND24 Y2
U 1 1 5F1B21E0
P 9600 2850
F 0 "Y2" V 9554 3094 50  0000 L CNN
F 1 "Crystal_GND24" V 9645 3094 50  0000 L CNN
F 2 "Crystal:Crystal_SMD_2520-4Pin_2.5x2.0mm" H 9600 2850 50  0001 C CNN
F 3 "~" H 9600 2850 50  0001 C CNN
	1    9600 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	9300 2700 9350 2700
Connection ~ 9350 2700
Wire Wire Line
	9350 3000 9300 3000
Connection ~ 9350 3000
Connection ~ 1450 2500
Wire Wire Line
	1450 2500 1150 2500
$Comp
L Device:C C1
U 1 1 5F1E5D93
P 950 2950
F 0 "C1" H 1065 2996 50  0000 L CNN
F 1 "18pF" H 1065 2905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 988 2800 50  0001 C CNN
F 3 "~" H 950 2950 50  0001 C CNN
	1    950  2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1600 2150 1450 2150
Wire Wire Line
	1450 2150 1450 2200
Connection ~ 1450 2200
Wire Wire Line
	1450 2500 2100 2500
Wire Wire Line
	2100 2500 2100 2550
Wire Wire Line
	2450 4150 2300 4150
Text GLabel 2300 4150 0    50   Input ~ 0
P9
Text GLabel 2300 4250 0    50   Input ~ 0
P8
Wire Wire Line
	2300 4250 2450 4250
Wire Wire Line
	2450 4350 2300 4350
Text GLabel 2300 4350 0    50   Input ~ 0
P7
Text GLabel 2300 4450 0    50   Input ~ 0
P6
Wire Wire Line
	2300 4450 2450 4450
Wire Wire Line
	2450 4550 2300 4550
Text GLabel 2300 4550 0    50   Input ~ 0
P5
Text GLabel 2300 4650 0    50   Input ~ 0
P4
Wire Wire Line
	2300 4650 2450 4650
Wire Wire Line
	3750 3950 3900 3950
Text GLabel 3900 3950 2    50   Input ~ 0
P3
Text GLabel 3900 4050 2    50   Input ~ 0
P2
Wire Wire Line
	3900 4050 3750 4050
Wire Wire Line
	3750 4150 3900 4150
Text GLabel 3900 4150 2    50   Input ~ 0
P1
Text GLabel 2300 3350 0    50   Input ~ 0
P10
Wire Wire Line
	2300 3350 2450 3350
Text GLabel 5250 3500 2    50   Input ~ 0
P10
Wire Wire Line
	5250 3500 5100 3500
Wire Wire Line
	5100 3400 5250 3400
Text GLabel 5250 3400 2    50   Input ~ 0
P9
Text GLabel 5250 3300 2    50   Input ~ 0
P8
Wire Wire Line
	5250 3300 5100 3300
Wire Wire Line
	5100 3200 5250 3200
Text GLabel 5250 3200 2    50   Input ~ 0
P7
Text GLabel 5250 3100 2    50   Input ~ 0
P6
Wire Wire Line
	5250 3100 5100 3100
Wire Wire Line
	5100 3000 5250 3000
Text GLabel 5250 3000 2    50   Input ~ 0
P5
Text GLabel 5250 2900 2    50   Input ~ 0
P4
Wire Wire Line
	5250 2900 5100 2900
Wire Wire Line
	5100 2800 5250 2800
Text GLabel 5250 2800 2    50   Input ~ 0
P3
Text GLabel 5250 2700 2    50   Input ~ 0
P2
Wire Wire Line
	5250 2700 5100 2700
Wire Wire Line
	5100 2600 5250 2600
Text GLabel 5250 2600 2    50   Input ~ 0
P1
$Comp
L power:GND #PWR0134
U 1 1 5F510F7C
P 8350 6000
F 0 "#PWR0134" H 8350 5750 50  0001 C CNN
F 1 "GND" H 8355 5827 50  0000 C CNN
F 2 "" H 8350 6000 50  0001 C CNN
F 3 "" H 8350 6000 50  0001 C CNN
	1    8350 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C24
U 1 1 5F5078DD
P 8350 5850
F 0 "C24" H 8465 5896 50  0000 L CNN
F 1 "C" H 8465 5805 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 8388 5700 50  0001 C CNN
F 3 "~" H 8350 5850 50  0001 C CNN
	1    8350 5850
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial J3
U 1 1 5F2F2255
P 10450 5700
F 0 "J3" H 10550 5675 50  0000 L CNN
F 1 "Conn_Coaxial" H 10550 5584 50  0000 L CNN
F 2 "Connector_Coaxial:SMA_Amphenol_132289_EdgeMount" H 10450 5700 50  0001 C CNN
F 3 " ~" H 10450 5700 50  0001 C CNN
	1    10450 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	9950 5700 10250 5700
Wire Wire Line
	8050 5700 8350 5700
Connection ~ 8650 5700
Connection ~ 8350 5700
Wire Wire Line
	8350 5700 8650 5700
Text GLabel 2300 3250 0    50   Input ~ 0
CE
Text GLabel 2300 3150 0    50   Input ~ 0
CSN
Wire Wire Line
	2300 3250 2450 3250
Wire Wire Line
	2300 3150 2450 3150
$EndSCHEMATC
