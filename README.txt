Pressure Monitor

Roy Dickens (rccdickens@yahoo.co.uk)

Original date 20-09-2022

Copyright (c) 2022 CC BY

Packing Machine Gas Indicator
 ----------------------------------------------------------------------------
 This program checks that there as is a set pressure to the nitrogen flush
 solenoid when gas is requested. If no gas is present then an audible / visual
 warning is triggered for a set time then silences itself. This will also have a button
 to immediately silence it. If gas pressure returns then the alert will be cancelled
 and the program will return to monitoring the the pressure
 
 ----------------------------------------------------------------------------
 Tolerance from 1 bar ~1.75v
 
 DIP (1 2)    Tolerance   Steps
 00           0%          0
 01           10%         20
 10           20%         40
 11           30%         60
 
 Offset 102 Steps. Sensor range output 0.5v - 4.5v for 0 - 30 psi
 ----------------------------------------------------------------------------
 
 To calibrate required gas pressure
 ----------------------------------
 1. Set pressure regulator on nitrogen bottle
 2. Set DIP3 to calibrate
 3. Press silence / calibrate button
 4. Lamp and sounder will pulse
 5. Reset DIP3 to normal
 6. Set tolerance DIP switches 1 & 2 as req.
 ******************************************************************************


PINS USED
A0  Sensor          Analog      Attached to pressure sensor
A1  Button          Digital     Used to calibrate / silence depending on DIP3
A2  Pulse input     Digital     Attached to gas pulse wire
A3  DIP4 Silence    Digital     Disables sounder silence timeout
A4  DIP3 Calibrate  Digital     Used to enable calibration button
A5  DIP2 ToleranceB Digital     Tolerance set LSB
A6  DIP1 ToleranceA Digital     Tolerance set MSB
A7  Lamp            Digital     Front panel lamp
B0  Indicator 1     Digital     Onboard indicator (last pulse was low on gas)
B1  Indicator 2     Digital     Onboard indicator (last pulse was had enough gas)
B2  Sounder         Digital     Front panel sounder
B3  'Reset          Digital     Reset IC


Release Notes

Date 4/10/2022
---------------
- Initial release version


Date 28/10/2022
---------------
- Connected spare IC pin A3 to DIP4 this will allow the sounder auto silence to be enabled or disabled
- Adjusted pulse speed so lamp flashes at the same rate as sounder pulses
- Adjusted resistor values to fit availability
- Added momentary GAS button to simulate gas pulse


Date 6/11/2022
---------------
- Fixed problem where button press starts silent alarm timer


Date 24/1/2023
--------------
- Added watchdog functionality
- Added button press to blink indicator once to show user cpu is still working