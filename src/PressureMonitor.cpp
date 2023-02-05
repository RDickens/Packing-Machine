/******************************************************************************
 * @file PressureMonitor.cpp
 * @author Roy Dickens (rccdickens@yahoo.co.uk)
 * @date 20-09-2022
 *
 * @copyright Copyright (c) 2022 CC BY
 *
 * @brief Packing Machine Gas Indicator
 * ----------------------------------------------------------------------------
 * This program checks that there as is a set pressure to the nitrogen flush
 * solenoid when gas is requested. If no gas is present then an audible / visual
 * warning is triggered for a set time then silences itself. This will also have a button
 * to immediately silence it. If gas pressure returns then the alert will be cancelled
 * and the program will return to monitoring the the pressure
 *
 * ----------------------------------------------------------------------------
 *
 * For notes see README.TXT

*/

#include <Arduino.h>
#include <EEPROM.h>
#include <TaskScheduler.h>
#include <avr/wdt.h>

#include "PinChangeInterrupt.h"

/******************************************************************************
 * @brief Defines
 *
 ******************************************************************************/
#define DUAL_ALERT_TIMEOUT 160   // How many flash / sounder toggles before auto silence
#define SPEED 750                // Sets lamp and buzzer pulse speed in miliseconds
#define LOW_COUNT 5              // How many measurements need to be below threshold before triggering alert (removes noise)
#define WATCHDOG_TIMEOUT WDTO_4S // How long till watchdog causes a cpu reset

#define SENSOR_PIN A0             // Nitrogen pressure sensor
#define BUTTON_PIN PIN_A1         // Button
#define PULSE_PIN PIN_A2          // Pulse pin from gas solenoid
#define CALIBRATE_SW PIN_A4       // Calibrate switch
#define TOLERANCE_A_SW PIN_A6     // Tolerance byte A switch
#define TOLERANCE_B_SW PIN_A5     // Tolerance byte B switch
#define SOUNDER_SILENCE_SW PIN_A3 // Sounder silence switch
#define LAMP_PIN PIN_A7           // Lamp pin
#define SOUNDER_PIN PIN_B2        // Sounder pin
#define INDICATOR1_PIN PIN_B0     // Indicator pin
#define INDICATOR2_PIN PIN_B1     // Indicator pin
#define SOUNDER_LEVEL HIGH        // What level triggers sounder. HIGH or LOW
#define LAMP_LEVEL HIGH           // What level triggers lamp. HIGH or LOW
#define PULSE_LEVEL LOW           // What level triggers pulse. HIGH or LOW
#define INDICATOR1_LEVEL HIGH     // What level triggers indicator 1. HIGH or LOW
#define INDICATOR2_LEVEL HIGH     // What level triggers indicator 2. HIGH or LOW
#define BUTTON_LEVEL LOW          // What level button triggers. HIGH or LOW
#define CALIBRATE_LEVEL LOW       // What level calibrate triggers. HIGH or LOW
#define SOUNDER_SILENCE_LEVEL LOW // What level sounder silence switch triggers. HIGH or LOW

/******************************************************************************
 * @brief optional defines
 *
 ******************************************************************************/
// #define PULSE_SOUNDER           // Enable if sounder gives a constant tone
// #define TIMEOUT // Do you want the indicator lights to timeout

/******************************************************************************
 * @brief optional routines
 *
 ******************************************************************************/
#define WATCHDOG // Do you want to enable the cpu watchdog

/******************************************************************************
 * @brief List all procedures
 *
 ******************************************************************************/
void measure();
void dualalertpulse();
void alertpulse();
void interruptroutine();
void readtolerance();
void readeeprom();
void writeeeprom();
void calibrate();
void clearindicators();
void setup();
void loop();

/******************************************************************************
 * @brief List optional procedures
 *
 ******************************************************************************/
#ifdef WATCHDOG
void resetreason();
#endif

/******************************************************************************
 * @brief List of all variables
 *
 ******************************************************************************/
int SensorValue;
int CalibrationValue;
bool val;
int ToleranceArray[] = {0, 20, 40, 60}; // Tolerance steps
int Tolerance;
bool LampLevel;
volatile bool pulseflag = false;
volatile bool continuousflag = false;
long debouncing_time = 15; // Debouncing Time in Milliseconds
volatile unsigned long last_micros;
int delaycount = 0;
int delaymax = LOW_COUNT; // How many measurements need to be below threshold before triggering alert (removes noise)
int flashspeed = SPEED;   // Sets lamp and buzzer speed
bool silenced = false;    // Has the alarm been silenced
volatile int reset = 0;   // Reason for last reset 0 unset, 1 Power on, 2 Brown out, 3 Reset button, 4 Watchdog

/******************************************************************************
 * @brief Tasks & scheduler
 *
 ******************************************************************************/
// Task Scheduler
Scheduler runner;

// Tasks
Task measurealarm(10 * TASK_SECOND, TASK_FOREVER, measure, &runner);                            // Alarm to take measurement if pulse pin is held low. i.e. FLOOD
Task triggeredalarm(flashspeed *TASK_MILLISECOND, DUAL_ALERT_TIMEOUT, dualalertpulse, &runner); // Alarm to trigger both sounder and lamp
Task silentalarm(flashspeed *TASK_MILLISECOND, TASK_FOREVER, alertpulse, &runner);              // Alarm to only flash lamp
Task clearindicatoralarm(5 * TASK_SECOND, TASK_ONCE, clearindicators, &runner);                 // Alarm to clear indicators on idle

/******************************************************************************
 * @brief Pressure measurement routine
 *
 ******************************************************************************/
void measure()
{

    // Read tolerance DIP switches
    readtolerance();

    // Read the analog value on sensor pin
    SensorValue = analogRead(SENSOR_PIN);

    // Clear indicator lights
    clearindicators();

    // Check to see if pressure is above required threshold
    if (SensorValue >= (CalibrationValue - ToleranceArray[Tolerance]) == true)
    {
        digitalWrite(INDICATOR1_PIN, !INDICATOR1_LEVEL); // Indicate pressure is ok
        digitalWrite(INDICATOR2_PIN, INDICATOR2_LEVEL);  // Indicate pressure is ok

        digitalWrite(LAMP_PIN, !LAMP_LEVEL);       // Clear any lamp alert
        digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL); // Clear any sounder alert

        delaycount = 0;

        triggeredalarm.abort(); // Cancel lamp / sounder alarm
        silentalarm.abort();    // Cancel silent alarm

        silenced = false;
    }
    else
    {
        digitalWrite(INDICATOR1_PIN, INDICATOR1_LEVEL);  // Indicate pressure is too low
        digitalWrite(INDICATOR2_PIN, !INDICATOR2_LEVEL); // Indicate pressure is too low

        delaycount++;

        if (delaycount >= delaymax) // Check to see if there has been enough low pressure readings to trigger alarm
        {
            if (silenced == false) // Check to see if alarm has been silenced
            {
                if (triggeredalarm.isEnabled() == false) // Is alarm already running
                    triggeredalarm.restart();            // Start alarm
            }

            delaycount = delaymax; // Prevent roll over
        }
    }

    pulseflag = false; // Clear individual pulse flag

    clearindicatoralarm.restartDelayed(); // Restart timer to clear indicators on idle
}

/******************************************************************************
 * @brief Alert strobing lamp and sounder
 *
 ******************************************************************************/
void dualalertpulse()
{
    silentalarm.abort(); // Cancel silent alarm

    LampLevel = !LampLevel; // Invert level

    if (LampLevel == true) // Turn on / off lamp / sounder depending on LampLevel
    {
        digitalWrite(LAMP_PIN, LAMP_LEVEL);
        digitalWrite(SOUNDER_PIN, SOUNDER_LEVEL);
    }
    else
    {
        digitalWrite(LAMP_PIN, !LAMP_LEVEL);

#ifdef PULSE_SOUNDER
        digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);
#endif
    }

    if ((digitalRead(SOUNDER_SILENCE_SW) == SOUNDER_SILENCE_LEVEL) && (silenced == false))
    {
        triggeredalarm.restartDelayed(flashspeed * TASK_MILLISECOND);
    }
    else
    {

        if ((triggeredalarm.isLastIteration() == true) || (silenced == true)) // Is it the last noisy alert or has it been silenced
        {
            digitalWrite(LAMP_PIN, !LAMP_LEVEL);
            digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);

            silentalarm.restart(); // Start a silent alarm
            triggeredalarm.abort();
            silenced = true;
        }
    }
}

/******************************************************************************
 * @brief Silent alert
 *
 ******************************************************************************/
void alertpulse()
{
    LampLevel = !LampLevel; // Invert level

    if (LampLevel == true) // Turn on / off lamp depending on LampLevel
    {
        digitalWrite(LAMP_PIN, LAMP_LEVEL);
    }
    else
    {
        digitalWrite(LAMP_PIN, !LAMP_LEVEL);
    }

    silenced = true;
}

/******************************************************************************
 * @brief Interrupt routine
 *
 * Runs on every change of the pulse pin. Also sets up  continuous readings if
 * pin is held at triggered level
 ******************************************************************************/
void interruptroutine()
{

    // Debounce
    if ((long)(micros() - last_micros) >= debouncing_time * 1000)
    {

        if (digitalRead(PULSE_PIN) == PULSE_LEVEL) // Check pulse pin level is rising or falling
        {
            pulseflag = true;      // Enable individual measurement
            continuousflag = true; // Enable continuous measurement
        }
        else
        {
            continuousflag = false; // Disable continuous reading
        }

        last_micros = micros(); // Debounce stuff
    }
}

/******************************************************************************
 * @brief Read tolerance DIP switches
 *
 ******************************************************************************/
void readtolerance()
{
    int temp;

    Tolerance = digitalRead(TOLERANCE_B_SW);
    Tolerance = Tolerance << 1;
    temp = digitalRead(TOLERANCE_A_SW);
    Tolerance = Tolerance + temp;
}

/******************************************************************************
 * @brief Read saved calibration data from EEPROM
 *
 ******************************************************************************/
void readeeprom()
{
    EEPROM.get(0, CalibrationValue);
}

/******************************************************************************
 * @brief Write calibration data to EEPROM
 *
 ******************************************************************************/
void writeeeprom()
{
    CalibrationValue = analogRead(SENSOR_PIN);

    EEPROM.put(0, CalibrationValue);
}

/******************************************************************************
 * @brief Calibrate pressure sensor to desired reading
 *
 ******************************************************************************/
void calibrate()
{
    // Pulse sounder and lamp to let operator know sensor is working
    digitalWrite(SOUNDER_PIN, SOUNDER_LEVEL);
    digitalWrite(LAMP_PIN, LAMP_LEVEL);
    digitalWrite(INDICATOR1_PIN, INDICATOR1_LEVEL);
    digitalWrite(INDICATOR2_PIN, INDICATOR2_LEVEL);
    delay(flashspeed);
    digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);
    digitalWrite(LAMP_PIN, !LAMP_LEVEL);
    digitalWrite(INDICATOR1_PIN, !INDICATOR1_LEVEL);
    digitalWrite(INDICATOR2_PIN, !INDICATOR2_LEVEL);
    delay(flashspeed);
    digitalWrite(SOUNDER_PIN, SOUNDER_LEVEL);
    digitalWrite(LAMP_PIN, LAMP_LEVEL);
    digitalWrite(INDICATOR1_PIN, INDICATOR1_LEVEL);
    digitalWrite(INDICATOR2_PIN, INDICATOR2_LEVEL);
    delay(flashspeed);
    digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);
    digitalWrite(LAMP_PIN, !LAMP_LEVEL);
    digitalWrite(INDICATOR1_PIN, !INDICATOR1_LEVEL);
    digitalWrite(INDICATOR2_PIN, !INDICATOR2_LEVEL);

    writeeeprom(); // Save reading to EEPROM

    do // Wait till calibration button is released
    {
        delay(100);
        val = digitalRead(BUTTON_PIN);

#ifdef WATCHDOG
        wdt_reset(); // Ensure watchdog doesn't trigger during calibration
#endif

    } while (val == BUTTON_LEVEL);
}

/******************************************************************************
 * @brief Clear indicators on timeout
 *
 ******************************************************************************/
void clearindicators()
{
#ifdef TIMEOUT
    digitalWrite(INDICATOR1_PIN, !INDICATOR1_LEVEL);
    digitalWrite(INDICATOR2_PIN, !INDICATOR2_LEVEL);
#endif
}

/******************************************************************************
 * @brief Flash lamp for the reason for last reset
 *
 * 1 Power on, 2 Brown out, 3 Reset button, 4 Watchdog
 ******************************************************************************/
#ifdef WATCHDOG
void resetreason()
{

    if (MCUSR & (_BV(PORF)))
    {
        // Power On
        reset = 1;
    }
    else if (MCUSR & (_BV(BORF)))
    {
        // Brownout
        reset = 2;
    }
    else if (MCUSR & _BV(EXTRF))
    {
        // Reset button or otherwise some software reset
        reset = 3;
    }

    else if (MCUSR & _BV(WDRF))
    {
        // Watchdog Reset
        reset = 4;
    }

    MCUSR = 0x00;
    wdt_disable();
}
#endif

/******************************************************************************
 * @brief Main setup
 *
 ******************************************************************************/
void setup()
{
    // Setup pins
    pinMode(SENSOR_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(PULSE_PIN, INPUT_PULLUP);
    pinMode(CALIBRATE_SW, INPUT_PULLUP);
    pinMode(TOLERANCE_A_SW, INPUT_PULLUP);
    pinMode(TOLERANCE_B_SW, INPUT_PULLUP);
    pinMode(SOUNDER_SILENCE_SW, INPUT_PULLUP);
    pinMode(LAMP_PIN, OUTPUT);
    pinMode(SOUNDER_PIN, OUTPUT);

    pinMode(INDICATOR1_PIN, OUTPUT);
    pinMode(INDICATOR2_PIN, OUTPUT);

// if watchdog is enabled display reason for last reset otherwise just say its a normal reset
#ifdef WATCHDOG

    resetreason();

    // Blink reset reason. 1 Power on, 2 Brown out, 3 Reset button, 4 Watchdog
    for (int loopcount = 0; loopcount < reset; loopcount++)
    {
        digitalWrite(LAMP_PIN, LAMP_LEVEL);
        digitalWrite(INDICATOR1_PIN, INDICATOR1_LEVEL);
        digitalWrite(INDICATOR2_PIN, INDICATOR2_LEVEL);
        delay(500);
        digitalWrite(LAMP_PIN, !LAMP_LEVEL);
        digitalWrite(INDICATOR1_PIN, !INDICATOR1_LEVEL);
        digitalWrite(INDICATOR2_PIN, !INDICATOR2_LEVEL);
        delay(500);
    }

    wdt_enable(WATCHDOG_TIMEOUT);
    wdt_reset();
    // delay(2500);
#else

    // Ensure watchdog is turned off
    MCUSR = 0x00;
    wdt_disable();

    reset = 1;
#endif

    attachPCINT(digitalPinToPCINT(PULSE_PIN), interruptroutine, CHANGE); // Interrupt setup

    if (reset == 1)
    {
        // Pulse sounder and lamp to let operator know sensor is working
        digitalWrite(SOUNDER_PIN, SOUNDER_LEVEL);
        digitalWrite(LAMP_PIN, LAMP_LEVEL);
        delay(flashspeed);
        digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);
        digitalWrite(LAMP_PIN, !LAMP_LEVEL);
        delay(flashspeed);
        digitalWrite(SOUNDER_PIN, SOUNDER_LEVEL);
        digitalWrite(LAMP_PIN, LAMP_LEVEL);
        delay(flashspeed);
        digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);
        digitalWrite(LAMP_PIN, !LAMP_LEVEL);
    }

    reset = 0;

    readeeprom(); // Read saved pressure value from EEPROM
}

/******************************************************************************
 * @brief Main loop
 *
 ******************************************************************************/
void loop()
{

#ifdef WATCHDOG
    wdt_reset(); // Reset watchdog timer
#endif

    if (pulseflag == true)
    {
        if (measurealarm.isEnabled() == false)
            measurealarm.enable();
    }
    else
    {
        if (continuousflag == true)
            measurealarm.enableIfNot();
        else
            measurealarm.abort();
    }

    val = digitalRead(BUTTON_PIN);
    if (val == BUTTON_LEVEL)
    {
        val = digitalRead(CALIBRATE_SW);
        if (val == CALIBRATE_LEVEL)
        {
            calibrate();
        }
        else
        {
            digitalWrite(LAMP_PIN, LAMP_LEVEL); // Briefly turn on lamp to show user cpu is working.
            delay(250);

            digitalWrite(LAMP_PIN, !LAMP_LEVEL);
            digitalWrite(SOUNDER_PIN, !SOUNDER_LEVEL);

            if ((triggeredalarm.isEnabled() == true) || (silentalarm.isEnabled() == true))
            {

                triggeredalarm.abort();
                silentalarm.enable();
                silenced = true;
            }
        }
    }

    runner.execute(); // Do task scheduler stuff
}