/*  NanoVersion-AWIS-GKinch_2025-03-23.ino
 *  -----------------------------------------------------------------------------------------------------------
 *  2025-03-23 - Rechecked the alarm set code after Gavin saw it run again at 2:35.
 *               (1) Deleted the set next 15 minute alarm function since we're now not using it.
 *               (2) Corrected the line that sets the alarm:
 *                   - WAS:  rtc.setAlarm2(alarmTime, DS3231_A2_Minute);
 *                   - SHOULD BE:  rtc.setAlarm2(alarmTime, DS3231_A2_Hour);
 *                   
 *  2025-03-20 - Change the pump to run @ 13:00 every day.
 *               (1) Added a function to do so.
 *               
 *  2025-03-07 - Reverted back to the Nano version as we had problems with the Nano33IOT (bricked?).
 *               (1) Versioned from Nano33IOT-AWIS-GKinch_2025-02-03.ino
 *               (2) Updated the Low Power library for the AVR board.
 *               (3) Slightly updated the setQuarterHourAlarm for perceived issue with new day rollover.
 *               (4) Added inclusion of stdint.h and changed variable type of the pump duration.
 *               
 *  Compilation messages:  
 *  Using library Low-Power at version 1.81 in folder: /Users/normoforan/SynologyDrive/Arduino/libraries/Low-Power
 *  Using library RTClib at version 2.1.4 in folder: /Users/normoforan/SynologyDrive/Arduino/libraries/RTClib 
 *  Using library Adafruit_BusIO at version 1.16.1 in folder: /Users/normoforan/SynologyDrive/Arduino/libraries/Adafruit_BusIO 
 *  Using library Wire at version 1.0 in folder: /Users/normoforan/Library/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/Wire 
 *  Using library SPI at version 1.0 in folder: /Users/normoforan/Library/Arduino15/packages/arduino/hardware/avr/1.8.6/libraries/...
 *  Sketch uses 11578 bytes (37%) of program storage space. Global variables use 1466 bytes (71%) of dynamic memory, 
 *  leaving 582 bytes for local variables. Maximum is 2048 bytes.
 *               
 *  2025-02-04 - Several code revisions later -- it works!
 *             - Wiring requirements:
 *                     Nano33IOT/Nano       ChronoDot RTC
 *                    ------------------------------------
 *                    Digital pin D2    ⟶     SQW
 *                    Ground - GND      ⟶     GND
 *                    3.3V              ⟶     VCC
 *                    SDA (Analog 4)    ⟶     SDA
 *                    SCL (Analog 5)    ⟶     SCL
 *  
 *  2025-02-03 - Renamed / versioned from Test_NanoXX-Pump-PumpMOSFET_2024-10-22-00.ino.
 *               (1) Revert low power library to SAMD support.
 *               (2) Add ChronoDot RTC support.
 *  
 *  2024-10-22 - 00 - Added Low Power library and code for AVR boards for testing on classic Nano 5V.
 *  
 *  2024-09-30 - 01 - Added Low Power library and code for SAMD boards:  Nano33IOT (3v3).
 *  
 *  2024-09-30 - 00 - Refined timing and print statements. Now using a logic level MOSFET:  IRLB8721 PbF.
 *               Nano 33 IOT:  Sketch uses 12852 bytes (4%) of program storage space. Global variables 
 *               use 3608 bytes (11%) of dynamic memory.
 *  
 *  2024-09-29 - Versioned from the SD card test with NPN transistor - PN2222A - to use for pump testing.
 *               See Journal Volume 8 page 131 for details.
 *  */
 
// LIBRARIES
// ------------------------------------------------------------------------------------------
#include <LowPower.h>             // Arduino Low Power library for AVR boards
#include <RTClib.h>               // Real Time Clock library
#include <Wire.h>                 // I2C support
#include <stdint.h>               // Support standard C variable types

//==================================================================================================================
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||               DEFINES            |||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//==================================================================================================================
#define usbSerial Serial        // for Notecard
#define PUMP_PWR_PIN 3          // Digital Pin 3 connects to the NPN transistor Base.
#define RTC_INTERRUPT_PIN 2     // Digital Pin 2: Interrupt pin wakes up the MCU - connects to the RTC SQW pin.
#define RTC_OFFSET 6            // Compile time to the actual time offset in seconds while setting the RTC.

//==================================================================================================================
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||      GLOBAL VARIABLES - CLASS OBJECTS       ||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//==================================================================================================================
char devName[] = "GKinch AWIS";
const uint32_t pumpingDuration = 200000UL;  //<--- ADJUST THIS VALUE to change pump operation time [in milliseconds]
int sampleCount = 1;                        // Sample counter

//_______________________________ RTC Support _________________________________________________________________________
RTC_DS3231 rtc;                   // RTC INSTANCE

DateTime now;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

volatile bool alarmTriggered = false;

//====================================================================================================================                               
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                                         |||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                FUNCTIONS                |||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                                         |||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//====================================================================================================================

//===========================================================================================================================
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||    FUNCTION SECTION 07 -- HOUSEKEEPING functions    |||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//===========================================================================================================================

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 07-03: PRINT sample grab INTRODUCTION to the serial monitor
// -------------------------------------------------------------------------------------------------------
void printSampleGrabINTRO() {
  
  usbSerial.println(F("============================================================================="));
    usbSerial.print(F("Starting Sample Grab # ")); usbSerial.println(sampleCount);
  usbSerial.println(F("............................................................................."));
  usbSerial.println("");
} // END OF THE FUNCTION

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 07-04-01: PRINT Setup banner        rev-01 12-26-2024 uses Function 07-05 to get filename
// -------------------------------------------------------------------------------------------------------
void printSetupBanner() {
  usbSerial.println(F("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"));
  usbSerial.println(F("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"));
  usbSerial.println(F("---------------------------------------------------------------"));
  usbSerial.println(F("                   *** RUNNING SETUP ***                       "));
  usbSerial.println(F("---------------------------------------------------------------"));
  printSketchFileName();
  usbSerial.print(F("Device name:  ")); usbSerial.println(devName);
  usbSerial.println(F("Baud rate = 115200"));
  usbSerial.println(F("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"));
  usbSerial.println(F("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"));
  usbSerial.println(F(""));
}

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 07-05: PRINT setup complete banner
// -------------------------------------------------------------------------------------------------------
void printSetupCompleteBanner() {
  Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
  Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
  Serial.println("||||||||||              SETUP COMPLETE               ||||||||||");
  Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
  Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
  Serial.println(F("---------------------------------------------------------------"));
}

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 07-06-00:  GET filename & extract from the full file path
//--------------------------------------------------------------------------------------------------------
void printSketchFileName() {
  
  // Get the full path from:  __FILE__  which is a preprocessor 
  // macro that expands to a C-string (a null-terminated char array).
  const char* fullPath = __FILE__;
  
  const char* fileName = fullPath;
  // Find the position of the last slash............................................
  for (const char* p = fullPath; *p != '\0'; p++) {
    if (*p == '/' || *p == '\\') {
      fileName = p + 1;             // Move pointer to the character after the slash
    }
  }
  Serial.println("The currently running sketch is:");
  Serial.println(fileName);                            // Print only the filename
  
} // END OF FUNCTION

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 0x-0x-00:  Print the date & time from the ChronoDot RTC
//--------------------------------------------------------------------------------------------------------
void printTimefromRTC(void) {        // presumes that DateTime now = rtc.now() has been called
                                     // to get the most current time.
  Serial.print("Current Date:  ");
  Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
  Serial.print(", ");
  Serial.print(now.year(), DEC);
  Serial.print('-');
  Serial.print(now.month(), DEC);
  Serial.print('-');
  Serial.println(now.day(), DEC);

  Serial.print("Current ChronoDot 3.0 time:  ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
    
}   // END OF FUNCTION

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 0x-0x-00:  Print temperature from the ChronoDot RTC
//--------------------------------------------------------------------------------------------------------
void printTempfromRTC(void) {

  Serial.print("Current ChronoDot temperature: ");
  Serial.print(rtc.getTemperature());
  Serial.println("°C");
    
}   // END OF FUNCTION

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 0x-0x-00:  Interrupt Service Routine (ISR)
//--------------------------------------------------------------------------------------------------------
void alarmISR() {
    
    alarmTriggered = true;
    
}   // END OF FUNCTION

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 0x-0x-01:  Quarter Hour Alarm - rev-01 @ 3/7/2025
//--------------------------------------------------------------------------------------------------------
void setQuarterHourAlarm() {
    
    now = rtc.now();
    rtc.clearAlarm(1);

    uint8_t nextMinute;
    uint8_t nextHour = now.hour();
    uint8_t nextDay = now.day();
    uint8_t nextMonth = now.month();
    uint16_t nextYear = now.year();
    
    // Determine next quarter-hour mark
    if (now.minute() < 15) nextMinute = 15;
    else if (now.minute() < 30) nextMinute = 30;
    else if (now.minute() < 45) nextMinute = 45;
    else { 
        nextMinute = 0;  // Roll over to next hour
        nextHour++;
        
        // Handle day/month/year rollover at midnight
        if (nextHour == 24) {
            nextHour = 0;  // Reset to midnight
            nextDay++;

            // Check for month rollover
            if (nextDay > DateTime(nextYear, nextMonth, 1).dayOfTheWeek()) {
                nextDay = 1;
                nextMonth++;

                // Check for year rollover (December to January)
                if (nextMonth > 12) {
                    nextMonth = 1;
                    nextYear++;
                }
            }
        }
    }

    // Set the alarm at the next quarter-hour
    rtc.setAlarm1(DateTime(nextYear, nextMonth, nextDay, nextHour, nextMinute, 0),
                  DS3231_A1_Minute);
    Serial.println("Next quarter-hour alarm now set.");
    Serial.println(F("---------------------------------------------------------------"));
}   // END OF FUNCTION

//--------------------------------------------------------------------------------------------------------
//:::::FUNCTION 0x-0x-00:  Alarm set to match @ given time every day - rev-00 @ 3/20/2025
//--------------------------------------------------------------------------------------------------------
void setDailyAlarm(uint8_t hour, uint8_t minute) {
    rtc.disableAlarm(1);  // Disable Alarm 1 if it was set
    rtc.disableAlarm(2);  // Disable any previous Alarm 2

    // Create a DateTime object for alarm (date values are ignored for Alarm 2)
    DateTime alarmTime(0, 0, 0, hour, minute, 0);

    // Set Alarm 2 to trigger when hours and minutes match
    rtc.setAlarm2(alarmTime, DS3231_A2_Hour);

    // Clear any existing alarm flag
    rtc.clearAlarm(2);

    // Ensure SQW pin is used for alarms, not frequency output
    rtc.writeSqwPinMode(DS3231_OFF);

    Serial.print("Alarm set for ");
    Serial.print(hour);
    Serial.print(":");
    Serial.println(minute);
}

//===========================================================================================================================
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                                         ||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                  SETUP                  ||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                                         ||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//===========================================================================================================================
void setup() {
  
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial && millis() < 10000);  // If USB is not connected, wait for it or
                                        // wait 10 seconds before proceeding.
  printSetupBanner();
  Serial.print("Configuring pump power pin...");
  digitalWrite(PUMP_PWR_PIN, LOW);      // Ensure the pump doesn't turn during pin config.
  pinMode(PUMP_PWR_PIN, OUTPUT);        // Configure pin as an output.
  Serial.println("\nDigital Pin 3 has been configured for power.");
  Serial.println(F("---------------------------------------------------------------"));
  Serial.println("Configuring Digital Pin 2 to receive RTC interrupt signal.");
  pinMode(RTC_INTERRUPT_PIN, INPUT_PULLUP);  // SQW is open-drain, so use pull-up
  
  // Attach an interrupt on falling edge (DS3231 pulls SQW low on alarm)
  attachInterrupt(digitalPinToInterrupt(RTC_INTERRUPT_PIN), alarmISR, FALLING);
    
  Serial.println("Checking for the Real Time Clock (RTC)...");
  if (! rtc.begin()) {
    Serial.println("*** RTC NOT FOUND!! ***");
    Serial.println("*** Program halting ***");
    while(1);
  }

  Serial.println("Found RTC...");

// ========================================================================================
// *** NOTE ON SETTING RTC TIME FROM ATTACHED COMPUTER TIME: Uncomment the following 
// two lines only if you need to reset the RTC time by reflashing this sketch. This
// sketch "remembers" & uses the date and time at which it was originally compiled. 
// As a result, these two lines of code will just reset the RTC back to that original
// compiled date if and when the program restarts -- which happens anytime the MCU 
// reboots after losing power or being manually reset. So for that reason, this
// sketch must be reflashed -- with the lines commented out -- AFTER the RTC has had
// its time set accurately the first time the program is uploaded after compilation.
// ========================================================================================
//  Serial.println("Set RTC time to datetime at which the sketch was originally compiled.");
//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  Serial.println("Getting the current time...");
  now = rtc.now() + RTC_OFFSET;
  // NOTE:  A few seconds are added to the reported time to adjust for the offset
  // between the time it takes the program to get time from the computer and then
  // use that time to set the time on the ChronoDot.
  printTimefromRTC();
  printTempfromRTC();
  Serial.println(F("---------------------------------------------------------------"));
  
  Serial.println("Configuring the Alarm 1 on the ChronoDot RTC...");
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);
  rtc.disable32K();                 // Disable the 32KHz output (optional)
  rtc.writeSqwPinMode(DS3231_OFF);  // Ensure SQW is not in square wave mode

  // Configure Alarm 2 to trigger at 13:00 daily
  setDailyAlarm(15, 45);
  
  Serial.println("ChronoDot alarm configuration complete.");
  delay(5000);
  printSetupCompleteBanner();
}

//===========================================================================================================================
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                                         ||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                   LOOP                  ||||||||||||||||||||||||||||||||||||||||||||||||||||||
//||||||||||||||||||||||||||||                                         ||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
//===========================================================================================================================
void loop(void) {

  if (alarmTriggered) {

    noInterrupts();             // Disable interrupts while modifying shared variables
    alarmTriggered = false;     // Reset the flag
    interrupts();               // Re-enable interrupts         
    
    now = rtc.now() + RTC_OFFSET;
    printTimefromRTC();
    printTempfromRTC();

    Serial.print("Starting Pump Cycle Number ");  Serial.println(sampleCount);
    digitalWrite(PUMP_PWR_PIN, HIGH);   // Turn the pump ON
    Serial.print("Pump cycling on for "); Serial.print(pumpingDuration/1000);
    Serial.println(" seconds.");
    delay(pumpingDuration);                              
    digitalWrite(PUMP_PWR_PIN, LOW);    // Turn the pump OFF
    Serial.print("Pump is now OFF. Cumulative run time:  ");
    
    if( ((pumpingDuration * sampleCount) / 1000 ) < 60 ) {
        Serial.print( (pumpingDuration * sampleCount) / 1000 );
        Serial.println(" seconds.");
        } else { 
        Serial.print( float(pumpingDuration * sampleCount) / 60000 );
        Serial.println(" minutes.");
        }

    sampleCount++;                      // Increment the counter by 1

    // Clear RTC alarm flag
    rtc.clearAlarm(2);

    Serial.println("MCU returning to sleep.");
    Serial.println(F("---------------------------------------------------------------"));
    delay(100);  // Allow serial output before sleeping
    }

    // Put the MCU to sleep until an RTC interrupt occurs
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
}


  
