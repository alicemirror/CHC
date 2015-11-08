
/**
	\file CircuitHealthStatus_ControllerBoard.ino
	\brief Firmware for the Circuit Health Controller board. Main sketch file
  
	\mainpage
  This firmware is for the board circuit revision 4 + LCD Alpha shift register display board
  
  This is the firmware sketch provided by default with the board. Users can modify parameters,
  strings and I/O behavior depending on their needs and the installation options.
  
  This board runs as an external device able to control - and eventually reset of power on/off your
  main circuit when some desired or critical / alarming considtions are detected through the sensors.
  
  \note The most relevant constants and parameters, subject to modification by the users to optimize and customize the Controller Board behaviour are mentioned in the documentation of the components.
  
  \note The mentioned server example settings can be changed to adapt the Controller behaviour to any kind of device
  or circuit.
  
  For further details on the board circuit and behaviour see the following article on <a href=http://www.element14.com/community/groups/arduino/blog/2015/11/07/circuit-health-control-arduino-compatible target=_blank><b>Element14 Arduino blog</b></a>
  
  The board is available on <a href=http://drobott.com/item/170/Programmable-Circuit-Health-Controller-Arduino-Compatible target=_nlank><b>Drobott.com</b></a> 
  
  For the last updated version clone the repository on <a href=https://github.com/alicemirror/CHC target=_blank><b>GitHub</b></a>
    
  \author Enrico Miglino <baleariddynamics@gmail.com>
  \date November 2015
  \version 1.1 (see the Version.h include file for the build and version details), hardware version 1.0 revision 4
  
  \copyright This software is open source released under the apache license
*/
#include <AlphaLCD.h>
#include <Streaming.h>
#include "UpTime.h"  

// Include the LCD helper and the application string
#include "LCD.h"
#include "Strings.h"
#include "Version.h"

//! Minimum PWM frequency to start fan
//! Set this value to a value not less than 60 to avoid the motor not starting.
//! \note This parameter value is preset based on a 60 cm diameter fan 12V powered. Different power motors can require
//! an higher frequency to start
#define FANSPEED_MIN 60

//! Maximum PWM frequency to reach
//! This is the max PWM frequency value. it is not needed to change it
#define FANSPEED_MAX 255

//! Minimum temperature to start fan
//! The temperature calculation is based on a case of about 15x15x15 cm internal size.
//! \note Take in account that the internal case temperature is expected to be lower than the controlled
//! board temperature. Setting this limit to a too high value may seriously damage the circuits.
#define TEMP_MIN 30

//! Maximum temperature before overheating error
//! If the internal temperature reach this level the board enter in a overheating risk. Leaving the system
//! working for long time at high temperatures may produce serious damage to the components.
#define TEMP_MAX 60

//! Loop update frequency
//! This value is a timing delay at the end of every cycle in the loop() function
#define UPDATE_FREQ 10

//! Fan full speed test during initialisation
//! This value has no influence on the system control. Just to check if the fan is fully operational (at maximum speed)
//! when the board is powered on.
#define FAN_TEST_MS 2500

//! PWM Pin conntrolling the fan speed
//! \warning This value is hardwired and should not be changed!
#define PWM_FAN 3
//! Analog pin controlling the temperature
//! \warning This value is hardwired and should not be changed!
#define ANALOG_TEMP 0
//! Reset button pin
//! \warning This value is hardwired and should not be changed!
#define RESET_BUTTON 2
//! Power on/off button pin
//! \warning This value is hardwired and should not be changed!
#define POWER_BUTTON 7
//! Vibration sensor pin
//! \warning This value is hardwired and should not be changed!
#define VIBRATION_SENSOR 0
//! Power control simulated button pin
//! \warning This value is hardwired and should not be changed!
#define SERVER_POWER_BTN 8
//! Reset control simulated button pin
//! \warning This value is hardwired and should not be changed!
#define SERVER_RESET_BTN 9

// ============================================================
// Application flags and status constants.
// Change these values only if you are aware of what this means.
// ============================================================
#define SERVER_ON  1    ///< Server powered on
#define SERVER_OFF 2    ///< Server powered off
#define SERVER_RESET 3  ///< Server restarting after reset
#define SERVER_POWER_TIME  10000   ///< msec for server going up (power On and Reset)
#define SERVER_POWEROFF_TIME 5000  ///< msec for server goind down
#define SERVER_RESET_TIME 5000    ///< msec for server goind down
#define BUTTON_PRESS_TIMEOUT 5000  ///< msec Timeout when a button remain pressed
#define POWER_ON_DELAY 5000    ///< Power on message delay before starting server
#define ALARM_TIMEOUT 5000     ///< If shock alarm is longer, the system is shutdown
#define FIRST_SHOCK_DELAY 1000    ///< ms before checking the alaram persistance
#define SENSOR_READINGS 500       ///< Number of vibration sensors reading for persistance check
#define PRESS_POWER_FIRST 1    ///< Power button has been pressed
#define PRESS_POWER_SECOND 2   ///< Power button pressed again to confirm (Only for shutdown)
#define PRESS_RESET_FIRST 3    ////< Reset button pressed
#define PRESS_RESET_SECOND 4   ///< Reset button pressed again to confirm
#define BUTTON_PRESS_NONE 0    ///< No buttons has been pressed
#define MIN_FANSPEED_PERC 10   ///< Minimum fan speed PWM percentage to start the fan motor
#define RESET_CYCLE_DURATION 500      ///< ms the simulated server reset button should remain pressed
#define POWERON_CYCLE_DURATION 500    ///< ms the simulated server power button should remain pressed to power on 
#define POWEROFF_CYCLE_DURATION 5000  ///< ms the simulated server power button should remain pressed to power off 

int buttonPressed; ///< the current button pressed
int serverStatus;  ///< the current status of the server
int temp;          ///< the current temperature value
int fanSpeed;      ///< the current fan speed
int fanSpeedPerc;  ///< the fan speed in percentage (for visualisation)
int prevFanSpeedPerc;  ///< Last fan speed percentage
int prevTemp;          ///< Last temperature read
unsigned long shockAlarmTimeout; ///< The alarm timeout counter
unsigned long startTimeSec;    ///< Time value for button press validity telay

//! LCD alphanumeric display class instance
//! Data, latch and clock pins depends on the LCD board connection.
//! \warning Don't change these settings!
AlphaLCD lcd(LCDclockPin, LCDlatchPin, LCDdataPin);

//! \brief Setup method on power-on
void setup()  {
  pinMode(PWM_FAN, OUTPUT);
  pinMode(ANALOG_TEMP, INPUT);
  pinMode(RESET_BUTTON, INPUT);
  pinMode(POWER_BUTTON, INPUT);
  pinMode(VIBRATION_SENSOR, INPUT);
  pinMode(SERVER_POWER_BTN, OUTPUT);
  pinMode(SERVER_RESET_BTN, OUTPUT);

  // Initial server status
  serverStatus = SERVER_OFF;
  // Initial buttons status
  buttonPressed = BUTTON_PRESS_NONE;

  // Initializes the LCD library
  lcd.begin(LCDCHARS, LCDROWS);
  // Turn LCD On
  lcd.display();
  // Initial message
  welcome();
  initFanTemp();
  testFan();
  
  now((time_t)millis()); // For UpTime initialisation
  initUptime();
  
}

//! \brief Main loop application
void loop(void) {
  // ==============================  
  // Check the schock risk status
  // ==============================
  // Shock alarm is checked only when the server is running
  if( (digitalRead(VIBRATION_SENSOR) == HIGH) && (serverStatus != SERVER_OFF) ){
    // Shock alarm - Initialise the count
    showShock();
    // wait a few second(s) to reduce the sensor sensitivity before checking
    // for risk condition persistance. This value is calibrated experimentally
    delay(FIRST_SHOCK_DELAY); 
    shockAlarmTimeout = millis();
    int numberShock; // counter of the detected vibrations
    // Exit from the alarm loop only when the alarm ends or the system shutdown process is started.
    boolean alarmSet = true;
    numberShock = 0;
    while(alarmSet) {
      // Read 100 times the sensor.
      for(int j = 0; j < SENSOR_READINGS; j++)  {
        if(digitalRead(VIBRATION_SENSOR) == HIGH)
          numberShock++;
      } // vibration counter loop
      // If alarm condition persists, update the display
      // to create a blinking effect at the end of every loop
      if(numberShock > 0)
            showShock();
      // Check for alarm timeout
      if( (millis() - shockAlarmTimeout) > ALARM_TIMEOUT)
        alarmSet = false; // just exit from the loop
    } // alarm timeout loop
    // If alarm condition persisted for too much time, the server is
    // powered off, else restore the normal conditions
    if(numberShock > 0) {
      showServerStartingStopping();
      execServerPowerOff();
      serverStatus = SERVER_OFF;
      buttonPressed = BUTTON_PRESS_NONE;
      initFanTemp();
    }
    else {
      // Restore the normal condition
      buttonPressed = BUTTON_PRESS_NONE;
      serverStatus = SERVER_ON;
      initFanTemp();
    }
  } // end vibration alarm check
  
  // ==============================  
  // Check the state of the buttons
  // ==============================
  // Manage Reset button ------------------------------------------
  if (checkPushReleaseButton(RESET_BUTTON) == LOW) {
      showAction(RESET_BUTTON);
      // Reset button
      if( (buttonPressed == PRESS_RESET_FIRST) && (serverStatus == SERVER_RESET) ){
        buttonPressed = PRESS_RESET_SECOND;
      } // Second button accepted
    else {
      if(serverStatus == SERVER_ON) {
        buttonPressed = PRESS_RESET_FIRST;
      } // Server on, can reset
      else {
        buttonPressed = BUTTON_PRESS_NONE;
        initFanTemp();
      } // Server off, reset impossible
    } // First press
  } // Reset button pressed
  
  // Manage Power on/off button -----------------------------------
  else if(checkPushReleaseButton(POWER_BUTTON) == LOW) {
    showAction(POWER_BUTTON);
    if( (buttonPressed == PRESS_POWER_FIRST) && (serverStatus == SERVER_ON) ) {
        buttonPressed = PRESS_POWER_SECOND;
    } // First button already pressed with server on
    else {
      // Power on the server
        buttonPressed = PRESS_POWER_FIRST;
    }
  } // Power Button pressed
  
  // =========================================
  // Process the current buttons status action  
  // =========================================
  switch(buttonPressed) {

   case BUTTON_PRESS_NONE:
    // No action request, check health status and go ahead
    checkHealthStatus();
    startTimeSec = millis(); // Initialise the timeout counter
   break;

   case PRESS_POWER_FIRST:
    if( (millis() - startTimeSec) > BUTTON_PRESS_TIMEOUT) {
      buttonPressed = BUTTON_PRESS_NONE;
      initFanTemp();
      } // First button timeout
    else {
      if(serverStatus == SERVER_OFF) {
        showPowerOn();
        execServerPowerOn();
        delay(POWER_ON_DELAY);
        showServerStartingStopping();
        delay(SERVER_POWER_TIME);  // Wait for server power on and start
        serverStatus = SERVER_ON;
        buttonPressed = BUTTON_PRESS_NONE;
        initFanTemp();
      }
      else {
        // Ask for confirmation to start poweroff sequence
        showPowerOff();
      }
    } // No timeout
   break;

   case PRESS_POWER_SECOND:
    showServerStartingStopping();
    execServerPowerOff();
    serverStatus = SERVER_OFF;
    buttonPressed = BUTTON_PRESS_NONE;
    initFanTemp();
   break;

   case PRESS_RESET_FIRST:
    if( (millis() - startTimeSec) > BUTTON_PRESS_TIMEOUT) {
      initFanTemp();
      serverStatus = SERVER_ON;
      buttonPressed = BUTTON_PRESS_NONE;
    } // First button timeout
    else {
      showReset();
      serverStatus = SERVER_RESET;
    }
   break;

   case PRESS_RESET_SECOND:
    // Reset sequence
    showServerStartingStopping();
    execServerReset();
    delay(SERVER_RESET_TIME);
    serverStatus = SERVER_ON;
    buttonPressed = BUTTON_PRESS_NONE;
    initFanTemp();
   break;  
   }
  
    delay(UPDATE_FREQ);
}

//! \brief Send a reset signal sequence to the server
void execServerReset() {
  digitalWrite(SERVER_RESET_BTN, HIGH);
  delay(RESET_CYCLE_DURATION);
  digitalWrite(SERVER_RESET_BTN, LOW);
}

//! \brief Send a power on signal sequence to the server
void execServerPowerOn() {
  digitalWrite(SERVER_POWER_BTN, HIGH);
  delay(POWERON_CYCLE_DURATION);
  digitalWrite(SERVER_POWER_BTN, LOW);
}

//! Send a reset signal sequence to the server
void execServerPowerOff() {
  digitalWrite(SERVER_POWER_BTN, HIGH);
  delay(POWEROFF_CYCLE_DURATION);
  digitalWrite(SERVER_POWER_BTN, LOW);
  delay(SERVER_POWEROFF_TIME);
}

/** 
  \brief Avoid the user keeping the button pressed
  
  \todo Optimize ths method with a more consistent series of samples.
  
  \param btn The digital pin corresponding to the button
*/
int checkPushReleaseButton(int btn) {
  int pressed = digitalRead(btn);
  
  return pressed;
}

/**
  \brief Check the health status of the system and update the display
*/
void checkHealthStatus() {
     temp = readTemp();     // get the temperature
     
     if(temp < TEMP_MIN) {   // if temp is lower than minimum temp
         fanSpeed = 0;      // fan is not spinning
         analogWrite(PWM_FAN, fanSpeed);  // spin the fan at the fanSpeed speed
     }
  
     if((temp >= TEMP_MIN) && (temp <= TEMP_MAX)) {  // if temperature is higher than minimum temp
         fanSpeed = map(temp, TEMP_MIN, TEMP_MAX, FANSPEED_MIN, FANSPEED_MAX); // the actual speed of fan
         analogWrite(PWM_FAN, fanSpeed);  // spin the fan at the fanSpeed speed
     } 
  
    // Calculate the fan speed percentage
    // as base 100 relation with the current temperature 
    fanSpeedPerc = map(temp, TEMP_MIN, TEMP_MAX, 10, 100);
    // If fanspeed is less than 10% the shown value is forced to 0 as the
    // applied PWM frequency is not sufficient to physically start the fan motor
    if(fanSpeedPerc < MIN_FANSPEED_PERC)
      fanSpeedPerc = 0;
    // Only when changes the display value is updated  
    if(prevFanSpeedPerc != fanSpeedPerc) {
      showFan(fanSpeedPerc);  
      prevFanSpeedPerc = fanSpeedPerc;
    }
    if(prevTemp != temp) {
      showTemp(temp);
      prevTemp = temp; 
    }

    showAction(BUTTON_PRESS_NONE);
    
    now((time_t)millis());
    updateTime();
}

/**
  \brief Read the analog value from the LM35 temperature sensor with the correction
  constant to convert in Celsius.
  
  \note Take in account that the LM35 temperature sensor is natively calibrated to provide
  Celsius measurements.
*/
int readTemp() {
  temp = analogRead(0);
  return temp * 0.48828125;
}

// ==============================================================
// -------- LCD Control functions
// ==============================================================

/**
  \brief Notification while a user action is active through a button press sequence
*/
void showAction(int btn) {
  lcd.setCursor(0, LCDBOTTOMROW);
  if(btn == POWER_BUTTON)
    lcd << _ACTION_ACTIVE_POWER;
  else if(btn == RESET_BUTTON)
    lcd << _ACTION_ACTIVE_RESET;
  else
    lcd << _NO_ACTION;

  delay(65);  // to draw the string
}

/**
 * \brief Welcome message shown at device power-on
 */
void welcome() {

  lcd.clear();
  lcd.setCursor(0, LCDTOPROW);
  lcd << project();
  lcd.setCursor(0, LCDBOTTOMROW);
  lcd << _VERSION << _SPACING << version() << _SPACING << build(); 
  delay(LCDMESSAGE_DELAY);
  lcd.clear();

  lcd.setCursor(LCDCHARS, LCDTOPROW);
  lcd.print(_WEB);
  
  // scroll 
  for (int positionCounter = 0; positionCounter < (LCDCHARS * 2); positionCounter++) {
    lcd.scrollDisplayLeft(); 
    delay(200);
  }
  lcd.clear();
}

//! \brief Update the uptime string
void updateTime() {
    int dd, hh, mm, ss;
  
    dd = day();
    hh = hour();
    mm = minute();
    ss = second();
  
    lcd.setCursor(LCD_SECTOR1 + LCD_UPTIME_OFFSET, LCDBOTTOMROW);
    lcd << ((dd<100)?"0":"") << ((dd<10)?"0":"") << dd << _UPTIME_DAYS << _SPACING 
        << ((hh<10)?"0":"") << hh  << _UPTIME_SEP << ((mm<10)?"0":"") << mm 
        << _UPTIME_SEP << ((ss<10)?"0":"") << ss;
}

//! \brief Initialise the Uptime string
void initUptime() {
  lcd.setCursor(LCD_SECTOR1, LCDBOTTOMROW);
  if(serverStatus == SERVER_ON)
    message(_UPTIME);
  else
    message(_UPTIMEOFF);
}

//! \brief Show the shock risk string
void showShock() {
  lcd.clear();
  delay(50);
  lcd.setCursor(LCD_SECTOR1 + 1, LCDTOPROW);
  message(_SHOCK1);
  lcd.setCursor(LCD_SECTOR3 + 3, LCDBOTTOMROW);
  message(_SHOCK2);
}

//! \brief Show the reset strings
void showReset() {
  lcd.clear();
  delay(50);
  lcd.setCursor(LCD_SECTOR1, LCDTOPROW);
  message(_RESET1);
  lcd.setCursor(LCD_SECTOR3, LCDBOTTOMROW);
  message(_RESET2);
}

//! \brief Show the powerOn strings
void showPowerOn() {
  lcd.clear();
  delay(50);
  lcd.setCursor(LCD_SECTOR1, LCDTOPROW);
  message(_POWERON1);
  lcd.setCursor(LCD_SECTOR3, LCDBOTTOMROW);
  message(_POWERON2);
}

//! \brief Show the powerOff strings
void showPowerOff() {
  lcd.clear();
  delay(50);
  lcd.setCursor(LCD_SECTOR1, LCDTOPROW);
  message(_POWEROFF1);
  lcd.setCursor(LCD_SECTOR3, LCDBOTTOMROW);
  message(_POWEROFF2);
}

//! \brief Show the server starting message
void showServerStartingStopping() {
  lcd.setCursor(LCD_SECTOR3, LCDBOTTOMROW);
  lcd << _EMPTY_HALF_LINE << _EMPTY_HALF_LINE;
  lcd.setCursor(LCD_SECTOR3, LCDBOTTOMROW);

  // If server is off, it is starting else it is stopping
  // or it is restarting
  if(serverStatus == SERVER_ON)  
    message(_POWEROFF_RUN);
  else if(serverStatus == SERVER_OFF)
    message(_POWERON_RUN);
  else if(serverStatus == SERVER_RESET)
    message(_POWER_RESTART);
}

/**
  \brief Initialize the temperature and fan fixed text

	prevFanSpeedPerc is used to reduce the number of display updates.
	Initializing the variable to -90 (that never will occour in the normal
	conditions) the value is forced for a first update when the program
	start else the 0% value (fan stopped) is shown only after the fan
	has started at least one time.
	
	prevTemp is used to reduce the number of display updates.
	Initializing the variable to an almost impossible value the startup
	condition forces a first update else the temperature is never shown until
	it does not changes at least one time.

*/
void initFanTemp() {
  lcd.clear();
  delay(100);
  lcd.setCursor(LCD_SECTOR1, LCDTOPROW);
  message(_TEMPERATURE);
  lcd.setCursor(LCD_SECTOR2, LCDTOPROW);
  message(_FANSPEED);

  prevFanSpeedPerc = -90;

  prevTemp = -10;
}

/**
  \brief Fan fixed text
*/
void testFan() {
  // Start the fan for test at max speed
   analogWrite(PWM_FAN, FANSPEED_MAX);
   delay(FAN_TEST_MS);
   analogWrite(PWM_FAN, FANSPEED_MIN);
}

/**
  \brief Update the display fan speed (in percentage)
  
  \param dFan current fan speed %
*/
void showFan(int dFan) {
  int outFan;
  
  if(dFan < 0)
    outFan = 0;
  else
    outFan = dFan;
  
  lcd.setCursor(LCD_SECTOR2 + LCD_SPEED_VAL_OFFSET, LCDTOPROW);
  lcd << _SPACING << _SPACING << _SPACING;
  lcd.setCursor(LCD_SECTOR2 + LCD_SPEED_VAL_OFFSET, LCDTOPROW);
  lcd << outFan << "%";
}

/**
  \brief Update the display temperature
  
  \param dTemp current temperature
*/
void showTemp(int dTemp) {
  lcd.setCursor(LCD_SECTOR1 + LCD_TEMP_VAL_OFFSET, LCDTOPROW);
  lcd << _SPACING << _SPACING;
  lcd.setCursor(LCD_SECTOR1 + LCD_TEMP_VAL_OFFSET, LCDTOPROW);
  lcd << dTemp << "C";
}

/**
 * \brief Display a string on the LCD at the cursor position
 *
 * \param m the message string
 */
void message(String m) {
	lcd.print(m);
}

/**
 * \brief Display an error message at the specified cursor coordinates
 *
 * The error message is shown for a LCDERROR_DELAY milliseconds.
 * After the timeout expires the screen is not cleared so the next steps should be managed
 * by the program flow. It is expected that error messages are shown in a calling code that
 * manages the error conditions.
 *
 * \param m the message string
 * \param x the cursor column zero based
 * \param y the row number zero based
 */
void error(String m, int x, int y) {
	message(m, x, y);
	delay(LCDERROR_DELAY);
}

/**
* \brief Display an error message at the cursor position
*
* The error message is shown for a LCDERROR_DELAY milliseconds.
* After the timeout expires the screen is not cleared so the next steps should be managed
* by the program flow. It is expected that error messages are shown in a calling code that
* manages the error conditions.
 *
 * \param m the string message
*/
void error(String m) {
	message(m);
	delay(LCDERROR_DELAY);
}

/**
 * \brief Display a string on the LCD at the specified cursor coordinates
 *
 * \param m the string message
 * \param x the cursor column zero based
 * \param y the row number zero based
 */
void message(String m, int x, int y) {
	lcd.setCursor(x, y);
	message(m);
}

/**
 * \brief Clean the display
 *
 * A delay is added after the hardware clear() call to give the display the time
 * to complete the operation.
 */
void clean() {
	lcd.clear();
	delay(LCDCLEAR_DELAY);
}


