/**
 *	\file LCD.h
 *  \brief LCD display Manager include file
 *
 *  Methods to manage the LCD output and display features, including some hard-coded strings like the welcome message.
*/

#ifndef __LCD_H__
#define __LCD_H__

#include <inttypes.h>
#include <Print.h>
#include <AlphaLCD.h>
#include <Streaming.h>

//! LCD Shift control pin - Clock signal
//! Define this value accordingly with the Arduino board connections
#define LCDclockPin 4
//! LCD Shift control pin - Latch signal
//! Define this value accordingly with the Arduino board connections
#define LCDlatchPin 5
//! LCD Shift control pin - Data signal
//! Define this value accordingly with the Arduino board connections
#define LCDdataPin 6
//! Display characters per line
//! Define this value accordingly with the LCD Hardware datasheet
#define LCDCHARS 16
//! Display rows
#define LCDROWS 2
//! The top row number of the LCD
#define LCDTOPROW 0
//! The bottom row number of the LCD
#define LCDBOTTOMROW 1
//! Top Left display sector column
#define LCD_SECTOR1 0
//! Top Right display sector column
#define LCD_SECTOR2 LCDCHARS / 2
//! Bottom Left display sector column
#define LCD_SECTOR3 0
//! Bottom Right display sector column
#define LCD_SECTOR4 LCDCHARS / 2

//! Temperature value offset position (right to the text)
//! Depends on the text lenght defined in Strings.h
#define LCD_TEMP_VAL_OFFSET 5
//! Speed perc. value offset position (right to the text)
//! Depends on the text lenght defined in Strings.h
#define LCD_SPEED_VAL_OFFSET 4
//! Uptime variable text offset
#define LCD_UPTIME_OFFSET 3

//! Delay after showing an error
#define LCDERROR_DELAY 5000
//! Delay after showing a temporary message e.g. the welcome screen
#define LCDMESSAGE_DELAY 5000
//! Delay after a clear display call to hardware has been done.
#define LCDCLEAR_DELAY 50

/**
 *\brief Manages the Alphanumeric display for program output messages.
 *
 * This class implements the \e AlphaLCD class that manages the Alphanumeric LCD display
 * hardware using three digital Arduino pins via a shift-out register.
 */
class LCD: public AlphaLCD {
//variables
public:
protected:
private:
	AlphaLCD lcd;								///< AlphaLCD class inherited instance

//functions
public:
	LCD();
	~LCD();
	void enable(bool s);				///< Set the display on or off
	void blink(bool set);				///< Set blink mode
	void error(String m);				///< shows an error message
	void error(String m, int x, int y);		///< shows an error message at specified coordinates
	void message(String m);				///< shows a string message
	void message(String m, int x, int y);		///< shows a string message at specified coordinates
	void clean();					///< clean the LCD screen
	void welcome();					///< shows the welcome message
        // Applicaion specific methods
        void showFan(int dFan);
        void showTemp(int dTemp);
        void initFanTemp();
        void showReset();
        void showPowerOn();
        void showPowerOff();
        void showAction();
        void initUptime();
        void showServerStartingStopping();
protected:
private:
	LCD( const LCD &c );
	LCD& operator=( const LCD &c );

}; //LCD

#endif //__LCD_H__
