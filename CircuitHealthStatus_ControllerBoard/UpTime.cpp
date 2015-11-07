/*
  UpTime.cpp
  
  Manages 
 
 */

#include "UpTime.h"

//! Value read from the millis() call in the calling program
unsigned long nowMillis = 0;

static tmElements_t tm;    ///< Last converted time values cached in the structure
static time_t cacheTime;   ///< Last time values used to update the time cache

/**
  \brief Refresh the cached time structure if time has changed from last call
  
  \param t Global time from the last power-on
*/
void refreshCache(time_t t) {
  if (t != cacheTime) {
    breakTime(t, tm); 
    cacheTime = t; 
  }
}

/**
  \brief Elapsed hours from the last day
*/
int hour() { 
  return hour(now()); 
}

/**
  \brief Elapsed hours from the last day
  
  \param t Global time from the last power-on
*/
int hour(time_t t) {
  refreshCache(t);
  return tm.Hour;  
}

/**
  \brief Elapsed minutes from the last hour
*/
int minute() {
  return minute(now()); 
}

/**
  \brief Elapsed minutes from the last hour
  
  \param t Global time from the last power-on
*/
int minute(time_t t) {
  refreshCache(t);
  return tm.Minute;  
}

/**
  \brief Elapsed seconds from the last minute
*/
int second() {
  return second(now()); 
}

/**
  \brief Elapsed seconds from the last minute
  
  \param t Global time from the last power-on
*/
int second(time_t t) {
  refreshCache(t);
  return tm.Second;
}

/** 
  \brief Elapsed days from the last power-on
*/
int day(){
  return(day(now())); 
}

/** 
  \brief Elapsed days from the last power-on
  
  \param t Global time from the last power-on
*/
int day(time_t t) {
  refreshCache(t);
  return tm.Day;
}
 
/** 
  \brief Split the time in seconds from the last power-on in days, hh:mm:ss
  
  Break the given time_t into time components
  
  \param timeInput Global time from the last power-on
*/  
void breakTime(time_t timeInput, tmElements_t &tm){

  uint8_t year;
  uint8_t month, monthLength;
  uint32_t time;
  unsigned long days;

  time = (uint32_t)timeInput;
  tm.Second = time % 60;
  time /= 60; // now it is minutes
  tm.Minute = time % 60;
  time /= 60; // now it is hours
  tm.Hour = time % 24;
  tm.Day = time / 24; // now it is days
  
}

/**
  \brief Assemble time elements into time_t 
  
  \param tm the time components structure pointer
*/
time_t makeTime(tmElements_t &tm){   
  int i;
  uint32_t seconds;

  seconds += (tm.Day-1) * SECS_PER_DAY;
  seconds += tm.Hour * SECS_PER_HOUR;
  seconds += tm.Minute * SECS_PER_MIN;
  seconds += tm.Second;

  return (time_t)seconds; 
}

static uint32_t sysTime = 0;
static uint32_t prevMillis = 0;
static uint32_t nextSyncTime = 0;
//static timeStatus_t Status = timeNotSet;

/**
  \brief Set the last millisecons from the last power-on
  
  This is the value used for calculations and should be updated
  everytime it is needed a new time split calculation with a call
  to the millis() method.
  
  \param ms Milliseconds from the last power-on
*/
void now(unsigned long ms) {
 nowMillis = ms;
}

/**
  \brief Saves the updated milliseconds time if one second has been elapsed.
  
  \note Best practice to have a reliable updated time since the last power-on
  requires that the main loop() function call the update more times during the same
  1000 ms elapsed (e.g. every 250 ms)
  
  \param ms Milliseconds from the last power-on
*/
time_t now() {
  if( (nowMillis - prevMillis) >= 1000) {
    sysTime++;
    prevMillis += 1000;	
  }

  return (time_t)sysTime;
}

