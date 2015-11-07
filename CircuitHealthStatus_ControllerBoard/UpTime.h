/**
  \file Uptime.h
  \brief time constans, macros and functions prototypes
*/

#ifndef _UpTime_h
#define _UpTime_h

#include <inttypes.h>
#ifndef __AVR__
#include <sys/types.h> // for __time_t_defined, but avr libc lacks sys/types.h
#endif

#if !defined(__time_t_defined)
typedef unsigned long time_t;
#endif

//! Enumerator to identify the time types
typedef enum {
    tmSecond, tmMinute, tmHour, tmDay
} tmByteFields;	   

//! Structure defining the used millis() converted values in the proper
//! format
typedef struct  { 
  uint8_t Second; 
  uint8_t Minute; 
  uint8_t Hour; 
  uint8_t Day;
} 	tmElements_t, TimeElements, *tmElementsPtr_t;

typedef time_t(*getExternalTime)();

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
 
//! Macros for fast elapsed time calculation
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)

int     hour();            // the hour now 
int     hour(time_t t);    // the hour for the given time
int     minute();          // the minute now 
int     minute(time_t t);  // the minute for the given time
int     second();          // the second now 
int     second(time_t t);  // the second for the given time
int     day();             // the day now 
int     day(time_t t);     // the day for the given time

void now(unsigned long ms);
time_t now();
//void    setTime(time_t t);
void    adjustTime(long adjustment);

/* low level functions to convert to and from system time                     */
void breakTime(time_t time, tmElements_t &tm);  // break time_t into elements
time_t makeTime(tmElements_t &tm);  // convert time elements into time_t

#endif /* _Time_h */

