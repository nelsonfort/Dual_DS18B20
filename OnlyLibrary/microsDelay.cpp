// microsDelay.cpp
// see the tutorial https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html

/*
 * (c)2018 Forward Computing and Control Pty. Ltd.
 * NSW Australia, www.forward.com.au
 * This code is not warranted to be fit for any purpose. You may only use it at your own risk.
 * This generated code may be freely used for both private and commercial use
 * provided this copyright is maintained. 
 *
 * 2021-2 Creating Micros library.  -- NF
 *
 */

// include Arduino.h for micros()
#include <Arduino.h>
#include "microsDelay.h"

microsDelay::microsDelay() {
  running = false; // not running on start
  startTime = 0; // not started yet
  finishNow = false; // do not finish early
}

/**
   Start a delay of this many microseconds
   @param delay in microsconds, 0 means justFinished() will return true on first call
*/
void microsDelay::start(unsigned long delay) {
  uS_delay = delay;
  startTime = micros();
  running = true;
  finishNow = false; // do not finish early
}

/**
   Stop the delay
   justFinished() will now never return true
   until after start(),restart() or repeat() called again
*/
void microsDelay::stop() {
  running = false;
  finishNow = false; // do not finish early
}

/**
   repeat()
   Do same delay again but allow for a possible delay in calling justFinished()
*/
void microsDelay::repeat() {
  startTime = startTime + uS_delay;
  running = true;
  finishNow = false; // do not finish early
}

/**
   restart()
   Start the same delay again starting from now
   Note: use repeat() when justFinished() returns true, if you want a regular repeating delay
*/
void microsDelay::restart() {
  start(uS_delay);
}

/**
   Force delay to end now
*/
void microsDelay::finish() {
  finishNow = true; // finish early
}

/**
  Has the delay ended/expired or has finish() been called?
  justFinished() returns true just once when delay first exceeded or the first time it is called after finish() called
*/
bool microsDelay::justFinished() {
  if (running && (finishNow || ((micros() - startTime) >= uS_delay))) {
    stop();
    return true;
  } // else {
  return false;
}

/**
  Is the delay running, i.e. justFinished() will return true at some time in the future
*/
bool microsDelay::isRunning() {
  return running;
}

/**
  Returns the last time this delay was started, in uS, by calling start(), repeat() or restart()
  Returns 0 if it has never been started
*/
unsigned long microsDelay::getStartTime() {
	return startTime;
}

/**
  How many uS remaining until delay finishes
  Returns 0 if justFinished() returned true or stop() called
*/
unsigned long microsDelay::remaining() {
  if (running) {
    unsigned long uS = micros(); // capture current micros() as it may tick over between uses below
    if (finishNow || ((uS - startTime) >= uS_delay)) {  // check if delay exceeded already but justFinished() has not been called yet
      return 0;
    } else {
      return (uS_delay - (uS - startTime));
    }
  } else { // not running. stop() called or justFinished() returned true
    return 0;
  }
}

/**
  The delay set in uS set in start
*/
unsigned long microsDelay::delay() {
  return uS_delay;
}
