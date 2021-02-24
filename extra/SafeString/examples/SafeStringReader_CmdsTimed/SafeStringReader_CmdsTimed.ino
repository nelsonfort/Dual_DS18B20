// SafeStringReader_CmdsTimed.ino
//
// Example of NON-Blocking read commmands from the Arduino Monitor input and acts on them
// the available commands are start stop
// Commands are delimited by space dot comma NL or CR, OR if there is no more input for 2secs
// If you set the Arduino Monitor to No line ending then the last command will be processed after 2sec
//
// These commands can be picked out of a line of user input
// start  stop
// The input line can be as long as you like 100's of Kb long, but only two small buffers need to parse the commands
//
// download and install the SafeString library from
// www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
#include "SafeStringReader.h"

// create an sfReader instance of SafeStringReader class
// that will handle commands upto 5 chars long
// delimited by space, comma or CarrageReturn or NewLine
// the createSafeStringReader( ) macro creates both the SafeStringReader (sfReader) and the necessary SafeString that holds input chars until a delimiter is found
// args are (ReaderInstanceName, expectedMaxCmdLength, delimiters)
createSafeStringReader(sfReader, 5, " ,\r\n");

bool running = true;
unsigned long loopCounter = 0;

void setup() {
  Serial.begin(9600);
  for (int i = 10; i > 0; i--) { // pause a little to give you time to open the Arduino monitor
    Serial.print(i); Serial.print(' '); delay(500);
  }
  Serial.println();
  SafeString::setOutput(Serial); // enable error messages and SafeString.debug() output to be sent to Serial
  if (running) {
    Serial.println(F(" Counter Started"));
  }
  sfReader.connect(Serial); // where SafeStringReader will read from
  sfReader.echoOn(); // echo back all input, by default echo is off
  sfReader.setTimeout(2000); // 2000mS = 2sec timeout
}

void handleStartCmd() {
  running = true;  Serial.println(); Serial.print(F("> start at Counter:")); Serial.println(loopCounter);
}
void handleStopCmd() {
  running = false; Serial.println(); Serial.print(F("> stop at Counter:")); Serial.println(loopCounter);
}

void loop() {
  if (sfReader.read()) {
    if (sfReader == "start") {
      handleStartCmd();
    } else if (sfReader == "stop") {
      handleStopCmd();
    } // else ignore unrecognized command
  } // else no delimited command yet

  // rest of code here is executed while the user typing in commands
  if (running) {
    loopCounter++;
    if ((loopCounter % 100000) == 0) { // print the current counter every now and again
      Serial.print(F("Counter:")); Serial.println(loopCounter);
    }
  }
}
