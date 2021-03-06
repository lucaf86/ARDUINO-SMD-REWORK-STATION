/*
 * push_button_callback.ino
 * 
 * Example program for the mdPushButton library
 * <https://github.com/sigmdel/mdPushButton>
 *
 */

// SPDX-License-Identifier: 0BSD 

#include <Arduino.h>           // for PlatformIO
#include "mdPushButton.h"

// Set the following macros if needed. In PlatformIO, the
// macros can be defined in the platformio.ini configuration file 

#ifndef SERIAL_BAUD
  #define SERIAL_BAUD 9600
#endif

#ifndef BUTTON_PIN
  #define BUTTONN_PIN 7
#endif

#ifndef ACTIVE
  #define ACTIVE LOW
#endif

#ifndef DEBUG_PUSH_BUTTON
  #define DEBUG_PUSH_BUTTON 0 
  // 0 - no debug, 1 - printSetup() available, 2 - adds state machine debug output 
  // Arduino IDE: it will be necessary to add this define in mdPushButton.h if not set to 0
#endif 

// ---------------------------------------------------


#if (ACTIVE == HIGH) 
  // Connection of an active HIGH normally open button would be
  //                         _________________
  //           __||__       |
  // Vcc ------o    o--+---=|BUTTON_PIN (GPIO)
  //                   |    |________________
  //         ____      |   
  // GND ---[____]-----+  External pull-down resistor (optional in some cases)
  //              
  // If there's no external pull-down resistor, set useInternalPullResistor to true
  // Many micro-controllers do not have internal pull-down resistors 
  //
  mdPushButton button = mdPushButton(BUTTON_PIN, HIGH, true);
#else  
  // Connection of an active LOW normally open button would be
  //                         ________________
  //           __||__       |
  // GND ------o    o--+---=|BUTTON_PIN (GPIO)
  //                   |    |________________
  //         ____      |   
  // Vcc ---[____]-----+  External pull-up resistor (optional in most cases)
  //              
  // If there's no external pull-up resistor, set useInternalPullResistor to true
  // Most micro-controlers have internal pull-up resistors
  // 
  // Acive LOW normally open push button without an external pull-up resistor
  // is the default connection
  //
  mdPushButton button = mdPushButton(BUTTON_PIN);
#endif

void DumpButtonTimings(char * msg = NULL) {
  if (msg) Serial.println(msg);
  Serial.print("  Debounce press time: ");
  Serial.println(button.setDebouncePressTime(0xFFFF));

  Serial.print("  Debounce release time: ");
  Serial.println(button.setDebounceReleaseTime(0xFFFF));

  Serial.print("  Multiple click wait time: ");
  Serial.println(button.setMultiClickTime(0xFFFF));

  Serial.print("  Hold time: ");
  Serial.println(button.setHoldTime(0xFFFF));

  Serial.println("  All times in milliseconds");
}

void ButtonPressed(int clicks) {
  switch (clicks) {
    case -1: Serial.println("Long button press"); break;
    case  1: Serial.println("Button pressed once"); break;
  default  : Serial.print("Button pressed "); Serial.print(clicks); Serial.println(" times"); break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) delay(10);
  #if defined(ESP8266)
    Serial.println("\n"); // skip boot garbage
  #endif   

  Serial.println("\nsetup() starting...");

  button.OnButtonClicked(ButtonPressed);

  #if (DEBUG_PUSH_BUTTON > 0)
  button.printSetup();
  #endif

  Serial.println("Using a more aggressive timing");

  Serial.print("Initial debounce press time: ");
  Serial.println(button.setDebouncePressTime(10));
  Serial.println("Debounce press set to 10 ms");

  Serial.print("Initial debounce release time: ");
  Serial.println(button.setDebounceReleaseTime(20));
  Serial.println("Debounce release time set to 20 ms");

  Serial.print("Initial mutiple click wait time: ");
  Serial.println(button.setMultiClickTime(300));
  Serial.println("Multiple click wait time set to 300 ms");

  Serial.print("Initial hold time: ");
  Serial.println(button.setHoldTime(1500));
  Serial.println("Hold time set to 1500 ms");
 
  Serial.println("setup() completed.");
}

void loop() {
  button.status();
  delay(40 + random(21)); // rest of loop takes 40 to 60 ms to execute 
}
