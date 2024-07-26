// Copyright 2024 James Roper 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Stepper motor controller
 * ------------------------
 *
 * This is the program for a stepper motor controller that I've built for
 * controlling my equatorial platform for my dobsonian telescope. The
 * parts that I've used to build this are:
 * 
 * - Arduino Nano V3.0 (equivalent) based on ATmega328
 * - A4988 stepper motor driver (though I'm replacing this with a TMC2209
 *   because the A4988 is too loud)
 * - 200 step bipolar stepper motor
 * - Adafruit 14-segment LED Alphanumeric Backpack, with HT16K33 driver
 * - PEC11R Rotary encoder module with push button
 * - LED momentary push button
 *
 * I power the setup with a 12V 9Ah SLA battery. While the Nano can run off
 * 12V with its own regulator, because I'm also powering the LED display off
 * the Nano's 5V pin, I don't want to overload it, so I use a 7808 voltage
 * regulator to drop the voltage to 8V before going to VIN. The Nano seems to
 * be fine with that, my temperature measurements indicate it's only a few
 * degrees above ambient.
 *
 * The LED momentary push button is used to start/stop the motor. The LED
 * itself is used to indicate if the motor is running or not.
 *
 * The rotary encoder is used to configure it. The push button allows you to
 * cycle through a menu, while the encoder allows you to set the value for
 * the menu item being configured. When cycling through the menu, the menu
 * title will scroll once across the display, before displaying the current
 * value. Turning the encoder will cause it to stop scrolling and immediately
 * display the value.
 *
 * The menu items are:
 *
 * - Speed. This is the speed of the stepper motor, measured in milliseconds
 *   between steps. Although it shows the speed down to 10ths of a
 *   microsecond, tuning it operates in 100 microsecond steps.
 * - Fine. This is for fine tuning the speed. It shows the speed down to the
 *   microsecond, and allows tuning it in microsecond increments. The most
 *   significant digit (ie, 10s of milliseconds) truncated.
 * - Dimmer. This controls the brightness of the display and running LED, and
 *   can be a value from 1 (least bright) to 16 (full brightness).
 * - Idle minutes. This is the number of minutes to wait before going into
 *   idle mode. In idle mode, an idle message scrolls across the screen. The
 *   motor will continue to run and can be turned on and off with the push
 *   button. To exit idle mode, push the menu button. Turning idle minutes
 *   below 1 will turn idle mode off.
 * - Direction. The direction that the motor will spin. The value can be zero
 *   or one. What these values mean depends on how you've wired the stepper
 *   motor, and how the motor is mechanically connected to your platform.
 *
 * The settings are persisted to EEPROM between power off/on, there's a 10
 * second delay between when you've last change the settings and when it
 * persists them to ensure the EEPROM isn't hammered (EEPROM is slow and only
 * supports a limited number of erase/write cycles).
 *
 * Unless your name is James too, I recommend changing the constants below for
 * the welcome message and idle message.
 *
 * This program uses a timer interrupt to drive the stepper motor, this
 * ensures other things like menu operation and don't impact the stepper
 * timing.
 */

#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#define DISPLAY_ADDRESS 0x70
#define DEBOUNCE_MILLIS 100

const Adafruit_AlphaNum4 display = Adafruit_AlphaNum4();

// This should match the wiring of the board
#define STEP 4 // STEP PIN of the A4988 driver
#define DIR 5 // DIR PIN of the A4988 driver
#define ENABLE 2 // ENABLE PIN of the A4988 driver, active low
#define ROTARY_A 7 // Rotary encoder A PIN
#define ROTARY_B 8 // Rotary encoder B PIN
#define MODE_SWITCH 9 // Rotary encoder switch PIN
#define RUNNING_SWITCH 6 // Running switch PIN
#define RUNNING_LED 3 // Running LED PIN, must be PWM capable

// Welcome and idle messages, change these to whatever you want.
#define WELCOME_MESSAGE "HELLO JAMES"
#define IDLE_MESSAGE "ENJOY CLEAR SKIES JAMES"

// The default direction is anti-clockwise
#define DIRECTION LOW

// These are the configuration parameters for the program. On start up, we
// read the configuration from EEPROM, these defaults are only used the first
// time we get run on a particular device.

// Brightness of the display and running LED. A value from 1 to 16.
int brightness = 4;

// Stepper motor pulse delay in microseconds.
// 1.5 RPM for a 200 step motor with 16 micro steps, approximately what my
// platform needs.
unsigned int stepperDelayMicros = 12500;

// The number of minutes before the display goes into idle mode. Zero turns
// idle mode off.
int idleMinutes = 2;

// The direction
uint8_t direction = DIRECTION;

void setup() {
  eepromInitialRead();

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(RUNNING_LED, OUTPUT);
  pinMode(ROTARY_A, INPUT);
  pinMode(ROTARY_B, INPUT);
  pinMode(MODE_SWITCH, INPUT);
  pinMode(RUNNING_SWITCH, INPUT);

  // Setup the interrupt for the stepper. We start with it disabled, since we
  // want to explicitly turn the motor on whenever the board starts up. We're
  // using timer1 because this gives us 16 bits of precision, which allows
  // microsecond accuracy for pulse delays of up to 65ms.
  cli();
  // Initialization, set everything to zero.
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   // initialize counter value to 0
  TIMSK1 = 0;  // initially we're disabled
  // We're running with an 8x prescaler, clock rate of 16Mhz, so the timer tick will be
  // 2Mhz. If we want to step every X microseconds, we need to interrupt at half that
  // delay, to send the high then low signals. So with two ticks at each microsecond
  // setting the compare match register to the micro second delay will do that.
  OCR1A = stepperDelayMicros;
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // 8x prescaler
  TCCR1B |= (1 << CS11);
  // Comment the above and uncomment the below to enable 1024x prescaling, useful for
  // development when flashing an LED rather than the motor
  // TCCR1B |= (1 << CS12) | (1 << CS10);
  sei();

  // Initialize the display
  display.begin(DISPLAY_ADDRESS);
  display.setBrightness(brightness);
  display.clear();

  // Initialize direction of the stepper motor
  digitalWrite(DIR, direction);
  // Ensure we start disabled - the ENABLE pin is active low.
  digitalWrite(ENABLE, HIGH);

  // read the encoder to initialise it
  readEncoder();

  // Display the welcome message, then display the first menu value.
  displayScrollOnce(WELCOME_MESSAGE, []() {
    displayUnsignedNumber(stepperDelayMicros / 10, 1);
  });
}

void loop() {
  inputLoop();
  displayLoop();
  eepromLoop();
}

/***********
 * Stepper *
 ***********/
bool running = false;
int stepState = LOW;

// Interrupt service routine for stepping
ISR(TIMER1_COMPA_vect) {
  if (stepState == LOW) {
    stepState = HIGH;
    digitalWrite(STEP, HIGH);
  } else {
    stepState = LOW;
    digitalWrite(STEP, LOW);
  }
}

/* Update the compare match register to the currently configured delay */
void updateStepperCMR() {
  // Because this is a long, we should disable interrupts while we write it to
  // ensure we don't write an inconsistent value.
  cli();
  OCR1A = stepperDelayMicros;
  sei();
}

void stop() {
  TIMSK1 = 0;  // disable timer1 interrupt

  running = false;
  stepState = LOW;
  digitalWrite(ENABLE, HIGH);
  digitalWrite(STEP, 0);
  analogWrite(RUNNING_LED, 0);
}

/*
 * Set the brightness of the running LED to the configured brightness, but
 * only if running. This is used to turn on the LED when we start running too.
 */
void updateRunningLedBrightness() {
  if (running) {
    analogWrite(RUNNING_LED, map(brightness, 1, 16, 4, 255));
  }
}

void start() {
  running = true;
  updateRunningLedBrightness();
  digitalWrite(DIR, direction);
  digitalWrite(ENABLE, LOW);
  // Docs say to wait a millisecond before sending ticks to the motor
  delay(1);

  TCNT1 = 0;                // set timer1 counter back to zero
  TIMSK1 |= (1 << OCIE1A);  // enable timer1 interrupt
}

/**********
 * EEPROM *
 **********/
// Magic string that gets written to the data first. This is used to detect if
// we're running on a device that hasn't been run on before, so that we don't
// load garbage. If we change the format of the EEPROM data, we can change this
// to "reset" it.
#define MAGIC "eqpf01"
// Writing to EEPROM is slow (takes multiple milliseconds), and EEPROM has
// limited erase/write cycles, so we don't want to write to it on every change
// of data. Instead, we wait until there's been no updates for 10 seconds, and
// then we persist.
#define EEPROM_PERSIST_DELAY_MILLIS 10000  // 10 seconds

// Set when any of our config params has been updated, but not yet persisted.
bool eepromDirty = false;
// When the last update happened.
unsigned long eepromLastUpdateMillis;

struct EepromData {
  // Must be the length of MAGIC plus one (for null terminator).
  char magic[7];
  unsigned int stepperDelayMicros;
  int brightness;
  int idleMinutes;
  uint8_t direction;
};

void eepromInitialRead() {
  EepromData data;
  EEPROM.get(0, data);
  // Check that the magic string in the EEPROM matches our magic string. If so,
  // use the data.
  if (strcmp(data.magic, MAGIC) == 0) {
    stepperDelayMicros = data.stepperDelayMicros;
    brightness = data.brightness;
    idleMinutes = data.idleMinutes;
    direction = data.direction;
  } else {
    // Otherwise, persist the default data.
    eepromPersist();
  }
}

void eepromPersist() {
  EepromData data;
  strcpy(data.magic, MAGIC);
  data.stepperDelayMicros = stepperDelayMicros;
  data.brightness = brightness;
  data.idleMinutes = idleMinutes;
  data.direction = direction;
  EEPROM.put(0, data);
}

void eepromLoop() {
  if (eepromDirty && millis() - eepromLastUpdateMillis > EEPROM_PERSIST_DELAY_MILLIS) {
    eepromDirty = false;
    eepromPersist();
  }
}

void eepromTouch() {
  eepromDirty = true;
  eepromLastUpdateMillis = millis();
}

/******************
 * INPUT HANDLING *
 ******************/
// Idle is a special mode, we can't cycle into it, but when in the idle mode,
// if we push the mode button, we'll end up in the speed mode.
#define CONFIG_MODE_IDLE -1
#define CONFIG_MODE_SPEED 0
#define CONFIG_MODE_FINE 1
#define CONFIG_MODE_BRIGHTNESS 2
#define CONFIG_MODE_IDLE_MINUTES 3
#define CONFIG_MODE_DIRECTION 4
// The number of config modes.
#define CONFIG_MODES 5

int configMode = CONFIG_MODE_SPEED;
bool debouncing = false;
unsigned long debounceStartMillis = 0;

// These are used for a debouncing rotary encoder.
uint8_t oldAB = 3;                                                                      // Lookup table index
int8_t encoderValue = 0;                                                                // Encoder value
const int8_t encoderStates[] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };  // Lookup table

// The rotary encoder switch is LOW active, so we need to start HIGH.
int modeSwitchState = HIGH;
int runningSwitchState = LOW;

unsigned long idleStartMillis = 0;

void inputLoop() {
  int encDir = readEncoder();
  if (encDir != 0) {
    idleReset();
    handleEncoderTurn(encDir);
  }

  // Global debounce. We don't expect multiple switches to be pushed at the
  // same time, so when any switch changes state, we wait for the debounce
  // delay before accepting any further input from any switch.
  if (debouncing) {
    if (millis() - debounceStartMillis >= DEBOUNCE_MILLIS) {
      debouncing = false;
    } else {
      return;
    }
  }

  if (pushed(MODE_SWITCH, &modeSwitchState, LOW)) {
    // Reset start time for idle timeout.
    idleReset();
    configMode = (configMode + 1) % CONFIG_MODES;
    switch (configMode) {
      case CONFIG_MODE_SPEED:
        displayScrollOnce("SPEED", []() {
          displayUnsignedNumber(stepperDelayMicros / 10, 1);
        });
        break;
      case CONFIG_MODE_FINE:
        displayScrollOnce("FINE", []() {
          displayUnsignedNumber(stepperDelayMicros, 0);
        });
        break;
      case CONFIG_MODE_BRIGHTNESS:
        displayScrollOnce("DIMMER", []() {
          displayUnsignedNumber(brightness, 4);
        });
        break;
      case CONFIG_MODE_IDLE_MINUTES:
        displayScrollOnce("IDLE MINUTES", []() {
          if (idleMinutes == 0) {
            displayStaticText("Off", 4);
          } else {
            displayUnsignedNumber(idleMinutes, 4);
          }
        });
        break;
      case CONFIG_MODE_DIRECTION:
        displayScrollOnce("DIRECTION", []() {
          displayUnsignedNumber(direction, 4);
       });
       break;
    }
  }

  if (pushed(RUNNING_SWITCH, &runningSwitchState, HIGH)) {
    if (running) {
      stop();
    } else {
      start();
    }
  }

  if (idleMinutes > 0 && configMode != CONFIG_MODE_IDLE && millis() - idleStartMillis > idleMinutes * 60000l) {
    // If we've been idle for the configured idle minutes, go into idle mode.
    configMode = CONFIG_MODE_IDLE;
    displayScroll(IDLE_MESSAGE);
  }
}

void idleReset() {
  idleStartMillis = millis();
}

void handleEncoderTurn(int encDir) {
  switch (configMode) {
    case CONFIG_MODE_SPEED:
      updateStepperDelayMicros(encDir * 100);
      displayUnsignedNumber(stepperDelayMicros / 10, 1);
      break;
    case CONFIG_MODE_FINE:
      updateStepperDelayMicros(encDir);
      displayUnsignedNumber(stepperDelayMicros, 0);
      break;
    case CONFIG_MODE_BRIGHTNESS:
      brightness += encDir;
      if (brightness < 1) {
        brightness = 1;
      } else if (brightness > 16) {
        brightness = 16;
      }
      display.setBrightness(brightness);
      updateRunningLedBrightness();
      displayUnsignedNumber(brightness, 4);
      break;
    case CONFIG_MODE_IDLE_MINUTES:
      idleMinutes += encDir;
      if (idleMinutes < 0) {
        idleMinutes = 0;
      } else if (idleMinutes > 30) {
        idleMinutes = 30;
      }
      if (idleMinutes == 0) {
        displayStaticText("Off", 4);
      } else {
        displayUnsignedNumber(idleMinutes, 4);
      }
      break;
    case CONFIG_MODE_DIRECTION:
      direction ^= HIGH;
      // This isn't needed, but when I first ran this on my Arduino I had a
      // bug and the value was set to 255, this ensures it's either 1 or 0.
      // This will fix any such data corruptions.
      direction &= HIGH;
      displayUnsignedNumber(direction, 4);
      break;

    case CONFIG_MODE_IDLE:
      return;
  }
  eepromTouch();
}

void updateStepperDelayMicros(int increment) {
  stepperDelayMicros += increment;
  if (stepperDelayMicros < 1000) {
    stepperDelayMicros = 1000;
  }
  if (stepperDelayMicros > 60000) {
    stepperDelayMicros = 60000;
  }
  updateStepperCMR();
}

/* Returns true if the button has changed state from up to down */
bool pushed(int pin, int *lastState, int pushedState) {
  int state = digitalRead(pin);
  if (state != *lastState) {
    debouncing = true;
    debounceStartMillis = millis();
    *lastState = state;
    return *lastState == pushedState;
  } else {
    return false;
  }
}

int readEncoder() {
  oldAB <<= 2;  // Remember previous state

  if (digitalRead(ROTARY_A)) oldAB |= 0x02;
  if (digitalRead(ROTARY_B)) oldAB |= 0x01;

  encoderValue += encoderStates[(oldAB & 0x0f)];

  if (encoderValue > 3) {
    encoderValue = 0;
    return -1;
  } else if (encoderValue < -3) {
    encoderValue = 0;
    return 1;
  }
  return 0;
}

/********************
 * DISPLAY HANDLING *
 ********************/
#define DISPLAY_MODE_SCROLL_ONCE 0
#define DISPLAY_MODE_SCROLL 1
#define DISPLAY_MODE_STATIC 2
#define DISPLAY_SCROLL_DELAY_MILLIS 300
String displayText;
void (*displayNext)();
unsigned int displayMode;
int displayPointer;
long displayLastRenderMillis;

/*
 * Scroll the given text once, and once done, invoke the passed in next
 * function. Typically, the next function should tell the display what to
 * display next.
 *
 * Note that if any other method to display something is invoked while
 * scrolling is happening, that will change the display mode and display
 * whatever that method requires. This means after entering a mode, the
 * operator can start immediately using the rotary encoder and it because
 * that updates the display as you increment through the value, the scrolling
 * will stop and the value will be displayed.
 */
void displayScrollOnce(String text, void (*next)()) {
  displayText = text;
  displayNext = next;
  displayMode = DISPLAY_MODE_SCROLL_ONCE;
  displayPointer = 0;
  displayRenderScroll();
}

/**
 * Display the given text, scrolling continuously.
 */
void displayScroll(String text) {
  displayText = text;
  displayMode = DISPLAY_MODE_SCROLL;
  displayPointer = 0;
  displayRenderScroll();
}

/*
 * Display the given number, inserting a decimal point at the given position.
 * If number > 9999, the most significant digits will be trimmed. If decimalPos
 * >= 4, no decimal point will be shown.
 */
void displayUnsignedNumber(unsigned int number, int decimalPos) {
  String text = String(number % 10000);
  displayStaticText(text, decimalPos);
}

/*
 * Display the given text. If text is more than four characters, the last four
 * characters will be displayed.
 */
void displayStaticText(String text, int decimalPos) {
  displayMode = DISPLAY_MODE_STATIC;
  for (int i = 0; i < 4; i++) {
    int nextCharIndex = text.length() - 4 + i;
    if (nextCharIndex < 0) {
      if (i >= decimalPos) {
        display.writeDigitAscii(i, '0', decimalPos == i);
      } else {
        display.writeDigitAscii(i, ' ');
      }
    } else {
      display.writeDigitAscii(i, text.charAt(nextCharIndex), decimalPos == i);
    }
  }
  display.writeDisplay();
}

void displayLoop() {
  switch (displayMode) {
    case DISPLAY_MODE_SCROLL_ONCE:
      if (millis() - displayLastRenderMillis >= DISPLAY_SCROLL_DELAY_MILLIS) {
        // If we've reached the end of the text to display, invoke the next
        // callback.
        if (displayPointer >= displayText.length() + 4) {
          displayNext();
        } else {
          displayRenderScroll();
        }
      }
      break;
    case DISPLAY_MODE_SCROLL:
      if (millis() - displayLastRenderMillis >= DISPLAY_SCROLL_DELAY_MILLIS) {
        // If we've reached the end of the text to display, reset to the
        // beginning.
        if (displayPointer >= displayText.length() + 4) {
          displayPointer = 0;
        }
        displayRenderScroll();
      }
      break;
  }
}

/* Render the next scroll increment */
void displayRenderScroll() {
  for (int i = 0; i < 4; i++) {
    int toDisplay = displayPointer - 4 + i;
    if (toDisplay >= 0 && toDisplay < displayText.length()) {
      display.writeDigitAscii(i, displayText.charAt(toDisplay));
    } else {
      display.writeDigitAscii(i, ' ');
    }
  }
  display.writeDisplay();
  displayPointer++;
  displayLastRenderMillis = millis();
}
