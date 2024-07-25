# Stepper motor controller

This is the program for a stepper motor controller that I've built for controlling my equatorial platform for my dobsonian telescope. The parts that I've used to build this are:
 
* Arduino Nano V3.0 (equivalent) based on ATmega328
* A4988 stepper motor driver (though I'm replacing this with a TMC2209 because the A4988 is too loud)
* 200 step bipolar stepper motor
* Adafruit 14-segment LED Alphanumeric Backpack, with HT16K33 driver
* PEC11R Rotary encoder module with push button
* LED push button

I power the setup with a 12V 9Ah SLA battery. While the Nano can run off 12V with its own regulator, because I'm also powering the LED display off the Nano's 5V pin, I don't want to overload it, so I use a 7808 voltage regulator to drop the voltage to 8V before going to VIN. The Nano seems to be fine with that, my temperature measurements indicate it's only a few degrees above ambient.

The LED momentary push button is used to start/stop the motor. The LED itself is used to indicate if the motor is running or not.

The rotary encoder is used to configure it. The push button allows you to cycle through a menu, while the encoder allows you to set the value for the menu item being configured. When cycling through the menu, the menu title will scroll once across the display, before displaying the current value. Turning the encoder will cause it to stop scrolling and immediately display the value.

The menu items are:

* **Speed** - This is the speed of the stepper motor, measured in milliseconds between steps. Although it shows the speed down to 10ths of a microsecond, tuning it operates in 100 microsecond steps.
* **Fine** - This is for fine tuning the speed. It shows the speed down to the microsecond, and allows tuning it in microsecond increments. The most significant digit (ie, 10s of milliseconds) truncated.
* **Dimmer** - This controls the brightness of the display and running LED, and can be a value from 1 (least bright) to 16 (full brightness).
* **Idle minutes** - This is the number of minutes to wait before going into idle mode. In idle mode, an idle message scrolls across the screen. The motor will continue to run and can be turned on and off with the push button. To exit idle mode, push the menu button. Turning idle minutes below 1 will turn idle mode off.
* **Direction** - The direction that the motor will spin. The value can be zero or one. What these values mean depends on how you've wired the stepper motor, and how the motor is mechanically connected to your platform.

The settings are persisted to EEPROM between power off/on, there's a 10 second delay between when you've last change the settings and when it persists them to ensure the EEPROM isn't hammered (EEPROM is slow and only supports a limited number of erase/write cycles).

Unless your name is James too, I recommend changing the constants below for the welcome message and idle message.

This program uses a timer interrupt to drive the stepper motor, this ensures other things like menu operation and don't impact the stepper timing.
