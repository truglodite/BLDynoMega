# Brushless Dyno Mega
<img src="https://static.rcgroups.net/forums/attachments/1/4/2/0/9/0/a10074131-131-dynoData.jpg" width="600"><img src="https://static.rcgroups.net/forums/attachments/1/4/2/0/9/0/a10074102-31-20160917_151711.jpg" width="200">

This code provides for a small BLDC motor dynamometer machine using an Arduino Uno or Mega. For Mega the pot controlled esc is enabled, for Uno etc it is disabled. The code outputs time, samples, (throttle), RPM, voltage, current, thrust, and torque through hardware serial. The output can be read, copied, and analyzed on a PC using your favorite serial terminal software (Arduino IDE, Putty, RealTerm, Atom, etc...), or stored on an SD card serial logger for later use. A sample period is set at compile time, which controls the time between printed lines of data, and the characteristics of the averaging filters.

## Sensors
- RPM
Eagle tree, or similar DIY schmitt trigger type 5V RPM sensors are supported via the FreqCount library. RPM is not averaged, because FreqCount requires a relatively long gate time (>100ms) for decent accuracy with essentially all BLDC setups we are interested in. Unfortunately for now, motor magnet count must be compiled in firmware.

- Torque & Thrust
A pair of HX711 24bit strain gage ADC/PGA's are wired to 1kg load cells for torque and thrust. Calibration weights (gm) and torques (gm*cm) are hardcoded at compile time as well, however this is not of much inconvenience since most setups will only use one weight for calibration. The moment arm acting on the torque load cell must be taken into account when entering your #define torque calibration load.

- Current & Voltage
An ADS1115 i2c 16bit ADC is used to convert current and voltage signals. An ACS71X hall sensor is used for current, and battery voltage comes from a precision voltage divider (<30ppm/C). This leaves adc's 3 and 4 free for future expansion. Both current and battery calibration slopes are hardcoded as well. The comments in the code give values for some commonly sourced components.

## Tare & Calibration
Tare and calibration routines average 500 (default) samples before storing. Tare by simply hitting the tare button. Calibration is a 4 step process, controlled by the calibration button, and guided by an LED indicator to show progress. 1 flash = torque tare, 2 flashes = torque load, 3 flashes = thrust tare, 4 flashes = thrust load. When each sequence is prepared physically, hit the calibration button to go to the next sequence until finished. When done, linear calibration slopes are stored to EEPROM, and will be saved between reboots. Tare must be used after every reboot. The data header row is printed after tare.

## Running
After taring Brushless Dyno Mega, hit the run button to start streaming data to serial. Hit run again to stop the data at any time.

## Throttle Control
Version Mega adds esc control using a pot. The esc is controlled with a custom Servo library. The modification simply comments out timer 5 in ServoTimers.h for compatibility with FreqCount, which uses timers 2 and 5. To do this, you may copy the Servo folder from Arduino libraries (in C:\Arduino X.X.X\) to your custom library folder (in /documents/). Modify the copied library and at compile time, with verbose enabled, you should see a comment that the proper modified
Servo is used. You can also use the #error trick to verify which one is being compiled... add #error anywhere in the ServoTimers.h you intend to use, and it should stop the compile with #error.

## Performance
This project averages 0.6-0.8ms between analog samples, with a sample gate of 10ms, at 57k baud, and 100ms RPM gate time on a mega328 chip. The same settings for 2560 average 4ms per sample from a mix of 3 ADC's (built in, HX711, and ADS1115). The HX711's support 80 or 10 samples per second (13ms or 100ms per conversion). The MCU waits for serial flush (keeps all the data), but RPM samples are very coarse. So default timing parameters are good enough to match, and they play well with most serial radios/loggers. You can of course play with timing to suit your needs as you wish.

## Mega Pinout
Pin | Connect to
--- | --------
A0 | throttle potentiometer (0-5V)
D0 | to serial RX (ex: Open Log, data radio, RealTerm...)
D1 | to serial TX (for future UI purposes)
D2 | Torque HX711 clock pin
D3 | Force HX711 clock pin
D4 | Torque HX711 data pin
D6 | calibration button to ground, no resistors (internal pullups used)
D7 | tare button...
D8 | start switch...
D12 | Force HX711 data pin (inamp to Mega A1 for coarse thrust... just one hx711)
D13 | status indicator (onboard LED)
D14 | brushless esc signal wire
D20 | SDA - ADS1115 data pin
D21 | SCL - ADS1115 clock pin
D47 | RPM sensor (Eagle Tree or DIY), RPM = count*60/2/#poles

## Nano Pinout
Pin | Connect to
---- | ----------
A4 | SDA - ADS1115 data pin
A5 | SCL - ADS1115 clock pin
D0 | to serial RX (ex: Open Log, data radio, RealTerm...)
D1 | to serial TX (for future UI purposes)
D2 | Torque HX711 clock pin
D3 | Force HX711 clock pin
D4 | Torque HX711 data pin
D5 | RPM sensor (Eagle Tree or DIY), RPM = count*60/2/#poles
D6 | calibration button to ground, no resistors (internal pullups used)
D7 | tare button...
D8 | start switch...
D12 | Force HX711 data pin
D13 | status indicator (onboard LED)

## ADS1115 Analog Inputs
A0 | vbatt (10k:2.4k precision battery voltage divider... 25V max)
A1 | current sensor (ACS7XX hall sensor)
ADR - GND | Sets default i2c address of 0x48
