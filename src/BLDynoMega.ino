//  BLDynoMega... and Uno too!
//  by: truglodite
//  updated: 5/25/2018
//
// todos:
//  throttle print bug (0-15% should be 0-100%)
//  current slope bug (incorrect value used... add documentation)
//  Add LCD
//  mag count and calibration UI (LCD?)
//  EEPROM wear leveling
//  temperature sensor support (Dallas i2c).
//  menu for manual or sensed temp
//////////////////////////////////////////////////////////////

#include <EEPROM.h>
#include <FreqCount/FreqCount.h>
#include <Q2HX711.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #include <Servo.h>
#endif

//User defined parameters----------------------
float nMagnets = 12.0;                         //number of magnets on the motor (2x poles)

float forceCalLoad = 356.0;                    //physical force calibration load (gm)
float torqueCalLoad = 904.0;                   //physical torque calibration load (gm*cm)

float vbattDivider = 0.2810;                   //voltage divider Vout/Vin... see table below
//R1/R2     Vout/Vin    Vmax (5V)   Vmax(3.3V)
//10k/2k4   0.1935      25.8V       17.1V
//10k/3k9   0.281       17.8V       11.7V

float currentSensor = 0.400;                   //Calibrated Vout/Ain for your sensor... see table below
//Allegro Model   Vout/Ain    Range   Type
//ACS714          0.100       20A     AC
//ACS7xx-20AU     0.200       20A     DC
//ACS7xx-10AU     0.400       10A     DC

unsigned int samplePeriod = 12; //delay between printed lines [milliseconds]
  //(10 default... 10 causing 'force spikes' during run, because sample time is 10ms max) *note loop times average 0.8ms
unsigned int tgate = 100;  //gate time for FreqCount [milliseconds] (100 default, ***MUST BE >= samplePeriod***NAN)
unsigned int baud = 57600;                     //serial baud rate (57600 default, 115200 doesn't work???)
char dataSeparator[] = ",";                    //comma separated values
int eepromTorqueAddress = 2 * 2 * sizeof(float);//address to store torque calibration slope (rotate for wear leveling if used often)
int eepromForceAddress = eepromTorqueAddress + sizeof(float);
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  unsigned int throttleDelay = 15;             //minimum time (ms) before throttle change is sent
#endif

//Analog Pin definitions-----------------------
//A4 and A5 pins are used for i2c with 328.
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  const byte throttlePot = 0;                  //A0 Throttle Potentiometer
#endif

//Digital Pin definitions----------------------
//Digital pins 0 and 1 are reserved for Serial
//Digital pin 47 is reserved for the FreqCount library (timers 1 & 2 are used, or 1 & 5 on mega)
//Digital pins 20 and 21 are reserved for i2c on Mega
//(328 noPWM: 3, 9, 10, 11) (mega noPWM: 9, 10, 44, 45, 46)

const byte forceClockPin = 3;
const byte torqueClockPin = 2;
const byte forceDataPin = 12;
const byte torqueDataPin = 4;


const byte startPin = 8;
const byte tarePin = 7;
const byte calPin = 6;
const byte ledPin = 13;                        //onboard LED
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  const byte escPin = 14;                      //BLDC speed controller
#endif
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  int throttle = 0;                            //throttle pot raw value
  int throttleMin = 1000;                      //Min throttle command in microseconds
  int throttleMax = 2000;                      //...
  int throttleOff = 900;                      //...
  unsigned long tthrottle = 0;                 //time of last throttle change
  Servo speedControl;
#endif

//Global variables (Holders)-----------------------------
unsigned long tstamp = 0;                      //time stamp
unsigned long tsample = 0;                     //time of last sample
float rpm = 0.0;                               //RPM
float rpmFactor = 0.0;                         //convert from Hz to RPM (based on gate time & pole count)
float samples = 0.0;                           //sample count
byte releaseWait = 1;                          //start button release routine trigger
byte dataset = 1;                              //dataset #

float tareN = 500.0;                           //# of measurements to average before saving to EEPROM
float calN = 500.0;                            //# of measurements to average before saving to EEPROM

float vbattSlope = 0.0253;                     //calibrated voltage slope

float currentRunTare = 500.0;                  //current run tare raw value
float currentSlope = 0.02639;                  //current slope (ACS714 @5V... A/step)

float forceRunTare = 110.0;                    //force tare raw value
float forceCalTare = 110.0;                    //force calibration tare raw value
float forceCalLoaded = 445.0;                  //force calibration load raw value
float forceSlope = 1.0680;                     //force slope (gm/step)

float torqueRunTare = 500.0;                   //torque run tare raw value
float torqueCalTare = 500.0;                   //torque calibration raw value tare
float torqueCalLoaded = 779.0;                 //torque calibration raw value with load
float torqueSlope = 3.2500;                    //(gm*cm/step)

float vbatt = 0.0;
float current = 0.0;
float force = 0.0;
float torque = 0.0;
float motorEfficiency = 0.0;
float powerIn = 0.0;                           //motor power input
float powerOut = 0.0;                          //motor power output

Q2HX711 torqueADC(torqueDataPin,torqueClockPin);
Q2HX711 forceADC(forceDataPin,forceClockPin);
Adafruit_ADS1115 ads;                          //vbatt and current ADC

//SETUP----------------------------------------
void setup() {
  pinMode(startPin,INPUT_PULLUP);              //Setup  start button with internal pull up
  pinMode(tarePin,INPUT_PULLUP);               //...     tare button...
  pinMode(calPin,INPUT_PULLUP);                //...calibrate button...
  pinMode(ledPin,OUTPUT);                      //Setup LED indicator

  Serial.begin(baud);                          //Start serial
  delay(2000);                                 //Arduino voodoo :P

  EEPROM.get(eepromTorqueAddress,torqueSlope); //Get torque calibration slope from EEPROM
  EEPROM.get(eepromForceAddress,forceSlope);   //Get force calibration slope from EEPROM
  delay(500);

  // Uncomment the line that matches your hardware...
  // Code                             Gain      Input Range         ADS1015  ADS1115
  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads.setGain(GAIN_ONE);           // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  // Enter the correct mV/bit value in the 2 lines below: Default works well with Allegro hall sensors (1-4V)
  vbattSlope = 0.125/vbattDivider/1000.0;        //Actual Volts per bit
  currentSlope = 0.125/currentSensor/1000.0;     //Actual Amps per bit
  ads.begin();

  rpmFactor = 120000.0 / float(tgate) / float(nMagnets);  //Calculate rpm multiplier now
                                               //RPM = 1000*60*2*count/#mags/tgate

  #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    speedControl.attach(escPin);
    speedControl.write(throttleOff);           //turn off ESC
  #endif

  FreqCount.begin(tgate);                      //Start FreqCount

  Serial.println("BL Dyno Mega");              //Print introduction
  Serial.println("by: Kevin Bernas");
  Serial.println();
  Serial.println("Column 1 special codes:");
  Serial.println("99  = Calibration");
  Serial.println("100 = Tare");
  Serial.println();
  Serial.println("Stored Parameters:");        //Print data from EEPROM
  Serial.print("Torque Slope = ");
  Serial.print(torqueSlope,5);
  Serial.println(" [gm*cm/bit]");
  Serial.print("Force Slope = ");
  Serial.print(forceSlope,5);
  Serial.println(" [gm/bit]");
  Serial.print("Vbatt Slope = ");
  Serial.print(vbattSlope,8);
  Serial.println(" [V/bit]");
  Serial.print("Current Slope = ");
  Serial.print(currentSlope,8);
  Serial.println(" [A/bit]");
  Serial.println();
  Serial.flush();

  digitalWrite(ledPin,HIGH);                   //Alive, turn on LED
} //END OF SETUP-------------------------------

//LOOP-----------------------------------------
void loop() {
  //If tare button is pressed (<2sec)/////////////////////////////////////////////////////////////
  if(digitalRead(tarePin) == LOW)  {
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      speedControl.write(throttleOff);         //turn off ESC
    #endif
    digitalWrite(ledPin,LOW);                  //turn off LED
    delay(2000);                               //wait 2 seconds for user to release button
    currentRunTare = 0.0;                      //Reset run tare values
    forceRunTare = 0.0;
    torqueRunTare = 0.0;
    for(int i = 0; i < tareN; i++) {           //Sum tareN values
      currentRunTare += ads.readADC_SingleEnded(1);
      while(!forceADC.readyToSend() || !torqueADC.readyToSend());
      forceRunTare += forceADC.read()/100.0;
      while(!forceADC.readyToSend() || !torqueADC.readyToSend());
      torqueRunTare += torqueADC.read()/100.0;
    }
    currentRunTare = currentRunTare / tareN;   //Calculate run tare values
    forceRunTare = forceRunTare / tareN;
    torqueRunTare = torqueRunTare / tareN;
    //Print tare values
    Serial.println("Tare,Time [ms],Current,Force,Torque");
    Serial.print("100");
    Serial.print(dataSeparator);
    Serial.print(millis());
    Serial.print(dataSeparator);
    Serial.print(currentRunTare,0);
    Serial.print(dataSeparator);
    Serial.print(forceRunTare,0);
    Serial.print(dataSeparator);
    Serial.println(torqueRunTare,0);
    Serial.println();
    //Print data header (with throttle for Mega only)
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      Serial.println("Dataset,Time[ms],Samples,Throttle,RPM,Vbatt,Current,Thrust[gm],Torque[gm*cm],Pin[W],Pout[W],effy");
    #else
      Serial.println("Dataset,Time[ms],Samples,RPM,Vbatt,Current,Thrust[gm],Torque[gm*cm],Pin[W],Pout[W],Meffy");
    #endif

    Serial.flush();
    digitalWrite(ledPin,HIGH);                 //turn on LED
  }
  //end of Tare routine//////////////////////////////////////////////////////////////////////////////

  //4 step calibration routine... (<2sec each press)/////////////////////////////////////////////////
  if(digitalRead(calPin) == LOW)  {
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      speedControl.write(throttleOff);         //turn off ESC
    #endif
    digitalWrite(ledPin,LOW);                  //turn off LED
    delay(2000);                               //wait 2 seconds for user to release button
    while(digitalRead(calPin) == HIGH) {       //1 flash while waiting for step 1
      digitalWrite(ledPin,HIGH);               //***Reboot now if you accidentally pushed cal ;)
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(600);
    }

    //Step 1: Torque Tare
    digitalWrite(ledPin,LOW);                  //turn off LED
    delay(2000);                               //wait 2 seconds for user to release button
    torqueCalTare = 0.0;                       //reset torqueCalTare
    for(int i = 0; i < calN; i++) {            //sum calN torque readings
      while(!torqueADC.readyToSend());
      torqueCalTare += torqueADC.read()/100.0; //divite by 100 to crop the noisy bits
    }
    torqueCalTare = torqueCalTare / calN;      //calculate torque calibration tare 'X1'
    while(digitalRead(calPin) == HIGH) {       //2 flashes while waiting for step 2
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(150);
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(500);
    }

    //Step 2: Torque Loaded
    digitalWrite(ledPin,LOW);                  //turn off LED
    delay(2000);                               //wait 2 seconds for user to release button
    torqueCalLoaded = 0.0;                     //reset torqueCalLoaded
    for(int i = 0; i < calN; i++) {            //sum calN torque readings
      while(!torqueADC.readyToSend());
      torqueCalLoaded += torqueADC.read()/100.0;
    }
    torqueCalLoaded = torqueCalLoaded / calN;  //calculate torque calibration loaded 'X2'
                                               //calculate torque calibration slope "m"
    torqueSlope = torqueCalLoad / (torqueCalLoaded - torqueCalTare);
    while(digitalRead(calPin) == HIGH) {       //3 flashes while waiting step 3
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(150);
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(150);
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(500);
    }

    //Step 3: Force Tare
    digitalWrite(ledPin,LOW);                  //turn off LED
    delay(2000);                               //wait 2 seconds for user to release button
    forceCalTare = 0;
    for(int i = 0; i < calN; i++) {            //sum calN force readings
      while(!forceADC.readyToSend());
      forceCalTare += forceADC.read()/100.0;
    }
    forceCalTare = forceCalTare / calN;        //calculate torque calibration tare 'X1'
    while(digitalRead(calPin) == HIGH) {       //4 flashes while waiting for step 4
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(150);
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(150);
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(150);
      digitalWrite(ledPin,HIGH);
      delay(150);
      digitalWrite(ledPin,LOW);
      delay(500);
    }

    //Step 4: Force Loaded
    digitalWrite(ledPin,LOW);                  //turn off LED
    delay(2000);                               //wait 2 seconds for user to release button
    forceCalLoaded = 0;
    for(int i = 0; i < calN; i++) {            //sum calN force readings
      while(!forceADC.readyToSend());
      forceCalLoaded += forceADC.read()/100.0;
    }
    forceCalLoaded = forceCalLoaded / calN;    //calculate force calibration loaded 'X2'
                                               //calculate force calibration slope "m"
    forceSlope = forceCalLoad / (forceCalLoaded - forceCalTare);
    //Print calibration values
    Serial.println("Cal,t[ms],torqueLoad[gm*cm],torqueTare,torqueLoaded,torqueSlope[gm*cm/bit],forceLoad[gm],forceTare,forceLoaded,forceSlope[gm/bit]");
    Serial.flush();                            //Attempt to fix buggy data output
    Serial.print("99");
    Serial.print(dataSeparator);
    Serial.print(millis());
    Serial.print(dataSeparator);
    Serial.print(torqueCalLoad);
    Serial.print(dataSeparator);
    Serial.print(torqueCalTare);
    Serial.print(dataSeparator);
    Serial.print(torqueCalLoaded);
    Serial.print(dataSeparator);
    Serial.print(torqueSlope,5);
    Serial.print(dataSeparator);
    Serial.print(forceCalLoad);
    Serial.print(dataSeparator);
    Serial.print(forceCalTare);
    Serial.print(dataSeparator);
    Serial.print(forceCalLoaded);
    Serial.print(dataSeparator);
    Serial.println(forceSlope,5);
    Serial.println();
    Serial.flush();

    EEPROM.put(eepromTorqueAddress,torqueSlope);//save torque calibration slope to EEPROM
    EEPROM.put(eepromForceAddress,forceSlope); //save force calibration slope to EEPROM
    delay(500);
    digitalWrite(ledPin,HIGH);                 //turn on LED
  }
  //end of calibration routine////////////////////////////////////////////////////////////////

  //If start button is released & the pause routine trigger is 0////////////////////////////
  if(digitalRead(startPin) == HIGH && releaseWait == 0) {
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      speedControl.write(throttleOff);         //turn off ESC
    #endif
    releaseWait = 1;                           //Do this only once
    dataset ++;                                //Increment dataset #
    samples = 0;                               //Reset sample count
    vbatt = 0;                                 //Reset capture variables...
    current = 0;
    force = 0;
    torque = 0;
    digitalWrite(ledPin,HIGH);                 //turn on LED
  }

  //If start button is pressed//////////////////////////////////////////////////////////////
  if(digitalRead(startPin) == LOW) {
    if(ledPin == HIGH) {
       digitalWrite(ledPin,LOW);               //turn off LED on first loop only
    }
    tstamp = millis();                         //Get time stamp
    samples++;                                 //Increment sample count

    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      if(tstamp - tthrottle >= throttleDelay) { //change throttle command if it is time
        throttle = analogRead(throttlePot);
        throttle = map(throttle, 0, 1023, throttleMin, throttleMax);
        speedControl.writeMicroseconds(throttle);
        tthrottle = millis();
      }
    #endif

    if(releaseWait == 1) {                     //if this is the first time through the loop...
      tsample = tstamp;                        //Reset sampling period (otherwise first row has just 1 measurement)
      releaseWait = 0;                         //Reset trigger for start button release routine
    }

    vbatt += ads.readADC_SingleEnded(0);       //Always add readings to sums
    current += ads.readADC_SingleEnded(1);

    //debug... force and torque occasionally read a spike in this function with two hx711's :(
    /*both sync clocks and separate clocks didn't fix it, nor did moving one of the reads before
    //vbatt and before current. Adding a large bypass cap didn't help either.
    //
    //Note: HX711 datasheet shows 24 bit shift registers, plus 1 to 3 bits for gain, timed with a >1us clock pulse.
    //All HX711 libs use untimed shiftLn() to read data (presumably clock pulses naturally never exceed 1us?).
    //This project runs both HX711's at 80Hz sample rate (13ms per sample max).
    */
    //Original code:
    force += forceADC.read()/100.0;            //div by 100 to chop off the noisy bits
    torque += torqueADC.read()/100.0;
    //
    //Debug code
    /*Serial.print("2,Start,");
    Serial.println(millis());
    float force1 = forceADC.read()/100.0;
    force += force1;
    Serial.print("2,force,");
    Serial.println(force1);
    Serial.print("2,Middle,");
    Serial.println(millis());
    float torque1 = torqueADC.read()/100.0;
    torque += torque1;
    Serial.print("2,torque,");
    Serial.println(torque1);
    Serial.print("2,End,");
    Serial.println(millis());
    end debug.....................
    */

    if (FreqCount.available()) {               //Refresh RPM measurement if available
      rpm = rpmFactor * float(FreqCount.read());
    }

    //After sample period has elapsed...
    if(tstamp - tsample >= samplePeriod) {
      vbatt = vbatt / float(samples);                 //Calculate averages...
      current = current / float(samples);
      force = force / float(samples);
      torque = torque / float(samples);

      vbatt =  vbattSlope * vbatt;             //Calculate calibrated values...
      current = currentSlope * (current - currentRunTare);
      force = forceSlope * (force - forceRunTare);
      torque = torqueSlope * (torque - torqueRunTare);
      powerIn = vbatt * current;
      powerOut = 1.026989 * torque * rpm / 100000.0;// 1.026949/100000 for gm*cm*RPM to Watts
      motorEfficiency = powerOut / powerIn;

      //Serial.print("E1,");                   //Prepend an "E" for "Bluetooth Graphics" Android app (limit 7 "columns"... crashes!!!)
      Serial.print(dataset);                   //Print formatted line of data...
      Serial.print(dataSeparator);
      Serial.print(tstamp);
      Serial.print(dataSeparator);
      Serial.print(samples,0);
      Serial.print(dataSeparator);
      #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        float throttlePercent = 100.0*(float(throttle)-float(throttleMin))/(float(throttleMax)-float(throttleMin));   //Convert servo micros to throttle %
        Serial.print(throttlePercent,0);
        Serial.print(dataSeparator);
      #endif
      Serial.print(rpm,0);
      Serial.print(dataSeparator);
      Serial.print(vbatt,3);
      Serial.print(dataSeparator);
      Serial.print(current,3);
      Serial.print(dataSeparator);
      Serial.print(force,3);
      Serial.print(dataSeparator);
      Serial.print(torque,3);
      Serial.print(dataSeparator);
      Serial.print(powerIn);
      Serial.print(dataSeparator);
      Serial.print(powerOut);
      Serial.print(dataSeparator);
      Serial.println(motorEfficiency,4);
      Serial.flush();
      samples = 0;                             //Reset sample count
      vbatt = 0.0;                               //Reset variables...
      current = 0.0;
      force = 0.0;
      torque = 0.0;
      tsample = millis();                      //Reset sample time
    }
  }
}
