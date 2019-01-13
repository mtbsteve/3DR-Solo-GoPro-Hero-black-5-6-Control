/*
  GoPro Hero black 5/6/7 Interface for 3DR Solo
  Based on all the great work from Konrad Iturbe (KonradIT)
  https://github.com/KonradIT/goprowifihack
  NOT FOR COMMERCIAL USE!
  This API will be updated as I get access to newly released cameras, 
  if you are a company, I highly recommend to enroll in the GoPro developer program.  
  Stephan Schindewolf, January 2018
  
  Firmata is a generic protocol for communicating with microcontrollers
  from software on a host computer. It is intended to work with
  any host computer software package.

  This library is free software; you can redistribute it and/or
  modify it under th
  To download a host software package, please click on the following link
  to open the list of Firmata client libraries in your default browser.

  https://github.com/firmata/arduino#firmata-client-libraries

  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  Copyright (C) 2010-2011 Paul Stoffregen.  All rights res erved.
  Copyright (C) 2009 Shigeru Kobayashi.  All rights reserved.
  Copyright (C) 2009-2016 Jeff Hoefs.  All rights reserved.
e terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  See file LICENSE.txt for further informations on licensing terms.

  Last updated October 16th, 2016
*/

#include <Servo.h>
#include <Wire.h>
#include <Firmata.h>
#include <SPI.h>
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"

#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1


/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[64];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
// default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

boolean isResetting = false;

// Forward declare a few functions to avoid compiler errors with older versions
// of the Arduino IDE.
void setPinModeCallback(byte, int);
void reportAnalogCallback(byte analogPin, int value);
void sysexCallback(byte, byte, byte*);

/* utility functions */
void wireWrite(byte data)
{
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}
// gopro global variables
// your GoPro wifi settings - specify up to 3 GoPros in arduino_secrets.h
String gp_ssid[3] = {SECRET_SSID1, SECRET_SSID2, SECRET_SSID3};
String gp_pass[3] = {SECRET_PASS1, SECRET_PASS2, SECRET_PASS3};
char gpserver[] = "10.5.5.9";
const int WOLPort = 7; // 7 or 9
byte gopro_mac[6];  
String requestURL;
int handshake_toggle = 0; //control the callback to Solo
int gopro_state_sent = 0; //checks if no gopro was connected
unsigned long currentMillis2;        // store the current value from millis()
unsigned long previousMillis2;       // for comparison with currentMillis
unsigned int gp_samplingInterval = 2000; // how often to run the main loop (in ms) to check for the gopro parameters
bool gpFunctionpattern[8];
bool gplastFunctionpattern[8];
bool gpvideoMode = true;
int gp_battery_status = 0;
int gp_recording_status = 0;
int gp_video_format = 0;
int gp_modesubmode = 0;
int gopro_model = 0;

//**************   Setup Wifi connection to Gopro
WiFiClient client;
WiFiUDP Udp;


/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS) {
    // reuse indexes of detached servos until all have been reallocated
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  } else {
    Firmata.sendString("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  // if we're detaching the last servo, decrement the count
  // otherwise store the index of the detached servo
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    // keep track of detached servos because we want to reuse their indexes
    // before incrementing the count of attached servos
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }

  servoPinMap[pin] = 255;
}

void enableI2CPins()
{
  byte i;
  // is there a faster way to do this? would probaby require importing
  // Arduino.h to get SCL and SDA pins
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      // mark pins as i2c so they are ignore in non i2c data requests
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }

  isI2CEnabled = true;

  Wire.begin();
}

/* disable the i2c pins so they can be used for other functions */
void disableI2CPins() {
  isI2CEnabled = false;
  // disable read continuous mode for all devices
  queryIndex = -1;
}

void readAndReportData(byte address, int theRegister, byte numBytes, byte stopTX) {
  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    wireWrite((byte)theRegister);
    Wire.endTransmission(stopTX); // default = true
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;  // fill the register with a dummy value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  // check to be sure correct number of bytes were returned by slave
  if (numBytes < Wire.available()) {
    Firmata.sendString("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C: Too few bytes received");
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    i2cRxData[2 + i] = wireRead();
  }

  // send slave address, register and received bytes
  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
    // disable i2c so pins can be used for other functions
    // the following if statements should reconfigure the pins properly
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO) {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
          // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
          digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);    // disable output driver
#if ARDUINO <= 100
        // deprecated since Arduino 1.0.1 - TODO: drop support in Firmata 2.6
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
#endif
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          // Disable PWM if pin mode was previously set to PWM.
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
    case PIN_MODE_SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        Firmata.setPinMode(pin, PIN_MODE_SERVO);
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          // pass -1 for min and max pulse values to use default values set
          // by Servo library
          attachServo(pin, -1, -1);
        }
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        // mark the pin as i2c
        // the user must call I2C_CONFIG to enable I2C for a device
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

/*
 * Sets the value of an individual pin. Useful if you want to set a pin value but
 * are not tracking the digital port state.
 * Can only be used on pins configured as OUTPUT.
 * Cannot be used to enable pull-ups on Digital INPUT pins.
 */
void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (Firmata.getPinMode(pin)) {
      case PIN_MODE_SERVO:
        if (IS_PIN_DIGITAL(pin))
          servos[servoPinMap[pin]].write(value);
        Firmata.setPinState(pin, value);
        break;
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        Firmata.setPinState(pin, value);
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          if (Firmata.getPinMode(pin) == OUTPUT) {
            pinWriteMask |= mask;
          } else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
            // only handle INPUT here for backwards compatibility
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            // only write to the INPUT pin to enable pullups if Arduino v1.0.0 or earlier
            pinWriteMask |= mask;
#endif
          }
          Firmata.setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      // prevent during system reset or all analog pin values will be reported
      // which may report noise for unconnected analog pins
      if (!isResetting) {
        // Send pin value immediately. This is helpful when connected via
        // ethernet, wi-fi or bluetooth so pin states can be known upon
        // reconnecting.
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    // Send port value immediately. This is helpful when connected via
    // ethernet, wi-fi or bluetooth so pin states can be known upon
    // reconnecting.
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      // need to invert the logic here since 0 will be default for client
      // libraries that have not updated to add support for restart tx
      if (argv[1] & I2C_END_TX_MASK) {
        stopTX = I2C_RESTART_TX;
      }
      else {
        stopTX = I2C_STOP_TX; // default
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
            wireWrite(data);
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data, stopTX);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            // too many queries, just ignore
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            // a slave register is specified
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);  // bytes to read
          }
          else {
            // a slave register is NOT specified
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);  // bytes to read
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          query[queryIndex].stopTX = stopTX;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          // if read continuous mode is enabled for only 1 i2c device, disable
          // read continuous reporting for that device
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            queryIndexToSkip = 0;
            // if read continuous mode is enabled for multiple devices,
            // determine which device to stop reading and remove it's data from
            // the array, shifiting other array data to fill the space
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }

            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
                query[i].stopTX = query[i + 1].stopTX;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));

      if (delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }

      if (!isI2CEnabled) {
        enableI2CPins();
      }

      break;
    case SERVO_CONFIG:
      if (argc > 4) {
        // these vars are here for clarity, they'll optimized away by the compiler
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_DIGITAL(pin)) {
          if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
            detachServo(pin);
          }
          attachServo(pin, minPulse, maxPulse);
          setPinModeCallback(pin, PIN_MODE_SERVO);
        }
      }
      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      } else {
        //Firmata.sendString("Not enough data");
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10); // 10 = 10-bit resolution
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);  // TODO: could assign a number to map to SCL or SDA
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;

    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
  }
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  isResetting = true;

  // initialize a defalt state
  // TODO: option to load config from EEPROM instead of default

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

  if (isI2CEnabled) {
    disableI2CPins();
  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;    // by default, reporting off
    portConfigInputs[i] = 0;  // until activated
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    // pins with analog capability default to analog input
    // otherwise, pins default to digital output
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }

    servoPinMap[i] = 255;
  }
  // by default, do not report any analog inputs
  analogInputsToReport = 0;

  detachedServoCount = 0;
  servoCount = 0;

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  /*
  TODO: this can never execute, since no pins default to digital input
        but it will be needed when/if we support EEPROM stored config
  for (byte i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i, portConfigInputs[i]), true);
  }
  */
  isResetting = false;
}

void setup()
{
  pinMode(0, INPUT); // Pin 0 to 6 used to transfer the GP commands
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT); 
  pinMode(7, INPUT); // Handshake to execute the GoPro command
  pinMode(8, OUTPUT); //bit 1 for sending Gopro Model
  pinMode(9, OUTPUT); //bit 2 for sending Gopro Model
  pinMode(10, OUTPUT); //flush the gopro settings to Solo
  pinMode(11, INPUT); //Handshake for Pymata readiness
  pinMode(12, OUTPUT); //bit for sending Gopro recording status
  pinMode(13, OUTPUT); //bit 1 for sending Gopro battery status
  pinMode(14, OUTPUT); //bit 2 for sending Gopro battery status
  pinMode(A0, OUTPUT); // used as digital to send video mode
  pinMode(A1, OUTPUT); // red: high as long as no GoPro wifi connection established
  pinMode(A3, OUTPUT); // green: high once GoPro is fully connected - both LEDs on means wifi hardware error
  pinMode(A2, OUTPUT); // bit 1 for sending the gopro mode setup
  pinMode(A4, OUTPUT); // bit 2 for sending the gopro mode setup
  pinMode(A5, OUTPUT); // bit 3 for sending the gopro mode setup
  pinMode(A6, OUTPUT); // bit 4 for sending the gopro mode setup

  digitalWrite(A1, LOW); // no wifi yet
  digitalWrite(8, LOW); // no valid gopro connected
  digitalWrite(9, LOW); 
  digitalWrite(10, LOW);
  digitalWrite(12, LOW); 
  digitalWrite(13, LOW);
  digitalWrite(14, LOW); 
  digitalWrite(A0, LOW);
  digitalWrite(A2, LOW); 
  digitalWrite(A3, LOW);
  digitalWrite(A4, LOW); 
  digitalWrite(A5, LOW);
  digitalWrite(A6, LOW);
  
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  // to use a port other than Serial, such as Serial1 on an Arduino Leonardo or Mega,
  // Call begin(baud) on the alternate serial port and pass it to Firmata to begin like this:
  // Serial1.begin(57600);
  // Firmata.begin(Serial1);
  // However do not do this if you are using SERIAL_MESSAGE

  Firmata.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for ATmega32u4-based boards and Arduino 101
  }

  systemResetCallback();  // reset to default config

  // ************** WiFi connection **************************
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    //Serial.println("WiFi shield not present");
    // don't continue:
    digitalWrite(A1, HIGH); // Error no wifi shield - 
    digitalWrite(A3, LOW); // no wifi shield - do not continue
    delay (500);
    digitalWrite(A1, LOW); // Error no wifi shield - 
    digitalWrite(A3, HIGH); // no wifi shield - do not continue
    delay (500);
    while (true);
  }
  // Initialize the GoPro handshake bitmap:
  for(int i=0; i<8; i++){
    gpFunctionpattern[i] = digitalRead(i);
  }
  for(int i=0; i<8; i++){
    gplastFunctionpattern[i] = gpFunctionpattern[i];
  }
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
 // Firmata Code   
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();

  /* STREAMREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while (Firmata.available())
    Firmata.processInput();

  // TODO - ensure that Stream buffer doesn't go over 60 bytes

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    // report i2c data for all device with read continuous mode enabled
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
      }
    }
  }

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.update();
#endif

// *********************** GoPro control *********************************************
  pinMode(8, OUTPUT); //bit 1 for sending Gopro Model
  pinMode(9, OUTPUT); //bit 2 for sending Gopro Model
  pinMode(12, OUTPUT); //bit for sending Gopro recording status
  pinMode(13, OUTPUT); //bit 1 for sending Gopro battery status
  pinMode(14, OUTPUT); //bit 2 for sending Gopro battery status
  pinMode(A0, OUTPUT); // used as digital to send video mode
  pinMode(A1, OUTPUT); // red: high as long as no GoPro wifi connection established
  pinMode(A3, OUTPUT); // green: high once GoPro is fully connected - both LEDs on means wifi hardware error
  pinMode(A2, OUTPUT); // bit 1 for sending the gopro mode setup
  pinMode(A4, OUTPUT); // bit 2 for sending the gopro mode setup
  pinMode(A5, OUTPUT); // bit 3 for sending the gopro mode setup
  pinMode(A6, OUTPUT); // bit 4 for sending the gopro mode setup
  
  // check GoPro WiFi connection only if pymata has acknowledged to be ready to listen to callbacks:
  if ((WiFi.status() != WL_CONNECTED) and (digitalRead(11) == HIGH)) {
    //Serial.println("not connected, need to connect");
    connectToGoPro();
    // if we are connected now to a gopro, lets now determine the model and the status
    if (WiFi.status() == WL_CONNECTED) {
      whichGoproModel();
      GoproStatus();
      send_gopro_model_to_solo(gopro_model, gp_battery_status, gp_recording_status, gp_video_format, gp_modesubmode); 
      gopro_state_sent = 0;
    }
    else if (gopro_state_sent == 0) {
      send_gopro_model_to_solo(gopro_model, gp_battery_status, gp_recording_status, gp_video_format, gp_modesubmode); // no Gopro connected
      gopro_state_sent = 1;
      }
  }
  
  currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 > gp_samplingInterval) {
    previousMillis2 += gp_samplingInterval;
    /* check battery and recording status every 2 seconds if we are connected  */
    if ((WiFi.status() == WL_CONNECTED) and (digitalRead(11) == HIGH)) {
       check_rec_battery_status();
       }
  }
  //Serial.println("in the loop and connected!");
  // check if handshake took place, then read command number
  gpFunctionpattern[7] = digitalRead(7);
  if (gpFunctionpattern[7] == true && (gpFunctionpattern[7] != gplastFunctionpattern[7])) {
     gplastFunctionpattern[7] = gpFunctionpattern[7];
     digitalWrite(A1, HIGH); // visualize read
     int gpCommand = 0;
     int bitValue[] = {1, 2, 4, 8, 16, 32, 64};
     for(int i=0; i<7; i++){
        if (digitalRead(i) == HIGH) {
          gpCommand = gpCommand + bitValue[i];
        }
     }
     // execute the GoPro command
     switch(gpCommand) {
      case 0: //stop video recording
         if(gpvideoMode == true) {
           cameraStop();
           check_rec_battery_status();
         }
         break;
      case 1: //shutter press
         cameraTrigger();
         check_rec_battery_status();
         break;
      case 2: // turn GoPro off
         cameraOff();
         break;
      case 3: //turn GoPro on
         cameraOn(gopro_mac, 6);
         break;
      case 4: //
         cameraVideoMode();
         gpvideoMode = true;
         break;
      case 5:
         cameraPhotoMode();
         gpvideoMode = false;
         break;
      case 6:
         cameraMultishotMode();
         gpvideoMode = true;
         break;
      case 7:
         cameraEVvideop2();
         break;
      case 8:
         cameraEVvideop15();
         break;
      case 9:
         cameraEVvideop1();
         break;
      case 10:
         cameraEVvideop05();
         break;
      case 11:
         cameraEVvideop0();
         break;
      case 12:
         cameraEVvideon05();
         break;
      case 13:
         cameraEVvideon1();
         break;
      case 14:
         cameraEVvideon15();
         break;
      case 15:
         cameraEVvideon2();
         break;
      case 16:
         cameraEVphotop2();
         break;
      case 17:
         cameraEVphotop15();
         break;
      case 18:
         cameraEVphotop1();
         break;
      case 19:
         cameraEVphotop05();
         break;
      case 20:
         cameraEVphotop0();
         break;
      case 21:
         cameraEVphoton05();
         break;
      case 22:
         cameraEVphoton1();
         break;
      case 23:
         cameraEVphoton15();
         break;
      case 24:
         cameraEVphoton2();
         break;
      case 25:
         cameraEVMultp2();
         break;
      case 26:
         cameraEVMultp15();
         break;
      case 27:
         cameraEVMultp1();
         break;
      case 28:
         cameraEVMultp05();
         break;
      case 29:
         cameraEVMultp0();
         break;
      case 30:
         cameraEVMultn05();
         break;
      case 31:
         cameraEVMultn1();
         break;
      case 32:
         cameraEVMultn15();
         break;
      case 33:
         cameraEVMultn2();
         break;
      case 34:
         cameraVideoRes4K();
         break;
      case 35:
         cameraVideoRes4Ks();
         break;
      case 36:
         cameraVideoRes27K();
         break;
      case 37:
         cameraVideoRes27Ks();
         break;
      case 38:
         cameraVideoRes27K43();
         break;
      case 39:
         cameraVideoRes1440p();
         break;
      case 40:
         cameraVideoRes1080ps();
         break;
      case 41:
         cameraVideoRes1080p();
         break;
      case 42:
         cameraVideoRes960p();
         break;
      case 43:
         cameraVideoRes720ps();
         break;
      case 44:
         cameraVideoRes720p();
         break;
      case 45:
         cameraVideoResWVGA();
         break;
      case 46:
         cameraVideo240fps();
         break;
      case 47:
         cameraVideo120fps();
         break;
      case 48:
         cameraVideo100fps();
         break;
      case 49:
         cameraVideo90fps();
         break;
      case 50:
         cameraVideo80fps();
         break;
      case 51:
         cameraVideo60fps();
         break;
      case 52:
         cameraVideo50fps();
         break;
      case 53:
         cameraVideo48fps();
         break;
      case 54:
         cameraVideo30fps();
         break;
      case 55:
         cameraVideo25fps();
         break;
      case 56:
         cameraVideo24fps();
         break;
      case 57:
         cameraFOVwide();
         break;
      case 58:
         cameraFOVmedium();
         break;
      case 59:
         cameraFOVnarrow();
         break;
      case 60:
         cameraFOVsuperview();
         break;
      case 61:
         cameraFOVlinear();
         break;
      case 62:
         cameraLowLighton();
         break;
      case 63:
         cameraLowLightoff();
         break;
      case 64:
         cameraPhotoRes12MP();
         break;
      case 65:
         cameraPhotoRes7MPwide();
         break;
      case 66:
         cameraPhotoRes7MPmedium();
         break;
      case 67:
         cameraPhotoRes5MP();
         break;
      case 68:
         cameraMultRes12MP();
         break;
      case 69:
         cameraMultRes7MPwide();
         break;
      case 70:
         cameraMultRes7MPmedium();
         break;
      case 71:
         cameraMultRes5MP();
         break;
      case 72:
         cameraBurst31();
         break;
      case 73:
         cameraBurst51();
         break;
      case 74:
         cameraBurst101();
         break;
      case 75:
         cameraBurst102();
         break;
      case 76:
         cameraBurst103();
         break;
      case 77:
         cameraBurst301();
         break;
      case 78:
         cameraBurst302();
         break;
      case 79:
         cameraBurst303();
         break;
      case 80:
         cameraBurst306();
         break;
      case 81:
         cameraNTSC();
         break;
      case 82:
         cameraPAL();
         break;
      case 83:
         cameraPTvideoOn();
         break;
      case 84:
         cameraPTvideoOff();
         break;
      case 85:
         cameraPTphotoOn();
         break;
      case 86:
         cameraPTphotoOff();
         break;
      case 87:
         cameraPTburstOn();
         break;
      case 88:
         cameraPTburstOff();
         break;
      case 89:
         cameraSecVideo();
         break;
      case 90:
         cameraSecTimLapseVideo();
         break;
      case 91:
         cameraVidPhoto();
         break;
      case 92:
         cameraLoopVideo();
         break;
      case 93:
         cameraSinglePhoto();
         break;
      case 94:
         cameraContPhoto();
         break;
      case 95:
         cameraNightPhoto();
         break;
      case 96:
         cameraBurstMultshot();
         break;
      case 97:
         cameraTimelapseMultShot();
         break;
      case 98:
         cameraNightlapseMultShot();
         break;
      case 99:
         cameraEISon();
         break;
      case 100:
         cameraEISoff();
         break;
      case 101:
         cameraPhotoRes12MPlin();
         break;
      case 102:
         cameraPhotoRes12MPmed();
         break;
      case 103:
         cameraPhotoRes12MPnarrow();
         break;
      case 104:
         cameraPhotoSuperphotooff();
         break;
      case 105:
         cameraPhotoSuperphotoauto();
         break;
      case 106:
         cameraPhotoSuperphotoHDR();
         break;
      case 107:
         cameraMultRes12MPlin();
         break;
      case 108:
         cameraMultRes12MPmed();
         break;
      case 109:
         cameraMultRes12MPnarrow();
         break;
      case 110:
         cameraTimeWarp();
         break;
      case 111:
         cameraTimeWarpSpeed2();
         break;
      case 112:
         cameraTimeWarpSpeed5();
         break;
      case 113:
         cameraTimeWarpSpeed10();
         break;
      case 114:
         cameraTimeWarpSpeed15();
         break;
      case 115:
         cameraTimeWarpSpeed30();
         break;
    }  
  }
  else {
     if (gpFunctionpattern[7] == false && (gpFunctionpattern[7] != gplastFunctionpattern[7])) {
        gplastFunctionpattern[7] = gpFunctionpattern[7];
        digitalWrite(A1, LOW); // turn off
        } 

     }
}


// *************** GoPro Control and setup functions **********************************************************
void cameraTrigger(void){
   requestURL = String ("GET /gp/gpControl/command/shutter?p=1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraStop(void){
   requestURL = String ("GET /gp/gpControl/command/shutter?p=0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraOff(void){
   requestURL = String ("GET /gp/gpControl/command/system/sleep HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoMode(void){
   requestURL = String ("GET /gp/gpControl/command/mode?p=0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoMode(void){
   requestURL = String ("GET /gp/gpControl/command/mode?p=1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultishotMode(void){
   requestURL = String ("GET /gp/gpControl/command/mode?p=2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideop2(void){
   requestURL = String ("GET /gp/gpControl/setting/15/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideop15(void){
   requestURL = String ("GET /gp/gpControl/setting/15/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideop1(void){
   requestURL = String ("GET /gp/gpControl/setting/15/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideop05(void){
   requestURL = String ("GET /gp/gpControl/setting/15/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideop0(void){
   requestURL = String ("GET /gp/gpControl/setting/15/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideon05(void){
   requestURL = String ("GET /gp/gpControl/setting/15/5 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideon1(void){
   requestURL = String ("GET /gp/gpControl/setting/15/6 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideon15(void){
   requestURL = String ("GET /gp/gpControl/setting/15/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVvideon2(void){
   requestURL = String ("GET /gp/gpControl/setting/15/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphotop2(void){
   requestURL = String ("GET /gp/gpControl/setting/26/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphotop15(void){
   requestURL = String ("GET /gp/gpControl/setting/26/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphotop1(void){
   requestURL = String ("GET /gp/gpControl/setting/26/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphotop05(void){
   requestURL = String ("GET /gp/gpControl/setting/26/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphotop0(void){
   requestURL = String ("GET /gp/gpControl/setting/26/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphoton05(void){
   requestURL = String ("GET /gp/gpControl/setting/26/5 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphoton1(void){
   requestURL = String ("GET /gp/gpControl/setting/26/6 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphoton15(void){
   requestURL = String ("GET /gp/gpControl/setting/26/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVphoton2(void){
   requestURL = String ("GET /gp/gpControl/setting/26/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultp2(void){
   requestURL = String ("GET /gp/gpControl/setting/39/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultp15(void){
   requestURL = String ("GET /gp/gpControl/setting/39/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultp1(void){
   requestURL = String ("GET /gp/gpControl/setting/39/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultp05(void){
   requestURL = String ("GET /gp/gpControl/setting/39/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultp0(void){
   requestURL = String ("GET /gp/gpControl/setting/39/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultn05(void){
   requestURL = String ("GET /gp/gpControl/setting/39/5 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultn1(void){
   requestURL = String ("GET /gp/gpControl/setting/39/6 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultn15(void){
   requestURL = String ("GET /gp/gpControl/setting/39/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEVMultn2(void){
   requestURL = String ("GET /gp/gpControl/setting/39/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes4K(void){
   requestURL = String ("GET /gp/gpControl/setting/2/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes4Ks(void){
   requestURL = String ("GET /gp/gpControl/setting/2/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes27K(void){
   requestURL = String ("GET /gp/gpControl/setting/2/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes27Ks(void){
   requestURL = String ("GET /gp/gpControl/setting/2/5 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes27K43(void){
   requestURL = String ("GET /gp/gpControl/setting/2/6 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes1440p(void){
   requestURL = String ("GET /gp/gpControl/setting/2/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes1080ps(void){
   requestURL = String ("GET /gp/gpControl/setting/2/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes1080p(void){
   requestURL = String ("GET /gp/gpControl/setting/2/9 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes960p(void){
   requestURL = String ("GET /gp/gpControl/setting/2/10 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes720ps(void){
   requestURL = String ("GET /gp/gpControl/setting/2/11 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoRes720p(void){
   requestURL = String ("GET /gp/gpControl/setting/2/12 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideoResWVGA(void){
   requestURL = String ("GET /gp/gpControl/setting/2/13 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo240fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo120fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo100fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo90fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo80fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo60fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/5 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo50fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/6 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo48fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo30fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo25fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/9 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVideo24fps(void){
   requestURL = String ("GET /gp/gpControl/setting/3/10 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraFOVwide(void){
   requestURL = String ("GET /gp/gpControl/setting/4/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraFOVmedium(void){
   requestURL = String ("GET /gp/gpControl/setting/4/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraFOVnarrow(void){
   requestURL = String ("GET /gp/gpControl/setting/4/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraFOVsuperview(void){
   requestURL = String ("GET /gp/gpControl/setting/4/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraFOVlinear(void){
   requestURL = String ("GET /gp/gpControl/setting/4/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraLowLighton(void){
   requestURL = String ("GET /gp/gpControl/setting/8/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraLowLightoff(void){
   requestURL = String ("GET /gp/gpControl/setting/8/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEISon(void){
   requestURL = String ("GET /gp/gpControl/setting/78/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraEISoff(void){
   requestURL = String ("GET /gp/gpControl/setting/78/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes12MP(void){
   requestURL = String ("GET /gp/gpControl/setting/17/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes7MPwide(void){
   requestURL = String ("GET /gp/gpControl/setting/17/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes7MPmedium(void){
   requestURL = String ("GET /gp/gpControl/setting/17/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes5MP(void){
   requestURL = String ("GET /gp/gpControl/setting/17/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes12MPlin(void){
   requestURL = String ("GET /gp/gpControl/setting/17/10 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes12MPmed(void){
   requestURL = String ("GET /gp/gpControl/setting/17/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoRes12MPnarrow(void){
   requestURL = String ("GET /gp/gpControl/setting/17/9 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoSuperphotooff(void){
   requestURL = String ("GET /gp/gpControl/setting/109/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoSuperphotoauto(void){
   requestURL = String ("GET /gp/gpControl/setting/109/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPhotoSuperphotoHDR(void){
   requestURL = String ("GET /gp/gpControl/setting/109/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes12MP(void){
   requestURL = String ("GET /gp/gpControl/setting/28/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes7MPwide(void){
   requestURL = String ("GET /gp/gpControl/setting/28/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes7MPmedium(void){
   requestURL = String ("GET /gp/gpControl/setting/28/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes5MP(void){
   requestURL = String ("GET /gp/gpControl/setting/28/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes12MPlin(void){
   requestURL = String ("GET /gp/gpControl/setting/28/10 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes12MPmed(void){
   requestURL = String ("GET /gp/gpControl/setting/28/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraMultRes12MPnarrow(void){
   requestURL = String ("GET /gp/gpControl/setting/28/9 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst31(void){
   requestURL = String ("GET /gp/gpControl/setting/29/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst51(void){
   requestURL = String ("GET /gp/gpControl/setting/29/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst101(void){
   requestURL = String ("GET /gp/gpControl/setting/29/2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst102(void){
   requestURL = String ("GET /gp/gpControl/setting/29/3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst103(void){
   requestURL = String ("GET /gp/gpControl/setting/29/4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst301(void){
   requestURL = String ("GET /gp/gpControl/setting/29/5 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst302(void){
   requestURL = String ("GET /gp/gpControl/setting/29/6 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst303(void){
   requestURL = String ("GET /gp/gpControl/setting/29/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurst306(void){
   requestURL = String ("GET /gp/gpControl/setting/29/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraNTSC(void){
   requestURL = String ("GET /gp/gpControl/setting/57/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPAL(void){
   requestURL = String ("GET /gp/gpControl/setting/57/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPTvideoOn(void){
   requestURL = String ("GET /gp/gpControl/setting/10/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPTvideoOff(void){
   requestURL = String ("GET /gp/gpControl/setting/10/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPTphotoOn(void){
   requestURL = String ("GET /gp/gpControl/setting/21/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPTphotoOff(void){
   requestURL = String ("GET /gp/gpControl/setting/21/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPTburstOn(void){
   requestURL = String ("GET /gp/gpControl/setting/34/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraPTburstOff(void){
   requestURL = String ("GET /gp/gpControl/setting/34/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraSecVideo(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=0&sub_mode=0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraSecTimLapseVideo(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=0&sub_mode=1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraVidPhoto(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=0&sub_mode=2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraLoopVideo(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=0&sub_mode=3 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraSinglePhoto(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=1&sub_mode=0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraContPhoto(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=1&sub_mode=1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraNightPhoto(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=1&sub_mode=2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraBurstMultshot(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=2&sub_mode=0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimelapseMultShot(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=2&sub_mode=1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraNightlapseMultShot(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=2&sub_mode=2 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimeWarp(void){
   requestURL = String ("GET /gp/gpControl/command/sub_mode?mode=0&sub_mode=4 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimeWarpSpeed2(void){
   requestURL = String ("GET /gp/gpControl/setting/111/7 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimeWarpSpeed5(void){
   requestURL = String ("GET /gp/gpControl/setting/111/8 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimeWarpSpeed10(void){
   requestURL = String ("GET /gp/gpControl/setting/111/9 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimeWarpSpeed15(void){
   requestURL = String ("GET /gp/gpControl/setting/111/0 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}
void cameraTimeWarpSpeed30(void){
   requestURL = String ("GET /gp/gpControl/setting/111/1 HTTP/1.1\r\n") + "Host: " + gpserver + "\r\n";
   sendRequest(requestURL);
}

void sendRequest(String thisRequest){
  client.stop();
  if (client.connect(gpserver, 80) and (WiFi.status() == WL_CONNECTED)) {
    //Serial.println("connected to server");
    // Make a HTTP request:
    client.println(thisRequest);
    client.println("Connection: close");
    client.println();
    //Serial.println("sent to server: " + thisRequest);
    }
  else if (WiFi.status() == WL_CONNECTED) {
    // if you couldn't make a connection:
    digitalWrite(A1, HIGH); // no connection yet
    digitalWrite(A3, LOW);  
    // you're connected now to the GoPro network, now lets check if the GoPro is turned on:
    // wakeup GoPro if needed
    
    bool gpsleep = false;
    while (!client.connect(gpserver, 80)) {
      // we are connected, but Gopro seems to sleep
      if (gpsleep == false) {
         cameraOn(gopro_mac, 6); // try to wake it up
         gpsleep = true;
         }
      client.stop();
      }
    delay(1000);
    digitalWrite(A1, LOW); 
    digitalWrite(A3, HIGH); // all good  
  }
}

void cameraOn(byte mac[], int n) {
    // WOL the GoPro
    // build the magic packet
    byte magicPacket[102];
    int i,c1,j=0;
    
    for (i=0; i<6; i++,j++) {
      magicPacket[j] = 0xFF;
    }
    
    for (i=0; i<16; i++) {
      for (c1=0; c1<6; c1++,j++)
        magicPacket[j] = mac[c1];
        }
    // send the magic packet
    Udp.begin(WOLPort);
    Udp.beginPacket(gpserver, WOLPort);
    Udp.write(magicPacket, sizeof magicPacket);
    Udp.endPacket();
    Udp.stop();
    delay(1000);
    //Serial.print("wakeup sent!");
}

void connectToGoPro() {
    WiFi.end();
    digitalWrite(A1, HIGH); // wifi is on, but 
    digitalWrite(A3, LOW); // not connected to GoPro
    // reset all Gopro status messages
    gopro_model = 0;
    gp_battery_status = 0;
    gp_recording_status = 0;
    gp_video_format = 0;
    gp_modesubmode = 0;

    // scan for networks and look for one of the specified SSIDs of your GoPros
    int numSsid = WiFi.scanNetworks();
    if (numSsid == -1)
      {
      //Serial.println("no specified Gopro in range");
      //while (true);
      return;
      }
    // check each found SSDI against defined list of GoPros
    bool exit_check = false;
    for (int thisNet = 0; thisNet < numSsid; thisNet++) {
      for (int gp_index = 0; gp_index < 3; gp_index++) {
        if (String(WiFi.SSID(thisNet)) == gp_ssid[gp_index]) {
           //Serial.println("Gopro found with SSID:");
           //Serial.println(WiFi.SSID(thisNet));
           // Connect to WPA/WPA2 GoPro network:
           WiFi.begin(gp_ssid[gp_index], gp_pass[gp_index]);
           // wait 6 seconds for connection:
           //delay(1000);
           WiFi.BSSID(gopro_mac); // get the MAC address of the connected GoPro
           // change order of the MAC array
           int start = 0;
           int last = 5;
           while (start < last) {
              int temp = gopro_mac[start];
              gopro_mac[start] = gopro_mac[last];
              gopro_mac[last] = temp;
              start++;
              last--;
              }
           //Serial.println("Connected to GoPro");
           digitalWrite(A1, HIGH); // connected to gopro, but have not yet identified the model yet
           digitalWrite(A3, HIGH); 
           exit_check = true;
           }
        if (exit_check == true) {
           break;
           }
        }
      if (exit_check == true) {
           break;
         }
    }
}

void whichGoproModel() {
  client.stop();
  bool gpsleep = false;
  while (!client.connect(gpserver, 80)) {
    // we are connected, but Gopro seems to sleep
    if (gpsleep == false) {
       cameraOn(gopro_mac, 6); // try to wake it up
       gpsleep = true;
       }
    client.stop();
    }
  delay(1000);
  //Serial.println("connected to server to find out gopro model");
  // Make a HTTP request:
  client.println("GET /gp/gpControl/info HTTP/1.1");
  client.println ("Host: " + String(gpserver));
  client.println("Connection: close");
  client.println();
    
  if (client.println() == 0) {
    //Serial.println("communication failed with client.println = 0");
    return; //communication failed;
    }
  // Check HTTP status
  char status[32] = {0};
  client.readBytesUntil('\r', status, sizeof(status));
  if (strcmp(status, "HTTP/1.1 200 OK") != 0) {
    return; //invalid response
  }
  // Skip HTTP headers
  char endOfHeaders[] = "json";
  client.find(endOfHeaders);
  char c = client.read();
  c = client.read();
    
  // Allocate JsonBuffer
  const size_t bufferSize = JSON_OBJECT_SIZE(1)
                          + JSON_OBJECT_SIZE(9)
                          + 230;
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root = jsonBuffer.parseObject(client);

  JsonObject& info = root["info"];
  int info_model_number = info["model_number"]; // eg 19
  String info_model_name = info["model_name"]; // eg "HERO5 Black"
  String info_firmware_version = info["firmware_version"]; // eg "HD5.02.02.60.00"
  String info_serial_number = info["serial_number"]; // eg "C3161124720272"
  String info_board_type = info["board_type"]; // eg "0x05"
  String info_ap_mac = info["ap_mac"]; // eg "f6dd9edb0a64"
  String info_ap_ssid = info["ap_ssid"]; // eg "GP24720272"
  String info_ap_has_default_credentials = info["ap_has_default_credentials"]; // eg "0"
  String info_capabilities = info["capabilities"]; // eg "16"
   
  if (info_model_name == ("HERO5 Black") or info_model_name == ("HERO5 Session")) {
    gopro_model = 1; // Hero 5 
   }
  else if (info_model_name == ("HERO6 Black")) {
    gopro_model = 2; // Hero 6 
   }
  else if (info_model_name == ("HERO7 Black")) {
    gopro_model = 3; // Hero 7   
   }
  else {
      gopro_model = 0;
   }   
  if (gopro_model != 0) {
    digitalWrite(A1, LOW); // connected to a valid gopro
    digitalWrite(A3, HIGH); 
  }  
digitalWrite(10, LOW); // flush the data and reset callback
}


void GoproStatus() {
  client.stop();
  if (client.connect(gpserver, 80)) {
    // now lets inquire the status and settings of the connected Gopro
    // Make a HTTP request:
    client.println("GET /gp/gpControl/status HTTP/1.1");
    client.println ("Host: " + String(gpserver));
    client.println("Connection: close");
    client.println();
    
    if (client.println() == 0) {
      //Serial.println("communication failed with client.println = 0");
      return; //communication failed;
      }
    // Check HTTP status
    char httpstate[32] = {0};
    client.readBytesUntil('\r', httpstate, sizeof(httpstate));
    if (strcmp(httpstate, "HTTP/1.1 200 OK") != 0) {
    return; //invalid response
      }
    // Skip HTTP headers
    char endOfHeaders[] = "json";
    client.find(endOfHeaders);
    char c = client.read();  
    c = client.read();
   
    // Allocate JsonBuffer
    const size_t bufferSize = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(64) + JSON_OBJECT_SIZE(73) + 840;
    DynamicJsonBuffer jsonBuffer(bufferSize);
    JsonObject& root = jsonBuffer.parseObject(client);
    // lets assign all Gopro paramters for future use
    JsonObject& status = root["status"];
    int status_1 = status["1"]; // 1
    int status_2 = status["2"]; // 3
    int status_3 = status["3"]; // 0
    int status_4 = status["4"]; // 0
    int status_6 = status["6"]; // 0
    int status_8 = status["8"]; // 0
    int status_9 = status["9"]; // 0
    int status_10 = status["10"]; // 0
    int status_11 = status["11"]; // 0
    int status_13 = status["13"]; // 0
    int status_14 = status["14"]; // 0
    int status_15 = status["15"]; // 0
    int status_16 = status["16"]; // 0
    int status_17 = status["17"]; // 1
    int status_19 = status["19"]; // 0
    int status_20 = status["20"]; // 0
    int status_21 = status["21"]; // 0
    int status_22 = status["22"]; // 0
    int status_23 = status["23"]; // 0
    int status_24 = status["24"]; // 0
    int status_26 = status["26"]; // 0
    int status_27 = status["27"]; // 0
    int status_28 = status["28"]; // 86
    const char* status_29 = status["29"]; // ""
    const char* status_30 = status["30"]; // "GP24784461"
    int status_31 = status["31"]; // 0
    int status_32 = status["32"]; // 0
    int status_33 = status["33"]; // 0
    int status_34 = status["34"]; // 4155
    int status_35 = status["35"]; // 6542
    int status_36 = status["36"]; // 3
    int status_37 = status["37"]; // 4
    int status_38 = status["38"]; // 3
    int status_39 = status["39"]; // 4
    const char* status_40 = status["40"]; // "%11%02%12%12%38%3A"
    int status_41 = status["41"]; // 0
    int status_42 = status["42"]; // 0
    int status_43 = status["43"]; // 0
    int status_44 = status["44"]; // 0
    int status_45 = status["45"]; // 0
    int status_46 = status["46"]; // 1
    int status_47 = status["47"]; // 1
    int status_48 = status["48"]; // 1
    int status_49 = status["49"]; // 0
    long status_54 = status["54"]; // 27655117
    int status_55 = status["55"]; // 1
    int status_56 = status["56"]; // 4
    long status_57 = status["57"]; // 146925
    int status_58 = status["58"]; // 0
    int status_59 = status["59"]; // 0
    int status_60 = status["60"]; // 500
    int status_61 = status["61"]; // 2
    int status_62 = status["62"]; // 0
    int status_63 = status["63"]; // 0
    int status_64 = status["64"]; // 0
    int status_65 = status["65"]; // 0
    int status_66 = status["66"]; // 0
    int status_67 = status["67"]; // 0
    int status_68 = status["68"]; // 0
    int status_69 = status["69"]; // 1
    int status_70 = status["70"]; // 84
    int status_71 = status["71"]; // 12
    int status_72 = status["72"]; // 16
    int status_73 = status["73"]; // 13
    int status_74 = status["74"]; // 0
    JsonObject& settings = root["settings"];
    int settings_1 = settings["1"]; // 0
    int settings_2 = settings["2"]; // 9
    int settings_3 = settings["3"]; // 8
    int settings_4 = settings["4"]; // 4
    int settings_5 = settings["5"]; // 0
    int settings_6 = settings["6"]; // 1
    int settings_7 = settings["7"]; // 1
    int settings_8 = settings["8"]; // 1
    int settings_9 = settings["9"]; // 0
    int settings_10 = settings["10"]; // 0
    int settings_11 = settings["11"]; // 0
    int settings_12 = settings["12"]; // 0
    int settings_13 = settings["13"]; // 1
    int settings_14 = settings["14"]; // 0
    int settings_15 = settings["15"]; // 4
    int settings_16 = settings["16"]; // 0
    int settings_17 = settings["17"]; // 10
    int settings_18 = settings["18"]; // 4
    int settings_19 = settings["19"]; // 0
    int settings_20 = settings["20"]; // 0
    int settings_21 = settings["21"]; // 1
    int settings_22 = settings["22"]; // 0
    int settings_23 = settings["23"]; // 0
    int settings_24 = settings["24"]; // 4
    int settings_25 = settings["25"]; // 0
    int settings_26 = settings["26"]; // 4
    int settings_27 = settings["27"]; // 0
    int settings_28 = settings["28"]; // 10
    int settings_29 = settings["29"]; // 5
    int settings_30 = settings["30"]; // 0
    int settings_31 = settings["31"]; // 0
    int settings_32 = settings["32"]; // 3601
    int settings_33 = settings["33"]; // 0
    int settings_34 = settings["34"]; // 0
    int settings_35 = settings["35"]; // 0
    int settings_36 = settings["36"]; // 0
    int settings_37 = settings["37"]; // 4
    int settings_38 = settings["38"]; // 0
    int settings_39 = settings["39"]; // 4
    int settings_50 = settings["50"]; // 0
    int settings_51 = settings["51"]; // 1
    int settings_52 = settings["52"]; // 0
    int settings_54 = settings["54"]; // 1
    int settings_57 = settings["57"]; // 0
    int settings_58 = settings["58"]; // 1
    int settings_59 = settings["59"]; // 6
    int settings_60 = settings["60"]; // 8
    int settings_61 = settings["61"]; // 1
    long settings_62 = settings["62"]; // 2500000
    int settings_63 = settings["63"]; // 7
    int settings_64 = settings["64"]; // 4
    int settings_68 = settings["68"]; // 0
    int settings_69 = settings["69"]; // 1
    int settings_70 = settings["70"]; // 0
    int settings_72 = settings["72"]; // 1
    int settings_73 = settings["73"]; // 0
    int settings_74 = settings["74"]; // 0
    int settings_75 = settings["75"]; // 3
    int settings_76 = settings["76"]; // 3
    int settings_77 = settings["77"]; // 0
    int settings_78 = settings["78"]; // 0
    int settings_79 = settings["79"]; // 0
    int settings_80 = settings["80"]; // 2
    int settings_81 = settings["81"]; // 3
    int settings_82 = settings["82"]; // 0
    int settings_83 = settings["83"]; // 1
    int settings_84 = settings["84"]; // 0
    int settings_85 = settings["85"]; // 0
    int settings_86 = settings["86"]; // 1
    int settings_87 = settings["87"]; // 40
    int settings_88 = settings["88"]; // 100
    int settings_89 = settings["89"]; // 12
    int settings_91 = settings["91"]; // 2
    int settings_92 = settings["92"]; // 12
    int settings_93 = settings["93"]; // 0
    int settings_94 = settings["94"]; // 0
    int settings_95 = settings["95"]; // 1
    int settings_96 = settings["96"]; // 0
    int settings_97 = settings["97"]; // 0
    int settings_98 = settings["98"]; // 0
    int settings_99 = settings["99"]; // 0

    // check battery status
    if (status_1 > 0) {
      gp_battery_status = status_2;
    }
    // check id recording is on
    gp_recording_status = status_8;
    // check if gopro is set to NTSC or PAL
    gp_video_format = settings_57;
    // check for set video or photo mode
    if (status_43 = 0) {  //video mode set
       switch(status_44) {
        case 0: // video mode
           gp_modesubmode = 0;
           break;
        case 1: //timelapse
           gp_modesubmode = 4;
           break;
        case 2: // video + photo mode (Hero 5 only)
           gp_modesubmode = 9;
           break;
        case 3: //video loop
           gp_modesubmode = 1;
           break;
        case 4: //video time warp Hero 7 only
           gp_modesubmode = 3;
           break;
           }
        }
    else if (status_43 = 1) {
       switch(status_44) {
        case 1: // photo mode
           gp_modesubmode = 6;
           break;
        case 2: // night photo
           gp_modesubmode = 8;
           break;
           }
       }
    else if (status_43 = 2) {
       switch(status_44) {
        case 0: // photo burst
           gp_modesubmode = 7;
           break;
        case 1: // timelapse photo
           gp_modesubmode = 5;
           break;
        case 2: // night timelapse
           gp_modesubmode = 2;
           break;
           }
       }
   }
}
    
void send_gopro_model_to_solo(int gopro_model, int gp_battery_status, int gp_recording_status, int gp_video_format, int gp_modesubmode) {
   // write Hero model back
   digitalWrite (8, HIGH && (gopro_model & B00000010)); 
   digitalWrite (9, HIGH && (gopro_model & B00000001)); 
   digitalWrite (12, HIGH && (gp_recording_status & B00000001)); 
   digitalWrite (13, HIGH && (gp_battery_status & B00000010)); 
   digitalWrite (14, HIGH && (gp_battery_status & B00000001)); 
   digitalWrite (A0, HIGH && (gp_video_format & B00000001));   
   digitalWrite (A2, HIGH && (gp_modesubmode & B00001000)); 
   digitalWrite (A4, HIGH && (gp_modesubmode & B00000100)); 
   digitalWrite (A5, HIGH && (gp_modesubmode & B00000010)); 
   digitalWrite (A6, HIGH && (gp_modesubmode & B00000001)); 
   digitalWrite(10, HIGH); // flush the data and initiate callback
}


void check_rec_battery_status() {
  client.stop();
  bool gp_flush = false;
  if (client.connect(gpserver, 80)) {
    // now lets inquire the status and settings of the connected Gopro
    // Make a HTTP request:
    client.println("GET /gp/gpControl/status HTTP/1.1");
    client.println ("Host: " + String(gpserver));
    client.println("Connection: close");
    client.println();
    
    if (client.println() == 0) {
      //Serial.println("communication failed with client.println = 0");
      return; //communication failed;
      }
    // Check HTTP status
    char httpstate[32] = {0};
    client.readBytesUntil('\r', httpstate, sizeof(httpstate));
    if (strcmp(httpstate, "HTTP/1.1 200 OK") != 0) {
    return; //invalid response
      }
    // Skip HTTP headers
    char endOfHeaders[] = "json";
    client.find(endOfHeaders);
    char c = client.read();  
    c = client.read();
   
    // Allocate JsonBuffer
    const size_t bufferSize = JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(64) + JSON_OBJECT_SIZE(73) + 840;
    DynamicJsonBuffer jsonBuffer(bufferSize);
    JsonObject& root = jsonBuffer.parseObject(client);
    // lets assign the Gopro parameters needed for this check
    JsonObject& status = root["status"];
    int status_1 = status["1"]; // 1
    int status_2 = status["2"]; // 3
    int status_8 = status["8"]; // 0
    
    // check battery status
    if (status_1 > 0) {
      if (gp_battery_status != status_2) {
        gp_battery_status = status_2;
        digitalWrite (13, HIGH && (gp_battery_status & B00000010)); 
        digitalWrite (14, HIGH && (gp_battery_status & B00000001)); 
        gp_flush = true;
        }
      }
    // check id recording is on
    if (gp_recording_status != status_8) {
        gp_recording_status = status_8;
        digitalWrite (12, HIGH && (gp_recording_status & B00000001)); 
        gp_flush = true;
    }  
    // write back to Solo if there is a change
    if (gp_flush == true) {
        digitalWrite(10, HIGH); // flush the data and initiate callback
        }
    }
}
    
