/** Program Name:    HarleyGPS_v.6
 * Written by:      Jesse Ragsdale
 * Date:            2015/11/03
 *
 * Credits:         Adafruit, LadyAda, Becky Stern, and Tyler Cooper
 *
 * Goal:            Record and display Average Speed, Maximum Speed, and distance
 *                  traveled in mph. Transmit info to iPhone via Bluefruit
 */

#include <Adafruit_GPS.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"
#define GPSECHO                     true

SoftwareSerial mySerial(9, 10);
Adafruit_GPS GPS(&mySerial);
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

boolean usingInterrupt = false;
void useInterrupt(boolean);

float maxSpeed = 0;
double latA = 0;
double longA = 0;
double latB = 0;
double longB = 0;
double deltaLat = 0;
double deltaLong = 0;
double change = 0;
double deltaDistance = 0;
double distance = 0;

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  while (!Serial);
  Serial.begin(115200);
  GPS.begin(9600);

  if (ble.begin(VERBOSE_MODE) ) {
    error(F("Couldn't find bluefruit, is it in CMD mode? Wiring good?"));
  }
  
  useInterrupt(true);  // Keeps track of if we use interrupt - off by default
  delay(500);

  Serial.println("Adafruit GPS Lib Test, HarleyGPS");
  ble.println("HarleyGPS Testing...");

  if (FACTORYRESET_ENABLE) {
    Serial.println("Factory Resetting...");
    if (! ble.factoryReset() ) {
      error(F("Cannae reset, dangnabit"));
    }
  }

  ble.echo(true); //disable command echo from bluefruit
  ble.info(); //print bluefruit info

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request only RMCGGA data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // Updates at 5 hertz
  //GPS.sendCommand(PGCMD_ANTENNA); // Sets to internal antenna

  Serial.println("Harley GPS: Starting...");
  ble.println("Harley GPS: Starting... (BLEprintout");

  while (true) {
    Serial.print("Start logging...");
    ble.println("Start logging... (BLEprintout)");
    if (GPS.LOCUS_StartLogger()) {
      GPS.LOCUS_StartLogger();
      Serial.print("Signal found!");
      ble.println("Signal found! 9BLEprintout)");
      latA = (GPS.latitudeDegrees);
      longA = (GPS.longitudeDegrees);
      break;
    } else {
      Serial.println("No signal found, dammit");
      ble.println("No signal found, damnit (BLEprintout)");
    }
  }
}

// interrupt called once a millisecond, looks for new GPS data and stores it.
SIGNAL(TIMER0_COMPA_vect) { 
  char c = GPS.read();

#ifdef UDR0
  if (GPSECHO)
    if (c) UNDRO = c;
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Since timer0 used for millis(), will use "compare a" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the COMP function anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop() {
  float speedMPH = (GPS.speed * 1.15078); // convert speed from knots to MPH

  if (!usingInterrupt) { // read data from the GPS in 'main loop'
    char c = GPS.read();
    if (GPSECHO)
      if (c) Serial.print(c);
  }

  if (GPS.newNMEAreceived()) { // acquire new fix
    if (!GPS.parse(GPS.lastNMEA()));
  }
  
  if (timer > millis()) timer = millis();

  // print out current data every .5 seconds
  if (millis() - timer > 500) {
    timer = millis(); // reset the timer

    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); 
    Serial.print(':');
    Serial.print(GPS.minute, DEC); 
    Serial.print(':');
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    Serial.print(GPS.milliseconds);
    Serial.print("\nDate: 20");
    Serial.print(GPS.year, DEC);
    Serial.print('/');
    Serial.print(GPS.month, DEC);
    Serial.print('/');
    Serial.print(GPS.day, DEC);
    Serial.print("\nFix:  ");
    Serial.print((int)GPS.fix);
    Serial.print(" quality: ");
    Serial.println((int)GPS.fixquality);
    Serial.print("Satellites: ");
    Serial.println((int)GPS.satellites);

    ble.print("\nTime: ");
    ble.print(GPS.hour, DEC);
    ble.print(":");
    ble.print(GPS.minute, DEC);
    ble.print(":");
    ble.print(GPS.seconds, DEC);
    ble.print(".");
    ble.println(GPS.milliseconds);
    ble.print("Date: 20");
    ble.print(GPS.year, DEC);
    ble.print("/");
    ble.print(GPS.month, DEC);
    ble.print("/");
    ble.println(GPS.day, DEC);
    ble.print("Fix: ");
    ble.println((int)GPS.fix);
    ble.print("Quality: ");
    ble.println((int)GPS.fixquality);
    ble.print("Satellites: ");
    ble.println((int)GPS.satellites);

    Serial.print(latA, 8); 
    Serial.print(", "); 
    Serial.println(longA, 8);
    ble.print(latA, 8); 
    ble.print(", "); 
    ble.println(longA, 8);
    latB = GPS.latitudeDegrees;
    longB = GPS.longitudeDegrees;
    Serial.print(latB, 8); 
    Serial.print(", "); 
    Serial.println(longB, 8);
    ble.print(latB, 8); 
    ble.print(", "); 
    ble.println(longB, 8);

    // compute the distance between points A and B
    deltaLat = pow(((latB, 8) - (latA, 8)), 2);
    deltaLong = pow(((longB, 8) - (longA, 8)), 2);
    change = pow(((deltaLong + deltaLat), 8), .5);
    deltaDistance = (change * .00062137);
    distance += deltaDistance;

    Serial.print("Change is ");
    Serial.println(change, 8);
    Serial.print("The change between two points is: ");
    Serial.println(deltaDistance, 8);

    Serial.print("The distance is: "); 
    Serial.print(distance, 8); 
    Serial.println(" miles.");
    Serial.print(speedMPH); 
    Serial.println(" mph");

    ble.print("Change is ");
    ble.println(change, 8);
    ble.print("The change between two points is: ");
    ble.println(deltaDistance, 8);

    ble.print("The distance is: "); 
    ble.print(distance, 8); 
    ble.println(" miles.");
    ble.print(speedMPH); 
    ble.println(" mph");

    // move point B's coordinates into point A so next set of coordinates
    // will be set to new point B
    latA = latB;
    longA = longB;

    //lcd.clear();
    //lcd.setCursor(0, 0);
    //lcd.print("Speed: ");
    //lcd.print(speedMPH);
    //lcd.setCursor(0, 1);

    // Find max speed - if higher than previous max speed, replace with new speed
    if (speedMPH > maxSpeed) {
      maxSpeed = speedMPH; 
    } else {
      maxSpeed = maxSpeed;
    }

    //lcd.print("Max Speed: ");
    //lcd.print(maxSpeed);
  } 
}
