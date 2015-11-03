/** Program Name:    HarleyGPS_v.6
 * Written by:      Jesse Ragsdale
 * Date:            2015/01/29
 *
 * Credits:         Adafruit, LadyAda, Becky Stern, and Tyler Cooper
 *
 * Goal:            Record and display Average Speed, Maximum Speed, and distance
 *                  traveled in mph. 
 */

#include <Adafruit_GPS.h>
#include <math.h>
#include <SoftwareSerial.h>
//#include <LiquidCrystal.h>

//LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
SoftwareSerial mySerial(2, 3);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO true

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

void setup() {
  //pinMode(13, OUTPUT);
  while (!Serial);
  Serial.begin(115200);
  //lcd.begin(16, 2);
  GPS.begin(9600);

  useInterrupt(true);  // Keeps track of if we use interrupt - off by default
  delay(500);

  Serial.println("Adafruit GPS Lib Test, HarleyGPS");
  //lcd.print("HarleyGPS v.6");
  //lcd.setCursor(0, 1);
  //lcd.print("Ready to run?");
  //delay(1000);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // Request only RMCGGA data
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); // Updates at 5 hertz
  GPS.sendCommand(PGCMD_ANTENNA); // Sets to internal antenna

  Serial.println("Harley GPS: Starting...");
  //lcd.setCursor(0, 1);
  //lcd.print("Starting GPS...");

  while (GPS.fix = 1) {
    //lcd.clear();
    //lcd.setCursor(0, 0);
    Serial.print("Start logging...");
    if (GPS.LOCUS_StartLogger()) {
      GPS.LOCUS_StartLogger();
      Serial.print("Signal found!");
      //digitalWrite(13, HIGH);
      latA = (GPS.latitudeDegrees);
      longA = (GPS.longitudeDegrees);
      break;
    } else {
      Serial.println("No signal found, dammit");
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

    Serial.print(latA, 8); 
    Serial.print(", "); 
    Serial.println(longA, 8);
    latB = GPS.latitudeDegrees;
    longB = GPS.longitudeDegrees;
    Serial.print(latB, 8); 
    Serial.print(", "); 
    Serial.println(longB, 8);

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
