/// GPS speed to CAN for LRR3, CAS-M II ///
/// Author: P. Jayaraman ///
/// Last edit date: 1/25/2016 ///

/// Description: Reads GPS speed from Adafruit Ultimate GPS Shield, translates to KPH, sends via CAN to LRR3 ///

// GPS Speed, Altitude, Latitude, Longitude, Satellites, HDOP, Date & Time over CAN
// Edited by : M. Wyant
// Last edit: 2/28/16

// Includes for GPS
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

// Includes for CAN shield
#include <mcp_can.h>
#include <SPI.h>

// Includes for math
#include <math.h>

// Declare and set the cs pin for SPI
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

// Designate pins 8 and 7 for SoftwareSerial comms between Arduino and GPS shield
SoftwareSerial mySerial(8,7);

// Designate the GPS to use the SoftwareSerial pins
Adafruit_GPS GPS(&mySerial);

// Codeword to enable echoing of GPS data to the serial console
// SET TO TRUE ONLY FOR DEBUGGING!!!
#define GPSECHO false

// Switch to use SoftwareSerial interrupt, will be off by default
boolean usingInterrupt = false;
void useInterrupt(boolean); // Don't touch this line, needed for weird Arduino stuff

// Variable declaration
typedef union {
  uint32_t  u32;
  float     f32;
} can_float;

//variables for all GPS data
uint8_t gpsDay;
uint8_t gpsMonth;
uint16_t gpsYear;
uint8_t gpsHour;
uint8_t gpsMinute;
uint8_t gpsSeconds;
uint16_t gpsMilliseconds;
byte gpsMillisecondsLSB;
byte gpsMillisecondsMSB;
byte gpsYearLSB;
byte gpsYearMSB;
can_float gpsLongitude;
byte gpsLon;
can_float gpsLatitude;
byte gpsLat;
can_float gpsAltitude;
can_float gpsHDOP;
uint8_t gpsSatellites;
float gpsSpeedKnots;
can_float gpsSpeedMPH;
char gpsNS; //North or South
char gpsEW; //East or West

// arrays to send messgaes over CAN
unsigned char canOutTime[5];
unsigned char canOutDate[4];
unsigned char canOutLongitude[5];
unsigned char canOutLatitude[5];
unsigned char canOutAltitude[4];
unsigned char canOutPrecision[5];
unsigned char canOutSpeed[4];

int i;

void setup()
{
  // Connect at max serial speed to best read GPS data
  Serial.begin(115200);
  Serial.println("Serial comms initialized");
  
  //Initialize the CAN-bus comms
START_INIT:
  if(CAN_OK == CAN.begin(CAN_1000KBPS)) // Initialize at 1000 kbaud speed
  {
    Serial.println("CAN-bus succesfully initialized at 1000k");
  }
  else
  {
    Serial.println("CAN-bus initialization failed, retrying...");
    delay(500);
    goto START_INIT; // Retry initialization
  }
  
  // Set up baud rate for comms between GPS and Arduino, default is 9600 NMEA
  GPS.begin(9600);
  Serial.println("GPS comms initialized");
  
  // Turn on RMC data from GPS only (recommended minimum); fix data not needed
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //Turn on recommended minimum data and fix data (includes altitude)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set update rate to 10 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  
  // Request an update on antenna status, USE ONLY FOR DEBUGGING]
  //GPS.sendCommand(PGCMD_ANTENNA);
  
  // Start an interrupt
  useInterrupt(true);
  
  delay(1000);
  
  // Ask for the firmware version from the GPS unit, USE ONLY FOR DEBUGGING
  //mySerial.println(PMTK_Q_RELEASE);
}

// Interrupt comparison definition - checks if data collected in interrupt is new
SIGNAL(TIMER0_COMPA_vect)
{
  char c = GPS.read();
  #ifdef UDR0
    if (GPSECHO)
      if (c) UDR0 = c;
  #endif
}

void useInterrupt(boolean v)
{
  if (v)
  {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else
  {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop()
{
  // if we choose not to use the interrupt, loop has to query for data each loop-through
  if (!usingInterrupt)
  {
    char c = GPS.read();
    
    // Debug lines, COMMENT OUT IF NOT DEBUGGING
    //if (GPSECHO)
    //  if (c) Serial.println(c);
  }
  
  // If a sentence is received, check checksum and parse it
  if(GPS.newNMEAreceived())
  {
    // If we fail to parse the sentence, wait for the next one... next line sets newNMEAreceived flag to false
    if(!GPS.parse(GPS.lastNMEA()))
      return;
  }
  
  // Reset the timer if it wraps around
  if (timer > millis()) timer = millis();


  // assigns gps data to variables
  gpsDay = GPS.day;
  gpsMonth = GPS.month;
  gpsYear = 2000 + GPS.year;
  gpsHour = GPS.hour;
  gpsMinute = GPS.minute;
  gpsSeconds = GPS.seconds;
  gpsMilliseconds = GPS.milliseconds;
  gpsHDOP.f32 = GPS.HDOP;
  gpsSatellites = GPS.satellites;
  gpsSpeedKnots = GPS.speed;
  gpsSpeedMPH.f32 = gpsSpeedKnots * 1.15078; //convert to miles per hour


  gpsYearLSB = extractByte(gpsYear,0);
  gpsYearMSB = extractByte(gpsYear,1);
  gpsMillisecondsLSB = extractByte(gpsMilliseconds,0);
  gpsMillisecondsMSB = extractByte(gpsMilliseconds,1);

  gpsLongitude.f32 = GPS.longitude;
  gpsEW = GPS.lon;
  gpsNS = GPS.lat;
  gpsLatitude.f32 = GPS.latitude;
  gpsAltitude.f32 = GPS.altitude;

  //build CAN messages
  canOutDate[0] = (byte) gpsDay;
  canOutDate[1] = (byte) gpsMonth;
  canOutDate[2] = gpsYearLSB;
  canOutDate[3] = gpsYearMSB;

  canOutTime[0] = (byte) gpsHour;  
  canOutTime[1] = (byte) gpsMinute; 
  canOutTime[2] = (byte) gpsSeconds; 
  canOutTime[3] = gpsMillisecondsLSB; 
  canOutTime[4] = gpsMillisecondsMSB;

  canOutLongitude[0] = extractByte(gpsLongitude.u32,0);
  canOutLongitude[1] = extractByte(gpsLongitude.u32,1);
  canOutLongitude[2] = extractByte(gpsLongitude.u32,2);
  canOutLongitude[3] = extractByte(gpsLongitude.u32,3);
  canOutLongitude[4] = gpsEW;

  canOutLatitude[0] = extractByte(gpsLatitude.u32,0);
  canOutLatitude[1] = extractByte(gpsLatitude.u32,1);
  canOutLatitude[2] = extractByte(gpsLatitude.u32,2);
  canOutLatitude[3] = extractByte(gpsLatitude.u32,3);
  canOutLatitude[4] = gpsNS;

  canOutAltitude[0] = extractByte(gpsAltitude.u32,0);
  canOutAltitude[1] = extractByte(gpsAltitude.u32,1);
  canOutAltitude[2] = extractByte(gpsAltitude.u32,2);
  canOutAltitude[3] = extractByte(gpsAltitude.u32,3);

  canOutPrecision[0] = extractByte(gpsHDOP.u32,0);
  canOutPrecision[1] = extractByte(gpsHDOP.u32,1);
  canOutPrecision[2] = extractByte(gpsHDOP.u32,2);
  canOutPrecision[3] = extractByte(gpsHDOP.u32,3);
  canOutPrecision[4] = (byte) gpsSatellites;

  canOutSpeed[0] = extractByte(gpsSpeedMPH.u32,0);
  canOutSpeed[1] = extractByte(gpsSpeedMPH.u32,1);
  canOutSpeed[2] = extractByte(gpsSpeedMPH.u32,2);
  canOutSpeed[3] = extractByte(gpsSpeedMPH.u32,3);
  
/// Send CAN messages
CAN.sendMsgBuf(0x2BC,0,4,canOutDate); //GPS Date CAN ID 700 MONTH DATE YEAR
CAN.sendMsgBuf(0x2BD,0,5,canOutTime); //GPS Time CAN ID 701 HOURS MINUTES SECONDS MICROSECONDS
CAN.sendMsgBuf(0x2BE,0,5,canOutLongitude); //GPS LONGITUDE CAN ID 702
CAN.sendMsgBuf(0x2BF,0,5,canOutLatitude); //GPS LATITUDE CAN ID 703
CAN.sendMsgBuf(0x2C0,0,4,canOutAltitude); //GPS ALTITUDE CAN ID 704
CAN.sendMsgBuf(0x2C1,0,5,canOutPrecision); //GPS PRECISION CAN ID 705 HDOP NUMBER OF SATELLITES
CAN.sendMsgBuf(0x2C2,0,4,canOutSpeed); //GPS SPEED CAN ID 706 MPH

/// section useful for debugging
/*
if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("HDOP"); Serial.println(GPS.HDOP);
      Serial.print("Speed (MPH): "); Serial.println(gpsSpeedMPH.f32);
    }
  }
*/

  delay(50);
}

// Function for extracting bytes from a 16 bit integer
byte extractByte(long value, int byteIdent) // Inputs are the long value to concate and an ID for which byte to get (0 = lsb, 1 = msb)
{
  byte output;
  if (byteIdent == 0)
  {
    // Extract LSB
    output = (byte) value & 0xFF;
  }
  else if (byteIdent == 1)
  {
    // Extract MSB
    output = (byte) (value >> 8) & 0xFF;
  }
  else if (byteIdent == 2) {
    output = (byte) (value >> 16) & 0xFF;
  }
  else {
    output = (byte) (value >> 24) & 0xFF;
  }
  return output;
}

  
  
