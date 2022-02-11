// sudo chmod a+rw /dev/ttyACM0

// LIBRARIES //
#include <Elegoo_GFX.h>
#include <Elegoo_TFTLCD.h>
#include "DHT.h"
#include <DS3231.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SD.h>
#include <SPI.h>
#include "Adafruit_MPR121.h"

// PINS INITIALIZATION //
const byte temperature_sensor_pin = 49;
const byte touch_sensor_led_pin = 31;
const byte light_sensor_pin = 15;
const byte sd_pin = 53;

// MEASURED VARIABLES //
float humidity = 0;
float temperature = 0;
float light = 0;

// RATS ID //
String last_rat = "";
String current_rat = "";

// TEMPERATURE SENSOR SETUP//
#define DHTPIN temperature_sensor_pin   // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // Temperature and humidity sensor model DHT 22 
DHT dht(DHTPIN, DHTTYPE);   //Initializa DHT sensor

// RFID SENSOR //
String tag = "";         // a String to hold incoming data
volatile bool stringComplete = false;  // whether the string is complete

// TOUCH SENSOR //
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
unsigned int touch_start = 0;
unsigned int touch_stop = 0;
unsigned int elapsed_time = 0;
#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif
Adafruit_MPR121 cap = Adafruit_MPR121();

// CLOCK SETUP //
DS3231 Clock;
byte Year = 22;
byte Month = 2;
byte Date = 10;
byte Hour = 13;
byte Minute = 43;
byte Second = 0;
bool Century  = false;
bool h12 ;
bool PM ;
unsigned int screen_timer = 0;

// LCD SCREEN SETUP //
#define update_screen_time 1000
#define LCD_CS A3
#define LCD_CD A2 
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4
#define BLACK   0x0000
#define GREEN   0x07E0

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

// SD CARD SETUP //
#define PIN_SD_CS sd_pin
File dataFile;
String dataString;
int cuentaImpresiones=0;

uint16_t read16(File f)
{
    uint16_t d;
    uint8_t b;
    b = f.read();
    d = f.read();
    d <<= 8;
    d |= b;
    return d;
}

uint32_t read32(File f)
{
    uint32_t d;
    uint16_t b;

    b = read16(f);
    d = read16(f);
    d <<= 16;
    d |= b;
    return d;
}

// FUNCTIONS //

void ReadSensors() {

  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  light=analogRead(light_sensor_pin);
  
  if (isnan(humidity) || isnan(temperature) ) {
  Serial.print('\n');
  Serial.print("There's a problem with the DHT sensor.");
  }

}

void SetClock() {

    Clock.setClockMode(false);  // set to 24h
    Clock.setSecond(Second);
    Clock.setMinute(Minute);
    Clock.setHour(Hour);
    Clock.setDate(Date);
    Clock.setMonth(Month);
    Clock.setYear(Year);

}

void GetClock() {

    Hour=Clock.getHour(h12, PM);
    Minute=Clock.getMinute();
    Second =Clock.getSecond();
    Date=Clock.getDate(); 
    Month=Clock.getMonth(Century);
    Year=Clock.getYear();

}

void UpdateScreen() {

  tft.setCursor(0,0);
  tft.fillScreen(BLACK);
  
  // Time from clock
  tft.setTextSize(2);
  if (Hour<10) {  
        tft.print("0");
  }
  tft.print(Hour);
  tft.print(":");
  if (Minute<10) {  
        tft.print("0");
  }
  tft.print(Minute);
  tft.print(":");
  if (Second<10) {  
        tft.print("0");
  }
  tft.print(Second);
  tft.setTextSize(2);

  // Date from clock
  tft.setCursor(215,0);
  if (Date<10) {  
        tft.print("0");
  }
  tft.print(Date);
  tft.print("/");
  if (Month<10) {  
        tft.print("0");
  }
  tft.print(Month);
  tft.print("/");
  tft.print(Year);

  // Sensors data
  tft.setCursor(0,50);
  tft.print("Rat ID:");
  tft.println(current_rat);
  tft.print("Temperature:");
  tft.print(temperature);
  tft.println(" C");
  tft.print("Humidity:");
  tft.print(humidity);
  tft.println(" %");
  tft.print("Light:");
  tft.println(light);

}

void Timer() {

  screen_timer=screen_timer+1;

}

// RFID tag reading
void serialEvent2() {

  while (Serial2.available()) {
    volatile char inChar = (char)Serial2.read();
    tag += inChar;

    if ((inChar == 13) && (!stringComplete)) {
      tag.trim();
      stringComplete = true;
    }
  }
  
}

void TouchSensor() {

  currtouched = cap.touched();
  
  for (uint8_t i=0; i<12; i++) {
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      digitalWrite(touch_sensor_led_pin, HIGH);
      touch_start = millis();
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      digitalWrite(touch_sensor_led_pin, LOW);
      touch_stop = millis();
    }
  }

  elapsed_time = touch_stop - touch_start;
  lasttouched = currtouched;
}

void SaveData() {

    cuentaImpresiones=cuentaImpresiones+1;

    if(cuentaImpresiones<11){
      dataString = "";
      dataString += String(current_rat);
      dataString += ",";
      dataString += String(humidity);
      dataString += ",";
      dataString += String(temperature);
      dataString += ",";
      dataString += String(light);
      dataString += ",";
      dataString += String(elapsed_time);
      dataString += ",";
      dataString += String(Date);
      dataString += ",";
      dataString += String(Month);
      dataString += ",";
      dataString += String(Year);
      dataString += ",";
      dataString += String(Hour);
      dataString += ",";
      dataString += String(Minute);
      dataString += ",";
      dataString += String(Second);
      if (dataFile) {
            dataFile.println(dataString);
            Serial.println(dataString);
      }
    }
    if(cuentaImpresiones==11){
      dataFile.close();
      Serial.println("Archivo Cerrado");
    }

    if (stringComplete) {
      MsTimer2::stop();
      tag = "";
      stringComplete = false;
      MsTimer2::start(); 
    }

}

void setup() {

  // Arduino communication protocols
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin(); // IC2 interface

  // LCD screen initialization
  uint16_t identifier = 0x9341;
  tft.begin(identifier);
  Serial.println("LCD ...  Success.");
  tft.setRotation(3);
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN);

  // Clock initialization
  SetClock();
  MsTimer2::set(1, Timer);
  MsTimer2::start();

  // SD initialization
  if (!SD.begin(PIN_SD_CS)) {
    Serial.println("SD ... Failure.");
    // while (1);
  }else
  Serial.println("SD ... Success.");
  tag.reserve(11);
  dataFile = SD.open("datalog1.txt", FILE_WRITE);

  // DHT sensor initialization
  dht.begin();
  Serial.println("DHT ... Success."); 

  // RFID initialization
  if (!Serial2) {
    Serial.println("RFID ... Failure.");
    // while (1);
  }else
  Serial.println("RFID ... Success."); 
  
  // Touch sensor initialization
  pinMode(touch_sensor_led_pin, OUTPUT);
  digitalWrite(touch_sensor_led_pin, LOW);
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 ... Failure.");
    // while (1);
  }
  Serial.println("MPR121 ... Success.");
}

void loop() {
  ReadSensors();
  GetClock();
  TouchSensor();

  if(screen_timer>update_screen_time){
    UpdateScreen();
    screen_timer=0;
  }
  
}

