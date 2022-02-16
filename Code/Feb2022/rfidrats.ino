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
const byte touch_sensor_pin = 19;
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
volatile byte touch = LOW;         // Estado inicial "No hay toque"
unsigned int relojON=0;             // Variables para medir los tiempos Altos y Bajos de la Señal de toque en milisegundos
unsigned int relojOFF=0;
bool Valid_OFF=false;               //Variables bandera para validar o no PULSOS validos
bool Valid_ON=false;
int TON_min=10;                    // Tiempos minimos en milisegundos para determinar un pulso valido
int TOFF_min=5;
unsigned int TON=0;                 //Variable de tiempo total de encendido y apagado
unsigned int TOFF=0;


// MPR121
uint16_t lasttouched = 0;
uint16_t currtouched = 0;
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

#define MAX_BMP         10                      // bmp file num
#define FILENAME_LEN    20                      // max file name length

const int __Gnbmp_height = 320;                 // bmp hight
const int __Gnbmp_width  = 240;                 // bmp width

unsigned char __Gnbmp_image_offset  = 0;        // offset

int __Gnfile_num = 1;                           // num of file

char __Gsbmp_files[5][FILENAME_LEN] =           // add file name here
{
"UAEMrot.bmp",
};
File bmpFile;

/*********************************************/
// This procedure reads a bitmap and draws it to the screen
// its sped up by reading many pixels worth of data at a time
// instead of just one pixel at a time. increading the buffer takes
// more RAM but makes the drawing a little faster. 20 pixels' worth
// is probably a good place

#define BUFFPIXEL       60                      // must be a divisor of 240 
#define BUFFPIXEL_X3    180                     // BUFFPIXELx3


// LCD SCREEN ADITIONAL SETUP //
void bmpdraw(File f, int x, int y)
{
    bmpFile.seek(__Gnbmp_image_offset);

    uint32_t time = millis();

    uint8_t sdbuffer[BUFFPIXEL_X3];                 // 3 * pixels to buffer

    for (int i=0; i< __Gnbmp_height; i++) {
        for(int j=0; j<(240/BUFFPIXEL); j++) {
            bmpFile.read(sdbuffer, BUFFPIXEL_X3);

            uint8_t buffidx = 0;
            int offset_x = j*BUFFPIXEL;
            unsigned int __color[BUFFPIXEL];

            for(int k=0; k<BUFFPIXEL; k++) {
                __color[k] = sdbuffer[buffidx+2]>>3;                        // read
                __color[k] = __color[k]<<6 | (sdbuffer[buffidx+1]>>2);      // green
                __color[k] = __color[k]<<5 | (sdbuffer[buffidx+0]>>3);      // blue

                buffidx += 3;
            }

      for (int m = 0; m < BUFFPIXEL; m ++) {
              tft.drawPixel(m+offset_x, i,__color[m]);
      }
        }
    }

    Serial.print(millis() - time, DEC);
    Serial.println(" ms");
}

boolean bmpReadHeader(File f) 
{
    // read header
    uint32_t tmp;
    uint8_t bmpDepth;

    if (read16(f) != 0x4D42) {
        // magic bytes missing
        return false;
    }

    // read file size
    tmp = read32(f);
    Serial.print("size 0x");
    Serial.println(tmp, HEX);

    // read and ignore creator bytes
    read32(f);

    __Gnbmp_image_offset = read32(f);
    Serial.print("offset ");
    Serial.println(__Gnbmp_image_offset, DEC);

    // read DIB header
    tmp = read32(f);
    Serial.print("header size ");
    Serial.println(tmp, DEC);

    int bmp_width = read32(f);
    int bmp_height = read32(f);

    if(bmp_width != __Gnbmp_width || bmp_height != __Gnbmp_height)  {    // if image is not 320x240, return false
        return false;
    }

    if (read16(f) != 1)
    return false;

    bmpDepth = read16(f);
    Serial.print("bitdepth ");
    Serial.println(bmpDepth, DEC);

    if (read32(f) != 0) {
        // compression not supported!
        return false;
    }

    Serial.print("compression ");
    Serial.println(tmp, DEC);

    return true;
}

// SD CARD SETUP
// These read data from the SD card file and convert them to big endian
// (the data is stored in little endian format!)

// LITTLE ENDIAN!
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

// LITTLE ENDIAN!
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
  tft.print("ID Rata:");
  tft.println(current_rat);
  tft.print("Temperatura:");
  tft.print(temperature);
  tft.println(" C");
  tft.print("Humedad:");
  tft.print(humidity);
  tft.println(" %");
  tft.print("Luz:");
  tft.println(light);
  tft.print("Tiempo contacto: ");
  tft.print(relojON);
  tft.println(" ms");
}

void Timer() {
  
  screen_timer=screen_timer+1;
      if(touch){
        if(Valid_OFF){
          relojON=relojON+1;  
        }
        else{
          relojON=relojON+relojOFF;
          relojOFF=0;
          Valid_OFF=true;
          
        } 
        
        if ((relojON>TON_min)&&(!Valid_ON)){  //Condiciones para determinar un pulso ON valido
          Valid_ON=true;
        }
      }
      else{
        relojOFF=relojOFF+1;   
        if((relojOFF>TOFF_min)&&(Valid_ON)&&(!Valid_OFF)){ //Se imprime el valor siempre y cuando el pulso de bajada sea valido
          
          Valid_OFF=true;
          MsTimer2::stop();
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
            dataString += String(relojON);
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
          else {
            Serial.println("Error Interrupt");
          }
          }
          if(cuentaImpresiones==11){
           dataFile.close();
           Serial.println("Archivo Cerrado");
          }
          }
          
          MsTimer2::start(); 
        }

      if (stringComplete) {
        MsTimer2::stop();
        last_rat=current_rat;
        current_rat = tag;
        tag = "";
        stringComplete = false;
        MsTimer2::start(); 
      }
  
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

/* MPR121 Capacitive Touch Sensor
void TouchSensor() {

  currtouched = cap.touched();
  
  for (uint8_t i=0; i<12; i++) {
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      digitalWrite(touch_sensor_led_pin, HIGH);
      touch_start = millis();
    }
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      digitalWrite(touch_sensor_led_pin, LOW);
      touch_stop = millis();
    }
  }

  elapsed_time = touch_stop - touch_start;
  lasttouched = currtouched;
} */

// Push button as touch sensor
void TouchSensorActivated() {
  touch = !touch;
  digitalWrite(touch_sensor_led_pin, touch);
  if(touch){                  //Si hay "toque" se inicializa las variables de conteo de reloj ON y se invalida el PULSO ON
    relojON=0;
    Valid_ON=false;

  }
  else{ 
    relojOFF=0;             //Si no hay "toque" se inicializa las variables de conteo de reloj OFF y se invalida el PULSO OFF
    Valid_OFF=false;
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
  tft.setRotation(1);
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN);

  // Clock initialization
  SetClock();
  MsTimer2::set(1, Timer);
  MsTimer2::start();

  // SD initialization
  if (!SD.begin(PIN_SD_CS)) {
    tft.println("SD ... Failure.");
    //while (1);
  }else
  tft.println("SD ... Success.");
  tag.reserve(11);
  dataFile = SD.open("datalog1.txt", FILE_WRITE);
  delay(2000);
  
  // DHT sensor initialization
  dht.begin();
  tft.println("DHT ... Success."); 
  delay(2000);

  // RFID initialization
  if (!Serial2) {
    tft.println("RFID ... Failure.");
    //while (1);
  }else
  tft.println("RFID ... Success."); 
  delay(2000);

  // Touch sensor initialization
  pinMode(touch_sensor_led_pin, OUTPUT);
  digitalWrite(touch_sensor_led_pin, LOW);
  attachInterrupt(digitalPinToInterrupt(touch_sensor_pin), TouchSensorActivated, CHANGE);

  // Adafruit MPR121
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 ... Failure.");
    // while (1);
  }
  // tft.println("MPR121 ... Success.");
  delay(2000);

}

void loop() {

  ReadSensors();
  GetClock();

  if(screen_timer>update_screen_time){
    UpdateScreen();
    screen_timer=0;
  }
  
}

