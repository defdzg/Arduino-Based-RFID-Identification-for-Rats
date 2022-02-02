#include <Elegoo_GFX.h>    // Core graphics library
#include <Elegoo_TFTLCD.h> // Hardware-specific library
#include "DHT.h"
#include <DS3231.h>
#include <Wire.h>
#include <MsTimer2.h>
#include <SD.h>
#include <SPI.h>


//Configuración del sensor de temperatura
#define DHTPIN 49        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22    // Modelo del sensor de temperatura y humedad DHT 22 
DHT dht(DHTPIN, DHTTYPE);
float h = 0;             // variable para almacenar la humedad
float t = 0;             // Variable para almacenar la temperatura
float luz=0;


//Configuracion del reloj
DS3231 Clock;
bool Century=false;
bool h12;
bool PM;
byte ADay, AHour, AMinute, ASecond, ABits;
bool ADy, A12h, Apm;

//Variables para inicializar el reloj en tiempo real
byte Year=20;
byte Month=7;
byte Date=8;
byte DoW;
byte Hour=15;
byte Minute=19;
byte Second=00;

//Variables para RFID
String inputString = "";         // a String to hold incoming data
String RataActual= "";
String RataPasada= "";
volatile bool stringComplete = false;  // whether the string is complete

//Variables para el toque por cero
const byte interruptPin = 19;       // Pin para recibir el detector de toque
const byte ledPin = 31;             //Solo es para verificar si se está tocando o no el sensor (se debe activar el LED al toque)
volatile byte touch = LOW;         // Estado inicial "No hay toque"
unsigned int relojON=0;             // Variables para medir los tiempos Altos y Bajos de la Señal de toque en milisegundos
unsigned int relojOFF=0;
bool Valid_OFF=false;               //Variables bandera para validar o no PULSOS validos
bool Valid_ON=false;
int TON_min=10;                    // Tiempos minimos en milisegundos para determinar un pulso valido
int TOFF_min=5;
unsigned int TON=0;                 //Variable de tiempo total de encendido y apagado
unsigned int TOFF=0;

 

//Pantalla
// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define PIN_SD_CS 53// Elegoo SD shields and modules: pin 53 on MEGA

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
unsigned int RelojPantalla=0; //contador en milisegundos para imprimir la informacion
#define NumeroMilis 1000
bool primeravez=true;
// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:

//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7

// Assign human-readable names to some common 16-bit color values:
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

Elegoo_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
// If using the shield, all control and data lines are fixed, and
// a simpler declaration can optionally be used:
// Elegoo_TFTLCD tft;

//Variables SD
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

/*********************************************/
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





void LeeSensores(){
 
 // Read humidity
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();

  luz=analogRead(A15);
  
  if (isnan(h) || isnan(t) ) {
  Serial.print('\n');
  Serial.print("Error de lectura sensor Humedad y Temperatura ");
  }  
 }
//

void LeeReloj() {
   Hour=Clock.getHour(h12, PM);
   Minute=Clock.getMinute();
   Second =Clock.getSecond();
}

void LeeCalendario() {
  Date=Clock.getDate(); 
  Month=Clock.getMonth(Century);
  Year=Clock.getYear();
}


void ImprimeStatus() {
  
    tft.setRotation(1);     
    tft.fillScreen(BLACK);
    
  
    tft.setCursor(0,0);
        tft.setTextSize(3);
    if (Hour<10) {  
            tft.print("0");
          }
        tft.print(Hour, DEC);
       
        tft.print(":");
        
        if (Minute<10) {  
            tft.print("0");
          }
        tft.print(Minute, DEC);
        tft.print(":");
       
        
        if (Second<10) {  
            tft.print("0");
          }
          
        tft.print(Second, DEC);
        tft.setTextSize(2);
        // Add AM/PM indicator
        if (h12) {
          if (PM) {
            tft.println(" PM");
          } else {
            tft.println(" AM");
          }
        }
       
      
        
        tft.setCursor(215,0);
        // Dia
        
        if (Date<10) {
          
            tft.print("0");}
          
          tft.print(Date, DEC);
          tft.print("/");
       
        if (Month<10) {  
            tft.print("0");
          }
        tft.print(Month, DEC);
       
        // Año
        tft.print("/");
        tft.println(Year, DEC);

  
    tft.setCursor(0,50);
    tft.setTextSize(2);
    tft.print("Temperatura:");
    tft.print(t);
    tft.println("C");
    tft.print("Humedad:");
    tft.print(h);
    tft.println("%");
    tft.print("Iluminacion:");
    tft.println(luz);
    tft.print("Rata ID:");
    tft.println(RataActual);
   
    
}
void setup() {
  uint16_t identifier = 0x9341;
  
  
  tft.begin(identifier);
  tft.setRotation(3);
  tft.fillScreen(BLACK);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  tft.setCursor(0,0);
  Serial.begin(9600);
  while (!Serial) {
   
  }
  Serial.println("LCD Ok");
  ///SD CARD
  if (!SD.begin(PIN_SD_CS)) { // Inicializa la comunicación con la tarjeta SD con el pin 53 como selector del SPI
    Serial.println("SD initialization failed, or not present!");
    while (1); // don't do anything more:
  }else
  Serial.println("SD initialization done."); 
  //
  
  // Start the I2C interface
  Wire.begin();
    
  //Inicializa sensor de temperatura y humedad
  dht.begin();
  Serial.println(F("Sensor de Humedad y Temperatura Ok"));

//Escribe fecha y hora del reloj
//  Clock.setClockMode(false);  // set to AM/PM
//  Clock.setYear(Year);
//  Clock.setMonth(Month);
//  Clock.setDate(Date);
//  Clock.setDoW(DoW);
//  Clock.setHour(Hour);
//  Clock.setMinute(Minute);
//  Clock.setSecond(Second);

//ConfiguraLector Tarjetas RFID
  Serial2.begin(9600);
  while (!Serial2) {
   
  }
  inputString.reserve(11);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  
  
  Serial.println("Lector RFID Ok");
  MsTimer2::set(1, Temporizador); // Interrupción para inicializar el contador del menu a 1 mili segundo
  attachInterrupt(digitalPinToInterrupt(interruptPin), Toque, CHANGE);
  MsTimer2::start();
  touch=LOW;

  //Despliega Imagen Primera vez 
  bmpFile = SD.open(__Gsbmp_files[0]);
    if (! bmpFile) {
        Serial.println("didnt find image");
        tft.setTextColor(WHITE);    tft.setTextSize(2);
        tft.println("Falta el logo");
        while (1);
    }

    if(! bmpReadHeader(bmpFile)) {
        Serial.println("bad bmp");
        tft.setTextColor(WHITE);    tft.setTextSize(1);
        tft.println("bad bmp");
        return;
    }
//    tft.setRotation(2);
//    bmpdraw(bmpFile, 0, 0);
//    bmpFile.close();
//    delay(1000);

   dataFile = SD.open("datalog1.txt", FILE_WRITE);



}

void Toque(){
  touch = !touch;
  digitalWrite(ledPin, touch);
  if(touch){                  //Si hay "toque" se inicializa las variables de conteo de reloj ON y se invalida el PULSO ON
    relojON=0;
    Valid_ON=false;
    
  }
  else{ 
    relojOFF=0;             //Si no hay "toque" se inicializa las variables de conteo de reloj OFF y se invalida el PULSO OFF
    Valid_OFF=false;
  }
   
}
void Temporizador() {
RelojPantalla=RelojPantalla+1;
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
            dataString += String(t);
            dataString += ",";
            dataString += String(h);
            dataString += ",";
            dataString += String(luz);
            dataString += ",";
            dataString += String(RataActual);
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
                Serial.println("Imprime");
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
        RataPasada=RataActual;
        RataActual=inputString;
        inputString = "";
        stringComplete = false;
        MsTimer2::start(); 
      }
  
}

void loop() {
  LeeSensores();
  LeeReloj();
  LeeCalendario();

  if(RelojPantalla>NumeroMilis){
    ImprimeStatus();
    RelojPantalla=0;
  }

  
 

////Lineas para programar el Lector RFID para el protocolo FDX-B ISO14223
////  String a;
////  Serial1.println("SD2");
////  while(Serial1.available()) {
////  a= Serial1.readString();// read the incoming data as string
////  Serial.println(a);
////  }
//
//
  
  
 }

void serialEvent2() {
  while (Serial2.available()) {
    // get the new byte:
    volatile char inChar = (char)Serial2.read();
    // add it to the inputString:
    inputString += inChar;
   
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if ((inChar == 13) && (!stringComplete)) {
      inputString.trim();
      stringComplete = true;
      
    }
  }
}




    
    
