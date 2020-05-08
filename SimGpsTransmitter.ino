#include <Adafruit_FONA.h>

#include <SoftwareSerial.h>
#include <TinyGPS++.h>

static String apn="em";
static String http="http://";
static String loggingPassword="";
static String serverIP="gps.oruss.com.mx";
static String IMEI="862643033176755";
// Pins where GPS and SIM modules are connected
static const int SimRXPin = 15, SimTXPin = 14;
static const int GPSRXPin = 9, GPSTXPin = 8;
static const int ErrorPin = 10, SimConnectionPin = 12;
char rssi2;
int DIGI1 = 4;
int DIGI2 = 5;
int AIN1 = A1;
int AIN2 = A2;
int Batteryin = A3;
#define FONA_RST 10

static const unsigned long frequency = 60000;

// Maximum time to wait SIM module for response
static long maxResponseTime = 30000;
String responseString;
TinyGPSPlus gps;
unsigned long previous=0;
unsigned long beltTime;

SoftwareSerial sim_ss(SimRXPin, SimTXPin);
SoftwareSerial gps_ss(GPSRXPin, GPSTXPin);
SoftwareSerial fonaSS = SoftwareSerial(SimRXPin, SimTXPin);
SoftwareSerial *fonaSerial = &fonaSS;
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Hardware serial is also possible!
//  HardwareSerial *fonaSerial = &Serial1;

// Use this for FONA 800 and 808s
        uint8_t n = fona.getRSSI();
        int8_t r;


void setup()
{
  int valor;
  pinMode(DIGI1, INPUT_PULLUP);
  pinMode(DIGI2, INPUT_PULLUP);
  pinMode(ErrorPin, OUTPUT);
  pinMode(SimConnectionPin, OUTPUT);
  digitalWrite(ErrorPin, LOW);
  digitalWrite(SimConnectionPin, LOW);          
  gps_ss.begin(9600); 
  sim_ss.begin(115200);
  sim_ss.println("AT+IPR=9600");
  delay(100);
  sim_ss.end();
  delay(100);
  sim_ss.begin(9600);
  Serial.begin(9600);  

  delay(5000); 
  Serial.println("Iniciando... esperando hasta que el modulo se conecte a la red"); 
  
   sim_ss.println("AT");
  // Wait until module is connected and ready
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();
  
  // Full mode
  sim_ss.println("AT+CFUN=1");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();

  sim_ss.write("AT+CSTT=\"");
  sim_ss.print(apn);
  sim_ss.write("\",\"\",\"\"\r\n");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();

  // Connect and get IP
  sim_ss.println("AT+CIICR");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();

  // Some more credentials
  sim_ss.write("AT+SAPBR=3,1,\"APN\",\"");
  sim_ss.print(apn);
  sim_ss.write("\"\r\n");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();
  
  sim_ss.println("AT+SAPBR=3,1,\"USER\",\"\"");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();
 
  sim_ss.println("AT+SAPBR=3,1,\"PWD\",\"\"");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();

  sim_ss.println("AT+SAPBR=1,1");
  waitUntilResponse("OK");
  blinkLed(SimConnectionPin);

updateSerial();

  sim_ss.println("AT+HTTPINIT");
  waitUntilResponse("OK");
  digitalWrite(SimConnectionPin, HIGH);

updateSerial();
  
  gps_ss.listen();
  previous = millis();
  Serial.println("Iniciando Comunicación Satelital");
  logInfo();

}

void blinkLed(int led)
{
  digitalWrite(led, HIGH);
  delay(20);
  digitalWrite(led, LOW);
}

void waitUntilResponse(String response)
{
  beltTime = millis();
  responseString="";
  String totalResponse = "";
  while(responseString.indexOf(response) < 0 && millis() - beltTime < maxResponseTime)
  {
    readResponse();
    totalResponse = totalResponse + responseString;
    Serial.println(responseString);
  }

  if(totalResponse.length() <= 0)
  {
    Serial.println("No hay respuesta del modulo. Revisar conexiones, tarjeta SIM y voltaje!");
    digitalWrite(ErrorPin, HIGH);
    delay(3000);
    exit(0); // No way to recover
  }
  else if (responseString.indexOf(response) < 0)
  {
    Serial.println("Respuesta inesperada del modulo");
    Serial.println(totalResponse);
    digitalWrite(ErrorPin, HIGH);
    delay(3000);
    exit(0); // No way to recover
  }
}
void readResponse()
{
  responseString = "";
  while(responseString.length() <= 0 || !responseString.endsWith("\n"))
  {
    tryToRead();

    if(millis() - beltTime > maxResponseTime)
    {
      return;
    }
  }
}
void tryToRead()
{
  while(sim_ss.available())
  {
    char c = sim_ss.read();  //gets one byte from serial buffer
    responseString += c; //makes the string readString
  }
}

void loop()
{
  // If we have data, decode and log the data
  while (gps_ss.available() > 0)
   if (gps.encode(gps_ss.read()))
    logInfo();

  // Test that we have had something from GPS module within first 10 seconds
  if (millis() - previous > 10000 && gps.charsProcessed() < 10)
  {
    Serial.println("error de coneción de GPS!");
    while(true);
  }

}
void logInfo()
{  
  
    uint8_t hr_val, min_val, sec_val;
    int Day_val, Month_val;
    long  Year_val, vel_val;
    double angle_val;
    int current_val;
    double alt_m_val;  
    int rssi_val; 
    int value = 0;
    int value2 = 0;
    int valueA1 = 0;
    int valueA2 = 0;
    int valueBat = 0;
    long positionA1,positionA2;
    int positionBat;

    Day_val = gps.date.day();
    Month_val = gps.date.month();
    Year_val = gps.date.year();
    hr_val = gps.time.hour(); 
    min_val = gps.time.minute();  
    sec_val = gps.time.second(); 
    alt_m_val = gps.altitude.meters();
    vel_val = 0;
    angle_val=0;
    rssi_val=0;
    current_val=0;

    value = digitalRead(DIGI1);
    value2 = digitalRead(DIGI2);
    valueA1 = analogRead(AIN1);
    valueA2 = analogRead(AIN2);
    valueBat = analogRead(Batteryin);
    positionA1 = map(valueA1, 0, 1023, 0, 5);
    positionA2 = map(valueA2, 0, 1023, 0, 5);
    positionBat = map(valueBat, 0, 1023, 0, 4);
  // Causes us to wait until we have satelite fix


  if(!gps.location.isValid())
  {
    Serial.println("Ubicación invalida. esperando por conexión satelital.");
    blinkLed(ErrorPin);
    return;
  }

String latitud(gps.location.lat(), DEC);
String longitud (gps.location.lng(),DEC);
String alt_string(gps.altitude.meters(),DEC);
String dig1state(value);
String digistate2(value2);
String Vel(vel_val); 
String angle(angle_val);
String rssi(rssi_val);
String bat(positionBat);
String amps(current_val);
String Hrs;
String Mins;
String Segs;
String Month;
String Day;

  if(millis() - previous > frequency)
  {
    if(gps.time.hour() < 10){
       Hrs = "0";
    }else{Hrs="";}
    if(gps.time.minute() < 10){
        Mins = "0";
    }else{Mins=""; }  
    if(gps.time.second() < 10){ 
        Segs = "0";
    }else{ Segs="";}
    if(gps.date.month() < 10){
        Month = "0";
    }else{Month="";}
    if(gps.date.day() < 10){
        Day = "0";
    }else{Day="";}
    
    sim_ss.listen();
    previous = millis();

 
        Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
        if (n == 0) r = -115;
        if (n == 1) r = -111;
        if (n == 31) r = -52;
        if ((n >= 2) && (n <= 30)) {
          r = map(n, 2, 30, -110, -54);
        }
        Serial.print(r); Serial.println(F(" dBm"));

   
    String  url ="AT+HTTPPARA=\"URL\",\"";
    url += http;
    url += serverIP;
    url += ":11904";
    url += "/?";
    url += IMEI;
    url += ",";
    url += Year_val;
    url += "-";
    url += Month;
    url += Month_val;
    url += "-";
    url += Day;
    url += Day_val;
    url += " ";
    url += Hrs;
    url += hr_val;
    url += ":";
    url += Mins;
    url += min_val;
    url += ":";
    url += Segs;
    url += sec_val;
    url += ",";
    url += latitud;
    url += ",";
    url += longitud;
    url += ",";
    url += alt_m_val;
    url += ",";
    url += vel_val;
    url += ",";
    url += angle_val;
    url += ",";
    url += dig1state;
    url += ",";
    url += digistate2;
    url += ",";
    url += positionA1;
    url += ",";
    url += positionA2;
    url += ",";
    url += n;
    url += ",";
    url += bat;
    url += ",";
    url += amps;
    url += "\"";
    sim_ss.println(url);
    waitUntilResponse("OK");
    digitalWrite(SimConnectionPin, LOW);
    sim_ss.println("AT+HTTPACTION=0");
    waitUntilResponse("+HTTPACTION:");
    digitalWrite(SimConnectionPin, HIGH); 
    gps_ss.listen();
  }
}
void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    sim_ss.write(Serial.read());
  }
  while(sim_ss.available()) 
  {
    Serial.write(sim_ss.read());
  }
}
