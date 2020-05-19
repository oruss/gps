#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <stdlib.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
static String apn = "em";
static String http = "http://";
static String loggingPassword = "";
static String serverIP = "gps.oruss.com.mx";
static String IMEI = "862643033176755";
// Pins where GPS and SIM modules are connected
static const int SimRXPin = 15, SimTXPin = 14;
static const int GPSRXPin = 9, GPSTXPin = 8;
static const int ErrorPin = 10, SimConnectionPin = 12;
MPU6050 sensor;
int DIGI1 = 4;
int DIGI2 = 5;
int AIN1 = A1;
int AIN2 = A2;
int ax, ay, az;
int gx, gy, gz;
int gps_fix;
int gps_hdop;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;
char r;
char n;
static const unsigned long frequency = 60000;
// Maximum time to wait SIM module for response
static long maxResponseTime = 40000;
String responseString;
TinyGPSPlus gps;
unsigned long previous = 0;
unsigned long beltTime;
SoftwareSerial sim_ss(SimRXPin, SimTXPin);
SoftwareSerial gps_ss(GPSRXPin, GPSTXPin);
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
  Wire.begin();
  sensor.initialize();
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
  responseString = "";
  String totalResponse = "";
  while (responseString.indexOf(response) < 0 && millis() - beltTime < maxResponseTime)
  {
    readResponse();
    totalResponse = totalResponse + responseString;
    Serial.println(responseString);
  }

  if (totalResponse.length() <= 0)
  {
    Serial.println("No response from the module. Check wiring, SIM-card and power!");
    digitalWrite(ErrorPin, HIGH);
    delay(30000);
    exit(0);
  }
  else if (responseString.indexOf(response) < 0)
  {
    Serial.println("Unexpected response from the module");
    Serial.println(totalResponse);
    digitalWrite(ErrorPin, HIGH);
    delay(30000);
    exit(0); // No way to recover
  }
}
void readResponse()
{
  responseString = "";
  while (responseString.length() <= 0 || !responseString.endsWith("\n"))
  {
    tryToRead();

    if (millis() - beltTime > maxResponseTime)
    {
      return;
    }
  }
}
void tryToRead()
{
  while (sim_ss.available())
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
    Serial.println("GPS wiring error!");
    while (true);
  }
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);
  dt = (millis() - tiempo_prev) / 1000.0;
  tiempo_prev = millis();
  float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
  float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);
  ang_x = 0.98 * (ang_x_prev + (gx / 131) * dt) + 0.02 * accel_ang_x;
  ang_y = 0.98 * (ang_y_prev + (gy / 131) * dt) + 0.02 * accel_ang_y;
  ang_x_prev = ang_x;
  ang_y_prev = ang_y;
  gps_fix = gps.satellites.value();
  gps_hdop = gps.hdop.value();
}
void logInfo()
{
  uint8_t hr_val, min_val, sec_val;
  int Day_val, Month_val;
  long  Year_val, vel_val;
  int current_val;
  double alt_m_val;
  int rssi_val;
  int value = 0;
  int value2 = 0;
  int valueA1 = 0;
  int valueA2 = 0;
  long positionA1, positionA2;
  double lectura;
  double Vout;
  double battery_val;
  Day_val = gps.date.day();
  Month_val = gps.date.month();
  Year_val = gps.date.year();
  hr_val = gps.time.hour();
  min_val = gps.time.minute();
  sec_val = gps.time.second();
  alt_m_val = gps.altitude.meters();
  rssi_val = 0;
  current_val = 0;
  value = digitalRead(DIGI1);
  value2 = digitalRead(DIGI2);
  valueA1 = analogRead(AIN1);
  valueA2 = analogRead(AIN2);
  lectura = analogRead(A3);
  positionA1 = map(valueA1, 0, 1023, 0, 5);
  positionA2 = map(valueA2, 0, 1023, 0, 5);
  // Causes us to wait until we have satelite fix


  if (!gps.location.isValid())
  {
    Serial.println("Ubicación invalida. esperando por conexión satelital.");
    blinkLed(ErrorPin);
    return;
  }

  String latitud(gps.location.lat(), DEC);
  String longitud (gps.location.lng(), DEC);
  String alt_string(gps.altitude.meters(), DEC);
  double vel(gps.speed.kmph());
  String dig1state(value);
  String digistate2(value2);
  String rssi(rssi_val);
  String amps(current_val);
  String Hrs;
  String Mins;
  String Segs;
  String Month;
  String Day;
  if (millis() - previous > frequency)
  {
    Vout = map(lectura, 0, 1023, 0, 400);
    battery_val = Vout / 100.00;
    if (gps.time.hour() < 10) {
      Hrs = "0";
    } else {
      Hrs = "";
    }
    if (gps.time.minute() < 10) {
      Mins = "0";
    } else {
      Mins = "";
    }
    if (gps.time.second() < 10) {
      Segs = "0";
    } else {
      Segs = "";
    }
    if (gps.date.month() < 10) {
      Month = "0";
    } else {
      Month = "";
    }
    if (gps.date.day() < 10) {
      Day = "0";
    } else {
      Day = "";
    }
    sim_ss.listen();
    previous = millis();
    sim_ss.println("AT+CSQ"); // SIGNAL STRENGTH
    delay(500);
    if (sim_ss.find(": ")) { // decode reply
      while (sim_ss.available()) {
        n = sim_ss.read();
        if (n == ',') break;
        if (n != '\n') {
          sim_ss.write(n);
          r = n;
        }
      }
      delay(900);
    }
    String  url = "AT+HTTPPARA=\"URL\",\"";
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
    url += vel;
    url += ",";
    url += ang_y;
    url += ",";
    url += dig1state;
    url += ",";
    url += digistate2;
    url += ",";
    url += positionA1;
    url += ",";
    url += positionA2;
    url += ",";
    url += r;
    url += ",";
    url +=  battery_val;
    url += ",";
    url += amps;
    url += ",";
    url += gps_fix;
    url += ",";
    url += gps_hdop;
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
  while (sim_ss.available())
  {
    Serial.write(sim_ss.read());
  }
}
