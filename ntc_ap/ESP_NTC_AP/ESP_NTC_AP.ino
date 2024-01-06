 
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ElegantOTA.h>

#include "ESPFrontEnd.h"

const char* ssid = "hamids_iPad";

const byte DNS_PORT = 53;

IPAddress apIP(192, 168, 1, 1);

DNSServer dnsServer;

const int ldrPin=5;
const int touchPin=2                                                                                                                                                                     ;
bool LampOn=0;
const int gasSensorPin=A0;
const int buzzerPin= 4;


const int BlueLED = 12; // D0
const int RedLED = 16;    // D1 
const int GreenLED = 14; // D2

// Setting PWM frequency, channels and bit resolution
const int frequency = 5000;
const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;
const int resolution = 8;

String LightMode;

long int lastTimeGasRead;


const double VCC = 3.3;             // NodeMCU on board 3.3v vcc
const double R2 = 10000;            // 10k ohm series resistor
const double adc_resolution = 1023*3.3; // 10-bit adc

// const double A = 0.001129148;   // thermistor equation parameters
// const double B = 0.000234125;
// const double C = 0.0000000876741; 
const double A = 0.8272069482e-3;   // thermistor equation parameters
const double B = 2.087897328e-4;
const double C = 0.8062131944e-7; 
ESP8266WebServer webServer(80);



void ICACHE_RAM_ATTR TouchFunc()
{
  LightMode="Stable";
  if (LampOn){
  analogWrite(RedLED,0);
  analogWrite(GreenLED,0);
  analogWrite(BlueLED,0);
  LampOn=0;
  Serial.println("Turn off Leds");
//  delay(1000);
  return;
  }
  else
  {
  analogWrite(RedLED,1023);
  analogWrite(GreenLED,1023);
  analogWrite(BlueLED,1023);
  LampOn=1;
  Serial.println("Turn on Leds");
//  delay(1000);
  return;
  }
}

void ICACHE_RAM_ATTR TurnOnLeds()
{
  if (LampOn && !digitalRead(ldrPin)){
  analogWrite(RedLED,0);
  analogWrite(GreenLED,0);
  analogWrite(BlueLED,0);
  LampOn=0;
  Serial.println("Turn off Leds");
//  delay(1000);
  return;
  } 
  if(!LampOn && digitalRead(ldrPin)){
  analogWrite(RedLED,500);
  analogWrite(GreenLED,500);
  analogWrite(BlueLED,500);
  LampOn=1;
  Serial.println("Turn on Leds");
//  delay(1000);
  return;
  }
}


void handleForm(){
  LightMode="Stable";
  String red_pin = webServer.arg(0); 
  String green_pin = webServer.arg(1);
  String blue_pin = webServer.arg(2);

if((red_pin != "") && (green_pin != "") && (blue_pin != ""))
{ 
  analogWrite(RedLED,1023-red_pin.toInt());
  analogWrite(GreenLED,1023-green_pin.toInt());
  analogWrite(BlueLED,1023-blue_pin.toInt());
}

  webServer.send(302, "text/plain", "Updated-- Press Back Button");
}

void flicker(){
  int RedPWM=random(1024);
  int GreenPWM=random(1024);
  int BluePWM=random(1024);
  
  if (LightMode=="flicker"){
  analogWrite(RedLED,RedPWM);
  analogWrite(GreenLED,GreenPWM);
  analogWrite(BlueLED,BluePWM);
  }
  delay(500);
  if (LightMode=="flicker"){
  analogWrite(RedLED,0);
  analogWrite(GreenLED,0);
  analogWrite(BlueLED,0);
  
  delay(500);
  }
  
}

void breath(){
  
  int RedPWM=random(1024);
  int GreenPWM=random(1024);
  int BluePWM=random(1024);
  for (int i=200;i>0;i--){
    webServer.handleClient();
    if (LightMode=="breath"){
    analogWrite(RedLED,i*RedPWM/200);
    analogWrite(GreenLED,i*GreenPWM/200);
    analogWrite(BlueLED,i*BluePWM/200);
    delay(20);
  }
  else{
    break;
  }
  }

  for (int i=0;i<200;i++){
    webServer.handleClient();
    if (LightMode=="breath"){
    analogWrite(RedLED,i*RedPWM/200);
    analogWrite(GreenLED,i*GreenPWM/200);
    analogWrite(BlueLED,i*BluePWM/200); 
    delay(20); 
  }
  else{
    break;
  }
  }
}


void Rain(){
  int RedPWM=random(1024);
  int GreenPWM=random(1024-RedPWM);
  int BluePWM=1024-RedPWM-GreenPWM;
  analogWrite(RedLED,RedPWM);
  analogWrite(GreenLED,GreenPWM);
  analogWrite(BlueLED,BluePWM);
  delay(500);
}

void handleLightMode(){
  Serial.println("setting the light mode");
  LightMode=webServer.arg(0);
  Serial.println(LightMode);

  webServer.send(302, "text/plain", "Updated-- Press Back Button");
}

void handleRoot() 
{
  LightMode="Stable";
Serial.println(webServer.arg(0));
String red_pin = webServer.arg(0); 
String green_pin = webServer.arg(1);
String blue_pin = webServer.arg(2);

if((red_pin != "") && (green_pin != "") && (blue_pin != ""))
{ 
 
  analogWrite(RedLED,1023-red_pin.toInt());
  analogWrite(GreenLED,1023-green_pin.toInt());
  analogWrite(BlueLED,1023-blue_pin.toInt());
}

Serial.print("Red: ");
Serial.println(red_pin.toInt()); 
Serial.print("Green: ");
Serial.println(green_pin.toInt()); 
Serial.print("Blue: ");
Serial.println(blue_pin.toInt()); 
Serial.println("سلام");

  
webServer.send_P(200, "text/html;charset=utf-8", webpage0);
}

void handleADC() {
  double Vout, Rth, temperature, adc_value; 

  adc_value = analogRead(A0);
  Vout = (adc_value * VCC) / adc_resolution;
  Rth = (VCC * R2 / Vout) - R2;
  Serial.println(" adc value");
  Serial.println(adc_value);
  Serial.println("rth");
  Serial.println(Rth);

/*  Steinhart-Hart Thermistor Equation:
 *  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
 *  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin

  temperature = temperature - 273.15;  // Temperature in degree celsius
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" degree celsius");

 String adcValue = String(temperature);
 webServer.send(200, "text/plane", adcValue); //Send ADC value only to client ajax request
}

void setup() {
  Serial.begin(115200);

  attachInterrupt(digitalPinToInterrupt(ldrPin), TurnOnLeds, FALLING);
  attachInterrupt(digitalPinToInterrupt(touchPin),TouchFunc,RISING);

  pinMode(touchPin,INPUT);
  WiFi.disconnect();   //added to start with the wifi off, avoid crashing
  WiFi.mode(WIFI_OFF); //added to start with the wifi off, avoid crashing
  WiFi.mode(WIFI_AP);
  
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("Virada Light");

  dnsServer.start(DNS_PORT, "*", apIP);

    Serial.println(WiFi.softAPIP());

  pinMode(RedLED,OUTPUT);
  pinMode(GreenLED,OUTPUT);
  pinMode(BlueLED,OUTPUT);
  pinMode(ldrPin,INPUT);
  pinMode(buzzerPin,OUTPUT);
  
  delay(1000);
  Serial.println();
  const int resolution = 1024;
  analogWriteRange(resolution);  
  
  digitalWrite(buzzerPin,LOW);

  webServer.on("/", handleRoot);
  webServer.on("/setRGB",handleForm);
  webServer.on("/readADC", handleADC); //This page is called by java Script AJAX
  webServer.on("/setLightMode",handleLightMode);
//  ElegantOTA.begin(&webServer);
  
  webServer.onNotFound([]() {
  webServer.send_P(200, "text/html;charset=utf-8", webpage0);
  });


  ElegantOTA.begin(&webServer);
  webServer.begin();
  }

void loop() {
dnsServer.processNextRequest();
webServer.handleClient();

if (LightMode=="flicker"){
  flicker();
}

if (LightMode=="breath"){
  breath();
}

if(millis()-lastTimeGasRead>2000){
  double Vout, Rth, temperature, adc_value; 

  adc_value = analogRead(A0);
  Vout = (adc_value * VCC) / adc_resolution;
  Rth = (VCC * R2 / Vout) - R2;
  Serial.println(" adc value");
  Serial.println(adc_value);
  Serial.println("rth");
  Serial.println(Rth);

/*  Steinhart-Hart Thermistor Equation:
 *  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
 *  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  */
  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin

  temperature = temperature - 273.15;  // Temperature in degree celsius
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.println(" degree celsius");
  lastTimeGasRead=millis(); 
}

}
