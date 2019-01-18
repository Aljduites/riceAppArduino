#define DEVICE_ID 23
#define DEVICE_NAME "ard"
#define TOKEN "~6381_8s1QXa7hpTy*v"
#include <RemoteMe.h>
#include <RemoteMeWebSocketConnector.h>
#include <RemoteMeDirectWebSocketConnector.h>
#include <Servo.h> //January 16, 2019

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager

//for LED status
#include <Ticker.h>
Ticker ticker;

uint8_t LEDpin = D6;// pin 12
Servo myservo;

int ricevalveUpper = 2; //upper solenoid valve for rice, output
int ricevalveLower = 3; //lower solenoid valve for rice, output
int watervalveUpper = 4; //upper solenoid valve for water, output
int motor = 6; //motor for washing of rice, output
int drainValve = 7; //solenoid valve for draining of wastewater, output
int ricecookerValve = 8; //solenoid valve for opening from washing container to rice cooker pot, output
int ricePower = 9; //linear actuator for switching the rice cooker, output
int cookSwitch = 10; //linear actuator for switching the rice cooker, output

int watersense = A0; //water sensor, input
int waterState;
int32_t riceCup;
int32_t cupSetrice;
float sensorWater;
int start = 15; //SIGNAL FROM ONLINE SYSTEM TO START COOKING
int32_t buttoncupSet; //information from the online system as to number of cups selected

RemoteMe& remoteMe = RemoteMe::getInstance(TOKEN, DEVICE_ID);

//*************** CODE FOR CONFORTABLE VARIABLE SET *********************

inline void setServo(int32_t i) {remoteMe.getVariables()->setInteger("servo", i); }
inline void setRelay_01(boolean b) {remoteMe.getVariables()->setBoolean("relay_01", b); }
inline void setCookTime(int32_t i, boolean b) {remoteMe.getVariables()->setIntegerBoolean("cookTime", i, b); }
inline void setRiceCookerStatus(boolean b) {remoteMe.getVariables()->setBoolean("riceCookerStatus", b); }

//*************** IMPLEMENT FUNCTIONS BELOW *********************
void onServoChange(int32_t i) {
  myservo.write(i);
}

void onRelay_01Change(boolean b) {
  digitalWrite(LEDpin, b ? HIGH : LOW);

}
void onCookTimeChange(int32_t i, boolean b) {
        //setup rice cooker
    for(boolean machineStatus=false;machineStatus!=true;machineStatus=(HIGH==(digitalRead(start)))) {
      cupSetrice = digitalRead(buttoncupSet);
      delay(50);
    }
  // RICE DISPENSING PROCESS
  for(riceCup = 0; riceCup < cupSetrice; riceCup++) {
    digitalWrite(ricevalveUpper, HIGH);
    delay(2000);
    digitalWrite(ricevalveLower, HIGH);
    delay(2000);
    digitalWrite(ricevalveLower, LOW);
    delay(2000);
    digitalWrite(ricevalveUpper, LOW);
    delay(2000);
  }

  //WATER DISPENSING PROCESS FOR FIRST WASHING
  for(riceCup = 0; riceCup < cupSetrice; riceCup++) {
    for(waterState = LOW; waterState != HIGH; waterState) {
      digitalWrite(watervalveUpper, HIGH);
      delay(50);

      sensorWater = analogRead(watersense); //capacitive proximity sensor for water
      delay(50);
      if(sensorWater > 512) {
        waterState = HIGH;
        delay(50);        
      } else {
        waterState = LOW;
        delay(50);
      }
    }

    digitalWrite(watervalveUpper, LOW);
    delay(100);
    digitalWrite(watervalveLower, HIGH);
    delay(3000);
    digitalWrite(watervalveLower, LOW);
    delay(100);
  }

  //WASHING PROCESS (FIRST WASH AND THEN DRAIN)
  digitalWrite(motor, HIGH);
  delay(8000);
  digitalWrite(drainValve, HIGH);
  delay(10000);
  digitalWrite(motor, LOW);
  digitalWrite(drainValve, LOW);
  delay(100);

  //WATER DISPENSING PROCESS FOR SECOND WASHING
  for(riceCup = 0; riceCup < cupSetrice; riceCup++) {
    for(waterState = LOW;waterState != HIGH;waterState) {
      digitalWrite(watervalveUpper, HIGH);
      delay(50)
      sensorWater = analogRead(watersense); //capacitive proximity sensor for water
      delay(50);
      if(sensorWater > 512) {
        waterState = HIGH;
        delay(50);
      } else {
        waterState = LOW;
        delay(50);
      }
    }
    digitalWrite(watervalveUpper, LOW);
    delay(100);
    digitalWrite(watervalveLower, HIGH);
    delay(3000);
    digitalWrite(watervalveLower, LOW);
    delay(100);
  }

  //WASHING PROCESS AND DRAIN (SECOND WASH)
  digitalWrite(motor, HIGH);
  delay(8000);
  digitalWrite(drainValve, HIGH);
  delay(10000);
  digitalWrite(motor, LOW);
  digitalWrite(drainValve, LOW);
  delay(100);

  //FINAL STAGE: FROM WASHING CONTAINER TO RICE COOKER POT

  digitalWrite(ricecookerValve, HIGH);
  digitalWrite(ricePower, HIGH);
  delay(3000);

  for(riceCup = 0; riceCup < cupSetrice; riceCup++) {
    for(waterState = LOW;waterState != HIGH; waterState) {
      digitalWrite(watervalveUpper, HIGH);
      delay(50);
      sensorWater = analogRead(watersense);
      delay(50);
      if(sensorWater > 512) {
        waterState = HIGH;
        delay(50);        
      } else {
        waterState = LOW;
        delay(50);
      }
    }
    digitalWrite(watervalveUpper, LOW);
    delay(100);
    digitalWrite(watervalveLower, HIGH);
    delay(3000);
    digitalWrite(watervalveLower, LOW);
    delay(100);
  }

  digitalWrite(ricecookerValve, LOW);
  delay(10000);

  // PRESSES RICE COOKER SWITCH TO COOK
  digitalWrite(cookSwitch, HIGH);
  delay(2000);
  digitalWrite(cookSwitch, LOW);

  //after x-minutes depending on the rice cup value: delay(x);
  delay(20000);
  digitalWrite(ricePower, LOW);

  //after this the program will be on stand-by for another process
}

void onRiceCookerStatusChange(boolean b) {
}

void tick(){
  //toggle state
  int state = digitalRead(14);
  digitalWrite(14, !state);
}

//get called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Enter config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  //set led pin as output
  pinMode(14, OUTPUT);
  //start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);

  //WiFiManager
  //Local initialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();

  //set callback that gets called when connecting to previous WiFi fails, and enters Access point mode
  wifiManager.setAPCallback(configModeCallback);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here AutoConnectAP
  if(!wifiManager.autoConnect("RiceApp")) {
    Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  }
    //if you get here you have connected to the WiFi
    Serial.println("Connected!!!");
    ticker.detach();
    //WiFi Connected 
    // in pin 14 or D5
    digitalWrite(14, HIGH);
    
    remoteMe.getVariables()->observeInteger("servo" ,onServoChange);
    remoteMe.getVariables()->observeBoolean("relay_01" ,onRelay_01Change);
    remoteMe.getVariables()->observeIntegerBoolean("cookTime" ,onCookTimeChange);
    remoteMe.getVariables()->observeBoolean("riceCookerStatus" ,onRiceCookerStatusChange);
    
    remoteMe.setConnector(new RemoteMeWebSocketConnector());
    remoteMe.setDirectConnector(new RemoteMeDirectWebSocketConnector());

    remoteMe.sendRegisterDeviceMessage(DEVICE_NAME);
//    pinMode(2, OUTPUT);
//    pinMode(4, OUTPUT);
//    pinMode(5, OUTPUT);
//    pinMode(15,OUTPUT);

    myservo.attach(D7);//    pinMode(13, OUTPUT);    
    pinMode(0, OUTPUT);
    pinMode(LEDpin, OUTPUT); //pinMode(12, OUTPUT);
    pinMode(16, OUTPUT);
    digitalWrite(LEDpin, LOW);

    //fors rice cooker
        //pinMode(start, INPUT);
    pinMode(ricevalveUpper, OUTPUT);
    pinMode(ricevalveLower, OUTPUT);
    pinMode(watervalveUpper, OUTPUT);
    pinMode(watervalveLower, OUTPUT);
    pinMode(motor, OUTPUT);
    pinMode(drainValve, OUTPUT);
    pinMode(cookSwitch, OUTPUT);
    pinMode(ricecookerValve, OUTPUT);
    pinMode(ricePower, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
    remoteMe.loop();
}
