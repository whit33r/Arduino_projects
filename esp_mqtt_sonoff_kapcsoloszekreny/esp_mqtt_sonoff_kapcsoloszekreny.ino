#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include "FastLED.h"

#define NUM_LEDS 1
#define DATA_PIN D8

CRGB leds[NUM_LEDS];
/*
 * Sárga +
 * sárga/feher -
 * data Zöld
 * 
 * 
 * led
 * 
 * 
 * 
 * barna +
 * barna feher -
 * zöld/feher data
 * 
 * 
 */

#define BUTTON          0                                    // (Don't Change for Original Sonoff, Sonoff SV, Sonoff Touch, Sonoff S20 Socket)
#define RELAY           D4                                   // (Don't Change for Original Sonoff, Sonoff SV, Sonoff Touch, Sonoff S20 Socket)
#define LED             D1                              // (Don't Change for Original Sonoff, Sonoff SV, Sonoff Touch, Sonoff S20 Socket)
#define DOOR_LOCK       D6
#define pin_door_status D2
#define LED_LOCK        D8
#define pin_motion      D7

#define MQTT_CLIENT     "Sonoff_heaterKapcsolosz"       // mqtt client_id (Must be unique for each Sonoff)
#define MQTT_SERVER     "10.10.0.22"                      // mqtt server
#define MQTT_PORT       1883                                 // mqtt port
#define MQTT_TOPIC      "home/sonoff/heater/1"          // mqtt topic (Must be unique for each Sonoff)
#define MQTT_USER       ""                               // mqtt user
#define MQTT_PASS       ""                               // mqtt password

#define WIFI_SSID       "WhTrUnifi24"                           // wifi ssid
#define WIFI_PASS       "basH3Rfr"                           // wifi password

#define VERSION    "\n\n----------------  Sonof Powerpoint v1.01pOTA  -----------------"

bool rememberRelayState = true;                              // If 'true' remembers the state of the relay before power loss.
bool OTAupdate = false;                                      // (Do not Change)
bool sendStatus = false;                                     // (Do not Change)
bool requestRestart = false;                                 // (Do not Change)

int kUpdFreq = 1;                                            // Update frequency in Mintes to check for mqtt connection
int kRetries = 10;                                           // WiFi retry count. Increase if not connecting to router.
int lastRelayState;                                          // (Do not Change)

unsigned long TTasks;                                        // (Do not Change)
unsigned long count = 0;  // (Do not Change)

bool doorOpen = false;
bool doorLock = false;
bool motion = false;
bool newDoorOpen = false;
bool newDoorLock = false;
bool newMotion = false;

extern "C" { 
  #include "user_interface.h" 
}

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient, MQTT_SERVER, MQTT_PORT);
Ticker btn_timer;
#define BUFFER_SIZE 100

void callback(const MQTT::Publish& pub) {
  Serial.println(pub.payload_string());
  Serial.println(pub.topic());
  if (pub.topic() == "home/sonoff/heater/1") {
      if (pub.payload_string() == "on") {
        digitalWrite(RELAY, HIGH);
    mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "on").set_retain().set_qos(1));

  }
  else if (pub.payload_string() == "off") {
        mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "off").set_retain().set_qos(1));
        digitalWrite(RELAY, LOW);
  }
  }
   if (pub.topic() == "home/sonoff/living_room/1") {
      if (pub.payload_string() == "on") {
        digitalWrite(D0, HIGH);
        
    mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "on").set_retain().set_qos(1));

  }
  else if (pub.payload_string() == "off") {
        mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "off").set_retain().set_qos(1));
        digitalWrite(D0, LOW);
  }
  }
  
 // sendStatus = true;

    Serial.print(" => ");
  if (pub.has_stream()) {
    uint8_t buf[BUFFER_SIZE];
    int read;
    while (read = pub.payload_stream()->read(buf, BUFFER_SIZE)) {
      Serial.write(buf, read);
    }
    pub.payload_stream()->stop();
    Serial.println("");
  } else
    Serial.println(pub.payload_string());
}

void setup() {
         FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  pinMode(LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
    pinMode(D0, OUTPUT);
  //pinMode(BUTTON, INPUT);
  digitalWrite(LED, HIGH);
  digitalWrite(RELAY, LOW);
    pinMode(pin_door_status, INPUT_PULLUP);
  pinMode(DOOR_LOCK, INPUT);
  pinMode(LED_LOCK, OUTPUT);
  pinMode(pin_motion, INPUT); 
delay(100);
  newDoorOpen = digitalRead(pin_door_status);
  newDoorLock = digitalRead(DOOR_LOCK);
  newMotion = digitalRead(pin_motion);

//INIT DS2812B
  leds[0] = CRGB::Black;
  FastLED.show();
  Serial.begin(115200);
  mqttClient.set_callback(callback);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.hostname("Sonoff-kapcsolosz");
  ArduinoOTA.setHostname("Sonoff-kapcsoloszOTA");
  ArduinoOTA.onStart([]() {
    OTAupdate = true;
    blinkLED(LED, 400, 2);
    digitalWrite(LED, HIGH);
    Serial.println("OTA Update Initiated . . .");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Ended . . .s");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    digitalWrite(LED, LOW);
    delay(5);
    digitalWrite(LED, HIGH);
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    blinkLED(LED, 40, 2);
    OTAupdate = false;
    Serial.printf("OTA Error [%u] ", error);
    if (error == OTA_AUTH_ERROR) Serial.println(". . . . . . . . . . . . . . . Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println(". . . . . . . . . . . . . . . Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println(". . . . . . . . . . . . . . . Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println(". . . . . . . . . . . . . . . Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println(". . . . . . . . . . . . . . . End Failed");
  });
  ArduinoOTA.begin();
  Serial.println(VERSION);
  Serial.print("\nUnit ID: ");
  Serial.print("esp8266-");
  Serial.print(ESP.getChipId(), HEX);
  Serial.print("\nConnecting to "); Serial.print(WIFI_SSID); Serial.print(" Wifi"); 
  while ((WiFi.status() != WL_CONNECTED) && kRetries --) {
    delay(500);
    Serial.print(" .");
  }
  if (WiFi.status() == WL_CONNECTED) {  
    Serial.println(" DONE");
    Serial.print("IP Address is: "); Serial.println(WiFi.localIP());
    Serial.print("Connecting to ");Serial.print(MQTT_SERVER);Serial.print(" Broker . .");
    delay(500);
    while (!mqttClient.connect(MQTT::Connect(MQTT_CLIENT).set_keepalive(90).set_auth(MQTT_USER, MQTT_PASS)) && kRetries --) {
      Serial.print(" .");
      delay(1000);
    }
    if(mqttClient.connected()) {
      Serial.println(" DONE");
      Serial.println("\n----------------------------  Logs  ----------------------------");
      Serial.println();
      mqttClient.subscribe("home/sonoff/living_room/1");
      mqttClient.subscribe("home/sonoff/heater/1");
      blinkLED(LED, 40, 8);
      if(digitalRead(RELAY) == HIGH)  {
        digitalWrite(LED, LOW);
      } else {
        digitalWrite(LED, HIGH);
      }

    }
    else {
      Serial.println(" FAILED!");
      Serial.println("\n----------------------------------------------------------------");
      Serial.println();
    }
  }
  else {
    Serial.println(" WiFi FAILED!");
    Serial.println("\n----------------------------------------------------------------");
    Serial.println();
  }
}

void loop() {
  ArduinoOTA.handle();
  if (OTAupdate == false) { 
    mqttClient.loop();
    timedTasks();
    //checkStatus();
    Checkdoor();
  }
}

void blinkLED(int pin, int duration, int n) {             
  for(int i=0; i<n; i++)  {  
    digitalWrite(pin, HIGH);        
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
}

void button() {
  if (!digitalRead(BUTTON)) {
    count++;
  } 
  else {
    if (count > 1 && count <= 40) {   
      digitalWrite(LED, !digitalRead(LED));
      digitalWrite(RELAY, !digitalRead(RELAY));
      sendStatus = true;
    } 
    else if (count >40){
      Serial.println("\n\nSonoff Rebooting . . . . . . . . Please Wait"); 
      requestRestart = true;
    } 
    count=0;
  }
}

void checkConnection() {
  if (WiFi.status() == WL_CONNECTED)  {
    if (mqttClient.connected()) {
      Serial.println("mqtt broker connection . . . . . . . . . . OK");
    } 
    else {
      Serial.println("mqtt broker connection . . . . . . . . . . LOST");
      requestRestart = true;
    }
  }
  else { 
    Serial.println("WiFi connection . . . . . . . . . . LOST");
    requestRestart = true;
  }
}

void checkStatus() {
  if (sendStatus) {
    if(digitalRead(LED) == LOW)  {
      if (rememberRelayState) {
        EEPROM.write(0, 1);
      }      
      mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/stat", "on").set_retain().set_qos(1));
      Serial.println("Relay . . . . . . . . . . . . . . . . . . ON");
    } else {
      if (rememberRelayState) {
        EEPROM.write(0, 0);
      }       
      mqttClient.publish(MQTT::Publish(MQTT_TOPIC"/stat", "off").set_retain().set_qos(1));
      Serial.println("Relay . . . . . . . . . . . . . . . . . . OFF");
    }
    if (rememberRelayState) {
      EEPROM.commit();
    }    
    sendStatus = false;
  }
  if (requestRestart) {
    blinkLED(LED, 400, 4);
    ESP.restart();
  }
}

void timedTasks() {
  if ((millis() > TTasks + (kUpdFreq*60000)) || (millis() < TTasks)) { 
    TTasks = millis();
    checkConnection();
      if (requestRestart) {
    blinkLED(LED, 400, 4);
    ESP.restart();
  }
  }
}

void Checkdoor(){
   newDoorOpen = digitalRead(pin_door_status);
   newDoorLock = digitalRead(DOOR_LOCK);
   newMotion = digitalRead(pin_motion); 
   
    if (doorOpen && !newDoorOpen) {
         doorOpen = false;
         Serial.println("Door closed");
         mqttClient.publish(MQTT::Publish("sensor/maindoor/door-status", "CLOSED").set_retain().set_qos(0));
         mqttClient.publish(MQTT::Publish("sensor/maindoor/motion", "Zárva").set_retain().set_qos(0));
         
         //sent = true;
       } else if (!doorOpen && newDoorOpen) {
         doorOpen = true;
         Serial.println("Door open");
//         client.publish("sensor/maindoor/motion", "Nyitva", true);
//         client.publish("sensor/maindoor/door-status", "Nyitva", true);
         mqttClient.publish(MQTT::Publish("sensor/maindoor/door-status", "OPEN").set_retain().set_qos(0));
         mqttClient.publish(MQTT::Publish("sensor/maindoor/motion", "Nyitva").set_retain().set_qos(0));         
         //sent = true;
       }

        if (doorLock && newDoorLock == HIGH) {
         doorLock = false;
         Serial.println("Lock closed");
  //leds[0] = CRGB::Blac;
  leds[0].red   = 1; //Green
  leds[0].green = 0;
  leds[0].blue  = 0;
  FastLED.show();
  delay(10);
  
         mqttClient.publish(MQTT::Publish("sensor/maindoor/door-status", "CLOSED").set_retain().set_qos(0));
         mqttClient.publish(MQTT::Publish("sensor/maindoor/lock", "Zárva").set_retain().set_qos(0));
         
         //sent = true;
       } else if (!doorLock && newDoorLock == LOW) {
         doorLock = true;
         Serial.println("Lock open");
  //leds[0] = CRGB::Red;
  leds[0].red   = 0;
  leds[0].green = 50; //RED
  leds[0].blue  = 0;
  FastLED.show();
//         client.publish("sensor/maindoor/motion", "Nyitva", true);
//         client.publish("sensor/maindoor/door-status", "Nyitva", true);
         mqttClient.publish(MQTT::Publish("sensor/maindoor/door-status", "OPEN").set_retain().set_qos(0));
         mqttClient.publish(MQTT::Publish("sensor/maindoor/lock", "Nyitva").set_retain().set_qos(0));         
         //sent = true;
       }

 
 if (motion && !newMotion) { 
   motion = false; 
   Serial.println("No more motion"); 
            mqttClient.publish(MQTT::Publish("sensor/maindoor/motionsen", "Zárva").set_retain().set_qos(0));   
 //  sent = true; 
 } else if (!motion && newMotion) { 
   motion = true; 
   Serial.println("Motion detected"); 
            mqttClient.publish(MQTT::Publish("sensor/maindoor/motionsen", "Nyitva").set_retain().set_qos(0));   
//   sent = true; 
}


       
  }
