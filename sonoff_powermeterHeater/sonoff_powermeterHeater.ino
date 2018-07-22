#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Ticker.h>

/*
 * Melyseg: 87mm  80.5 lyuktavolsag
 * Szelesseg: 92mm 85.5mm lyutavolsag
 * 
 * 
 * B- 9 pin
 * A+ 10 pin
 * 
 */

// POWER METER
#include <SDM.h>                                                                //import SDM template library
#define ASCII_ESC 27
#define BUFFER_SIZE 100
char bufout[10];
SDM<2400, D7, D5> sdm;        //TX =14  RX == 13 
char charBuf[50];
String s;  

#define BUTTON          0                                    // (Don't Change for Original Sonoff, Sonoff SV, Sonoff Touch, Sonoff S20 Socket)
#define RELAY           12                                   // (Don't Change for Original Sonoff, Sonoff SV, Sonoff Touch, Sonoff S20 Socket)
#define LED             13                                   // (Don't Change for Original Sonoff, Sonoff SV, Sonoff Touch, Sonoff S20 Socket)

#define MQTT_CLIENT     "Sonoff_pwrMeterHeater"       // mqtt client_id (Must be unique for each Sonoff)
#define MQTT_SERVER     "10.10.0.22"                      // mqtt server
#define MQTT_PORT       1883                                 // mqtt port
#define MQTT_TOPIC      "home/sonoff/pwrMeter/1"          // mqtt topic (Must be unique for each Sonoff)
#define MQTT_USER       ""                               // mqtt user
#define MQTT_PASS       ""         
#define furdo_motion_pin      D2// mqtt password
#define wc_motion_pin      D6

#define WIFI_SSID       "WhTrUnifi24"                           // wifi ssid
#define WIFI_PASS       "basH3Rfr"                           // wifi password

#define VERSION    "\n\n----------------  Sonoff Powerpoint v1.01pOTA  -----------------"

bool motion = false;
bool newMotion = false;
bool furdo_motion = false;
bool new_furdo_motion = false;
bool rememberRelayState = true;                              // If 'true' remembers the state of the relay before power loss.
bool OTAupdate = false;                                      // (Do not Change)
bool sendStatus = false;                                     // (Do not Change)
bool requestRestart = false;                                 // (Do not Change)

int kUpdFreq = 1;                                            // Update frequency in Mintes to check for mqtt connection
int kRetries = 10;                                           // WiFi retry count. Increase if not connecting to router.
int lastRelayState;       
bool LEDs = LOW;// (Do not Change)

unsigned long TTasks;                                        // (Do not Change)
unsigned long count = 0;                                     // (Do not Change)
unsigned long TTasks2;

extern "C" { 
  #include "user_interface.h" 
}

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient, MQTT_SERVER, MQTT_PORT);
Ticker btn_timer;

void callback(const MQTT::Publish& pub) {
  Serial.println(pub.payload_string());
  Serial.println(pub.topic());
  if (pub.topic() == "home/light/bathroom/1") {
      if (pub.payload_string() == "on") {
        digitalWrite(D1, HIGH);
    mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "on").set_retain().set_qos(1));

  }
  else if (pub.payload_string() == "off") {
        mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "off").set_retain().set_qos(1));
        digitalWrite(D1, LOW);
  }
  }

  if (pub.topic() == "home/light/wc/1") {
      if (pub.payload_string() == "on") {
        digitalWrite(D4, HIGH);
    mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "on").set_retain().set_qos(1));

  }
  else if (pub.payload_string() == "off") {
        mqttClient.publish(MQTT::Publish(pub.topic()+"/stat", "off").set_retain().set_qos(1));
        digitalWrite(D4, LOW);
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
  //pinMode(LED, OUTPUT);
  pinMode(wc_motion_pin, INPUT);
  pinMode(furdo_motion_pin, INPUT); 
  pinMode(D1, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(BUTTON, INPUT);
  Serial.begin(115200);
    sdm.begin();      
  btn_timer.attach(0.05, button);
  mqttClient.set_callback(callback);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.hostname("Sonoff-powermeter");
  
  ArduinoOTA.setHostname("Sonoff-powermeterOTA");
  ArduinoOTA.onStart([]() {
    OTAupdate = true;
    //blinkLED(LED, 400, 2);
    digitalWrite(LED, HIGH);
    Serial.println("OTA Update Initiated . . .");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Update Ended . . .s");
    ESP.restart();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //digitalWrite(LED, LOW);
    delay(5);
   // digitalWrite(LED, HIGH);
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
   // blinkLED(LED, 40, 2);
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
      mqttClient.subscribe("home/light/bathroom/1");
            mqttClient.subscribe("home/light/wc/1");
//      blinkLED(LED, 40, 8);

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

    Checkpwr();
}

void loop() {
  ArduinoOTA.handle();
  if (OTAupdate == false) { 
    mqttClient.loop();
    timedTasks();
    Checkdoor();
    //checkStatus();
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
      //digitalWrite(LED, !digitalRead(LED));
      LEDs = !LEDs;
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
    if(digitalRead(LEDs) == LOW)  {
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
    //blinkLED(LED, 400, 4);
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
  if ((millis() > TTasks2 + (kUpdFreq*20000)) || (millis() < TTasks2)) { 
    TTasks2 = millis();
    Checkpwr();
  }
 
}

void Checkpwr() {
  s = String(sdm.readVal(SDM120C_VOLTAGE));
    if (s != "nan") {
      s.toCharArray(charBuf, 50);
        mqttClient.publish(MQTT::Publish("sonoff/volt", charBuf).set_retain().set_qos(1));
    }
      delay(50);

  s = String(sdm.readVal(SDM120C_CURRENT));
    if (s != "nan") {
      s.toCharArray(charBuf, 50);
        mqttClient.publish(MQTT::Publish("sonoff/current", charBuf).set_retain().set_qos(1));
    }
      delay(50);
      
  s = String(sdm.readVal(SDM120C_POWER));
    if (s != "nan") {
      s.toCharArray(charBuf, 50);
        mqttClient.publish(MQTT::Publish("sonoff/power", charBuf).set_retain().set_qos(1));
    }
      delay(50);

  s = String(sdm.readVal(SDM120C_IMPORT_ACTIVE_ENERGY));
    if (s != "nan") {
      s.toCharArray(charBuf, 50);
        mqttClient.publish(MQTT::Publish("sonoff/importenergy", charBuf).set_retain().set_qos(1));
    }
      delay(50);

  s = String(sdm.readVal(SDM120C_EXPORT_ACTIVE_ENERGY));
    if (s != "nan") {
      s.toCharArray(charBuf, 50);
        mqttClient.publish(MQTT::Publish("sonoff/exportenergy", charBuf).set_retain().set_qos(1));
    }
      delay(50);

  s = String(sdm.readVal(SDM120C_TOTAL_ACTIVE_ENERGY));
    if (s != "nan") {
      s.toCharArray(charBuf, 50);
        mqttClient.publish(MQTT::Publish("sonoff/totalenergy", charBuf).set_retain().set_qos(1));
    }
      delay(50);
}

void Checkdoor(){
   newMotion = digitalRead(wc_motion_pin); 
   new_furdo_motion = digitalRead(furdo_motion_pin); 
 
 if (motion && !newMotion) { 
   motion = false; 
   Serial.println("No more motion"); 
            mqttClient.publish(MQTT::Publish("sensor/wc/motionsen", "Zárva").set_retain().set_qos(0));   
 //  sent = true; 
 } else if (!motion && newMotion) { 
   motion = true; 
   Serial.println("Motion detected"); 
            mqttClient.publish(MQTT::Publish("sensor/wc/motionsen", "Nyitva").set_retain().set_qos(0));   
//   sent = true; 
}

 if (furdo_motion && !new_furdo_motion) { 
   furdo_motion = false; 
   Serial.println("No more motion"); 
            mqttClient.publish(MQTT::Publish("sensor/bathroom/motionsen", "Zárva").set_retain().set_qos(0));   
 //  sent = true; 
 } else if (!furdo_motion && new_furdo_motion) { 
   furdo_motion = true; 
   Serial.println("Motion detected"); 
            mqttClient.publish(MQTT::Publish("sensor/bathroom/motionsen", "Nyitva").set_retain().set_qos(0));   
//   sent = true; 
}

       
  }
