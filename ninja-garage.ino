// ninja-garage - an MQTT based garage door opener
// (c) 2020

//#define ESP8266
#define DEBUG

#include <FS.h>
#include <LittleFS.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

//Pin Definitions
#define SENSOR_PIN 19
#define RELAY_PIN 20
#define LED_BUILTIN 17

//Magic Numbers
#define RELAY_PULSE 500
#define CONFIG_TIMEOUT 180
#define MQTT_QOS 1

//Door States
#define DOOR_OPEN 1
#define DOOR_CLOSED 0
#define DOOR_OPENING 2
#define DOOR_CLOSING 3

//Global Variables

int door_state = DOOR_CLOSED;
int door_new_state = DOOR_CLOSED;
boolean door_state_changed = false;
unsigned long door_state_change_time = 0;
bool shouldSaveConfig = false;

//VARIABLES STORED IN FS

char mqtt_server[64];
uint16_t mqtt_port = 1883;
char mqtt_set_target_door[96];
char mqtt_get_target_door[96];
char mqtt_get_current_door[96];
char mdns_name[32];
int door_move_time = 15000;
int sensor_bounce_time = 1000;
char mqtt_payload_open[16];
char mqtt_payload_closed[16];
char mqtt_payload_opening[16];
char mqtt_payload_closing[16];

//WiFi Manager
WiFiManager wm;
//bool wm_nonblocking = true;
WiFiManagerParameter param_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 64);
WiFiManagerParameter param_mqtt_port("mqtt_port", "MQTT Port", "1883", 6);
WiFiManagerParameter param_mqtt_set_target_door("mqtt_set_target_door", "MQTT Set Target Door", mqtt_set_target_door, 96);
WiFiManagerParameter param_mqtt_get_target_door("mqtt_get_target_door", "MQTT Get Target Door", mqtt_get_target_door, 96);
WiFiManagerParameter param_mqtt_get_current_door("mqtt_get_current_door", "MQTT Get Current Door", mqtt_get_current_door, 96);
WiFiManagerParameter param_mdns_name("mdns_name", "MDNS Name", mdns_name, 32);
WiFiManagerParameter param_mqtt_payload_open("mqtt_payload", "Payload: Open", mqtt_payload_open, 16);
WiFiManagerParameter param_mqtt_payload_closed("mqtt_payload", "Payload: Closed", mqtt_payload_closed, 16);
WiFiManagerParameter param_mqtt_payload_opening("mqtt_payload", "Payload: Opening", mqtt_payload_opening, 16);
WiFiManagerParameter param_mqtt_payload_closing("mqtt_payload", "Payload: Closing", mqtt_payload_closing, 16);
WiFiManagerParameter param_door_move_time("door_move_time", "Door Move Time (ms)", "15000", 16);
WiFiManagerParameter param_sensor_bounce_time("sensor_bounce", "Sensor Bounce Time (ms)", "1000", 16);

WiFiClient client;

Adafruit_MQTT_Client mqtt_client;

Adafruit_MQTT_Subscribe mqtts_set_target_door;


void saveConfigCallback(){
    Serial.println("Should save config!");
    shouldSaveConfig = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(3000);
  Serial.println("\nStarting");

  pinMode(SENSOR_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  bool needConf = false;

  unsigned long configstart = millis();

  Serial.println("Looking for button press to enter config mode:");
  
  WiFi.mode(WIFI_STA);
  
  if(LittleFS.begin()){
    if(LittleFS.open("/config.json", "r")){
      File configFile = LittleFS.open("/config.json", "r");
      if(configFile){
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError ) {
          #ifdef DEBUG
          Serial.println("\nparsed json");
          #endif
          
          strcpy(mqtt_server, json["mqtt_server"]);
          mqtt_port = atoi(json["mqtt_port"]);
          strcpy(mqtt_set_target_door, json["mqtt_set_target_door"]);
          strcpy(mqtt_get_target_door, json["mqtt_get_target_door"]);
          strcpy(mqtt_get_current_door, json["mqtt_get_current_door"]);
          
          strcpy(mdns_name, json["mdns_name"]);

          door_move_time = atoi(json["door_move_time"]);
          sensor_bounce_time = atoi(json["sensor_bounce_time"]);

          strcpy(mqtt_payload_open, json["mqtt_payload_open"]);
          strcpy(mqtt_payload_closed, json["mqtt_payload_closed"]);
          strcpy(mqtt_payload_opening, json["mqtt_payload_opening"]);
          strcpy(mqtt_payload_closing, json["mqtt_payload_closing"]);
          
          #ifdef DEBUG
          Serial.println("Variables as read from flash:");
          Serial.println(mqtt_server);
          Serial.println(mqtt_port);
          Serial.println(mqtt_set_target_door);
          Serial.println(mqtt_get_target_door);
          Serial.println(mqtt_get_current_door);
          Serial.println(door_move_time);
          Serial.println(mdns_name);
          Serial.println(door_move_time);
          Serial.println(sensor_bounce_time);
          Serial.println(mqtt_payload_open);
          Serial.println(mqtt_payload_closed);
          Serial.println(mqtt_payload_opening);
          Serial.println(mqtt_payload_closing);
          #endif

        } else {
          Serial.println("failed to load json config");
          needConf = true;
        }
      }

      
    }
    else{
      //no config file!
      Serial.println("No config file - but we can create it!");
      needConf = true;
    }
  }
  else{
    Serial.println("failed to mount FS. RIP ESP8266");
  }

  wm.setCountry("US");
  wm.setConfigPortalBlocking(true);

  //Convert ints to strings
  char mport[8];
  itoa(mqtt_port, mport, 10);

  char dmt[8];
  itoa(door_move_time, dmt, 10);

  char sbt[8];
  itoa(sensor_bounce_time, sbt, 10);

  wm.addParameter(&param_mqtt_server);
  wm.addParameter(&param_mqtt_port);
  wm.addParameter(&param_mqtt_set_target_door);
  wm.addParameter(&param_mqtt_get_target_door);
  wm.addParameter(&param_mqtt_get_current_door);
  wm.addParameter(&param_mdns_name);
  wm.addParameter(&param_door_move_time);
  wm.addParameter(&param_sensor_bounce_time);
  wm.addParameter(&param_mqtt_payload_open);
  wm.addParameter(&param_mqtt_payload_closed);
  wm.addParameter(&param_mqtt_payload_opening);
  wm.addParameter(&param_mqtt_payload_closing);

  param_mqtt_server.setValue(mqtt_server, 64);
  param_mqtt_port.setValue(mport, 6);
  param_mqtt_set_target_door.setValue(mqtt_set_target_door, 96);
  param_mqtt_get_target_door.setValue(mqtt_get_target_door, 96);
  param_mqtt_get_current_door.setValue(mqtt_get_current_door, 96);
  param_mdns_name.setValue(mdns_name, 32);
  param_door_move_time.setValue(dmt, 16);
  param_sensor_bounce_time.setValue(sbt, 16);

  wm.setSaveParamsCallback(saveParamConfigCallback);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  wm.setClass("invert");

  wm.setParamsPage(true);
  wm.setConfigPortalTimeout(240);
  wm.setBreakAfterConfig(true);
  wm.setAPClientCheck(true);
  wm.setRemoveDuplicateAPs(true);
  wm.setHostname(mdns_name);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setSaveParamsCallback(saveParamConfigCallback);

  bool res;

  if(needConf){
    //show the config screen
    wm.startConfigPortal("ConfigureME", "espconfig4me");
    digitalWrite(LED_BUILTIN, LOW);
    delay(10000);
    ESP.restart();    
  }else{
    #ifdef DEBUG
    Serial.println("Autoconnecting");
    #endif
    res = wm.autoConnect("ConfgureME", "espconfig4me");
  
    if(!res) {
      #ifdef DEBUG
      Serial.println("Failed to connect or hit timeout");
      #endif
      ESP.restart();
    }else{
      #ifdef DEBUG
      Serial.println("Connected!");
      #endif
      
    }
  }

  #ifdef DEBUG
  Serial.println("Variables in memory:");
  Serial.print("Door move time: ");
  Serial.println(door_move_time);
  Serial.print("Sensor bounce time: ");
  Serial.println(sensor_bounce_time);
  Serial.println(mqtt_server);
  Serial.println(mqtt_port);
  Serial.println(mqtt_set_target_door);
  Serial.println(mqtt_get_target_door);
  Serial.println(mqtt_get_current_door);
  Serial.println(mdns_name);
  Serial.println(door_move_time);
  Serial.println(mqtt_payload_open);
  Serial.println(mqtt_payload_closed);
  Serial.println(mqtt_payload_opening);
  Serial.println(mqtt_payload_closing);
  #endif

  //connect MQTT client
  mqtt_client = Adafruit_MQTT_Client(&client, mqtt_server, mqtt_port, "","");
  mqtts_set_target_door = Adafruit_MQTT_Subscribe(&mqtt_client, mqtt_set_target_door, MQTT_QOS_1);
  mqtts_set_target_door.setCallback(doorCMD);
}


String getParam(String name){
  String value;
  if(wm.server->hasArg(name)) {
    value = wm.server->arg(name);
  }
  return value;
}

void saveConfigCallback () {
  saveParams();
}

void saveParamConfigCallback () {
  saveParams();
  wm.stopConfigPortal();
}

void saveParams(){
  //Save config to flash
  #ifdef DEBUG
  Serial.println("saving config to flash");
  #endif
  DynamicJsonDocument json(1024);
  json["mqtt_server"] = getParam("mqtt_server");
  json["mqtt_port"] = getParam("mqtt_port");
  json["mdns_name"] = getParam("mdns_name");
  json["mqtt_set_target_door"] = getParam("mqtt_set_target_door");
  json["mqtt_get_target_door"] = getParam("mqtt_get_target_door");
  json["mqtt_get_current_door"] = getParam("mqtt_get_current_door");
  json["door_move_time"] = getParam("door_move_time");
  json["sensor_bounce_time"] = getParam("sensor_bounce_time");
  json["mqtt_payload_open"] = getParam("mqtt_payload_open");
  json["mqtt_payload_closed"] = getParam("mqtt_payload_closed");
  json["mqtt_payload_opening"] = getParam("mqtt_payload_opening");
  json["mqtt_payload_closing"] = getParam("mqtt_payload_closed");

  //Save config to running variables
  strcpy(mqtt_server, json["mqtt_server"]);
  mqtt_port = atoi(json["mqtt_port"]);
  strcpy(mdns_name, json["mdns_name"]);
  door_move_time = atoi(json["door_move_time"]);
  sensor_bounce_time = atoi(json["sensor_bounce_time"]);
  strcpy(mqtt_set_target_door, json["mqtt_set_target_door"]);
  strcpy(mqtt_get_target_door, json["mqtt_get_target_door"]);
  strcpy(mqtt_get_current_door, json["mqtt_get_current_door"]);
  strcpy(mqtt_payload_open, json["mqtt_payload_open"]);
  strcpy(mqtt_payload_closed, json["mqtt_payload_closed"]);
  strcpy(mqtt_payload_opening, json["mqtt_payload_opening"]);
  strcpy(mqtt_payload_closing, json["mqtt_payload_closing"]);

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    #ifdef DEBUG
    Serial.println("failed to open config file for writing");
    #endif
  }

  serializeJson(json, Serial);
  serializeJson(json, configFile);
  configFile.close();
  //end save -
}

void connectMQTT(){
  if(mqtt_client.connected()){
    return;
  }

  int8_t ret;
  uint8_t retries = 3;

  while(mqtt_client.connect() != 0){
    #ifdef DEBUG
    Serial.print("MQTT Connection error: ");
    Serial.println(mqtt_client.connectErrorString(ret));
    #endif
    mqtt_client.disconnect();
    delay(10000);
    retries--;
    if(retries == 0){
      while(1);
    }
  }
  #ifdef DEBUG
  Serial.println("MQTT Connected!");
  #endif
}

void doorCMD(char *data, uint16_t len){
  //Process a door command recieved via MQTT subscription
  #ifdef DEBUG
  Serial.print("Recieved MQTT CMD:");
  Serial.println(data);
  #endif
}

void loop(){
  connectMQTT();
  readDoor();
  mqtt_client.processPackets(10000);

  if(! mqtt_client.ping()){
    mqtt_client.disconnect();
  }
}

void readDoor(){
    int door = digitalRead(SENSOR_PIN);

    if(door != door_new_state){
      #ifdef DEBUG
      Serial.print("Door state changed, now ");
      Serial.println(door);
      #endif
      door_new_state = door;
      door_state_change_time = millis();
      door_state_changed = false; //clear the flag
    }else{
      if(door_new_state != door_state && (sensor_bounce_time + door_state_change_time > millis()) && !door_state_changed ){
        //state trying to change 
        door_state = door_new_state;
        door_state_changed = true;
          
        //publish the state change
        publishState();
        }
    }
}

void publishState(){
  door_state;

  if(door_state == DOOR_OPEN){
    mqtt_client.publish(mqtt_get_current_door, mqtt_payload_open, MQTT_QOS);
  }

  if(door_state == DOOR_CLOSED){
    mqtt_client.publish(mqtt_get_current_door, mqtt_payload_closed, MQTT_QOS);
  }

}
