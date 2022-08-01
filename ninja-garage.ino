// ninja-garage - an MQTT based garage door opener
// (c) 2022

#define DEBUG

#include <FS.h>
#include <LittleFS.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <user_interface.h>

//Pin Definitions
#define SENSOR_PIN D2
#define RELAY_PIN D1
#define LED_BUILTIN D4
#define CONFIG_PIN D6

//Magic Numbers
#define RELAY_PULSE 500
#define CONFIG_TIMEOUT 60
#define MQTT_QOS 1
#define UPDATE_INTERVAL 20000
#define WIFI_SSID_NAME "ConfigureME-GARAGE"
#define WIFI_CONFIG_PW "espconfig4me"

//Door States
#define DOOR_OPEN 0
#define DOOR_CLOSED 1
#define DOOR_OPENING 2
#define DOOR_CLOSING 3

#define CONFIG_PUSHED LOW

//Global Variables

int door_state = DOOR_CLOSED;
int door_new_state = DOOR_CLOSED;
boolean door_state_changed = false;
unsigned long door_state_change_time = 0;
bool shouldSaveConfig = false;
bool stopPortal = true;
unsigned long updateTime = 20000;
unsigned long statusChangeTime = 0;

//VARIABLES STORED IN FS

char mqtt_server[64];
uint16_t mqtt_port = 1883;
char mqtt_set_target_door[96];
char mqtt_get_target_door[96];
char mqtt_get_current_door[96];
char mdns_name[32] = "garagedoor";
int door_move_time = 16000;
int sensor_bounce_time = 1000;
char mqtt_payload_open[16];
char mqtt_payload_closed[16];
char mqtt_payload_opening[16];
char mqtt_payload_closing[16];
char mqtt_payload_toggle[16] = "on";

//WiFi Manager
WiFiManager wm;
//bool wm_nonblocking = true;
WiFiManagerParameter param_mqtt_server("mqtt_server", "MQTT Server", mqtt_server, 64);
WiFiManagerParameter param_mqtt_port("mqtt_port", "MQTT Port", "1883", 6);
WiFiManagerParameter param_mqtt_set_target_door("mqtt_set_target_door", "MQTT Set Target Door", mqtt_set_target_door, 96);
WiFiManagerParameter param_mqtt_get_target_door("mqtt_get_target_door", "MQTT Get Target Door", mqtt_get_target_door, 96);
WiFiManagerParameter param_mqtt_get_current_door("mqtt_get_current_door", "MQTT Get Current Door", mqtt_get_current_door, 96);
WiFiManagerParameter param_mdns_name("mdns_name", "MDNS Name", mdns_name, 32);
WiFiManagerParameter param_mqtt_payload_open("mqtt_payload_open", "Payload: Open", mqtt_payload_open, 16);
WiFiManagerParameter param_mqtt_payload_closed("mqtt_payload_closed", "Payload: Closed", mqtt_payload_closed, 16);
WiFiManagerParameter param_mqtt_payload_opening("mqtt_payload_opening", "Payload: Opening", mqtt_payload_opening, 16);
WiFiManagerParameter param_mqtt_payload_closing("mqtt_payload_closing", "Payload: Closing", mqtt_payload_closing, 16);
WiFiManagerParameter param_mqtt_payload_toggle("mqtt_payload_toggle", "Payload: Toggle", mqtt_payload_toggle, 16);
WiFiManagerParameter param_door_move_time("door_move_time", "Door Move Time (ms)", "15000", 16);
WiFiManagerParameter param_sensor_bounce_time("sensor_bounce_time", "Sensor Bounce Time (ms)", "1000", 16);

WiFiClient client;

Adafruit_MQTT_Client mqtt_client = Adafruit_MQTT_Client(&client, mqtt_server, mqtt_port, "","");

Adafruit_MQTT_Subscribe mqtts_set_target_door = Adafruit_MQTT_Subscribe(&mqtt_client, mqtt_set_target_door, MQTT_QOS_1);

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("\nStarting");
  #else
  Serial.setDebugOutput(false);
  #endif

  delay(3000);

  //get the reset reason
  rst_info *resetInfo;
  resetInfo = ESP.getResetInfoPtr();
  
  //Configure IO pins
  pinMode(SENSOR_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(CONFIG_PIN, INPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  bool needConf = false;

  if(resetInfo->reason == REASON_EXT_SYS_RST){
    //externally reset
    #ifdef DEBUG
    Serial.println("Externally reset");
    #endif
    //needConf = true;
  }

  //unsigned long configstart = millis();
  
  WiFi.mode(WIFI_STA);

  //Read the config file from LittleFS

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
          
          //Parse the JSON file and load into the variables
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
          //strcpy(mqtt_payload_toggle, json["mqtt_payload_toggle"]);
          
          #ifdef DEBUG
          Serial.println("Variables as read from flash/config:");
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
          Serial.println(mqtt_payload_toggle);
          #endif

        } else {
          //Failed to load config file, open the config menu
          #ifdef DEBUG
          Serial.println("failed to load json config");
          #endif
          needConf = true;
        }
      }

      
    }
    else{
      //no config file!
      #ifdef DEBUG
      Serial.println("No config file - but we can create it!");
      #endif
      needConf = true;
    }
  }
  else{
    //Internal flash failure - may need to wipe flash?
    #ifdef DEBUG
    Serial.println("failed to mount FS. RIP ESP8266");
    #endif
    needConf = true;
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

  //Load the variables into the config params

  param_mqtt_server.setValue(mqtt_server, 64);
  param_mqtt_port.setValue(mport, 6);
  param_mqtt_set_target_door.setValue(mqtt_set_target_door, 96);
  param_mqtt_get_target_door.setValue(mqtt_get_target_door, 96);
  param_mqtt_get_current_door.setValue(mqtt_get_current_door, 96);
  param_mdns_name.setValue(mdns_name, 32);
  param_door_move_time.setValue(dmt, 16);
  param_sensor_bounce_time.setValue(sbt, 16);
  param_mqtt_payload_open.setValue(mqtt_payload_open, 16);
  param_mqtt_payload_closed.setValue(mqtt_payload_closed, 16);
  param_mqtt_payload_opening.setValue(mqtt_payload_opening, 16);
  param_mqtt_payload_closing.setValue(mqtt_payload_closing, 16);
  param_mqtt_payload_toggle.setValue(mqtt_payload_toggle, 16);

  //Define all the config params

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
  wm.addParameter(&param_mqtt_payload_toggle);

  wm.setSaveParamsCallback(saveParamConfigCallback);

  std::vector<const char *> menu = {"wifi","info","param","sep","restart","exit"};
  wm.setMenu(menu);

  wm.setClass("invert");

  wm.setParamsPage(true);
  wm.setBreakAfterConfig(true);
  wm.setAPClientCheck(true);
  wm.setRemoveDuplicateAPs(true);
  wm.setHostname(mdns_name);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.setSaveParamsCallback(saveParamConfigCallback);

  //We default to showing the config pages but if there is a config failure, then we will increase the timeout
  if(needConf == true){
    wm.setConfigPortalTimeout(CONFIG_TIMEOUT * 3);
  }else{
    wm.setConfigPortalTimeout(CONFIG_TIMEOUT);
  }

  bool res;

  if(needConf){
    //show the config screen
    showConfPortal(); 
  }else{
    #ifdef DEBUG
    Serial.println("Autoconnecting");
    #endif
    res = wm.autoConnect(WIFI_SSID_NAME, WIFI_CONFIG_PW);
  
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
  Serial.print("MQTT server: ");
  Serial.println(mqtt_server);
  Serial.print("MQTT port: ");
  Serial.println(mqtt_port);
  Serial.print("SetTargetDoor: ");
  Serial.println(mqtt_set_target_door);
  Serial.print("GetTargetDoor: ");
  Serial.println(mqtt_get_target_door);
  Serial.print("GetCurrentDoor: ");
  Serial.println(mqtt_get_current_door);
  Serial.print("MDNS Name: ");
  Serial.println(mdns_name);
  Serial.print("Door Move Time: ");
  Serial.println(door_move_time);
  Serial.print("Payload Open: ");
  Serial.println(mqtt_payload_open);
  Serial.print("Payload closed: ");
  Serial.println(mqtt_payload_closed);
  Serial.print("Payload opening: ");
  Serial.println(mqtt_payload_opening);
  Serial.print("Payload closing: ");
  Serial.println(mqtt_payload_closing);
  Serial.print("Payload toggle: ");
  Serial.println(mqtt_payload_toggle);
  #endif

  //Setup MQTT client
  mqtt_client = Adafruit_MQTT_Client(&client, mqtt_server, mqtt_port);
  mqtts_set_target_door = Adafruit_MQTT_Subscribe(&mqtt_client, mqtt_set_target_door, MQTT_QOS_1);
  mqtts_set_target_door.setCallback(doorCMD);
  mqtt_client.subscribe(&mqtts_set_target_door);

  stopPortal = false;

}

void showConfPortal(){
  wm.startConfigPortal(WIFI_SSID_NAME, WIFI_CONFIG_PW);
  digitalWrite(LED_BUILTIN, LOW);
  delay(10000);
  ESP.restart();   
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
  json["mqtt_server"] = param_mqtt_server.getValue();
  json["mqtt_port"] = param_mqtt_port.getValue();
  json["mdns_name"] = param_mdns_name.getValue();
  json["mqtt_set_target_door"] = param_mqtt_set_target_door.getValue();
  json["mqtt_get_target_door"] = param_mqtt_get_target_door.getValue();
  json["mqtt_get_current_door"] = param_mqtt_get_current_door.getValue();
  json["door_move_time"] = param_door_move_time.getValue();
  json["sensor_bounce_time"] = param_sensor_bounce_time.getValue();
  json["mqtt_payload_open"] = param_mqtt_payload_open.getValue();
  json["mqtt_payload_closed"] = param_mqtt_payload_closed.getValue();
  json["mqtt_payload_opening"] = param_mqtt_payload_opening.getValue();
  json["mqtt_payload_closing"] = param_mqtt_payload_closing.getValue();
  json["mqtt_payload_toggle"] = param_mqtt_payload_toggle.getValue();

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
  strcpy(mqtt_payload_toggle, json["mqtt_payload_toggle"]);

  File configFile = LittleFS.open("/config.json", "w");
  if (!configFile) {
    #ifdef DEBUG
    Serial.println("failed to open config file for writing");
    #endif
  }

  serializeJson(json, Serial);
  serializeJson(json, configFile);
  configFile.close();
  //end save
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

  if(strcmp(data, mqtt_payload_open) == 0){
    #ifdef DEBUG
    Serial.print(mqtt_payload_open);
    Serial.println(" Door commanded to OPEN");
    #endif

    mqtt_client.publish(mqtt_get_target_door, mqtt_payload_open, MQTT_QOS);
    //Is the door already open or moving? If so, ignore this command.
    if(door_state == DOOR_OPEN || door_state == DOOR_OPENING || door_state == DOOR_CLOSING){
      //do nothing
      #ifdef DEBUG
      Serial.println("Door is already moving or open - ignoring open command!");
      #endif
    }else{
      door_state = DOOR_OPENING;
      pulseDoor();
      statusChangeTime = millis();
    }

  }

  if(strcmp(data, mqtt_payload_closed) == 0){
    #ifdef DEBUG
    Serial.print(mqtt_payload_closed);
    Serial.println(" Door commanded to CLOSE");
    #endif
    mqtt_client.publish(mqtt_get_target_door, mqtt_payload_closed, MQTT_QOS);
    door_state = DOOR_CLOSING;
    pulseDoor();
    publishState();
    statusChangeTime = millis();
  }

  if(strcmp(data, mqtt_payload_toggle) == 0){
    #ifdef DEBUG
    Serial.println("Door commanded to TOGGLE");
    #endif
    mqtt_client.publish(mqtt_get_target_door, data, MQTT_QOS);
    pulseDoor();
    statusChangeTime = millis();
    updateTime = millis();
  }
}

void pulseDoor(){
  digitalWrite(RELAY_PIN, HIGH);
  delay(RELAY_PULSE);
  digitalWrite(RELAY_PIN, LOW);
}

void loop(){
  connectMQTT();
  
  if(statusChangeTime + door_move_time < millis()){
    readDoor();
  }

  if(digitalRead(CONFIG_PIN) == CONFIG_PUSHED){
    #ifdef DEBUG
    Serial.println("Config button pressed!");
    #endif
    wm.setConfigPortalTimeout(CONFIG_TIMEOUT * 3);
    showConfPortal();
  }
  
  mqtt_client.processPackets(10000);

  if(! mqtt_client.ping()){
    mqtt_client.disconnect();
  }

  if(updateTime + UPDATE_INTERVAL < millis()){
    //time for an update
    #ifdef DEBUG
    Serial.print("Update: ");
    #endif
    publishState();
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
    }

    if(door_new_state != door_state && (sensor_bounce_time + door_state_change_time > millis()) && !door_state_changed ){
      //state trying to change 
      door_state = door_new_state;
      door_state_changed = true;
      updateTime = millis();
      #ifdef DEBUG
      Serial.print("StateChange: ");
      #endif
      publishState();
      }
    
}

void publishState(){

  if(door_state == DOOR_OPEN){
    mqtt_client.publish(mqtt_get_current_door, mqtt_payload_open, MQTT_QOS);
    #ifdef DEBUG
    Serial.println("Door is OPEN");
    #endif
  }

  if(door_state == DOOR_CLOSED){
    mqtt_client.publish(mqtt_get_current_door, mqtt_payload_closed, MQTT_QOS);
    #ifdef DEBUG
    Serial.println("Door is CLOSED");
    #endif
  }

    if(door_state == DOOR_OPENING){
    mqtt_client.publish(mqtt_get_current_door, mqtt_payload_opening, MQTT_QOS);
    #ifdef DEBUG
    Serial.println("Door is OPENING");
    #endif
  }

  if(door_state == DOOR_CLOSING){
    mqtt_client.publish(mqtt_get_current_door, mqtt_payload_closing, MQTT_QOS);
    #ifdef DEBUG
    Serial.println("Door is CLOSING");
    #endif
  }

}
