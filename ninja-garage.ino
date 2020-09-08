// ninja-garage - an MQTT based garage door opener
// (c) 2020

#include <FS.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <MQTT.h>
#include <ArduinoJson.h>

#define SENSOR_PIN D1
#define RELAY_PIN D2

#define RELAY_PULSE 500

#define CONFIG_TIMEOUT 180

#define DOOR_OPEN 0
#define DOOR_CLOSED 1
#define DOOR_OPENING 2
#define DOOR_CLOSING 3

const char* device_name = "NINJA-DOOR";

int door_state = DOOR_CLOSED;
int door_new_state = DOOR_CLOSED;

//VARIABLES STORED IN FS

char mqtt_server[40];
char t_dmt[10];
char t_sbt[10];

unsigned long door_state_change_time = 0;
int door_move_time = 15000;
int sensor_bounce_time = 2000;


bool shouldSaveConfig = false;

void saveConfigCallback(){
    Serial.println("Should save config!");
    shouldSaveConfig = true;
}

void setup(){
Serial.begin(115200);
    if(SPIFFS.begin()){
        Serial.println("mounted fs");
        if(SPIFFS.exists("/config.json")){
            Serial.println("reading config");
            File configFile = SPIFFS.open("/config.json", "r");
            if(configFile){
                Serial.println("opened config");
                size_t size = configFile.size();

                std::unique_ptr<char[]> buf(new char[size]);
                configFile.readBytes(buf.get(), size);
                DynamicJsonDocument json(1024);
                DeserializationError deserializeError = deserializeJson(json, buf.get());
                serializeJson(json, Serial);
                //DynamicJsonBuffer = jsonBuffer;
                //JsonObject& json = jsonBuffer.parseObject(buf.get());
                serializeJsonPretty(json, Serial);

                if (!deserializeError) {
                    Serial.println("\nparsed json");

                    strcpy(mqtt_server, json["mqtt_server"]);
                    //strcpy(t_dsct, json["dsct"]);
                    strcpy(t_dmt, json["dmt"]);
                    strcpy(t_sbt, json["sbt"]);
                }else{
                    Serial.println("failed to load config");
                }
                configFile.close();
            }
        }
    }else{
        Serial.println("failed to mount fs");
    }

    WiFiManagerParameter c_mqtt_server("server", "mqtt server", mqtt_server, 40);
    WiFiManagerParameter c_dmt("dmt", "time for door to move", t_dmt, 10);
    WiFiManagerParameter c_sbt("sbt", "Bounce time", t_sbt, 10);

    WiFiManager wifiManager;

    wifiManager.setSaveConfigCallback(saveConfigCallback);
    //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

    wifiManager.addParameter(&c_mqtt_server);
    wifiManager.addParameter(&c_dmt);
    wifiManager.addParameter(&c_sbt);

    wifiManager.setTimeout(300);

    if(!wifiManager.autoConnect(device_name)){
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        ESP.reset();
        delay(5000);
    }

    Serial.println("connected!");

    //read updated parameters

    strcpy(mqtt_server, c_mqtt_server.getValue());
    strcpy(t_dmt, c_dmt.getValue());
    strcpy(t_sbt, c_sbt.getValue());

    door_move_time = atoi(t_dmt);
    sensor_bounce_time = atoi(t_sbt);

    if(shouldSaveConfig){
        Serial.println("saving config");
        DynamicJsonDocument json(1024);
        json["mqtt_server"] = mqtt_server;
        json["dmt"] = t_dmt;
        json["sbt"] = t_sbt;

        File configFile = SPIFFS.open("/config.json", "w");
        if(!configFile){
            Serial.println("failed to open for writing");
        }

        serializeJsonPretty(json, Serial);
        serializeJson(json, configFile);
        configFile.close();
    }

}

void loop(){
    Serial.print("Door move time: ");
    Serial.println(door_move_time);
    Serial.print("Sensor bounce time: ");
    Serial.print(sensor_bounce_time);
}