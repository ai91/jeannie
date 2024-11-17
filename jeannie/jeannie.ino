#include <LittleFS.h>

#include <ESP8266WiFi.h>

#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoOTA.h>
#include <Ticker.h>

#include <WiFiUdp.h>
#include <mDNSResolver.h>

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <PubSubClient.h>         //https://github.com/knolleary/pubsubclient

//#define DEBUG

#ifdef DEBUG
 #define WEBSERVER_H "fix confict"
 #include <ESPAsyncTCP.h>         //https://github.com/mathieucarbou/esphome-ESPAsyncTCP/releases/tag/v2.0.0
 #include <ESPAsyncWebServer.h>   //https://github.com/mathieucarbou/ESPAsyncWebServer/releases/tag/v3.1.1
 #include <WebSerial.h>           //https://github.com/ayushsharma82/WebSerial/releases/tag/v2.0.6
                                  // to check log messages: http://ipaddress:8080/webserial
 AsyncWebServer server(8080);
 #define DEBUG_PRINT(x) WebSerial.print(x)
 #define DEBUG_PRINTLN(x) WebSerial.println(x)
 #define DEBUG_PRINT2(x,y) WebSerial.print(x,y)
 #define DEBUG_PRINTLN2(x,y) WebSerial.println(x,y)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINT2(x,y)
 #define DEBUG_PRINTLN2(x,y)
#endif

#define CONFIG_TIMEOUT_MS 300000
#define OTA_TIMEOUT_MS 300000

#define LOOP_PERIOD_MS 100
#define CONFIG_FILE "/config.json"

// --------------------------------------------------
// MQTT config
char mqttServer[40] = "TiffanyAching.local";
char mqttPort[6] = "1883";
char mqttClientName[40];
char mqttUser[40] = "moist";
char mqttPassword[40] = "password";
char mqttTopicState[40] = "ibnhouse/ac/bedroom/state";
char mqttTopicCmd[40] = "ibnhouse/ac/bedroom/cmd";
char mqttTopicStatus[40] = "ibnhouse/ac/bedroom/status";

boolean offline = true;

WiFiClient espClient;
WiFiManager wifiManager;
WiFiManagerParameter customMqttServer("server", "mqtt server", mqttServer, 40);
WiFiManagerParameter customMqttPort("port", "mqtt port", mqttPort, 6);
WiFiManagerParameter customMqttClientName("client", "mqtt client name", mqttClientName, 16);
WiFiManagerParameter customMqttUser("user", "mqtt user", mqttUser, 16);
WiFiManagerParameter customMqttPassword("password", "mqtt password", mqttPassword, 16);
WiFiManagerParameter customMqttTopicState("state", "state topic", mqttTopicState, 40);
WiFiManagerParameter customMqttTopicCmd("cmd", "cmd topic", mqttTopicCmd, 40);
WiFiManagerParameter customMqttTopicStatus("status", "liveness topic", mqttTopicStatus, 40);
bool wifiManagerSetupRunning = false;
unsigned long wifiManagerSetupStart;
bool otaRunning = false;
unsigned long otaStart;
bool restart = false;

PubSubClient mqttClient(espClient);
unsigned long mqttConnectAttempt = 0;
unsigned long mqttConnectDelay = 0;
bool mqttWasNeverOnline = true;
WiFiUDP udp;
mDNSResolver::Resolver mDnsResolver(udp);
IPAddress mqttServerIp = INADDR_NONE;

// commands
#define CMD_SETUP "set"
#define CMD_OTA "ota"
#define CMD_RESET "rst"
#define CMD_STATE "sta"

#define CMD_POWER_ON "power_on"
#define CMD_POWER_OFF "power_off"
#define CMD_MODE_COOL "mode_cool"
#define CMD_MODE_HEAT "mode_heat"
#define CMD_MODE_DRY "mode_dry"
#define CMD_MODE_FAN "mode_fan"
#define CMD_MODE_AUTO "mode_auto"
#define CMD_SPEED_AUTO "speed_auto"
#define CMD_SPEED_1 "speed_1"
#define CMD_SPEED_2 "speed_2"
#define CMD_SPEED_3 "speed_3"
#define CMD_SPEED_4 "speed_4"
#define CMD_SPEED_5 "speed_5"
#define CMD_TEMP_PREFIX "temp_"
#define CMD_TEMP_UPPREFIX "temp+"
#define CMD_TEMP_DOWNPREFIX "temp-"
#define CMD_SWING_ON "swing_on"
#define CMD_SWING_OFF "swing_off"
#define CMD_SWING_H_ON "swing_h_on"
#define CMD_SWING_H_OFF "swing_h_off"
#define CMD_SWING_V_ON "swing_v_on"
#define CMD_SWING_V_OFF "swing_v_off"
#define CMD_ENERGYSAVE_ON "evergysave_on"
#define CMD_ENERGYSAVE_OFF "evergysave_off"
#define CMD_BOOST_ON "boost_on"
#define CMD_BOOST_OFF "boost_off"
#define CMD_QUIET_ON "quiet_on"
#define CMD_QUIET_OFF "quiet_off"
#define CMD_SLEEP_OFF "sleep_off"
#define CMD_SLEEP_1 "sleep_1"
#define CMD_SLEEP_2 "sleep_2"
#define CMD_SLEEP_3 "sleep_3"
#define CMD_SLEEP_4 "sleep_4"
#define CMD_MUTE_ON "mute_on"
#define CMD_MUTE_OFF "mute_off"
#define CMD_CELSIUS_ON "celsius_on"
#define CMD_CELSIUS_OFF "celsius_off"

char mqttMsg[255];

// --------------------------------------------------
// LEDS
#define LED 16

#define LED_MODE_IGNORE -1
#define LED_MODE_OFF 0x00
#define LED_MODE_START 0x01
#define LED_MODE_SETUP 0x02
#define LED_MODE_ERROR 0x04
#define LED_MODE_WORK 0x08
#define LED_MODE_OTA 0x10
#define LED_MODE_SINGLE_TICK 0x11

Ticker ledsTicker;
int ledsMode;
int ledState = LOW;

// -------------------------------------------------
// current state
bool stateMuted = false;
byte statePower = 0;
byte stateSwingH = 0;
byte stateSwingV = 0;
byte stateTargetTemp = 0;
byte stateCurrentTemp = 0;
byte statePipeTemp = 0;
byte stateSettingsCelcius = 0;
byte stateSpeed = 0;
byte stateMode = 0;

long lastStateHash = 0;
#define STATE_READ_PERIOD_MS 60000
unsigned long lastStateReadMs = -1;
const int BUFFER_SIZE = 256;
byte buffer[BUFFER_SIZE];
byte CMD_STATUS[] = {0xF4, 0xF5, 0x00, 0x40, 0x0C, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x01, 0xB3, 0xF4, 0xFB};

byte CMD_TEMPLATE[] = {0xF4, 0xF5, 0x00, 0x40, 0x29, 0x00, 0x00, 0x01, 0x01, 0xFE, 0x01, 0x00, 0x00, 0x65, 0x00, 0x00, 
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0xF4, 0xFB};


void setup() {

  Serial.begin(9600);
  //Serial.setTimeout(500);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED, OUTPUT);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

#ifdef DEBUG
  WebSerial.begin(&server);
  server.begin();
#endif

  setLedsMode(LED_MODE_WORK);

  wifiManager.setAPCallback(configModeCallback);
  startWifiManager(false);

  // MQTT connection
  mqttClient.setCallback(mqttCallback);

  // OTA progress
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (digitalRead(LED) == HIGH) {
      digitalWrite(LED, LOW);
    } else {
      digitalWrite(LED, HIGH);
    }
  });

}

void setLedsMode(int mode){
  
  if (ledsMode != mode) {
    ledsMode = mode;
    switch(ledsMode) {
      case LED_MODE_OFF:
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED, HIGH);
        ledsTicker.detach();
        break;
      case LED_MODE_START:
        digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED, LOW);
        ledsTicker.detach();
        break;
      case LED_MODE_SETUP:
        ledsTicker.attach_ms(1000, ledsTick);
        break;
      case LED_MODE_OTA:
        ledsTicker.attach_ms(300, ledsTick);
        break;
      case LED_MODE_ERROR:
        ledsTicker.attach_ms(3000, ledsTick);
        break;
      case LED_MODE_WORK:
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED, HIGH);
        ledsTicker.detach();
        break;
      case LED_MODE_SINGLE_TICK:
        //digitalWrite(LED_BUILTIN, LOW);
        digitalWrite(LED, LOW);
        ledsTicker.attach_ms(300, ledsTick);
        break;
    }
  }
  
}

void ledsTick()
{
  int nLedState = ledState;
  ledState = ledState == HIGH ? LOW : HIGH;
  switch(ledsMode) {
    case LED_MODE_START:
    case LED_MODE_SETUP:
    case LED_MODE_OTA:
      digitalWrite(LED_BUILTIN, nLedState);
      digitalWrite(LED, nLedState);
      break;
    case LED_MODE_ERROR:
      digitalWrite(LED_BUILTIN, nLedState);
      digitalWrite(LED, nLedState);
      break;
    case LED_MODE_SINGLE_TICK:
      digitalWrite(LED_BUILTIN, HIGH);
      digitalWrite(LED, HIGH);
      ledsTicker.detach();
      ledsMode = LED_MODE_OFF;
      break;
  }
}

void configModeCallback (WiFiManager *myWiFiManager) {
  setLedsMode(LED_MODE_SETUP);
}

void loop() {

  if (otaRunning) {

    ArduinoOTA.handle();

    if ((millis() - otaStart) > OTA_TIMEOUT_MS) {
      restart = true;
    }

  } else {

  #ifdef DEBUG
    WebSerial.loop();
  #endif

    wifimanagerLoop();

    if (offline) {
      setLedsMode(LED_MODE_ERROR);
    } else {
      mqttLoop();
      if (mqttWasNeverOnline && mqttConnectDelay >= 5000) {
        startWifiManager(true);
      }
    }

    // update state once a minute
    unsigned long currentTimeMs=millis();
    if(currentTimeMs < lastStateReadMs || (currentTimeMs - lastStateReadMs > STATE_READ_PERIOD_MS)) {
      requestState();
      lastStateReadMs = currentTimeMs;
    } else {
      readState(false);
    }

    delay(LOOP_PERIOD_MS);
  }

  if (restart) {
    setLedsMode(LED_MODE_OFF);
    ESP.restart();
  }

}

void wifimanagerLoop() {

  wifiManager.process();
  if (wifiManagerSetupRunning) {
    if ((millis() - wifiManagerSetupStart) > CONFIG_TIMEOUT_MS) {
      wifiManager.stopConfigPortal();
      wifiManagerSetupStopped();
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (offline) {
      mDnsResolver.setLocalIP(WiFi.localIP());
      mqttServerIp = mDnsResolver.search(mqttServer);
      // MQTT connection
      if (mqttServerIp != INADDR_NONE) {
        mqttClient.setServer(mqttServerIp, atoi(mqttPort));
      } else {
        mqttClient.setServer(mqttServer, atoi(mqttPort));
      }
    }
    offline = false;
  } else {
    offline = true;
  }

  if (!offline) {
    mDnsResolver.loop();
  }

}

//callback notifying us of the need to save config
void saveParamsCallback () {

  //save the custom parameters to FS
  //read updated parameters
  strcpy(mqttServer, customMqttServer.getValue());
  strcpy(mqttPort, customMqttPort.getValue());
  strcpy(mqttClientName, customMqttClientName.getValue());
  strcpy(mqttUser, customMqttUser.getValue());
  strcpy(mqttPassword, customMqttPassword.getValue());
  strcpy(mqttTopicState, customMqttTopicState.getValue());
  strcpy(mqttTopicCmd, customMqttTopicCmd.getValue());
  strcpy(mqttTopicStatus, customMqttTopicStatus.getValue());

  JsonDocument json;
  json["mqtt_server"] = mqttServer;
  json["mqtt_port"] = mqttPort;
  json["mqtt_client_name"] = mqttClientName;
  json["mqtt_user"] = mqttUser;
  json["mqtt_password"] = mqttPassword;
  json["mqtt_state"] = mqttTopicState;
  json["mqtt_cmd"] = mqttTopicCmd;
  json["mqtt_status"] = mqttTopicStatus;

  File configFile = LittleFS.open(CONFIG_FILE, "w");
  serializeJson(json, configFile);
  configFile.close();
  //end save

  wifiManagerSetupStopped();
}

void wifiManagerSetupStarted(WiFiManager *myWiFiManager) {
  setLedsMode(LED_MODE_SETUP);
  wifiManagerSetupRunning = true;
  wifiManagerSetupStart = millis();
}

void wifiManagerSetupStopped() {
  restart = true; // don't restart immediately. let WifiManager finish handleWifiSave() execution
}

void startWifiManager(boolean onDemand) {

  if (wifiManagerSetupRunning) {
    return;
  }

  if (!onDemand) {

    String apName = "Jeannie-" + String(ESP.getChipId(), HEX);
    strcpy(mqttClientName, apName.c_str());

    if (LittleFS.begin()) {
      if (LittleFS.exists(CONFIG_FILE)) {
        //file exists, reading and loading
        File configFile = LittleFS.open(CONFIG_FILE, "r");
        if (configFile) {
          size_t size = configFile.size();
          // Allocate a buffer to store contents of the file.
          std::unique_ptr<char[]> buf(new char[size]);

          configFile.readBytes(buf.get(), size);
          JsonDocument json;
          DeserializationError jsonError = deserializeJson(json, buf.get());
          if (!jsonError) {
            if (json.containsKey("mqtt_server") && strlen(json["mqtt_server"]) > 0) strcpy(mqttServer, json["mqtt_server"]);
            if (json.containsKey("mqtt_port") && strlen(json["mqtt_port"]) > 0) strcpy(mqttPort, json["mqtt_port"]);
            if (json.containsKey("mqtt_client_name") && strlen(json["mqtt_client_name"]) > 0) strcpy(mqttClientName, json["mqtt_client_name"]);
            if (json.containsKey("mqtt_user") && strlen(json["mqtt_user"]) > 0) strcpy(mqttUser, json["mqtt_user"]);
            if (json.containsKey("mqtt_password") && strlen(json["mqtt_password"]) > 0) strcpy(mqttPassword, json["mqtt_password"]);
            if (json.containsKey("mqtt_state") && strlen(json["mqtt_state"]) > 0) strcpy(mqttTopicState, json["mqtt_state"]);
            if (json.containsKey("mqtt_cmd") && strlen(json["mqtt_cmd"]) > 0) strcpy(mqttTopicCmd, json["mqtt_cmd"]);
            if (json.containsKey("mqtt_status") && strlen(json["mqtt_status"]) > 0) strcpy(mqttTopicStatus, json["mqtt_status"]);
          }
        }
      }
    }
    //end read

    WiFi.hostname(mqttClientName);
    ArduinoOTA.setHostname(mqttClientName);

    customMqttServer.setValue(mqttServer, 40);
    customMqttPort.setValue(mqttPort, 6);
    customMqttClientName.setValue(mqttClientName, 40);
    customMqttUser.setValue(mqttUser, 40);
    customMqttPassword.setValue(mqttPassword, 40);
    customMqttTopicState.setValue(mqttTopicState, 40);
    customMqttTopicCmd.setValue(mqttTopicCmd, 40);
    customMqttTopicStatus.setValue(mqttTopicStatus, 40);

    wifiManager.setSaveParamsCallback(saveParamsCallback);
    wifiManager.setAPCallback(wifiManagerSetupStarted);

    wifiManager.setConfigPortalTimeout(CONFIG_TIMEOUT_MS / 1000);
    wifiManager.setConfigPortalBlocking(false);

    //add all your parameters here
    wifiManager.addParameter(&customMqttServer);
    wifiManager.addParameter(&customMqttPort);
    wifiManager.addParameter(&customMqttClientName);
    wifiManager.addParameter(&customMqttUser);
    wifiManager.addParameter(&customMqttPassword);
    wifiManager.addParameter(&customMqttTopicState);
    wifiManager.addParameter(&customMqttTopicCmd);
    wifiManager.addParameter(&customMqttTopicStatus);
  }

  if (onDemand) {
    wifiManager.startConfigPortal(mqttClientName);
  } else {
    wifiManager.autoConnect(mqttClientName);
  }

}

void cmdPower(bool enable) {
  sendCommand(enable ? 0x0C : 0x04, 0x12, false);
}

void cmdSetTemperature(byte temp) {
  byte tempVal = temp * 2 + 1;
  sendCommand(tempVal, 0x13, false);
}

void cmdFanSpeed(byte speed) {
  byte speedVal = 01;
  switch(speed) {
    case 1:
      speedVal = 0x0B;
      break;
    case 2:
      speedVal = 0x0D;
      break;
    case 3:
      speedVal = 0x0F;
      break;
    case 4:
      speedVal = 0x11;
      break;
    case 5:
      speedVal = 0x13;
      break;
  }
  sendCommand(speedVal, 0x10, true);
}

void cmdModeCool() {
  sendCommand(0x50, 0x12, true);
}

void cmdModeHeat() {
  sendCommand(0x30, 0x12, true);
}

void cmdModeDry() {
  sendCommand(0x70, 0x12, true);
}

void cmdModeFan() {
  sendCommand(0x10, 0x12, true);
}

void cmdModeAuto() {
  sendCommand(0x90, 0x12, true);
}

void cmdSwing(bool enable) {
  sendCommand(enable ? 0xF0 : 0x50, 0x20, false);
}

void cmdSwingH(bool enable) {
  byte swingH = enable ? 0x30 : 0x10;
  byte swingV = stateSwingV ? 0xC0 : 0x40;
  sendCommand(swingH | swingV, 0x20, false);
}

void cmdSwingV(bool enable) {
  byte swingH = stateSwingH ? 0x30 : 0x10;
  byte swingV = enable ? 0xC0 : 0x40;
  sendCommand(swingH | swingV, 0x20, false);
}

void cmdEnergySaving(bool enable) {
  sendCommand(enable ? 0x30 : 0x10, 0x21, false);
}

void cmdBoost(bool enable) {
  sendCommand(enable ? 0x0C : 0x04, 0x21, false);
}

void cmdQuiet(bool enable) {
  sendCommand(enable ? 0x30 : 0x10, 0x23, false);
}

void cmdSleep(byte mode) {
  byte modeVal = 01;
  switch(mode) {
    case 1:
      modeVal = 0x03;
      break;
    case 2:
      modeVal = 0x05;
      break;
    case 3:
      modeVal = 0x07;
      break;
    case 4:
      modeVal = 0x09;
      break;
  }
  sendCommand(modeVal, 0x11, false);
}

void cmdSettings(bool celsius) {
  sendCommand(celsius ? 0x01 : 0x03, 0x17, false);
}


void sendCommand(byte cmdVal, byte cmdPos, bool autoTurnOn) {
  byte cmd[sizeof(CMD_TEMPLATE)];
  memcpy(cmd, CMD_TEMPLATE, sizeof(CMD_TEMPLATE));

  prepareCommand(cmdVal, cmdPos, cmd);
  if (statePower == 0 && autoTurnOn) {
    prepareCommand(0x0C, 0x12, cmd);
  }

#ifdef DEBUG
    DEBUG_PRINTLN("Sending command: ");
    DEBUG_PRINTLN("-----------------------------------------------");
    WebSerial.flush();
    for (int i = 0; i < sizeof(cmd); i++) {
      if(cmd[i]<=0xf){
        DEBUG_PRINT("0");
      }
      DEBUG_PRINT2(cmd[i], HEX);
      DEBUG_PRINT(" ");
      if((i+1)%16 == 0) {
        DEBUG_PRINTLN();
        // yield();
        WebSerial.loop();
        yield();
      }
    }
    DEBUG_PRINTLN();
    DEBUG_PRINTLN("===============================================");
#endif

  Serial.write(cmd, sizeof(cmd));
}

void prepareCommand(byte cmdVal, byte cmdPos, byte* command) {

  command[cmdPos] = cmdVal;

  if (!stateMuted) {
    command[0x17] = command[0x17] | 0x4;
  }
  
  uint16_t checksum = calculateChecksum(command, sizeof(CMD_TEMPLATE));
  command[46] = (checksum >> 8) & 0xFF;
  command[47] = checksum & 0xFF;
}

uint16_t calculateChecksum(uint8_t* data, size_t length) {
  uint16_t checksum = 0;
  for (size_t i = 2; i < length-4; i++) {
    checksum += data[i];
  }
  return checksum;
}

byte getSwingV(byte* data) {
//  byte swing = (data[0x12] >> 1) & 0x1;
  byte swing = (data[0x23] >> 7) & 0x1;
  return swing;
}

byte getSwingH(byte* data) {
  byte swing = (data[0x23] >> 6) & 0x1;
  return swing;
}

byte getPower(byte* data) {
  byte power = (data[0x12] >> 3) & 0x1;
  return power;
}

 byte getMode(byte* data) {
  byte mode = data[0x12] >> 4;
  return mode;
}

char const * getModeStr(byte mode) {
  switch(mode) {
    case 0x0:
      return "fan";
    case 0x1:
      return "heat";
    case 0x2:
      return "cool";
    case 0x3:
      return "dry";
    case 0x7:
      return "auto";
  }
  return "";
}

byte getTargetTemp(byte* data) {
  byte temp = data[0x13];
  return temp;
}

byte getSpeed(byte* data) {
  byte speed = data[0x10];
  switch(speed) {
    case 0x0A:
      return 1;
    case 0x0C:
      return 2;
    case 0x0E:
      return 3;
    case 0x10:
      return 4;
    case 0x13:
      return 5;
    default:
      return 0;
  }
}

byte getCurrentTemp(byte* data) {
  byte temp = data[0x14];
  return temp;
}

byte getPipeTemp(byte* data) {
  byte temp = data[0x15];
  return temp;
}

byte getSettingsCelcius(byte* data) {
  byte farenheit = (data[0x1A] >> 1) & 0x1;
  return farenheit == 1 ? 0 : 1;
}

void requestState() {
  DEBUG_PRINTLN("Requesting state");
  Serial.write(CMD_STATUS, sizeof(CMD_STATUS));
  readState(true);
}

void readState(bool forceSend) {

  if (Serial.available() > 0 || forceSend) {
    int count = Serial.readBytes(buffer, BUFFER_SIZE);

    uint16_t calculatedCheckSum = 0;
    uint16_t receivedCheckSum = 0;
    if (count >= 130) {
      calculatedCheckSum = calculateChecksum(buffer, count);
      receivedCheckSum = (buffer[count-4] << 8) | buffer[count-3];
      if (calculatedCheckSum == receivedCheckSum) {
        statePower = getPower(buffer);
        stateSwingV = getSwingV(buffer);
        stateSwingH = getSwingH(buffer);
        stateMode = getMode(buffer);
        stateTargetTemp = getTargetTemp(buffer);
        stateCurrentTemp = getCurrentTemp(buffer);
        statePipeTemp = getPipeTemp(buffer);
        stateSettingsCelcius = getSettingsCelcius(buffer);
        stateSpeed = getSpeed(buffer);

        long stateHash = statePower ^ (stateSwingH << 1) ^ (stateSwingV << 2) ^ (stateSettingsCelcius << 3) ^ (stateMuted << 4) ^ (stateMode << 5) ^ (stateTargetTemp << 12) ^ (stateSpeed << 20);
        if ((stateHash != lastStateHash) || forceSend) {
          lastStateHash = stateHash;
          JsonDocument json;
          json["power"] = statePower;
          json["swing_v"] = stateSwingV;
          json["swing_h"] = stateSwingH;
          json["mode"] = getModeStr(stateMode);
          json["targetTemp"] = stateTargetTemp;
          json["currentTemp"] = stateCurrentTemp;
          json["pipeTemp"] = statePipeTemp;
          json["speed"] = stateSpeed;
          json["settingsCelcius"] = stateSettingsCelcius;
          json["mute"] = stateMuted ? 1 : 0;

          serializeJson(json, mqttMsg, sizeof(mqttMsg));
        
          mqttClient.publish(mqttTopicState, mqttMsg, false);
        }

      } else {
        DEBUG_PRINT("Received wrong packet ");
        DEBUG_PRINT(count);
        DEBUG_PRINT(" bytes. Calculated checksum: ");
        DEBUG_PRINT2(calculatedCheckSum, HEX);
        DEBUG_PRINT(" Received checksum: ");
        DEBUG_PRINTLN2(receivedCheckSum, HEX);
      }
    }

#ifdef DEBUG
    if (count > 0) {
      //if (count != 21 && count != 28 /*&& count != 150*/) {
      if (calculatedCheckSum != receivedCheckSum) {
        DEBUG_PRINT(count);
        DEBUG_PRINTLN(" bytes");
        DEBUG_PRINTLN("-----------------------------------------------");
        WebSerial.flush();
        for (int i = 0; i < count; i++) {
          if(buffer[i]<=0xf){
            DEBUG_PRINT("0");
          }
          DEBUG_PRINT2(buffer[i], HEX);
          DEBUG_PRINT(" ");
          if((i+1)%16 == 0) {
            DEBUG_PRINTLN();
            // yield();
            WebSerial.loop();
            yield();
          }
        }
        DEBUG_PRINTLN();
        DEBUG_PRINTLN("===============================================");
      }
    }
#endif
  }
}

void mqttLoop() {
  if (!mqttClient.connected()) {
    if (!mqttReconnect()) {
      return;
    }
  }
  mqttClient.loop();
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  setLedsMode(LED_MODE_SINGLE_TICK);
  DEBUG_PRINT("Message retrieved [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("]: ");
  for (int i = 0; i < length; i++) {
    DEBUG_PRINT((char)payload[i]);
  }
  DEBUG_PRINTLN();

  char payloadCopy[10];
  strncpy(payloadCopy, (char*)payload, length);
  payloadCopy[length] = 0x0;

  if (String(topic).equalsIgnoreCase(mqttTopicCmd)) {
    String cmd = String(payloadCopy);
    if (cmd.startsWith(CMD_SETUP)) {
      startWifiManager(true);
    } else if (cmd.startsWith(CMD_RESET)) {
      restart = true;
    } else if (cmd.startsWith(CMD_STATE)) {
      requestState();
    } else if (cmd.startsWith(CMD_OTA)) {
      otaStart = millis();
      otaRunning = true;
      setLedsMode(LED_MODE_OTA); // start fast blinking
      ArduinoOTA.begin();
    } else if (cmd.startsWith(CMD_POWER_ON)) {
      cmdPower(true);
    } else if (cmd.startsWith(CMD_POWER_OFF)) {
      cmdPower(false);
    } else if (cmd.startsWith(CMD_MODE_COOL)) {
      cmdModeCool();
    } else if (cmd.startsWith(CMD_MODE_HEAT)) {
      cmdModeHeat();
    } else if (cmd.startsWith(CMD_MODE_DRY)) {
      cmdModeDry();
    } else if (cmd.startsWith(CMD_MODE_FAN)) {
      cmdModeFan();
    } else if (cmd.startsWith(CMD_MODE_AUTO)) {
      cmdModeAuto();
    } else if (cmd.startsWith(CMD_SPEED_AUTO)) {
      cmdFanSpeed(0);
    } else if (cmd.startsWith(CMD_SPEED_1)) {
      cmdFanSpeed(1);
    } else if (cmd.startsWith(CMD_SPEED_2)) {
      cmdFanSpeed(2);
    } else if (cmd.startsWith(CMD_SPEED_3)) {
      cmdFanSpeed(3);
    } else if (cmd.startsWith(CMD_SPEED_4)) {
      cmdFanSpeed(4);
    } else if (cmd.startsWith(CMD_SPEED_5)) {
      cmdFanSpeed(5);
    } else if (cmd.startsWith(CMD_TEMP_PREFIX)) {
      byte temp = cmd.substring(strlen(CMD_TEMP_PREFIX)).toInt();
      cmdSetTemperature(temp);
    } else if (cmd.startsWith(CMD_TEMP_UPPREFIX)) {
      byte temp = stateTargetTemp + cmd.substring(strlen(CMD_TEMP_UPPREFIX)).toInt();
      cmdSetTemperature(temp);
    } else if (cmd.startsWith(CMD_TEMP_DOWNPREFIX)) {
      byte temp = stateTargetTemp - cmd.substring(strlen(CMD_TEMP_DOWNPREFIX)).toInt();
      cmdSetTemperature(temp);
    } else if (cmd.startsWith(CMD_SWING_ON)) {
      cmdSwing(true);
    } else if (cmd.startsWith(CMD_SWING_OFF)) {
      cmdSwing(false);
    } else if (cmd.startsWith(CMD_SWING_H_ON)) {
      cmdSwingH(true);
    } else if (cmd.startsWith(CMD_SWING_H_OFF)) {
      cmdSwingH(false);
    } else if (cmd.startsWith(CMD_SWING_V_ON)) {
      cmdSwingV(true);
    } else if (cmd.startsWith(CMD_SWING_V_OFF)) {
      cmdSwingV(false);
    } else if (cmd.startsWith(CMD_ENERGYSAVE_ON)) {
      cmdEnergySaving(true);
    } else if (cmd.startsWith(CMD_ENERGYSAVE_OFF)) {
      cmdEnergySaving(false);
    } else if (cmd.startsWith(CMD_BOOST_ON)) {
      cmdBoost(true);
    } else if (cmd.startsWith(CMD_BOOST_OFF)) {
      cmdBoost(false);
    } else if (cmd.startsWith(CMD_QUIET_ON)) {
      cmdQuiet(true);
    } else if (cmd.startsWith(CMD_QUIET_OFF)) {
      cmdQuiet(false);
    } else if (cmd.startsWith(CMD_SLEEP_OFF)) {
      cmdSleep(0);
    } else if (cmd.startsWith(CMD_SLEEP_1)) {
      cmdSleep(1);
    } else if (cmd.startsWith(CMD_SLEEP_2)) {
      cmdSleep(2);
    } else if (cmd.startsWith(CMD_SLEEP_3)) {
      cmdSleep(3);
    } else if (cmd.startsWith(CMD_SLEEP_4)) {
      cmdSleep(4);
    } else if (cmd.startsWith(CMD_MUTE_ON)) {
      stateMuted = true;
      requestState();
    } else if (cmd.startsWith(CMD_MUTE_OFF)) {
      stateMuted = false;
      requestState();
    } else if (cmd.startsWith(CMD_CELSIUS_ON)) {
      cmdSettings(true);
    } else if (cmd.startsWith(CMD_CELSIUS_OFF)) {
      cmdSettings(false);
    }
  }
}

boolean mqttReconnect() {
  if (!mqttClient.connected()) {
    if (millis() - mqttConnectAttempt > mqttConnectDelay ) { // don't attempt more often than a delay
      mqttConnectDelay += 1000; // increase reconnect attempt delay by 1 second
      if (mqttConnectDelay > 60000) { // don't attempt more frequently than once a minute
        mqttConnectDelay = 60000;
      }
      DEBUG_PRINT("Attempting MQTT connection...");
      // Attempt to connect
      mqttConnectAttempt = millis();
      if (mqttClient.connect(mqttClientName, mqttUser, mqttPassword, mqttTopicStatus, 1, true, "offline")) {
        DEBUG_PRINTLN("connected");
        setLedsMode(LED_MODE_WORK);
        mqttClient.publish(mqttTopicStatus, "online", true);
        // Once connected resubscribe
        mqttClient.subscribe(mqttTopicCmd);
        mqttConnectDelay = 0;
        mqttWasNeverOnline = false;
        return true;
      } else {
        DEBUG_PRINT("failed, rc=");
        DEBUG_PRINTLN(mqttClient.state());
        return false;
      }
    }
  }
  return false;
}
