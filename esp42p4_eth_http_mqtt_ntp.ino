
#define PROD 1 //  REMEMBER to change to 1 for prod deploy

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <RadioLib.h>
//#include <Wire.h>
#include "Secret.h"
#include "esp32-hal-cpu.h"
#include "time.h"

// enable/disable feature at compile
#define LAN 1  // 0 to disable, 1 to enable
#define WLAN 0 // 0 to disable, 1 to enable
#define LORA 1 
#define MQTTenable 1 // set to 0 for debug
#define NTP 1
#define NEOPIXEL 0
#define LOG_SERIAL 0
#define LOG_WEB 1
#define LOG_BOTH 2
#define LOG_MODE LOG_WEB // define log mode


#define BUFLEN 64
#define ERRBUFLEN 265
#define LOG_LINES 500
#define LOG_LINE_LEN 200
char logBuffer[LOG_LINES][LOG_LINE_LEN];
int logHead = 0;

#if PROD == 1
bool debugMode = false;  // global
#else
bool debugMode = true;  // global
#endif
#define logDbg(fmt, ...) if (debugMode) logMsg(fmt, ##__VA_ARGS__)

const char VERSION[] = "v3.2.6";
char lastError[ERRBUFLEN] = {0};  // initialize empty

#ifndef ETH_PHY_MDC
  #define ETH_PHY_TYPE  ETH_PHY_IP101  // IP101GRI chip
  #define ETH_PHY_ADDR  0
  #define ETH_PHY_MDC   31
  #define ETH_PHY_MDIO  52
  #define ETH_PHY_POWER 51
  #define ETH_CLK_MODE  EMAC_CLK_EXT_IN
#endif

//lora
#define LORA_CS    17
#define LORA_CLK   14
#define LORA_MOSI  15
#define LORA_MISO  16
#define LORA_RST   18
#define LORA_BUSY  19
const uint8_t loraSF = 11;

//GND
#define LORA_RXEN  21
#define LORA_TXEN  22
//#define LORA_DIO2  21
#define LORA_DIO1  20
// 3V

Module mod(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
SX1262 radio(&mod);
SemaphoreHandle_t loraMutex = xSemaphoreCreateMutex();
volatile bool loraRcvFlag = false;
unsigned long lastLoraRcv;
#define LORA_MAX_SILENCE (10 * 60 * 1000)  // 10 minutes

// Set web server port number to 80
WebServer server(80);

#if LAN == 1
  NetworkClient network;
#elif WLAN == 1
  WiFiClient network;
#endif  

static bool eth_connected = false;
static bool lora_initialized = false;
IPAddress ip;

MQTTClient mqtt = MQTTClient(256);
int mqttPubInt = 30 * 1000;
unsigned long lastMqttAttempt = 0;
const unsigned long MQTT_RETRY_INTERVAL = 2000; // 2 seconds

#if PROD == 1
  const char mqttTopic[] = "iot/power/h6"; 
#else
  const char mqttTopic[] = "iot/power/h6test"; 
#endif
const char mqttCmd[] = "cmd";
char mqttCmdTopic[BUFLEN];

unsigned long mqttPublishTime = 0;
struct tm mqttTimeInfo;
char mqttLastPublishDate[40];
int cntMReCon = 0;
int cntMDisCon = 0;
int cntMPub = 0;
int cntMPubErr = 0;
int cntMRcv = 0;
int cntWifiReConn = 0;
int cntBadBme = 0;
int cntBme = 0;
int cntConsBadBme = 0;
int cntLora = 0;
int cntLoraOk = 0;
int cntLoraErr = 0;
int cntLoraInv = 0;
int cntLoraChecksum = 0;
int cntLoraSnd = 0;
int cntLoraReset = 0;

const int initTemp = 15.0;

String header;
#if PROD == 0
  float bmeTemp = 22.0;
  float bmeHum = 50;
  float bmePres = 1000;
  float bmeGasRes = 0;
#else
  float bmeTemp = 1;
  float bmeHum = 30;
  float bmePres = 900;
  float bmeGasRes = 0;
#endif
float deltaBuf5 = 0.0;
float deltaBuf2 = 0.0;
float deltaWW = 0.0;

float rawBmeTemp, rawBmeHum, rawBmePres, rawBmeGasRes;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

String bootTimeStr;  // Save formatted boot time string

// task handles
TaskHandle_t TPostMqtt;
TaskHandle_t TGetNtpTime;
TaskHandle_t TWatchDog;
#if LORA == 1
  TaskHandle_t TWatchDogLora;
  TaskHandle_t TLoraReceiveTask;
#endif
TaskHandle_t TMonLED;
#if WLAN == 1
  TaskHandle_t TWifiCheckReconn;
#endif  
TaskHandle_t TLoopMqtt;

// h6 lora
char currMsg[BUFLEN * 3 + 1];
char lastGoodMsg[BUFLEN * 3 + 1];
int loraLastRcvErr = 0;
int loraLastSndErr = 0;

// structure for the lora messages,
struct ShellyPWEN {
  uint32_t energy;    // Wh
  int16_t power;      // W (can be negative)
  uint32_t pvEnergy;  // Wh
  int16_t pvPower;    // W (can be negative)
};

struct ShellyENEXP {
    uint32_t exportEnergy;
};

struct ShellyHEALTH {
    uint32_t uptime;
    int8_t   tempC;
    int8_t   wifiRssi;
    uint8_t  wifiStatus;

};

struct LoraStatus {
  float_t rssi;
  float_t snr;
  int cntCSerr;
};

struct HeatTemp {
  float tempBuf5;
  float tempBuf2;
  float tempWW;
};

HeatTemp bufTemp;
ShellyPWEN h6PwEn;
ShellyPWEN h6PwEn_prev;
ShellyENEXP h6EnExp;
ShellyHEALTH h6Health;
bool h6PwEn_valid = false;
bool h6EnExp_valid  = false;
bool h6Health_valid = false;
LoraStatus loraStatus;
LoraStatus loraStatus_prev;

enum class LoraTel : uint8_t {
    // for now use hamming distance >=2 since we do not use a checksum ehre
    H6_PWEN         = 0x11,  // periodic power and energy data
    H6_ENEXP        = 0x22,  // energy export data
    H6_HEALTH       = 0x33,  // health status data, wlan, ha
    GET_H6_ENEXP    = 0xaa,  // request energy export data
    GET_H6_HEALTH   = 0xbb,  // request health data
    SET_ESP_DBGOFF  = 0xf0,  // disable debug logging
    SET_ESP_DBGON   = 0xf1,  // enable debug logging
    RECON_MQTT      = 0xf2   // reconnect mqtt
};

enum class WifiStatus : uint8_t {
    GOT_IP       = 0,
    CONNECTED    = 1,
    CONNECTING   = 2,
    DISCONNECTED = 3,
    UNKNOWN      = 15
};

// ntp
const long  gmtOffset_sec = 3600;
const char* ntpServer = "de.pool.ntp.org"; 
const int   daylightOffset_sec = 3600;

// update
unsigned long lastFailedUpdate = 0;
const unsigned long updateDelay = 5000; // 10 seconds in milliseconds

// HTML page for OTA update
const char* updatePage = R"rawliteral(
<!DOCTYPE html>
<html>
<head><title>ESP32P4 lora OTA Update</title></head>
<body>
  <h1>ESP32P4 lora OTA Update</h1>
  <form method="POST" action="/update" enctype="multipart/form-data">
    <input type="file" name="firmware">
    <input type="submit" value="Upload Firmware">
  </form>
</body>
</html>
)rawliteral";

// led tests
// Define the pin where the built-in RGB LED is connected
#if NEOPIXEL == 1
#define LED_PIN 48
// Define the number of LEDs in the strip (usually 1 for built-in LED)
#define NUM_LEDS 1
Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
struct rgbColor { int r; int g; int b;};
volatile rgbColor ledColor = {0,0,0};
#endif
#if NEOPIXEL == 1
  #define NEO(code) code
#else
  #define NEO(code)
#endif

////////////////////////////////////////////////////
// helper functions

// round to digits, standard round() returns integer here
float floatRound(float value, int digits) {
  return round(value * 10.0 * digits) / 10.0 * digits;
}

// caluclate temp delta, deal with 0 C issue, suggested by ChatGPT
float calcDelta(float curTemp, float temp, float eps) {
  float deltaPerc = 0;
  const float maxPercDelta = 50.0; // max allowed change bound

  if(abs(temp) < eps) {
    deltaPerc = (curTemp - temp) / eps;
    deltaPerc = fmin(fmax(deltaPerc * 100, -maxPercDelta), maxPercDelta);
  } else {
    deltaPerc = (curTemp - temp) / temp;
  }
  return deltaPerc;
}

uint32_t bytesToUint32(uint8_t* bytes, int offset) {
  return ((uint32_t)bytes[offset] << 24) |
         ((uint32_t)bytes[offset + 1] << 16) |
         ((uint32_t)bytes[offset + 2] << 8) |
         bytes[offset + 3];
}

uint16_t bytesToUint16(uint8_t* bytes, int offset) {
  return ((uint16_t)bytes[offset] << 8) | bytes[offset + 1];
}

int16_t bytesToInt16(uint8_t* bytes, int offset) {
  uint16_t val = bytesToUint16(bytes, offset);
  // Handle sign extension for negative values
  if (val & 0x8000) {
    return (int16_t)(val - 65536);
  }
  return (int16_t)val;
}

void bufferToHex(const uint8_t *buf, size_t len, char *out, size_t outSize) {
  // Required size: len * 3 (AA␠) or len * 2 + 1 (no spaces)
  if (outSize < (len * 3)) return;

  char *p = out;
  for (size_t i = 0; i < len; i++) {
    sprintf(p, "%02X", buf[i]);
    p += 2;
    if (i < len - 1) *p++ = ' ';
  }
  *p = '\0';
}

void logError(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);

  // Write into the global buffer
  vsnprintf(lastError, ERRBUFLEN, fmt, args);
  
  va_end(args);

  // Optionally also print to Serial
  Serial.printf("%s\n", lastError);
}

void clearLastError() {
    lastError[0] = '\0';  // reset
}

void logMsg(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    
    #if LOG_MODE == LOG_WEB || LOG_MODE == LOG_BOTH
        vsnprintf(logBuffer[logHead], LOG_LINE_LEN, fmt, args);
        logHead = (logHead + 1) % LOG_LINES;
    #endif
    
    #if LOG_MODE == LOG_SERIAL || LOG_MODE == LOG_BOTH
        vprintf(fmt, args);
        printf("\n");
    #endif
    
    va_end(args);
}

const char* wifiStatusToStr(uint8_t s) {
    switch(s) {
        case (uint8_t)WifiStatus::GOT_IP:       return "got ip";
        case (uint8_t)WifiStatus::CONNECTED:    return "connected";
        case (uint8_t)WifiStatus::CONNECTING:   return "connecting";
        case (uint8_t)WifiStatus::DISCONNECTED: return "disconnected";
        case (uint8_t)WifiStatus::UNKNOWN:      return "unknown";
        default:                                return "???";
    }
}

////////////////////////////

uint8_t calculateXorChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

uint8_t calculateCrc8(uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool parsePwEn(uint8_t* data, size_t len, ShellyPWEN* result) {
  int telLen = 14;
  if (len != telLen) {  // Changed from 12 to 13
    logMsg(" WARN: unexpected payload pwen length: %d bytes (expected %d)\n", len, telLen);
    return false;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = data[telLen-1];
  uint8_t calculatedChecksum = calculateCrc8(data, telLen-1);
  
  if (receivedChecksum != calculatedChecksum) {
    cntLoraChecksum++;
    logMsg("ERR checksum failed! Received: 0x%02X, Calculated: 0x%02X", 
                  receivedChecksum, calculatedChecksum);
    return false;
  }
  
  // assign to target structure
  result->energy = bytesToUint32(data, 1);
  result->power = bytesToInt16(data, 5);
  result->pvEnergy = bytesToUint32(data, 7);
  result->pvPower = bytesToInt16(data, 11);
  
  return true;
}

bool parseEnExp(uint8_t* data, size_t len, ShellyENEXP* result) {
  int telLen = 6;
  if (len != telLen) {  
    logMsg(" WARN: unexpected payload enexp length: %d bytes (expected %d)\n", len, telLen);
    return false;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = data[telLen-1];
  uint8_t calculatedChecksum = calculateCrc8(data, telLen-1);
  
  if (receivedChecksum != calculatedChecksum) {
    cntLoraChecksum++;
    logMsg("ERR checksum failed! rcv: 0x%02X, calc: 0x%02X\n", receivedChecksum, calculatedChecksum);
    return false;
  }
  
  // assign to target structure
  result->exportEnergy = bytesToUint32(data, 1);
  
  return true;
}


bool parseHealth(uint8_t* data, size_t len, ShellyHEALTH* result) {
  int telLen = 9;
  if (len != telLen) {  
    logMsg(" WARN: unexpected payload health length: %d bytes (expected %d)\n", len, telLen);
    return false;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = data[telLen-1];
  uint8_t calculatedChecksum = calculateCrc8(data, telLen-1);
  
  if (receivedChecksum != calculatedChecksum) {
    cntLoraChecksum++;
    logMsg("ERR checksum failed! rcv: 0x%02X, calc: 0x%02X\n", receivedChecksum, calculatedChecksum);
    return false;
  }
  
  // assign to target structure
  result->uptime = bytesToUint32(data, 1);
  uint8_t loraTemp = data[5];
  int8_t temp = (int8_t)loraTemp; // cast reinterprets the bits as signed, undoes the +256 in encode for negative numbers
  result->tempC = temp; 
  result->wifiRssi = data[6];
  result->wifiStatus = data[7];
  
  return true;
}


void printPwEnData(const ShellyPWEN& data) {
  logDbg(" energy:  %u Wh  power:  %d W,   pvEnergy:  %u Wh  pvPower:  %d W", data.energy, data.power, data.pvEnergy, data.pvPower);
}

void printEnExpData(const ShellyENEXP& data) {
  logDbg(" h6ExpEnergy:  %u Wh", data.exportEnergy);
}

void printHealthData(const ShellyHEALTH& data) {
    uint32_t uptime = data.uptime;
    uint32_t days   = uptime / 86400;
    uint32_t hours  = (uptime % 86400) / 3600;
    uint32_t mins   = (uptime % 3600) / 60;
    logDbg(" uptime: %ud %02uh %02um, temp: %d C, wifi rssi %d dBm, wifi status: %s", days, hours, mins, data.tempC, data.wifiRssi, wifiStatusToStr(data.wifiStatus));
}

// Validation function
bool validatePwEnData(const ShellyPWEN& current, const ShellyPWEN& previous, bool hasPrevious) {
  // Check pvpower range (0 to 950W)
  if (current.pvPower < 0 || current.pvPower > 950) {
    logMsg("  WARN  pvpower %d out of range [0, 950]\n", current.pvPower);
    return false;
  }
  
  // Check house power range (example: -10000 to 32000W)
  if (current.power < -950 || current.power > 5000) {
    logMsg("  WARN  power %d out of range\n", current.power);
    return false;
  }

  // Only check jumps if we have previous data
  if (hasPrevious) {
    // coarse check energy data
    if (current.energy < previous.energy) {
      logMsg("  WARN: energy decreased to %d from %d\n", current.energy, previous.energy);
      return false;
    }
    if (current.pvEnergy < previous.pvEnergy) {
      logMsg("  WARN: pv energy decreased to %d from %d\n", current.pvEnergy, previous.pvEnergy);
      return false;
    }

    // Check for upward energy jump > 1% 
    // bug cntLora != 2 && does not allow for errors and the switch to good data after the second packet
    if (current.energy > previous.energy && previous.energy > 0) {
      float increasePercent = ((float)(current.energy - previous.energy) / previous.energy) * 100.0;
      if (increasePercent > 1.0) {
        logMsg(" ERR validation failed: energy increased by %.2f%% (%u -> %u Wh)\n", 
                      increasePercent, previous.energy, current.energy);
        return false;
      }
    }
    
    // Check for upward PV energy jump > 1%
    if (current.pvEnergy > previous.pvEnergy && previous.pvEnergy > 0) {
      float pvIncreasePercent = ((float)(current.pvEnergy - previous.pvEnergy) / previous.pvEnergy) * 100.0;
      if (pvIncreasePercent > 1.0) {
        logMsg(" ERR validation failed: pvEnergy increased by %.2f%% (%u -> %u Wh)\n", 
                      pvIncreasePercent, previous.pvEnergy, current.pvEnergy);
        return false;
      }
    }
  }
    
  return true;
}

/////////////////////////////////////////////////
// tasks

IRAM_ATTR void setFlag(void) {
    loraRcvFlag = true;
}
     
void PostMQTT(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
    // for lora do not publish  
#if LORA != 1      
    if(millis() - mqttPublishTime > mqttPubInt) {
#if sendMQTT == 1      
      sendMQTTTemp();
#endif      
      mqttPublishTime = millis();
    }
#endif
   vTaskDelay(250 / portTICK_PERIOD_MS);
  } // for
}


void GetNtpTime(void * parameter) {
  // seems the ntp client does not need to run in a loop
  for(;;) {
     vTaskDelay(pdMS_TO_TICKS(10*60*1000)); 
  }
}

// wifi handler
void WifiCheckReconn(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(1000);

  for(;;) {
    NEO(ledColor.r = 255; ledColor.g = 100;)
    NEO(delay(500);)
    NEO(ledColor.r = 0; ledColor.g = 0;)

    if (WiFi.status() != WL_CONNECTED) {
        //ledColor.r = 255;
        cntWifiReConn++;
        Serial.printf("wifi NOT connected, %d\n", cntWifiReConn);
        WiFi.reconnect();     
        delay(2000);
        if (WiFi.status() == WL_CONNECTED) {
          Serial.printf("wifi reconn ok\n");
          NEO(ledColor.r = 0;)
        }
        else {
          Serial.printf("wifi reconn FAILED\n");
        }
    }  
    vTaskDelay(pdMS_TO_TICKS(45*1000));
  } // for
}

// Ethernet event handler
void onEthEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      logMsg("eth started");
      ETH.setHostname("esp32-p4-device");
      break;
      
    case ARDUINO_EVENT_ETH_CONNECTED:
      logMsg("eth connected");
      break;
      
    case ARDUINO_EVENT_ETH_GOT_IP:
      ip = ETH.localIP();
      logMsg("eth got ip, ip: %d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
      eth_connected = true;
      break;
      
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("eth disconnected");
      eth_connected = false;
      break;
  }
}

void WatchDog(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
    NEO(ledColor.b = 255;)
    NEO(delay(500);)
    NEO(ledColor.b = 0;)

    // this works fine
    NEO(led.setPixelColor(0, led.Color(0, 0, 255));)  // Set blue
    NEO(led.show();)

    vTaskDelay(pdMS_TO_TICKS(60*1000));
  } // for
}

void WatchDogLora(void* parameter) {
    struct tm timeinfo;
    char timeString[50];

    delay(5000);
    logMsg("starting lora watchdog, max silence: %d min", LORA_MAX_SILENCE/1000/60);

    for(;;) {
        unsigned long elapsed = millis() - lastLoraRcv;
        
        if (lastLoraRcv > 0 && elapsed > (LORA_MAX_SILENCE)) {
            cntLoraReset++;
              getLocalTime(&timeinfo);
              strftime(timeString, sizeof(timeString), "%Y-%m-%d %H:%M:%S", &timeinfo);

            logMsg("WARN: no lora rcv for %lu min, kicking radio at: %s", elapsed / 60000, timeString);
            if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(1000))) {
                radio.standby();
                radio.startReceive();
                xSemaphoreGive(loraMutex);
            }
        } else {
            logDbg("lora watchdog ok, last rcv %lu sec ago", elapsed / 1000);
        }
        vTaskDelay(pdMS_TO_TICKS(2*60*1000)); // run every 2 minutes, also good to see if code still runs at all...
    }
}

void LoopMqtt(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
        if (mqtt.connected()) {
            mqtt.loop();
        } else {
            logMsg("MQTT disconnected, reconnecting...");
            connectMQTT();
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void LoraReceiveTask(void* parameter) {
  uint8_t buffer[BUFLEN] = {0};  // zero initialised;
  Serial.println("LoRa receive task started");
  
  for(;;) {
    if(loraRcvFlag) {
      loraRcvFlag = false;
    
      if (xSemaphoreTake(loraMutex, portMAX_DELAY)) {
        memset(buffer, 0, BUFLEN);  // clear buffer first
        size_t len = radio.getPacketLength(true);
        logDbg("lora rcv raw packet len: %d", len);
        //if (len == 0) len = BUFLEN; // if len is 0, fall back to BUFLEN first claude suggestion
        if (len == 0) {
            logDbg("zero length packet, ignoring");
            radio.startReceive();
            xSemaphoreGive(loraMutex);
            continue;  // skip to next iteration of for(;;)
        }        
        int status  = radio.readData(buffer, len);
        float rssi = radio.getRSSI();
        float snr  = radio.getSNR();
        radio.startReceive();  // back to receive immediately
        xSemaphoreGive(loraMutex);

        if (status == RADIOLIB_ERR_NONE) {
          lastLoraRcv = millis();
          cntLora++;
          bufferToHex(buffer, len, currMsg, BUFLEN);
          logMsg("lora rcv: %d bytes,  tel: %02x,  rssi: %.1f dBm, snr: %.1f dB", len, buffer[0], rssi, snr);
          logMsg("raw data: %s", currMsg);

          uint8_t telType = buffer[0];
          switch (static_cast<LoraTel>(telType)) {
              case LoraTel::H6_PWEN:
                  ShellyPWEN data;
                  if (parsePwEn(buffer, len, &data)) {
                      h6PwEn_valid = true;
                      printPwEnData(data);
                      if (validatePwEnData(data, h6PwEn_prev, h6PwEn_valid)) {
                        // Data is good - update global and save as previous
                        cntLoraOk++;
                        h6PwEn_prev = h6PwEn;
                        h6PwEn = data;
                        h6PwEn_valid = true;
                        memcpy(lastGoodMsg, currMsg, strlen(currMsg) + 1); // copy old message
#if MQTTenable == 1
                      mqttPubLoraPwEn();
#endif
                      }
                      else {
                        cntLoraInv++;
                        logMsg("data validation failed - ignoring packet, rssi: %.1f dBm, snr: %.1f dB", rssi, snr);
                      }
                  }
                  break;

              case LoraTel::H6_ENEXP:
                  if (parseEnExp(buffer, len, &h6EnExp)) {
                      h6EnExp_valid = true;
                      printEnExpData(h6EnExp);
#if MQTTenable == 1
                      mqttPubLoraEnExp();
#endif
                  }
                  break;

              case LoraTel::H6_HEALTH:
                  if (parseHealth(buffer, len, &h6Health)) {
                      h6Health_valid = true;
                      printHealthData(h6Health);
#if MQTTenable == 1
                      mqttPubLoraHealth();
#endif
                  }
                  break;

              // outgoing requests — ignore silently, not an error
              case LoraTel::GET_H6_ENEXP:
              case LoraTel::GET_H6_HEALTH:
                  logDbg("ignoring own request echo: 0x%02X", telType);
                  break;
              default:
                  logMsg("unknown tel type: 0x%02X, ignoring", telType);
                  break;
          }      

          loraStatus_prev = loraStatus;
          loraStatus.rssi = rssi;
          loraStatus.snr = snr;
          loraStatus.cntCSerr = cntLoraChecksum;
#if MQTTenable == 1
          mqttPubLoraStatus();
#endif    
        }   
        else if (status == RADIOLIB_ERR_RX_TIMEOUT) {
          // Normal timeout - no packet received
        } 
        else {
          logMsg("[SX1262] receive error: %d", status);
          cntLoraErr++;
          loraLastRcvErr = status;
        }
      } // if (xSemaphoreTake
    }  // loraRcvFlag  
    
    // CRITICAL: Always yield to prevent watchdog
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

#if NEOPIXEL == 1
void MonLED(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  //delay(1000);
  led.setBrightness(16); // set global brightness 255 is 100 %
  led.clear();
  led.show();

  for(;;) {
    led.setPixelColor(0, led.Color(ledColor.r, ledColor.g, ledColor.b));
    led.show(); // Turn off LED initially
    delay(50);
  } // for
}
#endif

bool initLoRa() {
  pinMode(LORA_RXEN, OUTPUT);
  pinMode(LORA_TXEN, OUTPUT);
  digitalWrite(LORA_RXEN, LOW);
  digitalWrite(LORA_TXEN, LOW);
  
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);
  
  pinMode(LORA_RST, OUTPUT);
  pinMode(LORA_BUSY, INPUT);
  pinMode(LORA_DIO1, INPUT);
  
  // Reset module
  digitalWrite(LORA_RST, LOW);
  delay(100);
  digitalWrite(LORA_RST, HIGH);
  delay(100);

  SPI.begin(LORA_CLK, LORA_MISO, LORA_MOSI);
  delay(100);

  //Serial.println("\n=== RadioLib init ===");
  int state = radio.begin(867.125);
  
  if (state == RADIOLIB_ERR_NONE) {
    logMsg("lora radio.begin ok");
    
    // Configure RF switches
    radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
    
    // Configure LoRa parameters
    radio.setSpreadingFactor(loraSF); //sf12 gets around 0 dB
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setOutputPower(22);
    
    logMsg("lora config ok, SF: %u", loraSF);
    return true;
  } 
  else {
    logMsg("ERR lora init failed, code: %d", state);
    return false;
  }
}

//  -----------------------------

void connectMQTT() {
  // Connect to the MQTT broker
  //mqtt.begin(MQTT_BROKER, MQTT_PORT, network); // move to setup to be able to use this call for reconnects
  char mqttConnecTime[40];

  // check if we need to do anything
  if (mqtt.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttAttempt < MQTT_RETRY_INTERVAL) return;
  lastMqttAttempt = now;
 
  getLocalTime(&mqttTimeInfo);
  strftime(mqttConnecTime, sizeof(mqttConnecTime), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);
 
  logMsg("connecting to mqtt broker %s:%d at: %s ...", MQTT_BROKER, MQTT_PORT, mqttConnecTime);
  
  if (mqtt.connect(MQTT_CLIENT_ID)) {
      //logMsg("MQTT connected");
      mqtt.publish("iot/power/h6/availability", "online", true, 1);

      char subTopic[64];
      snprintf(subTopic, sizeof(subTopic), "%s/%s", mqttTopic, mqttCmd);
      mqtt.subscribe(subTopic);
      logMsg("subscribed to: %s", subTopic);

      cntMReCon++;
  } else {
      Serial.print("!m");
      cntMDisCon++;
  }
}

void mqttPubTempData() {
  bool published;
  StaticJsonDocument<200> message;
  char topic[128];

  snprintf(topic, sizeof(topic), "%s", mqttTopic);

  message["timestamp"] = millis();
  message["buffer5"] = bufTemp.tempBuf5;
  message["buffer2"] = bufTemp.tempBuf2;
  message["ww"] = bufTemp.tempWW;
  char messageBuffer[512];
  serializeJson(message, messageBuffer);

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);
 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(topic, messageBuffer); //, false, 1); // no retain, qos 0, without them getting retCode 1 even if data arrive in HA, qos 1 still responds with 1
  if (published) {
    NEO(ledColor.g = 255;)
    logDbg("%s  mqtt pub temp, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPub++;
    NEO(delay(200);)
    NEO(ledColor.g = 0;)
  }
  else {
    NEO(ledColor.g = 255;   ledColor.b = 255;)
    logMsg("%s  ERROR mqtt pub temp, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPubErr++;
    NEO(delay(500);)
    NEO(ledColor.g = 0;   ledColor.b = 0;)
  }
}

void rcvMqttMessage(String &topic, String &payload) {
    // convert once, work with c-strings avoid head problems
    //const char* tpc = topic.c_str();
    //char cmdTopic[64];
    //snprintf(cmdTopic, sizeof(cmdTopic), "%s/cmd", mqttTopic);
    //if (strcmp(tpc, cmdTopic) == 0)   handleMqttRcv(payload);
  String payload_log = payload; // copy
  payload_log.replace("\n", " ");
  payload_log.replace("\r", " ");
  logDbg("mqtt rcv msg: %s = %s", topic.c_str(), payload_log.c_str()); 
  cntMRcv++;
  if (topic.startsWith(mqttTopic)) {
      String subtopic = topic.substring(strlen(mqttTopic) + 1); // "cmd", "status" etc
      if (subtopic == mqttCmd)
        handleMqttRcv(payload);
  }

}

void handleMqttRcv(String &payload) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (error) {
        logMsg("handleMqttRcv, JSON parse error: %s", error.c_str());
        return;
    }

    const char* cmd = doc[mqttCmd];
    if (!cmd) {
        logMsg("handleMqttRcv, no cmd key in payload");
        return;
    }

    if (strcmp(cmd, "getH6Export") == 0) sendLoraRequest(LoraTel::GET_H6_ENEXP);
    else if (strcmp(cmd, "getH6Health") == 0) sendLoraRequest(LoraTel::GET_H6_HEALTH);
    else if (strcmp(cmd, "setEspDbgON")  == 0) { debugMode = true;  logMsg("debug on");  }
    else if (strcmp(cmd, "setEspDbgOFF") == 0) { debugMode = false; logMsg("debug off"); }
    else if (strcmp(cmd, "reconnMqtt") == 0) { connectMQTT(); }
    //else if (strcmp(cmd, "reset")       == 0) ESP.restart();
    else logMsg("handleMqttRcv, unknown cmd: %s", cmd);
}

void mqttPubLoraPwEn() {
  bool published;
  StaticJsonDocument<200> message;
  char topic[128];

  snprintf(topic, sizeof(topic), "%s/data/pwen", mqttTopic);

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);

  message["timestamp"] = mqttLastPublishDate;
  message["h6energy"] = h6PwEn.energy;
  message["h6power"] = h6PwEn.power;
  message["h6pvenergy"] = h6PwEn.pvEnergy;
  message["h6pvpower"] = h6PwEn.pvPower;
  char messageBuffer[512];
  serializeJson(message, messageBuffer);
 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(topic, messageBuffer); //, false, 1); // no retain, qos 0, without them getting retCode 1 even if data arrive in HA, qos 1 still responds with 1
  if (published) {
    NEO(ledColor.g = 255;)
    logDbg("%s  mqtt pub pwen, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPub++;
    NEO(delay(200);)
    NEO(ledColor.g = 0;)
  }
  else {
    NEO(ledColor.g = 255;   ledColor.b = 255;)
    logMsg("%s  ERROR mqtt pub pwen, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPubErr++;
    NEO(delay(500);)
    NEO(ledColor.g = 0;   ledColor.b = 0;)
  }
}

void mqttPubLoraEnExp() {
  bool published;
  StaticJsonDocument<200> message;
  char topic[128];

  snprintf(topic, sizeof(topic), "%s/data/enexp", mqttTopic);

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);

  message["timestamp"] = mqttLastPublishDate;
  message["exportEnergy"] = h6EnExp.exportEnergy;
  char messageBuffer[512];
  serializeJson(message, messageBuffer);
 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(topic, messageBuffer, true, 1); //retain and qos 1 to deal with ha restarts
  if (published) {
    NEO(ledColor.g = 255;)
    logDbg("%s  mqtt pub enexp, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPub++;
    NEO(delay(200);)
    NEO(ledColor.g = 0;)
  }
  else {
    NEO(ledColor.g = 255;   ledColor.b = 255;)
    logMsg("%s  ERROR mqtt pub enexp, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPubErr++;
    NEO(delay(500);)
    NEO(ledColor.g = 0;   ledColor.b = 0;)
  }
}

void mqttPubLoraHealth() {
  bool published;
  StaticJsonDocument<200> message;
  char topic[128];

  snprintf(topic, sizeof(topic), "%s/data/health", mqttTopic);

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);

  message["timestamp"] = mqttLastPublishDate;
  message["uptime"] = h6Health.uptime;
  message["tempC"] = h6Health.tempC;
  message["wifiRssi"] = h6Health.wifiRssi;
  message["wifiStatus"] = wifiStatusToStr(h6Health.wifiStatus);
  char messageBuffer[512];
  serializeJson(message, messageBuffer);
 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(topic, messageBuffer, true, 1); // true,1 for retain and QoS at least once, to deal with HA restarts //, false, 1); // no retain, qos 0, without them getting retCode 1 even if data arrive in HA, qos 1 still responds with 1
  if (published) {
    NEO(ledColor.g = 255;)
    logDbg("%s  mqtt pub health, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPub++;
    NEO(delay(200);)
    NEO(ledColor.g = 0;)
  }
  else {
    NEO(ledColor.g = 255;   ledColor.b = 255;)
    logMsg("%s  ERROR mqtt pub health, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPubErr++;
    NEO(delay(500);)
    NEO(ledColor.g = 0;   ledColor.b = 0;)
  }
}
// send mqtt commands to lora
void sendLoraRequest(LoraTel cmd) {
    int16_t status;
    uint8_t packet[1] = { static_cast<uint8_t>(cmd) };  // enum value goes straight into the byte
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(5000))) {
        radio.standby();
        status = radio.transmit(packet, 1);
        radio.startReceive();  // ← back to listening
         xSemaphoreGive(loraMutex);
        if(status == RADIOLIB_ERR_NONE ) {
          cntLoraSnd++;
          logMsg("send lora msg: 0x%x", packet[0]);
        }
        else {
          logMsg("[SX1262] Transmit error: %d", status);
          loraLastSndErr = status;
        }
       
    } 
    else {
        logMsg("Radio mutex timeout");
    }
}

void mqttPubLoraStatus() {
  bool published;
  StaticJsonDocument<200> message;
  char topic[128];

  snprintf(topic, sizeof(topic), "%s/status", mqttTopic);

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);

  message["timestamp"] = mqttLastPublishDate;
  message["rssi"] = loraStatus.rssi;
  message["snr"] = loraStatus.snr;
  message["checksumErr"] = loraStatus.cntCSerr;
  char messageBuffer[512];
  serializeJson(message, messageBuffer);

 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(topic, messageBuffer); //, false, 1); // no retain, qos 0, without them getting retCode 1 even if data arrive in HA, qos 1 still responds with 1
  if (published) {
    NEO(ledColor.g = 255;)
    //logDbg("%s  mqtt pub lorastatus, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPub++;
    NEO(delay(200);)
    NEO(ledColor.g = 0;)
  }
  else {
    NEO(ledColor.g = 255;   ledColor.b = 255;)
    logMsg("%s  ERROR mqtt pub lorastatus, topic: %s, payload: %s", mqttLastPublishDate, topic, messageBuffer);
    cntMPubErr++;
    logMsg("  publish failed, forcing reconnect");
    mqtt.disconnect();   // force state reset
    NEO(delay(500);)
    NEO(ledColor.g = 0;   ledColor.b = 0;)
  }
}
// ==================== html server pages, thanks ChatGPT, well, besides the test or testLED fiasko
void handleRoot() {
#if LAN == 1  
  IPAddress ip = ETH.localIP();
#elif WLAN == 1
  IPAddress ip = WiFi.localIP();
  int32_t rssi = WiFi.RSSI();
#endif 
 

  int tempInt = temperatureRead();
  unsigned long uptime = millis() / 1000;
  struct tm timeinfo;
  char timeString[50];
  int mqttConnected = 0;

  getLocalTime(&timeinfo);
  strftime(timeString, sizeof(timeString), "%A, %B %d %Y %H:%M:%S", &timeinfo);

#if MQTTenable == 1  
  if(mqtt.connected()) {
    mqttConnected = 1;
  } else {
    mqttConnected = 0;
    cntMDisCon++;
  }
#endif 

  String html = "<!DOCTYPE html><html>";
  html += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
  html += "<link rel=\"icon\" href=\"data:,\">";
  html += "<title>ESP32P4 lora</title></head><body>";
  html += "<style>table { border-collapse: collapse; width: 30%; text-align: center; } th, td { border: 1px solid black; padding: 3px; }  </style>";

#if PROD == 1  
  html += "<h1>Prod LoRa esp32p4</h1>";
#else
  html += "<h1>test lora esp32p4</h1>";
#endif  
  html += "<p>time: " + String(timeString) + ", IP: " + ip.toString();
#if WLAN == 1
  html += ", rssi: " + String(rssi) + " dBm, WiFi reconnects: " + cntWifiReConn;
#endif
  html += "</p>";
  html += "<p>cpu Frequency: " + String(getCpuFrequencyMhz()) + " MHz, Core: " + String(xPortGetCoreID()) + ", Internal Temp: " + String(tempInt) + " C";
  html += "<br>free heap: " + String(ESP.getFreeHeap() / 1024.0, 1) + " KB" + ", min free: " + String(ESP.getMinFreeHeap() / 1024.0, 1) + " KB";     
// task stack high water marks - how close to stack overflow
  html += "<br>stack remaining: ";
  html += "LoRa task: " + String(uxTaskGetStackHighWaterMark(TLoraReceiveTask) * 4) + " bytes, ";
  html += "WatchDog: "  + String(uxTaskGetStackHighWaterMark(TWatchDog) * 4) + " bytes, ";
  html += "WatchDogLora: "  + String(uxTaskGetStackHighWaterMark(TWatchDogLora) * 4) + " bytes, ";
  html += "LoopMqtt: "  + String(uxTaskGetStackHighWaterMark(TLoopMqtt) * 4) + " bytes, ";
  html += "PostMQTT: "  + String(uxTaskGetStackHighWaterMark(TPostMqtt) * 4) + " bytes</p>";  
  html += "<p>  boot at: " + String(bootTimeStr) + ", uptime: " + String(uptime) +" sec</p>";
  //html += String("<p><ul><li>energy:   ") + String(h6EnPV.energy) + " Wh,  power: " + String(h6EnPV.power) + " W</li>";
  //html += String("<li>pvEnergy: ") + String(h6EnPV.pvEnergy) + " Wh,  pvPower: " + String(h6EnPV.pvPower) + " W</li>";
#if LORA == 1  
  html += "<p>loraSF: " + String(loraSF) +  ", lora watchdog resets: " + String(cntLoraReset);
  html += "<br>rcv cntLora: " + String(cntLora) + ",  lora ok: " + String(cntLoraOk) + ",  lora invalid: " + String(cntLoraInv) + ",  loraErr: " + String(cntLoraErr) + ",  checksum error: " + String(cntLoraChecksum);
  html += "<br>lora sent: " + String(cntLoraSnd) + "</p>";
  html += "<p>lastError: " + String(lastError) + ",  last lora send error: " + String(loraLastSndErr);
  html += "<br>curr: " + String(currMsg) + "<br>good: " + String(lastGoodMsg) + "</p>";
  //html += String("<p><ul><li>bmeTem: ") + String(bmeTemp) + " C</li><li>bmeHum: " + String(bmeHum) + " %</li><li>bmePres: " + String(bmePres) + " hPa</li>";
  //html += String("<li>bmeGasRes: ") + String(bmeGasRes) + " kOhm</li>";
  //html += String("<li>cntBme: ") + String(cntBme) + "</li><li>bad val counts: bme: " + String(cntBadBme) + "</li></ul></p>";

  //html += "<p><table style=\"width: 80%; table-layout: fixed;\><colgroup><col style=\"width: 10%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"></colgroup>";
  html += "<p><table><colgroup><col style=\"width: 10%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"></colgroup>";
  html += "<tr><th>values</th><th style=\"white-space: nowrap;\">energy [Wh]</th><th style=\"white-space: nowrap;\">PVenergy [Wh]</th><th style=\"white-space: nowrap;\">power [W]</th><th style=\"white-space: nowrap;\">PVpower [W]</th><th style=\"white-space: nowrap;\">rssi [dBm]</th><th style=\"white-space: nowrap;\">snr [dB]</th></tr>";
  html += "<tr><td>prev</td><td>"+ String(h6PwEn_prev.energy)  + "</td><td>" + String(h6PwEn_prev.pvEnergy)  + "</td><td>" + String(h6PwEn_prev.power) + "</td><td>" + String(h6PwEn_prev.pvPower) + "</td><td>" + String(loraStatus_prev.rssi, 1) + "</td><td>" + String(loraStatus_prev.snr, 1) + "</td></tr>";
  html += "<tr><td>curr</td><td>" + String(h6PwEn.energy)  + "</td><td>" + String(h6PwEn.pvEnergy)  + "</td><td>" + String(h6PwEn.power) + "</td><td>" + String(h6PwEn.pvPower) + "</td><td>" + String(loraStatus.rssi, 1) + "</td><td>" + String(loraStatus.snr, 1) + "</td></tr>";
  html += "</table></p>";
#endif

  //html += "<p><table><colgroup><col style=\"width: 12%;\"><col style=\"width: 20%;\"><col style=\"width: 20%;\"><col style=\"width: 20%;\"></colgroup>";
  //html += "<tr><th>desc</th><th>temp C</th><th>delta %</th><th>raw C</th><th>bad vals</th><th>consect bad</th></tr>";
  //html += String("<tr><td>buf5</td><td>") + tempBuf5  + String("</td><td>") + floatRound(deltaBuf5*100, 1)  + String("</td><td>") + rawBuf5 + String("</td><td>") + cntBadBuf5 + String("</td><td>") + cntConsBadBuf5 + String("</td></tr>");
  //html += String("<tr><td>buf2</td><td>") + tempBuf2  + String("</td><td>") + floatRound(deltaBuf2*100, 1)  + String("</td><td>") + rawBuf2 + String("</td><td>") + cntBadBuf2 + String("</td><td>") + cntConsBadBuf2 + String("</td></tr>");
  //html += String("<tr><td>ww</td><td>") + tempWW  + String("</td><td>") + floatRound(deltaWW*100, 1)  + String("</td><td>") + rawWW + String("</td><td>") + cntBadWW + String("</td><td>") + cntConsBadWW + String("</td></tr>");
  //html += "</table></p>";

#if MQTTenable == 1  
  html += "<p>mqtt broker: " + String(MQTT_BROKER) + ", client: " + String(MQTT_CLIENT_ID) + ", topic: " + String(mqttTopic)  +
          "<br>Last Published: " + String(mqttLastPublishDate) + ", connected: " + String(mqttConnected) + "</p>";
  html += "<p><ul><li>mqtt pubs: " + String(cntMPub) + "</li><li>mqtt pub errors: " + String(cntMPubErr) + "</li><li>mqtt reconnects: " + String(cntMReCon) + 
          "</li><li>mqtt disconnects: " + String(cntMDisCon) + "</li>";
  html += "<li>mqtt rcv: " + String(cntMRcv) + "</li>";        
  html += "</ul></p>";
#endif  
  html += "<p>" + String(VERSION) + "</p>";
  html += "<p><a href=\"/info\">info</a> <a href=\"/log\">log</a> <a href=\"/ota\">ota</a> </p>";        
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleInfo() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32P4 lora info</title></head><body>";
  html += "<h1>hw info</h1> <ul> <li>esp32 p4 waveshare dev board</li> <li>lora SX1262 HF core board</li> </ul>";
  html += "<h1>sw info</h1> <ul> <li>arduino ide 2.3.8</li> <li>espressif 3.3.7</li> <li>RadioLib 7.6</li> </ul>";
  html += "<p> to build OTA update, in Arduino IDE, Menu Sketch, Export Compiled Binary, upload the esp32p4_eth_http_mqtt_ntp.ino.bin file";
  html += "<p>" + String(VERSION) + "</p>";
  html += "<p><a href=\"/\">Back to Home</a></p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleLog() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32P4 lora log</title></head><body>";
  html += "<pre>";
  for (int i = 0; i < LOG_LINES; i++) {
      int idx = (logHead + i) % LOG_LINES;  // oldest first
      if (logBuffer[idx][0] != '\0') {
          html += String(logBuffer[idx]) + "\n";
      }
  }
  html += "</pre><p><a href=\"/\">Back to Home</a></p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleTestLED() {
  Serial.println("LED test triggered via REST API");
  server.send(200, "text/plain", "LED test starting");
  //testLED(300, 150);
}

void handleTest() {
  Serial.println("test url called");
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>test</title></head><body>";
  html += "<h1>test/h1> <p>test</p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}


// Function to check authentication with delay
bool checkAuthentication() {
  unsigned long currentTime = millis();
  if (currentTime - lastFailedUpdate < updateDelay) {
    server.send(429, "text/html", "<h1>Too many failed attempts. Try again later.</h1>");
    return false;
  }
  
  if (!server.authenticate(otaUsr, otaPW)) {
    lastFailedUpdate = millis();
    server.requestAuthentication();
    return false;
  }
  
  return true;
}

// Serve OTA update page
void handleOTAUpdatePage() {
  Serial.println("handleOTAUpdatePage()");
  //if (!checkAuthentication()) return;
  server.send(200, "text/html", updatePage);
}

// Handle firmware upload
void handleFirmwareUpload() {
  //Serial.println("handleFirmwareUpload()"); // gets called very often during upload
  HTTPUpload& upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    Serial.printf("OTA Update: Start uploading: %s\n", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { // Start with unknown size
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      Serial.printf("OTA Update: Success, %u bytes received, resetting\n", upload.totalSize);
      delay(2000);  // Give time for response to be sent
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
}

void handleUpdate() {
  Serial.println("handleUpdate()");
  //if (!checkAuthentication()) return;
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", "<h1>Update Success! Rebooting...</h1>");
  delay(1000);
  ESP.restart();
}

void handleReset() {
  server.send(200, "text/html", "<p>ESP32 restarting...</p>");
  delay(2000);          // short delay to ensure client gets the response
  ESP.restart();       // trigger ESP32 reboot
}

/////////////////////////////////

void setup() {
  IPAddress ip;
  struct tm timeinfo;
  char upTimeBuf[32];
  
  Serial.begin(115200);
  logMsg("starting ---------- %s -------------", VERSION);

  /*
  Serial.printf("\nbme setup\n");

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!bme.begin(0x77)) {
    Serial.println("BME688 not found");
  } 
  else
  {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    //bme.setGasHeater(0, 0);  // Disable gas readings for now
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(300, 100); // Temp in °C, duration in ms
  }
  */

#if LAN == 1
  // ethernet setup
  Network.onEvent(onEthEvent);
  logMsg("starting eth...");
  ETH.begin(); 
  //logMsg("waiting for eth connection...");
  while (!eth_connected) {
    delay(1000);
    Serial.print("!e");
  }
  ip = ETH.localIP();
  //logMsg("eth connected, ip: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
#endif  

#if WLAN == 1
  // Connect to Wi-Fi network with SSID and password
  logMsg("connecting to ssid: %s", ssid);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    NEO(ledColor.r = 255;)
    delay(1000);
    Serial.print("!w");
    cntWifiReConn++;
    delay(150);
  }
  ip = WiFi.localIP();
  logMsg("WiFi connected to: %s, ip: %d.%d.%d.%d", ssid, ip[0], ip[1], ip[2], ip[3]);
#endif

#if LORA == 1  
  logDbg("LoRa init...");
  while (!lora_initialized) {
    lora_initialized = initLoRa();
    if (!lora_initialized) {
      Serial.print("!l");
      delay(5000);  // Wait 5 seconds before retry
    }
  }
  logDbg("LoRa init ok");
  // start interrupt driven receive
  loraMutex = xSemaphoreCreateMutex(); //need to create here, NOT globally, FreeRTOS must be running
  radio.setDio1Action(setFlag);
  radio.startReceive();  
#endif

  // setup ntp before MQTT to get time in log
  //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); // has a fixed offset, does not work in winter
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", ntpServer);
  logMsg("ntp server: %s, CET-1CEST,M3.5.0,M10.5.0/3", ntpServer);
  getLocalTime(&timeinfo);
  char timeBuf[32];
  strftime(timeBuf, sizeof(timeBuf), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  logMsg("Time: %s", timeBuf);

  // check mqtt and post data if not LORA
#if MQTTenable == 1  
  xTaskCreatePinnedToCore(
      PostMQTT, "PostMqtt", 4096,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TPostMqtt,  /* Task handle. */
      0); /* Core where the task should run */
#endif

  xTaskCreatePinnedToCore(
      WatchDog, "TaskWatchDog", 4096,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TWatchDog,  /* Task handle. */
      1); /* Core where the task should run */

#if LORA == 1
  xTaskCreatePinnedToCore(
    WatchDogLora, "TaskWatchDogLora", 4096, NULL, 0, &TWatchDogLora, 1);
#endif

#if WLAN == 1
  xTaskCreatePinnedToCore(
      WifiCheckReconn, "TaskWifiCheckReconn", 4096,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TWifiCheckReconn,  /* Task handle. */
      1); /* Core where the task should run */
#endif

#if LORA == 1
  xTaskCreatePinnedToCore(
    LoraReceiveTask, "TaskLoRaRcv", 8192, /* Stack size */
    NULL, /* input params */  1, /* Priority */ &TLoraReceiveTask,     // Task handle
    1    // Core (0 or 1)
  );
#endif

  // start web server Define routes and start, next time you test, make sure run tests against the test board ip, NOT the prod one...
  server.on("/", handleRoot);
  server.on("/test", handleTest);
  server.on("/info", handleInfo);
  server.on("/log", handleLog);
  //server.on("/testled", handleTestLED);
  server.on("/ota", handleOTAUpdatePage);
  server.on("/update", HTTP_POST, handleUpdate, handleFirmwareUpload);
  server.on("/reset", handleReset);

  /*
  server.on("/led", HTTP_GET, []() {
    server.send(200, "text/plain", "LED test starting");
    testLED(300, 150);  
  });
  */
  server.begin();
  Serial.printf("HTTP web server started");

#if NTP == 1
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Waiting for NTP time...");
    delay(500);
  }
#endif

  strftime(upTimeBuf, sizeof(upTimeBuf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  bootTimeStr = String(upTimeBuf);

#if MQTTenable == 1
  mqtt.setKeepAlive(180);  // set mqtt keepalive
  mqtt.setWill("iot/power/h6/availability", "offline", true, 1); // generate an availability feature for the HA sensors
  mqtt.begin(MQTT_BROKER, MQTT_PORT, network);
  mqtt.onMessage(rcvMqttMessage);
  connectMQTT();

  xTaskCreatePinnedToCore(LoopMqtt, "TaskLoopMqtt", 8192, NULL, 0, &TLoopMqtt, 0); // create task AFTER conenctMqtt()
#endif

#if NEOPIXEL == 1
  xTaskCreatePinnedToCore(
      MonLED, "TaskMonLED", 4096,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TMonLED,  /* Task handle. */
      1); /* Core where the task should run */
#endif

}

void loop() {
  //Serial.println("Waiting for HTTP requests...");
  server.handleClient(); // WebServer client connection handling
  //delay(1000);  // Prevents flooding the serial monitor
}

// get temp from MAX6675 for buffer and warm water tanks
// 1.0.3 filter bad values, deal with inital value problem
// 1.0.4 add buf2
// 1.1.0 ota update feature
// 1.2.0 add on board led color info, solder first!!, add wifi reconnect and check mqtt publish error
// 2.0.0a copy of the esp32_temp to test bme688
// 2.1.0 using bosch library
// 3.0 p4 eth
// 3.0.1 add wifi back in and enable wifi at compile time
// 3.0.2 integrate sx1262 lora module, seems to only work on p4 dev, not the waveshare nano board
// 3.0.5 add DEFINE for MQTT, in debug set LAN and MQTT to 0 and LORA to 1 to be able to just test LORA
// 3.0.6 add /reset page, print lora msg as hex
// 3.0.8 add mqtt last will / availablity
// 3.1.0 split lora data and status, merge in some changes from esp32p4 compile switches 
// 3.2.0 add mqtt subscription and send lora messages to request e.g. h6 power export data
// 3.2.1 add html logging, move version info to the bottom
// 3.2.4 fix multi mqtt loop after receiver running in loop(), merge led code back in
// 3.2.5 add lora watchdog task to attempt to remedy receive hangs

/*
radio.setFrequency(868.0);       // Frequency in MHz
radio.setSpreadingFactor(7);     // 7-12 (higher = longer range, slower)
radio.setBandwidth(125.0);       // 125, 250, 500 kHz
radio.setCodingRate(5);          // 5-8 (higher = more error correction)
radio.setOutputPower(14);        // TX power in dBm (max 22 dBm for SX1262)
radio.setSyncWord(0x12);         // Private networks use 0x12, LoRaWAN uses 0x34
radio.setPreambleLength(8);      // Preamble length
*/
