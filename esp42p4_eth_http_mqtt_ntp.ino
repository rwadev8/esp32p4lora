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

#define PROD 1 //  REMEMBER to change to 1 for prod deploy

#include <Arduino.h>
#include <ETH.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <MQTTClient.h>
#include <ArduinoJson.h>
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

#define BUFLEN 64
#define ERRBUFLEN 265

const char VERSION[] = "v3.0.6";
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

//GND
#define LORA_RXEN  21
#define LORA_TXEN  22
//#define LORA_DIO2  21
#define LORA_DIO1  20
// 3V

Module mod(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY);
SX1262 radio(&mod);

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

#if PROD == 1
  const char mqttTopicBufferTemp[] = "iot/power/h6/status"; 
#else
  const char mqttTopicBufferTemp[] = "iot/power/h6/test"; 
#endif

unsigned long mqttPublishTime = 0;
struct tm mqttTimeInfo;
char mqttLastPublishDate[40];
int cntMReCon = 0;
int cntMDisCon = 0;
int cntMPub = 0;
int cntMPubErr = 0;
int cntWifiReConn = 0;
int cntBadBme = 0;
int cntBme = 0;
int cntConsBadBme = 0;
int cntLora = 0;
int cntLoraOk = 0;
int cntLoraErr = 0;
int cntLoraInv = 0;
int cntLoraChecksum = 0;

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
#if WLAN == 1
  TaskHandle_t TWifiCheckReconn;
#endif  

// h6 lora
char currMsg[BUFLEN * 3 + 1];
char lastGoodMsg[BUFLEN * 3 + 1];

struct ShellyData {
  uint32_t h6energy;    // Wh
  int16_t h6power;      // W (can be negative)
  uint32_t h6pvenergy;  // Wh
  int16_t h6pvpower;    // W (can be negative)
  float_t rssi;
  float_t snr;
};

ShellyData h6EnPV;
ShellyData h6EnPV_prev;
bool h6EnPV_valid = false;

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
//#define LED_PIN 48
// Define the number of LEDs in the strip (usually 1 for built-in LED)
//#define NUM_LEDS 1
//Adafruit_NeoPixel led(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
//struct rgbColor { int r; int g; int b;};
//volatile rgbColor ledColor = {0,0,0};

////////////////////////////////////////////////////
// helper functions

// round to digits, standard round() returns integer here
float FloatRound(float value, int digits) {
  return round(value * 10.0 * digits) / 10.0 * digits;
}

// caluclate temp delta, deal with 0 C issue, suggested by ChatGPT
float CalcDelta(float curTemp, float temp, float eps) {
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

////////////////////////////

uint8_t calculateXorChecksum(uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

bool parseLoraPayload(uint8_t* data, size_t len, ShellyData* result) {
  if (len != 13) {  // Changed from 12 to 13
    logError("Warning: Unexpected payload length: %d bytes (expected 13)\n", len);
    return false;
  }
  
  // Verify checksum
  uint8_t receivedChecksum = data[12];
  uint8_t calculatedChecksum = calculateXorChecksum(data, 12);
  
  if (receivedChecksum != calculatedChecksum) {
    cntLoraChecksum++;
    logError("Checksum failed! Received: 0x%02X, Calculated: 0x%02X\n", 
                  receivedChecksum, calculatedChecksum);
    return false;
  }
  
  // Parse data (same as before)
  result->h6energy = bytesToUint32(data, 0);
  result->h6power = bytesToInt16(data, 4);
  result->h6pvenergy = bytesToUint32(data, 6);
  result->h6pvpower = bytesToInt16(data, 10);
  
  return true;
}

void printShellyData(const ShellyData& data) {
  //Serial.println("\n=== Shelly Data Received ===");
  Serial.printf("h6energy:   %u Wh  h6power:    %d W\n", data.h6energy, data.h6power);
  Serial.printf("h6pvenergy: %u Wh  h6pvpower:  %d W\n", data.h6pvenergy, data.h6pvpower);
  Serial.printf("RSSI: %.1f dBm, SNR: %.1f dB\n", data.rssi, data.snr);
}

// Validation function
bool validateShellyData(const ShellyData& current, const ShellyData& previous, bool hasPrevious) {
  // Check pvpower range (0 to 950W)
  if (current.h6pvpower < 0 || current.h6pvpower > 950) {
    logError("  WARN  pvpower %d out of range [0, 950]\n", current.h6pvpower);
    return false;
  }
  
  // Check house power range (example: -10000 to 32000W)
  if (current.h6power < -950 || current.h6power > 5000) {
    logError("  WARN  power %d out of range\n", current.h6power);
    return false;
  }

  // Only check jumps if we have previous data
  if (hasPrevious) {
    // coarse check energy data
    if (cntLora != 2 && current.h6energy < previous.h6energy) {
      logError("  WARN: energy decreased to %d from %d\n", current.h6energy, previous.h6energy);
      return false;
    }
    if (cntLora != 2 && current.h6pvenergy < previous.h6pvenergy) {
      logError("  WARN: pv energy decreased to %d from %d\n", current.h6pvenergy, previous.h6pvenergy);
      return false;
    }

    // Check for upward energy jump > 1%
    if (cntLora != 2 && current.h6energy > previous.h6energy) {
      float increasePercent = ((float)(current.h6energy - previous.h6energy) / previous.h6energy) * 100.0;
      if (increasePercent > 1.0) {
        logError("Validation failed: h6energy increased by %.2f%% (%u -> %u Wh)\n", 
                      increasePercent, previous.h6energy, current.h6energy);
        return false;
      }
    }
    
    // Check for upward PV energy jump > 1%
    if (cntLora != 2 && current.h6pvenergy > previous.h6pvenergy) {
      float pvIncreasePercent = ((float)(current.h6pvenergy - previous.h6pvenergy) / previous.h6pvenergy) * 100.0;
      if (pvIncreasePercent > 1.0) {
        logError("Validation failed: h6pvenergy increased by %.2f%% (%u -> %u Wh)\n", 
                      pvIncreasePercent, previous.h6pvenergy, current.h6pvenergy);
        return false;
      }
    }
  }
    
  return true;
}

/////////////////////////////////////////////////
// tasks
     
void PostMQTT(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
#if MQTTenable == 1    
    if(!mqtt.connected()) {
      //TODO a reconnect would be really helpful here
      Serial.print("m");
      delay(1000);
      connectMQTT();
      cntMReCon++;
    }
    mqtt.loop(); // this i supposed to keep the connection alive according to chatcpt
#endif    
    // for lora do not publish  
#if LORA != 1      
    if(millis() - mqttPublishTime > mqttPubInt) {
      sendMQTT();
      mqttPublishTime = millis();
    }
    delay(250);
#endif
   
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
    //ledColor.r = 255; ledColor.g = 100;
    //delay(500);
    //ledColor.r = 0; ledColor.g = 0;

    if (WiFi.status() != WL_CONNECTED) {
        //ledColor.r = 255;
        cntWifiReConn++;
        Serial.printf("wifi NOT connected, %d\n", cntWifiReConn);
        WiFi.reconnect();     
        delay(2000);
        if (WiFi.status() == WL_CONNECTED) {
          Serial.printf("wifi reconn ok\n");
          //ledColor.r = 0;
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
      Serial.println("ETH Started");
      ETH.setHostname("esp32-p4-device");
      break;
      
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH Connected");
      break;
      
    case ARDUINO_EVENT_ETH_GOT_IP:
      ip = ETH.localIP();
      Serial.printf("Ethernet connected, IP: %d.%d.%d.%d\n", 
                    ip[0], ip[1], ip[2], ip[3]);
      eth_connected = true;
      break;
      
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
  }
}

void WatchDog(void * parameter) {
  // at startup wait a few seconds to allow connections to stablize
  delay(5000);

  for(;;) {
    //ledColor.b = 255;
    //delay(500);
    //ledColor.b = 0;

    // this works fine
    //led.setPixelColor(0, led.Color(0, 0, 255));  // Set blue
    //led.show();

    vTaskDelay(pdMS_TO_TICKS(60*1000));
  } // for
}

void LoraReceiveTask(void* parameter) {
  uint8_t buffer[BUFLEN];
  Serial.println("LoRa receive task started");
  
  for(;;) {
    // Set a short timeout for receive (e.g., 1 second)
    radio.setDio1Action(NULL);  // Disable interrupt
    int state = radio.receive(buffer, BUFLEN);
    
    if (state == RADIOLIB_ERR_NONE) {
      cntLora++;
      size_t len = radio.getPacketLength();
      bufferToHex(buffer, len, currMsg, BUFLEN);
      Serial.printf("LoRa packet received: %d bytes\n", len);
      
      Serial.print("Raw data: ");
      for (size_t i = 0; i < len; i++) {
        Serial.printf("%02X ", buffer[i]);
      }
      Serial.println();
      
      ShellyData data;
      if (parseLoraPayload(buffer, len, &data)) {
        data.rssi = radio.getRSSI();
        data.snr = radio.getSNR();
        printShellyData(data);
        if (validateShellyData(data, h6EnPV_prev, h6EnPV_valid)) {
          // Data is good - update global and save as previous
          cntLoraOk++;
          h6EnPV_prev = h6EnPV;
          h6EnPV = data;
          h6EnPV_valid = true;
          memcpy(lastGoodMsg, currMsg, strlen(currMsg) + 1); // copy old message
#if LORA == 1
          sendMQTT();
#endif        
        } else {
          cntLoraInv++;
          Serial.println("Data validation failed - ignoring packet");
        }
        

      }
      
    } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      // Normal timeout - no packet received
    } else {
      Serial.printf("[SX1262] Receive error: %d\n", state);
      cntLoraErr++;
    }
    
    // CRITICAL: Always yield to prevent watchdog
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

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
    Serial.println("success!");
    
    // Configure RF switches
    radio.setRfSwitchPins(LORA_RXEN, LORA_TXEN);
    
    // Configure LoRa parameters
    radio.setSpreadingFactor(12);
    radio.setBandwidth(125.0);
    radio.setCodingRate(5);
    radio.setOutputPower(22);
    
    Serial.println("LoRa config ok");
    return true;
  } else {
    Serial.printf("LoRa init failed, code: %d\n", state);
    return false;
  }
}

//  -----------------------------

void connectMQTT() {
  // Connect to the MQTT broker
  //mqtt.begin(MQTT_BROKER, MQTT_PORT, network); // move to setup to be able to use this call for reconnects

  Serial.printf("Connecting to MQTT broker %s:%d\n", MQTT_BROKER, MQTT_PORT);
  
  while (!mqtt.connect(MQTT_CLIENT_ID)) {
    //ledColor.g = 255; ledColor.b = 255;
    Serial.print("!m");
    delay(250);
    //ledColor.g = 0;  ledColor.b = 0;
    delay(500);
    cntMDisCon++;
  }

  if (!mqtt.connected()) {
      Serial.println("MQTT Connection failed");
      return;
  }
  cntMReCon++;
}

void sendMQTT() {
  bool published;
  StaticJsonDocument<200> message;

  getLocalTime(&mqttTimeInfo);
  strftime(mqttLastPublishDate, sizeof(mqttLastPublishDate), "%Y-%m-%d %H:%M:%S", &mqttTimeInfo);

  message["timestamp"] = mqttLastPublishDate;
  message["h6energy"] = h6EnPV.h6energy;
  message["h6power"] = h6EnPV.h6power;
  message["h6pvenergy"] = h6EnPV.h6pvenergy;
  message["h6pvpower"] = h6EnPV.h6pvpower;
  message["rssi"] = h6EnPV.rssi;
  message["snr"] = h6EnPV.snr;
  char messageBuffer[512];
  serializeJson(message, messageBuffer);

 
  // starting to supect that the retCode is not meaningful in this case, the original code did not have it
  published = mqtt.publish(mqttTopicBufferTemp, messageBuffer); //, false, 1); // no retain, qos 0, without them getting retCode 1 even if data arrive in HA, qos 1 still responds with 1
  if (published) {
    //ledColor.g = 255;
    Serial.printf("%s  sent MQTT, topic: %s, payload: %s\n", mqttLastPublishDate, mqttTopicBufferTemp, messageBuffer);
    cntMPub++;
    //delay(200);
    //ledColor.g = 0;
  }
  else {
    //ledColor.g = 255;   ledColor.b = 255;
    Serial.printf("%s  ERROR sending MQTT, topic: %s, payload: %s\n", mqttLastPublishDate, mqttTopicBufferTemp, messageBuffer);
    cntMPubErr++;
    //delay(500);
    //ledColor.g = 0;   ledColor.b = 0;
  }
}

// ==================== html server pages, thanks ChatGPT, well, besides the test or testLED fiasko
void handleRoot() {
#if LAN == 1  
  IPAddress ip = ETH.localIP();
#elif WLAN == 1
  IPAddress ip = WiFi.localIP();
#endif 
 

  int tempInt = temperatureRead();
  unsigned long uptime = millis() / 1000;
  struct tm timeinfo;
  char timeString[50];
  int mqttConnected = 0;

  getLocalTime(&timeinfo);
  strftime(timeString, sizeof(timeString), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  if(mqtt.connected()) {
    mqttConnected = 1;
  } else {
    mqttConnected = 0;
    cntMDisCon++;
  }

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
  html += "<p>time: " + String(timeString) + ", IP: " + ip.toString()  + "</p>";
  html += "<p>cpu Frequency: " + String(getCpuFrequencyMhz()) + " MHz, Core: " + String(xPortGetCoreID()) +
          ", Internal Temp: " + String(tempInt) + " C</p>";
  html += "<p>uptime: " + String(uptime) +" seconds <br>  boot at: " + String(bootTimeStr) + "</p>";
  //html += String("<p><ul><li>h6energy:   ") + String(h6EnPV.h6energy) + " Wh,  h6power: " + String(h6EnPV.h6power) + " W</li>";
  //html += String("<li>h6pvenergy: ") + String(h6EnPV.h6pvenergy) + " Wh,  h6pvpower: " + String(h6EnPV.h6pvpower) + " W</li>";
  html += "<li>cntLora: " + String(cntLora) + ",  lora ok: " + String(cntLoraOk) + ",  lora invalid: " + String(cntLoraInv) + ",  loraErr: " + String(cntLoraErr) + ",  checksum error: " + String(cntLoraChecksum) + "</li></ul>";
  html += "<p>lastError: " + String(lastError) + "<br>curr: " + String(currMsg) + "<br>good: " + String(lastGoodMsg) + "</p>";

  //html += String("<p><ul><li>bmeTem: ") + String(bmeTemp) + " C</li><li>bmeHum: " + String(bmeHum) + " %</li><li>bmePres: " + String(bmePres) + " hPa</li>";
  //html += String("<li>bmeGasRes: ") + String(bmeGasRes) + " kOhm</li>";
  //html += String("<li>cntBme: ") + String(cntBme) + "</li><li>bad val counts: bme: " + String(cntBadBme) + "</li></ul></p>";

  //html += "<p><table style=\"width: 80%; table-layout: fixed;\><colgroup><col style=\"width: 10%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"></colgroup>";
  html += "<p><table><colgroup><col style=\"width: 10%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"><col style=\"width: 15%;\"></colgroup>";
  html += "<tr><th>values</th><th style=\"white-space: nowrap;\">energy [Wh]</th><th style=\"white-space: nowrap;\">PVenergy [Wh]</th><th style=\"white-space: nowrap;\">power [W]</th><th style=\"white-space: nowrap;\">PVpower [W]</th><th style=\"white-space: nowrap;\">rssi [dBm]</th><th style=\"white-space: nowrap;\">snr [dB]</th></tr>";
  html += "<tr><td>prev</td><td>"+ String(h6EnPV_prev.h6energy)  + "</td><td>" + String(h6EnPV_prev.h6pvenergy)  + "</td><td>" + String(h6EnPV_prev.h6power) + "</td><td>" + String(h6EnPV_prev.h6pvpower) + "</td><td>" + String(h6EnPV_prev.rssi, 1) + "</td><td>" + String(h6EnPV_prev.snr, 1) + "</td></tr>";
  html += "<tr><td>curr</td><td>" + String(h6EnPV.h6energy)  + "</td><td>" + String(h6EnPV.h6pvenergy)  + "</td><td>" + String(h6EnPV.h6power) + "</td><td>" + String(h6EnPV.h6pvpower) + "</td><td>" + String(h6EnPV.rssi, 1) + "</td><td>" + String(h6EnPV.snr, 1) + "</td></tr>";
  html += "</table></p>";

  //html += "<p><table><colgroup><col style=\"width: 12%;\"><col style=\"width: 20%;\"><col style=\"width: 20%;\"><col style=\"width: 20%;\"></colgroup>";
  //html += "<tr><th>desc</th><th>temp C</th><th>delta %</th><th>raw C</th><th>bad vals</th><th>consect bad</th></tr>";
  //html += String("<tr><td>buf5</td><td>") + tempBuf5  + String("</td><td>") + FloatRound(deltaBuf5*100, 1)  + String("</td><td>") + rawBuf5 + String("</td><td>") + cntBadBuf5 + String("</td><td>") + cntConsBadBuf5 + String("</td></tr>");
  //html += String("<tr><td>buf2</td><td>") + tempBuf2  + String("</td><td>") + FloatRound(deltaBuf2*100, 1)  + String("</td><td>") + rawBuf2 + String("</td><td>") + cntBadBuf2 + String("</td><td>") + cntConsBadBuf2 + String("</td></tr>");
  //html += String("<tr><td>ww</td><td>") + tempWW  + String("</td><td>") + FloatRound(deltaWW*100, 1)  + String("</td><td>") + rawWW + String("</td><td>") + cntBadWW + String("</td><td>") + cntConsBadWW + String("</td></tr>");
  //html += "</table></p>";
  html += "<p>mqtt broker: " + String(MQTT_BROKER) + ", client: " + String(MQTT_CLIENT_ID) + ", topic: " + String(mqttTopicBufferTemp)  +
          "<br>Last Published: " + String(mqttLastPublishDate) + ", connected: " + String(mqttConnected) + "</p>";
  html += "<p><ul><li>mqtt pubs: " + String(cntMPub) + "</li><li>mqtt errors: " + String(cntMPubErr) + "</li><li>mqtt reconnects: " + String(cntMReCon) + 
          "</li><li>mqtt disconnects: " + String(cntMDisCon) + "</li>";
  //html += "<li>WiFi reconnects: " + cntWifiReConn + "</li>";
  html += "</ul></p>";
  html += "<p><a href=\"/info\">info</a> <a href=\"/ota\">ota</a> </p>";        
  html += "</body></html>";

  server.send(200, "text/html", html);
}

void handleInfo() {
  String html = "<!DOCTYPE html><html>";
  html += "<head><title>ESP32P4 lora info</title></head><body>";
  html += "<h1>hw info</h1> <ul> <li>esp32 p4 waveshare dev board</li> <li>lora SX1262 HF core board</li> </ul>";
  html += "<h1>sw info</h1> <ul> <li>arduino ide 2.3.7</li> <li>espressif 3.x</li> <li>RadioLib 7.5</li> </ul>";
  html += "<p> to build OTA update, in Arduino IDE, Menu Sketch, Export Compiled Binary, upload the esp32p4_eth_http_mqtt_ntp.ino.bin file";
  html += "<p>" + String(VERSION) + "</p>";
  html += "<p><a href=\"/\">Back to Home</a></p>";
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
  Serial.printf("\n\nstarting ---------- %s -------------\n", VERSION);

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
  Serial.println("Waiting for LoRa initialization...");
  while (!lora_initialized) {
    lora_initialized = initLoRa();
    if (!lora_initialized) {
      Serial.print("!l");
      delay(5000);  // Wait 5 seconds before retry
    }
  }
  Serial.println("LoRa init ok\n");

#if LAN == 1
  // ethernet setup
  Network.onEvent(onEthEvent);
  Serial.println("Starting Ethernet...");
  ETH.begin(); 
  Serial.println("Waiting for Ethernet connection...");
  while (!eth_connected) {
    delay(1000);
    Serial.print("!e");
  }
  ip = ETH.localIP();
  Serial.printf("\eth connected, ip: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
#endif  

#if WLAN == 1
  // Connect to Wi-Fi network with SSID and password
  Serial.printf("connecting to ssid: %s\n", ssid);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    //ledColor.r = 255;
    delay(1000);
    Serial.print("!w");
    cntWifiReConn++;
    delay(150);
  }
  ip = WiFi.localIP();
  Serial.printf("\nWiFi connected to: %s, ip: %d.%d.%d.%d\n", ssid, ip[0], ip[1], ip[2], ip[3]);
#endif

  // check mqtt and post data if not LORA
  xTaskCreatePinnedToCore(
      PostMQTT, "PostMqtt", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TPostMqtt,  /* Task handle. */
      0); /* Core where the task should run */


  xTaskCreatePinnedToCore(
      WatchDog, "TaskWatchDog", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TWatchDog,  /* Task handle. */
      1); /* Core where the task should run */

#if WLAN == 1
  xTaskCreatePinnedToCore(
      WifiCheckReconn, "TaskWifiCheckReconn", 10000,  NULL,  /* Task input parameter */
      0,  /* Priority of the task */  &TWifiCheckReconn,  /* Task handle. */
      1); /* Core where the task should run */
#endif

  // Create LoRa receive task on core 1
  xTaskCreatePinnedToCore(
    LoraReceiveTask,   // Function
    "LoRa RX",         // Name
    4096,              // Stack size
    NULL,              // Parameters
    1,                 // Priority
    NULL,              // Task handle
    1                  // Core (0 or 1)
  );

  // setup ntp
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.printf("ntp server: %s, gmtOff: %i s\n", ntpServer, gmtOffset_sec);
  getLocalTime(&timeinfo);
  Serial.println();
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  // start web server Define routes and start, next time you test, make sure run tests against the test board ip, NOT the prod one...
  server.on("/", handleRoot);
  server.on("/test", handleTest);
  server.on("/info", handleInfo);
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
  Serial.printf("HTTP web server started\n");

#if NTP == 1
  while (!getLocalTime(&timeinfo)) {
    Serial.println("Waiting for NTP time...");
    delay(500);
  }
#endif

  strftime(upTimeBuf, sizeof(upTimeBuf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  bootTimeStr = String(upTimeBuf);

#if MQTTenable == 1
  mqtt.begin(MQTT_BROKER, MQTT_PORT, network);
  connectMQTT();
#endif

}

void loop() {
  //Serial.println("Waiting for HTTP requests...");
  server.handleClient(); // WebServer client connection handling
  //delay(1000);  // Prevents flooding the serial monitor
}

/*
radio.setFrequency(868.0);       // Frequency in MHz
radio.setSpreadingFactor(7);     // 7-12 (higher = longer range, slower)
radio.setBandwidth(125.0);       // 125, 250, 500 kHz
radio.setCodingRate(5);          // 5-8 (higher = more error correction)
radio.setOutputPower(14);        // TX power in dBm (max 22 dBm for SX1262)
radio.setSyncWord(0x12);         // Private networks use 0x12, LoRaWAN uses 0x34
radio.setPreambleLength(8);      // Preamble length
*/
