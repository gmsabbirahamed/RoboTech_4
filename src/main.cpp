#if defined(ESP8266)
#include <ESP8266WiFi.h>
#define THINGSBOARD_ENABLE_PROGMEM 0
#elif defined(ESP32) || defined(RASPBERRYPI_PICO) || defined(RASPBERRYPI_PICO_W)
#include <WiFi.h>
#endif
//this is second Branch
#include "DHT.h"

#define Led_1 4 // fan light-----------------
#define Led_2 2
#define Led_3 14
#define Fan_1 16
#define Fan_2 17
#define Fan_3 18  // fan light----xxxxxxxxxxx-------------

#define Button_1 39
#define Button_2 36

#define Parking_1 19
#define Parking_2 26

#define motion_sensor 27
#define DHTPIN 25
#define fire_sensor 35     //     sensor ----------------
#define solar_sensor 32
#define ldr_pin 33
#define water_pin 34    //     sensor ------xxxxxxx-------

#define out_pin1 5
#define out_pin2 23

#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>

#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

constexpr char WIFI_SSID[] = "Ekhoni Reset Kore Dao";
constexpr char WIFI_PASSWORD[] = "ekhonghumabo";

constexpr char TOKEN[] = "X5OXExobQMEIEkx45Tsf";
constexpr char THINGSBOARD_SERVER[] = "thingsboard.cloud";
constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr size_t MAX_ATTRIBUTES = 20U;

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoardSized<Default_Fields_Amount, 100, MAX_ATTRIBUTES> tb(mqttClient, MAX_MESSAGE_SIZE);

constexpr char LED_1_STATE_ATTR[] = "led1State";
constexpr char LED_2_STATE_ATTR[] = "led2State";
constexpr char LED_3_STATE_ATTR[] = "led3State";
constexpr char FAN_1_STATE_ATTR[] = "fan1State";
constexpr char FAN_2_STATE_ATTR[] = "fan2State";
constexpr char FAN_3_STATE_ATTR[] = "fan3State";
constexpr char AUTO_STATE_ATTR[] = "autoState";

volatile bool attributesChanged = false;

volatile bool led1State = false; // LED1 state managed by push button and RPC
volatile bool led2State = false; // LED2 state managed by push button and RPC
volatile bool led3State = false; // LED3 state managed by push button and RPC
volatile bool fan1State = false; // FAN1 state managed by push button and RPC
volatile bool fan2State = false; // FAN2 state managed by push button and RPC
volatile bool fan3State = false; // FAN3 state managed by push button and RPC
volatile bool autoState = false; 

int parking1State = 0;
int parking2State = 0;
int parking99State = 1;

//volatile bool fireState = false;
volatile bool motionState = false;

bool Auto = false;//  ------------------For automation

constexpr int16_t telemetrySendInterval = 2000U;
uint32_t previousDataSend;

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

const bool reconnect() {
  const wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    return true;
  }

  InitWiFi();
  return true;
}

// RPC function to set led1 state
void processSetLed1Mode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set led state RPC method");
  int led1StateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(led1StateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (led1StateFromRPC != 0 && led1StateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  led1State = led1StateFromRPC;  // Update the LED state
  digitalWrite(Led_1, led1State ? LOW : HIGH);  // Update LED physically
  Serial.print("Led 1: ");
  Serial.println(led1State ? LOW : HIGH);
  attributesChanged = true;  // Mark attributes changed
  response_doc["sw1State"] = led1State;  // Return the current mode
  response.set(response_doc);
}

// RPC function to set led2 state
void processSetLed2Mode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set led state RPC method");
  int led2StateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(led2StateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (led2StateFromRPC != 0 && led2StateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  led2State = led2StateFromRPC;  // Update the LED state
  digitalWrite(Led_2, led2State ? HIGH : LOW);  // Update LED physically
  Serial.print("Led 2: ");
  Serial.println(led2State ? HIGH : LOW);
  attributesChanged = true;  // Mark attributes changed
  response_doc["sw2State"] = led2State;  // Return the current mode
  response.set(response_doc);
}

// RPC function to set led3 state
void processSetLed3Mode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set led state RPC method");
  int led3StateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(led3StateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (led3StateFromRPC != 0 && led3StateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  led3State = led3StateFromRPC;  // Update the LED state
  digitalWrite(Led_3, led3State ? HIGH : LOW);  // Update LED physically
  Serial.print("Led 3: ");
  Serial.println(led3State ? HIGH : LOW);
  attributesChanged = true;  // Mark attributes changed
  response_doc["sw3State"] = led3State;  // Return the current mode
  response.set(response_doc);
}

// RPC function to set fan1 state
void processSetFan1Mode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set fan state RPC method");
  int fan1StateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(fan1StateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (fan1StateFromRPC != 0 && fan1StateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  fan1State = fan1StateFromRPC;  // Update the FAN state
  digitalWrite(Fan_1, fan1State ? LOW : HIGH);  // Update FAN physically
  Serial.print("Fan 1: ");
  Serial.println(fan1State ? HIGH : LOW);
  attributesChanged = true;  // Mark attributes changed
  response_doc["sw4State"] = fan1State;  // Return the current mode
  response.set(response_doc);
}

// RPC function to set fan2 state
void processSetFan2Mode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set led state RPC method");
  int fan2StateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(fan2StateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (fan2StateFromRPC != 0 && fan2StateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  fan2State = fan2StateFromRPC;  // Update the FAN state
  digitalWrite(Fan_2, fan2State ? HIGH : LOW);  // Update FAN physically
  Serial.print("Fan 2: ");
  Serial.println(fan2State ? HIGH : LOW);
  attributesChanged = true;  // Mark attributes changed
  response_doc["sw5State"] = fan2State;  // Return the current mode
  response.set(response_doc);
}

// RPC function to set fan3 state
void processSetFan3Mode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set led state RPC method");
  int fan3StateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(fan3StateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (fan3StateFromRPC != 0 && fan3StateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  fan3State = fan3StateFromRPC;  // Update the FAN state
  digitalWrite(Fan_3, fan3State ? HIGH : LOW);  // Update FAN physically
  Serial.print("Fan 3: ");
  Serial.println(fan3State ? HIGH : LOW);
  attributesChanged = true;  // Mark attributes changed
  response_doc["sw6State"] = fan3State;  // Return the current mode
  response.set(response_doc);
}

// RPC function to set auto state
void processSetAutoMode(const JsonVariantConst &data, JsonDocument &response) {
  Serial.println("Received the set auto state RPC method");
  int autoStateFromRPC = data;
  Serial.print("Mode to change: ");
  Serial.println(autoStateFromRPC);
  StaticJsonDocument<1> response_doc;
  if (autoStateFromRPC != 0 && autoStateFromRPC != 1) {
    response_doc["error"] = "Unknown mode!";
    response.set(response_doc);
    return;
  }
  autoState = autoStateFromRPC;  // Update the FAN state

  attributesChanged = true;  // Mark attributes changed
  response_doc["sw7State"] = autoState;  // Return the current mode
  response.set(response_doc);
}

// Define RPC callback
const std::array<RPC_Callback, 7U> callbacks = {
  RPC_Callback{ "sw1State", processSetLed1Mode },
  RPC_Callback{ "sw2State", processSetLed2Mode },
  RPC_Callback{ "sw3State", processSetLed3Mode },
  RPC_Callback{ "sw4State", processSetFan1Mode },
  RPC_Callback{ "sw5State", processSetFan2Mode },
  RPC_Callback{ "sw6State", processSetFan3Mode },
  RPC_Callback{ "sw7State", processSetAutoMode }
};


void setup() {
  Serial.begin(SERIAL_DEBUG_BAUD);
  pinMode(Led_1, OUTPUT);
  pinMode(Led_2, OUTPUT);
  pinMode(Led_3, OUTPUT);
  pinMode(Fan_1, OUTPUT);
  pinMode(Fan_2, OUTPUT);
  pinMode(Fan_3, OUTPUT);
  pinMode(Button_1, INPUT_PULLUP);  // Set the button as input
  pinMode(Button_2, INPUT_PULLUP);  // Set the button as input
  pinMode(Parking_1, INPUT);
  pinMode(Parking_2, INPUT);
  pinMode(fire_sensor, INPUT);
  pinMode(motion_sensor, INPUT);
  pinMode(solar_sensor, INPUT);
  pinMode(water_pin, INPUT);
  pinMode(ldr_pin, INPUT);
  pinMode(out_pin1, OUTPUT);
  pinMode(out_pin2, OUTPUT);

  dht.begin();

  delay(1000);
  InitWiFi();
}

//////////////////////////////////////////////////////////////////////////////////////
void loop() {
  delay(10);

  int solar_power = analogRead(solar_sensor);
  int ldr = digitalRead(ldr_pin);
  int fireState = digitalRead(fire_sensor);
  motionState = digitalRead(motion_sensor);
  parking1State = digitalRead(Parking_1);
  parking2State = digitalRead(Parking_2);
  int water_value = analogRead(water_pin);

  float humidity = dht.readHumidity();
  float temperatureC = dht.readTemperature();
  if (isnan(humidity) || isnan(temperatureC)) {
    Serial.println("Failed to read from DHT sensor!");
    temperatureC = 99 ;
    //return;
  }
  

  if (!reconnect()) {
    return;
  }

  if (!tb.connected()) {
    Serial.print("Connecting to: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
      Serial.println("Failed to connect");
      return;
    }
    tb.sendAttributeData("macAddress", WiFi.macAddress().c_str());
    Serial.println("Subscribing for RPC...");
    if (!tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend())) {
      Serial.println("Failed to subscribe for RPC");
      return;
    }
  }

  /*if(WiFi.status() != WL_CONNECTED || !tb.connected()) {
    digitalWrite(out_pin1, 1);
  }else {
    digitalWrite(out_pin1, 0);
  }*/



  if(autoState == 1) {          //         automation +++++++++++++++++++++++++
    digitalWrite(Led_1, motionState);
  }
  
  if(autoState == 1) {
    if(temperatureC > 33) {
      digitalWrite(Fan_1, 1);
    } else if(temperatureC < 33) {
      digitalWrite(Fan_1, 0);
    }
  }

  // power consumption ..----------------------------------------------------------------
  int power_cons = 6;
  int power_adj = random(3,7);
  power_cons = power_cons + power_adj;
  if(Led_1 == LOW) {
    power_cons = power_cons + 3;
  }
  if(Led_2 == HIGH) {
    power_cons = power_cons + 3;
  }
  if(Led_3 == HIGH) {
    power_cons = power_cons + 3;
  }
  if(Fan_1 == LOW) {
    power_cons = power_cons + 4;
  }
  if(Fan_2 == HIGH) {
    power_cons = power_cons + 4;
  }
  if(Fan_3 == HIGH) {
    power_cons = power_cons + 4;
  }
  if(out_pin2 == HIGH) {
    power_cons = power_cons + 4;
  }
  if(out_pin1 == HIGH) {
    power_cons = power_cons + 4;
  }
  // power consumption ..-------------------------xxxxxxxxxxxxxxxxxx---------------------

  digitalWrite(out_pin2, ldr);
  digitalWrite(out_pin1, fireState);
  
  

  // Read button state and toggle LED if pressed
  if (digitalRead(Button_1) == LOW) {
    led1State = !led1State;  // Toggle LED state
    digitalWrite(Led_1, led1State ? LOW : HIGH);  // Update LED
    tb.sendTelemetryData(LED_1_STATE_ATTR, led1State);  // Send state to ThingsBoard
    tb.sendAttributeData(LED_1_STATE_ATTR, led1State);  // Sync state
    delay(500);  // Debounce delay
  }
  if (digitalRead(Button_2) == LOW) {
    fan1State = !fan1State;  // Toggle LED state
    digitalWrite(Fan_1, fan1State ? LOW : HIGH);  // Update LED
    tb.sendTelemetryData(FAN_1_STATE_ATTR, fan1State);  // Send state to ThingsBoard
    tb.sendAttributeData(FAN_1_STATE_ATTR, fan1State);  // Sync state
    delay(500);  // Debounce delay
  }

  // Sync real-time LED state with ThingsBoard if changed via RPC
  if (attributesChanged) {
    attributesChanged = false;
    tb.sendTelemetryData(LED_1_STATE_ATTR, led1State);
    tb.sendAttributeData(LED_1_STATE_ATTR, led1State);
    tb.sendTelemetryData(LED_2_STATE_ATTR, led2State);
    tb.sendAttributeData(LED_2_STATE_ATTR, led2State);
    tb.sendTelemetryData(LED_3_STATE_ATTR, led3State);
    tb.sendAttributeData(LED_3_STATE_ATTR, led3State);
    tb.sendTelemetryData(FAN_1_STATE_ATTR, fan1State);
    tb.sendAttributeData(FAN_1_STATE_ATTR, fan1State);
    tb.sendTelemetryData(FAN_2_STATE_ATTR, fan2State);
    tb.sendAttributeData(FAN_2_STATE_ATTR, fan2State);
    tb.sendTelemetryData(FAN_3_STATE_ATTR, fan3State);
    tb.sendAttributeData(FAN_3_STATE_ATTR, fan3State);
    tb.sendTelemetryData(AUTO_STATE_ATTR, autoState);
    tb.sendAttributeData(AUTO_STATE_ATTR, autoState);
  }

  // Send telemetry data every telemetrySendInterval time
  if (millis() - previousDataSend > telemetrySendInterval) {
    previousDataSend = millis();
    tb.sendTelemetryData("temperature", temperatureC);
    tb.sendAttributeData("rssi", WiFi.RSSI());
    tb.sendAttributeData("channel", WiFi.channel());
    tb.sendAttributeData("bssid", WiFi.BSSIDstr().c_str());
    tb.sendAttributeData("localIp", WiFi.localIP().toString().c_str());
    tb.sendAttributeData("ssid", WiFi.SSID().c_str());

    tb.sendTelemetryData("parking_1", parking1State);
    tb.sendTelemetryData("parking_2", parking2State);
    tb.sendTelemetryData("parking_99", parking99State);
    tb.sendTelemetryData("Solar_Panel", solar_power);
    tb.sendTelemetryData("Day", ldr);
    tb.sendTelemetryData("Motion", motionState);
    tb.sendTelemetryData("Fire", fireState);
    tb.sendTelemetryData("freequency", random(50, 60));
    tb.sendTelemetryData("voltage", random(232, 242));
    tb.sendTelemetryData("water", random(846, 860));
    tb.sendTelemetryData("water_P", water_value);
    tb.sendTelemetryData("power_consumption", power_cons);
  }

  tb.loop();
}

