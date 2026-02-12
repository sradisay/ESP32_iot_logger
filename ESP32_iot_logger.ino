#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DFRobot_ADS1115.h> // Ensure you have the DFRobot_ADS1115 library installed

// --- I2C ADS1115 Setup ---
DFRobot_ADS1115 ads(&Wire);
#define I2C_SDA 21
#define I2C_SCL 22

#define LED_PIN         2     
#define RESET_PIN       0     
#define RESET_TIME_MS   5000  
#define SAMPLE_RATE_MS  100   
#define BATCH_SIZE      50    

Preferences preferences;
WebServer server(80);
DNSServer dnsServer;
WiFiClient espClient;
PubSubClient client(espClient);

volatile float acquisitionBuffer[BATCH_SIZE]; 
volatile int   acquisitionIndex = 0;
volatile bool  bufferFull = false;

float txBuffer[BATCH_SIZE]; 

TaskHandle_t TaskSensor;
portMUX_TYPE bufferMux = portMUX_INITIALIZER_UNLOCKED;

struct Config {
  String ssid;
  String pass;
  String mqtt_server;
  int    mqtt_port;
  String device_id;
} config;

const char* HTML_FORM = R"(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:sans-serif;padding:20px;background:#f4f4f9;}
  form{background:#fff;padding:20px;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,0.1);}
  input{display:block;width:100%;margin:10px 0;padding:12px;box-sizing:border-box;border:1px solid #ddd;border-radius:4px;}
  label{font-weight:bold;margin-top:10px;display:block;}
  button{width:100%;padding:15px;background:#28a745;color:white;border:none;border-radius:4px;font-size:16px;cursor:pointer;}
  button:hover{background:#218838;}
  h2{text-align:center;color:#333;}
</style>
</head><body>
<h2>Logger Provisioning</h2>
<form action="/save" method="POST">
  <label>WiFi SSID</label><input name="s" placeholder="Network Name">
  <label>WiFi Password</label><input name="p" type="password" placeholder="Password">
  <label>MQTT Host</label><input name="m" placeholder="emqx.sradisay.dev" value="emqx.sradisay.dev">
  <label>MQTT Port</label><input name="o" value="1883">
  <label>Device ID</label><input name="d" placeholder="logger-001">
  <button type="submit">Save Configuration</button>
</form></body></html>
)";

void sensorTask(void * pvParameters) {
  // Initialize I2C for the specific core
  Wire.begin(I2C_SDA, I2C_SCL);
  
  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0); // 0x48
  ads.setGain(eGAIN_TWOTHIRDS);              // Necessary for the 0-10V module range
  ads.setMode(eMODE_SINGLE);                 // Single-shot mode for stability
  
  // Change the check to this:
  ads.init(); 
  Serial.println("ADC Initialized");

  // Optional: Add a small delay to let the chip settle
  vTaskDelay(pdMS_TO_TICKS(100)); 
  
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_RATE_MS);

  for(;;) {
    // Read from the external I2C module (returns mV)
    // We convert to V (0-10V)
    float voltage = ads.readVoltage(0) / 1000.0;

    portENTER_CRITICAL(&bufferMux);
    if (acquisitionIndex < BATCH_SIZE) {
      acquisitionBuffer[acquisitionIndex] = voltage;
      acquisitionIndex++;
    } else {
      bufferFull = true; 
    }
    portEXIT_CRITICAL(&bufferMux);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ... (checkFactoryReset and startSoftAP functions remain unchanged) ...
void checkFactoryReset() {
  static unsigned long pressStart = 0;
  static bool pressed = false;
  if (digitalRead(RESET_PIN) == LOW) {
    if (!pressed) { pressed = true; pressStart = millis(); Serial.println("Button Pressed..."); }
    if (millis() - pressStart > 2000) { digitalWrite(LED_PIN, (millis() / 100) % 2); }
    if (millis() - pressStart > RESET_TIME_MS) {
      Serial.println("!!! FACTORY RESET !!!");
      preferences.begin("logger_conf", false);
      preferences.clear();
      preferences.end();
      digitalWrite(LED_PIN, HIGH);
      delay(2000);
      ESP.restart();
    }
  } else { pressed = false; digitalWrite(LED_PIN, LOW); }
}

void startSoftAP() {
  Serial.println("Starting SoftAP Provisioning Mode...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Logger_Setup"); 
  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/", HTTP_GET, []() { server.send(200, "text/html", HTML_FORM); });
  server.on("/save", HTTP_POST, []() {
    String s = server.arg("s");
    String m = server.arg("m");
    String d = server.arg("d");
    if(s.length() > 0 && m.length() > 0 && d.length() > 0) {
      preferences.begin("logger_conf", false);
      preferences.putString("ssid", s);
      preferences.putString("pass", server.arg("p"));
      preferences.putString("mqtt", m);
      preferences.putInt("port", server.arg("o").toInt());
      preferences.putString("dev_id", d);
      preferences.end();
      server.send(200, "text/html", "<h1>Saved! Device Rebooting...</h1>");
      delay(1000);
      ESP.restart();
    } else { server.send(400, "text/plain", "Error: Missing Fields"); }
  });
  server.onNotFound([]() { server.send(200, "text/html", HTML_FORM); });
  server.begin();
  while(true) { dnsServer.processNextRequest(); server.handleClient(); checkFactoryReset(); delay(10); }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(RESET_PIN, INPUT_PULLUP);

  preferences.begin("logger_conf", true);
  config.ssid = preferences.getString("ssid", "");
  config.pass = preferences.getString("pass", "");
  config.mqtt_server = preferences.getString("mqtt", "");
  config.mqtt_port = preferences.getInt("port", 1883);
  config.device_id = preferences.getString("dev_id", "");
  preferences.end();

  if (config.ssid == "" || config.mqtt_server == "") {
    startSoftAP(); 
  }

  // Create the task on Core 1 (Core 0 is usually busy with WiFi)
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 1, &TaskSensor, 1);

  WiFi.begin(config.ssid.c_str(), config.pass.c_str());
  client.setServer(config.mqtt_server.c_str(), config.mqtt_port);
  client.setBufferSize(BATCH_SIZE * 4 + 100); 
}

void reconnect() {
  static unsigned long lastAttempt = 0;
  if (millis() - lastAttempt > 5000) {
    lastAttempt = millis();
    Serial.print("Connecting MQTT...");
    if (client.connect(config.device_id.c_str())) {
      Serial.println("Connected");
      digitalWrite(LED_PIN, HIGH); 
    } else {
      Serial.print("Failed rc=");
      Serial.println(client.state());
    }
  }
}

void loop() {
  checkFactoryReset();

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) reconnect();
    client.loop();
  }

  bool readyToSend = false;
  
  portENTER_CRITICAL(&bufferMux);
  if (bufferFull) {
    memcpy(txBuffer, (void*)acquisitionBuffer, sizeof(acquisitionBuffer));
    acquisitionIndex = 0; 
    bufferFull = false;   
    readyToSend = true;
  }
  portEXIT_CRITICAL(&bufferMux);

  if (readyToSend && client.connected()) {
    String topic = "data/" + config.device_id + "/batch";
    
    // Transmitting the array of floats (4 bytes each)
    client.publish(topic.c_str(), (uint8_t*)txBuffer, sizeof(txBuffer));
    
    Serial.printf("Sent %d I2C samples (0-10V scale) to %s\n", BATCH_SIZE, topic.c_str());
    
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
  }
}
