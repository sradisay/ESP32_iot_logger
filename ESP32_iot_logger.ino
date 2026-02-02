#include <Arduino.h>
#include <WiFi.h>
#include <WiFiProv.h>
#include <WiFiUdp.h>
#include <time.h>

#define LED_PIN         2       
#define BUTTON_PIN      0       
#define ANALOG_PIN      34      
#define UDP_PORT        4444
#define SERVICE_NAME    "ESP32_LOGGER"
#define POP             NULL    

WiFiUDP udp;
bool isServerConfigured = false;
IPAddress serverIP;

unsigned long lastSensorRead = 0;
unsigned long lastBeaconSend = 0;
unsigned long buttonPressTime = 0;
bool buttonHeld = false;
const unsigned long SENSOR_INTERVAL = 3000; 
const unsigned long BEACON_INTERVAL = 2000; 

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;      
const int   daylightOffset_sec = 0;

void sysProvEvent(arduino_event_t *sys_event);
void checkResetButton();
String getFormattedTime();

void setup() {
    Serial.begin(115200);
    
    delay(1000);

    WiFi.mode(WIFI_STA);
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(ANALOG_PIN, INPUT);

    WiFi.onEvent(sysProvEvent);

    Serial.println("\n--- ESP32 Logger Booting ---");

    WiFiProv.beginProvision(
        NETWORK_PROV_SCHEME_BLE, 
        NETWORK_PROV_SCHEME_HANDLER_FREE_BTDM, 
        NETWORK_PROV_SECURITY_1, 
        POP, 
        SERVICE_NAME
    );
}

void loop() {
    checkResetButton();

    if (WiFi.status() != WL_CONNECTED) {
        digitalWrite(LED_PIN, millis() % 1000 < 500 ? HIGH : LOW); 
        return; 
    } else {
        digitalWrite(LED_PIN, HIGH); 
    }

    int packetSize = udp.parsePacket();
    if (packetSize) {
        char packetBuffer[255];
        int len = udp.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;
        
        String msg = String(packetBuffer);

        if (msg.startsWith("CONFIG_SET_IP:")) {
            String ipStr = msg.substring(14); 
            serverIP.fromString(ipStr);
            isServerConfigured = true;
            Serial.print("Server IP Configured: "); Serial.println(serverIP);
        }
    }

    if (!isServerConfigured && (millis() - lastBeaconSend > BEACON_INTERVAL)) {
        udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_PORT); 
        String beacon = "LOGGER_BEACON:" + WiFi.macAddress();
        udp.print(beacon);
        udp.endPacket();
        Serial.println("Broadcasting Beacon...");
        lastBeaconSend = millis();
    }

    if (isServerConfigured && (millis() - lastSensorRead > SENSOR_INTERVAL)) {
        int analogVal = analogRead(ANALOG_PIN);
        String timestamp = getFormattedTime();
        String payload = "LOG:" + timestamp + "," + String(analogVal);

        udp.beginPacket(serverIP, UDP_PORT);
        udp.print(payload);
        udp.endPacket();
        Serial.printf("Sent Data: %s\n", payload.c_str());
        lastSensorRead = millis();
    }
}


void sysProvEvent(arduino_event_t *sys_event) {
    switch (sys_event->event_id) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.print("\nConnected! IP: ");
            Serial.println(WiFi.localIP());
            udp.begin(UDP_PORT); 
            configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
            break;
            
        case ARDUINO_EVENT_PROV_START:
            Serial.println("\nProvisioning Started. Please use Espressif BLE App.");
            break;

        case ARDUINO_EVENT_PROV_CRED_RECV:
            Serial.println("\nCredentials Received...");
            break;
            
        case ARDUINO_EVENT_PROV_END:
            Serial.println("\nProvisioning Session Ended.");
            break;
            
        default:
            break;
    }
}

void checkResetButton() {
    int btnState = digitalRead(BUTTON_PIN);
    if (btnState == LOW) {
        if (!buttonHeld) {
            buttonHeld = true;
            buttonPressTime = millis();
        }
        if ((millis() - buttonPressTime) > 10000) {
            Serial.println("\nFactory Reset Triggered! Erasing WiFi Config...");
            for(int i=0; i<5; i++) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
                delay(100);
            }
            WiFi.disconnect(true, true); 
            delay(500);
            Serial.println("Restarting...");
            ESP.restart();
        }
    } else {
        buttonHeld = false;
    }
}

String getFormattedTime() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
        return "N/A";
    }
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}