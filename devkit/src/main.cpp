/*
 *
 * vim: sta:et:sw=4:ts=4:sts=4
 */
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>

#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "bsec.h"
#include "BluetoothSerial.h"

//BluetoothSerial SerialBT;

/* Configure the BSEC library with information about the sensor
   18v/33v = Voltage at Vdd. 1.8V or 3.3V
   3s/300s = BSEC operating mode, BSEC_SAMPLE_RATE_LP or BSEC_SAMPLE_RATE_ULP
   4d/28d = Operating age of the sensor in days
   generic_18v_3s_4d
   generic_18v_3s_28d
   generic_18v_300s_4d
   generic_18v_300s_28d
   generic_33v_3s_4d
   generic_33v_3s_28d
   generic_33v_300s_4d
   generic_33v_300s_28d
   */
const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

//#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
#define STATE_SAVE_PERIOD	UINT32_C(3 * 60 * 1000) // 360 minutes - 4 times a day

//#define SECRET_SSID ""
//#define SECRET_PASS ""
#ifndef SECRET_SSID
#include "arduino_secrets.h"
#endif

const char *ssid = SECRET_SSID;
const char *password = SECRET_PASS;

const char* mqttBroker = "test.mosquitto.org";
const int mqtt_port = 1883;  // Sets the server details.
const char* ntpServer = "ntp1.tecnico.ulisboa.pt";
//const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 3600;

const char willTopic[] = "ipfn/esp32/devkit/will";
const char inTopic[]   = "ipfn/esp32/devkit/in";
const char outTopic[]  = "ipfn/esp32/devkit/out";
String willPayload = "oh no!, esp32 lost Mqtt ";
bool willRetain = true;
int willQos = 1;
int subscribeQos = 1;

#define MSG_BUFFER_SIZE 50
int wifiOK = 0;
int wifiRetries = 0 ;
bool ntpOK = false;
bool relayState = false;

bool led_state = false;
bool led_blink = false;
int ledPeriod = 2 * 1000UL;

const int msgPeriod = 15 * 1000U;
const int wifiPeriod = 300 * 1000U;

#define WIFI_RETRY 20

// By default 'pool.ntp.org' is used with 60 seconds update interval and
WiFiClient wClient;
MqttClient mqttClient(wClient);

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

// Create an object of the class Bsec
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;

void onMqttMessage(int messageSize);

int setupMqtt(){
    mqttClient.stop();
    delay(20);
    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(mqttBroker);

    if (!mqttClient.connect(mqttBroker, mqtt_port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());
        return -1;
    }
    else
        Serial.println("You're connected to the MQTT broker!");

    // set the message receive callback
    mqttClient.onMessage(onMqttMessage);

    Serial.print("Subscribing to topic: ");
    Serial.println(inTopic);

    // the the library supports subscribing at QoS 0, 1, or 2
    mqttClient.subscribe(inTopic, subscribeQos);
    // mqttClient.unsubscribe(inTopic);
    return 0;
}
/*
   void setClockNtp(uint32_t timeout) {
   NTP.begin(ntpServer, "pool.ntp.org");
   NTP.waitSet(timeout);
   time_t now = time(nullptr);
   struct tm timeinfo;
   if(now < 8 * 3600 * 2){
   Serial.println("Could not get ntp time");
   }
   else{
   gmtime_r(&now, &timeinfo);
   Serial.print("Current ntp time: ");
   Serial.print(asctime(&timeinfo));
   }
   }
   */
void reconnectWifi() {
    delay(10);
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected() == 1)
        return;
    Serial.print("No link. Wifi "); Serial.print(WiFi.status());
    Serial.print(" MQTT "); Serial.println(mqttClient.connected());
    mqttClient.unsubscribe(inTopic);
    wifiOK = 0;
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Re-coonnecting to ");
    Serial.println(ssid);

    WiFi.disconnect();
    WiFi.begin(ssid, password);
    for(int i = 0; i < WIFI_RETRY; i++){
        if (WiFi.status() != WL_CONNECTED) {
            Serial.print(".");
            led_state = not led_state;
            digitalWrite(LED_BUILTIN, led_state);
            delay(500);
            //continue;
        }
        else {
            wifiOK = 1;
            led_state = true;
            //led_blink = true;
            ledPeriod = 2000UL;
            Serial.println("");
            Serial.print("WiFi connected, IP address: ");
            Serial.println(WiFi.localIP());
            digitalWrite(LED_BUILTIN, led_state);
            if(setupMqtt() == 0)
                wifiRetries = 0;
            break;
        }
    }
}

// Entry point for the example
void setup(void)
{
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
    Serial.begin(115200);
    //SerialBT.begin("ESP32test"); //Bluetooth device name

    Wire.begin();

    iaqSensor.begin(BME680_I2C_ADDR_PRIMARY, Wire);
    output = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();

    iaqSensor.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();

    loadState();

    bsec_virtual_sensor_t sensorList1[2] = {
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
    };

    iaqSensor.updateSubscription(sensorList1, 2, BSEC_SAMPLE_RATE_ULP);
    checkIaqSensorStatus();

    bsec_virtual_sensor_t sensorList2[5] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList2, 5, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();

    WiFi.mode(WIFI_STA);
    reconnectWifi();
    //setClockNtp(10000);
    // Init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    //printLocalTime();
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        //return;
    }
    else {
        Serial.print("Current ntp time: ");
        Serial.println(asctime(&timeinfo));
    }


    // Print the header
    //output = "Timestamp [ms], raw temperature [°C], pressure [hPa], raw relative humidity [%], gas [Ohm], IAQ, IAQ accuracy, temperature [°C], relative humidity [%]";
    //Serial.println(output);
}
void print_bme680(unsigned long time){
    /* Message format all floats, except pressure, iaq (uint_8)
       time, temperature, pressure, humidity, co2Equivalent, iaq, iaqAccuracy, staticIaq, rawTemperature 
       rawHumidity,  rawHumidity */
    output = "1 " + String(time,DEC);
    output += ", " + String(iaqSensor.temperature, 1);
    output += ", " + String(iaqSensor.pressure,0);
    output += ", " + String(iaqSensor.humidity,1);
    output += ", " + String(iaqSensor.co2Equivalent,3);
    output += ", " + String(iaqSensor.breathVocEquivalent,3);
    output += ", " + String(iaqSensor.iaq); //, DEC)
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.rawHumidity);
    Serial.println(output);
    //SerialBT.println(output);

}
// Function that is looped forever
void loop(void)
{
    static  unsigned long lastMsg = 0;
    static  unsigned long lastWifiCheck = 0;
    static  unsigned long lastLed = 0;
    char msg[MSG_BUFFER_SIZE];

    StaticJsonDocument<256> doc;
    struct tm timeinfo;

    // call poll() regularly to allow the library to receive MQTT messages and
    // send MQTT keep alives which avoids being disconnected by the broker
    mqttClient.poll();

    unsigned long time_trigger = millis();
    unsigned long nowMs = millis();
    if (nowMs - lastMsg > msgPeriod) {
        lastMsg = nowMs;

        if(!getLocalTime(&timeinfo)) {
            Serial.println("Failed to obtain time");
            //return;
        }
        else {
            Serial.print("Current ntp time: ");
            Serial.println(asctime(&timeinfo));
        }
        if (iaqSensor.run()) { // If new data is available
            //print_bme680(time_trigger);

            output = String(time_trigger);
            output += ", " + String(iaqSensor.temperature);
            output += ", " + String(iaqSensor.humidity);
            output += ", " + String(iaqSensor.pressure);

            output += ", " + String(iaqSensor.gasResistance);
            output += ", " + String(iaqSensor.iaq);
            output += ", " + String(iaqSensor.iaqAccuracy);
            output += ", " + String(iaqSensor.rawTemperature);
            output += ", " + String(iaqSensor.rawHumidity);
            Serial.println(output);
            //SerialBT.println(output);

            //Serial.print("iaq= ");
            //Serial.println(iaqSensor.iaq);

            doc["temperature"] = iaqSensor.temperature;
            doc["humidity"] = iaqSensor.humidity;
            doc["pressure"] = iaqSensor.pressure;
            //doc[""] = iaqSensor.;
            //doc["payload"]   = payload;
            bool retained = false; int qos = 1; // bool dup = false;
            mqttClient.beginMessage(outTopic, (unsigned long) measureJson(doc), retained, qos, false);
            serializeJson(doc, mqttClient);
            mqttClient.endMessage();

            updateState();
        } else {
            checkIaqSensorStatus();
        }
    }
}

// Helper function definitions
void checkIaqSensorStatus(void)
{
    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
            output = "BSEC error code : " + String(iaqSensor.status);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        } else {
            output = "BSEC warning code : " + String(iaqSensor.status);
            Serial.println(output);
        }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
            output = "BME680 error code : " + String(iaqSensor.bme680Status);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        } else {
            output = "BME680 warning code : " + String(iaqSensor.bme680Status);
            Serial.println(output);
        }
    }
}

void errLeds(void)
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
}

void loadState(void)
{
    if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
        // Existing state in EEPROM
        Serial.println("Reading state from EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            bsecState[i] = EEPROM.read(i + 1);
            Serial.println(bsecState[i], HEX);
        }

        iaqSensor.setState(bsecState);
        checkIaqSensorStatus();
    } else {
        // Erase the EEPROM with zeroes
        Serial.println("Erasing EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
    }
}

void updateState(void)
{
    bool update = false;
    if (stateUpdateCounter == 0) {
        /* First state update when IAQ accuracy is >= 3 */
        if (iaqSensor.iaqAccuracy >= 3) {
            update = true;
            stateUpdateCounter++;
        }
    } else {
        /* Update every STATE_SAVE_PERIOD minutes */
        if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
            update = true;
            stateUpdateCounter++;
        }
    }

    if (update) {
        iaqSensor.getState(bsecState);
        checkIaqSensorStatus();

        Serial.println("Writing state to EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
            EEPROM.write(i + 1, bsecState[i]);
            Serial.println(bsecState[i], HEX);
        }

        EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
        EEPROM.commit();
    }
}

void onMqttMessage(int messageSize) {
    StaticJsonDocument<256> doc;

    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', duplicate = ");
    Serial.print(mqttClient.messageDup() ? "true" : "false");
    Serial.print(", QoS = ");
    Serial.print(mqttClient.messageQoS());
    Serial.print(", retained = ");
    Serial.print(mqttClient.messageRetain() ? "true" : "false");
    Serial.print("', length ");
    Serial.print(messageSize);
    //Serial.print(", msg:");

    deserializeJson(doc, mqttClient);

}
