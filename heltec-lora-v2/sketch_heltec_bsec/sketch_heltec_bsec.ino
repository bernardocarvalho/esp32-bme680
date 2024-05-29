/*
arduino-cli
compile:
 arduino-cli compile --fqbn heltec:esp32:heltec_wifi_lora_32_V2 sketch_heltec_bsec.ino
 arduino-cli upload -p /dev/ttyUSB0 --fqbn heltec:esp32:heltec_wifi_lora_32_V2 sketch_heltec_bsec.ino1

 arduino-cli monitor  -p /dev/ttyUSB0 -c baudrate=115200
 
 mosquitto_sub -h "test.mosquitto.org" -p 1883 -t "ipfn/heltecV2/#"

Used libraries            Version  Path
NTPClient               3.2.1    /home/bernardo/Arduino/libraries/NTPClient
WiFi                    2.0.0    /home/bernardo/Arduino/hardware/heltec/esp32/libraries/WiFi
ArduinoMqttClient       0.1.8    /home/bernardo/Arduino/libraries/ArduinoMqttClient
ArduinoJson             7.0.4    /home/bernardo/Arduino/libraries/ArduinoJson
Heltec ESP32 Dev-Boards 2.0.2    /home/bernardo/Arduino/libraries/Heltec_ESP32_Dev-Boards
Wire                    2.0.0    /home/bernardo/Arduino/hardware/heltec/esp32/libraries/Wire
EEPROM                  2.0.0    /home/bernardo/Arduino/hardware/heltec/esp32/libraries/EEPROM
BSEC Software Library   1.8.1492 /home/bernardo/Arduino/libraries/BSEC_Software_Library
SPI                     2.0.0    /home/bernardo/Arduino/hardware/heltec/esp32/libraries/SPI

Print Current UNIX Time
date +%s

1268727836
Convert Epoch To Current Time
date -d @1268727836
date -d "1970-01-01 1268727836 sec GMT"
 *
 * this project also realess in GitHub:
 */

#include "Arduino.h"
#include <NTPClient.h>
#include "WiFi.h"
#include <WiFiUdp.h>
#include <ArduinoMqttClient.h>
// #include <MQTT.h>
//MQTTClient mqttClient;
#include <WiFiMulti.h>
#include <ArduinoJson.h>


//#include "images.h"

//#include "LoRaWan_APP.h"
//#include <Wire.h>  
#include "HT_SSD1306Wire.h"

#include <EEPROM.h>
#include "bsec.h"

#include "arduino_secrets.h"

SSD1306Wire  oled_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
WiFiUDP ntpUDP;

// By default 'pool.ntp.org' is used with 60 seconds update interval and
// no offset
NTPClient timeClient(ntpUDP);

// You can specify the time server pool and the offset, (in seconds)
// additionally you can specify the update interval (in milliseconds).
// NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600, 60000);


const char willTopic[] = "ipfn/heltecv2/will";
const char inTopic[]   = "ipfn/heltecV2/in";
const char outTopic[]  = "ipfn/heltecV2/bme680";
String willPayload = "oh no!, Heltec V2 lost Mqtt Connection";
const bool willRetain = true;
const int willQos = 1;
const int subscribeQos = 1;

unsigned int bme68x_errors = 0; 
unsigned int bme68x_warnings = 0; 


#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

#define bme68x_I2C_ADDR 0x77

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

Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;
// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

// Helper function definitions
void print_bme680(unsigned long time);
void oled_bme680(unsigned long time);
void mqtt_pub_bme680(unsigned long time);

WiFiMulti wifiMulti;
const char* mqttBroker = "test.mosquitto.org";
const int mqttPort = 1883;  // Sets the server details.

void OledClear(int x, int y, int dx, int dy) {
    oled_display.setColor(BLACK);
    oled_display.fillRect(x, y, dx, dy);
    oled_display.setColor(WHITE);
}
/*
float round_dig(float val, int digits){

}
*/ 

uint8_t connectMultiWiFi()
{
    // https://github.com/khoih-prog/WiFiMulti_Generic/blob/main/examples/WiFiMulti/WiFiMulti.ino 
    // #define WIFI_MULTI_1ST_CONNECT_WAITING_MS           500L 
#define WIFI_MULTI_CONNECT_WAITING_MS                   500L

    WiFi.disconnect();

    int i = 0;

    uint8_t status = wifiMulti.run();

    while ( ( i++ < 20 ) && ( status != WL_CONNECTED ) )
    {
        status = WiFi.status();

        if ( status == WL_CONNECTED )
            break;
        else
            delay(WIFI_MULTI_CONNECT_WAITING_MS);
    }
    if ( status == WL_CONNECTED )
    {
        Serial.println(F("WiFi connected after time: ")); //, i);
        Serial.print(F("SSID: ")); //, WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
        Serial.print(WiFi.SSID());//, F(",RSSI="), WiFi.RSSI());
        Serial.print(F(", RSSI="));//, WiFi.RSSI());
        Serial.print(WiFi.RSSI());

        Serial.print(F(", Channel: "));// , WiFi.channel(), F(",IP address:"), WiFi.localIP() );
        Serial.println(WiFi.channel()); //, F(",IP address:"), WiFi.localIP() );
        timeClient.begin();
    }
    else
    {
        Serial.println(F("WiFi not connected"));

        if (wifiMulti.run() != WL_CONNECTED)
        {
            Serial.println("WiFi not connected!");
            delay(1000UL);
        }
    }

    return status;
}


void mqttConnect(void)
{
    char buff[20];

    uint8_t status ;
    if ( (WiFi.status() != WL_CONNECTED) || (WiFi.RSSI() == 0) )
    {
        OledClear(0, 0, 80, 10);
        Serial.println(F("\nWiFi lost. Call connectMultiWiFi"));
        status = connectMultiWiFi();
    }
    Serial.println("Check  Wifi...");
    // if(wifiMulti.run() == WL_CONNECTED) {
    if(status == WL_CONNECTED) {
        OledClear(0, 0, 80, 10);
        IPAddress ip = WiFi.localIP();
        sprintf(buff, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        //sprintf(buff, "Ip:%s", WiFi.localIP());
        oled_display.drawString(0, 0, buff);
        //oled_display.drawString(100, 0, "WOK");
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("\nconnecting...");
        //while (!client.connect("arduino", "public", "public")) {
        //while (!client.connect("heltecV2")) { //, "public", "public")) {

        mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
        mqttClient.print(willPayload);
        mqttClient.endWill();
        for (int i = 0; i < 10; i++ ) {
            //if (mqttClient.connect("heltecV2")) {
            if (mqttClient.connect(mqttBroker, mqttPort)) {
                sprintf(buff, "MQTT:%s Connected", mqttBroker);
                // Serial.print(mqttBroker);
                Serial.println(buff);
                break;
            }
            //                        { //, "public", "public")) {
            Serial.print(".");
            delay(1000UL);
        }

        }
            else {
                oled_display.drawString(0, 0, "No Wifi Connection");
            }

            oled_display.display();
        }

        void WIFISetUp(void)
        {
            char buff[20];
            // Set WiFi to station mode and disconnect from an AP if it was previously connected
            WiFi.disconnect(true);
            delay(100);
            //     Add list of wifi networks
            wifiMulti.addAP(SECRET_SSID0,SECRET_PASS0);
            wifiMulti.addAP("TP-Link_37E8","87936148");
            WiFi.mode(WIFI_STA);
            //WiFi.setAutoConnect(true);
            delay(100);
            // WiFi.scanNetworks will return the number of networks found
            int n = WiFi.scanNetworks();
            Serial.println("scan done");
            if (n == 0) {
                Serial.println("no networks found");
                oled_display.drawString(0, 0, "No Wifi Connection");
            } 
            else {
                oled_display.drawString(0, 0, "Connecting...");
                oled_display.display();
                Serial.print(n);
                Serial.println(" networks found");
                for (int i = 0; i < n; ++i) {
                    // Print SSID and RSSI for each network found
                    Serial.print(i + 1);
                    Serial.print(": ");
                    Serial.print(WiFi.SSID(i));
                    Serial.print(" (");
                    Serial.print(WiFi.RSSI(i));
                    Serial.print(")");
                    Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
                    delay(10);
                }
            }

        }


void setup()
{
    //VextON();
    delay(100);
    oled_display.init();
    oled_display.setFont(ArialMT_Plain_10);
    //oled_display.display();
    //logo();
    delay(100); 
    oled_display.clear();
    Wire.begin(SDA_OLED, SCL_OLED); //Scan OLED's I2C address via I2C0
                                    //Wire1.begin(SDA, SCL);        //If there have other device on I2C1, scan the device address via I2C1

    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length

    Serial.begin(115200);

    WIFISetUp();
    //mqttConnect();

    iaqSensor.begin(bme68x_I2C_ADDR, Wire);
    output = "0, BSEC library version " + String(iaqSensor.version.major) + "." +
        String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);

    iaqSensor.setConfig(bsec_config_iaq); 
    checkIaqSensorStatus();

    loadState();

    Serial.println("0, bme68x 0x77 OK");

    bsec_virtual_sensor_t sensorList[10] = {
        BSEC_OUTPUT_RAW_TEMPERATURE,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_HUMIDITY,
        BSEC_OUTPUT_RAW_GAS,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    };

    iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP);
    checkIaqSensorStatus();

    //chipid=ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
    //Serial.printf("ESP32ChipID=%04X",(uint16_t)(chipid>>32));//print High 2 bytes
    //Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.

    //attachInterrupt(0,interrupt_GPIO0,FALLING);
    //oled_display.drawString(0, 10, packet);
    delay(100);
    //oled_display.clear();
    oled_bme680_init();
    oled_display.display();
    pinMode(LED ,OUTPUT);
    digitalWrite(LED, LOW);  
}

void loop(void)
{
    static bool led_status = false;
    static  unsigned long lastMsg = 0;
    const int mqttPeriod = 5 * 1000U;

    unsigned long now = millis();

    if (iaqSensor.run()) { // If new data is available
        print_bme680(now);
        oled_bme680(now);
        //mqttClient.beginMessage(outTopic);
        //mqttClient.print("hello ");
        // mqttClient.endMessage();
        updateState();
        if ( (WiFi.status() != WL_CONNECTED) ) {
            mqttConnect();

        }
        if ( WiFi.status() == WL_CONNECTED ) {
            timeClient.update();
            mqttClient.poll();
            if (now > lastMsg + mqttPeriod) {
                lastMsg = now;
                Serial.println(timeClient.getFormattedTime());
                // call poll() regularly to allow the library to send MQTT keep alives which
                // avoids being disconnected by the broker
                mqtt_pub_bme680(now);
            }
        }
        digitalWrite(LED, led_status);  
        led_status = not led_status;
    } else {
        checkIaqSensorStatus();
    }

}

void print_bme680(unsigned long time){
    char buff[80];
    sprintf(buff, "E:%u W:%u s:%u T:%5.1f P:%5.0f H:%6.1f BVE:%4.1f I:%4.1f", bme68x_errors, bme68x_warnings, time/1000, iaqSensor.temperature,
            iaqSensor.pressure/100, iaqSensor.humidity, iaqSensor.breathVocEquivalent, iaqSensor.iaq);
    Serial.println(buff);
    output += ", " + String(iaqSensor.rawTemperature);
    //output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    //output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
   // output += ", " + String(iaqSensor.temperature);
    //output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    //output += ", " + String(iaqSensor.breathVocEquivalent);
}

void oled_bme680_init(){
     //oled_display.setColor(BLACK);
    char buff[40];
    oled_display.setColor(WHITE);
    oled_display.setTextAlignment(TEXT_ALIGN_LEFT);
    //oled_display.drawString(0, 11, 
    sprintf(buff, "Temp Press  Hum  Voc");
    //oled_display.drawString(0, 11, "Temp Press Hum   Voc");
    oled_display.drawString(0, 10, buff);
    sprintf(buff, "Iaq   IaqAccuracy");
    oled_display.drawString(0, 34, buff);
//    oled_display.drawString(10, 11, "Temp");
//    oled_display.drawString(40, 11, "Press");
//    oled_display.drawString(80, 11, "HumVoc");
//    oled_display.drawString(120, 11, "Voc");
}

void mqtt_pub_bme680(unsigned long time){
    StaticJsonDocument<256> doc;
    bool retained = false;
    int qos = 1;
    char buff[20];

    sprintf(buff, "%5.2f", iaqSensor.temperature);
    //doc["Temp"] = iaqSensor.temperature;
    doc["EpochTime"] = timeClient.getEpochTime();
    doc["Temp"] = buff; // iaqSensor.temperature;
    sprintf(buff, "%6.1f", iaqSensor.pressure/100);
    doc["Press"] = buff; //iaqSensor.pressure/100;
    sprintf(buff, "%4.1f", iaqSensor.humidity);
    doc["Humid"] = buff; //iaqSensor.humidity;
    mqttClient.beginMessage(outTopic,  (unsigned long) measureJson(doc), retained, qos, false);
    serializeJson(doc, mqttClient);
    mqttClient.endMessage();
}

void oled_bme680(unsigned long time){
    char buff[40];
    OledClear(0, 22, 128, 11);
    OledClear(0, 50, 128, 14);
    //oled_display.setColor(BLACK);
    //oled_display.fillRect(0, 22, 128, 11);
    //oled_display.fillRect(0, 50, 128, 14);
    oled_display.setTextAlignment(TEXT_ALIGN_LEFT);
    sprintf(buff, "%5.1f %5.0f %6.1f %6.1f", iaqSensor.temperature, iaqSensor.pressure/100,
            iaqSensor.humidity, iaqSensor.breathVocEquivalent);
    oled_display.drawString(0, 22, buff);
    // oled_display.drawString(20, 22, String(iaqSensor.temperature) );
    // oled_display.drawString(60, 22, String(iaqSensor.pressure/1e2) );
    //oled_display.drawString(100, 22, String(iaqSensor.humidity) );
    sprintf(buff, "%4.1f %5.0f", iaqSensor.iaq, iaqSensor.iaqAccuracy);
    oled_display.drawString(10, 50, buff);
    //oled_display.drawString(20, 44, String(iaqSensor.breathVocEquivalent) );
    //oled_display.drawString(60, 44, String(iaqSensor.iaq) );
    // oled_display.drawString(100, 50, String(iaqSensor.iaqAccuracy) );

    oled_display.display();
}

void checkIaqSensorStatus(void)
{
    OledClear(100, 0, 27, 11);
    if (iaqSensor.bsecStatus != BSEC_OK) {
        if (iaqSensor.bsecStatus < BSEC_OK) {
            output = "0, BSEC error code : " + String(iaqSensor.bsecStatus);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        } else {
            output = "0, BSEC warning code : " + String(iaqSensor.bsecStatus);
            Serial.println(output);
        }
    }

    if (iaqSensor.bme68xStatus != BME68X_OK) {
        oled_display.drawString(100, 0, "NOK");
        if (iaqSensor.bme68xStatus < BME68X_OK) {
            output = "0, bme68x error code : " + String(iaqSensor.bme68xStatus);
            Serial.println(output);
            bme68x_errors++;
            //for (;;)
            errLeds(); /* Halt in case of failure */
        } else {
            output = "0, bme68x warning code : " + String(iaqSensor.bme68xStatus);
            bme68x_warnings++;
            Serial.println(output);
        }
        oled_display.drawString(100, 0, "BNOK");
        delay(2000);
    }
    else
    {
        oled_display.drawString(100, 0, "BOK ");
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
        Serial.println("0, Reading state from EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
            bsecState[i] = EEPROM.read(i + 1);
            //Serial.println(bsecState[i], HEX);
        }

        iaqSensor.setState(bsecState);
        checkIaqSensorStatus();
    } else {
        // Erase the EEPROM with zeroes
        Serial.println("0, Erasing EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
            EEPROM.write(i, 0);

        EEPROM.commit();
    }
}

void updateState(void)
{
    bool update = false;
    /* Set a trigger to save the state. Here, the state is saved every STATE_SAVE_PERIOD with the first state being saved once the algorithm achieves full calibration, i.e. iaqAccuracy = 3 */
    if (stateUpdateCounter == 0) {
        if (iaqSensor.iaqAccuracy >= 3) {
            update = true;
            stateUpdateCounter++;
        }
    } else {
        /* Update every STATE_SAVE_PERIOD milliseconds */
        if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
            update = true;
            stateUpdateCounter++;
        }
    }

    if (update) {
        iaqSensor.getState(bsecState);
        checkIaqSensorStatus();

        Serial.println("0, Writing state to EEPROM");

        for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
            EEPROM.write(i + 1, bsecState[i]);
            //Serial.println(bsecState[i], HEX);
        }

        EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
        EEPROM.commit();
    }
}

/* vim: set filetype=cpp  sta:et:sw=4:ts=4:sts=4 */
