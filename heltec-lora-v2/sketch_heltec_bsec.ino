/*
arduino-cli
compile:
 arduino-cli compile --fqbn heltec:esp32:heltec_wifi_lora_32_V2 sketch_heltec_bsec.ino
 arduino-cli upload -p /dev/ttyUSB0 --fqbn heltec:esp32:heltec_wifi_lora_32_V2 sketch_heltec_bsec.ino1

 arduino-cli monitor  -p /dev/ttyUSB0 -c baudrate=115200

 *
 * this project also realess in GitHub:
 */

#include "Arduino.h"
#include "WiFi.h"
#include <WiFiMulti.h>
#include <ArduinoJson.h>
//#include "images.h"

//#include "LoRaWan_APP.h"
//#include <Wire.h>  
#include "HT_SSD1306Wire.h"

#include <EEPROM.h>
#include "bsec.h"

SSD1306Wire  oled_display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

//const char* ssid     = "MikroTok";
//const char* password = "MikroToklab1.59";


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

WiFiMulti wifiMulti;

void WIFISetUp(void)
{
    WiFiMulti wifiMulti;
    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.disconnect(true);
    delay(100);
    //     Add list of wifi networks
    wifiMulti.addAP("MikroTok","MikroToklab1.59");
    wifiMulti.addAP("TP-Link_37E8","87936148");
    //fill in "Your WiFi SSID","Your Password"
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
        // Connect to Wi-Fi using wifiMulti (connects to the SSID with strongest connection)
        Serial.println("Connecting Wifi...");
        if(wifiMulti.run() == WL_CONNECTED) {
            oled_display.drawString(0, 0, "Connecting...OK.");
            oled_display.drawString(100, 0, "WOK");
            Serial.println("");
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
        }
        else {
            oled_display.drawString(0, 0, "No Wifi Connection");
        }
    }

    oled_display.display();
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
    unsigned long time_trigger = millis();

    if (iaqSensor.run()) { // If new data is available
        print_bme680(time_trigger);
        oled_bme680(time_trigger);
        updateState();
    } else {
        checkIaqSensorStatus();
    }
    digitalWrite(LED, led_status);  
    led_status = not led_status;

}

void print_bme680(unsigned long time){
    output = "1, " + String(time);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.pressure);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    output += ", " + String(iaqSensor.iaq);
    output += ", " + String(iaqSensor.iaqAccuracy);
    output += ", " + String(iaqSensor.temperature);
    output += ", " + String(iaqSensor.humidity);
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.co2Equivalent);
    output += ", " + String(iaqSensor.breathVocEquivalent);
    Serial.println(output);
}

void oled_bme680_init(){
     //oled_display.setColor(BLACK);
    //oled_display.fillRect(10, 11, 140, 10);
    char buff[40];
    oled_display.setColor(WHITE);
    oled_display.setTextAlignment(TEXT_ALIGN_LEFT);
    //oled_display.drawString(0, 11, 
    sprintf(buff, "Temp Press Hum  Voc");
    //oled_display.drawString(0, 11, "Temp Press Hum   Voc");
    oled_display.drawString(0, 10, buff);
//    oled_display.drawString(10, 11, "Temp");
//    oled_display.drawString(40, 11, "Press");
//    oled_display.drawString(80, 11, "HumVoc");
//    oled_display.drawString(120, 11, "Voc");
}

void oled_bme680(unsigned long time){
    char buff[40];
    oled_display.setColor(BLACK);
    oled_display.fillRect(0, 22, 128, 11);
    oled_display.fillRect(0, 50, 128, 11);
    //Heltec.display->fillRect(60, 22, 40, 10);
    //oled_display.fillRect(10, 44, 140, 10);
    //Heltec.display->drawString(20, 22, "              ");
    oled_display.setColor(WHITE);
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
        if (iaqSensor.bme68xStatus < BME68X_OK) {
            output = "0, bme68x error code : " + String(iaqSensor.bme68xStatus);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        } else {
            output = "0, bme68x warning code : " + String(iaqSensor.bme68xStatus);
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

