/*  Board Heltec Wifi Lora 32 (V2)
 *  Heltec Automation I2C scanner example (also it's a basic example how to use I2C1)
 *
 * ESP32 have two I2C (I2C0 and I2C1) bus
 *
 * 
 * OLED is connected to I2C0, so if scan with Wire (I2C0), the return address should be 0x3C.
 *
 * If you need scan other device address in I2C1...
 *		- Comment all Wire.***() codes;
 * 		- Uncomment all Wire1.***() codes;
 *
 * I2C scan example and I2C0
 *
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/HelTecAutomation/Heltec_ESP32
 * */

//I2C device BME680 found at address 0x77  !

#include "Arduino.h"
#include "heltec.h"
#include <EEPROM.h>
#include "bsec.h"

#include <WiFi.h>

const char* ssid     = "Vodafone-05C121";
const char* password = "dpsjKk34U2";

#define BME680_I2C_ADDR 0x77

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
#define STATE_SAVE_PERIOD	UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day

#define STATE_SAVE_PERIOD	UINT32_C( 1000) // 360 minutes - 4 times a day

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

void print_bme680(unsigned long time);

Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;

void connect_wifi()
{
    // connecting to a WiFi network
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}



void scan_I2C()
{
    byte error, address;
    int nDevices;

    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
    {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();

        //    Wire1.beginTransmission(address);
        //    error = Wire1.endTransmission();

        if (error == 0)
        {
            Serial.print("I2C device found at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.print(address,HEX);
            Serial.println("  !");

            nDevices++;
        }
        else if (error==4)
        {
            Serial.print("Unknown error at address 0x");
            if (address<16)
                Serial.print("0");
            Serial.println(address,HEX);
        }
    }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");

}

void setup()
{
    Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
    Wire.begin(SDA_OLED, SCL_OLED); //Scan OLED's I2C address via I2C0
    //Wire1.begin(SDA, SCL);        //If there have other device on I2C1, scan the device address via I2C1

    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
    Serial.begin(115200); //Not needed
    while(!Serial);    // time to get serial running
    delay(5000);
    //Serial.println(F("0, BME680 test Init"));

    iaqSensor.begin(BME680_I2C_ADDR, Wire);
    output = "0, BSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
    Serial.println(output);
    checkIaqSensorStatus();

    iaqSensor.setConfig(bsec_config_iaq);
    checkIaqSensorStatus();

    loadState();

    Serial.println("0, BME680 0x77 OK");

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

    Heltec.display->flipScreenVertically();
    Heltec.display->setFont(ArialMT_Plain_10);
    // clear the display
    Heltec.display->clear();

    Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    Heltec.display->drawString(0, 0, "Starting BME ");

    //Heltec.display->drawString(10, 128, String(millis()));
    // write the buffer to the display
    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
    Heltec.display->drawString(0, 11, "Temp   P (mBar)  Hum %");

    //Heltec.display->drawString(0, 9, "Temp VOC IAQ");

    // The coordinates define the center of the text
    //Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
    //Heltec.display->drawString(64, 22, "Center aligned (64,22)");

    // The coordinates define the right end of the text
    //Heltec.display->setTextAlignment(TEXT_ALIGN_RIGHT);
    //Heltec.display->drawString(128, 33, "Right aligned (128,33)");

    Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
    Heltec.display->drawString(0, 33, "VOC      Iac      Iacq");

    Heltec.display->display();
    errLeds();
    //connect_wifi();
}


void loop(void)
{
    unsigned long time_trigger = millis();
    if (iaqSensor.run()) { // If new data is available
        print_bme680(time_trigger);
        updateState();
    } else {
        checkIaqSensorStatus();
    }
}

void loop2()
{
    //scan_I2C();
    void read_bme680(void);
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

    //   Heltec.display->clear();
    Heltec.display->setColor(BLACK);
    Heltec.display->fillRect(10, 22, 140, 10);
    //Heltec.display->fillRect(60, 22, 40, 10);
    Heltec.display->fillRect(10, 44, 140, 10);
    //Heltec.display->drawString(20, 22, "              ");
    Heltec.display->setColor(WHITE);
    Heltec.display->setTextAlignment(TEXT_ALIGN_CENTER);
    //Heltec.display->drawString(20, 22, "              ");
    Heltec.display->drawString(20, 22, String(iaqSensor.temperature) );
    Heltec.display->drawString(60, 22, String(iaqSensor.pressure/1e2) );
    Heltec.display->drawString(100, 22, String(iaqSensor.humidity) );

    Heltec.display->drawString(20, 44, String(iaqSensor.breathVocEquivalent) );
    Heltec.display->drawString(60, 44, String(iaqSensor.iaq) );
    Heltec.display->drawString(100, 44, String(iaqSensor.iaqAccuracy) );

    Heltec.display->display();
    //errLeds();
}


// Helper function definitions
void checkIaqSensorStatus(void)
{
    if (iaqSensor.status != BSEC_OK) {
        if (iaqSensor.status < BSEC_OK) {
            output = "0, BSEC error code : " + String(iaqSensor.status);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        } else {
            output = "0, BSEC warning code : " + String(iaqSensor.status);
            Serial.println(output);
        }
    }

    if (iaqSensor.bme680Status != BME680_OK) {
        if (iaqSensor.bme680Status < BME680_OK) {
            output = "0, BME680 error code : " + String(iaqSensor.bme680Status);
            Serial.println(output);
            for (;;)
                errLeds(); /* Halt in case of failure */
        } else {
            output = "0, BME680 warning code : " + String(iaqSensor.bme680Status);
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
