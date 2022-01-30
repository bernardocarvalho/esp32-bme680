/**
 * @file main.cp
 * @brief ESP32 software to mesure Air quality 
 * @author Bernardo Carvalho / IPFN
 * @date 29/01/2022
 *
 * @copyright Copyright 2016 - 2021 IPFN-Instituto Superior Tecnico, Portugal
 * Licensed under the EUPL, Version 1.2 only (the "Licence");
 * You may not use this work except in compliance with the Licence.
 * You may obtain a copy of the Licence, available in 23 official languages of
 * the European Union, at:
 * https://joinup.ec.europa.eu/community/eupl/og_page/eupl-text-11-12
 *
 * @warning Unless required by applicable law or agreed to in writing, software
 * distributed under the Licence is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the Licence for the specific language governing permissions and
 * limitations under the Licence.
 *
 * @details
 * 
 */
 
// Include the correct display library
#include <Arduino.h>
#include <WiFi.h>
#include "SSD1306Wire.h"        // 
#include <EEPROM.h>
#include "bsec.h"
//#include <CRC.h>
#include "CRC8.h"

//#define BME_PRESENT 0

// WiFi credentials.
char const * WIFI_SSID = "Cabovisao-E30F";
char const * WIFI_PASS = "e0cec31ae30f"; 

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};
#define STATE_SAVE_PERIOD      UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
#define SAVE_PERIOD	           UINT32_C( 1000) // in millis

#define BME680_I2C_ADDR 0x77

/***     Global vars   ***/
Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

String output;
// Initialize the OLED display using Arduino Wire:
SSD1306Wire display(0x3c, OLED_SDA, OLED_SCL);   // ADDRESS,  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
float iaq, rawTemperature, pressure, rawHumidity, gasResistance, stabStatus, runInStatus, temperature, humidity,
      staticIaq, co2Equivalent, breathVocEquivalent, compGasValue, gasPercentage;
uint8_t iaqAccuracy;
CRC8 crc;

void wifi_connect(void) {
	// Connect to Wifi.
	Serial.println();
	Serial.println();
	Serial.print("Connecting to ");
	Serial.println(WIFI_SSID);

	// Set WiFi to station mode and disconnect from an AP if it was previously connected
	//	WiFi.mode(WIFI_STA);
	//	WiFi.disconnect();
	delay(100);

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	Serial.println("Connecting...");

    for (int i=10; i-- >10;) {
	
	//while (WiFi.status() != WL_CONNECTED) {
		// Check to see if connecting failed.
		// This is due to incorrect credentials
		if (WiFi.status() == WL_CONNECT_FAILED) {
			Serial.println("Failed to connect to WIFI. Please verify credentials: ");
			Serial.println();
			Serial.print("SSID: ");
			Serial.println(WIFI_SSID);
			Serial.print("Password: ");
			Serial.println(WIFI_PASS);
			Serial.println();
		}

		delay(1000);
	    if (WiFi.status() == WL_CONNECTED){
            //Serial.println("");
            Serial.println("WiFi connected");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());

            Serial.println("Hello World, I'm connected to the internets!!");
            break;
        } 
    }

}

/**
   Configure the BSEC library with information about the sensor
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

// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
// Helper functions declarations
void checkIaqSensorStatus(void);
void errLeds(void);
void loadState(void);
void updateState(void);

void print_bme680(unsigned long time);

void scan_I2C() {
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

void bme680_setup()
{
    EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length
 
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
}
void print_bme680(unsigned long time){
/* Message format all floats, except pressure, iaq (uint_8)
 time, temperature, pressure, humidity, co2Equivalent, iaq, iaqAccuracy, staticIaq, rawTemperature 
             rawHumidity,  rawHumidity */
    output = "1 " + String(time,DEC);
    output += ", " + String(iaqSensor.temperature, 1);
    output += ", " + String(iaqSensor.pressure,0);
    output += ", " + String(iaqSensor.humidity,1);
    output += ", " + String(iaqSensor.co2Equivalent,1);
    output += ", " + String(iaqSensor.breathVocEquivalent,1);
    output += ", " + String(iaqSensor.iaq, DEC);
    output += ", " + String(iaqSensor.iaqAccuracy,1);
    output += ", " + String(iaqSensor.staticIaq,1);
    output += ", " + String(iaqSensor.rawTemperature,1);
    output += ", " + String(iaqSensor.rawHumidity, 1);
    output += ", " + String(iaqSensor.rawHumidity, 1);
    Serial.println(output);
}
void print_bme680_mockup(unsigned long time){
    uint16_t length;
    char c;
    output = "1, " + String(time, DEC);
    output += ", " + String(temperature, 1);
    output += ", " + String(pressure, 0);
    output += ", " + String(humidity, 1);
    output += ", " + String(co2Equivalent, 1);
    output += ", " + String(breathVocEquivalent, 1);
    output += ", " + String(iaq,1);
    output += ", " + String(iaqAccuracy, DEC);
    /*
    output += ", " + String(iaqSensor.staticIaq);
    output += ", " + String(iaqSensor.rawTemperature);
    output += ", " + String(iaqSensor.rawHumidity);
    output += ", " + String(iaqSensor.gasResistance);
    */
    length = output.length();
    output += ", " + String(length);
    Serial.print(output);

    crc.restart();
    for(int i =0; i < length; i++){
        c = output[i];
        crc.add(c);

    }
    Serial.print(", ");

    //crc.add(output);
    //Serial.println(crc8((uint8_t *)str, length));
    Serial.print(crc.getCRC(), HEX);
    Serial.print(", ");
    Serial.println(crc.getCRC());
    //Serial.println(crc8((uint8_t *)str, length), HEX);

}


void draw_oled_mockup(unsigned long time){
    display.setFont(ArialMT_Plain_10);

    // The coordinates define the left starting point of the text
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 10, "Left aligned (0,10)");
    display.drawString(20, 22, String(temperature) );
    display.drawString(60, 22, String(pressure/1e2) );
    display.drawString(100, 22, String(humidity) );

    display.drawString(20, 44, String(breathVocEquivalent) );
    display.drawString(60, 44, String(iaq) );
    display.drawString(100, 44, String(iaqAccuracy) );
}

void draw_oled(unsigned long time){
    display.setFont(ArialMT_Plain_10);

    // The coordinates define the left starting point of the text
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 10, "Left aligned (0,10)");

    // The coordinates define the center of the text
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.drawString(64, 22, "Center aligned (64,22)");

    // The coordinates define the right end of the text
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    display.drawString(128, 33, "Right aligned (128,33)");

    display.drawString(20, 22, String(iaqSensor.temperature) );
    display.drawString(60, 22, String(iaqSensor.pressure/1e2) );
    display.drawString(100, 22, String(iaqSensor.humidity) );

    display.drawString(20, 44, String(iaqSensor.breathVocEquivalent) );
    display.drawString(60, 44, String(iaqSensor.iaq) );
    display.drawString(100, 44, String(iaqSensor.iaqAccuracy) );
}
/*
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
*/

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

void setup() {
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    Serial.println(__FILE__);
    //Serial.println();
    //Serial.println();

    wifi_connect();
#ifdef BME_PRESENT
    bme680_setup();
#endif    
    // Initialising the UI will init the display too.
    display.init();

    display.flipScreenVertically();
    display.setFont(ArialMT_Plain_10);
}

void loop() {
    static unsigned long nextTime = 0;
    unsigned long now = millis();
    static bool led_state = LOW;

#ifdef BME_PRESENT
    if (iaqSensor.run()) { // If new data is available
        now = millis();
        print_bme680(now);
        updateState();
        led_state = !led_state;   
        digitalWrite(LED_BUILTIN, led_state); 
    } else {
        checkIaqSensorStatus();
    }
#else
    if ( now > nextTime ) {  
        nextTime = now + SAVE_PERIOD; 
        led_state = !led_state;   
        digitalWrite(LED_BUILTIN, led_state);
        temperature = random(0,500)/10.0;
        pressure = random(980,1020)*100.0;
        humidity = random(500,1000)/10.0;
        co2Equivalent = random(0,9000)/1.0;
        breathVocEquivalent = random(0,2000)/10.0;
        iaq = random(0,500)/1.0;
        iaqAccuracy = random(0,3);

        print_bme680_mockup(now);
    }
#endif
}
