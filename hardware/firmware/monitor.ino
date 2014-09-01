#include <Wire.h>

#include "TSL2561.h"
#include "OneWire.h"
#include "DHT22.h"

#define PIN_ONEWIRE 9
#define PIN_DHT22   6
#define PIN_LED     13

#define DEBUG       1

#define ERROR(...) if (DEBUG) {Serial.print(__VA_ARGS__);}

DHT22   myDHT22(PIN_DHT22);
bool    dht22_ok;

TSL2561 tsl(TSL2561_ADDR_FLOAT);
bool    tsl2561_ok;

OneWire ds(PIN_ONEWIRE);
byte    max31850_addr[8];
bool    max31850_ok;

bool tsl2561_setup(void) {
    if (!tsl.begin())
        return false;

    //tsl.setGain(TSL2561_GAIN_0X);           // set no gain (for bright situtations)
    tsl.setGain(TSL2561_GAIN_16X);            // set 16x gain (for dim situations)

    tsl.setTiming(TSL2561_INTEGRATIONTIME_13MS);      // shortest integration time (bright light)
    //tsl.setTiming(TSL2561_INTEGRATIONTIME_101MS);   // medium integration time (medium light)
    //tsl.setTiming(TSL2561_INTEGRATIONTIME_402MS);   // longest integration time (dim light)

    return true;
}

bool tsl2561_sample(uint16_t &ir, uint16_t &full, uint16_t &visible, uint32_t &lux) {
    uint32_t lum = tsl.getFullLuminosity();
    ir = lum >> 16;
    full = lum & 0xFFFF;
    visible = full - ir;
    lux = tsl.calculateLux(full, ir);

    return true;
}

bool max31850_setup(void) {
    uint8_t i = 10;
    while (i--) {
        if (ds.search(max31850_addr)) {
            //found something
            if (OneWire::crc8(max31850_addr, 7) == max31850_addr[7]) {
                //crc ok
                if (max31850_addr[0] == 0x3B) {
                    //max31850 found
                    return true;
                }
            }
        }
        ds.reset_search();
        delay(250);
    }
    return false;
}

bool max31850_sample(float &celsius) {
    byte present = 0;
    byte data[12];

    ds.reset();
    ds.select(max31850_addr);
    ds.write(0x44, 1);        // start conversion, with parasite power on at the end

    delay(1000);
    // we might do a ds.depower() here, but the reset will take care of it.

    present = ds.reset();
    if (!present) {
        ERROR("max31850: not detected\n");
        return false;
    }

    ds.select(max31850_addr);
    // Read Scratchpad
    ds.write(0xBE);

    // we need 9 bytes
    for (uint8_t i = 0; i < 9; i++)
        data[i] = ds.read();

    if (OneWire::crc8(data, 8) != data[8]) {
        ERROR("max31850: CRC error\n");
        return false;
    }

    int16_t raw = (data[1] << 8) | data[0];
    if (raw & 0x01) {
        ERROR("max31850: fault\n");
        return false;
    }

    celsius = (float)raw / 16.0;
    return true;
}

bool dht22_setup(void) {
    //sensor requires >2s warm-up after power-on.
    delay(3000);
    return myDHT22.readData() == DHT_ERROR_NONE;
}

bool dht22_sample(float &celcius, float &relativehumidity) {
    DHT22_ERROR_t errorCode;
    delay(1000);

    errorCode = myDHT22.readData();
    switch(errorCode) {
        case DHT_ERROR_NONE:
            celcius = myDHT22.getTemperatureC();
            relativehumidity = myDHT22.getHumidity();
            return true;
        case DHT_ERROR_CHECKSUM:
        case DHT_BUS_HUNG:
        case DHT_ERROR_NOT_PRESENT:
        case DHT_ERROR_ACK_TOO_LONG:
        case DHT_ERROR_SYNC_TIMEOUT:
        case DHT_ERROR_DATA_TIMEOUT:
        case DHT_ERROR_TOOQUICK:
            ERROR("dht22: error ");
            ERROR(errorCode, HEX);
            ERROR("\n");
            return false;
    }
}


void setup(void) {
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);

    digitalWrite(PIN_LED, HIGH);

    tsl2561_ok = tsl2561_setup();
    max31850_ok = max31850_setup();
    dht22_ok = dht22_setup();

    if (! (tsl2561_ok && max31850_ok && dht22_ok)) {
        if (!tsl2561_ok)
            Serial.println("ERROR: tsl2561");
        if (!max31850_ok)
            Serial.println("ERROR: max31850");
        if (!dht22_ok)
            Serial.println("ERROR: dht22");
        while (1) {
            digitalWrite(PIN_LED, 0x01 ^ digitalRead(PIN_LED));
            delay(100);
        }
    }
}

void loop(void) {
    uint16_t ir, full, visible;
    uint32_t lux;
    float celcius, celcius2, rh;

    bool tsl2561_sample_ok = tsl2561_sample(ir, full, visible, lux);
    bool max31850_sample_ok = max31850_sample(celcius);
    bool dht22_sample_ok = dht22_sample(celcius2, rh);

    if (tsl2561_sample_ok) {
        Serial.print("LIGHT: ");
        Serial.print("ir="); Serial.print(ir);   Serial.print(" ");
        Serial.print("full="); Serial.print(full);   Serial.print(" ");
        Serial.print("visible="); Serial.print(full - ir);   Serial.print(" ");
        Serial.print("lux="); Serial.println(lux);
    }
    if (max31850_sample_ok) {
        Serial.print("TEMPERATURE: ");
        Serial.print("tempc="); Serial.println(celcius);
    }
    if (dht22_sample_ok) {
        Serial.print("HUMIDITY: ");
        Serial.print("tempc="); Serial.print(celcius2);   Serial.print(" ");
        Serial.print("rh="); Serial.println(rh);
    }

    digitalWrite(PIN_LED, 0x01 ^ digitalRead(PIN_LED));
}

