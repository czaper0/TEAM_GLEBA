#include <Arduino.h>
#include <MKRWAN.h>
#include "Adafruit_seesaw.h"
#include "arduino_secrets.h"
#include "ArduinoLowPower.h" 

#define BAUDRATE 9600
//#define DHTPIN 11
//#define DHTTYPE DHT22
// Switch pins
#define SWITCH_TEMP 7
#define SWITCH_PM 9

Adafruit_seesaw ss;

// LoRa connection
LoRaModem Lora;
String appEui = SECRET_APP_EUI;
String appKey = SECRET_APP_KEY;

// For storing measurements
typedef struct measurement
{
    float temp;
    uint16_t cap;
    uint16_t wynik;
} Measurement;

// For setting the timings related to measurements

struct timeSettings
{
    // Time sequence: turn on PM sensor, wait secondsForPmBeforeTemperature, turn on Temp sensor, 
    // wait secondsForTemperatureAndPM, make measure measureTryTimes times, wait timeAfterMeasuresTime seconds
    unsigned int secondsForPmBeforeTemperature;
    unsigned int secondsForTemperatureAndPM;
    unsigned int measureTryTimes;
    unsigned int minutesBetweenMeasurements; // Minutes between send n and n+1 measure
    unsigned int timeAfterMeasuresTime; // timeAfterMeasuresTime = minutesBetweenMeasurements*60 - (secondsForPmBeforeTemperature+secondsForTemperatureAndPM)
} TimeSettings;

//
//DHT dht(DHTPIN, DHTTYPE);


// Sending data to LoRa
void send_data(const Measurement& measurement)
{
    String msg = " cap: " + String(measurement.cap) +
                 " temp: " + String(measurement.temp) +
                 " RH % :" + String(measurement.wynik);
    Serial.println(msg);
    Lora.beginPacket();
    Lora.print(msg);
    int err = Lora.endPacket(true);

    if (err > 0)
    {
        Serial.println("Message sent correctly!");
    } 
    else 
    {
        Serial.println("Error sending message :(");
        Serial.println("(you may send a limited amount of messages per minute, depending on the signal strength");
        Serial.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
    }
}

// Measuring temperature and moisture
void measure_temp_cap(Measurement& measurement)
{ 
    measurement.temp = ss.getTemp();
    measurement.cap = ss.touchRead(0);
    measurement.wynik = map(measurement.cap,332,1016,0,100);
    Serial.println(measurement.cap);
}

// Turning sensors on/off
// TODO: mozliwe, ze trzeba uruchamiac komunikacje za kazdym razem (prawdopodobnie tak)
void sensors_on()
{
    const uint8_t start_SDS_cmd[] =
        {
            0xAA, 0xB4, 0x06, 0x01, 0x01,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            0xFF, 0xFF, 0x05, 0xAB};
    Serial1.write(start_SDS_cmd, sizeof(start_SDS_cmd));
    delay(TimeSettings.secondsForPmBeforeTemperature);
    digitalWrite(SWITCH_TEMP, HIGH);
    digitalWrite(SWITCH_PM, HIGH);
    digitalWrite(0, HIGH); 
    delay(TimeSettings.secondsForTemperatureAndPM);
}

void sensors_off()
{
    digitalWrite(SWITCH_TEMP, LOW);
    digitalWrite(SWITCH_PM, LOW);
    const uint8_t stop_SDS_cmd[] =
      {
          0xAA, 0xB4, 0x06, 0x01, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00,
          0xFF, 0xFF, 0x05, 0xAB};
    Serial1.write(stop_SDS_cmd, sizeof(stop_SDS_cmd));
}

void set_times()
{
    TimeSettings.secondsForPmBeforeTemperature = 45;
    TimeSettings.secondsForTemperatureAndPM = 15;
    TimeSettings.measureTryTimes = 20;
    TimeSettings.minutesBetweenMeasurements = 5;
    TimeSettings.timeAfterMeasuresTime = TimeSettings.minutesBetweenMeasurements*60 - (TimeSettings.secondsForPmBeforeTemperature+TimeSettings.secondsForTemperatureAndPM);
    //convert seconds to miliseconds
    TimeSettings.secondsForPmBeforeTemperature = TimeSettings.secondsForPmBeforeTemperature*1000;
    TimeSettings.secondsForTemperatureAndPM = TimeSettings.secondsForTemperatureAndPM*1000;
    TimeSettings.minutesBetweenMeasurements = TimeSettings.minutesBetweenMeasurements*60000;
    TimeSettings.timeAfterMeasuresTime = TimeSettings.timeAfterMeasuresTime*1000;
}

//
// Init and main loop
//

void setup()
{
    // Set times from set_times() function
    set_times();
    digitalWrite(LED_BUILTIN, LOW);
    // Turn on switches
    pinMode(SWITCH_TEMP, OUTPUT);
    digitalWrite(SWITCH_TEMP, HIGH);

    // Turn on switches
    pinMode(SWITCH_PM, OUTPUT);
    digitalWrite(SWITCH_PM, HIGH);

    pinMode(0,OUTPUT);
    digitalWrite(0, HIGH);
    
    // Turn on sensor communictaion
    Serial.begin(BAUDRATE);
    Serial1.begin(BAUDRATE, SERIAL_8N1);
    
    Serial.begin(115200);

    if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1);
    } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
    }
    
    // Connect to LoRa
    if (!Lora.begin(EU868))
    {
        Serial.println("Failed to start module");
        while (1) {}
    };
    Serial.print("Your module version is: ");
    Serial.println(Lora.version());
    Serial.print("Your device EUI is: ");
    Serial.println(Lora.deviceEUI());
    int connected = Lora.joinOTAA(appEui, appKey);
    if (!connected)
    {
        Serial.println("Something went wrong; are you indoor? Move near a window and retry");
        // ?XD
        while (1) {}
    }

    // Set poll interval to 60 secs.
    Lora.minPollInterval(60);
    // NOTE: independently by this setting the modem will
    // not allow to send more than one message every 2 minutes,
    // this is enforced by firmware and can not be changed.
}

void loop()
{
    Measurement meas;
    sensors_on();
    for(int i=0;i<100;i++)
    {
        measure_temp_cap(meas);
    }
   // sensors_off();
    send_data(meas);
    LowPower.deepSleep((uint32_t) TimeSettings.timeAfterMeasuresTime); 
    delay(1000);
    
}
