#include <Arduino.h>
#include <MKRWAN.h>
#include "arduino_secrets.h"
#include "ArduinoLowPower.h" 

#define BAUDRATE 9600

#define SWITCH_TEMP 7
#define SWITCH_PM 9

LoRaModem Lora;
String appEui = "70B3D57ED0032AF4";
String appKey = "65378345064F51C304DECB79CA9CFF3C";

typedef struct device
{
    int meas;
    float RH;
    float longitude;
    float latitude;
} Device;

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

void send_data(const Device& device)
{
    String msg = " soil moisture: " + String(device.meas) +
                 " RH % :" + String(device.RH)+
                 " longitude: " + String(device.longitude) +
                 " latitude: " + String(device.latitude);

    Lora.beginPacket();
    Lora.print(msg);
    int err = Lora.endPacket(true);

    if (err > 0)
    {
        Serial.println("Message sent correctly!");
    } 
    else 
    {
        Serial.println("Error sending message 😞");
        Serial.println("(you may send a limited amount of messages per minute, depending on the signal strength");
        Serial.println("it may vary from 1 message every couple of seconds to 1 message every minute)");
    }
}

void measure (Device& device)
{ 
    device.meas= analogRead(A1);
    device.RH = map(device.meas,0,800,0,100); //w zaleznosci od czujnika trzeba tu wartosci pozmieniac
    Serial.println(device.meas);
}

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



void setup() {
  // put your setup code here, to run once:
  set_times();
  Serial.begin(BAUDRATE);
    Serial1.begin(BAUDRATE, SERIAL_8N1);
    
    Serial.begin(115200);
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

void loop() { 
  // put your main code here, to run repeatedly:
  Device sensor;
  sensor.longitude=21.002362;
  sensor.latitude=52.21136;
  sensors_on();
    for(int i=0;i<100;i++)
    {
        measure(sensor);
    }
   // sensors_off();
    send_data(sensor);
    delay(1000);
}