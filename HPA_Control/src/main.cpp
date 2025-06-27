#include <IcsHardSerialClass.h>

#ifdef ARDUINO_UNOR4_WIFI
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Arduino_FreeRTOS.h>

TaskHandle_t loop_task, udp_task;
void loop_thread_func(void *pvParameters);
void udp_thread_func(void *pvParameters);

const char WIFI_SSID[] = "HPA_Measurement";
const char WIFI_PASSWORD[] = "HPA_Password";

struct ControlData
{
    float rudder;   // 操舵角
    float elevator; // エレベータ角
    int trim;       // トリム角
};

const int UDP_PORT = 15646;   // UDPポート番号
const int UDP_INTERVAL = 100; // UDP送信間隔（ミリ秒）
WiFiUDP wifiUdp;              // 操舵 -> 計測の通信
#endif

// #define SERIAL_DEBUG

const byte EN_PIN = 2;
const long ICS_BAUDRATE = 115200;
const int TIMEOUT = 10;

int off0;
int off1;
float a0 = 0.0; // rudder
float a1 = 0.0; // elevater
float rudMax = 833.3;
float eleMax = 555.6;
float trim_min = 13.9;
int b0 = 0;
int b1 = 0;

int trimVal;
const int trimMax = 10;
long previous;
const int TRIM_INTERVAL = 1000;
const int BUTTON_WAIT = 50; // チャタリング防止

const int TRIM_UP_PIN = 6;
const int TRIM_DOWN_PIN = 7;
const int RUDDER_PIN = A0;
const int ELEVATOR_PIN = A1;

#if defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
#define ICS_SERIAL Serial1
#else
#define ICS_SERIAL Serial
#endif
IcsHardSerialClass krs(&ICS_SERIAL, EN_PIN, ICS_BAUDRATE, TIMEOUT); // インスタンス＋ENピン(2番ピン)およびUARTの指定

void setup()
{
#if (defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)) && defined(SERIAL_DEBUG)
    Serial.begin(115200);
    delay(1000);
#endif
    krs.begin(); // サーボモータの通信初期設定

    pinMode(RUDDER_PIN, INPUT);
    pinMode(ELEVATOR_PIN, INPUT);
    pinMode(TRIM_UP_PIN, INPUT_PULLUP);
    pinMode(TRIM_DOWN_PIN, INPUT_PULLUP);

    off0 = analogRead(RUDDER_PIN);
    off1 = analogRead(ELEVATOR_PIN);

    trimVal = 0;
    previous = millis();

#ifdef ARDUINO_UNOR4_WIFI
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiUdp.begin(UDP_PORT);

    xTaskCreate(loop_thread_func, static_cast<const char *>("Loop Thread"), 4096, nullptr, 2, &loop_task);
    xTaskCreate(udp_thread_func, static_cast<const char *>("UDP Thread"), 4096, nullptr, 1, &udp_task);
    vTaskStartScheduler();
    
    for (;;)
        ;
#endif
}

void loop()
{
    if (digitalRead(TRIM_UP_PIN) == LOW && millis() - previous > TRIM_INTERVAL)
    {
        trimVal += 1;
        previous = millis();
    }
    if (digitalRead(TRIM_DOWN_PIN) == LOW && millis() - previous > TRIM_INTERVAL)
    {
        trimVal -= 1;
        previous = millis();
    }
    if (digitalRead(TRIM_UP_PIN) == HIGH && digitalRead(TRIM_DOWN_PIN) == HIGH && millis() - previous > BUTTON_WAIT)
    {
        previous = 0;
    }

    if (trimVal > trimMax)
        trimVal = trimMax;
    if (trimVal < -trimMax)
        trimVal = -trimMax;

    a0 = 0.15 * (float(analogRead(RUDDER_PIN) - off0) / 512.0) + 0.85 * a0;
    a1 = 0.15 * (float(analogRead(ELEVATOR_PIN) - off1) / 512.0) + 0.85 * a1;

    if (-1 < a0 && a0 < -0.02)
    {
        a0 = a0;
    }
    else if (-0.02 < a0 && a0 < 0.02)
    {
        a0 = 0.0;
    }
    else if (0.02 < a0 && a0 < 1.0)
    {
        a0 = a0;
    }

    if (a0 < -1)
    {
        a0 = -1.0;
    }
    else if (1.0 < a0)
    {
        a0 = 1.0;
    }

    if (-1.0 < a1 && a1 < -0.02)
    {
        a1 = a1;
    }
    else if (-0.02 < a1 && a1 < 0.02)
    {
        a1 = 0.0;
    }
    else if (0.02 < a1 && a1 < 1.0)
    {
        a1 = a1;
    }

    if (a1 < -1.0)
    {
        a1 = -1.0;
    }
    else if (1.0 < a1)
    {
        a1 = 1.0;
    }

#ifdef SERIAL_DEBUG
    Serial.print(a0);
    Serial.print("\t");
    Serial.println(a1);
#endif

    b0 = (int)(7500 + 1381.3 + rudMax * a0);
    b1 = (int)(7500 + 1547.3 - eleMax * a1 + trim_min * trimVal);

#ifdef SERIAL_DEBUG
    Serial.print(b0);
    Serial.print("\t");
    Serial.println(b1);
#endif

    krs.setPos(0, b0);
    krs.setPos(1, b1);
}

#ifdef ARDUINO_UNOR4_WIFI
void loop_thread_func(void *pvParameters)
{
    for (;;)
    {
        loop();
        delay(10);
    }
}

void udp_thread_func(void *pvParameters)
{
    //----- UDP 送信（100 ms 間隔） -----
    for (;;)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            ControlData controlData;
            controlData.rudder = a0;
            controlData.elevator = a1;
            controlData.trim = trimVal;

            // 計測のIPアドレスが固定でないため、ブロードキャストとして送信
            IPAddress localIP = WiFi.localIP();
            IPAddress subnet = WiFi.localIP();
            IPAddress measurementIP(
                localIP[0] | (~subnet[0]),
                localIP[1] | (~subnet[1]),
                localIP[2] | (~subnet[2]),
                localIP[3] | (~subnet[3]));

            wifiUdp.beginPacket(measurementIP, UDP_PORT);
            wifiUdp.write((uint8_t *)(&controlData), sizeof(ControlData));
            wifiUdp.endPacket();
        }
        delay(100);
    }
}
#endif
