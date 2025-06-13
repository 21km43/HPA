#include <Arduino.h>
#include <M5CoreS3.h>
#include <SoftwareSerial.h>
#define sensor_t sensor_t_
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#undef sensor_t
#include <M5_IMU_PRO.h>
#include <MadgwickAHRS.h>
#include <MahonyAHRS.h>
#include <esp32/rom/crc.h>
#include <WiFi.h>
#include <WiFiUDP.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <mbedtls/md.h>
#include <WiFiClientSecure.h>
#include <MQTTClient.h>
#include <ctime>
#include "secrets.h"

struct ControlData
{
  float rudder;   // 操舵角
  float elevator; // エレベータ角
  int trim;       // トリム角
};

const int UDP_PORT = 15646; // UDPポート番号
WiFiUDP wifiUdp;            // 操舵 -> 計測の通信

constexpr int USB_BAUDRATE = 115200;

#define GPSSerial Serial2
constexpr int GPS_SERIAL_BAUDRATE = 115200;
constexpr int GPS_RX = 13, GPS_TX = 5;

SoftwareSerial ALTSerial;
constexpr int ALT_SERIAL_BAUDRATE = 9600;
constexpr int ALT_RX = 8, ALT_TX = 9, ALT_REDE_PIN = 10;

#define LoRaSerial Serial1
constexpr int LORA_SERIAL_BAUDRATE = 9600;
constexpr int LORA_RX = 18, LORA_TX = 17;

constexpr int RPM_PIN = 14;
constexpr int TACHO_PIN[2] = {1, 2};
constexpr int SD_SPI_SCK_PIN = 36;
constexpr int SD_SPI_MISO_PIN = 35;
constexpr int SD_SPI_MOSI_PIN = 37;
constexpr int SD_SPI_CS_PIN = 4;

float rudder_rotation = 0.0f;
float elevator_rotation = 0.0f;
int trim = 0;

int watchdog_count;

#pragma region OTA
void ota_handle(void *parameter)
{
  for (;;)
  {
    ArduinoOTA.handle();
    delay(1000);
  }
}

void setupOTA(const char *nameprefix)
{
  // Configure the hostname
  uint16_t maxlen = strlen(nameprefix) + 7;
  char *fullhostname = new char[maxlen];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  ArduinoOTA.setHostname(fullhostname);
  delete[] fullhostname;

  ArduinoOTA.onStart([]()
                     {
	//NOTE: make .detach() here for all functions called by Ticker.h library - not to interrupt transfer process in any way.
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); });

  ArduinoOTA.onEnd([]()
                   { Serial.println("\nEnd"); });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });

  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("\nAuth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("\nBegin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("\nConnect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("\nReceive Failed");
    else if (error == OTA_END_ERROR) Serial.println("\nEnd Failed"); });

  ArduinoOTA.begin();

  Serial.println("OTA Initialized");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  xTaskCreate(
      ota_handle,   /* Task function. */
      "OTA_HANDLE", /* String with name of task. */
      10000,        /* Stack size in bytes. */
      NULL,         /* Parameter passed as input of the task */
      1,            /* Priority of the task. */
      NULL);        /* Task handle. */
}
#pragma endregion

#pragma region BMP280
// 気圧計による気圧、温度の測定及び高度の計算
constexpr int BMP280_SENSOR_ADDR = 0x76;
Adafruit_BMP280 bmp(&Wire1);
float ground_pressure = 1013.25f;
float temperature = 0;
float pressure = 0;
float bmp_altitude = 0;

void InitBMP280()
{
  if (!bmp.begin(BMP280_SENSOR_ADDR))
  {
    Serial.println("BMP280 init failed!");
    return;
  }
  Serial.println("BMP280 OK!");

  delay(100);

  ground_pressure = bmp.readPressure() / 100.0f;
}
void GetBMP280()
{
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure() / 100.0f;
  bmp_altitude = bmp.readAltitude(ground_pressure);
}
#pragma endregion

#pragma region BMI270
// 6軸+3軸センサによる姿勢角の計算
constexpr int BIM270_SENSOR_ADDR = 0x68;
constexpr int AHRS_SAMPLING = 100;

BMI270::BMI270 bmi270;
Madgwick ahrs_madgwick_6dof, ahrs_madgwick_9dof;
Mahony ahrs_mahony_6dof, ahrs_mahony_9dof;

float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
int16_t magX, magY, magZ;

float roll_mad6 = 0, pitch_mad6 = 0, yaw_mad6 = 0;
float roll_mad9 = 0, pitch_mad9 = 0, yaw_mad9 = 0;
float roll_mah6 = 0, pitch_mah6 = 0, yaw_mah6 = 0;
float roll_mah9 = 0, pitch_mah9 = 0, yaw_mah9 = 0;

void ahrs_task(void *pvParameters)
{
  while (1)
  {
    ahrs_madgwick_6dof.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
    ahrs_madgwick_9dof.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);
    ahrs_mahony_6dof.updateIMU(gyroX, gyroY, gyroZ, accelX, accelY, accelZ);
    ahrs_mahony_9dof.update(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, magX, magY, magZ);

    roll_mad6 = ahrs_madgwick_6dof.getRoll();
    roll_mad6 = roll_mad6 < 0 ? roll_mad6 + 180.0f : roll_mad6 - 180.0f;
    pitch_mad6 = ahrs_madgwick_6dof.getPitch();
    yaw_mad6 = ahrs_madgwick_6dof.getYaw();

    roll_mad9 = ahrs_madgwick_9dof.getRoll();
    roll_mad9 = roll_mad9 < 0 ? roll_mad9 + 180.0f : roll_mad9 - 180.0f;
    pitch_mad9 = ahrs_madgwick_9dof.getPitch();
    yaw_mad9 = ahrs_madgwick_9dof.getYaw();

    roll_mah6 = ahrs_mahony_6dof.getRoll();
    roll_mah6 = roll_mah6 < 0 ? roll_mah6 + 180.0f : roll_mah6 - 180.0f;
    pitch_mah6 = ahrs_mahony_6dof.getPitch();
    yaw_mah6 = ahrs_mahony_6dof.getYaw();

    roll_mah9 = ahrs_mahony_9dof.getRoll();
    roll_mah9 = roll_mah9 < 0 ? roll_mah9 + 180.0f : roll_mah9 - 180.0f;
    pitch_mah9 = ahrs_mahony_9dof.getPitch();
    yaw_mah9 = ahrs_mahony_9dof.getYaw();

    delay(AHRS_SAMPLING / 4);
  }
}

void InitBMI270()
{
  if (!bmi270.init(I2C_NUM_1, BIM270_SENSOR_ADDR))
  {
    Serial.println("Failed to find BMI270");
    return;
  }

  ahrs_madgwick_6dof.begin(AHRS_SAMPLING);
  ahrs_madgwick_9dof.begin(AHRS_SAMPLING);
  ahrs_mahony_6dof.begin(AHRS_SAMPLING);
  ahrs_mahony_9dof.begin(AHRS_SAMPLING);

  xTaskCreatePinnedToCore(ahrs_task, "ahrs_task", 2048, NULL, 1, NULL, 0);

  Serial.println("BMI270 OK!");
}
void GetBMI270()
{
  bmi270.readAcceleration(accelX, accelY, accelZ);
  bmi270.readGyroscope(gyroX, gyroY, gyroZ);
  bmi270.readMagneticField(magX, magY, magZ);
}
#pragma endregion

#pragma region GPS
// GPSの測定
TinyGPSPlus gps;
double gps_latitude = 0;     // 緯度（小数第9位まで）
double gps_longitude = 0;    // 経度（小数第9位まで）
uint16_t gps_year = 0;       // 西暦
uint8_t gps_month = 0;       // 月
uint8_t gps_day = 0;         // 日
uint8_t gps_hour = 0;        // 時
uint8_t gps_minute = 0;      // 分
uint8_t gps_second = 0;      // 秒
uint8_t gps_centisecond = 0; // センチ秒
double gps_altitude = 0;     // 高度 メートル単位
double gps_course = 0;       // 進行方向(deg)
double gps_speed = 0;        // 対地速度(m/s) 精度は高くないので参考程度に

void InitGPS()
{
  GPSSerial.begin(GPS_SERIAL_BAUDRATE, SERIAL_8N1, GPS_RX, GPS_TX);
}
void GetGPS()
{
  while (GPSSerial.available() > 0)
  {
    const char c = GPSSerial.read();
    gps.encode(c);
    // Serial.write(c);

    if (gps.date.isUpdated() || gps.time.isUpdated())
    {
      struct tm t_utc, *t_jst;
      t_utc.tm_year = gps.date.year() - 1900;
      t_utc.tm_mon = gps.date.month() - 1;
      t_utc.tm_mday = gps.date.day();
      t_utc.tm_hour = gps.time.hour();
      t_utc.tm_min = gps.time.minute();
      t_utc.tm_sec = gps.time.second();
      t_utc.tm_isdst = 0;
      time_t unixtime = mktime(&t_utc) + 60 * 60 * 9; // JST
      t_jst = localtime(&unixtime);
      gps_year = t_jst->tm_year + 1900;
      gps_month = t_jst->tm_mon + 1;
      gps_day = t_jst->tm_mday;
      gps_hour = t_jst->tm_hour;
      gps_minute = t_jst->tm_min;
      gps_second = t_jst->tm_sec;

      gps_centisecond = gps.time.centisecond();
    }

    if (gps.location.isUpdated())
    {
      gps_latitude = gps.location.lat();
      gps_longitude = gps.location.lng();
    }
    if (gps.altitude.isUpdated())
    {
      gps_altitude = gps.altitude.meters();
    }
    if (gps.course.isUpdated())
    {
      gps_course = gps.course.deg();
    }
    if (gps.speed.isUpdated())
    {
      gps_speed = gps.speed.mps();
    }
  }
}
#pragma endregion

#pragma region ALTITUDE
// 超音波センサによる高度計のコード

/*
https://files.seeedstudio.com/wiki/RS485_Ultrasonic_level_Sensor/RS485-750cm-Ultrasonic-Level-Sensor.pdf
1. Read the calculated value of distance:
Command: 01 03 01 00 00 01 85 F6
Return: 01 03 02 02 F2 38 A1
Description: The slave address is 0x01, the calculated value of distance is 0x02F2, convert to decimal is 754, the distance
value = 754mm
2. Read the real-time distance value:
Command: 01 03 01 01 00 01 D4 36
Return: 01 03 02 02 EF F8 A8
Description: The slave address is 0x01, the real-time distance value is 0x02EF, convert to decimal is 754, the distance
value = 751mm
3. Read the temperature value:
Command: 01 03 01 02 00 01 24 36
Return: 01 03 02 01 2C B8 09
Description: The slave address is 0x01, the temperature is 0x012C, convert to decimal is 300, the temperature
value = 30.0℃
4. Modify the slave address:
Command: 01 06 02 00 00 05 48 71
Return: 01 06 02 00 00 05 48 71
Description: Change the address 0x01 to 0x05.
5. Modify the baud rate:
Command: 05 06 02 01 00 01 19 F6
Return: 05 06 02 01 00 01 19 F6
Description: Slave address is 0x05, change the baud rate to 0x01 (2400bps)
*/

unsigned char cmd[] = {0x01, 0x03, 0x01, 0x00, 0x00, 0x01, 0x85, 0xf6}; // 温度補正あり
// unsigned char cmd[] = {0x01, 0x03, 0x01, 0x01, 0x00, 0x01, 0xd4, 0x36}; // 温度補正無し
float altitude = 10.0; // 高度(m)

void altitude_task(void *pvParameters)
{
  uint8_t buff[16];
  while (1)
  {
    digitalWrite(ALT_REDE_PIN, HIGH);
    ALTSerial.write(cmd, 8);
    digitalWrite(ALT_REDE_PIN, LOW);
    int dataLen = ALTSerial.available();
    if (dataLen > 0)
    {
      uint8_t buff[16];
      int recvSize = ALTSerial.readBytes(buff, dataLen);

      uint16_t dist = buff[3] << 8 | buff[4];
      altitude = dist / 1000.0f;
    }
    delay(200);
  }
}

void GetAltitude()
{
}

void InitAltitude()
{
  pinMode(ALT_REDE_PIN, OUTPUT);
  ALTSerial.begin(ALT_SERIAL_BAUDRATE, SWSERIAL_8N1, ALT_RX, ALT_TX);
  xTaskCreatePinnedToCore(altitude_task, "altitude_task", 4096, NULL, 1, NULL, 0);
}
#pragma endregion

#pragma region TACHO
// 対気速度計のコード
hw_timer_t *timer = NULL;

volatile uint16_t tach_interrupts = 0;

boolean slit_rori_last = true;

uint32_t tach_rotation = 0;
uint32_t tach_last_time = 0;
uint32_t tach_delta_time = 0;

float air_speed = 30.0;

const uint32_t min_tach_delta = 150000;

void GetTacho()
{
  tach_delta_time = micros() - tach_last_time;

  if (tach_delta_time > min_tach_delta)
  {
    noInterrupts();
    if (tach_interrupts > 5)
    {
      tach_rotation = (uint32_t)((double)1000000000.0 * ((double)tach_interrupts / tach_delta_time));
      air_speed = tach_rotation * tach_rotation * -7.0 * pow(10, -16.0) + tach_rotation * 3.0 * pow(10, -7.0);
    }
    else
    {
      tach_rotation = 0;
      air_speed = 0;
    }
    interrupts();
    tach_interrupts = 0;
    tach_last_time = micros();
  }
}

void IRAM_ATTR tach_interrupt_count()
{
  if (digitalRead(TACHO_PIN[0]) != slit_rori_last)
  {
    tach_interrupts++;
    slit_rori_last = !slit_rori_last;
  }
}

void InitTacho()
{
  pinMode(TACHO_PIN[0], INPUT);
  pinMode(TACHO_PIN[1], INPUT);

  interrupts();
  timer = timerBegin(0, getApbFrequency() / 1000000, true);
  timerAttachInterrupt(timer, &tach_interrupt_count, true);
  timerAlarmWrite(timer, 20, true);
  timerAlarmEnable(timer);
  tach_last_time = micros();
}
#pragma endregion

#pragma region ROTATION_SPEED
// 回転数計のコード
const int SLIT_NUM = 36;
volatile uint16_t propeller_interrupts = 0;
uint32_t propeller_rotation = 106666;
uint32_t propeller_last_time = 0;
#define min_interrupts 5

uint32_t propeller_delta_time = 0;

const uint32_t min_propeller_delta = 500000;

void propeller_interrupt_count()
{
  propeller_interrupts++;
}

void attachPropeller()
{
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), propeller_interrupt_count, CHANGE);
}

void detachPropeller()
{
  detachInterrupt(digitalPinToInterrupt(RPM_PIN));
}

void GetRPM()
{
  propeller_delta_time = micros() - propeller_last_time;
  if (propeller_delta_time > min_propeller_delta)
  {
    detachPropeller(); // 計算中に値が変わらないようにする
    if (propeller_interrupts > min_interrupts)
    {
      // propeller_rotation = 割り込み数 / (スリット数 * 2.0)
      propeller_rotation = propeller_interrupts / (2.0 * SLIT_NUM) / (propeller_delta_time / 1000000.0) * 60.0;
    }
    else
    {
      propeller_rotation = 0;
    }
    attachPropeller();
    propeller_interrupts = 0;
    propeller_last_time = micros();
  }
}

void InitRPM()
{
  pinMode(RPM_PIN, INPUT_PULLUP);

  attachPropeller();
  interrupts();
  propeller_last_time = micros();
}
#pragma endregion

#pragma region LOG
// ログをmicroSDに保存
File fp;
constexpr char FILE_PATH[] = "/data.csv";
#define PRINT_COMMA fp.print(", ")

void InitSD();

void SDWriteTask(void *pvParameters)
{
  SPI.begin(SD_SPI_SCK_PIN, SD_SPI_MISO_PIN, SD_SPI_MOSI_PIN, SD_SPI_CS_PIN);

  if (!SD.begin(SD_SPI_CS_PIN, SPI, 25000000))
  {
    delay(5000);
    InitSD();
    vTaskDelete(NULL);
    return;
  }

  if (!SD.exists(FILE_PATH))
  {
    fp = SD.open(FILE_PATH, FILE_WRITE);

    // 基準値の設定
    fp.println(
        "Date,Time,Latitude,Longitude,GPSAltitude,GPSCourse,GPSSpeed,"
        "AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,MagX,MagY,MagZ,"
        "Roll_Mad6,Pitch_Mad6,Yaw_Mad6,Roll_Mad9,Pitch_Mad9,Yaw_Mad9,"
        "Roll_Mah6,Pitch_Mah6,Yaw_Mah6,Roll_Mah9,Pitch_Mah9,Yaw_Mah9,"
        "Temperature,Pressure,GroundPressure,BMPAltitude,Altitude,AirSpeed,"
        "PropellerRotationSpeed,Rudder,Elevator,Trim,RunningTime");
  }
  else
  {
    fp = SD.open(FILE_PATH, FILE_APPEND);
  }

  uint32_t start_time = millis();

  while (fp)
  {
    fp.printf("%04d-%02d-%02d", gps_year, gps_month, gps_day);
    PRINT_COMMA;
    fp.printf("%02d:%02d:%02d.%02d", gps_hour, gps_minute, gps_second, gps_centisecond);
    PRINT_COMMA;
    fp.printf("%.9f", gps_latitude);
    PRINT_COMMA;
    fp.printf("%.9f", gps_longitude);
    PRINT_COMMA;
    fp.print(gps_altitude);
    PRINT_COMMA;
    fp.print(gps_course);
    PRINT_COMMA;
    fp.print(gps_speed);
    PRINT_COMMA;
    fp.print(accelX);
    PRINT_COMMA;
    fp.print(accelY);
    PRINT_COMMA;
    fp.print(accelZ);
    PRINT_COMMA;
    fp.print(gyroX);
    PRINT_COMMA;
    fp.print(gyroY);
    PRINT_COMMA;
    fp.print(gyroZ);
    PRINT_COMMA;
    fp.print(magX);
    PRINT_COMMA;
    fp.print(magY);
    PRINT_COMMA;
    fp.print(magZ);
    PRINT_COMMA;
    fp.print(roll_mad6);
    PRINT_COMMA;
    fp.print(pitch_mad6);
    PRINT_COMMA;
    fp.print(yaw_mad6);
    PRINT_COMMA;
    fp.print(roll_mad9);
    PRINT_COMMA;
    fp.print(pitch_mad9);
    PRINT_COMMA;
    fp.print(yaw_mad9);
    PRINT_COMMA;
    fp.print(roll_mah6);
    PRINT_COMMA;
    fp.print(pitch_mah6);
    PRINT_COMMA;
    fp.print(yaw_mah6);
    PRINT_COMMA;
    fp.print(roll_mah9);
    PRINT_COMMA;
    fp.print(pitch_mah9);
    PRINT_COMMA;
    fp.print(yaw_mah9);
    PRINT_COMMA;
    fp.print(temperature);
    PRINT_COMMA;
    fp.print(pressure);
    PRINT_COMMA;
    fp.print(ground_pressure);
    PRINT_COMMA;
    fp.print(bmp_altitude);
    PRINT_COMMA;
    fp.print(altitude);
    PRINT_COMMA;
    fp.print(air_speed);
    PRINT_COMMA;
    fp.print(propeller_rotation);
    PRINT_COMMA;
    fp.printf("%.3f", rudder_rotation);
    PRINT_COMMA;
    fp.printf("%.3f", elevator_rotation);
    PRINT_COMMA;
    fp.printf("%d", trim);
    PRINT_COMMA;
    fp.print(millis() / 1000.0);
    PRINT_COMMA;
    fp.println();
    delay(50);

    if (millis() - start_time > 10000)
    {
      // 10秒ごとにファイルを閉じて再オープン
      break;
    }
  }

  fp.close();
  SD.end();
  InitSD();
  vTaskDelete(NULL);
}

void InitSD()
{
  xTaskCreatePinnedToCore(SDWriteTask, "SDWriteTask", 4096, NULL, 1, NULL, 0);
}
#pragma endregion

#pragma region JSON
// JSONを作成する
JsonDocument json_data, json_array;
char json_string[4096];

void CreateJson()
{
  // JSONに変換したいデータを連想配列で指定する
  char date_str[32], time_str[32];
  sprintf(date_str, "%04d-%02d-%02d", gps_year, gps_month, gps_day);
  sprintf(time_str, "%02d:%02d:%02d.%02d", gps_hour, gps_minute, gps_second, gps_centisecond);
  json_data["Date"] = date_str;
  json_data["Time"] = time_str;
  json_data["Latitude"] = gps_latitude;
  json_data["Longitude"] = gps_longitude;
  json_data["GPSAltitude"] = gps_altitude;
  json_data["GPSCourse"] = gps_course;
  json_data["GPSSpeed"] = gps_speed;
  json_data["AccelX"] = accelX;
  json_data["AccelY"] = accelY;
  json_data["AccelZ"] = accelZ;
  json_data["GyroX"] = gyroX;
  json_data["GyroY"] = gyroY;
  json_data["GyroZ"] = gyroZ;
  json_data["MagX"] = magX;
  json_data["MagY"] = magY;
  json_data["MagZ"] = magZ;
  json_data["Roll_Mad6"] = roll_mad6;
  json_data["Pitch_Mad6"] = pitch_mad6;
  json_data["Yaw_Mad6"] = yaw_mad6;
  json_data["Roll_Mad9"] = roll_mad9;
  json_data["Pitch_Mad9"] = pitch_mad9;
  json_data["Yaw_Mad9"] = yaw_mad9;
  json_data["Roll_Mah6"] = roll_mah6;
  json_data["Pitch_Mah6"] = pitch_mah6;
  json_data["Yaw_Mah6"] = yaw_mah6;
  json_data["Roll_Mah9"] = roll_mah9;
  json_data["Pitch_Mah9"] = pitch_mah9;
  json_data["Yaw_Mah9"] = yaw_mah9;
  json_data["Temperature"] = temperature;
  json_data["Pressure"] = pressure;
  json_data["GroundPressure"] = ground_pressure;
  json_data["BMPAltitude"] = bmp_altitude;
  json_data["Altitude"] = altitude;
  json_data["AirSpeed"] = air_speed;
  json_data["PropellerRotationSpeed"] = propeller_rotation;
  json_data["Rudder"] = rudder_rotation;
  json_data["Elevator"] = elevator_rotation;
  json_data["Trim"] = trim;
  json_data["LoRaRSSI"] = 0;
  json_data["RunningTime"] = millis() / 1000.0;

  json_array["data"] = json_data;

  // JSONフォーマットの文字列に変換する
  serializeJson(json_array, json_string, sizeof(json_string));
}
#pragma endregion

#pragma region SERVER

#pragma region LORA
#pragma pack(1)
struct LoRaData
{
  int16_t GPSYear;
  int8_t GPSMonth;
  int8_t GPSDay;
  int8_t GPSHour;
  int8_t GPSMinute;
  int8_t GPSSecond;
  int8_t GPSCentiSecond;
  double Latitude;
  double Longitude;
  double GPSAltitude;
  double GPSCourse;
  double GPSSpeed;
  float AccelX;
  float AccelY;
  float AccelZ;
  float GyroX;
  float GyroY;
  float GyroZ;
  int16_t MagX;
  int16_t MagY;
  int16_t MagZ;
  float Roll_Mad6;
  float Pitch_Mad6;
  float Yaw_Mad6;
  float Roll_Mad9;
  float Pitch_Mad9;
  float Yaw_Mad9;
  float Roll_Mah6;
  float Pitch_Mah6;
  float Yaw_Mah6;
  float Roll_Mah9;
  float Pitch_Mah9;
  float Yaw_Mah9;
  float Temperature;
  float Pressure;
  float GroundPressure;
  float BMPAltitude;
  float Altitude;
  float AirSpeed;
  float PropellerRotationSpeed;
  float Rudder;
  float Elevator;
  int Trim;
  float RunningTime;
};

// 送信データに巡回符号検査を付加（SHA256だとデータサイズが大きく不向きなため）
struct LoRaPacket
{
  LoRaData data;
  uint32_t CRC32;
};

// パケットサイズは200Byte以下にすること
static_assert(sizeof(LoRaPacket) <= 200, "LoRaPacket size is too large! Keep packet size under 200 Bytes.");

void LoRaSendTask(void *pvParameters)
{
  while (1)
  {
    LoRaPacket lora_packet;

    lora_packet.data.GPSYear = gps_year;
    lora_packet.data.GPSMonth = gps_month;
    lora_packet.data.GPSDay = gps_day;
    lora_packet.data.GPSHour = gps_hour;
    lora_packet.data.GPSMinute = gps_minute;
    lora_packet.data.GPSSecond = gps_second;
    lora_packet.data.GPSCentiSecond = gps_centisecond;
    lora_packet.data.Latitude = gps_latitude;
    lora_packet.data.Longitude = gps_longitude;
    lora_packet.data.GPSAltitude = gps_altitude;
    lora_packet.data.GPSCourse = gps_course;
    lora_packet.data.GPSSpeed = gps_speed;
    lora_packet.data.AccelX = accelX;
    lora_packet.data.AccelY = accelY;
    lora_packet.data.AccelZ = accelZ;
    lora_packet.data.GyroX = gyroX;
    lora_packet.data.GyroY = gyroY;
    lora_packet.data.GyroZ = gyroZ;
    lora_packet.data.MagX = magX;
    lora_packet.data.MagY = magY;
    lora_packet.data.MagZ = magZ;
    lora_packet.data.Roll_Mad6 = roll_mad6;
    lora_packet.data.Pitch_Mad6 = pitch_mad6;
    lora_packet.data.Yaw_Mad6 = yaw_mad6;
    lora_packet.data.Roll_Mad9 = roll_mad9;
    lora_packet.data.Pitch_Mad9 = pitch_mad9;
    lora_packet.data.Yaw_Mad9 = yaw_mad9;
    lora_packet.data.Roll_Mah6 = roll_mah6;
    lora_packet.data.Pitch_Mah6 = pitch_mah6;
    lora_packet.data.Yaw_Mah6 = yaw_mah6;
    lora_packet.data.Roll_Mah9 = roll_mah9;
    lora_packet.data.Pitch_Mah9 = pitch_mah9;
    lora_packet.data.Yaw_Mah9 = yaw_mah9;
    lora_packet.data.Temperature = temperature;
    lora_packet.data.Pressure = pressure;
    lora_packet.data.GroundPressure = ground_pressure;
    lora_packet.data.BMPAltitude = bmp_altitude;
    lora_packet.data.Altitude = altitude;
    lora_packet.data.AirSpeed = air_speed;
    lora_packet.data.PropellerRotationSpeed = propeller_rotation;
    lora_packet.data.Rudder = rudder_rotation;
    lora_packet.data.Elevator = elevator_rotation;
    lora_packet.data.Trim = trim;
    lora_packet.data.RunningTime = millis() / 1000.0;

    lora_packet.CRC32 = (~crc32_le((uint32_t)~(0xffffffff), (const uint8_t *)&(lora_packet.data), sizeof(lora_packet.data))) ^ 0xffffffff;

    LoRaSerial.write((byte *)&lora_packet, sizeof(lora_packet));

    delay(100);
  }
}

void InitLoRa()
{
  LoRaSerial.begin(LORA_SERIAL_BAUDRATE, SERIAL_8N1, LORA_RX, LORA_TX);
  xTaskCreatePinnedToCore(LoRaSendTask, "LoRaSendTask", 8192, NULL, 1, NULL, 0);
}
#pragma endregion

void sha256(const char *p_payload, unsigned char *p_hmacResult)
{
  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
  mbedtls_md_starts(&ctx);
  mbedtls_md_hmac_update(&ctx, (const unsigned char *)p_payload, strlen(p_payload));
  mbedtls_md_finish(&ctx, p_hmacResult);
  mbedtls_md_free(&ctx);
}

// HTTPサーバーでの処理
WebServer server(80);

void handleRoot()
{
  server.send(HTTP_CODE_OK, "text/plain", "index");
}
void handleNotFound()
{
  server.send(HTTP_CODE_NOT_FOUND, "text/plain", "Not Found");
}
void handleSetGroundPressure()
{
  if (server.hasArg("Pressure"))
  {
    ground_pressure = server.arg("Pressure").toFloat();
  }
  server.send(HTTP_CODE_OK, "text/plain", String(ground_pressure));
}
void handleGetGroundPressure()
{
  server.send(HTTP_CODE_OK, "text/plain", String(ground_pressure));
}
void handleGetMeasurementData()
{
  unsigned char SHA256[32];
  sha256(json_string, SHA256);
  char SHA256_str[64 + 1];
  for (int i = 0; i < 32; i++)
  {
    sprintf((char *)&SHA256_str[i * 2], "%02x", SHA256[i]);
  }
  server.sendHeader("SHA256", SHA256_str);

  server.send(HTTP_CODE_OK, "application/json", json_string);
}

void InitServer()
{
  setupOTA("HPA");
  server.on("/", handleRoot);
  server.on("/SetGroundPressure", handleSetGroundPressure);
  server.on("/GetGroundPressure", handleGetGroundPressure);
  server.on("/GetMeasurementData", handleGetMeasurementData);
  server.onNotFound(handleNotFound);
  server.begin();
}
#pragma endregion

#pragma region AWS
#define THINGNAME "HPA"
// The MQTT topics that this device should publish/subscribe
#define AWS_IOT_PUBLISH_TOPIC "hpa/pub"

WiFiClientSecure net = WiFiClientSecure();
MQTTClient client = MQTTClient(256);

void connectAWS()
{
  WiFi.reconnect();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  IPAddress localIP = WiFi.localIP();  
  WiFi.config(
    IPAddress(localIP[0], localIP[1], localIP[2], 140), 
    WiFi.gatewayIP(), 
    WiFi.subnetMask(),
    IPAddress(8, 8, 8, 8),
    IPAddress(8, 8, 4, 4)
  );

  WiFi.reconnect();
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
  }

  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.begin(AWS_IOT_ENDPOINT, 8883, net);

  Serial.println("Connecting to AWS IoT...");

  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }

  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }

  Serial.println("AWS IoT Connected!");
}

void publishMessage()
{
  client.publish(AWS_IOT_PUBLISH_TOPIC, json_string);
}

void AWS_task(void *parameter)
{
  connectAWS();
  while (1)
  {
    if (!client.loop() || WiFi.status() != WL_CONNECTED)
    {
      Serial.println("AWS IoT Disconnected!");
      connectAWS();
    }
    publishMessage();
    delay(100);
  }
}
#pragma endregion

void watchdog_task(void *pvParameters)
{
  while (1)
  {
    if (watchdog_count > 5)
    {
      Serial.println("Watchdog triggered! Restarting...");
      ESP.restart();
    }
    watchdog_count++;
    delay(1000);
  }
}

void setup()
{
  Serial.begin(USB_BAUDRATE);

  auto cfg = M5.config();
  CoreS3.begin(cfg);
  CoreS3.Ex_I2C.begin();

  // Configure and start the WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  wifiUdp.begin(UDP_PORT);

  InitBMP280();
  delay(100);
  InitBMI270();
  delay(100);
  InitGPS();
  delay(100);
  InitTacho();
  delay(100);
  InitRPM();
  delay(100);
  InitAltitude();
  delay(100);

  InitSD();
  delay(100);
  InitServer();
  delay(100);
  InitLoRa();
  delay(100);

  // 最低でも8kBのスタックサイズが必要
  xTaskCreatePinnedToCore(AWS_task, "AWS_task", 16384, NULL, 1, NULL, 1);

  // 5秒以上データが更新されない場合にESP32をリセットする
  watchdog_count = 0;
  xTaskCreatePinnedToCore(watchdog_task, "watchdog_task", 2048, NULL, 1, NULL, 1);
}

void loop()
{
  CoreS3.update();

  if (WiFi.status() == WL_CONNECTED)
  {
    server.handleClient();
  }
  if (wifiUdp.parsePacket() == sizeof(ControlData))
  {
    ControlData controlData;
    wifiUdp.read((char *)(&controlData), sizeof(ControlData));

    rudder_rotation = controlData.rudder;
    elevator_rotation = controlData.elevator;
    trim = controlData.trim;
  }

  GetBMP280();
  GetBMI270();
  GetGPS();
  GetAltitude();
  GetTacho();
  GetRPM();
  CreateJson();

  // Display
  static uint32_t last_print = 0;
  if (millis() - last_print > 500)
  {
    last_print = millis();

    CoreS3.Display.setCursor(0, 0);
    CoreS3.Display.clear();

    CoreS3.Display.printf("Temperature: %.2f *C\r\n", temperature);
    CoreS3.Display.printf("Pressure: %.2f Pa\r\n", pressure);
    CoreS3.Display.printf("Altitude: %.2f m\r\n", altitude);
    CoreS3.Display.printf("GPS Date: %04d-%02d-%02d\r\n", gps_year, gps_month, gps_day);
    CoreS3.Display.printf("GPS Time: %02d:%02d:%02d.%02d\r\n", gps_hour, gps_minute, gps_second, gps_centisecond);
    CoreS3.Display.printf("Latitude: %.9f, Longitude: %.9f\r\n", gps_latitude, gps_longitude);
    CoreS3.Display.printf("GPS Altitude: %.2f m\r\n", gps_altitude);
    CoreS3.Display.printf("GPS Course: %.2f deg\r\n", gps_course);
    CoreS3.Display.printf("GPS Speed: %.2f m/s\r\n", gps_speed);
    CoreS3.Display.printf("ax: %.2f, ay: %.2f, az: %.2f\r\n", accelX, accelY, accelZ);
    CoreS3.Display.printf("gx: %.2f, gy: %.2f, gz: %.2f\r\n", gyroX, gyroY, gyroZ);
    CoreS3.Display.printf("mx: %d, my: %d, mz: %d\r\n", magX, magY, magZ);
    CoreS3.Display.printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", roll_mad9, pitch_mad9, yaw_mad9);
    CoreS3.Display.printf("Air Speed: %.2f\r\n", air_speed);
    CoreS3.Display.printf("Propeller Rotation Speed: %d\r\n", propeller_rotation);
    CoreS3.Display.printf("Rudder: %.2f, Elevator: %.2f, Trim: %d\r\n", rudder_rotation, elevator_rotation, trim);
    CoreS3.Display.printf("Wi-Fi Status: %s\r\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Not Connected");
    CoreS3.Display.printf("IP Address: %s\r\n", WiFi.localIP().toString().c_str());
    CoreS3.Display.printf("Watchdog count: %d\r\n", watchdog_count);
  }

  watchdog_count = 0;
}
