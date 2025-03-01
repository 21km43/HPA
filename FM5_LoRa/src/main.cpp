#include <Arduino.h>
#include <SoftwareSerial.h>
#include <M5_LoRa_E220_JP.h>
#include <esp32/rom/crc.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <mbedtls/md.h>

#define LORA_CONFIG

#define LORA_SERIAL Serial0
constexpr uint8_t LORA_RX = 20;
constexpr uint8_t LORA_TX = 21;

constexpr char SSID[] = "FM5_LoRa";
constexpr char PASSPHRASE[] = "FM5_Password";

const IPAddress localIP(192, 168, 43, 141); // 自身のIPアドレス
const IPAddress gateway(192, 168, 43, 1);   // デフォルトゲートウェイ
const IPAddress subnet(255, 255, 255, 0);   // サブネットマスク

#pragma region OTA
void ota_handle(void *parameter)
{
  for (;;)
  {
    ArduinoOTA.handle();
    delay(1000);
  }
}

void setupOTA(const char *nameprefix, const char *ssid, const char *password)
{
  // Configure the hostname
  uint16_t maxlen = strlen(nameprefix) + 7;
  char *fullhostname = new char[maxlen];
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(fullhostname, maxlen, "%s-%02x%02x%02x", nameprefix, mac[3], mac[4], mac[5]);
  ArduinoOTA.setHostname(fullhostname);
  delete[] fullhostname;

  // Configure and start the WiFi station
  WiFi.mode(WIFI_STA);
  WiFi.config(localIP, gateway, subnet);
  WiFi.begin(ssid, password);

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

#pragma region SERVER

#pragma region LORA
// LoRa関連
LoRa_E220_JP lora;
struct LoRaConfigItem_t lora_config;

struct LoRaData
{
  int GPSYear;
  int GPSMonth;
  int GPSDay;
  int GPSHour;
  int GPSMinute;
  int GPSSecond;
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
  float Trim;
  float RunningTime;
};

// 送信データに巡回符号検査を付加（SHA256だとデータサイズが大きく不向きなため）
struct LoRaPacket
{
  LoRaData data;
  uint32_t CRC32;
};

// パケットサイズは200バイト以下にすること
static_assert(sizeof(LoRaPacket) <= 200, "LoRaPacket size is too large! Keep packet size under 200 Bytes.");

constexpr char AID[] = "7777";
// JSON構造体、文字列
JsonDocument json_array, json_data;
char json_string[4096];
// json_stringのSHA256（HEX文字列）
char SHA256_str[64 + 1];
// LoRaのパケットとRSSI
LoRaPacket lora_packet;
int lora_rssi = 0;

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

void LoRaRecvTask(void *pvParameters)
{
  while (1)
  {
    RecvFrame_t lora_frame;

    if (lora.RecieveFrame(&lora_frame) == 0 && lora_frame.recv_data_len == sizeof(LoRaPacket) && (*((LoRaPacket *)(lora_frame.recv_data))).CRC32 == ((~crc32_le((uint32_t)~(0xffffffff), (const uint8_t *)&((*((LoRaPacket *)(lora_frame.recv_data))).data), sizeof(LoRaData))) ^ 0xffffffff))
    {
      lora_rssi = lora_frame.rssi;
      memcpy(&lora_packet, lora_frame.recv_data, lora_frame.recv_data_len);

      // JSONに変換したいデータを連想配列で指定する
      json_data["Latitude"] = lora_packet.data.Latitude;
      json_data["Longitude"] = lora_packet.data.Longitude;
      json_data["GPSAltitude"] = lora_packet.data.GPSAltitude;
      json_data["GPSCourse"] = lora_packet.data.GPSCourse;
      json_data["GPSSpeed"] = lora_packet.data.GPSSpeed;
      json_data["AccelX"] = lora_packet.data.AccelX;
      json_data["AccelY"] = lora_packet.data.AccelY;
      json_data["AccelZ"] = lora_packet.data.AccelZ;
      json_data["GyroX"] = lora_packet.data.GyroX;
      json_data["GyroY"] = lora_packet.data.GyroY;
      json_data["GyroZ"] = lora_packet.data.GyroZ;
      json_data["MagX"] = lora_packet.data.MagX;
      json_data["MagY"] = lora_packet.data.MagY;
      json_data["MagZ"] = lora_packet.data.MagZ;
      json_data["Roll_Mad6"] = lora_packet.data.Roll_Mad6;
      json_data["Pitch_Mad6"] = lora_packet.data.Pitch_Mad6;
      json_data["Yaw_Mad6"] = lora_packet.data.Yaw_Mad6;
      json_data["Roll_Mad9"] = lora_packet.data.Roll_Mad9;
      json_data["Pitch_Mad9"] = lora_packet.data.Pitch_Mad9;
      json_data["Yaw_Mad9"] = lora_packet.data.Yaw_Mad9;
      json_data["Roll_Mah6"] = lora_packet.data.Roll_Mah6;
      json_data["Pitch_Mah6"] = lora_packet.data.Pitch_Mah6;
      json_data["Yaw_Mah6"] = lora_packet.data.Yaw_Mah6;
      json_data["Roll_Mah9"] = lora_packet.data.Roll_Mah9;
      json_data["Pitch_Mah9"] = lora_packet.data.Pitch_Mah9;
      json_data["Yaw_Mah9"] = lora_packet.data.Yaw_Mah9;
      json_data["Temperature"] = lora_packet.data.Temperature;
      json_data["Pressure"] = lora_packet.data.Pressure;
      json_data["GroundPressure"] = lora_packet.data.GroundPressure;
      json_data["BMPAltitude"] = lora_packet.data.BMPAltitude;
      json_data["Altitude"] = lora_packet.data.Altitude;
      json_data["AirSpeed"] = lora_packet.data.AirSpeed;
      json_data["PropellerRotationSpeed"] = lora_packet.data.PropellerRotationSpeed;
      json_data["Rudder"] = lora_packet.data.Rudder;
      json_data["Elevator"] = lora_packet.data.Elevator;
      json_data["Trim"] = lora_packet.data.Trim;
      json_data["RunningTime"] = lora_packet.data.RunningTime;

      char time_str[32];
      sprintf(time_str, "%d-%d-%d %d:%02d:%02d", lora_packet.data.GPSYear, lora_packet.data.GPSMonth, lora_packet.data.GPSDay, lora_packet.data.GPSHour, lora_packet.data.GPSMinute, lora_packet.data.GPSSecond);
      json_array["AID"] = AID;
      json_array["Time"] = time_str;
      json_array["data"] = json_data;

      // JSONフォーマットの文字列に変換する
      serializeJson(json_array, json_string, sizeof(json_string));

      unsigned char SHA256[32];
      sha256(json_string, SHA256);
      for (int i = 0; i < 32; i++)
      {
        sprintf((char *)&SHA256_str[i * 2], "%02x", SHA256[i]);
      }
    }

    delay(100);
  }
}

void InitLoRa()
{
  lora.Init(&LORA_SERIAL, 9600, SERIAL_8N1, LORA_RX, LORA_TX);

  lora.SetDefaultConfigValue(lora_config);

  lora_config.own_address = 0x0000;
  lora_config.baud_rate = BAUD_9600;
  lora_config.air_data_rate = BW500K_SF5;
  lora_config.subpacket_size = SUBPACKET_200_BYTE;
  lora_config.rssi_ambient_noise_flag = RSSI_AMBIENT_NOISE_ENABLE;
  lora_config.transmitting_power = TX_POWER_13dBm;
  lora_config.own_channel = 0x0A;
  lora_config.rssi_byte_flag = RSSI_BYTE_ENABLE;
  lora_config.transmission_method_type = UART_P2P_MODE;
  lora_config.lbt_flag = LBT_DISABLE;
  lora_config.wor_cycle = WOR_2000MS;
  lora_config.encryption_key = 0x1234;
  lora_config.target_address = 0x0000;
  lora_config.target_channel = 0x0A;

#ifdef LORA_CONFIG
  if (lora.InitLoRaSetting(lora_config) != 0)
  {
    while (1)
    {
      Serial.println("LoRa configure failed!");
      Serial.println("Please pull the M0, M1 to HIGH if you want to configure the LoRa module.");
      delay(1000);
    }
  }
  else
  {
    while (1)
    {
      Serial.println("LoRa Configuretion OK!");
      delay(1000);
    }
  }
#else
  xTaskCreatePinnedToCore(LoRaRecvTask, "LoRaRecvTask", 8192, NULL, 1, NULL, 0);
#endif
}
#pragma endregion

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
void handleGetMeasurementData()
{
  server.sendHeader("SHA256", SHA256_str);
  server.send(HTTP_CODE_OK, "application/json", json_string);
}

void InitServer()
{
  setupOTA("FM5_LoRa", SSID, PASSPHRASE);

  server.on("/", handleRoot);
  server.on("/GetMeasurementData", handleGetMeasurementData);
  server.onNotFound(handleNotFound);
  server.begin();
}
#pragma endregion

void setup()
{
  Serial.begin(115200);

  InitServer();
  delay(100);
  InitLoRa();
  delay(100);
}

void loop()
{
  server.handleClient();
}
