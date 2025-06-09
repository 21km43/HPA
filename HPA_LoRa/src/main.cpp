#include <Arduino.h>
#include <esp32c3/rom/crc.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <mbedtls/md.h>
#include <vector>

// #define LORA_CONFIG

#define LORA_SERIAL Serial0
constexpr uint8_t LORA_RX = 8;
constexpr uint8_t LORA_TX = 7;
constexpr uint8_t M0 = 5;
constexpr uint8_t M1 = 6;

#ifdef LORA_CONFIG
#include <config_E220.h>
config_E220 e220conf(LORA_SERIAL, M0, M1);
byte responcedata[11] = {0};

byte set_data_buff[11] = {0x00};
#endif
constexpr char SSID[] = "HPA_LoRa";
constexpr char PASSPHRASE[] = "HPA_Password";

const IPAddress localIP(192, 168, 200, 140); // 自身のIPアドレス
const IPAddress gateway(192, 168, 200, 157); // デフォルトゲートウェイ
const IPAddress subnet(255, 255, 255, 0);    // サブネットマスク

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
  float Trim;
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

// JSON構造体、文字列
JsonDocument json_array, json_data;
char json_string[4096];
// json_stringのSHA256（HEX文字列）
char SHA256_str[64 + 1];
// LoRaのパケットとRSSI
std::vector<uint8_t> lora_packet_buff;
bool lora_received = false;
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
    while (LORA_SERIAL.available() > 0)
    {
      lora_packet_buff.push_back(LORA_SERIAL.read());
      if (lora_packet_buff.size() == sizeof(LoRaPacket))
      {
        uint32_t crc32 = (~crc32_le((uint32_t)~(0xffffffff), lora_packet_buff.data(), sizeof(LoRaData))) ^ 0xffffffff;
        if ((*(uint32_t *)(lora_packet_buff.data() + sizeof(LoRaData))) == crc32)
        {
          memcpy(&lora_packet, lora_packet_buff.data(), sizeof(LoRaPacket));
          lora_packet_buff.clear();
          delay(10);
          lora_rssi = LORA_SERIAL.read() - 255;
          lora_received = true;
          break;
        }
        else
        {
          lora_packet_buff.erase(lora_packet_buff.begin());
        }
      }
    }

    if (lora_received)
    {
      lora_received = false;

      // JSONに変換したいデータを連想配列で指定する
      char date_str[32], time_str[32];
      sprintf(date_str, "%04d-%02d-%02d", lora_packet.data.GPSYear, lora_packet.data.GPSMonth, lora_packet.data.GPSDay);
      sprintf(time_str, "%02d:%02d:%02d.%02d", lora_packet.data.GPSHour, lora_packet.data.GPSMinute, lora_packet.data.GPSSecond, lora_packet.data.GPSCentiSecond);
      json_data["Date"] = date_str;
      json_data["Time"] = time_str;
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
      json_data["LoRaRSSI"] = lora_rssi;
      json_data["RunningTime"] = lora_packet.data.RunningTime;

      json_array["data"] = json_data;

      // JSONフォーマットの文字列に変換する
      serializeJson(json_array, json_string, sizeof(json_string));

      unsigned char SHA256[32];
      sha256(json_string, SHA256);
      for (int i = 0; i < 32; i++)
      {
        sprintf((char *)&SHA256_str[i * 2], "%02x", SHA256[i]);
      }

      Serial.println(json_string);
    }

    delay(10);
  }
}

void InitLoRa()
{
  LORA_SERIAL.begin(9600, SERIAL_8N1, LORA_RX, LORA_TX);
#ifndef LORA_CONFIG
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
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(localIP, gateway, subnet);
  WiFi.softAP(SSID, PASSPHRASE);

  server.on("/", handleRoot);
  server.on("/GetMeasurementData", handleGetMeasurementData);
  server.onNotFound(handleNotFound);
  server.begin();
}
#pragma endregion

void setup()
{
  Serial.begin(115200);

  InitLoRa();
  delay(100);
#ifndef LORA_CONFIG
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  InitServer();
  delay(100);
#endif
}

void loop()
{
#ifdef LORA_CONFIG
  String serial_input;
  if (Serial.available() > 0)
  {
    serial_input = Serial.readStringUntil('\n');
    serial_input.trim();
  }

  // show command
  if (serial_input.equals("show") > 0)
  {
    e220conf.Show();
  }
  // address command
  if (serial_input.startsWith("address "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String addressStr = serial_input.substring(spaceIndex + 1);
    int address = addressStr.toInt();
    e220conf.SetAddress(address, set_data_buff);
    byte w_data_buff[2];
    w_data_buff[0] = set_data_buff[0];
    w_data_buff[1] = set_data_buff[1];
    e220conf.WriteResister(0x00, 0x02, w_data_buff);
  }
  // bandrate command
  if (serial_input.startsWith("baudrate "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String baudrateStr = serial_input.substring(spaceIndex + 1);
    int baudrate = baudrateStr.toInt();
    e220conf.SetUartBaudrate(baudrate, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[2];
    e220conf.WriteResister(0x02, 0x01, w_data_buff);
  }
  // sf command
  if (serial_input.startsWith("sf "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String sfStr = serial_input.substring(spaceIndex + 1);
    int sf = sfStr.toInt();
    e220conf.SetSF(sf, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[2];
    e220conf.WriteResister(0x02, 0x01, w_data_buff);
  }
  // bw command
  if (serial_input.startsWith("bw "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String bwStr = serial_input.substring(spaceIndex + 1);
    int bw = bwStr.toInt();
    e220conf.SetBW(bw, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[2];
    e220conf.WriteResister(0x02, 0x01, w_data_buff);
  }
  // subpacketlength command
  if (serial_input.startsWith("subpacket "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String subpacketlengthStr = serial_input.substring(spaceIndex + 1);
    int subpacketlength = subpacketlengthStr.toInt();
    e220conf.SetSubpacketLength(subpacketlength, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[3];
    e220conf.WriteResister(0x03, 0x01, w_data_buff);
  }
  // power command
  if (serial_input.startsWith("power "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String powerStr = serial_input.substring(spaceIndex + 1);
    int power = powerStr.toInt();
    e220conf.SetTxPower(power, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[3];
    e220conf.WriteResister(0x03, 0x01, w_data_buff);
  }
  // channel command
  if (serial_input.startsWith("channel "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String channelStr = serial_input.substring(spaceIndex + 1);
    int channel = channelStr.toInt();
    e220conf.SetChannel(channel, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[4];
    e220conf.WriteResister(0x04, 0x01, w_data_buff);
  }
  // worcycle command
  if (serial_input.startsWith("worcycle "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String worcycleStr = serial_input.substring(spaceIndex + 1);
    int worcycle = worcycleStr.toInt();
    e220conf.SetWorCycle(worcycle, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[5];
    e220conf.WriteResister(0x05, 0x01, w_data_buff);
  }
  // rssi_noise command
  if (serial_input.startsWith("rssi_noise "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String rssi_noiseStr = serial_input.substring(spaceIndex + 1);
    int rssi_noise = rssi_noiseStr.toInt();
    e220conf.SetRssiNoiseAvailable(rssi_noise, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[3];
    e220conf.WriteResister(0x03, 0x01, w_data_buff);
  }
  // rssi_byte command
  if (serial_input.startsWith("rssi_byte "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String rssi_byteStr = serial_input.substring(spaceIndex + 1);
    int rssi_byte = rssi_byteStr.toInt();
    e220conf.SetRssiByteAvailable(rssi_byte, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[5];
    e220conf.WriteResister(0x05, 0x01, w_data_buff);
  }
  // tx_method command
  if (serial_input.startsWith("tx_method "))
  {
    int spaceIndex = serial_input.indexOf(' ');
    String tx_methodStr = serial_input.substring(spaceIndex + 1);
    int tx_method = tx_methodStr.toInt();
    e220conf.SetTxMethod(tx_method, set_data_buff);
    byte w_data_buff[1];
    w_data_buff[0] = set_data_buff[5];
    e220conf.WriteResister(0x05, 0x01, w_data_buff);
  }
  // default command
  if (serial_input.equals("default") > 0)
  {
    e220conf.SetDefault();
  }

  delay(10);
#else
  server.handleClient();
#endif
}
