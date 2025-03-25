# HPA計測系統

## Hardware

* [M5Stack CoreS3 SE](https://www.switch-science.com/products/9690)
* [M5Stack Core2用ポート拡張モジュール](https://www.switch-science.com/products/8308)
* [M5Stack用GNSSモジュール 気圧/IMU/地磁気センサ付き（NEO-M9N/BMP280/BMI270/BMM150）](https://www.switch-science.com/products/9276?_pos=2&_sid=b1554245d&_ss=r)
* microSD 16GB

## Cloud Service

* [AWS IoT Core](https://aws.amazon.com/jp/iot-core)

## CoreS3 SE Pinout

* Port A ... G1, G2 (Air Speed)
* Port B ... G8, G9 (Altitude)
* Port C ... G18, G17 (LoRa)
* Port D ... G14, G10 (G14 = Toggle RS485 inout for altitude sensor, G10 = Propeller Rotation Speed)
* Port E ... G13, G5 (Control board)

## GNSS

* PPS ... G0 (Not used)
* TX ... G6
* RX ... G7

### AWS参考情報

- [マイコン ESP32 を使って AWS IoT Core と Pub/Sub 通信するまで](https://dev.classmethod.jp/articles/esp32-aws-iot-pubsub-basic)
- [AWS IoT Coreで受け取ったデバイスデータをAmazon DynamoDBに保存してみた](https://dev.classmethod.jp/articles/saving-device-data-from-iot-core-to-dynamodb)
