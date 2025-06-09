# HPA計測系統

## TODO
- M5Stack CoreS3 SE用のプログラムになっているので今年の利用が終了次第、以下の変更を加える
  - 6軸+3軸センサの設定変更（Ex_I2C -> In_I2C, I2C_NUM_1 -> I2C_NUM_0）
  - 気圧計・温度計削除（高度計から温度の取得が可能、AWS側のルールも変更）

## Hardware

### M5Stack CoreS3 & Boards

* [M5Stack CoreS3](https://www.switch-science.com/products/8960)
* [M5Stack Core2用ポート拡張モジュール](https://www.switch-science.com/products/8308)
* [M5Stack用GPSユニット v1.1](https://www.switch-science.com/products/10037)
* microSD (up to 16GB)

### Sensors

* [EE-SX460-P1](https://www.fa.omron.co.jp/products/family/3605)
* [RE12D-100-201-1](https://www.nidec-components.com/j/product/detail/00000062)
* [RS485 750cm Ultrasonic Level Sensor](https://www.seeedstudio.com/RS485-750cm-Ultrasonic-Level-Sensor-p-5587.html)

### Wireless unit

* [E220-900T22S(JP)](https://dragon-torch.tech/rf-modules/lora/e220-900t22s-jp)

### Others

* [RS485 driver unit](https://github.com/21km43/UART-RS485)

## Software

* [PlatformIO](https://platformio.org)
* [VSCode](https://code.visualstudio.com)

## Cloud Service

* [AWS IoT Core](https://aws.amazon.com/jp/iot-core)
* [直近のJSONデータ取得用URL](https://62u95gbc60.execute-api.us-east-1.amazonaws.com/test/items/hpa/latest)

## CoreS3 SE Pinout

* Port A ... G1, G2 (Air Speed)
* Port B ... G8, G9 (Altitude)
* Port C ... G18, G17 (LoRa)
* Port D ... G14, G10 (G14 = Toggle RS485 inout for altitude sensor, G10 = Propeller Rotation Speed)
* Port E ... G13, G5 (GPS)

## GNSS

* PPS ... G0 (Not used)
* TX ... G6
* RX ... G7

## ログ保存方法

本システムではログの保存について以下の2つをサポートしている

- microSDへの保存
- MQTTによる外部サーバーへのアップロード

MQTTでは、モノの名前を`HPA`、Publish時のトピック名を`hpa/pub`としている。以下、MQTTブローカーをAWSに構築した場合の情報を掲載する。

### 参考情報

- [マイコン ESP32 を使って AWS IoT Core と Pub/Sub 通信するまで](https://dev.classmethod.jp/articles/esp32-aws-iot-pubsub-basic)
- [AWS IoT Coreで受け取ったデバイスデータをAmazon DynamoDBに保存してみた](https://dev.classmethod.jp/articles/saving-device-data-from-iot-core-to-dynamodb)

なお、ポリシーについては`iot:Connect`と`iot:Publish`のみ許可すれば良い。

### SQLステートメント

ルールで以下のSQLステートメントを適用

```sql
SELECT
  topic(1) AS ID,
  timestamp() AS Timestamp,
  data.Date AS Date,
  data.Time AS Time,
  data.Latitude AS Latitude,
  data.Longitude AS Longitude,
  data.GPSAltitude AS GPSAltitude,
  data.GPSCourse AS GPSCourse,
  data.GPSSpeed AS GPSSpeed,
  data.AccelX AS AccelX,
  data.AccelY AS AccelY,
  data.AccelZ AS AccelZ,
  data.GyroX AS GyroX,
  data.GyroY AS GyroY,
  data.GyroZ AS GyroZ,
  data.MagX AS MagX,
  data.MagY AS MagY,
  data.MagZ AS MagZ,
  data.Roll_Mad6 AS Roll_Mad6,
  data.Pitch_Mad6 AS Pitch_Mad6,
  data.Yaw_Mad6 AS Yaw_Mad6,
  data.Roll_Mad9 AS Roll_Mad9,
  data.Pitch_Mad9 AS Pitch_Mad9,
  data.Yaw_Mad9 AS Yaw_Mad9,
  data.Roll_Mah6 AS Roll_Mah6,
  data.Pitch_Mah6 AS Pitch_Mah6,
  data.Yaw_Mah6 AS Yaw_Mah6,
  data.Roll_Mah9 AS Roll_Mah9,
  data.Pitch_Mah9 AS Pitch_Mah9,
  data.Yaw_Mah9 AS Yaw_Mah9,
  data.Temperature AS Temperature,
  data.Pressure AS Pressure,
  data.GroundPressure AS GroundPressure,
  data.BMPAltitude AS BMPAltitude,
  data.Altitude AS Altitude,
  data.AirSpeed AS AirSpeed,
  data.PropellerRotationSpeed AS PropellerRotationSpeed,
  data.Rudder AS Rudder,
  data.Elevator AS Elevator,
  data.Trim AS Trim,
  data.LoRaRSSI AS LoRaRSSI,
  data.RunningTime AS timestamp() / 1000,
FROM
  'hpa/pub'
```

### DynamoDB上のテーブル作成と削除

```bash
aws dynamodb create-table --table-name HPA_Table --attribute-definitions AttributeName=ID,AttributeType=S AttributeName=Timestamp,AttributeType=N --key-schema AttributeName=ID,KeyType=HASH AttributeName=Timestamp,KeyType=RANGE --billing-mode PAY_PER_REQUEST --table-class STANDARD

aws dynamodb delete-table --table-name HPA_Table
```

### DynamoDBからのCSVダウンロード

CloudShellで以下のコマンドを実行

```bash
aws dynamodb scan --table-name HPA_Table | jq -r '.Items[] | [.Timestamp.N, .Time.S, .Latitude.N, .Longitude.N, .GPSAltitude.N, .GPSCourse.N, .GPSSpeed.N, .AccelX.N, .AccelY.N, .AccelZ.N, .GyroX.N, .GyroY.N, .GyroZ.N, .MagX.N, .MagY.N, .MagZ.N, .Roll_Mad6.N, .Pitch_Mad6.N, .Yaw_Mad6.N, .Roll_Mad9.N, .Pitch_Mad9.N, .Yaw_Mad9.N, .Roll_Mah6.N, .Pitch_Mah6.N, .Yaw_Mah6.N, .Roll_Mah9.N, .Pitch_Mah9.N, .Yaw_Mah9.N, .Temperature.N, .Pressure.N, .GroundPressure.N, .BMPAltitude.N, .Altitude.N, .AirSpeed.N, .PropellerRotationSpeed.N, .Rudder.N, .Elevator.N, .Trim.N, .LoRaRSSI.N, .RunningTime.N] | @csv' >> out.csv

sed -i '1iTimestamp, Date, Time, Latitude, Longitude, GPSAltitude, GPSCourse, GPSSpeed, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, Roll_Mad6, Pitch_Mad6, Yaw_Mad6, Roll_Mad9, Pitch_Mad9, Yaw_Mad9, Roll_Mah6, Pitch_Mah6, Yaw_Mah6, Roll_Mah9, Pitch_Mah9, Yaw_Mah9, Temperature, Pressure, GroundPressure, BMPAltitude, Altitude, AirSpeed, PropellerRotationSpeed, Rudder, Elevator, Trim, LoRaRSSI, RunningTime' out.csv

sed -i '/,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,/d' out.csv
```

もし上のコマンドでできなければCloudShellでデータベースのみをダウンロードし、ローカルでCSVに加工するのも可。1行目をAWSのCloudShellで実行し、`out.txt`を任意のLinuxマシンへダウンロードして2行目以降をローカルで実行する。また、Linuxで事前に`jq`をインストールする。

```bash
aws dynamodb scan --table-name HPA_Table > out.txt

cat out.txt | jq -r '.Items[] | [.Timestamp.N, .Time.S, .Latitude.N, .Longitude.N, .GPSAltitude.N, .GPSCourse.N, .GPSSpeed.N, .AccelX.N, .AccelY.N, .AccelZ.N, .GyroX.N, .GyroY.N, .GyroZ.N, .MagX.N, .MagY.N, .MagZ.N, .Roll_Mad6.N, .Pitch_Mad6.N, .Yaw_Mad6.N, .Roll_Mad9.N, .Pitch_Mad9.N, .Yaw_Mad9.N, .Roll_Mah6.N, .Pitch_Mah6.N, .Yaw_Mah6.N, .Roll_Mah9.N, .Pitch_Mah9.N, .Yaw_Mah9.N, .Temperature.N, .Pressure.N, .GroundPressure.N, .BMPAltitude.N, .Altitude.N, .AirSpeed.N, .PropellerRotationSpeed.N, .Rudder.N, .Elevator.N, .Trim.N, .LoRaRSSI.N, .RunningTime.N] | @csv' >> out.csv

sed -i '1iTimestamp, Date, Time, Latitude, Longitude, GPSAltitude, GPSCourse, GPSSpeed, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, Roll_Mad6, Pitch_Mad6, Yaw_Mad6, Roll_Mad9, Pitch_Mad9, Yaw_Mad9, Roll_Mah6, Pitch_Mah6, Yaw_Mah6, Roll_Mah9, Pitch_Mah9, Yaw_Mah9, Temperature, Pressure, GroundPressure, BMPAltitude, Altitude, AirSpeed, PropellerRotationSpeed, Rudder, Elevator, Trim, LoRaRSSI, RunningTime' out.csv

sed -i '/,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,,/d' out.csv
```

- [ExcelでTimeStampを日本時間（JST）に置き換える方法](https://qiita.com/ajitama/items/c0b65ac7489f84c3b394)
