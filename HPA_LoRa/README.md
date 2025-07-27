# LoRa受信プログラム

## Hardware

* [ESP32-C3-WROOM-02](https://akizukidenshi.com/catalog/g/g117493/)
* [E220-900T22S(JP)](https://dragon-torch.tech/rf-modules/lora/e220-900t22s-jp)

### Suggested Usage

* [E220-900T22S（JP）-EV1](https://dragon-torch.tech/rf-modules/lora/e220-900t22s-jp-ev1)
* [ESP32-C3-DevBoard](https://github.com/21km43/ESP32-C3-DevBoard)

[ESP32-C3-DevBoard](https://github.com/21km43/ESP32-C3-DevBoard)の6, 7, 8, 9, 5V, GNDを[E220-900T22S（JP）-EV1](https://dragon-torch.tech/rf-modules/lora/e220-900t22s-jp-ev1)に接続。

## Software

* [PlatformIO](https://platformio.org)
* [VSCode](https://code.visualstudio.com)

## Config

`// #define LORA_CONFIG`のコメントを解除したプログラムを書き込み、以下のコマンドをシリアルモニタから順次実行。（コマンドを入力する度に少し時間を空けること）

```
default
address 19782
baudrate 9600
sf 11
bw 125
power 13
channel 5
worcycle 500
rssi_noise 0
rssi_byte 1
tx_method 0
```

## IP Configuration

このプログラムを書き込んだデバイスのアクセスポイントに接続すれば、[Android](https://github.com/WASA-EET/EET23)からデータを閲覧できる。ただしマイコンが固定のIPアドレスにHTTP POSTする構造となっているので、ネットワーク設定を手動で次のように設定する。

* IPアドレス ... 192.168.1.50
* デフォルトゲートウェイ ... 192.168.1.1
* サブネットマスク ... 255.255.255.0

もしくはAPに接続して`http://192.168.1.1/GetData`にHTTP GETリクエストを送って取得する。
