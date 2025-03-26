# LoRa受信プログラム

## Hardware

* [ESP32-C3-WROOM-02](https://akizukidenshi.com/catalog/g/g117493/)
* [E220-900T22S(JP)](https://dragon-torch.tech/rf-modules/lora/e220-900t22s-jp)

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
