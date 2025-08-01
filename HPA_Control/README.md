# 操舵系統

## TODO

- 計測システムの都合により、操舵→計測への操舵データの送信をUDPで行っているが、有線（近距離ならUART、長距離ならRS485）で行う方が良い。

## Hardware

* [Arduino UNO R4 WiFi](https://docs.arduino.cc/hardware/uno-r4-wifi) ... [Arduino UNO R4 Minima](https://docs.arduino.cc/hardware/uno-r4-minima)や[Arduino UNO R3](https://docs.arduino.cc/hardware/uno-rev3)でも動作可能。ただし計測へのUDP送信機能は使えなくなる。
* [KSBシールド2](https://kondo-robot.com/product/03149)
* [ICS変換基板](https://kondo-robot.com/product/03121)
* [KRS-4034HV ICS](https://kondo-robot.com/product/krs-4034hv-ics) ... 最大トルクは41.7kgf・cm。基本的にはトルクが高いほど良い
* [TX-26PRR-B10K](https://www.tbm-japan.com/product/07joy.html) ... アナログ入力ができれば他のジョイスティックも利用可

## Software

* [PlatformIO](https://platformio.org)
* [VSCode](https://code.visualstudio.com)

## Libraries

* [ICS Library for Arduino ver.3](https://kondo-robot.com/faq/ics-library-a3) ... [lib/IcsClass_V300](./lib/IcsClass_V300)に配置

## GPIO Usage
* D0 ... RX for ICS
* D1 ... TX for ICS
* D2 ... EN for ICS
* D6 ... Trim up button (internal pull-up)
* D7 ... Trim down button (internal pull-up)
* A0 ... Rudder joystick (analog input)
* A1 ... Elevator joystick (analog input)
