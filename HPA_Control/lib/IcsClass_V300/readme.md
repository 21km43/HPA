# IcsClass
---
## Overview
近藤科学製ICS機器をArduinoのSerialから動かすためのライブラリです。  
This library is for connecting a Kondo Kagaku ICS device from an Arudino serial port.


## Description
近藤科学製サーボモータや受信機等ICS機器をArduinoで動かすためのライブラリです。  
This library is for connecting a Kondo Kagaku servo motor, receiver or other ICS device using Arudino.

サーボモータの位置制御(ポジション)コマンドやストレッチ等のパラメータ変更がArduinoからできるようになります。
 It enables servo motor position commands and parameter changes like for “stretch” to be performed from Arudino.

受信機(KRR-5FH)に接続することで、送信機(KRC-5FH)からのデータも取得できます。
By connecting to a receiver (KRR-5FH), data can also be acquired from a transmitter (KRC-5FH).

## Contents
root/
 ├IcsClass_Vxxx.zip
 ├readmeTop.md (this file)
 ├ICS変換基板forArduino_xxx.pdf (manual)
 └FunctionList Vxxx(See to index_html)

## Requirement
各圧縮されたライブラリの中身のreadme.mdファイルをご覧ください。
Refer to the compressed readme.md files, which are the contents of the library.

## Usage
ライブラリは3つの圧縮ファイルに分かれています。
The library is divided into three compressed files.

使用する状況に応じてライブラリをインポートしてください。
Import the library depending on conditions of use.

* Serial(HardwareSerial)を使う場合
IcsClass only

詳しくは配布フォルダのマニュアル(PDF)をご覧ください。
For details, refer to the manual (PDF) in the folder provided.

コマンドの詳細については、[コマンドリファレンス](https://kondo-robot.com/faq/ics3-5_3-6softwaremanual_2)をご覧ください。


## Licence
各圧縮されたライブラリの中身のreadme.mdファイルをご覧ください。
Refer to the compressed readme.md files, which are the contents of the library.

## Author
近藤科学株式会社  01/2025
Kondo Kagaku co.,ltd.
近藤科学ホームページ:(<https://kondo-robot.com/>)
