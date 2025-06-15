# 人力飛行機 電装システム

人力飛行機の電装システムリポジトリ。2025年鳥人間コンテストにて利用

- [HPA_Control](/HPA_Control) ... 尾翼操舵システム
- [HPA_Measurement](/HPA_Measurement) ... 機体計測システム
- [HPA_LoRa](/HPA_LoRa) ... LoRa受信システム

詳細は各フォルダのREADMEを参照

## Notice

`secrets.h`にWi-Fiのパラメータ、AWSの秘密鍵等を設定する。以下のコマンドでGitの追跡対象から除外する。

```
git update-index --assume-unchanged HPA_Measurement/src/secrets.h
```

## Diagram

![HPA.drawio](HPA.drawio.svg)
