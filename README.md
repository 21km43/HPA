# 人力飛行機 電装システム

人力飛行機の電装システムリポジトリ

- [HPA_Control](/HPA_Control) ... 尾翼操舵システム
- [HPA_Measurement](/HPA_Measurement) ... 機体計測システム（AWS）
- [HPA_LoRa](/HPA_LoRa) ... LoRa受信システム（通信の予備系統）

詳細は各フォルダのREADMEを参照 

## 関連ツール

- [Android UI](https://github.com/WASA-EET/EET23)
- [Web UI](https://github.com/21km43/WASA_2025_GUI_JS)

## 2025鳥コン使用記録

- 動作において特に問題は発生しなかった。ただし、Android側でテザリングを再起動すると再接続処理で操舵が止まることを確認しているため、操舵と計測間の通信は有線（UART, RS485）で接続することが望ましい。
- 計測系統に挿入されたmicroSDにログが記録されていることを確認した。水没してmicroSDにも浸水していたが、それまでに保存された記録は保持されていた。
- LoRaはナビゲータに所有させ、通信の予備系統として利用。今回は終始[Web UI](https://github.com/21km43/WASA_2025_GUI_JS)で閲覧できたため使用せず。

各種プログラムの改善点はそれぞれのプロジェクトディレクトリのREDAMEに記載

## Notice

`secrets.h`にWi-Fiのパラメータ、AWSの秘密鍵等を設定する。以下のコマンドでGitの追跡対象から除外する。

```
git update-index --assume-unchanged HPA_Measurement/src/secrets.h
```

## Diagram

![HPA.drawio](HPA.drawio.svg)
