import csv
from datetime import datetime, timedelta, timezone
import sys

def extract_csv_data(csv_path, input_str):
    try:
        dt_str, duration_str = input_str.rsplit(' ', 1)
        jst_time = datetime.strptime(dt_str, "%Y-%m-%d %H:%M:%S")
        extract_seconds = int(duration_str)
    except ValueError:
        print("Invalid input format. ex: 2025-06-09 14:30:00 10")
        return

    # JST → UTC に変換
    JST = timezone(timedelta(hours=9))
    UTC = timezone.utc
    jst_time = jst_time.replace(tzinfo=JST)
    utc_start = jst_time.astimezone(UTC)
    utc_end = utc_start + timedelta(seconds=extract_seconds)

    # UTCをミリ秒のタイムスタンプに変換
    start_ms = int(utc_start.timestamp() * 1000)
    end_ms = int(utc_end.timestamp() * 1000)

    # CSVを読み込んで該当データを抽出
    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)  # 1行目：ヘッダー
        filtered_rows = [headers]

        for row in reader:
            try:
                timestamp_ms = int(row[0])
                if start_ms <= timestamp_ms < end_ms:
                    filtered_rows.append(row)
            except ValueError:
                continue

    # 結果表示
    for row in filtered_rows:
        print(', '.join(row))

if __name__ == "__main__":
    # ex: python csv_extract.py data.csv "2025-06-09 17:00:00 10"
    if len(sys.argv) != 3:
        print("Usage: python csv_extract.py <csv path> \"yyyy-mm-dd hh:mm:ss extract_seconds\"")
    else:
        extract_csv_data(sys.argv[1], sys.argv[2])
