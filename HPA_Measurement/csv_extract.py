import csv
from datetime import datetime, timedelta, timezone
import sys

def extract_csv_data(csv_path, input_str):
    try:
        dt_str, duration_str = input_str.rsplit(' ', 1)
        jst_time = datetime.strptime(dt_str, "%Y-%m-%d %H:%M:%S")
        extract_seconds = int(duration_str)
    except ValueError:
        print("Invalid input format. ex: 2025-06-08 7:00:00 10")
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

    with open(csv_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        headers = next(reader)

        # RunningTimeの列を検索
        try:
            running_time_index = headers.index("RunningTime")
        except ValueError:
            print("ヘッダー行に 'RunningTime' 列が見つかりません。")
            return

        filtered_rows = [headers]
        data_rows = []

        for row in reader:
            try:
                timestamp_ms = int(row[0])
                if start_ms <= timestamp_ms < end_ms:
                    data_rows.append(row)
            except ValueError:
                continue

        if not data_rows:
            print("指定された期間に該当するデータがありません。")
            return

        # 最初の RunningTime の値を取得し差分化
        try:
            base_running_time = float(data_rows[0][running_time_index])
        except ValueError:
            print("初期の RunningTime 値が数値として無効です。")
            return

        for row in data_rows:
            try:
                rt_value = float(row[running_time_index])
                row[running_time_index] = str(rt_value - base_running_time)
            except ValueError:
                pass  # 変換できなければそのまま

        filtered_rows.extend(data_rows)

        # 出力
        for row in filtered_rows:
            print(','.join(row))

if __name__ == "__main__":
    # ex: python csv_extract.py data.csv "2025-06-08 7:00:00 10"
    if len(sys.argv) != 3:
        print("Usage: python csv_extract.py <csv path> \"yyyy-mm-dd hh:mm:ss extract_seconds\"")
    else:
        extract_csv_data(sys.argv[1], sys.argv[2])
