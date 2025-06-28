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

        # RunningTime, Roll, Pitch, Yawの列を検索
        try:
            running_time_index = headers.index("RunningTime")
            roll_mad6_index = headers.index("Roll_Mad6")
            pitch_mad6_index = headers.index("Pitch_Mad6")
            yaw_mad6_index = headers.index("Yaw_Mad6")
            roll_mad9_index = headers.index("Roll_Mad9")
            pitch_mad9_index = headers.index("Pitch_Mad9")
            yaw_mad9_index = headers.index("Yaw_Mad9")
            roll_mah6_index = headers.index("Roll_Mah6")
            pitch_mah6_index = headers.index("Pitch_Mah6")
            yaw_mah6_index = headers.index("Yaw_Mah6")
            roll_mah9_index = headers.index("Roll_Mah9")
            pitch_mah9_index = headers.index("Pitch_Mah9")
            yaw_mah9_index = headers.index("Yaw_Mah9")
        except ValueError:
            print("ヘッダーが不正です")
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

        # 最初のRunningTime, Roll, Pitch, Yawの値を取得し差分化
        try:
            base_running_time = float(data_rows[0][running_time_index])
            base_roll_mad6 = float(data_rows[0][roll_mad6_index])
            base_pitch_mad6 = float(data_rows[0][pitch_mad6_index])
            base_yaw_mad6 = float(data_rows[0][yaw_mad6_index])
            base_roll_mad9 = float(data_rows[0][roll_mad9_index])
            base_pitch_mad9 = float(data_rows[0][pitch_mad9_index])
            base_yaw_mad9 = float(data_rows[0][yaw_mad9_index])
            base_roll_mah6 = float(data_rows[0][roll_mah6_index])
            base_pitch_mah6 = float(data_rows[0][pitch_mah6_index])
            base_yaw_mah6 = float(data_rows[0][yaw_mah6_index])
            base_roll_mah9 = float(data_rows[0][roll_mah9_index])
            base_pitch_mah9 = float(data_rows[0][pitch_mah9_index])
            base_yaw_mah9 = float(data_rows[0][yaw_mah9_index])
        except ValueError:
            print("CSVの数値形式が不正です")
            return

        for row in data_rows:
            try:
                row[running_time_index] = str(float(row[running_time_index]) - base_running_time)
                row[roll_mad6_index] = str(float(row[roll_mad6_index]) - base_roll_mad6)
                row[pitch_mad6_index] = str(float(row[pitch_mad6_index]) - base_pitch_mad6)
                row[yaw_mad6_index] = str(float(row[yaw_mad6_index]) - base_yaw_mad6)
                row[roll_mad9_index] = str(float(row[roll_mad9_index]) - base_roll_mad9)
                row[pitch_mad9_index] = str(float(row[pitch_mad9_index]) - base_pitch_mad9)
                row[yaw_mad9_index] = str(float(row[yaw_mad9_index]) - base_yaw_mad9)
                row[roll_mah6_index] = str(float(row[roll_mah6_index]) - base_roll_mah6)
                row[pitch_mah6_index] = str(float(row[pitch_mah6_index]) - base_pitch_mah6)
                row[yaw_mah6_index] = str(float(row[yaw_mah6_index]) - base_yaw_mah6)
                row[roll_mah9_index] = str(float(row[roll_mah9_index]) - base_roll_mah9)
                row[pitch_mah9_index] = str(float(row[pitch_mah9_index]) - base_pitch_mah9)
                row[yaw_mah9_index] = str(float(row[yaw_mah9_index]) - base_yaw_mah9)
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
