import csv
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import sys
import os

def extract_pcl_pose_data(bag_path):
    # .db3ファイルを自動検出
    db_files = [f for f in os.listdir(bag_path) if f.endswith('.db3')]
    if not db_files:
        print("No .db3 file found in the bag directory.")
        return
    db_path = os.path.join(bag_path, db_files[0])
    print(f"Using database file: {db_path}")

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # トピックID取得
    cursor.execute("SELECT id FROM topics WHERE name = '/pcl_pose'")
    topic_id = cursor.fetchone()
    if topic_id is None:
        print("Topic /pcl_pose not found in the rosbag.")
        return
    topic_id = topic_id[0]

    # メッセージ型取得
    cursor.execute("SELECT type FROM topics WHERE name = '/pcl_pose'")
    msg_type_str = cursor.fetchone()[0]
    msg_type = get_message(msg_type_str)

    # メッセージ取得
    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    messages = cursor.fetchall()

    data_list = []
    for timestamp, raw_data in messages:
        msg = deserialize_message(raw_data, msg_type)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        data_list.append((timestamp, x, y))

    data_list.sort(key=lambda x: x[0])

    output_path = os.path.join(os.getcwd(), os.path.basename(bag_path) + ".csv")

    with open(output_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'liner_x', 'liner_y'])
        for row in data_list:
            writer.writerow(row)

    print(f"Data successfully written to {output_path}")

def main():
    if len(sys.argv) != 2:
        print("Usage: extract_pcl_pose <rosbag_path> <output_csv>")
    else:
        rclpy.init()
        extract_pcl_pose_data(sys.argv[1])
        rclpy.shutdown()