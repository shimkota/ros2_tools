import csv
import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import sys

def extract_pcl_pose_data(bag_path, output_csv):
    conn = sqlite3.connect(f"{bag_path}/rosbag2_0.db3")
    cursor = conn.cursor()

    cursor.execute("SELECT id FROM topics WHERE name = '/pcl_pose'")
    topic_id = cursor.fetchone()
    if topic_id is None:
        print("Topic /pcl_pose not found in the rosbag.")
        return
    topic_id = topic_id[0]

    cursor.execute("SELECT type FROM topics WHERE name = '/pcl_pose'")
    msg_type_str = cursor.fetchone()[0]
    msg_type = get_message(msg_type_str)

    cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = ?", (topic_id,))
    messages = cursor.fetchall()

    data_list = []
    for timestamp, raw_data in messages:
        msg = deserialize_message(raw_data, msg_type)
        x = msg.pose.position.x
        y = msg.pose.position.y
        data_list.append((timestamp, x, y))

    data_list.sort(key=lambda x: x[0])

    with open(output_csv, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'liner_x', 'liner_y'])
        for row in data_list:
            writer.writerow(row)

    print(f"Data successfully written to {output_csv}")

def main():
    if len(sys.argv) != 3:
        print("Usage: extract_pcl_pose <rosbag_path> <output_csv>")
    else:
        rclpy.init()
        extract_pcl_pose_data(sys.argv[1], sys.argv[2])
        rclpy.shutdown()
