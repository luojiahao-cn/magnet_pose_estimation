import rosbag
import numpy as np
from collections import defaultdict
import matplotlib.pyplot as plt
import matplotlib
from tabulate import tabulate
import os
import glob

matplotlib.rcParams['font.sans-serif'] = ['SimHei']
matplotlib.rcParams['axes.unicode_minus'] = False

def analyze_magnetic_field(bag_file):
    topic = '/magnetic_field/raw_data'
    sensor_data = defaultdict(list)
    summary = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            sensor_data[msg.sensor_id].append([
                msg.mag_x,
                msg.mag_y,
                msg.mag_z
            ])

    var_x_list, var_y_list, var_z_list = [], [], []
    range_x_list, range_y_list, range_z_list = [], [], []

    for sensor_id, data in sensor_data.items():
        data = np.array(data)
        if len(data) > 0:
            mean_x = np.mean(data[:, 0])
            mean_y = np.mean(data[:, 1])
            mean_z = np.mean(data[:, 2])
            std_x = np.std(data[:, 0])
            std_y = np.std(data[:, 1])
            std_z = np.std(data[:, 2])
            range_x = np.max(data[:, 0]) - np.min(data[:, 0])
            range_y = np.max(data[:, 1]) - np.min(data[:, 1])
            range_z = np.max(data[:, 2]) - np.min(data[:, 2])
            summary.append([
                sensor_id, mean_x, mean_y, mean_z,
                std_x, std_y, std_z,
                range_x, range_y, range_z
            ])
            var_x_list.append(std_x)
            var_y_list.append(std_y)
            var_z_list.append(std_z)
            range_x_list.append(range_x)
            range_y_list.append(range_y)
            range_z_list.append(range_z)

    output = []
    output.append(f"文件：{os.path.basename(bag_file)}\n")
    output.append(tabulate(
        summary,
        headers=[
            "传感器ID", "X均值", "Y均值", "Z均值",
            "X标准差", "Y标准差", "Z标准差",
            "X极值差", "Y极值差", "Z极值差"
        ],
        tablefmt="github",
        floatfmt=".6f"
    ))
    if var_x_list:
        output.append("\n各通道平均标准差和极值差：")
        output.append(f"X标准差均值: {np.mean(np.sqrt(var_x_list)):.6f}，Y标准差均值: {np.mean(np.sqrt(var_y_list)):.6f}，Z标准差均值: {np.mean(np.sqrt(var_z_list)):.6f}")
        output.append(f"X极值差均值: {np.mean(range_x_list):.6f}，Y极值差均值: {np.mean(range_y_list):.6f}，Z极值差均值: {np.mean(range_z_list):.6f}")
    output.append("-" * 100 + "\n")
    return "\n".join(output)

if __name__ == "__main__":
    bag_dir = "src/magnetic_pose_estimation/tool"
    bag_files = glob.glob(os.path.join(bag_dir, "*.bag"))
    if not bag_files:
        print("未找到任何bag文件。")
    else:
        result_lines = []
        for bag_file in bag_files:
            result_lines.append(analyze_magnetic_field(bag_file))
        # 输出到文件
        with open("src/magnetic_pose_estimation/tool/magnetic_field_analysis_result.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(result_lines))
        print("分析结果已保存到 magnetic_field_analysis_result.txt")