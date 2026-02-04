import argparse, os, glob, logging, sys
from pathlib import Path
import csv
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

logging.basicConfig(level=logging.DEBUG, format="%(levelname)-8s %(asctime)s %(lineno)-4d  %(message)s")

workpath = str(Path(__file__).resolve().parent.parent)

logging.debug(f"workpath: {workpath}")


def find_bag_files(dataset_dir):
    """查找 dataset 目录下的所有 bag 文件"""
    bag_files = []
    if os.path.exists(dataset_dir):
        bag_files = glob.glob(os.path.join(dataset_dir, "*.bag"))
        if not bag_files:
            logging.warning(f"在 {dataset_dir} 目录下未找到 bag 文件")
        else:
            logging.info(f"找到 {len(bag_files)} 个 bag 文件: {bag_files}")
    else:
        logging.error(f"dataset 目录不存在: {dataset_dir}")
    return bag_files


def extract_imu_topics(bag_file):
    """从 bag 文件中提取所有可能的 IMU 话题"""
    imu_topics = []
    try:
        # 创建类型存储以处理没有消息定义的 bag
        typestore = get_typestore(Stores.ROS2_FOXY)

        with AnyReader([Path(bag_file)], default_typestore=typestore) as reader:
            # 打印所有话题和数据类型
            logging.info("Bag 文件中的话题列表:")
            for connection in reader.connections:
                topic_name = connection.topic
                msg_type = connection.msgtype
                logging.info(f"  话题: {topic_name} -> 类型: {msg_type}")

            # 获取所有话题信息
            for connection in reader.connections:
                topic_name = connection.topic
                msg_type = connection.msgtype

                # 检查是否为 IMU 话题
                if msg_type == 'sensor_msgs/msg/Imu' or 'imu' in topic_name.lower() or 'IMU' in topic_name:
                    imu_topics.append(topic_name)
                    logging.info(f"找到 IMU 话题: {topic_name} (类型: {msg_type})")

        if not imu_topics:
            logging.warning(f"在 {bag_file} 中未找到 IMU 话题")

    except Exception as e:
        logging.error(f"读取 bag 文件 {bag_file} 时出错: {e}")

    return imu_topics


def extract_imu_data(bag_file, imu_topic, output_file):
    """从 bag 文件中提取 IMU 数据并保存为 CSV"""
    try:
        # 创建类型存储以处理没有消息定义的 bag
        typestore = get_typestore(Stores.ROS2_FOXY)

        with AnyReader([Path(bag_file)], default_typestore=typestore) as reader:
            with open(output_file, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w', 'linear_acceleration_x', 'linear_acceleration_y', 'linear_acceleration_z', 'angular_velocity_x', 'angular_velocity_y', 'angular_velocity_z']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                count = 0
                # 获取 IMU 话题的连接
                imu_connections = [x for x in reader.connections if x.topic == imu_topic]

                if not imu_connections:
                    logging.error(f"未找到话题 {imu_topic}")
                    return False

                # 读取消息
                for connection, timestamp, rawdata in reader.messages(connections=imu_connections):
                    # 反序列化消息
                    msg = reader.deserialize(rawdata, connection.msgtype)

                    # 提取数据
                    row = {
                        'timestamp': timestamp / 1e9,  # 转换为秒
                        'orientation_x': msg.orientation.x,
                        'orientation_y': msg.orientation.y,
                        'orientation_z': msg.orientation.z,
                        'orientation_w': msg.orientation.w,
                        'linear_acceleration_x': msg.linear_acceleration.x,
                        'linear_acceleration_y': msg.linear_acceleration.y,
                        'linear_acceleration_z': msg.linear_acceleration.z,
                        'angular_velocity_x': msg.angular_velocity.x,
                        'angular_velocity_y': msg.angular_velocity.y,
                        'angular_velocity_z': msg.angular_velocity.z,
                    }

                    writer.writerow(row)
                    count += 1

                logging.info(f"成功提取 {count} 条 IMU 数据到 {output_file}")

    except Exception as e:
        logging.error(f"提取 IMU 数据时出错: {e}")
        return False

    return True


def extract_gps_topics(bag_file):
    """从 bag 文件中提取所有可能的 GPS 话题"""
    gps_topics = []
    try:
        # 创建类型存储以处理没有消息定义的 bag
        typestore = get_typestore(Stores.ROS2_FOXY)

        with AnyReader([Path(bag_file)], default_typestore=typestore) as reader:
            # 获取所有话题信息
            for connection in reader.connections:
                topic_name = connection.topic
                msg_type = connection.msgtype

                # 检查是否为 GPS 话题 (NavSatFix 或 gps 关键词)
                if msg_type == 'sensor_msgs/msg/NavSatFix' or 'gps' in topic_name.lower() or 'GPS' in topic_name or 'navsat' in topic_name.lower():
                    gps_topics.append(topic_name)
                    logging.info(f"找到 GPS 话题: {topic_name} (类型: {msg_type})")

        if not gps_topics:
            logging.warning(f"在 {bag_file} 中未找到 GPS 话题")

    except Exception as e:
        logging.error(f"读取 bag 文件 {bag_file} 时出错: {e}")

    return gps_topics


def extract_gps_data(bag_file, gps_topic, output_file):
    """从 bag 文件中提取 GPS 数据并保存为 CSV"""
    try:
        # 创建类型存储以处理没有消息定义的 bag
        typestore = get_typestore(Stores.ROS2_FOXY)

        with AnyReader([Path(bag_file)], default_typestore=typestore) as reader:
            with open(output_file, 'w', newline='') as csvfile:
                fieldnames = ['timestamp', 'latitude', 'longitude', 'altitude', 'status_status', 'status_service']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()

                count = 0
                # 获取 GPS 话题的连接
                gps_connections = [x for x in reader.connections if x.topic == gps_topic]

                if not gps_connections:
                    logging.error(f"未找到话题 {gps_topic}")
                    return False

                # 读取消息
                for connection, timestamp, rawdata in reader.messages(connections=gps_connections):
                    # 反序列化消息
                    msg = reader.deserialize(rawdata, connection.msgtype)

                    # 提取数据
                    row = {
                        'timestamp': timestamp / 1e9,  # 转换为秒
                        'latitude': msg.latitude,
                        'longitude': msg.longitude,
                        'altitude': msg.altitude,
                        'status_status': msg.status.status,
                        'status_service': msg.status.service,
                    }

                    writer.writerow(row)
                    count += 1

                logging.info(f"成功提取 {count} 条 GPS 数据到 {output_file}")

    except Exception as e:
        logging.error(f"提取 GPS 数据时出错: {e}")
        return False

    return True


def main():
    parser = argparse.ArgumentParser(description="将 dataset 下的 ROS bag 文件转换为 CSV (IMU 或 GPS)")
    parser.add_argument('--bag', type=str, help='指定要转换的 bag 文件路径')
    parser.add_argument('--output', type=str, help='指定输出 CSV 文件路径')
    parser.add_argument('--topic', type=str, help='指定要提取的 IMU 话题名称')
    parser.add_argument('--gps', action='store_true', help='提取 GPS 数据 (默认提取 IMU 数据)')
    parser.add_argument('--gps-topic', type=str, help='指定要提取的 GPS 话题名称')

    args = parser.parse_args()

    # 确定 bag 文件
    bag_file = args.bag
    if not bag_file:
        dataset_dir = os.path.join(workpath, "dataset")
        bag_files = find_bag_files(dataset_dir)
        if not bag_files:
            logging.error("未找到 bag 文件，请使用 --bag 参数指定")
            sys.exit(1)
        bag_file = bag_files[0]  # 使用第一个找到的 bag 文件

    logging.info(f"处理 bag 文件: {bag_file}")

    # 根据参数决定提取 IMU 还是 GPS 数据
    if args.gps:
        # 提取 GPS 数据
        output_file = args.output
        if not output_file:
            output_file = os.path.join(workpath, "gps_data.csv")

        # 确定 GPS 话题
        gps_topic = args.gps_topic
        if not gps_topic:
            gps_topics = extract_gps_topics(bag_file)
            if not gps_topics:
                logging.error("未找到 GPS 话题，请使用 --gps-topic 参数指定")
                sys.exit(1)
            gps_topic = gps_topics[0]  # 使用第一个找到的 GPS 话题

        logging.info(f"提取 GPS 话题: {gps_topic}")
        logging.info(f"输出文件: {output_file}")

        # 提取数据
        success = extract_gps_data(bag_file, gps_topic, output_file)

        if success:
            logging.info("GPS 数据转换完成!")
        else:
            logging.error("GPS 数据转换失败!")
            sys.exit(1)
    else:
        # 提取 IMU 数据 (默认)
        # 确定输出文件
        output_file = args.output
        if not output_file:
            output_file = os.path.join(workpath, "imu_data.csv")

        # 确定 IMU 话题
        imu_topic = args.topic
        if not imu_topic:
            imu_topics = extract_imu_topics(bag_file)
            if not imu_topics:
                logging.error("未找到 IMU 话题，请使用 --topic 参数指定")
                sys.exit(1)
            imu_topic = imu_topics[0]  # 使用第一个找到的 IMU 话题

        logging.info(f"提取 IMU 话题: {imu_topic}")
        logging.info(f"输出文件: {output_file}")

        # 提取数据
        success = extract_imu_data(bag_file, imu_topic, output_file)

        if success:
            logging.info("IMU 数据转换完成!")
        else:
            logging.error("IMU 数据转换失败!")
            sys.exit(1)


if __name__ == "__main__":
    main()
