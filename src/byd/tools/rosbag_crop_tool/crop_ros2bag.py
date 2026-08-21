#!/usr/bin/env python3

"""
ROS2 bag 时间裁剪工具

功能:
1. 输入人类时间:
      09:32
      09:32:15

2. 自动转换 ROS timestamp

3. 截取:
      target +- offset minutes

Example:

1. 保存所有话题

python3 crop_ros2bag.py \
    input_bag \
    09:32 \
    5 \
    output_bag

2. 保存特定话题

python3 crop_ros2bag.py \
    input_bag \
    09:32 \
    5 \
    output_bag
    --profile topic_profiles.yaml

3. 保存多个时间点

python3 batch_crop.py \
bag \
times.txt \
5


"""

import argparse
import datetime
import os
import yaml

import rosbag2_py


def open_reader(path):

    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(
            uri=path,
            storage_id=""
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )

    return reader



def get_bag_time(path):

    reader = open_reader(path)

    start = None
    end = None

    while reader.has_next():

        _, _, stamp = reader.read_next()

        if start is None:
            start = stamp

        end = stamp

    return start, end



def parse_time(bag_start, time_str):

    date = datetime.datetime.fromtimestamp(
        bag_start / 1e9
    )

    value = time_str.split(":")

    hour = int(value[0])
    minute = int(value[1])

    second = 0

    if len(value) == 3:
        second = int(value[2])


    target = date.replace(
        hour=hour,
        minute=minute,
        second=second,
        microsecond=0
    )


    # 自动处理跨天

    if target.timestamp() * 1e9 < bag_start:

        target += datetime.timedelta(days=1)


    return int(
        target.timestamp() * 1e9
    )



def generate_output_path(
        input_bag,
        time_str,
        minutes):

    """
    根据输入自动生成输出目录

    example:

    input:
        ~/autoware/log/run1

    time:
        09:32

    minutes:
        5


    output:
        ~/autoware/log/0932_05
    """


    parent = os.path.dirname(
        os.path.abspath(input_bag)
    )


    time_name = time_str.replace(
        ":",
        ""
    )


    minute_name = f"{minutes:02d}"


    folder = f"{time_name}_{minute_name}"


    return os.path.join(
        parent,
        folder
    )



def load_topics(profile):

    if profile is None:
        return None


    with open(profile, "r") as f:

        data = yaml.safe_load(f)


    return data["topics"]



def crop(
        input_bag,
        output_bag,
        center_time,
        offset,
        topics=None):


    bag_start, bag_end = get_bag_time(
        input_bag
    )


    print("====================")
    print("Original bag:")

    print(
        datetime.datetime.fromtimestamp(
            bag_start / 1e9
        )
    )

    print(
        datetime.datetime.fromtimestamp(
            bag_end / 1e9
        )
    )


    center = parse_time(
        bag_start,
        center_time
    )


    start = (
        center
        -
        offset * 60 * 1000000000
    )


    end = (
        center
        +
        offset * 60 * 1000000000
    )


    print("====================")
    print("Crop range:")

    print(
        datetime.datetime.fromtimestamp(
            start / 1e9
        )
    )

    print(
        datetime.datetime.fromtimestamp(
            end / 1e9
        )
    )



    reader = open_reader(
        input_bag
    )


    writer = rosbag2_py.SequentialWriter()


    writer.open(
        rosbag2_py.StorageOptions(
            uri=output_bag,
            storage_id="sqlite3"
        ),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
    )


    topic_types = {
        t.name: t.type
        for t in reader.get_all_topics_and_types()
    }


    for name, type_ in topic_types.items():

        if topics:

            if name not in topics:
                continue


        writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=name,
                type=type_,
                serialization_format="cdr"
            )
        )



    count = 0


    while reader.has_next():

        topic, data, stamp = reader.read_next()


        if stamp < start:
            continue


        if stamp > end:
            break


        if topics:

            if topic not in topics:
                continue


        writer.write(
            topic,
            data,
            stamp
        )


        count += 1



    print("====================")
    print(
        "Messages:",
        count
    )

    print(
        "Output:",
        output_bag
    )



def main():

    parser = argparse.ArgumentParser(
        description="ROS2 bag time crop tool"
    )


    parser.add_argument(
        "bag",
        help="input ros2 bag path"
    )


    parser.add_argument(
        "time",
        help="time point HH:MM or HH:MM:SS"
    )


    parser.add_argument(
        "minutes",
        type=int,
        help="crop range before and after minutes"
    )


    parser.add_argument(
        "--profile",
        help="topic profile yaml"
    )


    args = parser.parse_args()



    output = generate_output_path(
        args.bag,
        args.time,
        args.minutes
    )


    print("====================")
    print(
        "Output directory:",
        output
    )



    if os.path.exists(output):

        print(
            "ERROR:",
            output,
            "already exists"
        )

        exit(1)



    topics = load_topics(
        args.profile
    )


    crop(
        args.bag,
        output,
        args.time,
        args.minutes,
        topics
    )



if __name__ == "__main__":

    main()