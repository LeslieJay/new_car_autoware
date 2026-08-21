#!/usr/bin/env python3

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

    reader=open_reader(path)

    start=None
    end=None


    while reader.has_next():

        _,_,stamp=reader.read_next()

        if start is None:
            start=stamp

        end=stamp


    return start,end



def parse_time(bag_start, time_str):

    date=datetime.datetime.fromtimestamp(
        bag_start/1e9
    )


    data=time_str.split(":")

    hour=int(data[0])
    minute=int(data[1])

    second=0

    if len(data)==3:
        second=int(data[2])


    t=date.replace(
        hour=hour,
        minute=minute,
        second=second,
        microsecond=0
    )


    # 自动跨天

    if t.timestamp()*1e9 < bag_start:

        t += datetime.timedelta(days=1)


    return int(t.timestamp()*1e9)




def load_topics(profile):

    if profile is None:
        return None


    with open(
        profile,
        "r"
    ) as f:

        data=yaml.safe_load(f)


    return data["topics"]




def crop(
        input_bag,
        output_bag,
        center_time,
        offset,
        topics=None):


    bag_start,bag_end=get_bag_time(
        input_bag
    )


    print(
        "bag range:"
    )

    print(
        datetime.datetime.fromtimestamp(
            bag_start/1e9
        )
    )

    print(
        datetime.datetime.fromtimestamp(
            bag_end/1e9
        )
    )


    center=parse_time(
        bag_start,
        center_time
    )


    start=center-offset*60*1000000000
    end=center+offset*60*1000000000


    print(
        "crop:"
    )

    print(
        datetime.datetime.fromtimestamp(
            start/1e9
        ),
        "~",
        datetime.datetime.fromtimestamp(
            end/1e9
        )
    )



    reader=open_reader(
        input_bag
    )


    writer=rosbag2_py.SequentialWriter()


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


    topic_types={

        t.name:t.type

        for t in reader.get_all_topics_and_types()

    }


    for name,type_ in topic_types.items():

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



    count=0


    while reader.has_next():

        topic,data,stamp=reader.read_next()


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


        count+=1



    print(
        "messages:",
        count
    )



def main():

    parser=argparse.ArgumentParser()


    parser.add_argument(
        "bag"
    )

    parser.add_argument(
        "time"
    )

    parser.add_argument(
        "minutes",
        type=int
    )

    parser.add_argument(
        "output"
    )


    parser.add_argument(
        "--profile"
    )


    args=parser.parse_args()


    topics=load_topics(
        args.profile
    )


    crop(
        args.bag,
        args.output,
        args.time,
        args.minutes,
        topics
    )



if __name__=="__main__":

    main()