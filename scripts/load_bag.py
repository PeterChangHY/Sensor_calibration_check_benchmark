import rosbag
import difflib
import fastbag
from munch import Munch
import os
import rospy
import sys


def buffered_message_generator(bags, topics, tolerance=0.01, rate=0.1):
    topic_queues = {}
    topic_skips = {}
    topic_counts = {}
    for topic in topics:
        topic_queues[topic] = []
        topic_skips[topic] = 0
        topic_counts[topic] = 0

    for b in bags:
        check_topic_exist_in_bag(b,topics)
        start_timestamp = b.get_start_time()
        end_time = b.get_end_time()
        frame = {}
        while start_timestamp < end_time:
            #print("start_timestamp", start_timestamp)
            for topic, msg, timestamp in b.read_messages(topics=topics, start_time=rospy.Time.from_sec(start_timestamp)):
                m = Munch()
                m.topic = topic
                m.message = msg
                m.timestamp = timestamp
                #print("m.timestamp", timestamp)
                if "perception" in m.topic:
                    # would need to parse the proto...
                    current_timestamp = m.timestamp.to_sec()
                else:
                    #current_timestamp = m.message.header.stamp.to_sec()
                    current_timestamp = m.timestamp.to_sec()
                topic_queues[m.topic].append((current_timestamp, m))
                topic_counts[m.topic] += 1

                # assuming that messages do not arrive out of order, we can
                # safely discard anything older than the newest message on the
                # ends of the queues
                newest_tip = 0
                for topic in topics:
                    if topic_queues[topic] and topic_queues[topic][0][0] > newest_tip:
                        newest_tip = topic_queues[topic][0][0]
                for topic in topics:
                    while topic_queues[topic] and (newest_tip - topic_queues[topic][0][0]) > tolerance:
                        topic_skips[topic] += 1
                        topic_queues[topic].pop(0)

                # we need at least one message on each topic
                if not all(topic_queues.values()):
                    continue
                # at this point, they must be within tolerance and present
                for topic in topics:
                    #print(topic)
                    #print('newest tip', newest_tip)
                    frame[topic] = topic_queues[topic].pop(0)[1]
                break
            start_timestamp += rate
            yield frame

    print("skips", topic_skips)
    print("counts", topic_counts)


def load_rosbags_from_files(bag_files):
    bags = []
    for bag_file in bag_files:
        if os.path.splitext(bag_file)[1] == ".db":
            bag = fastbag.Reader(bag_file)
            bag.open()
        elif os.path.splitext(bag_file)[1] == ".bag":
            bag = rosbag.Bag(bag_file)
        else:
            raise RuntimeError
        bags.append(bag)
    return bags

def load_rosbag_from_file(bag_file):
    if os.path.splitext(bag_file)[1] == ".db":
        bag = fastbag.Reader(bag_file)
        bag.open()
    elif os.path.splitext(bag_file)[1] == ".bag":
        bag = rosbag.Bag(bag_file)
    else:
        raise RuntimeError
    return bag


def check_topic_exist_in_bag(ros_bag, topics):
    bag_topics = ros_bag.get_type_and_topic_info().topics.keys()
    flag = True
    for topic in topics:
        if topic not in bag_topics:
            print(
                "Cannot find topic: %s in bag, maybe use: %s"
                % (topic, difflib.get_close_matches(topic, bag_topics, n=1)[0])
            )
            flag = False
    return flag
